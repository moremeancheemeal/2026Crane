#include "MotorControlTask.h"

#include "can.h"
#include "usart.h"
#include "vofaPlus.h"
#include "XYPath.h"
#include "VisionComm.h"
#include "VisionPositionMap.h"

#include <math.h>

#define GANTRY_SYNC_FAULT_COUNTS      20000

/* ============================================================================
 * 编译期配置表 —— 每个轴一行,改参数只改这里
 *   V/A/J 用输出轴 RPM 量纲(见 MotorControlTask.h 换算说明)
 *   M3508 范围参考:Vmax ≤ 400(硬上限469);Amax=(1~5)×Vmax;Jmax=(3~10)×Amax
 * ----------------------------------------------------------------------------
 * 字段顺序:
 *   can_index, dir, reduction,
 *   vmax_rpm, amax_rpm, jmax_rpm,
 *   pos_kp, pos_ki, pos_kd, pos_max, pos_ilim,
 *   spd_kp, spd_ki, spd_kd, spd_max, spd_ilim
 * ============================================================================ */
axis_cfg_t axis_cfg[AXIS_NUM] =
{
    /* AXIS_GANTRY_L (0x201) 龙门左,X —— 恢复已验证的独立轨迹参数 */
    { 0, -1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 305.0f, 3052.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_GANTRY_R (0x202) 龙门右,X —— 与左对称安装,dir 取 +1 */
    { 1, +1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 305.0f, 3052.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_SLIDE_Y (0x203) 滑行,Y —— 直移与 XY 路径共用此组约束
     * 此行同时是【绕障合速度】上限:过桩时龙门 X 被这里的 V/A/J 卡着(不是龙门自己的 350)。
     * 合速度矢量分解 => |vx|,|vy| <= 合速度;两轴均 M3508(实用顶 ~400),故合速度顶到 350 安全。
     * V 350,A 900,J 6000。 */
    { 2, +1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 900.0f, 6000.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_LIFT_Z (0x204) 升降,Z —— V 保持不变,A/J 比上一版提高一倍 */
    { 3, +1, MOTOR_REDUCTION_RATIO_3508,
      100.0f, 320.0f, 1600.0f,
      0.004f, 0.0f, 0.0f, 350u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_YAW (0x205) 旋转,Yaw —— 慢稳 */
    { 4, +1, MOTOR_REDUCTION_RATIO_3508,
      100.0f, 200.0f, 1200.0f,
      0.004f, 0.0f, 0.0f, 100u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_CLAW (0x206) 爪子 —— 速度由本行 vmax_rpm 单独设定 */
    { 5, +1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 305.0f, 3052.0f,
      0.004f, 0.0f, 0.0f, 100u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },
};

axis_t          axis[AXIS_NUM];
moto_measure_t  Motor[MOTOR_NUM];
volatile motor_diag_t MotorDiag = {0};

static uint8_t motor_control_enable = 0U;

/* XY 联动路径插值器(折线 + 拐角圆弧)。路径模式下 X/Y 的位置环 ref 取自这里,
 * 绕过各轴自己的 axis[i].traj,保证末端沿确定几何路径绕障。其余轴不受影响。 */
static xy_path_t crane_path;

/* ---- X 轴标定点(留作参考,来自原四点位测试) ---- */
const int32_t X_CalPoints[4] = { X_CAL_POINT0, X_CAL_POINT1, X_CAL_POINT2, X_CAL_POINT3 };

/* ============================================================================
 * 初始化 / 设目标 / 读状态
 * ============================================================================ */
void Axis_InitAll(void)
{
    for (uint8_t i = 0; i < AXIS_NUM; i++)
    {
        const axis_cfg_t *c = &axis_cfg[i];

        /* S 曲线:RPM -> counts 量纲 */
        TrajTrack_Init(&axis[i].traj,
                       c->vmax_rpm * TRAJ_RPM_TO_COUNTS,
                       c->amax_rpm * TRAJ_RPM_TO_COUNTS,
                       c->jmax_rpm * TRAJ_RPM_TO_COUNTS,
                       MOTOR_CONTROL_PERIOD, 0.0f);

        /* 双环 PID */
        PID_struct_init(&axis[i].pid_pos, POSITION_PID,
                        c->pos_max, c->pos_ilim, c->pos_kp, c->pos_ki, c->pos_kd);
        PID_struct_init(&axis[i].pid_spd, POSITION_PID,
                        c->spd_max, c->spd_ilim, c->spd_kp, c->spd_ki, c->spd_kd);

        axis[i].target = 0;
    }

    /* XY 联动路径与水平滑台共用 V/A/J。 */
    XYPath_Init(&crane_path,
                axis_cfg[AXIS_SLIDE_Y].vmax_rpm * TRAJ_RPM_TO_COUNTS,
                axis_cfg[AXIS_SLIDE_Y].amax_rpm * TRAJ_RPM_TO_COUNTS,
                axis_cfg[AXIS_SLIDE_Y].jmax_rpm * TRAJ_RPM_TO_COUNTS,
                MOTOR_CONTROL_PERIOD);

    MotorDiag.fault_code = MOTOR_FAULT_NONE;
}

/* 读某轴实测位置(dir 校正,与轨迹/目标同坐标系) */
int32_t Axis_GetPos(axis_id_t id)
{
    const axis_cfg_t *c = &axis_cfg[id];
    return c->dir * Motor[c->can_index].total_angle;
}

static int32_t Gantry_GetAveragePos(void)
{
    int32_t left = Axis_GetPos(AXIS_GANTRY_L);
    int32_t right = Axis_GetPos(AXIS_GANTRY_R);
    return (int32_t)(((int64_t)left + (int64_t)right) / 2);
}

uint8_t Axis_IsBusy(axis_id_t id)
{
    return TrajTrack_IsBusy(&axis[id].traj);
}

/* 真正到位 = 轨迹已走完 且 实测位置进了 ±tol_counts 容差。非阻塞,主循环轮询。
 * 只判 !busy 仅表示"S曲线规划走完";高扭矩/重载时纯P位置环可能还差一点稳态误差,
 * 故再加位置容差才算物理到位。 */
uint8_t Axis_IsReached(axis_id_t id, int32_t tol_counts)
{
    int32_t err;

    if (Axis_IsBusy(id))
    {
        return 0U;                      /* 轨迹还在跑 */
    }

    err = Axis_GetPos(id) - axis[id].target;
    if (err < 0)
    {
        err = -err;
    }
    return (err <= tol_counts) ? 1U : 0U;
}

void Axis_SetTarget(axis_id_t id, int32_t target)
{
    axis[id].target = target;
    TrajTrack_SetTarget(&axis[id].traj, (float)target);
}

/* 同时给左右龙门相同 X 终点,两侧各自规划独立轨迹。 */
void Axis_SetTargetXGantry(int32_t x)
{
    Axis_SetTarget(AXIS_GANTRY_L, x);
    Axis_SetTarget(AXIS_GANTRY_R, x);
}

/* ============================================================================
 * XY 联动路径上层封装 —— 给 CraneFlow / 测试入口用,不直接碰 crane_path 内部
 * ----------------------------------------------------------------------------
 * 典型用法(非阻塞):
 *   CranePath_Run(pts, num, blend);     // 构建并启动一条绕障路径
 *   ... 每周期 MotorSysTask 自动 XYPath_Tick,X/Y 自动跟踪 ...
 *   if (CranePath_IsDone()) CranePath_Finish();   // 到位后收尾,X/Y 交还各轴 traj
 * ============================================================================ */

/* 构建并启动一条路径。pts 为 counts 域航点(map_pt_t),blend 为拐角圆弧半径 counts。
 * 启动瞬间把 X/Y 各轴 traj 同步到实测位置,避免收尾切换时跳变。 */
uint8_t CranePath_Run(const map_pt_t *pts, uint8_t num, float blend_radius)
{
    float bean_clearance;
    float box_clearance;
    uint8_t bean_ok;
    uint8_t box_ok;

    if (!XYPath_Build(&crane_path, pts, num, blend_radius))
    {
        MotorDiag.fault_code = MOTOR_FAULT_PATH_BUILD;
        MotorDiag.path_min_clearance = 0.0f;
        XYPath_Stop(&crane_path);
        MotorControl_Stop();
        return 0U;
    }

    bean_ok = XYPath_ValidateClearance(&crane_path,
                                       (float)OBSTACLE_BEAN_X_COUNTS,
                                       (float)OBSTACLE_BEAN_Y_COUNTS,
                                       OBSTACLE_CLEARANCE_COUNTS,
                                       &bean_clearance);
    box_ok = XYPath_ValidateClearance(&crane_path,
                                      (float)OBSTACLE_BOX_X_COUNTS,
                                      (float)OBSTACLE_BOX_Y_COUNTS,
                                      OBSTACLE_CLEARANCE_COUNTS,
                                      &box_clearance);
    MotorDiag.path_min_clearance = (bean_clearance < box_clearance) ? bean_clearance : box_clearance;

    if (!bean_ok || !box_ok)
    {
        MotorDiag.fault_code = MOTOR_FAULT_PATH_CLEARANCE;
        XYPath_Stop(&crane_path);
        MotorControl_Stop();
        return 0U;
    }

    if (!MotorControl_IsEnabled())
    {
        MotorControl_Enable();
    }

    XYPath_Start(&crane_path,
                 (float)Gantry_GetAveragePos(),
                 (float)Axis_GetPos(AXIS_SLIDE_Y));
    MotorDiag.fault_code = MOTOR_FAULT_NONE;
    return 1U;
}

uint8_t CranePath_IsActive(void)
{
    return XYPath_IsActive(&crane_path);
}

uint8_t CranePath_IsDone(void)
{
    return XYPath_IsDone(&crane_path);
}

/* 收尾:退出路径模式,把 X/Y 各轴 traj/target 复位到当前实测位置,
 * 使后续单轴 Axis_SetTarget 从当前点平滑接续,不跳变。 */
void CranePath_Finish(void)
{
    XYPath_Stop(&crane_path);

    int32_t xl = Axis_GetPos(AXIS_GANTRY_L);
    int32_t xr = Axis_GetPos(AXIS_GANTRY_R);
    int32_t y = Axis_GetPos(AXIS_SLIDE_Y);

    axis[AXIS_GANTRY_L].target = xl;
    TrajTrack_Reset(&axis[AXIS_GANTRY_L].traj, (float)xl);
    axis[AXIS_GANTRY_R].target = xr;
    TrajTrack_Reset(&axis[AXIS_GANTRY_R].traj, (float)xr);
    axis[AXIS_SLIDE_Y].target = y;
    TrajTrack_Reset(&axis[AXIS_SLIDE_Y].traj, (float)y);
}

/* ============================================================================
 * Z 升降轴上层封装 —— 用"高度 counts"指挥,内部走 S 曲线
 *   高度 = 相对上电位置(Z0)的 counts;上电时电机 total_angle 已归零,故 Z0 = 位置 0。
 *   方向:负 = 上升(机械实际方向决定;set_z 不做内部翻转,传入符号即目标符号)。
 * ============================================================================ */
void set_z(int32_t z_counts)
{
    Axis_SetTarget(AXIS_LIFT_Z, z_counts);
}

/* ============================================================================
 * 夹爪旋转电机两位置控制
 *   CLAW_OPEN_ANGLE / CLAW_CLOSED_ANGLE 是两个固定位置,外部直接调用即可。
 *   为了避免在失能状态下设置目标被 MotorTargetSync 覆盖,这里会先确保控制已使能。
 * ============================================================================ */
static void TwoPosAxisSetTarget(axis_id_t id, int32_t angle_counts)
{
    if (!MotorControl_IsEnabled())
    {
        MotorControl_Enable();
    }

    Axis_SetTarget(id, angle_counts);
}

void ClawSetTarget(int32_t angle_counts)
{
    TwoPosAxisSetTarget(AXIS_CLAW, angle_counts);
}

void ClawOpen(void)
{
    ClawSetTarget(CLAW_OPEN_ANGLE);
}

void ClawClose(void)
{
    ClawSetTarget(CLAW_CLOSED_ANGLE);
}

void ClawToggle(void)
{
    if (Axis_GetPos(AXIS_CLAW) < ((CLAW_OPEN_ANGLE + CLAW_CLOSED_ANGLE) / 2))
    {
        ClawOpen();
    }
    else
    {
        ClawClose();
    }
}

void YawSetTarget(int32_t angle_counts)
{
    TwoPosAxisSetTarget(AXIS_YAW, angle_counts);
}

void YawOpen(void)
{
    YawSetTarget(YAW_OPEN_ANGLE);
}

void YawClose(void)
{
    YawSetTarget(YAW_CLOSED_ANGLE);
}

void YawToggle(void)
{
    if (Axis_GetPos(AXIS_YAW) < ((YAW_OPEN_ANGLE + YAW_CLOSED_ANGLE) / 2))
    {
        YawClose();
    }
    else
    {
        YawOpen();
    }
}

/* ============================================================================
 * 滑行电机(SLIDE_Y)八位置控制
 *   货箱3个位置:SLIDE_Y_POS_1 / _POS_3 / _POS_2 (取货区)
 *   空货箱5个位置:SLIDE_Y_POS_4 ~ _POS_8 (放置区)
 *   为避免在失能状态下设置目标被 MotorTargetSync 覆盖,
 *   这里会先确保控制已使能。
 * ============================================================================ */
static void SlideYSetTarget(int32_t y_counts)
{
    if (!MotorControl_IsEnabled())
    {
        MotorControl_Enable();
    }

    Axis_SetTarget(AXIS_SLIDE_Y, y_counts);
}

void SlideYGoTo(uint8_t pos)
{
    int32_t target;
    switch (pos)
    {
        case 1: target = SLIDE_Y_POS_1; break;
        case 2: target = SLIDE_Y_POS_2; break;
        case 3: target = SLIDE_Y_POS_3; break;
        case 4: target = SLIDE_Y_POS_4; break;
        case 5: target = SLIDE_Y_POS_5; break;
        case 6: target = SLIDE_Y_POS_6; break;
        case 7: target = SLIDE_Y_POS_7; break;
        case 8: target = SLIDE_Y_POS_8; break;
        default: return;                 /* 无效位置 */
    }
    SlideYSetTarget(target);
}

void SlideYGoTo1(void) { SlideYSetTarget(SLIDE_Y_POS_1); }
void SlideYGoTo2(void) { SlideYSetTarget(SLIDE_Y_POS_2); }
void SlideYGoTo3(void) { SlideYSetTarget(SLIDE_Y_POS_3); }
void SlideYGoTo4(void) { SlideYSetTarget(SLIDE_Y_POS_4); }
void SlideYGoTo5(void) { SlideYSetTarget(SLIDE_Y_POS_5); }
void SlideYGoTo6(void) { SlideYSetTarget(SLIDE_Y_POS_6); }
void SlideYGoTo7(void) { SlideYSetTarget(SLIDE_Y_POS_7); }
void SlideYGoTo8(void) { SlideYSetTarget(SLIDE_Y_POS_8); }

uint8_t SlideY_IsReached(void)
{
    return Axis_IsReached(AXIS_SLIDE_Y, 5000);   /* 容差 5000 counts,待标定后调整 */
}

int32_t Z_GetHeight(void)
{
    return Axis_GetPos(AXIS_LIFT_Z);
}

uint8_t Z_IsReached(int32_t tol_counts)
{
    return Axis_IsReached(AXIS_LIFT_Z, tol_counts);
}

/* ============================================================================
 * Z 升降高度测试状态机(分步式,验证升降加减速)
 * 高度序列(counts,负=上升,相对上电 Z0):每去一个取货层前先抬顶,放货后抬顶再归零
 *   顶 -> 三层 -> 顶 -> 二层 -> 顶 -> 一层 -> 放货 -> 顶 -> 归零 -> 循环
 * 分步式:按键每按一次推进一步,交替"使能去下一高度 / 失能"。
 * ⚠ 注意:Z 为升降轴,在"顶"高度失能会被重力拉下;分步式按你要求保留,
 *   若掉落不可接受,改连贯式(一按完成抬顶+受控下降,全程不失能)。
 * ============================================================================ */
#define Z_TOP          Z_TOP_TRAVEL   /* 最高运输位使用本次实测标定 */
#define Z_HEIGHT_NUM   9
static const int32_t z_height_points[Z_HEIGHT_NUM] =
{
    Z_TOP,     /* 抬顶(去三层前) */
    Z_LAYER_3, /* 第三点位 */
    Z_TOP,     /* 抬顶(去二层前) */
    Z_LAYER_2, /* 第二点位 */
    Z_TOP,     /* 抬顶(去一层前) */
    Z_LAYER_1, /* 第一点位 */
    Z_PLACE,   /* 放货高度 */
    Z_TOP,     /* 放货后抬顶 */
    Z_BOTTOM,  /* 回 Z0(上电高度) */
};
static uint8_t z_height_index = 0U;   /* 下一个要去的高度下标 */

void ZHeightStep(void)
{
    if (MotorControl_IsEnabled())
    {
        /* 使能态再按 -> 失能(可观察/手动) */
        MotorControl_Stop();
        return;
    }

    /* 失能态再按 -> 使能并去下一个高度 */
    MotorControl_Enable();              /* 内部 MotorTargetSync 已把轨迹复位到实测位置 */

    set_z(z_height_points[z_height_index]);
    z_height_index++;
    if (z_height_index >= Z_HEIGHT_NUM)
    {
        z_height_index = 0U;            /* 走完回零后循环 */
    }
}

/* ============================================================================
 * 自动取放货流程(非阻塞状态机)
 * ----------------------------------------------------------------------------
 * 按键推进"大步骤",步骤内部"到位后自动接续"由 CraneFlow_Tick() 每控制周期轮询。
 * 升降优先:先升/降 Z 到位,再横移 X(或反之),避免未抬升就横移撞障碍。
 *
 *   按1: 升到顶 ↳到位后 横移到 X_CAL_POINT0
 *   按2: 降到三层 ↳到位后 爪闭合 ↳到位后 升到顶
 *   按3: 横移到 X_CAL_POINT3
 *   按4: 降到放货高度
 *   按5: 爪松开 ↳到位后 升到顶
 *   按6: 横移回原点 X0 ↳到位后 降到底部 Z0
 *
 * 全程保持使能(Z 升降轴不失能,避免重力掉落)。
 * ============================================================================ */
#define FLOW_Z_LEVEL3      Z_LAYER_3   /* 第三点位 */
#define FLOW_Z_PLACE       Z_PLACE     /* 放货高度 */
#define FLOW_Z_BOTTOM      Z_BOTTOM    /* 底部 = 上电 Z0 */
#define FLOW_X_HOME        (0)         /* 原点 = 上电 X0 */

/* 到位容差(分轴):X 行程大给宽些,Z 中等,爪行程很大给最宽 */
#define FLOW_TOL_X         15000
#define FLOW_TOL_Z         10000
#define FLOW_TOL_CLAW      40000

/* 流程子状态:某个大步骤内"等到位再接续"的阶段 */
typedef enum
{
    SUB_DONE = 0,       /* 当前大步骤已跑完,等下一次按键 */

    /* 按1: 升顶 -> 横移 P0 */
    SUB_1_LIFT_TOP,
    SUB_1_MOVE_P0,
    /* 按2: 降三层 -> 爪闭合 -> 升顶 */
    SUB_2_DOWN_L3,
    SUB_2_CLAW_CLOSE,
    SUB_2_LIFT_TOP,
    /* 按3: 横移 P3 */
    SUB_3_MOVE_P3,
    /* 按4: 降放货 */
    SUB_4_DOWN_PLACE,
    /* 按5: 爪松开 -> 升顶 */
    SUB_5_CLAW_OPEN,
    SUB_5_LIFT_TOP,
    /* 按6: 横移回原点 -> 降底部 */
    SUB_6_MOVE_HOME,
    SUB_6_DOWN_BOTTOM,
} crane_sub_t;

static uint8_t     flow_step = 0U;       /* 已触发的大步骤数 0..6 */
static crane_sub_t flow_sub  = SUB_DONE; /* 当前步骤内的子阶段 */

/* 龙门到位:左右两个电机都进容差才算到位(单判一侧会让漂移/落后的另一侧被忽略) */
static uint8_t GantryReached(int32_t tol)
{
    return (Axis_IsReached(AXIS_GANTRY_L, tol) && Axis_IsReached(AXIS_GANTRY_R, tol));
}

/* ============================================================================
 * Vision coordinated flow
 * ----------------------------------------------------------------------------
 * Classification travel order: 2 -> 3 -> 1; pick order: 1 -> 3 -> 2.
 * MATCH positions arrive in classification order and are remapped below to the
 * fixed item indexes before the independent pick order is applied.
 * ============================================================================ */
#define VISION_ITEM_NUM     3U
#define FLOW_TOL_Y          15000
#define FLOW_TOL_YAW        10000

typedef enum
{
    VISION_SUB_IDLE = 0,

    VISION_SUB_SAMPLE_LIFT_TOP,
    VISION_SUB_SAMPLE_MOVE_XY,
    VISION_SUB_SAMPLE_SEND_S,
    VISION_SUB_SAMPLE_WAIT_SDONE,
    VISION_SUB_WAIT_MATCH,

    VISION_SUB_PICK_LIFT_TOP,
    VISION_SUB_PICK_MOVE_XY,
    VISION_SUB_PICK_DOWN,
    VISION_SUB_PICK_CLAW_CLOSE,
    VISION_SUB_PICK_LIFT_AFTER,

    VISION_SUB_PLACE_MOVE_XY,
    VISION_SUB_PLACE_YAW,
    VISION_SUB_PLACE_DOWN,
    VISION_SUB_PLACE_CLAW_OPEN,
    VISION_SUB_PLACE_LIFT_AFTER,
    VISION_SUB_PLACE_YAW_RESET,

    VISION_SUB_RETURN_MOVE_HOME,
    VISION_SUB_RETURN_DOWN_BOTTOM,
    VISION_SUB_RETURN_CLAW_CLOSE,
    VISION_SUB_DONE,
    VISION_SUB_ERROR,
} vision_sub_t;

static const mission_t vision_pick_table[VISION_ITEM_NUM] =
{
    { BOX_PICK_1, BOX_PLACE_4 },
    { BOX_PICK_2, BOX_PLACE_4 },
    { BOX_PICK_3, BOX_PLACE_4 },
};
static const uint8_t vision_sample_order[VISION_ITEM_NUM] = { 1U, 2U, 0U };
static const uint8_t vision_pick_order[VISION_ITEM_NUM] = { 0U, 2U, 1U };
static box_t vision_place_table[VISION_ITEM_NUM];

typedef enum
{
    VISION_ZONE_LEFT = 0,
    VISION_ZONE_MIDDLE,
    VISION_ZONE_RIGHT,
} vision_zone_t;

#define VISION_PATH_MAX_POINTS  6U

typedef enum
{
    VISION_PATH_IDLE = 0,
    VISION_PATH_ACTIVE,
    VISION_PATH_DIRECT,
} vision_path_state_t;

static map_pt_t vision_path_points[VISION_PATH_MAX_POINTS];
static map_pt_t vision_path_target;
static vision_path_state_t vision_path_state = VISION_PATH_IDLE;

static uint8_t vision_active = 0U;
static uint8_t vision_host_ready = 0U;
static uint8_t vision_classify_done = 0U;
static uint8_t vision_match_ready = 0U;
static uint8_t vision_sample_index = 0U;
static uint8_t vision_item_index = 0U;
static vision_sub_t vision_sub = VISION_SUB_IDLE;

static uint8_t VisionFlow_ValueReached(int32_t value, int32_t target, int32_t tol)
{
    int64_t diff = (int64_t)value - (int64_t)target;

    if (diff < 0)
    {
        diff = -diff;
    }
    return (diff <= (int64_t)tol) ? 1U : 0U;
}

static uint8_t VisionFlow_XYAtTarget(int32_t x_counts, int32_t y_counts)
{
    return (VisionFlow_ValueReached(Axis_GetPos(AXIS_GANTRY_L), x_counts, FLOW_TOL_X) &&
            VisionFlow_ValueReached(Axis_GetPos(AXIS_GANTRY_R), x_counts, FLOW_TOL_X) &&
            VisionFlow_ValueReached(Axis_GetPos(AXIS_SLIDE_Y), y_counts, FLOW_TOL_Y));
}

static vision_zone_t VisionFlow_GetZone(int32_t x_counts)
{
    if (x_counts < OBSTACLE_BEAN_X_COUNTS)
    {
        return VISION_ZONE_LEFT;
    }
    if (x_counts > OBSTACLE_BOX_X_COUNTS)
    {
        return VISION_ZONE_RIGHT;
    }
    return VISION_ZONE_MIDDLE;
}

static uint8_t VisionFlow_AddObstacleGate(int32_t center_x, int32_t gate_y,
                                          int8_t direction, uint8_t *point_num)
{
    if (point_num == 0 || *point_num > (VISION_PATH_MAX_POINTS - 2U))
    {
        return 0U;
    }

    if (direction > 0)
    {
        vision_path_points[*point_num].x = center_x - OBSTACLE_GATE_HALF_X_COUNTS;
        vision_path_points[*point_num].y = gate_y;
        (*point_num)++;
        vision_path_points[*point_num].x = center_x + OBSTACLE_GATE_HALF_X_COUNTS;
        vision_path_points[*point_num].y = gate_y;
        (*point_num)++;
    }
    else
    {
        vision_path_points[*point_num].x = center_x + OBSTACLE_GATE_HALF_X_COUNTS;
        vision_path_points[*point_num].y = gate_y;
        (*point_num)++;
        vision_path_points[*point_num].x = center_x - OBSTACLE_GATE_HALF_X_COUNTS;
        vision_path_points[*point_num].y = gate_y;
        (*point_num)++;
    }

    return 1U;
}

static uint8_t VisionFlow_StartXYPath(int32_t x_counts, int32_t y_counts)
{
    int32_t start_x = Gantry_GetAveragePos();
    int32_t start_y = Axis_GetPos(AXIS_SLIDE_Y);
    vision_zone_t start_zone = VisionFlow_GetZone(start_x);
    vision_zone_t target_zone = VisionFlow_GetZone(x_counts);
    uint8_t point_num = 0U;

    if (CranePath_IsActive() || vision_path_state != VISION_PATH_IDLE)
    {
        CranePath_Finish();
    }

    vision_path_target.x = x_counts;
    vision_path_target.y = y_counts;

    if (VisionFlow_XYAtTarget(x_counts, y_counts))
    {
        vision_path_state = VISION_PATH_IDLE;
        return 1U;
    }

    if (start_x == x_counts && start_y == y_counts)
    {
        Axis_SetTargetXGantry(x_counts);
        Axis_SetTarget(AXIS_SLIDE_Y, y_counts);
        vision_path_state = VISION_PATH_DIRECT;
        return 1U;
    }

    vision_path_points[point_num].x = start_x;
    vision_path_points[point_num].y = start_y;
    point_num++;

    if (start_zone < target_zone)
    {
        if (start_zone == VISION_ZONE_LEFT)
        {
            if (!VisionFlow_AddObstacleGate(OBSTACLE_BEAN_X_COUNTS,
                                            DETOUR_BEAN_Y_COUNTS, +1, &point_num))
            {
                MotorDiag.fault_code = MOTOR_FAULT_PATH_BUILD;
                MotorControl_Stop();
                return 0U;
            }
        }
        if (target_zone == VISION_ZONE_RIGHT)
        {
            if (!VisionFlow_AddObstacleGate(OBSTACLE_BOX_X_COUNTS,
                                            DETOUR_BOX_Y_COUNTS, +1, &point_num))
            {
                MotorDiag.fault_code = MOTOR_FAULT_PATH_BUILD;
                MotorControl_Stop();
                return 0U;
            }
        }
    }
    else if (start_zone > target_zone)
    {
        if (start_zone == VISION_ZONE_RIGHT)
        {
            if (!VisionFlow_AddObstacleGate(OBSTACLE_BOX_X_COUNTS,
                                            DETOUR_BOX_Y_COUNTS, -1, &point_num))
            {
                MotorDiag.fault_code = MOTOR_FAULT_PATH_BUILD;
                MotorControl_Stop();
                return 0U;
            }
        }
        if (target_zone == VISION_ZONE_LEFT)
        {
            if (!VisionFlow_AddObstacleGate(OBSTACLE_BEAN_X_COUNTS,
                                            DETOUR_BEAN_Y_COUNTS, -1, &point_num))
            {
                MotorDiag.fault_code = MOTOR_FAULT_PATH_BUILD;
                MotorControl_Stop();
                return 0U;
            }
        }
    }

    if (point_num >= VISION_PATH_MAX_POINTS)
    {
        MotorDiag.fault_code = MOTOR_FAULT_PATH_BUILD;
        MotorControl_Stop();
        return 0U;
    }

    vision_path_points[point_num] = vision_path_target;
    point_num++;
    if (!CranePath_Run(vision_path_points, point_num, PATH_CORNER_BLEND_COUNTS))
    {
        vision_path_state = VISION_PATH_IDLE;
        return 0U;
    }

    vision_path_state = VISION_PATH_ACTIVE;
    return 1U;
}

static uint8_t VisionFlow_XYPathReached(void)
{
    if (vision_path_state == VISION_PATH_IDLE)
    {
        return VisionFlow_XYAtTarget(vision_path_target.x, vision_path_target.y);
    }

    if (vision_path_state == VISION_PATH_DIRECT)
    {
        if (GantryReached(FLOW_TOL_X) && Axis_IsReached(AXIS_SLIDE_Y, FLOW_TOL_Y))
        {
            vision_path_state = VISION_PATH_IDLE;
            return 1U;
        }
        return 0U;
    }

    if (!CranePath_IsDone() ||
        !VisionFlow_XYAtTarget(vision_path_target.x, vision_path_target.y))
    {
        return 0U;
    }

    CranePath_Finish();
    vision_path_state = VISION_PATH_IDLE;
    return 1U;
}

static void VisionFlow_StartSampleMove(void)
{
    set_z(Z_TOP);
    YawOpen();
    vision_sub = VISION_SUB_SAMPLE_LIFT_TOP;
}

static void VisionFlow_StartPickPlace(void)
{
    set_z(Z_TOP);
    YawOpen();
    vision_sub = VISION_SUB_PICK_LIFT_TOP;
}

static void VisionFlow_Reset(void)
{
    if (CranePath_IsActive() || vision_path_state != VISION_PATH_IDLE)
    {
        CranePath_Finish();
    }

    vision_active = 0U;
    vision_classify_done = 0U;
    vision_match_ready = 0U;
    vision_sample_index = 0U;
    vision_item_index = 0U;
    vision_path_state = VISION_PATH_IDLE;
    vision_sub = VISION_SUB_IDLE;
}

uint8_t CraneFlow_IsVisionActive(void)
{
    return vision_active;
}

void CraneFlow_OnVisionReady(void)
{
    vision_host_ready = 1U;
}

void CraneFlow_OnVisionStart(void)
{
    (void)vision_host_ready;

    MotorControl_Enable();
    flow_step = 0U;
    flow_sub = SUB_DONE;

    VisionFlow_Reset();
    vision_active = 1U;
    VisionFlow_StartSampleMove();
}

void CraneFlow_OnVisionClassifyDone(void)
{
    if (vision_active)
    {
        vision_classify_done = 1U;
    }
}

void CraneFlow_OnVisionMatch(uint8_t pos1, uint8_t pos2, uint8_t pos3)
{
    if (!VisionPosition_GetPlace(pos1, &vision_place_table[vision_sample_order[0]]) ||
        !VisionPosition_GetPlace(pos2, &vision_place_table[vision_sample_order[1]]) ||
        !VisionPosition_GetPlace(pos3, &vision_place_table[vision_sample_order[2]]))
    {
        vision_sub = VISION_SUB_ERROR;
        return;
    }

    vision_match_ready = 1U;
}

void CraneFlow_OnVisionError(void)
{
    if (vision_active)
    {
        vision_sub = VISION_SUB_ERROR;
    }
}

static void CraneFlow_VisionTick(void)
{
    switch (vision_sub)
    {
        case VISION_SUB_SAMPLE_LIFT_TOP:
            if (Z_IsReached(FLOW_TOL_Z))
            {
                const box_t *pick = &vision_pick_table[vision_sample_order[vision_sample_index]].pick;
                if (vision_sample_index == 0U)
                {
                    ClawOpen();
                }
                if (VisionFlow_StartXYPath(pick->x_counts, pick->y_counts))
                {
                    vision_sub = VISION_SUB_SAMPLE_MOVE_XY;
                }
                else
                {
                    vision_sub = VISION_SUB_ERROR;
                }
            }
            break;

        case VISION_SUB_SAMPLE_MOVE_XY:
            if (VisionFlow_XYPathReached())
            {
                vision_sub = VISION_SUB_SAMPLE_SEND_S;
            }
            break;

        case VISION_SUB_SAMPLE_SEND_S:
            if (VisionComm_SendClassifyTrigger() == HAL_OK)
            {
                vision_classify_done = 0U;
                vision_sub = VISION_SUB_SAMPLE_WAIT_SDONE;
            }
            else
            {
                vision_sub = VISION_SUB_ERROR;
            }
            break;

        case VISION_SUB_SAMPLE_WAIT_SDONE:
            if (vision_classify_done)
            {
                vision_classify_done = 0U;
                vision_sample_index++;
                if (vision_sample_index < VISION_ITEM_NUM)
                {
                    VisionFlow_StartSampleMove();
                }
                else
                {
                    vision_sub = VISION_SUB_WAIT_MATCH;
                }
            }
            break;

        case VISION_SUB_WAIT_MATCH:
            if (vision_match_ready)
            {
                vision_item_index = 0U;
                VisionFlow_StartPickPlace();
            }
            break;

        case VISION_SUB_PICK_LIFT_TOP:
            if (Z_IsReached(FLOW_TOL_Z) && Axis_IsReached(AXIS_YAW, FLOW_TOL_YAW))
            {
                const box_t *pick = &vision_pick_table[vision_pick_order[vision_item_index]].pick;
                if (VisionFlow_StartXYPath(pick->x_counts, pick->y_counts))
                {
                    vision_sub = VISION_SUB_PICK_MOVE_XY;
                }
                else
                {
                    vision_sub = VISION_SUB_ERROR;
                }
            }
            break;

        case VISION_SUB_PICK_MOVE_XY:
            if (VisionFlow_XYPathReached())
            {
                set_z(vision_pick_table[vision_pick_order[vision_item_index]].pick.z_counts);
                vision_sub = VISION_SUB_PICK_DOWN;
            }
            break;

        case VISION_SUB_PICK_DOWN:
            if (Z_IsReached(FLOW_TOL_Z))
            {
                ClawClose();
                vision_sub = VISION_SUB_PICK_CLAW_CLOSE;
            }
            break;

        case VISION_SUB_PICK_CLAW_CLOSE:
            if (Axis_IsReached(AXIS_CLAW, FLOW_TOL_CLAW))
            {
                set_z(Z_TOP);
                vision_sub = VISION_SUB_PICK_LIFT_AFTER;
            }
            break;

        case VISION_SUB_PICK_LIFT_AFTER:
            if (Z_IsReached(FLOW_TOL_Z))
            {
                const box_t *place = &vision_place_table[vision_pick_order[vision_item_index]];
                if (VisionFlow_StartXYPath(place->x_counts, place->y_counts))
                {
                    vision_sub = VISION_SUB_PLACE_MOVE_XY;
                }
                else
                {
                    vision_sub = VISION_SUB_ERROR;
                }
            }
            break;

        case VISION_SUB_PLACE_MOVE_XY:
            if (VisionFlow_XYPathReached())
            {
                YawSetTarget(vision_place_table[vision_pick_order[vision_item_index]].yaw_counts);
                vision_sub = VISION_SUB_PLACE_YAW;
            }
            break;

        case VISION_SUB_PLACE_YAW:
            if (Axis_IsReached(AXIS_YAW, FLOW_TOL_YAW))
            {
                set_z(vision_place_table[vision_pick_order[vision_item_index]].z_counts);
                vision_sub = VISION_SUB_PLACE_DOWN;
            }
            break;

        case VISION_SUB_PLACE_DOWN:
            if (Z_IsReached(FLOW_TOL_Z))
            {
                ClawOpen();
                vision_sub = VISION_SUB_PLACE_CLAW_OPEN;
            }
            break;

        case VISION_SUB_PLACE_CLAW_OPEN:
            if (Axis_IsReached(AXIS_CLAW, FLOW_TOL_CLAW))
            {
                set_z(Z_TOP);
                vision_sub = VISION_SUB_PLACE_LIFT_AFTER;
            }
            break;

        case VISION_SUB_PLACE_LIFT_AFTER:
            if (Z_IsReached(FLOW_TOL_Z))
            {
                YawOpen();
                vision_sub = VISION_SUB_PLACE_YAW_RESET;
            }
            break;

        case VISION_SUB_PLACE_YAW_RESET:
            if (Axis_IsReached(AXIS_YAW, FLOW_TOL_YAW))
            {
                vision_item_index++;
                if (vision_item_index < VISION_ITEM_NUM)
                {
                    VisionFlow_StartPickPlace();
                }
                else
                {
                    if (VisionFlow_StartXYPath(FLOW_X_HOME, 0))
                    {
                        vision_sub = VISION_SUB_RETURN_MOVE_HOME;
                    }
                    else
                    {
                        vision_sub = VISION_SUB_ERROR;
                    }
                }
            }
            break;

        case VISION_SUB_RETURN_MOVE_HOME:
            if (VisionFlow_XYPathReached())
            {
                set_z(FLOW_Z_BOTTOM);
                vision_sub = VISION_SUB_RETURN_DOWN_BOTTOM;
            }
            break;

        case VISION_SUB_RETURN_DOWN_BOTTOM:
            if (Z_IsReached(FLOW_TOL_Z))
            {
                ClawClose();
                vision_sub = VISION_SUB_RETURN_CLAW_CLOSE;
            }
            break;

        case VISION_SUB_RETURN_CLAW_CLOSE:
            if (Axis_IsReached(AXIS_CLAW, FLOW_TOL_CLAW))
            {
                vision_active = 0U;
                vision_sub = VISION_SUB_DONE;
            }
            break;

        case VISION_SUB_ERROR:
            if (CranePath_IsActive() || vision_path_state != VISION_PATH_IDLE)
            {
                CranePath_Finish();
            }
            MotorControl_Stop();
            vision_path_state = VISION_PATH_IDLE;
            vision_active = 0U;
            break;

        case VISION_SUB_IDLE:
        case VISION_SUB_DONE:
        default:
            break;
    }
}

/* 按键:推进到下一个大步骤。仅当上一步骤的自动接续已跑完(flow_sub==SUB_DONE)才响应,
 * 避免中途打断。走完第6步后再按 -> 从头重来。 */
void CraneFlow_Step(void)
{
    if (vision_active)
    {
        return;
    }

    if (!MotorControl_IsEnabled())
    {
        MotorControl_Enable();           /* 首次按下先使能(各轴轨迹同步到实测位置) */
    }

    if (flow_sub != SUB_DONE)
    {
        return;                          /* 上一步骤还在自动接续,忽略本次按键 */
    }

    if (flow_step >= 6U)
    {
        flow_step = 0U;                  /* 整套结束后再按 -> 从头 */
    }

    flow_step++;
    switch (flow_step)
    {
        case 1U: set_z(Z_TOP);                       flow_sub = SUB_1_LIFT_TOP;  break; /* 升顶->横移P0 */
        case 2U: set_z(FLOW_Z_LEVEL3);               flow_sub = SUB_2_DOWN_L3;   break; /* 降三层->夹->升顶 */
        case 3U: Axis_SetTargetXGantry(X_CAL_POINT3);flow_sub = SUB_3_MOVE_P3;   break; /* 横移P3 */
        case 4U: set_z(FLOW_Z_PLACE);                flow_sub = SUB_4_DOWN_PLACE;break; /* 降放货 */
        case 5U: ClawOpen();                         flow_sub = SUB_5_CLAW_OPEN; break; /* 松开->升顶 */
        case 6U: Axis_SetTargetXGantry(FLOW_X_HOME); flow_sub = SUB_6_MOVE_HOME; break; /* 回原点->降底 */
        default: break;
    }
}

/* 每控制周期调用:处理大步骤内部"到位后自动接续"。跑到子序列末尾 -> SUB_DONE 等下一次按键。 */
void CraneFlow_Tick(void)
{
    if (vision_active)
    {
        CraneFlow_VisionTick();
        return;
    }

    switch (flow_sub)
    {
        /* 按1 */
        case SUB_1_LIFT_TOP:
            if (Z_IsReached(FLOW_TOL_Z)) { Axis_SetTargetXGantry(X_CAL_POINT0); flow_sub = SUB_1_MOVE_P0; }
            break;
        case SUB_1_MOVE_P0:
            if (GantryReached(FLOW_TOL_X)) { flow_sub = SUB_DONE; }
            break;

        /* 按2 */
        case SUB_2_DOWN_L3:
            if (Z_IsReached(FLOW_TOL_Z)) { ClawClose(); flow_sub = SUB_2_CLAW_CLOSE; }
            break;
        case SUB_2_CLAW_CLOSE:
            if (Axis_IsReached(AXIS_CLAW, FLOW_TOL_CLAW)) { set_z(Z_TOP); flow_sub = SUB_2_LIFT_TOP; }
            break;
        case SUB_2_LIFT_TOP:
            if (Z_IsReached(FLOW_TOL_Z)) { flow_sub = SUB_DONE; }
            break;

        /* 按3 */
        case SUB_3_MOVE_P3:
            if (GantryReached(FLOW_TOL_X)) { flow_sub = SUB_DONE; }
            break;

        /* 按4 */
        case SUB_4_DOWN_PLACE:
            if (Z_IsReached(FLOW_TOL_Z)) { flow_sub = SUB_DONE; }
            break;

        /* 按5 */
        case SUB_5_CLAW_OPEN:
            if (Axis_IsReached(AXIS_CLAW, FLOW_TOL_CLAW)) { set_z(Z_TOP); flow_sub = SUB_5_LIFT_TOP; }
            break;
        case SUB_5_LIFT_TOP:
            if (Z_IsReached(FLOW_TOL_Z)) { flow_sub = SUB_DONE; }
            break;

        /* 按6 */
        case SUB_6_MOVE_HOME:
            if (GantryReached(FLOW_TOL_X)) { set_z(FLOW_Z_BOTTOM); flow_sub = SUB_6_DOWN_BOTTOM; }
            break;
        case SUB_6_DOWN_BOTTOM:
            if (Z_IsReached(FLOW_TOL_Z)) { flow_sub = SUB_DONE; }
            break;

        case SUB_DONE:
        default:
            break;                       /* 等按键 */
    }
}

/* ============================================================================
 * 龙门四点位测试状态机(验证左右同步)
 * 按键每按一次推进一步,交替"使能去下一点 / 失能":
 *   第1按 使能->point0  第2按 失  第3按 使能->point1 ... 第9按 使能->零点  第10按 失(整轮结束)
 * 目标全部用 Axis_SetTargetXGantry 同时给左右两个龙门电机,两侧应同步跟踪。
 * 零点 = 首次使能时记录的起始 X 位置;走完零点并失能后整轮锁定,再按不动。
 * ============================================================================ */
#define GANTRY_LOOP_POINT_NUM   5   /* 4 标定点 + 1 零点 */
static int32_t gantry_loop_points[GANTRY_LOOP_POINT_NUM];
static uint8_t gantry_loop_index    = 0U;
static uint8_t gantry_loop_finished = 0U;
static uint8_t gantry_zero_captured = 0U;

void GantryFourPointStep(void)
{
    if (gantry_loop_finished)
    {
        return;                         /* 整轮已结束,按键不再响应 */
    }

    if (MotorControl_IsEnabled())
    {
        /* 使能态再按 -> 失能。若刚走完的是零点(最后一步)则整轮收尾 */
        MotorControl_Stop();
        if (gantry_loop_index >= GANTRY_LOOP_POINT_NUM)
        {
            gantry_loop_finished = 1U;
        }
        return;
    }

    /* 失能态再按 -> 使能并去下一个点位 */
    MotorControl_Enable();              /* 内部 MotorTargetSync 已把轨迹复位到实测位置 */

    /* 首次使能时记录零点 = 起始 X 位置,作为本轮最后一个目标 */
    if (!gantry_zero_captured)
    {
        gantry_loop_points[0] = X_CalPoints[0];
        gantry_loop_points[1] = X_CalPoints[1];
        gantry_loop_points[2] = X_CalPoints[2];
        gantry_loop_points[3] = X_CalPoints[3];
        gantry_loop_points[4] = Gantry_GetAveragePos();        /* 左右平均零点 */
        gantry_zero_captured = 1U;
    }

    Axis_SetTargetXGantry(gantry_loop_points[gantry_loop_index]);
    gantry_loop_index++;
}

/* 清所有轴 PID 累积量(使能瞬间用,避免上次残留导致突跳) */
static void MotorPidClear(void)
{
    for (uint8_t i = 0; i < AXIS_NUM; i++)
    {
        axis[i].pid_pos.iout = 0.0f;
        axis[i].pid_pos.pos_out = 0.0f;
        axis[i].pid_pos.last_pos_out = 0.0f;
        axis[i].pid_spd.iout = 0.0f;
        axis[i].pid_spd.pos_out = 0.0f;
        axis[i].pid_spd.last_pos_out = 0.0f;
    }
}

/* 使能瞬间把每个轴的轨迹与目标复位到当前实测位置,避免 out 从旧值跳变 */
static void MotorTargetSync(void)
{
    for (uint8_t i = 0; i < AXIS_NUM; i++)
    {
        int32_t pos = Axis_GetPos((axis_id_t)i);
        axis[i].target = pos;
        TrajTrack_Reset(&axis[i].traj, (float)pos);
    }
}

/* ============================================================================
 * 使能 / 失能
 * ============================================================================ */
void MotorControl_Enable(void)
{
    MotorTargetSync();
    MotorPidClear();
    MotorDiag.fault_code = MOTOR_FAULT_NONE;
    motor_control_enable = 1;
}

void MotorControl_Stop(void)
{
    motor_control_enable = 0;
    MotorPidClear();
    SetMotorCurrent1(&hcan1, 0, 0, 0, 0);
    SetMotorCurrent2(&hcan1, 0, 0, 0, 0);
}

uint8_t MotorControl_IsEnabled(void)
{
    return motor_control_enable;
}

/* ============================================================================
 * 控制循环
 * ============================================================================ */
void MotorPidCalc(void)
{
    uint8_t path_on = XYPath_IsActive(&crane_path);

    for (uint8_t i = 0; i < AXIS_NUM; i++)
    {
        const axis_cfg_t *c = &axis_cfg[i];

        float ref;
        float ff;

        /* 路径模式左右共用 x_ref;普通模式各自使用独立轨迹。 */
        if (path_on && (i == AXIS_GANTRY_L || i == AXIS_GANTRY_R))
        {
            ref = crane_path.x_ref;
            ff = crane_path.x_ff / RPM_TO_COUNTS_PER_SEC;
        }
        else if (path_on && i == AXIS_SLIDE_Y)
        {
            ref = crane_path.y_ref;
            ff = crane_path.y_ff / RPM_TO_COUNTS_PER_SEC;
        }
        else
        {
            ref = axis[i].traj.out;
            /* 速度前馈:轨迹瞬时速度(转子 counts/s)-> 转子 RPM,叠加到速度环 ref,
             * 消除纯P位置环滞后,使电机随 S 曲线同步减速、到点即零速、不过冲反转。 */
            ff  = axis[i].traj.vel / RPM_TO_COUNTS_PER_SEC;
        }

        if (i == AXIS_GANTRY_L)
        {
            MotorDiag.gantry_left_ref = ref;
        }
        else if (i == AXIS_GANTRY_R)
        {
            MotorDiag.gantry_right_ref = ref;
        }

        pid_calc(&axis[i].pid_pos,
                 (float)(c->dir * Motor[c->can_index].total_angle),
                 ref);
        float speed_ref = axis[i].pid_pos.pos_out * c->reduction + ff;

        pid_calc(&axis[i].pid_spd,
                 (float)(c->dir * Motor[c->can_index].speed_rpm),
                 speed_ref);
    }
}

/* 把各轴速度环输出(电流)按 can_index 装进两帧:0x200(电机1-4) / 0x1FF(电机5-8) */
void DataCanSend(void)
{
    s16 iq[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    for (uint8_t i = 0; i < AXIS_NUM; i++)
    {
        const axis_cfg_t *c = &axis_cfg[i];
        /* 速度环输出经 dir 校正后写回对应电机槽位(can_index) */
        iq[c->can_index] = (s16)(c->dir * axis[i].pid_spd.pos_out);
    }

    SetMotorCurrent1(&hcan1, iq[0], iq[1], iq[2], iq[3]);  /* ID 0x201~0x204 */
    SetMotorCurrent2(&hcan1, iq[4], iq[5], iq[6], iq[7]);  /* ID 0x205~0x208 */
}

/* vofa 上报 4 通道。
 * 默认(VOFA_WATCH_GANTRY_SYNC=1):龙门左右同步诊断
 *   ch0 龙门左实际位置  ch1 龙门右实际位置  ch2 目标位置 out  ch3 左右位置差(同步误差)
 *   —— 看 ch3:若全程贴近 0,说明左右同步良好;若发散,说明仍不同步。
 * 否则:监视单轴 VOFA_WATCH_AXIS 的 实际速度/目标速度/实际位置/目标位置。 */
#define VOFA_WATCH_GANTRY_SYNC   0
#define VOFA_UART8_REPORT_ENABLE 0

void MotorVofaReport(void)
{
#if VOFA_WATCH_GANTRY_SYNC
    int32_t pos_l = Axis_GetPos(AXIS_GANTRY_L);
    int32_t pos_r = Axis_GetPos(AXIS_GANTRY_R);
    float   l     = (float)pos_l;
    float   r     = (float)pos_r;
    float   tgt   = XYPath_IsActive(&crane_path) ? crane_path.x_ref : axis[AXIS_GANTRY_L].traj.out;
    float   err   = (float)(pos_l - pos_r);           /* 左右同步误差 */

    Vofa_Send_Data4(l, r, tgt, err);
#else
    const axis_cfg_t *c = &axis_cfg[VOFA_WATCH_AXIS];

    float act_speed = (float)(c->dir * Motor[c->can_index].speed_rpm) * RPM_TO_COUNTS_PER_SEC;
    float tgt_speed = axis[VOFA_WATCH_AXIS].traj.vel;
    float act_pos   = (float)(c->dir * Motor[c->can_index].total_angle);
    float tgt_pos   = axis[VOFA_WATCH_AXIS].traj.out;

    Vofa_Send_Data4(act_speed, tgt_speed, act_pos, tgt_pos);
#endif
}

static float MotorDistanceToObstacle(int32_t x, int32_t y,
                                     int32_t obstacle_x, int32_t obstacle_y)
{
    float dx = (float)x - (float)obstacle_x;
    float dy = (float)y - (float)obstacle_y;
    return sqrtf(dx * dx + dy * dy);
}

static uint8_t MotorRuntimeSafetyCheck(void)
{
    int32_t left = Axis_GetPos(AXIS_GANTRY_L);
    int32_t right = Axis_GetPos(AXIS_GANTRY_R);
    int32_t actual_x = Gantry_GetAveragePos();
    int32_t actual_y = Axis_GetPos(AXIS_SLIDE_Y);
    int64_t sync_error = (int64_t)left - (int64_t)right;
    float bean_distance = MotorDistanceToObstacle(actual_x, actual_y,
                                                   OBSTACLE_BEAN_X_COUNTS,
                                                   OBSTACLE_BEAN_Y_COUNTS);
    float box_distance = MotorDistanceToObstacle(actual_x, actual_y,
                                                  OBSTACLE_BOX_X_COUNTS,
                                                  OBSTACLE_BOX_Y_COUNTS);

    MotorDiag.gantry_left_pos = left;
    MotorDiag.gantry_right_pos = right;
    MotorDiag.gantry_sync_error = (int32_t)sync_error;
    MotorDiag.path_x_ref = crane_path.x_ref;
    MotorDiag.path_y_ref = crane_path.y_ref;
    MotorDiag.path_x_ff = crane_path.x_ff;
    MotorDiag.path_y_ff = crane_path.y_ff;
    MotorDiag.actual_min_clearance = (bean_distance < box_distance) ? bean_distance : box_distance;

    if (sync_error > GANTRY_SYNC_FAULT_COUNTS || sync_error < -GANTRY_SYNC_FAULT_COUNTS)
    {
        MotorDiag.fault_code = MOTOR_FAULT_GANTRY_SYNC;
        if (vision_active)
        {
            vision_sub = VISION_SUB_ERROR;
        }
        MotorControl_Stop();
        return 0U;
    }

    if (MotorDiag.actual_min_clearance < OBSTACLE_CLEARANCE_COUNTS)
    {
        MotorDiag.fault_code = MOTOR_FAULT_RUNTIME_CLEARANCE;
        if (vision_active)
        {
            vision_sub = VISION_SUB_ERROR;
        }
        MotorControl_Stop();
        return 0U;
    }

    return 1U;
}

void MotorSysTask(void)
{
#if VOFA_UART8_REPORT_ENABLE
    static uint8_t vofa_div = 0U;
#endif

    if (!motor_control_enable)
    {
        SetMotorCurrent1(&hcan1, 0, 0, 0, 0);
        SetMotorCurrent2(&hcan1, 0, 0, 0, 0);
    }
    else
    {
        /* 全部轴推进各自轨迹;路径模式下 X/Y 的独立输出暂不送入位置环。 */
        for (uint8_t i = 0; i < AXIS_NUM; i++)
        {
            TrajTrack_Calc(&axis[i].traj);
        }

        /* 路径模式:推进弧长 S 曲线 -> 插值出 X/Y 平面 ref(MotorPidCalc 里取用)。
         * 与 TrajTrack_Calc 同频(2ms)。路径模式下龙门/Y 单轴轨迹不被 PID 采用,
         * 收尾时由 CranePath_Finish 复位到实测位置。 */
        if (XYPath_IsActive(&crane_path))
        {
            XYPath_Tick(&crane_path);
        }

        if (!MotorRuntimeSafetyCheck())
        {
            return;
        }

        MotorPidCalc();
        DataCanSend();
    }

#if VOFA_UART8_REPORT_ENABLE
    /* vofa 始终发送:500Hz 控制循环分频到 250Hz */
    if (++vofa_div >= VOFA_REPORT_DIVIDER)
    {
        vofa_div = 0U;
        MotorVofaReport();
    }
#endif
}
