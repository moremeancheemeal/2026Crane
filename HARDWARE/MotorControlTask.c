#include "MotorControlTask.h"

#include "can.h"
#include "usart.h"
#include "vofaPlus.h"
#include "XYPath.h"

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
    /* AXIS_GANTRY_L (0x201) 龙门左,X —— 高速大行程,沿用已验证 nice 的那套 */
    { 0, -1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 305.0f, 3052.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_GANTRY_R (0x202) 龙门右,X —— 与左对称安装,dir 取 +1 */
    { 1, +1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 305.0f, 3052.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_SLIDE_Y (0x203) 滑行,Y —— 高速 */
    { 2, +1, MOTOR_REDUCTION_RATIO_3508,
      300.0f, 1200.0f, 6000.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u },

    /* AXIS_LIFT_Z (0x204) 升降,Z —— 小行程柔和;Vmax 测试改 100 RPM */
    { 3, +1, MOTOR_REDUCTION_RATIO_3508,
      100.0f, 240.0f, 1500.0f,
      0.004f, 0.0f, 0.0f, 100u, 10u,
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

    /* XY 联动路径插值器:弧长 S 曲线的 V/A/J 取保守值(沿路径合速度的 counts 量纲)。
     * ⚠ 合速度会按切向分解到 X、Y,任一分量不得超该轴上限。这里先用 SLIDE_Y 的约束
     *   作为合速度上限(Y 通常是较慢/较短的一轴),联调时按实际路径再调。 */
    XYPath_Init(&crane_path,
                axis_cfg[AXIS_SLIDE_Y].vmax_rpm * TRAJ_RPM_TO_COUNTS,
                axis_cfg[AXIS_SLIDE_Y].amax_rpm * TRAJ_RPM_TO_COUNTS,
                axis_cfg[AXIS_SLIDE_Y].jmax_rpm * TRAJ_RPM_TO_COUNTS,
                MOTOR_CONTROL_PERIOD);
}

/* 读某轴实测位置(dir 校正,与轨迹/目标同坐标系) */
int32_t Axis_GetPos(axis_id_t id)
{
    const axis_cfg_t *c = &axis_cfg[id];
    return c->dir * Motor[c->can_index].total_angle;
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

    if (TrajTrack_IsBusy(&axis[id].traj))
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

/* 同时给龙门架左右同一个 X 目标,保证两侧同步 */
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
void CranePath_Run(const map_pt_t *pts, uint8_t num, float blend_radius)
{
    if (!MotorControl_IsEnabled())
    {
        MotorControl_Enable();
    }

    XYPath_Build(&crane_path, pts, num, blend_radius);
    XYPath_Start(&crane_path,
                 (float)Axis_GetPos(AXIS_GANTRY_L),
                 (float)Axis_GetPos(AXIS_SLIDE_Y));
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
    int32_t y  = Axis_GetPos(AXIS_SLIDE_Y);

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
        ClawClose();
    }
    else
    {
        ClawOpen();
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
#define Z_TOP          (-230000)   /* 升降最高(留余量,避免顶死硬限位) */
#define Z_HEIGHT_NUM   9
static const int32_t z_height_points[Z_HEIGHT_NUM] =
{
    Z_TOP,     /* 抬顶(去三层前) */
    -142959,   /* 第三层 */
    Z_TOP,     /* 抬顶(去二层前) */
    -99655,    /* 第二层 */
    Z_TOP,     /* 抬顶(去一层前) */
    -58970,    /* 第一层 */
    -36243,    /* 放货高度(与 FLOW_Z_PLACE 对齐,原 -87476 为过时旧值) */
    Z_TOP,     /* 放货后抬顶 */
    0,         /* 回 Z0(上电高度) */
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
#define FLOW_Z_LEVEL3      (-142959)   /* 第三层 */
#define FLOW_Z_PLACE       (-36243)    /* 放货高度 */
#define FLOW_Z_BOTTOM      (0)         /* 底部 = 上电 Z0 */
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

/* 按键:推进到下一个大步骤。仅当上一步骤的自动接续已跑完(flow_sub==SUB_DONE)才响应,
 * 避免中途打断。走完第6步后再按 -> 从头重来。 */
void CraneFlow_Step(void)
{
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
        gantry_loop_points[4] = Axis_GetPos(AXIS_GANTRY_L);   /* 零点 */
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

        /* 路径模式:X(GANTRY_L/R)与 Y(SLIDE_Y)的位置环 ref 取自 XYPath,
         * 绕过各轴 axis[i].traj,保证末端沿确定几何路径绕障。
         * 前馈暂置 0(后期再按 x_ff/y_ff 接入);其余轴仍走各自 traj。 */
        if (path_on && (i == AXIS_GANTRY_L || i == AXIS_GANTRY_R))
        {
            ref = crane_path.x_ref;
            ff  = 0.0f;
        }
        else if (path_on && i == AXIS_SLIDE_Y)
        {
            ref = crane_path.y_ref;
            ff  = 0.0f;
        }
        else
        {
            ref = axis[i].traj.out;
            /* 速度前馈:轨迹瞬时速度(转子 counts/s)-> 转子 RPM,叠加到速度环 ref,
             * 消除纯P位置环滞后,使电机随 S 曲线同步减速、到点即零速、不过冲反转。 */
            ff  = axis[i].traj.vel / RPM_TO_COUNTS_PER_SEC;
        }

        pid_calc(&axis[i].pid_pos,
                 (float)(c->dir * Motor[c->can_index].total_angle),
                 ref);
        pid_calc(&axis[i].pid_spd,
                 (float)(c->dir * Motor[c->can_index].speed_rpm),
                 axis[i].pid_pos.pos_out * c->reduction + ff);
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

void MotorVofaReport(void)
{
#if VOFA_WATCH_GANTRY_SYNC
    int32_t pos_l = Axis_GetPos(AXIS_GANTRY_L);
    int32_t pos_r = Axis_GetPos(AXIS_GANTRY_R);
    float   l     = (float)pos_l;
    float   r     = (float)pos_r;
    float   tgt   = axis[AXIS_GANTRY_L].traj.out;     /* L/R 共享同一目标 */
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

void MotorSysTask(void)
{
    static uint8_t vofa_div = 0U;

    if (!motor_control_enable)
    {
        SetMotorCurrent1(&hcan1, 0, 0, 0, 0);
        SetMotorCurrent2(&hcan1, 0, 0, 0, 0);
    }
    else
    {
        /* 每个控制周期推进一次所有轴的轨迹,得到此刻瞬时目标 */
        for (uint8_t i = 0; i < AXIS_NUM; i++)
        {
            TrajTrack_Calc(&axis[i].traj);
        }

        /* 路径模式:推进弧长 S 曲线 -> 插值出 X/Y 平面 ref(MotorPidCalc 里取用)。
         * 与 TrajTrack_Calc 同频(2ms)。X/Y 的 axis[].traj 仍在上面被推进,但路径模式下
         * 其 out 不被 MotorPidCalc 采用,联调收尾时由 CranePath_Stop 把 traj 复位对齐。 */
        if (XYPath_IsActive(&crane_path))
        {
            XYPath_Tick(&crane_path);
        }

        MotorPidCalc();
        DataCanSend();
    }

    /* vofa 始终发送:500Hz 控制循环分频到 250Hz */
    if (++vofa_div >= VOFA_REPORT_DIVIDER)
    {
        vofa_div = 0U;
        MotorVofaReport();
    }
}
