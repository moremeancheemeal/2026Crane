#include "MotorControlTask.h"

#include "can.h"
#include "usart.h"
#include "vofaPlus.h"

/* ============================================================================
 * 编译期配置表 —— 每个轴一行,改参数只改这里
 *   V/A/J 用输出轴 RPM 量纲(见 MotorControlTask.h 换算说明)
 *   M3508 范围参考:Vmax ≤ 400(硬上限469);Amax=(1~5)×Vmax;Jmax=(3~10)×Amax
 * ----------------------------------------------------------------------------
 * 字段顺序:
 *   can_index, dir, reduction,
 *   vmax_rpm, amax_rpm, jmax_rpm,
 *   pos_kp, pos_ki, pos_kd, pos_max, pos_ilim,
 *   spd_kp, spd_ki, spd_kd, spd_max, spd_ilim,
 *   spd_scale
 * ============================================================================ */
axis_cfg_t axis_cfg[AXIS_NUM] =
{
    /* AXIS_GANTRY_L (0x201) 龙门左,X —— 高速大行程,沿用已验证 nice 的那套 */
    { 0, -1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 305.0f, 3052.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u,
      1.0f },

    /* AXIS_GANTRY_R (0x202) 龙门右,X —— 与左对称安装,dir 取 +1 */
    { 1, +1, MOTOR_REDUCTION_RATIO_3508,
      350.0f, 305.0f, 3052.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u,
      1.0f },

    /* AXIS_SLIDE_Y (0x203) 滑行,Y —— 高速 */
    { 2, +1, MOTOR_REDUCTION_RATIO_3508,
      300.0f, 1200.0f, 6000.0f,
      0.004f, 0.0f, 0.0f, 170u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u,
      1.0f },

    /* AXIS_LIFT_Z (0x204) 升降,Z —— 小行程柔和;Vmax 测试改 100 RPM */
    { 3, +1, MOTOR_REDUCTION_RATIO_3508,
      100.0f, 240.0f, 1500.0f,
      0.004f, 0.0f, 0.0f, 100u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u,
      1.0f },

    /* AXIS_YAW (0x205) 旋转,Yaw —— 慢稳 */
    { 4, +1, MOTOR_REDUCTION_RATIO_3508,
      100.0f, 200.0f, 1200.0f,
      0.004f, 0.0f, 0.0f, 100u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u,
      1.0f },

    /* AXIS_CLAW (0x206) 爪子 —— 中等(原 CLAW_SPEED_SCALE=5 用 spd_scale 体现) */
    { 5, +1, MOTOR_REDUCTION_RATIO_3508,
      150.0f, 450.0f, 3000.0f,
      0.004f, 0.0f, 0.0f, 100u, 10u,
      12.0f, 0.15f, 2.0f, 15000u, 500u,
      5.0f },
};

axis_t          axis[AXIS_NUM];
moto_measure_t  Motor[MOTOR_NUM];

static uint8_t motor_control_enable = 0U;

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
 * Z 升降轴上层封装 —— 用"高度 counts"指挥,内部走 S 曲线
 *   高度 = 相对上电位置(Z0)的 counts;上电时电机 total_angle 已归零,故 Z0 = 位置 0。
 *   方向:负 = 上升(机械实际方向决定;set_z 不做内部翻转,传入符号即目标符号)。
 * ============================================================================ */
void set_z(int32_t z_counts)
{
    Axis_SetTarget(AXIS_LIFT_Z, z_counts);
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
#define Z_TOP          (-236461)   /* 升降最大高度(顶部) */
#define Z_HEIGHT_NUM   9
static const int32_t z_height_points[Z_HEIGHT_NUM] =
{
    Z_TOP,     /* 抬顶(去三层前) */
    -142959,   /* 第三层 */
    Z_TOP,     /* 抬顶(去二层前) */
    -99655,    /* 第二层 */
    Z_TOP,     /* 抬顶(去一层前) */
    -58970,    /* 第一层 */
    -87476,    /* 放货高度 */
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
    for (uint8_t i = 0; i < AXIS_NUM; i++)
    {
        const axis_cfg_t *c = &axis_cfg[i];

        float ref = axis[i].traj.out;
        /* 速度前馈:轨迹瞬时速度(转子 counts/s)-> 转子 RPM,叠加到速度环 ref,
         * 消除纯P位置环滞后,使电机随 S 曲线同步减速、到点即零速、不过冲反转。 */
        float ff  = axis[i].traj.vel / RPM_TO_COUNTS_PER_SEC;

        pid_calc(&axis[i].pid_pos,
                 (float)(c->dir * Motor[c->can_index].total_angle),
                 ref);
        pid_calc(&axis[i].pid_spd,
                 (float)(c->dir * Motor[c->can_index].speed_rpm),
                 axis[i].pid_pos.pos_out * c->reduction * c->spd_scale + ff);
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
