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

    /* AXIS_LIFT_Z (0x204) 升降,Z —— 小行程柔和 */
    { 3, +1, MOTOR_REDUCTION_RATIO_3508,
      120.0f, 240.0f, 1500.0f,
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

/* vofa 上报 VOFA_WATCH_AXIS 的 4 通道(速度统一为转子 counts/s,与位置量纲一致):
 *   ch0 实际速度  speed_rpm -> counts/s
 *   ch1 目标速度  轨迹 vel
 *   ch2 实际位置  total_angle(dir 校正)
 *   ch3 目标位置  轨迹 out */
void MotorVofaReport(void)
{
    const axis_cfg_t *c = &axis_cfg[VOFA_WATCH_AXIS];

    float act_speed = (float)(c->dir * Motor[c->can_index].speed_rpm) * RPM_TO_COUNTS_PER_SEC;
    float tgt_speed = axis[VOFA_WATCH_AXIS].traj.vel;
    float act_pos   = (float)(c->dir * Motor[c->can_index].total_angle);
    float tgt_pos   = axis[VOFA_WATCH_AXIS].traj.out;

    Vofa_Send_Data4(act_speed, tgt_speed, act_pos, tgt_pos);
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
