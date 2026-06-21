#include "MotorControlTask.h"

#include "can.h"

MotorSystem_t MotorSystem;
pid_t Moto_sys_Pid_Spd[MOTOR_NUM];
pid_t Moto_sys_Pid_Pos[MOTOR_NUM];
moto_measure_t Motor[MOTOR_NUM];
traj_track_t MotorTraj[MOTOR_NUM];

int StraightTargetAngle;
int SlideAngle;
int ClawTargetAngle;

static uint8_t motor_control_enable = 0U;

void MotorTrajInit(void)
{
    /* 直行两个 3508 共用 STRAIGHT_MOTOR_0 这条轨迹;MOTOR_1 这条留作占位,保持同步 */
    TrajTrack_Init(&MotorTraj[STRAIGHT_MOTOR_0],
                   TRAJ_STRAIGHT_VMAX, TRAJ_STRAIGHT_AMAX, TRAJ_STRAIGHT_JMAX,
                   MOTOR_CONTROL_PERIOD, 0.0f);
    TrajTrack_Init(&MotorTraj[STRAIGHT_MOTOR_1],
                   TRAJ_STRAIGHT_VMAX, TRAJ_STRAIGHT_AMAX, TRAJ_STRAIGHT_JMAX,
                   MOTOR_CONTROL_PERIOD, 0.0f);
    TrajTrack_Init(&MotorTraj[SLIDE_203],
                   TRAJ_SLIDE_VMAX, TRAJ_SLIDE_AMAX, TRAJ_SLIDE_JMAX,
                   MOTOR_CONTROL_PERIOD, 0.0f);
    TrajTrack_Init(&MotorTraj[CLAW_206],
                   TRAJ_CLAW_VMAX, TRAJ_CLAW_AMAX, TRAJ_CLAW_JMAX,
                   MOTOR_CONTROL_PERIOD, 0.0f);
}

static void MotorPidClear(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        Moto_sys_Pid_Pos[i].iout = 0.0f;
        Moto_sys_Pid_Pos[i].pos_out = 0.0f;
        Moto_sys_Pid_Pos[i].last_pos_out = 0.0f;
        Moto_sys_Pid_Spd[i].iout = 0.0f;
        Moto_sys_Pid_Spd[i].pos_out = 0.0f;
        Moto_sys_Pid_Spd[i].last_pos_out = 0.0f;
    }
}

static void MotorTargetSync(void)
{
    StraightTargetAngle = ((STRAIGHT_MOTOR0_SIGN * Motor[STRAIGHT_MOTOR_0].total_angle) +
                           (STRAIGHT_MOTOR1_SIGN * Motor[STRAIGHT_MOTOR_1].total_angle)) / 2;
    SlideAngle = Motor[SLIDE_203].total_angle;
    ClawTargetAngle = Motor[CLAW_206].total_angle;

    /* 使能瞬间把轨迹复位到当前实际位置,避免 out 从 0 跳变 */
    TrajTrack_Reset(&MotorTraj[STRAIGHT_MOTOR_0], (float)StraightTargetAngle);
    TrajTrack_Reset(&MotorTraj[SLIDE_203], (float)SlideAngle);
    TrajTrack_Reset(&MotorTraj[CLAW_206], (float)ClawTargetAngle);
}

void Straight(uint8_t direction, int targetdangle)
{
    if (direction == STRAIGHT_BACKWARD)
    {
        StraightTargetAngle -= targetdangle;
    }
    else
    {
        StraightTargetAngle += targetdangle;
    }

    TrajTrack_SetTarget(&MotorTraj[STRAIGHT_MOTOR_0], (float)StraightTargetAngle);
}

void SLIDE(int angle)
{
    SlideAngle = angle;
    TrajTrack_SetTarget(&MotorTraj[SLIDE_203], (float)SlideAngle);
}

void Claw(uint8_t direction, int targetdangle)
{
    if (direction == CLAW_OPEN)
    {
        ClawTargetAngle -= targetdangle;
        if (ClawTargetAngle < CLAW_OPEN_ANGLE)
        {
            ClawTargetAngle = CLAW_OPEN_ANGLE;
        }
    }
    else
    {
        ClawTargetAngle += targetdangle;
        if (ClawTargetAngle > CLAW_CLOSED_ANGLE)
        {
            ClawTargetAngle = CLAW_CLOSED_ANGLE;
        }
    }

    TrajTrack_SetTarget(&MotorTraj[CLAW_206], (float)ClawTargetAngle);
}

void ClawSetAngle(int target_angle)
{
    if (target_angle < CLAW_OPEN_ANGLE)
    {
        target_angle = CLAW_OPEN_ANGLE;
    }
    else if (target_angle > CLAW_CLOSED_ANGLE)
    {
        target_angle = CLAW_CLOSED_ANGLE;
    }

    ClawTargetAngle = target_angle;
    TrajTrack_SetTarget(&MotorTraj[CLAW_206], (float)ClawTargetAngle);
}

void Motor6Angle(int targetdangle)
{
    Claw(CLAW_CLOSE, targetdangle);
}

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

void MotorSysTask(void)
{
    if (!motor_control_enable)
    {
        SetMotorCurrent1(&hcan1, 0, 0, 0, 0);
        SetMotorCurrent2(&hcan1, 0, 0, 0, 0);
        return;
    }

    /* 每个控制周期推进一次轨迹,得到此刻的瞬时目标 */
    TrajTrack_Calc(&MotorTraj[STRAIGHT_MOTOR_0]);
    TrajTrack_Calc(&MotorTraj[SLIDE_203]);
    TrajTrack_Calc(&MotorTraj[CLAW_206]);

    MotorPidCalc();
    DataCanSend();
}

void MotorPidCalc(void)
{
    /* 位置环 ref 改用轨迹生成器输出的瞬时目标 out,电机"跟踪"平滑曲线 */
    float straight_ref = MotorTraj[STRAIGHT_MOTOR_0].out;
    float slide_ref    = MotorTraj[SLIDE_203].out;
    float claw_ref     = MotorTraj[CLAW_206].out;

    pid_calc(&Moto_sys_Pid_Pos[STRAIGHT_MOTOR_0],
             STRAIGHT_MOTOR0_SIGN * Motor[STRAIGHT_MOTOR_0].total_angle,
             straight_ref);
    pid_calc(&Moto_sys_Pid_Spd[STRAIGHT_MOTOR_0],
             STRAIGHT_MOTOR0_SIGN * Motor[STRAIGHT_MOTOR_0].speed_rpm,
             Moto_sys_Pid_Pos[STRAIGHT_MOTOR_0].pos_out * MOTOR_REDUCTION_RATIO_3508);

    pid_calc(&Moto_sys_Pid_Pos[STRAIGHT_MOTOR_1],
             STRAIGHT_MOTOR1_SIGN * Motor[STRAIGHT_MOTOR_1].total_angle,
             straight_ref);
    pid_calc(&Moto_sys_Pid_Spd[STRAIGHT_MOTOR_1],
             STRAIGHT_MOTOR1_SIGN * Motor[STRAIGHT_MOTOR_1].speed_rpm,
             Moto_sys_Pid_Pos[STRAIGHT_MOTOR_1].pos_out * MOTOR_REDUCTION_RATIO_3508);

    pid_calc(&Moto_sys_Pid_Pos[SLIDE_203], Motor[SLIDE_203].total_angle, slide_ref);
    pid_calc(&Moto_sys_Pid_Spd[SLIDE_203],
             Motor[SLIDE_203].speed_rpm,
             Moto_sys_Pid_Pos[SLIDE_203].pos_out * MOTOR_REDUCTION_RATIO_3508);

    pid_calc(&Moto_sys_Pid_Pos[CLAW_206], Motor[CLAW_206].total_angle, claw_ref);
    pid_calc(&Moto_sys_Pid_Spd[CLAW_206],
             Motor[CLAW_206].speed_rpm,
             Moto_sys_Pid_Pos[CLAW_206].pos_out * MOTOR_REDUCTION_RATIO_3508 * CLAW_SPEED_SCALE);
}

void DataCanSend(void)
{
    SetMotorCurrent1(&hcan1,
                     (s16)(STRAIGHT_MOTOR0_SIGN * Moto_sys_Pid_Spd[STRAIGHT_MOTOR_0].pos_out),
                     (s16)(STRAIGHT_MOTOR1_SIGN * Moto_sys_Pid_Spd[STRAIGHT_MOTOR_1].pos_out),
                     (s16)Moto_sys_Pid_Spd[SLIDE_203].pos_out,
                     0);
    SetMotorCurrent2(&hcan1,
                     0,
                     (s16)Moto_sys_Pid_Spd[CLAW_206].pos_out,
                     0,
                     0);
}
