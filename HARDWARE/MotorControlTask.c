#include "MotorControlTask.h"

#include "can.h"

MotorSystem_t MotorSystem;
pid_t Moto_sys_Pid_Spd[MOTOR_NUM];
pid_t Moto_sys_Pid_Pos[MOTOR_NUM];
moto_measure_t Motor[MOTOR_NUM];

int StraightTargetAngle;
int SlideAngle;
int ClawTargetAngle;

static uint8_t motor_control_enable = 0U;
static uint8_t straight_loop_enable = 0U;
static uint8_t straight_loop_index = 0U;
static uint8_t straight_loop_waiting = 0U;
static uint32_t straight_loop_wait_tick = 0U;
static int32_t straight_loop_points[4];

#define STRAIGHT_LOOP_ERROR_THRESHOLD 10000L
#define STRAIGHT_LOOP_WAIT_MS         1000U

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
}

static int32_t StraightCurrentAngle(void)
{
    return ((STRAIGHT_MOTOR0_SIGN * Motor[STRAIGHT_MOTOR_0].total_angle) +
            (STRAIGHT_MOTOR1_SIGN * Motor[STRAIGHT_MOTOR_1].total_angle)) / 2;
}

static void StraightLoopGotoPoint(uint8_t index)
{
    int32_t target = straight_loop_points[index];
    int32_t delta = target - StraightTargetAngle;

    if (delta > 0)
    {
        Straight(STRAIGHT_FORWARD, delta);
    }
    else if (delta < 0)
    {
        Straight(STRAIGHT_BACKWARD, -delta);
    }
}

static void StraightFourPointLoopTask(void)
{
    int32_t error;

    if (!straight_loop_enable)
    {
        return;
    }

    error = StraightTargetAngle - StraightCurrentAngle();

    if (!straight_loop_waiting)
    {
        if (error < STRAIGHT_LOOP_ERROR_THRESHOLD && error > -STRAIGHT_LOOP_ERROR_THRESHOLD)
        {
            straight_loop_waiting = 1U;
            straight_loop_wait_tick = HAL_GetTick();
        }

        return;
    }

    if ((uint32_t)(HAL_GetTick() - straight_loop_wait_tick) >= STRAIGHT_LOOP_WAIT_MS)
    {
        straight_loop_index = (uint8_t)((straight_loop_index + 1U) % 4U);
        straight_loop_waiting = 0U;
        StraightLoopGotoPoint(straight_loop_index);
    }
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
}

void StraightFourPointLoopStart(int32_t point0, int32_t point1, int32_t point2, int32_t point3)
{
    straight_loop_points[0] = point0;
    straight_loop_points[1] = point1;
    straight_loop_points[2] = point2;
    straight_loop_points[3] = point3;

    straight_loop_enable = 1U;
    straight_loop_index = 0U;
    straight_loop_waiting = 0U;
    straight_loop_wait_tick = 0U;

    StraightLoopGotoPoint(straight_loop_index);
}

void StraightFourPointLoopStop(void)
{
    straight_loop_enable = 0U;
    straight_loop_index = 0U;
    straight_loop_waiting = 0U;
    straight_loop_wait_tick = 0U;
}

void SLIDE(int angle)
{
    SlideAngle = angle;
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
    StraightFourPointLoopStop();
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

    StraightFourPointLoopTask();
    MotorPidCalc();
    DataCanSend();
}

void MotorPidCalc(void)
{
    pid_calc(&Moto_sys_Pid_Pos[STRAIGHT_MOTOR_0],
             STRAIGHT_MOTOR0_SIGN * Motor[STRAIGHT_MOTOR_0].total_angle,
             StraightTargetAngle);
    pid_calc(&Moto_sys_Pid_Spd[STRAIGHT_MOTOR_0],
             STRAIGHT_MOTOR0_SIGN * Motor[STRAIGHT_MOTOR_0].speed_rpm,
             Moto_sys_Pid_Pos[STRAIGHT_MOTOR_0].pos_out * MOTOR_REDUCTION_RATIO_3508);

    pid_calc(&Moto_sys_Pid_Pos[STRAIGHT_MOTOR_1],
             STRAIGHT_MOTOR1_SIGN * Motor[STRAIGHT_MOTOR_1].total_angle,
             StraightTargetAngle);
    pid_calc(&Moto_sys_Pid_Spd[STRAIGHT_MOTOR_1],
             STRAIGHT_MOTOR1_SIGN * Motor[STRAIGHT_MOTOR_1].speed_rpm,
             Moto_sys_Pid_Pos[STRAIGHT_MOTOR_1].pos_out * MOTOR_REDUCTION_RATIO_3508);

    pid_calc(&Moto_sys_Pid_Pos[SLIDE_203], Motor[SLIDE_203].total_angle, SlideAngle);
    pid_calc(&Moto_sys_Pid_Spd[SLIDE_203],
             Motor[SLIDE_203].speed_rpm,
             Moto_sys_Pid_Pos[SLIDE_203].pos_out * MOTOR_REDUCTION_RATIO_3508);

    pid_calc(&Moto_sys_Pid_Pos[CLAW_206], Motor[CLAW_206].total_angle, ClawTargetAngle);
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
