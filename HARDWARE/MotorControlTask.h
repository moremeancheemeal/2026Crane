#ifndef __MOTOR_CONTROL_TASK_H
#define __MOTOR_CONTROL_TASK_H

#include "PID.h"
#include "CanBus_Task.h"

#define MOTOR_REDUCTION_RATIO_3508 19.2f

#define STRAIGHT_FORWARD           0U
#define STRAIGHT_BACKWARD          1U
#define STRAIGHT_MOTOR0_SIGN       (-1)
#define STRAIGHT_MOTOR1_SIGN       1
#define CLAW_CLOSE                 0U
#define CLAW_OPEN                  1U
#define CLAW_OPEN_ANGLE            0
#define CLAW_CLOSED_ANGLE          6684672
#define CLAW_SPEED_SCALE           5.0f

enum
{
    STRAIGHT_MOTOR_0 = 0,
    STRAIGHT_MOTOR_1 = 1,
    SLIDE_203 = 2,
    CLAW_206 = 3,
    MOTOR6_206 = CLAW_206,
    MOTOR_NUM = 4,
};

typedef struct
{
    int32_t straight_motor_angle_0;
    int32_t straight_motor_angle_1;
    int32_t slide_motor_angle_203;
    int32_t claw_angle_206;

    int32_t straight_motor_rpm_0;
    int32_t straight_motor_rpm_1;
    int32_t slide_motor_rpm_203;
    int32_t claw_rpm_206;
} MotorSystem_t;

extern MotorSystem_t MotorSystem;
extern pid_t Moto_sys_Pid_Spd[MOTOR_NUM];
extern pid_t Moto_sys_Pid_Pos[MOTOR_NUM];
extern moto_measure_t Motor[MOTOR_NUM];
extern int StraightTargetAngle;
extern int SlideAngle;
extern int ClawTargetAngle;

void Straight(uint8_t direction, int targetdangle);
void SLIDE(int angle);
void Claw(uint8_t direction, int targetdangle);
void ClawSetAngle(int target_angle);
void Motor6Angle(int targetdangle);
void MotorControl_Enable(void);
void MotorControl_Stop(void);
uint8_t MotorControl_IsEnabled(void);
void MotorSysTask(void);
void MotorPidCalc(void);
void DataCanSend(void);

#endif
