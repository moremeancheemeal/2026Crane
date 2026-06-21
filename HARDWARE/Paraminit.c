#include "mytype.h"
#include "CanBus_Task.h"
#include "PID.h"
#include "Paraminit.h"
#include "ControlTask.h"
#include "MotorControlTask.h"

void ParamInit(void)
{
    for (int i = 0; i < 2; i++)
    {
        PID_struct_init(&Moto_sys_Pid_Pos[i], POSITION_PID, 30, 10, 0.004f, 0.0f, 0.0f);
    }

    PID_struct_init(&Moto_sys_Pid_Pos[SLIDE_203], POSITION_PID, 70, 10, 0.004f, 0.0f, 0.0f);
    PID_struct_init(&Moto_sys_Pid_Pos[CLAW_206], POSITION_PID, 70, 10, 0.004f, 0.0f, 0.0f);

    for (int i = 0; i < MOTOR_NUM; i++)
    {
        PID_struct_init(&Moto_sys_Pid_Spd[i], POSITION_PID, 15000, 500, 12.0f, 0.15f, 2.0f);
    }

    MotorTrajInit();
}
