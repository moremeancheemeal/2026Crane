#include "RC_Task.h"
#include "mytype.h"
#include "CanBus_Task.h"
#include "PID.h"
#include "Paraminit.h"
#include "MotorTest.h"
#include "ControlTask.h"
#include "ChassisTask.h"

int32_t set_speed_test;

void ChassisSpeedTest(void)
{
    
     if(RC_UPPER_LEFT_SW_DOWN)
    {
				SetMotorCurrent1(&hcan1,0,0,0,0);  

    } 
		else 
		{
			if(Remote_Control_Mode == 0)
			{
					if(RC_UPPER_RIGHT_SW_DOWN)
					{
							set_speed_test = 100;
					}
					if(RC_UPPER_RIGHT_SW_MID)
					{
							set_speed_test = 0;
					}
					if(RC_UPPER_RIGHT_SW_UP)
					{
							set_speed_test = -100;
					}
					pid_calc(&Moto_Chassis_Pid_Spd[0], Chassis_Motor[0].speed_rpm, set_speed_test*19.2);
					pid_calc(&Moto_Chassis_Pid_Spd[1], Chassis_Motor[1].speed_rpm, set_speed_test*19.2);
					pid_calc(&Moto_Chassis_Pid_Spd[2], Chassis_Motor[2].speed_rpm, set_speed_test*19.2);
					pid_calc(&Moto_Chassis_Pid_Spd[3], Chassis_Motor[3].speed_rpm, set_speed_test*19.2);
		
			
			
			}
				
				SetMotorCurrent1(&hcan1, Moto_Chassis_Pid_Spd[0].pos_out,Moto_Chassis_Pid_Spd[1].pos_out,0,0);	         
		}

}
