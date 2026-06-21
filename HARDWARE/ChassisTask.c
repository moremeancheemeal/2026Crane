#include "RC_Task.h"
#include "mytype.h"
#include "CanBus_Task.h"
#include "PID.h"
#include "Paraminit.h"
#include "MotorTest.h"
#include "ControlTask.h"
#include "ChassisTask.h"
#include "Ramp.h"
#include "flow_decode.h"

//标志位
char gimbal_view_chasiis_flag = 0;//用于底盘方向改变标志位

//结构体
chassis_t  Chassis;  /*用于底盘*/
pid_t  Moto_Chassis_Pid_Spd[4];  //CAN1速度环PID结构体
pid_t  Moto_Chassis_Pid_Pos[4];
moto_measure_t  Chassis_Motor[4];  //底盘电机参数结构体

pid_t x_coor_pid;
pid_t y_coor_pid;
pid_t angle_pid;

extern uint8_t limit_control_flag;

/*********************************底盘所有的任务*******************************/
	void ChassisTask(void)
{
	
//		ChassisSpeedTest();
    ChassisDataUpdate(); 
    ChassisPidCalc(); 
    ChassisDataCanSend();

}

/***************底盘赋值任务（包括PC模式还有RC模式的赋值）**********************/
void ChassisDataUpdate(void)
{
		if(Remote_Control_Mode == 0)//RC控制
		{
			int16_t chassis_vx_channel = 0, chassis_vy_channel = 0, chassis_vw_channel = 0;
		
        /*遥控器死区限制*/
        chassis_vx_channel = RcDeadlineLimit(remote_control.ch1, 10)*CHASSIS_MAXSPEED_RPM/660;
        chassis_vy_channel = RcDeadlineLimit(remote_control.ch2, 10)*CHASSIS_MAXSPEED_RPM/660;
        chassis_vw_channel = RcDeadlineLimit(remote_control.ch3, 10)*CHASSIS_MAXSPEED_RPM/660;

        Chassis.vx = chassis_vx_channel; //左右平移
        Chassis.vy = chassis_vy_channel; //前后运动
        Chassis.vw = chassis_vw_channel; //自转
        
        Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
        Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
        Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
		
		}
		else if(Remote_Control_Mode == 1)//坐标设置模式
		{
			if (angle_diff(coor.target_angle, coor.angle) <= 5.2f && (angle_diff(coor.target_angle, coor.angle)) >= -5.2f)
			{
				PID_struct_init(&angle_pid, POSITION_PID,10, 10, -500.0f, 0.0f, 0.0f);//角度环闭环
				limit_control_flag = 1;
				
			}
			else if(coor.angle == coor.target_angle)
			{
					angle_pid.iout = 0;
					angle_pid.pos_out = 0;
			}
			else 
			{
				PID_struct_init(&angle_pid, POSITION_PID,CHASSIS_MAXSPEED_RPM, 5, -3.0f, -0.0039f, 0.0f);//角度环闭环
				limit_control_flag = 0;
			}

			Chassis.vx = pid_calc(&x_coor_pid, coor.flow_x, coor.target_x);
			Chassis.vy = pid_calc(&y_coor_pid, coor.flow_y, coor.target_y);
			Chassis.vw = angle_pid_calc(&angle_pid, coor.angle, coor.target_angle);
			if (coor.flow_x - coor.target_x < 300  &&
				  coor.flow_x - coor.target_x > -300)
			  {
				  Chassis.vx = 0;
			  }
			if ( coor.flow_y - coor.target_y < 300  &&
				  coor.flow_y - coor.target_y > -300)
			{
				   Chassis.vy = 0;
			}
			ChassisRampInit(); //底盘斜坡初始化
			FirstOrderFilterCali(&chassis_cmd_slow_set_vx, Chassis.vx);//一阶低通滤波计算,左右平移
			FirstOrderFilterCali(&chassis_cmd_slow_set_vy, Chassis.vy);//一阶低通滤波计算,前后平移
			FirstOrderFilterCali(&chassis_cmd_slow_set_vw, Chassis.vw);//一阶低通滤波计算,转向

			if(Chassis.vx<0.001f&&Chassis.vx>(-0.001f))
			{
			   chassis_cmd_slow_set_vx.out = 0; //左右平移
			}
			if(Chassis.vy<0.001f&&Chassis.vy>(-0.001f))
			{
			   chassis_cmd_slow_set_vy.out = 0; //前后移动
			}
			Chassis.vx = chassis_cmd_slow_set_vx.out;
			Chassis.vy = chassis_cmd_slow_set_vy.out; 
			Chassis.vw = chassis_cmd_slow_set_vw.out;
			
			Chassis.fr_motor_rpm_201 = +Chassis.vx - Chassis.vy + Chassis.vw;
			Chassis.fl_motor_rpm_202 = +Chassis.vx + Chassis.vy + Chassis.vw;
			Chassis.rl_motor_rpm_203 = -Chassis.vx + Chassis.vy + Chassis.vw;
			Chassis.rr_motor_rpm_204 = -Chassis.vx - Chassis.vy + Chassis.vw;
		}

}

/******************************底盘pid计算*************************************/
void ChassisPidCalc(void)
{
	 
        pid_calc(&Moto_Chassis_Pid_Spd[FRON_RIGH_201], Chassis_Motor[FRON_RIGH_201].speed_rpm, Chassis.fr_motor_rpm_201*REDUCTION_RATIO_3508);
        pid_calc(&Moto_Chassis_Pid_Spd[FRON_LEFT_202], Chassis_Motor[FRON_LEFT_202].speed_rpm, Chassis.fl_motor_rpm_202*REDUCTION_RATIO_3508);
        pid_calc(&Moto_Chassis_Pid_Spd[REAR_LEFT_203], Chassis_Motor[REAR_LEFT_203].speed_rpm, Chassis.rl_motor_rpm_203*REDUCTION_RATIO_3508);
        pid_calc(&Moto_Chassis_Pid_Spd[REAR_RIGH_204], Chassis_Motor[REAR_RIGH_204].speed_rpm, Chassis.rr_motor_rpm_204*REDUCTION_RATIO_3508);

}

/******************************底盘电流发送*************************************/
void ChassisDataCanSend(void)
{
   	if(RC_UPPER_LEFT_SW_DOWN)
    {
				SetMotorCurrent1(&hcan1,0,0,0,0);  

    } 
		else 
		{
				SetMotorCurrent1(&hcan1, Moto_Chassis_Pid_Spd[FRON_RIGH_201].pos_out, Moto_Chassis_Pid_Spd[FRON_LEFT_202].pos_out, 
								 Moto_Chassis_Pid_Spd[REAR_LEFT_203].pos_out, Moto_Chassis_Pid_Spd[REAR_RIGH_204].pos_out);	         
		}
   
}
