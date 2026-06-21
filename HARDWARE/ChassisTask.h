#ifndef __CHASSISTASK_H
#define __CHASSISTASK_H

#include "pid.h"
#include "CanBus_Task.h"

//3508在带有麦轮负载且悬空的情况下的最大转速为430rpm左右，设置最大速度尽量不要超过这个数值
//#define CHASSIS_MAXSPEED_RPM            80       //普通时候的速度，工程底盘电机最大转速，单位：rpm   (300)
#define CHASSIS_MAXSPEED_RPM            200       //普通时候的速度，工程底盘电机最大转速，单位：rpm   (300)
#define BACK_CHASSIS_SHIFT_MAXSPEED_RPM    300    //工程底盘SHIFT向后电机最大转速，单位：rpm
#define CHASSIS_SHIFT_MAXSPEED_RPM     300    //工程底盘电机SHIFT最大转速，单位：rpm
#define CHASSIS_VX_MAXSPEED_RPM       100    //底盘左右平移时，底盘电机的最大速度，单位：rpm
#define REDUCTION_RATIO_3508          19.2f   //3508电机减速比
#define CHASSIS_CATCH_MAXSPEED_RPM    200       //工程底盘对位电机最大转速，单位：rpm


/*底盘电机序号*/
enum{
    
    FRON_RIGH_201 = 0, //前右
    FRON_LEFT_202 = 1, //前左
    REAR_LEFT_203 = 2, //后左
    REAR_RIGH_204 = 3, //后右
    
};

/*底盘结构体*/
typedef struct{
    
	  //底盘电机
    int32_t  fr_motor_angle_201; //前右电机
    int32_t  fl_motor_angle_202; //前左电机
    int32_t  rl_motor_angle_203; //后左电机
    int32_t  rr_motor_angle_204; //后右电机
	
	int32_t  fr_motor_rpm_201; //前右电机
    int32_t  fl_motor_rpm_202; //前左电机
    int32_t  rl_motor_rpm_203; //后左电机
    int32_t  rr_motor_rpm_204; //后右电机
    
    //车体坐标系中的速度
    float vx; //左右平移
    float vy; //前后
    float vw; //自转
    
    //车体坐标系中的位移
    float car_vx; 
    float car_vy;
    float car_vw; 
    
    //轮子对应车体坐标系的速度
    float wheel_rad_201; //轮子的转速，单位：rad/s
    float wheel_rad_202;
    float wheel_rad_203;
    float wheel_rad_204;
    
    
		//
		int16_t     last_vx;
		int16_t     real_vx;
		int16_t     last_vy;
		int16_t     real_vy;
		int8_t      vx_add_sub;
		int8_t      vy_add_sub;

}chassis_t;

typedef __packed struct
{
    float input;        //输入数据
    float out;          //输出数据  
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

//结构体
extern chassis_t  Chassis;  /*用于底盘*/
extern pid_t  Moto_Chassis_Pid_Spd[4];  //CAN1速度环PID结构体
extern pid_t  Moto_Chassis_Pid_Pos[4];  //CAN1位置环PID结构体
extern moto_measure_t  Chassis_Motor[4];  //底盘电机参数结构体
extern pid_t x_coor_pid;
extern pid_t y_coor_pid;
extern pid_t angle_pid;


void ChassisTask(void);
void ChassisDataUpdate(void);
void ChassisPidCalc(void);
void ChassisDataCanSend(void);

#endif
