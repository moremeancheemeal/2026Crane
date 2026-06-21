#ifndef __RAMP_H
#define __RAMP_H

#include "mytype.h"
#include "ChassisTask.h"
	
//
//底盘任务控制间隔 0.001s
#define CHASSIS_CONTROL_TIME 0.001f
/*************************************************************************************************/
//数值越大滤波系数就越大，数值变化越平缓
//#define CHASSIS_ACCEL_X_NUM 0.8666666667f  //0.0633333333f  //左右平移
//#define CHASSIS_ACCEL_Y_NUM 0.5333333333f  //0.0799999999f  //前后运动
//#define CHASSIS_ACCEL_W_NUM 0.2833333333f  //0.1333333333f 

#define CHASSIS_ACCEL_W_NUM 0.07   //0.1333333333f 
//#define CHASSIS_ACCEL_X_NUM 0.0733333333f  //左右平移
//#define CHASSIS_ACCEL_Y_NUM 0.4899999999f  //前后运动

#define CHASSIS_ACCEL_X_NUM 0.03  //左右平移
#define CHASSIS_ACCEL_Y_NUM 0.03 //前后运动

/************************************************************************************************/


//用于底盘一阶滤波启动
typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;


//typedef struct{
//    
//    int32_t compare_value_up;
//    int32_t compare_value_dowm;
//    int sin_ramp_switch;
//    
//}SinRampState;


extern first_order_filter_type_t chassis_cmd_slow_set_vx;
extern first_order_filter_type_t chassis_cmd_slow_set_vy;
extern first_order_filter_type_t chassis_cmd_slow_set_vw;


//一阶滤波初始化
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period,float num[1]);

//一阶滤波计算
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input);


void ChassisRampInit(void);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);


//int32_t SinRampCalc(SinRampState *sin_state, int32_t variation, uint8_t Sin_control_time_up, uint8_t Sin_control_time_down);

//void SinRampInit(SinRampState *sin_state);

//void SinTest(void);

#endif
