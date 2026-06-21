#include "Ramp.h"
#include "RC_Task.h"
#include "CanBus_Task.h"
#include "ChassisTask.h"


/*????????????*/
first_order_filter_type_t chassis_cmd_slow_set_vx;
first_order_filter_type_t chassis_cmd_slow_set_vy;
first_order_filter_type_t chassis_cmd_slow_set_vw;

//float CHASSIS_ACCEL_X_NUM;  //0.0633333333f  //???????
//float CHASSIS_ACCEL_Y_NUM;  //0.0799999999f  //??????
//float CHASSIS_ACCEL_W_NUM;  //0.1333333333f 


/*************************************************************************************************************
  * @brief          ?????????????
  * @author         RM
  * @param[in]      ????????????
  * @param[in]      ??????????¦Ë s
  * @param[in]      ???????
  * @retval         ?????
*************************************************************************************************************/
void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
//void FirstOrderFilterInit(first_order_filter_type_t *first_order_filter_type, float frame_period, float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
//    first_order_filter_type->out = 0.0f;
}

/*************************************************************************************************************
  * @brief          ????????????
  * @author         RM
  * @param[in]      ????????????
  * @param[in]      ??????????¦Ë s
  * @retval         ?????
*************************************************************************************************************/
void FirstOrderFilterCali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}



/**
  * @brief          §Ò???????????
  * @author         RM
  * @param[in]      §Ò??????????
  * @param[in]      ??????????¦Ë s
  * @param[in]      ????
  * @param[in]      ??§³?
  * @retval         ?????
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out =0.0f;
}

/**
  * @brief          §Ò???????????????????????§Ö???? ????¦Ë? /s ????????????????
  * @author         RM
  * @param[in]      §Ò??????????
  * @param[in]      ?????
  * @param[in]      ???????
  * @retval         ?????
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
    
}


void ChassisRampInit(void)
{
	
    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		const static float chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM};

    FirstOrderFilterInit(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    FirstOrderFilterInit(&chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    FirstOrderFilterInit(&chassis_cmd_slow_set_vw, CHASSIS_CONTROL_TIME, chassis_w_order_filter);
}





/**************************************END*******************************************/


