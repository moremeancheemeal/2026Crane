#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

void ParamInit(void);

/*资源岛抬升机构PID位置环参数*/
#define Maxout_Lift_Island_pos 5000
#define MaxIlimit_Lift_Island_pos 10
#define KP_Lift_Island_pos 0.015f
#define KI_Lift_Island_pos 0.0f
#define KD_Lift_Island_pos 0.0f

/*资源岛抬升机构PID速度环参数*/
#define Maxout_Lift_Island_spd 12000
#define MaxIlimit_Lift_Island_spd 500
#define KP_Lift_Island_spd 24.0f
#define KI_Lift_Island_spd 0.05f
#define KD_Lift_Island_spd 2.0f

/*救援机构PID位置环参数*/
#define Maxout_Rescue_pos 5000
#define MaxIlimit_Rescue_pos 10
#define KP_Rescue_pos 0.1f
#define KI_Rescue_pos 0.0f
#define KD_Rescue_pos 0.0f

/*救援机构PID速度环参数*/
#define Maxout_Rescue_spd 30000
#define MaxIlimit_Rescue_spd 8000
#define KP_Rescue_spd 20.0f
#define KI_Rescue_spd 0.15f
#define KD_Rescue_spd 2.0f

/*W A S D 加速斜坡周期*/
#define   RAMP_W_D_CONTROL_TIME         3  //W D的控制周期
#define   RAMP_W_D_MAX                  200
#define   RAMP_W_D_MIN					0


#define   RAMP_A_S_CONTROL_TIME         3   //A S的控制周期
#define   RAMP_A_S_MAX                  0
#define   RAMP_A_S_MIN					-200




#endif


