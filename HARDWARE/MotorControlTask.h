#ifndef __MOTOR_CONTROL_TASK_H
#define __MOTOR_CONTROL_TASK_H

#include "PID.h"
#include "CanBus_Task.h"
#include "TrajTrack.h"

#define MOTOR_REDUCTION_RATIO_3508 19.2f

/* ============================================================================
 * 量纲换算层 —— 用直观的"输出轴 RPM"设定 S 曲线约束,编译期转成内部 counts
 * ----------------------------------------------------------------------------
 * 内部轨迹生成器(TrajTrack)统一以"转子编码器 counts"为单位:
 *     位置 counts / 速度 counts·s^-1 / 加速度 counts·s^-2 / 加加速度 counts·s^-3
 * counts 数字大又抽象,故配置表里用【输出轴 RPM】填写,由下面的系数自动换算。
 *
 *   输出轴转 1 圈 = 减速比 × 8192 counts = 19.2 × 8192 = 157286.4 counts
 *   1 分钟 = 60 秒  =>  1 输出轴RPM = 157286.4 / 60 = 2621.44 counts/s
 * V/A/J 共用同一系数(每高一阶仅在单位上多一个 1/s,数值系数相同)。
 * 参考:M3508 经 19.2 减速箱,输出轴极限约 469 RPM(转子 ~9000 RPM)。
 * ============================================================================ */
#define MOTOR_ENCODER_CPR       8192.0f   /* 编码器每转子圈 counts */
/* 输出轴 RPM -> 内部 counts/s 的换算系数 */
#define TRAJ_RPM_TO_COUNTS      (MOTOR_REDUCTION_RATIO_3508 * MOTOR_ENCODER_CPR / 60.0f)
/* 转子 RPM <-> 编码器 counts/s(8192 counts/转),用于速度前馈与 vofa 速度上报 */
#define RPM_TO_COUNTS_PER_SEC   (MOTOR_ENCODER_CPR / 60.0f)

/* ============================================================================
 * 轴定义 —— 6 个独立 M3508,都挂 CAN1,ID 0x201~0x206
 *   GANTRY_L/R 为龙门架左右两侧,独立闭环、共享 X 目标(保证两侧同步)
 * ============================================================================ */
typedef enum
{
    AXIS_GANTRY_L = 0,   /* 0x201 龙门左,X */
    AXIS_GANTRY_R = 1,   /* 0x202 龙门右,X(与 L 共享目标) */
    AXIS_SLIDE_Y  = 2,   /* 0x203 滑行,Y(与 X 构成水平面) */
    AXIS_LIFT_Z   = 3,   /* 0x204 升降,Z */
    AXIS_YAW      = 4,   /* 0x205 旋转,Yaw */
    AXIS_CLAW     = 5,   /* 0x206 爪子 */
    AXIS_NUM      = 6,
} axis_id_t;

/* 物理电机数量(与轴数一一对应,can_index = 0..MOTOR_NUM-1) */
#define MOTOR_NUM   AXIS_NUM

/* 一个轴的全部静态参数(编译期配置表的一行) */
typedef struct
{
    uint8_t  can_index;     /* 在 Motor[] / CAN 回调里的下标(本工程 = CAN ID - 0x201) */
    int8_t   dir;           /* 安装方向 +1 / -1,使实测位置与轨迹同号 */
    float    reduction;     /* 减速比(M3508 = 19.2) */

    /* S 曲线约束(输出轴 RPM 量纲,Init 时 × TRAJ_RPM_TO_COUNTS 转 counts) */
    float    vmax_rpm;      /* 最高速度    输出轴 RPM      */
    float    amax_rpm;      /* 最大加速度  输出轴 RPM/s    */
    float    jmax_rpm;      /* 最大加加速度 输出轴 RPM/s^2 */

    /* 位置环 PID(对应 PID_struct_init 入参) */
    float    pos_kp, pos_ki, pos_kd;
    uint32_t pos_max;       /* 位置环输出限幅(= 速度环目标上限,输出轴RPM量纲) */
    uint32_t pos_ilim;      /* 位置环积分限幅 */

    /* 速度环 PID */
    float    spd_kp, spd_ki, spd_kd;
    uint32_t spd_max;       /* 速度环输出限幅(= 电流上限) */
    uint32_t spd_ilim;      /* 速度环积分限幅 */

    float    spd_scale;     /* 速度环参考额外缩放(原 CLAW_SPEED_SCALE,一般 1.0) */
} axis_cfg_t;

/* 一个轴的运行态(取代原来散落的全局) */
typedef struct
{
    traj_track_t traj;      /* S 曲线轨迹生成器 */
    pid_t        pid_pos;   /* 位置环 */
    pid_t        pid_spd;   /* 速度环 */
    int32_t      target;    /* 当前最终目标(counts,dir 校正后坐标系) */
} axis_t;

extern axis_cfg_t axis_cfg[AXIS_NUM];   /* 编译期配置表(定义在 .c) */
extern axis_t     axis[AXIS_NUM];        /* 运行态 */
extern moto_measure_t Motor[MOTOR_NUM];  /* CAN 回传的电机原始数据 */

/* ---- X 轴标定点(留作参考,来自原四点位测试;单位:输出轴 counts) ---- */
#define X_CAL_POINT0   (-1636607)
#define X_CAL_POINT1   (-2019861)
#define X_CAL_POINT2   (1652387)
#define X_CAL_POINT3   (1860000)

/* vofa 监视:选要画曲线的轴(实际速度/目标速度/实际位置/目标位置 4 通道) */
#define VOFA_WATCH_AXIS    AXIS_LIFT_Z
/* vofa 上报分频:控制循环 500Hz,每 2 周期发一帧 => 250Hz */
#define VOFA_REPORT_DIVIDER   2

/* ============================================================================
 * 封装 API
 * ============================================================================ */
void    Axis_InitAll(void);                          /* 按配置表初始化全部轴(traj + 两个 PID) */
void    Axis_SetTarget(axis_id_t id, int32_t target);/* 设某轴最终目标,内部走 TrajTrack_SetTarget */
void    Axis_SetTargetXGantry(int32_t x);            /* 同时给 GANTRY_L/R 同一 X 目标(同步) */
int32_t Axis_GetPos(axis_id_t id);                   /* 读某轴实测位置(dir 校正,counts) */
uint8_t Axis_IsBusy(axis_id_t id);                   /* 某轴是否正在跑轨迹 */
/* 某轴是否"真正到位":轨迹已走完(!busy) 且 实测位置进了 ±tol_counts 容差。
 * 非阻塞,主循环里轮询即可。tol_counts 取几百~几千,看精度要求。 */
uint8_t Axis_IsReached(axis_id_t id, int32_t tol_counts);

void    MotorControl_Enable(void);                   /* 同步所有轴到实测位置 + 清 PID + 使能 */
void    MotorControl_Stop(void);                     /* 失能,电流清零 */
uint8_t MotorControl_IsEnabled(void);

void    MotorSysTask(void);                          /* 每控制周期:推进轨迹 + PID + CAN + vofa */
void    MotorPidCalc(void);                          /* 双环 PID(遍历所有轴) */
void    DataCanSend(void);                           /* 把各轴速度环输出装帧发 CAN */
void    MotorVofaReport(void);                       /* vofa 上报 VOFA_WATCH_AXIS 的 4 通道 */

/* 龙门四点位测试:按键每按一次推进一步(交替 使能去下一点 / 失能),
 * 用 Axis_SetTargetXGantry 同步驱动左右两个龙门电机。由按键回调调用。 */
void    GantryFourPointStep(void);

/* ---- Z 升降轴上层封装(在 Axis_* 之上再包一层,用"高度 counts"指挥) ----
 * 高度 = 相对上电位置的 counts;上电位置 = Z0(电机 total_angle 上电归零)。
 * 方向:负 = 上升(机械实际方向决定,set_z 不做内部翻转,你传什么符号就是什么)。 */
void    set_z(int32_t z_counts);          /* 设 Z 目标高度,内部走 S 曲线平滑升降 */
int32_t Z_GetHeight(void);                /* 读当前 Z 实际高度(counts) */
uint8_t Z_IsReached(int32_t tol_counts);  /* Z 是否到位(非阻塞,轨迹走完且进容差) */

/* Z 升降高度测试:按键交替式推进(三层->二层->一层->放货->回零,循环)。由按键回调调用。 */
void    ZHeightStep(void);

#endif
