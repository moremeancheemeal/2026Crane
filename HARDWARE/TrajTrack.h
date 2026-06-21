#ifndef __TRAJ_TRACK_H
#define __TRAJ_TRACK_H

#include "stm32f4xx_hal.h"

/*
 * 通用 S 型(jerk 受限七段式)轨迹生成器
 * ------------------------------------------------------------------
 * 思路:上层只设定"最终要到达的位置"(target),生成器在每个控制周期
 *       调用一次 TrajTrack_Calc(),按照加加速度(jerk)受限的七段式
 *       加减速曲线,算出"此刻电机应该在的位置"并写入 out。
 *       把 out 当作位置环 PID 的 ref,电机就会平滑地"跟踪"这条曲线,
 *       而不是被 PID 一步拉到终点。
 *
 * 约束:Vmax / Amax / Jmax,单位与 target 一致(本工程用编码器 counts)。
 *       counts / counts·s^-1 / counts·s^-2 / counts·s^-3。
 *
 * 规划:rest-to-rest(每段从静止规划到静止),起点取上一条轨迹的终点
 *       (即当前 out)。
 *
 * 中途改目标:做完再接 —— 当前轨迹未走完时,新的目标会被暂存为 pending,
 *             待当前轨迹结束后自动接着规划。
 */

/* 控制周期,与 TIM3(1kHz)调用 TrajTrack_Calc 的节奏一致 */
#define MOTOR_CONTROL_PERIOD   0.001f

typedef struct
{
    /* ---- 约束(运行前由 Init 设定,可随时改) ---- */
    float v_max;            /* 最大速度  counts/s   */
    float a_max;            /* 最大加速度 counts/s^2 */
    float j_max;            /* 最大加加速度 counts/s^3 */
    float period;           /* 控制周期 s */

    /* ---- 已规划好的七段式曲线(正向距离域,即位移恒 >=0) ---- */
    float seg_j[7];         /* 每段 jerk */
    float seg_t0[7];        /* 每段起始的累计时间 */
    float seg_p0[7];        /* 每段起始位移 */
    float seg_v0[7];        /* 每段起始速度 */
    float seg_a0[7];        /* 每段起始加速度 */
    float total_time;       /* 整条轨迹总时长 */
    float dir;              /* 运动方向 +1 / -1 */
    float plan_start;       /* 规划时的起点位置(= out) */

    /* ---- 实时状态 ---- */
    float target;           /* 当前正在跟踪的最终目标 */
    float out;              /* 此刻目标位置(喂给 PID 的 ref) */
    float vel;              /* 此刻速度,便于监视 */
    float acc;              /* 此刻加速度,便于监视 */
    float t;                /* 当前轨迹已运行的时间 */
    uint8_t busy;           /* 1 = 轨迹运行中 */

    /* ---- 做完再接:暂存目标 ---- */
    float pending_target;
    uint8_t has_pending;
} traj_track_t;

/* 初始化:设定约束、控制周期,并把 out/target 置为 start_pos(不产生跳变) */
void TrajTrack_Init(traj_track_t *tr, float v_max, float a_max, float j_max,
                    float period, float start_pos);

/* 设定最终目标:空闲则立即规划,运行中则暂存(做完再接) */
void TrajTrack_SetTarget(traj_track_t *tr, float target);

/* 把 out/target 直接复位到 pos,清掉当前轨迹与 pending(用于使能/同步) */
void TrajTrack_Reset(traj_track_t *tr, float pos);

/* 推进一个控制周期,返回此刻目标位置 out。每个周期调用一次。 */
float TrajTrack_Calc(traj_track_t *tr);

/* 当前是否正在跟踪轨迹 */
uint8_t TrajTrack_IsBusy(const traj_track_t *tr);

#endif
