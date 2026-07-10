#ifndef __XY_PATH_H
#define __XY_PATH_H

#include "stm32f4xx_hal.h"
#include "TrajTrack.h"
#include "MapCoords.h"

/* ============================================================================
 * XY 联动路径插值器(折线 + 拐角圆弧混合)
 * ----------------------------------------------------------------------------
 * 目的:比赛不能越障,X(龙门)与 Y(小车)必须在水平面内沿一条【确定的几何
 *       路径】协同运动,而不是各走各的 S 曲线导致末端轨迹形状不可控。
 *
 * 架构:路径 / 速度分离
 *   1) 几何路径 p(s) = (x(s), y(s)),s = 沿路径弧长,0 .. total_len。
 *      由若干航点(map_pt_t)定义,拐角用圆弧混合保证切向连续(全程不停顿)。
 *   2) 弧长上的速度规划:复用现有 TrajTrack(7 段 jerk 受限 S 曲线),对标量
 *      s 做 0 -> total_len 的规划 —— 这就是你们已验证的"S 型加减速算法",
 *      只是被作用对象从单电机位置换成了"路径进度 s"。
 *   3) 每控制周期 XYPath_Tick():
 *        s   = traj.out         (本刻路径进度)
 *        sdot= traj.vel         (本刻沿路径速率)
 *        (x,y)         = 路径在 s 处插值
 *        (dx/ds,dy/ds) = 路径在 s 处单位切向
 *        x_ref = x, y_ref = y                         -> 喂 X/Y 位置环
 *        x_ff  = (dx/ds)*sdot, y_ff = (dy/ds)*sdot    -> 喂 X/Y 速度前馈
 *
 * 单位:全部用 counts 域(与 TrajTrack / 轴目标一致)。航点由 MapCoords.h
 *       的 map_pt_t 给出,取放与绕障点均使用实测目标值。
 *
 * ⚠ 集成约束:进入路径模式时,X(GANTRY_L/R)与 Y(SLIDE_Y)的位置环 ref
 *   必须来自本模块,绕过各自的 axis[i].traj(否则双重滤波破坏联动)。
 *   其余轴(Z/Yaw/Claw)不受影响。集成点见 XYPath_Tick 的使用说明。
 * ============================================================================ */

#define XY_PATH_MAX_WAYPOINTS   8       /* 单条路径最多航点数(够 S 绕障用) */
#define XY_PATH_MAX_SEGMENTS    (2 * XY_PATH_MAX_WAYPOINTS)  /* 直线段+圆弧段上界 */

/* 段类型:直线 / 圆弧(拐角混合) */
typedef enum
{
    XY_SEG_LINE = 0,
    XY_SEG_ARC,
} xy_seg_type_t;

/* 一段几何元(直线或圆弧),沿弧长参数化 */
typedef struct
{
    xy_seg_type_t type;
    float s_start;          /* 本段起始累计弧长 */
    float length;           /* 本段弧长 */

    /* 直线段:起点 + 单位方向 */
    float x0, y0;           /* 起点 */
    float ux, uy;           /* 单位方向(直线)/ 起点切向(圆弧备用) */

    /* 圆弧段:圆心 / 半径 / 起始角 / 旋向 */
    float cx, cy;           /* 圆心 */
    float radius;           /* 半径 */
    float ang0;             /* 起始角(rad) */
    float ang_dir;          /* +1 逆时针 / -1 顺时针 */
} xy_seg_t;

/* 路径插值器状态 */
typedef struct
{
    /* ---- 几何:折线 + 拐角圆弧拼成的段序列 ---- */
    xy_seg_t  seg[XY_PATH_MAX_SEGMENTS];
    uint8_t   seg_num;
    float      total_len;       /* 路径总弧长 */

    /* ---- 弧长上的 S 曲线(复用 TrajTrack) ---- */
    traj_track_t  traj;          /* 规划 s: 0 -> total_len */

    /* ---- 本刻输出(counts 域) ---- */
    float   x_ref, y_ref;        /* 平面位置参考 -> X/Y 位置环 ref */
    float   x_ff,  y_ff;         /* 平面速度前馈 -> X/Y 速度环前馈(counts/s) */
    float   s, s_dot;            /* 本刻路径进度 / 速率,便于监视 */

    uint8_t active;              /* 1 = 路径模式生效(X/Y ref 取本模块) */
} xy_path_t;

/* ============================================================================
 * API
 * ============================================================================ */

/* 初始化:绑定弧长 S 曲线的 V/A/J(counts 量纲)与控制周期。不产生运动。
 *   v/a/j 是【沿路径的合速度】约束,需保证分解到 X、Y 后都不超各轴上限
 *   (见 .c 里的保守取值说明)。 */
void XYPath_Init(xy_path_t *p, float v_max, float a_max, float j_max, float period);

/* 用一组航点构建路径:折线连接,拐角按 blend_radius 做圆弧混合。
 *   pts/num:航点数组(counts 域,map_pt_t)。
 *   blend_radius:拐角圆弧半径(counts);0 = 纯折线(拐角处速度方向不连续)。
 *   构建后路径就绪但未启动,需再调 XYPath_Start;返回 1 表示构建成功。
 *   安全模式不允许自动缩小 blend_radius,空间不足时返回 0。 */
uint8_t XYPath_Build(xy_path_t *p, const map_pt_t *pts, uint8_t num, float blend_radius);

/* 解析检查整条路径到指定圆心的最小距离。
 *   min_distance:允许的最小中心距(counts)。
 *   actual_min_distance:可选输出实际最小中心距,可传 0。
 * 返回 1 表示所有直线/圆弧均满足净空。 */
uint8_t XYPath_ValidateClearance(const xy_path_t *p,
                                 float center_x, float center_y,
                                 float min_distance,
                                 float *actual_min_distance);

/* 启动:从当前几何起点沿 s 规划到 total_len,置 active=1。
 *   start_xy:进入路径模式瞬间的实测平面位置(counts),用于把首航点对齐到
 *             实际位置避免跳变(通常应等于 pts[0])。 */
void XYPath_Start(xy_path_t *p, float start_x, float start_y);

/* 每控制周期调用一次(挂在 2ms 控制节奏里,与 TrajTrack_Calc 同频):
 *   推进弧长 S 曲线 -> 在 s 处插值出 x_ref/y_ref 与速度前馈 x_ff/y_ff。
 *   调用方随后把 x_ref/x_ff 同时给 GANTRY_L/R,y_ref/y_ff 给 SLIDE_Y。 */
void XYPath_Tick(xy_path_t *p);

/* 路径是否已走完(弧长 S 曲线跑完)。非阻塞轮询。 */
uint8_t XYPath_IsDone(const xy_path_t *p);

/* 是否处于路径模式(X/Y ref 应取本模块而非各轴 traj) */
uint8_t XYPath_IsActive(const xy_path_t *p);

/* 退出路径模式(置 active=0,X/Y 交还各轴 traj) */
void XYPath_Stop(xy_path_t *p);

#endif /* __XY_PATH_H */
