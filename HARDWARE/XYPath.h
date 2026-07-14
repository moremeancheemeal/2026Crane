#ifndef __XY_PATH_H
#define __XY_PATH_H

#include "stm32f4xx_hal.h"
#include "TrajTrack.h"
#include "MapCoords.h"

/* ============================================================================
 * XY 联动路径插值器(纯折线 + 逐段 rest-to-rest 全停)
 * ----------------------------------------------------------------------------
 * 目的:比赛不能越障,X(龙门)与 Y(小车)必须在水平面内沿一条【确定的几何
 *       路径】协同运动,而不是各走各的 S 曲线导致末端轨迹形状不可控。
 *
 * ⚠ 大惯量龙门实测:只要过桩换向时结构还在动,整机就晃。拐角圆弧磨光滑
 *   (切向连续)不足以消震 —— 必须在每个航点真正停到零速。故本模块采用:
 *
 * 架构:纯折线 + 逐段独立 S 曲线,航点处 rest-to-rest 全停
 *   1) 几何路径 = 航点直线相连(无圆弧)。第 i 段方向固定为单位向量 (ux,uy)。
 *   2) 逐段速度规划:对【当前段】复用 TrajTrack(7 段 jerk 受限 S 曲线),
 *      把该段长度当 1 维位置做 0 -> length 的 rest-to-rest 规划。段末 v=0、
 *      a=0;下一段又从 v=0、a=0 起。段内方向恒定 => X/Y 只是被同一标量曲线
 *      按 (ux,uy) 固定分解,全程无速度/加速度不连续 => 不顿。
 *   3) 每控制周期 XYPath_Tick():推进当前段的标量 S 曲线;走完则切到下一段
 *      重新规划(自然经过零速)。位置/前馈按当前段方向分解:
 *        sp    = traj.out                         (当前段内进度 0..length)
 *        x_ref = seg.x0 + seg.ux*sp,  y_ref 同理  -> 喂 X/Y 位置环
 *        x_ff  = seg.ux*traj.vel,     y_ff 同理    -> 喂 X/Y 速度前馈
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

/* 逐段进给率缩放开关:
 *   1 = 每段按方向把合速度上限放大到 base_vmax/max(|ux|,|uy|),使【占主导的轴】
 *       在任意段方向都吃满单轴上限(base_vmax)。纯 X/纯 Y 段吃满 100%,斜段
 *       主导轴仍 100%、另一轴按比例更低。因逐段停车,段内方向恒定,放大不引入
 *       任何不连续;任一轴分量恒 <= base_vmax,安全。
 *   0 = 固定合速度上限 base_vmax(斜段主导轴吃不满,只有轴向段吃满)。
 * 上板对比:先用 1(X/Y 都能顶到 350),想更保守再试 0。 */
#define XY_PATH_PER_SEG_FEEDRATE      1

/* 航点驻留:到达每个中间航点、速度归零后,额外保持 N 个控制周期(位置钉住、
 * 前馈为 0)再规划下一段,给大惯量龙门时间把余振沉降掉。0 = 不驻留(仍会
 * 因 rest-to-rest 在航点瞬时经过零速)。控制周期 2ms,故 25 拍 ≈ 50ms。
 * 每趟只在少数中间航点各停一次,总耗时增加有限。 */
#define XY_PATH_WAYPOINT_DWELL_TICKS  25U

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
    /* ---- 几何:纯折线段序列(无圆弧) ---- */
    xy_seg_t  seg[XY_PATH_MAX_SEGMENTS];
    uint8_t   seg_num;
    float      total_len;       /* 路径总弧长(所有段长之和,仅供监视) */

    /* ---- 逐段 S 曲线(复用 TrajTrack) ----
     * 每段独立规划 0 -> seg.length 的 rest-to-rest 曲线;走完切下一段。 */
    traj_track_t  traj;          /* 规划【当前段】段内进度: 0 -> seg[seg_idx].length */
    uint8_t   seg_idx;           /* 当前正在走的段下标 */

    /* ---- 速度约束基准(counts 域,单轴上限)。逐段进给率缩放用。 ---- */
    float   base_vmax;           /* 沿段合速度基准上限 = 单轴 vmax(counts/s) */

    /* ---- 航点驻留计数:段末速度归零后钉住位置若干拍再进下一段 ---- */
    uint16_t dwell_cnt;          /* 剩余驻留拍数;0 = 非驻留态 */

    /* ---- 本刻输出(counts 域) ---- */
    float   x_ref, y_ref;        /* 平面位置参考 -> X/Y 位置环 ref */
    float   x_ff,  y_ff;         /* 平面速度前馈 -> X/Y 速度环前馈(counts/s) */
    float   s, s_dot;            /* 本刻全局进度 / 沿段速率,便于监视 */

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
