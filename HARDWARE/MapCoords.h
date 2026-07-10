#ifndef __MAP_COORDS_H
#define __MAP_COORDS_H

#include <stdint.h>

/* ============================================================================
 * 场地坐标体系 —— 所有运行目标统一使用 dir 校正后的编码器 counts
 * ----------------------------------------------------------------------------
 *   X : 龙门行走方向。+X 朝放置区,-X 朝取货区。
 *   Y : 小车滑行方向。+Y 朝 ①/④,-Y 朝 ②/⑧,③/⑥ 接近中线。
 *   Z : 升降方向。越负越高。
 *
 * 龙门 X 来自两侧电机实测值的统一目标:
 *     X = (-Motor[0].total_angle + Motor[1].total_angle) / 2
 * 奇数和按 C 整数除法向 0 截断。XYPath / 轴目标 / 到位判断均使用 counts。
 * ============================================================================ */

/* XYPath 的平面航点(counts 域)。 */
typedef struct
{
    int32_t x;
    int32_t y;
} map_pt_t;

/* 一个取/放姿态的最终目标。Yaw=0 表示保持正向。 */
typedef struct
{
    int32_t x_counts;
    int32_t y_counts;
    int32_t z_counts;
    int32_t yaw_counts;
} box_t;

typedef struct
{
    box_t pick;
    box_t place;
} mission_t;

/* ---- 旧四点位测试标定值,仅供手动测试和历史对照 ---- */
#define MAP_XCAL_P0    (-1636607)
#define MAP_XCAL_P1    (-2019861)
#define MAP_XCAL_P2    ( 1652387)
#define MAP_XCAL_P3    ( 1860000)

/* ---- 升降高度(counts,越负越高) ---- */
#define Z_TOP_TRAVEL    (-317467)   /* 最高运输/扫描位置 */
#define Z_LAYER_1       (-58970)    /* ② 第三高:下 1 盒纸箱 */
#define Z_LAYER_2       (-99655)    /* ① 第二高:下 2 盒纸箱 */
#define Z_LAYER_3       (-142959)   /* ③ 第一高:下 3 盒纸箱 */
#define Z_PLACE         (-36243)    /* 旧手动流程通用放货高度 */
#define Z_BOTTOM        (0)

#define POSE_YAW_ZERO   (0)

/* ---- 取货区实测点位:扫描使用相同 XY + Z_TOP_TRAVEL,抓取时下降到各自 Z ---- */
#define BOX_PICK_1   { -2176922,  511698, Z_LAYER_2, POSE_YAW_ZERO }  /* ① 白芸豆,第二高 */
#define BOX_PICK_2   { -2176920, -524778, Z_LAYER_1, POSE_YAW_ZERO }  /* ② 绿豆,第三高   */
#define BOX_PICK_3   { -1893572,    5851, Z_LAYER_3, POSE_YAW_ZERO }  /* ③ 黄豆,第一高   */

/* ---- 放置区实测点位 ---- */
#define BOX_PLACE_4  { 1509918,  902867, -77957,  96336 }
#define BOX_PLACE_5  { 1753683,  427058, -68465,  POSE_YAW_ZERO }
#define BOX_PLACE_6  { 1753682,    9348, -62618,  POSE_YAW_ZERO }
#define BOX_PLACE_7  { 1753684, -420153, -45950,  POSE_YAW_ZERO }
#define BOX_PLACE_8  { 1509917, -916231, -83679, 100915 }

/* 规则图 S 形路径:
 *   豆侧左柱走负 Y,奶箱侧右柱走正 Y。
 * 两张柱边截图只采用龙门左右电机得到 X;Y 分别复用有效的⑦/⑤点位。 */
#define DETOUR_BEAN_X_COUNTS    (-1251279)
#define DETOUR_BEAN_Y_COUNTS    (-420153)
#define DETOUR_BOX_X_COUNTS     (828350)
#define DETOUR_BOX_Y_COUNTS     (427058)
#define DETOUR_BEAN_PT          { DETOUR_BEAN_X_COUNTS, DETOUR_BEAN_Y_COUNTS }
#define DETOUR_BOX_PT           { DETOUR_BOX_X_COUNTS,  DETOUR_BOX_Y_COUNTS  }

/* 150mm × X/Y 实测平均比例 ≈ 158394 counts。 */
#define PATH_CORNER_BLEND_COUNTS  158394.0f

/* 用户可编辑的默认任务表。视觉模式下实际 place 由 MATCH 数据覆盖。 */
#define MISSION_TABLE  { \
    { BOX_PICK_3, BOX_PLACE_6 },   /* 黄豆   -> ⑥ */ \
    { BOX_PICK_1, BOX_PLACE_5 },   /* 白芸豆 -> ⑤ */ \
    { BOX_PICK_2, BOX_PLACE_7 },   /* 绿豆   -> ⑦ */ \
}

/* 到位容差(counts)。 */
#define TOL_X       15000
#define TOL_Y       15000
#define TOL_Z       10000
#define TOL_CLAW    40000

#endif /* __MAP_COORDS_H */
