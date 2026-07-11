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
#define Z_TOP_TRAVEL    (-300000)   /* 最高运输/扫描位置 */
#define Z_LAYER_1       (-100000)   /* ① 点位探测/抓取高度 */
#define Z_LAYER_2       (-50000)   /* ② 点位探测/抓取高度 */
#define Z_LAYER_3       (-150000)   /* ③ 点位探测/抓取高度 */
#define Z_PLACE         (-75000)    /* ④～⑧统一放货高度 */
#define Z_BOTTOM        (0)

#define POSE_YAW_ZERO   (0)

/* ---- 取货区实测点位:扫描使用相同 XY + Z_TOP_TRAVEL,抓取时下降到各自 Z ---- */
#define PICK_1_Y_COUNTS    ( 528305)
#define PICK_2_Y_COUNTS    (-524778)
#define PICK_3_Y_COUNTS    (   12843)
#define BOX_PICK_1   { -2160287, PICK_1_Y_COUNTS, Z_LAYER_1, POSE_YAW_ZERO }  /* ① 白芸豆 */
#define BOX_PICK_2   { -2160287, PICK_2_Y_COUNTS, Z_LAYER_2, POSE_YAW_ZERO }  /* ② 绿豆   */
#define BOX_PICK_3   { -1893572, PICK_3_Y_COUNTS, Z_LAYER_3, POSE_YAW_ZERO }  /* ③ 黄豆   */

/* ---- 放置区实测点位 ---- */
#define PLACE_4_Y_COUNTS   ( 902867)
#define PLACE_5_Y_COUNTS   ( 427058)
#define PLACE_6_Y_COUNTS   (   9348)
#define PLACE_7_Y_COUNTS   (-420153)
#define PLACE_8_Y_COUNTS   (-916231)
#define BOX_PLACE_4  { 1519525, PLACE_4_Y_COUNTS, Z_PLACE,  96336 }
#define BOX_PLACE_5  { 1745095, PLACE_5_Y_COUNTS, Z_PLACE,  POSE_YAW_ZERO }
#define BOX_PLACE_6  { 1745095, PLACE_6_Y_COUNTS, Z_PLACE,  POSE_YAW_ZERO }
#define BOX_PLACE_7  { 1745095, PLACE_7_Y_COUNTS, Z_PLACE,  POSE_YAW_ZERO }
#define BOX_PLACE_8  { 1519525, PLACE_8_Y_COUNTS, Z_PLACE, 100915 }

/* 规则图 S 形路径:
 *   左柱走负 Y,右柱走正 Y。两个 X 值均为圆柱中心,Y 均为场地中线 0。
 *   150mm × X/Y 实测平均比例 ≈ 158394 counts,即约 1055.96 counts/mm。 */
#define MAP_COUNTS_PER_MM               1055.96f

#define OBSTACLE_BEAN_X_COUNTS          (-1251279)
#define OBSTACLE_BEAN_Y_COUNTS          (0)
#define OBSTACLE_BOX_X_COUNTS           (828350)
#define OBSTACLE_BOX_Y_COUNTS           (0)

/* 夹具/货物安全包络:
 * 51mm 圆柱半径 + 200mm 货物半径 + 50mm 静态余量 + 50mm 制动余量 = 351mm。 */
#define OBSTACLE_RADIUS_COUNTS           53854.0f
#define PAYLOAD_RADIUS_COUNTS           211192.0f
#define OBSTACLE_STATIC_MARGIN_COUNTS    52798.0f
#define OBSTACLE_BRAKE_MARGIN_COUNTS     52798.0f
#define OBSTACLE_CLEARANCE_COUNTS        370642.0f

/* 每根圆柱使用前后两个门点。门点 X 距圆心 300mm,Y 沿用已验证的⑦/⑤位置。 */
#define OBSTACLE_GATE_HALF_X_COUNTS      316788
#define DETOUR_BEAN_Y_COUNTS             (-420153)
#define DETOUR_BOX_Y_COUNTS              (427058)

/* 150mm 拐角圆弧。 */
#define PATH_CORNER_BLEND_COUNTS         158394.0f

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
