#ifndef __MAP_COORDS_H
#define __MAP_COORDS_H

#include <stdint.h>

/* ============================================================================
 * 场地坐标体系 —— 龙门起重机取放点 / 障碍 / 绕障航点
 * ----------------------------------------------------------------------------
 * 坐标系约定(原点 = 场地中心 = 起始区 400x400 的中心,距左墙 2000mm):
 *   X : 龙门行走方向(长边 4000mm)。+X 朝放置区(右),-X 朝取货区(左)。
 *       与代码符号一致 —— X<0 取货侧,X>0 放置侧,可直接喂 Axis_SetTargetXGantry。
 *   Y : 小车滑行方向(短边 2000mm)。+Y 朝 ①(图上方),-Y 朝 ②(图下方),
 *       ③/⑥ 在中线 Y=0。
 *   Z : 升降高度,见 MotorControlTask.c 的 z_height_points(越负越高)。
 *
 * ⚠ 标定状态(2026/06/23):
 *   - X 换算已由四个标定点反推得到(见下方 X_COUNTS_PER_MM / MAP_X_*)。
 *   - Y 换算【尚未标定】,所有 Y 暂以 mm 估值给出,counts 待补(MAP_Y_TODO)。
 *   - 障碍中心/半径、箱半宽为估值,需实测后回填。
 * ============================================================================ */

/* ---------------------------------------------------------------------------
 * X 轴 counts <-> mm 换算(已标定)
 *   由 P1=-2019861(X=-1855mm) 与 P3=+1860000(X=+1830mm) 线性拟合:
 *     k = 3879861 / 3685 ≈ 1052.9 counts/mm
 *     counts = k * X_mm + b ,  b ≈ -66770
 *   物理校验:157286 counts/输出圈 / k ≈ 149mm/圈(⌀47.5mm 带轮),合理。
 * --------------------------------------------------------------------------- */
#define X_COUNTS_PER_MM        1052.9f
#define X_COUNTS_OFFSET        (-66770.0f)
#define X_MM_TO_COUNTS(x_mm)   ((int32_t)(X_COUNTS_PER_MM * (float)(x_mm) + X_COUNTS_OFFSET))
#define X_COUNTS_TO_MM(c)      (((float)(c) - X_COUNTS_OFFSET) / X_COUNTS_PER_MM)

/* ---------------------------------------------------------------------------
 * Y 轴 counts <-> mm 换算【待标定】
 *   标两点(如 ① 顶位 与 ② 底位)后回填 k_Y / b_Y,再启用下面的宏。
 * --------------------------------------------------------------------------- */
#define Y_COUNTS_PER_MM        1.0f          /* TODO: 实测回填 */
#define Y_COUNTS_OFFSET        0.0f          /* TODO: 实测回填 */
#define Y_MM_TO_COUNTS(y_mm)   ((int32_t)(Y_COUNTS_PER_MM * (float)(y_mm) + Y_COUNTS_OFFSET))
#define Y_COUNTS_TO_MM(c)      (((float)(c) - Y_COUNTS_OFFSET) / Y_COUNTS_PER_MM)
#define MAP_Y_TODO             0             /* Y 未标定时的占位 counts */

/* ---------------------------------------------------------------------------
 * 一个平面点(counts 域,直接可用作 X/Y 轴目标)
 * --------------------------------------------------------------------------- */
typedef struct
{
    int32_t x;      /* 龙门 X 目标 counts */
    int32_t y;      /* 小车 Y 目标 counts(未标定前填 MAP_Y_TODO) */
} map_pt_t;

/* ---------------------------------------------------------------------------
 * 四个 X 标定点(实测原值,来自 MotorControlTask.h 的 X_CAL_POINT*)
 *   反推物理位置见下表注释,供溯源/校验用。
 * --------------------------------------------------------------------------- */
#define MAP_XCAL_P0    (-1636607)   /* X≈-1491mm,取货 ③ 列(中线右移≈364) */
#define MAP_XCAL_P1    (-2019861)   /* X≈-1855mm,取货 ①② 列(最左) */
#define MAP_XCAL_P2    ( 1652387)   /* X≈+1633mm,放置 ⑤⑥⑦⑧ 列 */
#define MAP_XCAL_P3    ( 1860000)   /* X≈+1830mm,放置 ④ 列(稍靠中心) */

/* ===========================================================================
 * 取货区箱位(X 为标定反推 counts,已可用;Y 为 mm 估值,counts 待标定)
 *   ① 白芸豆,下 2 纸箱;② 绿豆,下 1 纸箱;③ 黄豆,下 3 纸箱。
 * =========================================================================== */
/*                         X(counts)      Y(counts, TODO)   [X_mm , Y_mm估] */
#define PICK_1   ((map_pt_t){ MAP_XCAL_P1, MAP_Y_TODO })   /* -1855 , +795 */
#define PICK_2   ((map_pt_t){ MAP_XCAL_P1, MAP_Y_TODO })   /* -1855 , -795 */
#define PICK_3   ((map_pt_t){ MAP_XCAL_P0, MAP_Y_TODO })   /* -1491 ,    0 */

/* 取货 Y 估值(mm),Y 标定后改用 Y_MM_TO_COUNTS 填入上面 */
#define PICK_1_Y_MM     ( 795)
#define PICK_2_Y_MM     (-795)
#define PICK_3_Y_MM     (   0)

/* ===========================================================================
 * 放置区箱位(放置侧 Y 间距按图标 400mm;④ 单独一列,⑤⑥⑦⑧ 同列)
 * =========================================================================== */
/*                         X(counts)      Y(counts, TODO)   [X_mm , Y_mm估] */
#define PLACE_4  ((map_pt_t){ MAP_XCAL_P3, MAP_Y_TODO })   /* +1830 , +800 */
#define PLACE_5  ((map_pt_t){ MAP_XCAL_P2, MAP_Y_TODO })   /* +1633 , +400 */
#define PLACE_6  ((map_pt_t){ MAP_XCAL_P2, MAP_Y_TODO })   /* +1633 ,    0 */
#define PLACE_7  ((map_pt_t){ MAP_XCAL_P2, MAP_Y_TODO })   /* +1633 , -400 */
#define PLACE_8  ((map_pt_t){ MAP_XCAL_P2, MAP_Y_TODO })   /* +1633 , -800 (空货箱) */

#define PLACE_4_Y_MM    ( 800)
#define PLACE_5_Y_MM    ( 400)
#define PLACE_6_Y_MM    (   0)
#define PLACE_7_Y_MM    (-400)
#define PLACE_8_Y_MM    (-800)

/* ===========================================================================
 * 障碍 / 起始区(mm,均为估值,需实测回填)
 *   两障碍压在中线 Y=0,挡住 ③->⑥ 直线,故绕障必须在 Y 上偏。
 * =========================================================================== */
#define START_ZONE_X_MM      0
#define START_ZONE_Y_MM      0
#define START_ZONE_HALF_MM   200        /* 400x400 标识,非障碍 */

#define OBST_L_X_MM        (-700)       /* TODO 左障碍中心 X */
#define OBST_L_Y_MM        (   0)       /* TODO 左障碍中心 Y */
#define OBST_R_X_MM        ( 700)       /* TODO 右障碍中心 X */
#define OBST_R_Y_MM        (   0)       /* TODO 右障碍中心 Y */
#define OBST_RADIUS_MM     ( 200)       /* TODO 障碍半径(含安全余量) */

/* ===========================================================================
 * 绕障 S 航点(③ -> ⑥ 示例;折线 + 拐角圆滑过渡)
 *   方向可镜像(绕左障下侧 / 绕右障上侧)。Y 标定后把估值换成 counts。
 *   ③(-1491,0) -> (-700,-400) -> (0,0) -> (+700,+400) -> ⑥(+1633,0)
 * =========================================================================== */
#define SPATH_NUM    5
/* 仅 mm 估值,供路径规划占位;counts 版待 Y 标定后生成 */
#define SPATH_X_MM   { -1491, -700,    0,  700, 1633 }
#define SPATH_Y_MM   {     0, -400,    0,  400,    0 }

/* 拐角圆滑过渡半径(mm),折线拐角处用圆弧/Bezier 混合,全程速度方向连续 */
#define SPATH_CORNER_BLEND_MM   150     /* TODO 按实际间隙调 */

#endif /* __MAP_COORDS_H */
