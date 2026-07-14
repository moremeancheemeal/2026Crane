#include "XYPath.h"

#include <float.h>
#include <math.h>

/* 数值保护 */
#define XYP_EPS   1e-4f

/* ============================================================================
 * 内部:几何构建
 * ----------------------------------------------------------------------------
 * 输入航点折线 P0-P1-...-Pn-1。每个【内部拐角】Pi(1<=i<=n-2)用一段圆弧把
 * 进入方向与离开方向平滑相切,使切向连续(全程速度方向不突变)。
 *
 * 拐角圆弧构造(标准 "fillet / corner blend"):
 *   设进入单位向量 u_in = (Pi - Pi-1)/|..|,离开单位向量 u_out = (Pi+1 - Pi)/|..|。
 *   半角 theta:cos(2*phi) = u_in . u_out,圆弧需在拐角两侧各退 d = R / tan(phi)。
 *   切点 T_in = Pi - d*u_in,T_out = Pi + d*u_out。
 *   圆心 = 沿角平分线方向从 Pi 退 R/sin(phi)。
 * 退让距离 d 受相邻线段长度限制,自动夹紧避免圆弧吃掉整段直线。
 * ============================================================================ */

static float xyp_hypot(float dx, float dy)
{
    return sqrtf(dx * dx + dy * dy);
}

/* 逐段进给率:按当前段方向把合速度上限放大到 base_vmax/max(|ux|,|uy|),
 * 使占主导的轴吃满 base_vmax。max(|ux|,|uy|)∈[1/√2,1],给下限兜底防除零。 */
static float xyp_seg_vmax(const xy_path_t *p, const xy_seg_t *sg)
{
#if XY_PATH_PER_SEG_FEEDRATE
    float ax = fabsf(sg->ux);
    float ay = fabsf(sg->uy);
    float w  = (ax > ay) ? ax : ay;
    if (w < 0.5f)
    {
        w = 0.5f;
    }
    return p->base_vmax / w;
#else
    (void)sg;
    return p->base_vmax;
#endif
}

/* 追加一条直线段(从 (x0,y0) 到 (x1,y1)),返回新的累计弧长 */
static float xyp_push_line(xy_path_t *p, float s_acc,
                           float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float len = xyp_hypot(dx, dy);

    if (len < XYP_EPS || p->seg_num >= XY_PATH_MAX_SEGMENTS)
    {
        return s_acc;               /* 退化/超容,跳过 */
    }

    xy_seg_t *sg = &p->seg[p->seg_num++];
    sg->type    = XY_SEG_LINE;
    sg->s_start = s_acc;
    sg->length  = len;
    sg->x0 = x0;  sg->y0 = y0;
    sg->ux = dx / len;
    sg->uy = dy / len;

    return s_acc + len;
}

void XYPath_Init(xy_path_t *p, float v_max, float a_max, float j_max, float period)
{
    /* 逐段 S 曲线:把"当前段长度"当作 1 维位置去规划(每段 rest-to-rest)。
     * v/a/j 单位 = counts(沿段)/s, /s^2, /s^3。
     *   v_max = 单轴上限(base_vmax);逐段进给率会按段方向把该段合速度上限
     *   放大到 base_vmax/max(|ux|,|uy|),使主导轴吃满 base_vmax。
     *   a/j 保持为基准(不放大),即主导轴加速度 <= a_max,偏保守以抑制震动。 */
    TrajTrack_Init(&p->traj, v_max, a_max, j_max, period, 0.0f);

    p->base_vmax = v_max;
    p->seg_num   = 0;
    p->seg_idx   = 0;
    p->total_len = 0.0f;
    p->x_ref = p->y_ref = 0.0f;
    p->x_ff  = p->y_ff  = 0.0f;
    p->s = p->s_dot = 0.0f;
    p->dwell_cnt = 0U;
    p->active = 0U;
}

uint8_t XYPath_Build(xy_path_t *p, const map_pt_t *pts, uint8_t num, float blend_radius)
{
    (void)blend_radius;             /* 纯折线 + 逐段全停:不做拐角圆弧混合 */

    p->seg_num   = 0;
    p->seg_idx   = 0;
    p->total_len = 0.0f;
    p->active    = 0U;

    if (pts == 0 || num < 2U)
    {
        return 0U;                  /* 至少两点才成路径 */
    }
    if (num > XY_PATH_MAX_WAYPOINTS)
    {
        return 0U;
    }

    /* 航点直线相连:相邻两点铺一段直线。退化段(重合点)自动跳过。 */
    float s_acc = 0.0f;
    for (uint8_t i = 1; i < num; i++)
    {
        s_acc = xyp_push_line(p, s_acc,
                              (float)pts[i - 1].x, (float)pts[i - 1].y,
                              (float)pts[i].x,     (float)pts[i].y);
    }

    p->total_len = s_acc;

    return (p->seg_num > 0U && p->total_len > XYP_EPS) ? 1U : 0U;
}

static float xyp_line_distance_to_point(const xy_seg_t *sg, float px, float py)
{
    float dx = px - sg->x0;
    float dy = py - sg->y0;
    float along = dx * sg->ux + dy * sg->uy;

    if (along < 0.0f)
    {
        along = 0.0f;
    }
    else if (along > sg->length)
    {
        along = sg->length;
    }

    return xyp_hypot(sg->x0 + sg->ux * along - px,
                     sg->y0 + sg->uy * along - py);
}

uint8_t XYPath_ValidateClearance(const xy_path_t *p,
                                 float center_x, float center_y,
                                 float min_distance,
                                 float *actual_min_distance)
{
    float path_min = FLT_MAX;

    if (p == 0 || p->seg_num == 0U || min_distance < 0.0f)
    {
        if (actual_min_distance != 0)
        {
            *actual_min_distance = 0.0f;
        }
        return 0U;
    }

    for (uint8_t i = 0U; i < p->seg_num; i++)
    {
        const xy_seg_t *sg = &p->seg[i];
        float distance = xyp_line_distance_to_point(sg, center_x, center_y);

        if (distance < path_min)
        {
            path_min = distance;
        }
    }

    if (actual_min_distance != 0)
    {
        *actual_min_distance = path_min;
    }

    return (path_min + XYP_EPS >= min_distance) ? 1U : 0U;
}

/* 把当前段 seg_idx 交给 TrajTrack 规划:0 -> 段长 的 rest-to-rest S 曲线。
 * 段内进给率上限按段方向缩放(主导轴吃满 base_vmax);a/j 用基准不放大。 */
static void XYPath_PlanCurrentSeg(xy_path_t *p)
{
    const xy_seg_t *sg = &p->seg[p->seg_idx];

    p->traj.v_max = xyp_seg_vmax(p, sg);
    TrajTrack_Reset(&p->traj, 0.0f);
    TrajTrack_SetTarget(&p->traj, sg->length);
}

/* 用当前段进度 sp(0..段长)算平面位置与速度前馈,写入本刻输出。 */
static void XYPath_EmitFromSeg(xy_path_t *p, float sp, float sp_dot)
{
    const xy_seg_t *sg = &p->seg[p->seg_idx];

    if (sp < 0.0f) sp = 0.0f;
    if (sp > sg->length) sp = sg->length;

    p->x_ref = sg->x0 + sg->ux * sp;
    p->y_ref = sg->y0 + sg->uy * sp;
    p->x_ff  = sg->ux * sp_dot;
    p->y_ff  = sg->uy * sp_dot;

    p->s     = sg->s_start + sp;         /* 全局进度,仅供监视 */
    p->s_dot = sp_dot;
}

void XYPath_Start(xy_path_t *p, float start_x, float start_y)
{
    (void)start_x;
    (void)start_y;
    /* 从第 0 段起,逐段 rest-to-rest。起点几何即 pts[0]。
     * start_x/y 预留用于将来"实测起点对齐/补偿"。 */
    p->seg_idx   = 0;
    p->dwell_cnt = 0U;
    p->active    = 1U;

    XYPath_PlanCurrentSeg(p);

    /* 预填首刻输出(段起点、零速),避免 active 后首周期 ref 跳变 */
    XYPath_EmitFromSeg(p, 0.0f, 0.0f);
}

void XYPath_Tick(xy_path_t *p)
{
    if (!p->active)
    {
        return;
    }

    /* 航点驻留:段末速度已归零,钉住在【当前段起点】(=刚到达的航点)若干拍,
     * 让龙门余振沉降。注意此处 seg_idx 已指向新段,故停在 sp=0(新段起点),
     * 而非 seg.length(那是下一个航点,会造成位置跳变 -> 顿挫)。 */
    if (p->dwell_cnt > 0U)
    {
        p->dwell_cnt--;
        XYPath_EmitFromSeg(p, 0.0f, 0.0f);
        return;
    }

    /* 推进当前段的 rest-to-rest S 曲线 */
    float sp     = TrajTrack_Calc(&p->traj);
    float sp_dot = p->traj.vel;
    XYPath_EmitFromSeg(p, sp, sp_dot);

    /* 当前段走完(轨迹不忙,已到段末零速):切下一段,期间进入驻留 */
    if (!TrajTrack_IsBusy(&p->traj))
    {
        if (p->seg_idx + 1U < p->seg_num)
        {
            p->seg_idx++;
            XYPath_PlanCurrentSeg(p);
            p->dwell_cnt = XY_PATH_WAYPOINT_DWELL_TICKS;   /* 到达中间航点,停一下 */
            XYPath_EmitFromSeg(p, 0.0f, 0.0f);
        }
        /* 末段走完则保持在终点;XYPath_IsDone 会报完成 */
    }
}

uint8_t XYPath_IsDone(const xy_path_t *p)
{
    /* 最后一段跑完(traj 不忙)且不在驻留中,即整条路径走完 */
    return (p->active &&
            (p->seg_idx + 1U >= p->seg_num) &&
            p->dwell_cnt == 0U &&
            !TrajTrack_IsBusy(&p->traj)) ? 1U : 0U;
}

uint8_t XYPath_IsActive(const xy_path_t *p)
{
    return p->active;
}

void XYPath_Stop(xy_path_t *p)
{
    p->active = 0U;
}
