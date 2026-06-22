#include "XYPath.h"

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

/* 追加一段圆弧(圆心 c,半径 R,起始角 a0,旋向 adir,圆心角 dang>=0) */
static float xyp_push_arc(xy_path_t *p, float s_acc,
                          float cx, float cy, float R,
                          float a0, float adir, float dang)
{
    float len = R * dang;

    if (len < XYP_EPS || p->seg_num >= XY_PATH_MAX_SEGMENTS)
    {
        return s_acc;
    }

    xy_seg_t *sg = &p->seg[p->seg_num++];
    sg->type    = XY_SEG_ARC;
    sg->s_start = s_acc;
    sg->length  = len;
    sg->cx = cx;  sg->cy = cy;
    sg->radius  = R;
    sg->ang0    = a0;
    sg->ang_dir = (adir >= 0.0f) ? 1.0f : -1.0f;

    return s_acc + len;
}

void XYPath_Init(xy_path_t *p, float v_max, float a_max, float j_max, float period)
{
    /* 弧长 S 曲线:把"沿路径合速度"当作 1 维位置去规划。
     * v/a/j 单位 = counts(沿路径)/s, /s^2, /s^3。
     * ⚠ 取值需保守:合速度会按切向分解到 X、Y,任一轴分量不得超该轴上限。
     *   简单保守做法:v_max <= min(各轴 vmax 的 counts/s)。联调时再调。 */
    TrajTrack_Init(&p->traj, v_max, a_max, j_max, period, 0.0f);

    p->seg_num   = 0;
    p->total_len = 0.0f;
    p->x_ref = p->y_ref = 0.0f;
    p->x_ff  = p->y_ff  = 0.0f;
    p->s = p->s_dot = 0.0f;
    p->active = 0U;
}

void XYPath_Build(xy_path_t *p, const map_pt_t *pts, uint8_t num, float blend_radius)
{
    p->seg_num   = 0;
    p->total_len = 0.0f;

    if (pts == 0 || num < 2U)
    {
        return;                     /* 至少两点才成路径 */
    }
    if (num > XY_PATH_MAX_WAYPOINTS)
    {
        num = XY_PATH_MAX_WAYPOINTS;
    }

    float s_acc = 0.0f;

    /* cur = 当前已铺到的"线段实际起点"。首段从 pts[0] 出发;遇到拐角时,
     * 直线只铺到入切点 T_in,然后铺圆弧到出切点 T_out,cur 更新为 T_out。 */
    float cur_x = (float)pts[0].x;
    float cur_y = (float)pts[0].y;

    for (uint8_t i = 1; i + 1 < num; i++)
    {
        float px = (float)pts[i].x,     py = (float)pts[i].y;       /* 拐角顶点 */
        float ax = (float)pts[i - 1].x, ay = (float)pts[i - 1].y;   /* 前一点 */
        float bx = (float)pts[i + 1].x, by = (float)pts[i + 1].y;   /* 后一点 */

        /* 入/出单位向量 */
        float inx = px - ax, iny = py - ay;
        float outx = bx - px, outy = by - py;
        float in_len  = xyp_hypot(inx, iny);
        float out_len = xyp_hypot(outx, outy);

        if (blend_radius < XYP_EPS || in_len < XYP_EPS || out_len < XYP_EPS)
        {
            /* 不混合:直接铺到拐角顶点(纯折线,拐角处速度方向突变) */
            s_acc = xyp_push_line(p, s_acc, cur_x, cur_y, px, py);
            cur_x = px;  cur_y = py;
            continue;
        }

        float uinx = inx / in_len,  uiny = iny / in_len;
        float uoutx = outx / out_len, uouty = outy / out_len;

        /* 拐角夹角:cos(2*phi) = u_in . u_out,phi = 半张角的余角处理 */
        float dot = uinx * uoutx + uiny * uouty;
        if (dot >  1.0f) dot =  1.0f;
        if (dot < -1.0f) dot = -1.0f;

        /* 转折角 turn = 进入与离开方向夹角;phi = (pi - turn)/2 用于退让公式。
         * 用 tan(phi) = sin/.. 的稳定写法:half = turn/2,d = R / tan((pi-turn)/2)
         *                                              = R * tan(turn/2)。 */
        float turn = acosf(dot);                 /* 0..pi */
        if (turn < XYP_EPS)
        {
            /* 几乎共线,无需圆弧 */
            s_acc = xyp_push_line(p, s_acc, cur_x, cur_y, px, py);
            cur_x = px;  cur_y = py;
            continue;
        }

        float R = blend_radius;
        float d = R * tanf(turn * 0.5f);         /* 拐角两侧退让距离 */

        /* 退让受相邻段长限制(从 cur 到 P 的剩余长 与 P 到 B 的长) */
        float avail_in  = xyp_hypot(px - cur_x, py - cur_y);
        float avail_out = out_len;
        float dmax = (avail_in < avail_out ? avail_in : avail_out) * 0.5f;
        if (d > dmax)
        {
            d = dmax;
            R = d / tanf(turn * 0.5f);           /* 半径随退让一起缩 */
        }

        /* 切点 */
        float tin_x = px - d * uinx,  tin_y = py - d * uiny;     /* 入切点 */
        float tout_x = px + d * uoutx, tout_y = py + d * uouty;  /* 出切点 */

        /* 1) 直线:cur -> 入切点 */
        s_acc = xyp_push_line(p, s_acc, cur_x, cur_y, tin_x, tin_y);

        /* 2) 圆弧:入切点 -> 出切点
         * 圆心在角平分线上,距切点 R(垂直于切向)。用入切点法向求圆心:
         * 旋向由叉积 (u_in x u_out) 决定。 */
        float cross = uinx * uouty - uiny * uoutx;   /* >0 左转(逆时针) */
        float ndir  = (cross >= 0.0f) ? 1.0f : -1.0f;

        /* 入切点处指向圆心的单位法向 = 旋转 u_in 90 度(按旋向) */
        float nx = -uiny * ndir;
        float ny =  uinx * ndir;
        float cx = tin_x + R * nx;
        float cy = tin_y + R * ny;

        /* 起始角 = 圆心->入切点 的方位角;圆心角 = turn(切向转过的角度) */
        float a0 = atan2f(tin_y - cy, tin_x - cx);
        s_acc = xyp_push_arc(p, s_acc, cx, cy, R, a0, ndir, turn);

        /* cur 更新到出切点,下一段直线从这里继续 */
        cur_x = tout_x;  cur_y = tout_y;
    }

    /* 末段:cur -> 最后一个航点 */
    s_acc = xyp_push_line(p, s_acc, cur_x, cur_y,
                          (float)pts[num - 1].x, (float)pts[num - 1].y);

    p->total_len = s_acc;
}

/* 在弧长 s 处求位置 (x,y) 与单位切向 (tx,ty) */
static void XYPath_Eval(const xy_path_t *p, float s,
                        float *x, float *y, float *tx, float *ty)
{
    /* 夹紧到 [0, total_len] */
    if (s < 0.0f) s = 0.0f;
    if (s > p->total_len) s = p->total_len;

    /* 找 s 所在段:最大的 i 满足 seg[i].s_start <= s */
    uint8_t idx = 0;
    for (uint8_t i = 0; i < p->seg_num; i++)
    {
        if (p->seg[i].s_start <= s)
        {
            idx = i;
        }
    }

    const xy_seg_t *sg = &p->seg[idx];
    float ds = s - sg->s_start;
    if (ds < 0.0f) ds = 0.0f;
    if (ds > sg->length) ds = sg->length;

    if (sg->type == XY_SEG_LINE)
    {
        *x = sg->x0 + sg->ux * ds;
        *y = sg->y0 + sg->uy * ds;
        *tx = sg->ux;
        *ty = sg->uy;
    }
    else    /* XY_SEG_ARC */
    {
        float ang = sg->ang0 + sg->ang_dir * (ds / sg->radius);
        *x = sg->cx + sg->radius * cosf(ang);
        *y = sg->cy + sg->radius * sinf(ang);
        /* 切向 = d/ds 位置:对圆弧 = 旋转半径方向 90 度,符号随旋向 */
        *tx = -sinf(ang) * sg->ang_dir;
        *ty =  cosf(ang) * sg->ang_dir;
    }
}

void XYPath_Start(xy_path_t *p, float start_x, float start_y)
{
    (void)start_x;
    (void)start_y;
    /* 弧长从 0 起,规划到 total_len。起点位置由路径首段几何决定;
     * start_x/y 预留用于将来"实测起点对齐/补偿",当前几何起点即 pts[0]。 */
    TrajTrack_Reset(&p->traj, 0.0f);
    TrajTrack_SetTarget(&p->traj, p->total_len);

    p->s = 0.0f;
    p->s_dot = 0.0f;
    p->active = 1U;

    /* 预填首刻输出,避免 active 后首周期 ref 为 0 跳变 */
    float tx, ty;
    XYPath_Eval(p, 0.0f, &p->x_ref, &p->y_ref, &tx, &ty);
    p->x_ff = 0.0f;
    p->y_ff = 0.0f;
}

void XYPath_Tick(xy_path_t *p)
{
    if (!p->active)
    {
        return;
    }

    /* 推进弧长 S 曲线 */
    p->s     = TrajTrack_Calc(&p->traj);
    p->s_dot = p->traj.vel;                 /* 沿路径速率 counts/s */

    float tx, ty;
    XYPath_Eval(p, p->s, &p->x_ref, &p->y_ref, &tx, &ty);

    /* 速度前馈 = 单位切向 * 沿路径速率,分解到 X/Y */
    p->x_ff = tx * p->s_dot;
    p->y_ff = ty * p->s_dot;
}

uint8_t XYPath_IsDone(const xy_path_t *p)
{
    /* 弧长 S 曲线跑完(traj 不忙)即几何路径走完 */
    return (p->active && !TrajTrack_IsBusy(&p->traj)) ? 1U : 0U;
}

uint8_t XYPath_IsActive(const xy_path_t *p)
{
    return p->active;
}

void XYPath_Stop(xy_path_t *p)
{
    p->active = 0U;
}
