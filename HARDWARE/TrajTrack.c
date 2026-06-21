#include "TrajTrack.h"

#include <math.h>

/* 约束下限保护,避免除零 */
#define TRAJ_EPS   1e-6f

/* ------------------------------------------------------------------
 * 内部:根据约束与位移 h(>=0),规划一条 rest-to-rest 的七段式
 * jerk 受限 S 曲线,结果写入 tr 的 seg_* / total_time。
 * 位移域恒为正,方向由 tr->dir 处理。
 * ------------------------------------------------------------------ */
static void TrajTrack_BuildProfile(traj_track_t *tr, float h)
{
    float v = tr->v_max;
    float a = tr->a_max;
    float j = tr->j_max;

    float Tj;       /* 单个 jerk 段时长 */
    float Ta;       /* 加速段总时长(= 减速段 Td,对称) */
    float Tv;       /* 匀速段时长 */
    float vlim;     /* 实际达到的峰值速度 */

    float dur[7];
    float a_peak;
    uint8_t i;

    /* 退化:几乎不动 */
    if (h < TRAJ_EPS)
    {
        for (i = 0; i < 7; i++)
        {
            tr->seg_j[i]  = 0.0f;
            tr->seg_t0[i] = 0.0f;
            tr->seg_p0[i] = 0.0f;
            tr->seg_v0[i] = 0.0f;
            tr->seg_a0[i] = 0.0f;
        }
        tr->total_time = 0.0f;
        return;
    }

    /* 1) 先假设跟踪到 v_max,判断加速段是否能达到 a_max */
    if (v * j >= a * a)         /* a_max 可达 */
    {
        Tj = a / j;
        Ta = Tj + v / a;
    }
    else                        /* a_max 不可达 */
    {
        Tj = sqrtf(v / j);
        Ta = 2.0f * Tj;
    }

    /* 加速到 v_max 再减速回 0 所需的位移(对称,无匀速) */
    float dist_no_cruise = v * Ta;

    if (dist_no_cruise <= h)
    {
        /* 2a) v_max 可达,存在匀速段 */
        vlim = v;
        Tv   = (h - dist_no_cruise) / v;
    }
    else
    {
        /* 2b) 峰值速度达不到 v_max,无匀速段,重解 vlim */
        Tv = 0.0f;

        /* 判断峰值时 a_max 是否可达:临界位移 h_th = 2*a^3/j^2 */
        float h_th = 2.0f * a * a * a / (j * j);

        if (h >= h_th)          /* a_max 可达,解二次方程 (1/a)v^2 + (a/j)v - h = 0 */
        {
            float A = 1.0f / a;
            float B = a / j;
            float C = -h;
            vlim = (-B + sqrtf(B * B - 4.0f * A * C)) / (2.0f * A);
            Tj   = a / j;
            Ta   = Tj + vlim / a;
        }
        else                    /* a_max 不可达:vlim = (h^2 * j / 4)^(1/3) */
        {
            vlim = powf(h * h * j * 0.25f, 1.0f / 3.0f);
            Tj   = sqrtf(vlim / j);
            Ta   = 2.0f * Tj;
        }
    }

    a_peak = j * Tj;            /* 峰值加速度(a_max 可达时即 a_max) */

    /* 3) 组装七段:jerk 与各段时长 */
    dur[0] = Tj;            tr->seg_j[0] =  j;     /* 加加速 */
    dur[1] = Ta - 2.0f*Tj; tr->seg_j[1] =  0.0f;  /* 匀加速 */
    dur[2] = Tj;           tr->seg_j[2] = -j;     /* 减加速 */
    dur[3] = Tv;           tr->seg_j[3] =  0.0f;  /* 匀速   */
    dur[4] = Tj;           tr->seg_j[4] = -j;     /* 加减速 */
    dur[5] = Ta - 2.0f*Tj; tr->seg_j[5] =  0.0f;  /* 匀减速 */
    dur[6] = Tj;           tr->seg_j[6] =  j;     /* 减减速 */

    (void)a_peak;

    /* 4) 顺序积分,填充每段起始的 t0/p0/v0/a0 */
    float t0 = 0.0f, p0 = 0.0f, v0 = 0.0f, a0 = 0.0f;
    for (i = 0; i < 7; i++)
    {
        float d  = dur[i];
        float jj = tr->seg_j[i];

        if (d < 0.0f) d = 0.0f;     /* 数值保护 */

        tr->seg_t0[i] = t0;
        tr->seg_p0[i] = p0;
        tr->seg_v0[i] = v0;
        tr->seg_a0[i] = a0;

        /* 段末状态作为下一段起始 */
        p0 += v0 * d + 0.5f * a0 * d * d + (1.0f / 6.0f) * jj * d * d * d;
        v0 += a0 * d + 0.5f * jj * d * d;
        a0 += jj * d;
        t0 += d;
    }

    tr->total_time = t0;
}

/* 在已规划曲线上求 t 时刻的位移/速度/加速度(位移域,>=0) */
static void TrajTrack_Eval(const traj_track_t *tr, float t,
                           float *p, float *v, float *a)
{
    int8_t idx = 0;
    int8_t i;

    if (t <= 0.0f)
    {
        *p = tr->seg_p0[0];
        *v = tr->seg_v0[0];
        *a = tr->seg_a0[0];
        return;
    }

    /* 找到 t 所在段:最大的 i 满足 seg_t0[i] <= t */
    for (i = 0; i < 7; i++)
    {
        if (tr->seg_t0[i] <= t)
        {
            idx = i;
        }
    }

    float tau = t - tr->seg_t0[idx];
    float p0  = tr->seg_p0[idx];
    float v0  = tr->seg_v0[idx];
    float a0  = tr->seg_a0[idx];
    float jj  = tr->seg_j[idx];

    *p = p0 + v0 * tau + 0.5f * a0 * tau * tau + (1.0f / 6.0f) * jj * tau * tau * tau;
    *v = v0 + a0 * tau + 0.5f * jj * tau * tau;
    *a = a0 + jj * tau;
}

/* 用当前起点(out)和最终目标启动一条新轨迹 */
static void TrajTrack_StartPlan(traj_track_t *tr, float target)
{
    float delta = target - tr->out;

    tr->target     = target;
    tr->plan_start = tr->out;
    tr->dir        = (delta >= 0.0f) ? 1.0f : -1.0f;
    tr->t          = 0.0f;

    TrajTrack_BuildProfile(tr, fabsf(delta));

    tr->busy = (tr->total_time > TRAJ_EPS) ? 1U : 0U;
}

void TrajTrack_Init(traj_track_t *tr, float v_max, float a_max, float j_max,
                    float period, float start_pos)
{
    tr->v_max  = (v_max > TRAJ_EPS) ? v_max : TRAJ_EPS;
    tr->a_max  = (a_max > TRAJ_EPS) ? a_max : TRAJ_EPS;
    tr->j_max  = (j_max > TRAJ_EPS) ? j_max : TRAJ_EPS;
    tr->period = (period > TRAJ_EPS) ? period : MOTOR_CONTROL_PERIOD;

    TrajTrack_Reset(tr, start_pos);
}

void TrajTrack_Reset(traj_track_t *tr, float pos)
{
    tr->target      = pos;
    tr->out         = pos;
    tr->vel         = 0.0f;
    tr->acc         = 0.0f;
    tr->t           = 0.0f;
    tr->total_time  = 0.0f;
    tr->plan_start  = pos;
    tr->dir         = 1.0f;
    tr->busy        = 0U;
    tr->has_pending = 0U;
    tr->pending_target = pos;
}

void TrajTrack_SetTarget(traj_track_t *tr, float target)
{
    if (tr->busy)
    {
        /* 做完再接:暂存最新目标 */
        tr->pending_target = target;
        tr->has_pending    = 1U;
    }
    else
    {
        TrajTrack_StartPlan(tr, target);
    }
}

float TrajTrack_Calc(traj_track_t *tr)
{
    if (!tr->busy)
    {
        /* 空闲:若有暂存目标则启动,否则保持 */
        if (tr->has_pending)
        {
            tr->has_pending = 0U;
            TrajTrack_StartPlan(tr, tr->pending_target);
        }

        if (!tr->busy)
        {
            tr->vel = 0.0f;
            tr->acc = 0.0f;
            return tr->out;
        }
    }

    tr->t += tr->period;

    if (tr->t >= tr->total_time)
    {
        /* 轨迹走完:精确落到目标,转空闲 */
        tr->t    = tr->total_time;
        tr->out  = tr->target;
        tr->vel  = 0.0f;
        tr->acc  = 0.0f;
        tr->busy = 0U;
        return tr->out;
    }

    float p, v, a;
    TrajTrack_Eval(tr, tr->t, &p, &v, &a);

    tr->out = tr->plan_start + tr->dir * p;
    tr->vel = tr->dir * v;
    tr->acc = tr->dir * a;

    return tr->out;
}

uint8_t TrajTrack_IsBusy(const traj_track_t *tr)
{
    return tr->busy;
}
