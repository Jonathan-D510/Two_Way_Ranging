#include "uwb_nav.h"
#include "uwb_twr.h"
#include "uwb_ids.h"
#include "uwb_ctrl.h"
#include "uart.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <string.h>

/* ----------- USER SETTINGS ----------- */
#define MAIN_Z_M        0.98f
#define FOB_Z_M         0.98f

#define STARTUP_AVG_SAMPLES         30u
#define STARTUP_SAMPLE_TOL_MAIN_M   0.15f
#define STARTUP_SAMPLE_TOL_FOB_M    0.20f
#define STARTUP_SAMPLE_TOL_DF_M     0.20f

#define NAV_PERIOD_MS        200u
#define CMD_TIMEOUT_MS       300u
#define RANGE_MAX_M          50.0f

/* Direct MAIN->FOB distance correction experiment */
#define DF_BIAS_M            0.00f

/* Fixed bias constants from the last good known-position calibration */
#define BIAS_MAIN_A1_M   (-0.034f)
#define BIAS_MAIN_A2_M   ( 0.195f)
#define BIAS_MAIN_A3_M   (-0.992f)

#define BIAS_FOB_A1_M    ( 0.000f)
#define BIAS_FOB_A2_M    ( 0.000f)
#define BIAS_FOB_A3_M    ( 0.000f)

/* MAIN dynamic Kalman tuning */
#define KF_Q_MAIN_STILL         0.0001f
#define KF_R_MAIN_STILL         0.3500f

#define KF_Q_MAIN_MOVE          0.0040f
#define KF_R_MAIN_MOVE          0.0400f

/* Hysteresis for still/move mode switching */
#define MAIN_MOVE_ENTER_M       0.055f
#define MAIN_MOVE_EXIT_M        0.025f

/* Still-state hold layer for final MAIN coordinates */
#define MAIN_STILL_HOLD_ENTER_MS            800u
#define MAIN_STILL_HOLD_RADIUS_M            0.025f
#define MAIN_STILL_RELEASE_M                0.070f

/* Dynamic release confirmation */
#define MAIN_STILL_RELEASE_CONFIRM_BASE     3u
#define MAIN_STILL_RELEASE_CONFIRM_MAX      6u
#define MAIN_STILL_RELEASE_CONFIRM_STEP_MS  2500u

/* New-center confirmation after a release attempt */
#define MAIN_STILL_RECENTER_CONFIRM_SAMPLES 3u
#define MAIN_STILL_RECENTER_RADIUS_M        0.025f
#define MAIN_STILL_RECENTER_ALPHA           0.35f

/* Final still-only displayed-output clamp */
#define MAIN_STILL_OUT_ENTER_MS             1200u
#define MAIN_STILL_OUT_HOLD_RADIUS_M        0.015f
#define MAIN_STILL_OUT_BLEND_RADIUS_M       0.030f
#define MAIN_STILL_OUT_RECENTER_RADIUS_M    0.050f
#define MAIN_STILL_OUT_RECENTER_SAMPLES     4u
#define MAIN_STILL_OUT_RECENTER_ALPHA       0.25f

/* FOB Kalman tuning */
#define KF_Q_FOB                0.0030f
#define KF_R_FOB                0.0200f

/* Heading updates only when rover movement is large enough */
#define HEADING_MOVE_MIN_M      0.05f
#define ROVER_HEADING_INIT_DEG  0.0f

/* Light smoothing of rover motion vector before heading */
#define HEADING_VEC_ALPHA       0.35f

/* Very light smoothing of final rover->FOB relative vector */
#define RELVEC_ALPHA            0.30f

/* Soft continuity clamp on MAIN solved position only */
#define MAIN_STEP_MAX_M         0.18f
/* ------------------------------------ */

typedef struct { float x, y, z; } vec3;

typedef struct
{
    uint32_t n;
    float sum;
} avg_accum_t;

typedef struct
{
    float x;
    float p;
    uint8_t init;
} kalman1d_t;

static const vec3 A1 = { 3.40f, 0.98f, 2.00f };
static const vec3 A2 = { 1.11f, 0.00f, 2.09f };
static const vec3 A3 = { 2.13f, 3.87f, 2.14f };

static uint8_t  g_my_id = 0;
static uint32_t g_last_ms = 0;
static uint8_t  g_cmd_seq = 0;
static uint32_t g_dbg_ctr = 0;
static uint8_t  g_nav_ready = 0;

/* latest raw measurements */
static float g_df  = NAN;   /* MAIN -> FOB */
static float g_d1  = NAN;   /* MAIN -> A1 */
static float g_d2  = NAN;   /* MAIN -> A2 */
static float g_d3  = NAN;   /* MAIN -> A3 */
static float g_a1f = NAN;   /* A1 -> FOB */
static float g_a2f = NAN;   /* A2 -> FOB */
static float g_a3f = NAN;   /* A3 -> FOB */

/* startup averaging accumulators */
static avg_accum_t g_avg_d1  = {0};
static avg_accum_t g_avg_d2  = {0};
static avg_accum_t g_avg_d3  = {0};
static avg_accum_t g_avg_a1f = {0};
static avg_accum_t g_avg_a2f = {0};
static avg_accum_t g_avg_a3f = {0};
static avg_accum_t g_avg_df  = {0};

/* raw continuity */
static float g_last_main_raw_x = NAN;
static float g_last_main_raw_y = NAN;
static uint8_t g_have_last_main_raw = 0;

static float g_last_fob_raw_x = NAN;
static float g_last_fob_raw_y = NAN;
static uint8_t g_have_last_fob_raw = 0;

/* filtered state */
static kalman1d_t g_kf_main_x = {0};
static kalman1d_t g_kf_main_y = {0};
static kalman1d_t g_kf_fob_x  = {0};
static kalman1d_t g_kf_fob_y  = {0};

/* dynamic MAIN motion mode: 0=still, 1=move */
static uint8_t g_main_move_mode = 1;
static uint32_t g_main_still_since_ms = 0;
static float g_main_hold_x = NAN;
static float g_main_hold_y = NAN;
static uint8_t g_main_hold_active = 0;
static uint8_t g_main_hold_release_count = 0;
static uint32_t g_main_hold_active_since_ms = 0;

/* pending recenter candidate while still */
static uint8_t g_main_recenter_active = 0;
static float g_main_recenter_x = NAN;
static float g_main_recenter_y = NAN;
static uint8_t g_main_recenter_count = 0;

/* final still-only displayed output clamp state */
static uint8_t  g_main_out_hold_active = 0;
static float    g_main_out_x = NAN;
static float    g_main_out_y = NAN;
static uint32_t g_main_out_hold_since_ms = 0;
static uint8_t  g_main_out_recenter_active = 0;
static float    g_main_out_recenter_x = NAN;
static float    g_main_out_recenter_y = NAN;
static uint8_t  g_main_out_recenter_count = 0;

/* rover heading state */
static float g_prev_main_filt_x = NAN;
static float g_prev_main_filt_y = NAN;
static uint8_t g_have_prev_main_filt = 0;
static float g_rover_heading_deg = ROVER_HEADING_INIT_DEG;
static uint8_t g_have_rover_heading = 0;

/* light smoothing of rover motion vector used for heading */
static float g_heading_vx_filt = NAN;
static float g_heading_vy_filt = NAN;
static uint8_t g_have_heading_vec_filt = 0;

/* very light final relative-vector smoothing */
static float g_rel_dx_filt = NAN;
static float g_rel_dy_filt = NAN;
static uint8_t g_have_rel_filt = 0;

static int is_ok(float d)
{
    return isfinite(d) && d > 0.05f && d < RANGE_MAX_M;
}

static float dist2_xy(float x1, float y1, float x2, float y2)
{
    float dx = x1 - x2;
    float dy = y1 - y2;
    return sqrtf(dx*dx + dy*dy);
}

static float horiz_from_d(float d3, float z_anchor, float z_target)
{
    float dz = z_anchor - z_target;
    float v = d3*d3 - dz*dz;
    if (v <= 0.0f) return NAN;
    return sqrtf(v);
}

static float corr(float d, float b)
{
    float v;
    if (!isfinite(d) || !isfinite(b)) return NAN;
    v = d - b;
    if (v < 0.05f) v = 0.05f;
    return v;
}

static float rad2deg(float r)
{
    return r * 57.2957795f;
}

static float wrap_deg_360(float deg)
{
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

static float wrap_deg_180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

static void avg_reset(avg_accum_t *a)
{
    a->n = 0;
    a->sum = 0.0f;
}

static float avg_get(const avg_accum_t *a)
{
    if (a->n == 0) return NAN;
    return a->sum / (float)a->n;
}

static void avg_add_gated(avg_accum_t *a, float v, float tol)
{
    float cur;

    if (!isfinite(v)) return;

    if (a->n < 3u)
    {
        a->sum += v;
        a->n++;
        return;
    }

    cur = avg_get(a);
    if (!isfinite(cur)) return;

    if (fabsf(v - cur) <= tol)
    {
        a->sum += v;
        a->n++;
    }
}

static void kf_reset(kalman1d_t *k)
{
    k->x = 0.0f;
    k->p = 1.0f;
    k->init = 0;
}

static void kf_seed(kalman1d_t *k, float x0)
{
    k->x = x0;
    k->p = 0.05f;
    k->init = 1;
}

static float kf_update(kalman1d_t *k, float z, float q, float r)
{
    float k_gain;

    if (!isfinite(z)) return k->x;

    if (!k->init)
    {
        k->x = z;
        k->p = 1.0f;
        k->init = 1;
        return k->x;
    }

    k->p += q;
    k_gain = k->p / (k->p + r);
    k->x   = k->x + k_gain * (z - k->x);
    k->p   = (1.0f - k_gain) * k->p;

    return k->x;
}

static uint8_t main_hold_required_release_samples(uint32_t now)
{
    uint32_t hold_age_ms;
    uint8_t req = MAIN_STILL_RELEASE_CONFIRM_BASE;

    if (!g_main_hold_active || g_main_hold_active_since_ms == 0u)
        return MAIN_STILL_RELEASE_CONFIRM_BASE;

    hold_age_ms = now - g_main_hold_active_since_ms;

    while ((req < MAIN_STILL_RELEASE_CONFIRM_MAX) &&
           (hold_age_ms >= MAIN_STILL_RELEASE_CONFIRM_STEP_MS))
    {
        req++;
        hold_age_ms -= MAIN_STILL_RELEASE_CONFIRM_STEP_MS;
    }

    return req;
}

static void main_recenter_reset(void)
{
    g_main_recenter_active = 0;
    g_main_recenter_x = NAN;
    g_main_recenter_y = NAN;
    g_main_recenter_count = 0;
}

static void main_out_recenter_reset(void)
{
    g_main_out_recenter_active = 0;
    g_main_out_recenter_x = NAN;
    g_main_out_recenter_y = NAN;
    g_main_out_recenter_count = 0;
}

static int trilat2d(float *outx, float *outy,
                    const vec3 *p1, float r1,
                    const vec3 *p2, float r2,
                    const vec3 *p3, float r3)
{
    float A = 2.0f * (p2->x - p1->x);
    float B = 2.0f * (p2->y - p1->y);
    float C = (r1*r1 - r2*r2)
            + (p2->x*p2->x - p1->x*p1->x)
            + (p2->y*p2->y - p1->y*p1->y);

    float D = 2.0f * (p3->x - p1->x);
    float E = 2.0f * (p3->y - p1->y);
    float F = (r1*r1 - r3*r3)
            + (p3->x*p3->x - p1->x*p1->x)
            + (p3->y*p3->y - p1->y*p1->y);

    float det = A*E - B*D;
    float x, y;

    if (fabsf(det) < 1e-6f) return -1;

    x = (C*E - B*F) / det;
    y = (A*F - C*D) / det;

    if (!isfinite(x) || !isfinite(y)) return -2;

    *outx = x;
    *outy = y;
    return 0;
}

static int circle_circle_intersections(float x0, float y0, float r0,
                                       float x1, float y1, float r1,
                                       float *xa, float *ya,
                                       float *xb, float *yb)
{
    float dx = x1 - x0;
    float dy = y1 - y0;
    float d = sqrtf(dx*dx + dy*dy);

    if (!isfinite(d) || d < 1e-6f) return -1;
    if (d > (r0 + r1)) return -2;
    if (d < fabsf(r0 - r1)) return -3;

    {
        float a = (r0*r0 - r1*r1 + d*d) / (2.0f * d);
        float h2 = r0*r0 - a*a;
        float xm, ym, rx, ry, h;

        if (h2 < 0.0f) h2 = 0.0f;
        h = sqrtf(h2);

        xm = x0 + a * dx / d;
        ym = y0 + a * dy / d;

        rx = -dy * (h / d);
        ry =  dx * (h / d);

        *xa = xm + rx;
        *ya = ym + ry;
        *xb = xm - rx;
        *yb = ym - ry;
    }

    return 0;
}

static float fob_candidate_score(float x, float y)
{
    float score = 0.0f;

    if (!isfinite(x) || !isfinite(y))
        return 1e9f;

    if (is_ok(g_df) && g_have_last_main_raw &&
        isfinite(g_last_main_raw_x) && isfinite(g_last_main_raw_y))
    {
        float err_df = fabsf(dist2_xy(x, y, g_last_main_raw_x, g_last_main_raw_y) - g_df);
        score += 4.0f * err_df;
    }

    if (g_have_last_fob_raw && isfinite(g_last_fob_raw_x) && isfinite(g_last_fob_raw_y))
    {
        score += 1.0f * dist2_xy(x, y, g_last_fob_raw_x, g_last_fob_raw_y);
    }

    return score;
}

static void choose_candidate(float x1, float y1,
                             float x2, float y2,
                             float *xo, float *yo)
{
    float s1 = fob_candidate_score(x1, y1);
    float s2 = fob_candidate_score(x2, y2);

    if (s1 <= s2)
    {
        *xo = x1;
        *yo = y1;
    }
    else
    {
        *xo = x2;
        *yo = y2;
    }
}

static void clamp_main_step(float *x, float *y)
{
    float dx, dy, step_m, scale;

    if (!g_have_last_main_raw) return;
    if (!isfinite(*x) || !isfinite(*y)) return;
    if (!isfinite(g_last_main_raw_x) || !isfinite(g_last_main_raw_y)) return;

    dx = *x - g_last_main_raw_x;
    dy = *y - g_last_main_raw_y;
    step_m = sqrtf(dx*dx + dy*dy);

    if (!isfinite(step_m) || step_m <= MAIN_STEP_MAX_M) return;
    if (step_m < 1e-6f) return;

    scale = MAIN_STEP_MAX_M / step_m;
    *x = g_last_main_raw_x + dx * scale;
    *y = g_last_main_raw_y + dy * scale;
}

static void update_anchor_to_fob_ranges(void)
{
    float m = NAN;
    uint8_t st = 0;
    int rc;

    rc = uwb_ctrl_range_fob_via_anchor(g_my_id, UWB_ID_A1, &g_cmd_seq, &m, &st, CMD_TIMEOUT_MS);
    if (rc == 0 && st == 0 && is_ok(m)) g_a1f = m; else g_a1f = NAN;
    HAL_Delay(5);

    m = NAN; st = 0;
    rc = uwb_ctrl_range_fob_via_anchor(g_my_id, UWB_ID_A2, &g_cmd_seq, &m, &st, CMD_TIMEOUT_MS);
    if (rc == 0 && st == 0 && is_ok(m)) g_a2f = m; else g_a2f = NAN;
    HAL_Delay(5);

    m = NAN; st = 0;
    rc = uwb_ctrl_range_fob_via_anchor(g_my_id, UWB_ID_A3, &g_cmd_seq, &m, &st, CMD_TIMEOUT_MS);
    if (rc == 0 && st == 0 && is_ok(m)) g_a3f = m; else g_a3f = NAN;
}

static int solve_main_xy_from_ranges(float d1m, float d2m, float d3m, float *mx, float *my)
{
    float d1c = corr(d1m, BIAS_MAIN_A1_M);
    float d2c = corr(d2m, BIAS_MAIN_A2_M);
    float d3c = corr(d3m, BIAS_MAIN_A3_M);

    float r1 = horiz_from_d(d1c, A1.z, MAIN_Z_M);
    float r2 = horiz_from_d(d2c, A2.z, MAIN_Z_M);
    float r3 = horiz_from_d(d3c, A3.z, MAIN_Z_M);

    if (isfinite(r1) && isfinite(r2) && isfinite(r3))
    {
        if (trilat2d(mx, my, &A1, r1, &A2, r2, &A3, r3) == 0)
            return 0;
    }

    return -1;
}

static int solve_fob_xy_from_ranges(float a1m, float a2m, float a3m, float *fx, float *fy)
{
    float a1c = corr(a1m, BIAS_FOB_A1_M);
    float a2c = corr(a2m, BIAS_FOB_A2_M);
    float a3c = corr(a3m, BIAS_FOB_A3_M);

    float r1 = horiz_from_d(a1c, A1.z, FOB_Z_M);
    float r2 = horiz_from_d(a2c, A2.z, FOB_Z_M);
    float r3 = horiz_from_d(a3c, A3.z, FOB_Z_M);

    int have1 = isfinite(r1);
    int have2 = isfinite(r2);
    int have3 = isfinite(r3);

    if (have1 && have2 && have3)
    {
        if (trilat2d(fx, fy, &A1, r1, &A2, r2, &A3, r3) == 0)
            return 0;
    }

    if (have2 && have3)
    {
        float xa, ya, xb, yb;
        if (circle_circle_intersections(A2.x, A2.y, r2, A3.x, A3.y, r3,
                                        &xa, &ya, &xb, &yb) == 0)
        {
            choose_candidate(xa, ya, xb, yb, fx, fy);
            return 0;
        }
    }

    if (have1 && have3)
    {
        float xa, ya, xb, yb;
        if (circle_circle_intersections(A1.x, A1.y, r1, A3.x, A3.y, r3,
                                        &xa, &ya, &xb, &yb) == 0)
        {
            choose_candidate(xa, ya, xb, yb, fx, fy);
            return 0;
        }
    }

    if (have1 && have2)
    {
        float xa, ya, xb, yb;
        if (circle_circle_intersections(A1.x, A1.y, r1, A2.x, A2.y, r2,
                                        &xa, &ya, &xb, &yb) == 0)
        {
            choose_candidate(xa, ya, xb, yb, fx, fy);
            return 0;
        }
    }

    return -1;
}

static int startup_ready_enough(void)
{
    int main_ready = (g_avg_d1.n >= STARTUP_AVG_SAMPLES &&
                      g_avg_d2.n >= STARTUP_AVG_SAMPLES &&
                      g_avg_d3.n >= STARTUP_AVG_SAMPLES);

    int fob_ready = ((g_avg_a1f.n >= STARTUP_AVG_SAMPLES && g_avg_a2f.n >= STARTUP_AVG_SAMPLES) ||
                     (g_avg_a1f.n >= STARTUP_AVG_SAMPLES && g_avg_a3f.n >= STARTUP_AVG_SAMPLES) ||
                     (g_avg_a2f.n >= STARTUP_AVG_SAMPLES && g_avg_a3f.n >= STARTUP_AVG_SAMPLES));

    return main_ready && fob_ready;
}

void uwb_nav_init(uint8_t my_id)
{
    g_my_id = my_id;
    g_last_ms = HAL_GetTick();
    g_cmd_seq = 0;
    g_dbg_ctr = 0;
    g_nav_ready = 0;

    g_df = g_d1 = g_d2 = g_d3 = NAN;
    g_a1f = g_a2f = g_a3f = NAN;

    avg_reset(&g_avg_d1);
    avg_reset(&g_avg_d2);
    avg_reset(&g_avg_d3);
    avg_reset(&g_avg_a1f);
    avg_reset(&g_avg_a2f);
    avg_reset(&g_avg_a3f);
    avg_reset(&g_avg_df);

    g_last_main_raw_x = g_last_main_raw_y = NAN;
    g_have_last_main_raw = 0;
    g_last_fob_raw_x = g_last_fob_raw_y = NAN;
    g_have_last_fob_raw = 0;

    g_main_move_mode = 1;
    g_main_still_since_ms = 0;
    g_main_hold_x = NAN;
    g_main_hold_y = NAN;
    g_main_hold_active = 0;
    g_main_hold_release_count = 0;
    g_main_hold_active_since_ms = 0;
    main_recenter_reset();

    g_main_out_hold_active = 0;
    g_main_out_x = NAN;
    g_main_out_y = NAN;
    g_main_out_hold_since_ms = 0;
    main_out_recenter_reset();

    g_prev_main_filt_x = NAN;
    g_prev_main_filt_y = NAN;
    g_have_prev_main_filt = 0;
    g_rover_heading_deg = ROVER_HEADING_INIT_DEG;
    g_have_rover_heading = 0;

    g_heading_vx_filt = NAN;
    g_heading_vy_filt = NAN;
    g_have_heading_vec_filt = 0;

    g_rel_dx_filt = NAN;
    g_rel_dy_filt = NAN;
    g_have_rel_filt = 0;

    kf_reset(&g_kf_main_x);
    kf_reset(&g_kf_main_y);
    kf_reset(&g_kf_fob_x);
    kf_reset(&g_kf_fob_y);

    uwb_twr_init(g_my_id);

    dbg_printf("NAV: init my_id=%u self-start refined mode, anchors fixed\r\n", (unsigned)g_my_id);
    dbg_printf("NAV: startup averaging %u samples, gated main_tol=%.2f fob_tol=%.2f df_tol=%.2f | DF_BIAS_M=%.2f\r\n",
               (unsigned)STARTUP_AVG_SAMPLES,
               (double)STARTUP_SAMPLE_TOL_MAIN_M,
               (double)STARTUP_SAMPLE_TOL_FOB_M,
               (double)STARTUP_SAMPLE_TOL_DF_M,
               (double)DF_BIAS_M);
}

void uwb_nav_step(void)
{
    uint32_t now = HAL_GetTick();
    float df = 0.0f, d1 = 0.0f, d2 = 0.0f, d3 = 0.0f;
    int rf, r1, r2, r3;

    float mx_raw, my_raw, fx_raw, fy_raw;
    float mx_filt, my_filt, fx_filt, fy_filt;
    float mx_out, my_out;
    float dx, dy, dist_m;
    float alpha_deg, psi_deg, theta_deg;
    float q_main, r_main;
    int main_ok, fob_ok;

    if ((now - g_last_ms) < NAV_PERIOD_MS) return;
    g_last_ms = now;
    g_dbg_ctr++;

    rf = uwb_twr_range_to(UWB_ID_FOB, &df); HAL_Delay(5);
    r1 = uwb_twr_range_to(UWB_ID_A1,  &d1); HAL_Delay(5);
    r2 = uwb_twr_range_to(UWB_ID_A2,  &d2); HAL_Delay(5);
    r3 = uwb_twr_range_to(UWB_ID_A3,  &d3); HAL_Delay(5);

    if (rf == 0 && is_ok(df)) g_df = df; else g_df = NAN;
    if (r1 == 0 && is_ok(d1)) g_d1 = d1; else g_d1 = NAN;
    if (r2 == 0 && is_ok(d2)) g_d2 = d2; else g_d2 = NAN;
    if (r3 == 0 && is_ok(d3)) g_d3 = d3; else g_d3 = NAN;

    update_anchor_to_fob_ranges();

    if (!g_nav_ready)
    {
        avg_add_gated(&g_avg_d1,  g_d1,  STARTUP_SAMPLE_TOL_MAIN_M);
        avg_add_gated(&g_avg_d2,  g_d2,  STARTUP_SAMPLE_TOL_MAIN_M);
        avg_add_gated(&g_avg_d3,  g_d3,  STARTUP_SAMPLE_TOL_MAIN_M);
        avg_add_gated(&g_avg_a1f, g_a1f, STARTUP_SAMPLE_TOL_FOB_M);
        avg_add_gated(&g_avg_a2f, g_a2f, STARTUP_SAMPLE_TOL_FOB_M);
        avg_add_gated(&g_avg_a3f, g_a3f, STARTUP_SAMPLE_TOL_FOB_M);
        avg_add_gated(&g_avg_df,  g_df,  STARTUP_SAMPLE_TOL_DF_M);

        dbg_printf("NAV: (startup) d1[%lu/%u]=%.2f d2[%lu/%u]=%.2f d3[%lu/%u]=%.2f | a1f[%lu/%u]=%.2f a2f[%lu/%u]=%.2f a3f[%lu/%u]=%.2f | df=%.2f\r\n",
                   (unsigned long)g_avg_d1.n,  (unsigned)STARTUP_AVG_SAMPLES, (double)g_d1,
                   (unsigned long)g_avg_d2.n,  (unsigned)STARTUP_AVG_SAMPLES, (double)g_d2,
                   (unsigned long)g_avg_d3.n,  (unsigned)STARTUP_AVG_SAMPLES, (double)g_d3,
                   (unsigned long)g_avg_a1f.n, (unsigned)STARTUP_AVG_SAMPLES, (double)g_a1f,
                   (unsigned long)g_avg_a2f.n, (unsigned)STARTUP_AVG_SAMPLES, (double)g_a2f,
                   (unsigned long)g_avg_a3f.n, (unsigned)STARTUP_AVG_SAMPLES, (double)g_a3f,
                   (double)g_df);

        if (startup_ready_enough())
        {
            float mx0 = NAN, my0 = NAN, fx0 = NAN, fy0 = NAN;
            float d1a = avg_get(&g_avg_d1);
            float d2a = avg_get(&g_avg_d2);
            float d3a = avg_get(&g_avg_d3);
            float a1a = avg_get(&g_avg_a1f);
            float a2a = avg_get(&g_avg_a2f);
            float a3a = avg_get(&g_avg_a3f);

            int mrc = solve_main_xy_from_ranges(d1a, d2a, d3a, &mx0, &my0);
            int frc = solve_fob_xy_from_ranges(a1a, a2a, a3a, &fx0, &fy0);

            if ((mrc == 0) && (frc == 0))
            {
                float init_alpha = wrap_deg_360(rad2deg(atan2f(fy0 - my0, fx0 - mx0)));

                g_nav_ready = 1;

                g_last_main_raw_x = mx0;
                g_last_main_raw_y = my0;
                g_have_last_main_raw = 1;

                g_last_fob_raw_x = fx0;
                g_last_fob_raw_y = fy0;
                g_have_last_fob_raw = 1;

                kf_seed(&g_kf_main_x, mx0);
                kf_seed(&g_kf_main_y, my0);
                kf_seed(&g_kf_fob_x,  fx0);
                kf_seed(&g_kf_fob_y,  fy0);

                g_prev_main_filt_x = mx0;
                g_prev_main_filt_y = my0;
                g_have_prev_main_filt = 1;

                g_rover_heading_deg = init_alpha;
                g_have_rover_heading = 1;

                g_rel_dx_filt = fx0 - mx0;
                g_rel_dy_filt = fy0 - my0;
                g_have_rel_filt = 1;

                g_main_move_mode = 0;
                g_main_still_since_ms = now;

                g_main_hold_x = mx0;
                g_main_hold_y = my0;
                g_main_hold_active = 1;
                g_main_hold_release_count = 0;
                g_main_hold_active_since_ms = now;
                main_recenter_reset();

                g_main_out_x = mx0;
                g_main_out_y = my0;
                g_main_out_hold_active = 1;
                g_main_out_hold_since_ms = now;
                main_out_recenter_reset();

                dbg_printf("NAV: READY refined self-start main=(%.2f,%.2f) fob=(%.2f,%.2f) alpha0=%.2f\r\n",
                           (double)mx0, (double)my0, (double)fx0, (double)fy0, (double)init_alpha);
            }
            else
            {
                dbg_printf("NAV: startup solve failed main_rc=%d fob_rc=%d\r\n", mrc, frc);
            }
        }

        return;
    }

    mx_raw = my_raw = fx_raw = fy_raw = NAN;

    main_ok = (solve_main_xy_from_ranges(g_d1, g_d2, g_d3, &mx_raw, &my_raw) == 0);

    if (main_ok)
    {
        clamp_main_step(&mx_raw, &my_raw);

        g_last_main_raw_x = mx_raw;
        g_last_main_raw_y = my_raw;
        g_have_last_main_raw = 1;
    }

    fob_ok = (solve_fob_xy_from_ranges(g_a1f, g_a2f, g_a3f, &fx_raw, &fy_raw) == 0);

    if (!main_ok || !fob_ok)
    {
        dbg_printf("NAV: no_fix main_ok=%d fob_ok=%d | df=%.2f | d1=%.2f d2=%.2f d3=%.2f | a1f=%.2f a2f=%.2f a3f=%.2f\r\n",
                   main_ok, fob_ok,
                   (double)g_df,
                   (double)g_d1, (double)g_d2, (double)g_d3,
                   (double)g_a1f, (double)g_a2f, (double)g_a3f);
        return;
    }

    g_last_fob_raw_x = fx_raw;
    g_last_fob_raw_y = fy_raw;
    g_have_last_fob_raw = 1;

    if (g_have_prev_main_filt)
    {
        float pred_mdx = mx_raw - g_prev_main_filt_x;
        float pred_mdy = my_raw - g_prev_main_filt_y;
        float pred_move_m = sqrtf(pred_mdx*pred_mdx + pred_mdy*pred_mdy);

        if (g_main_move_mode)
        {
            if (pred_move_m < MAIN_MOVE_EXIT_M)
                g_main_move_mode = 0;
        }
        else
        {
            if (pred_move_m > MAIN_MOVE_ENTER_M)
                g_main_move_mode = 1;
        }
    }
    else
    {
        g_main_move_mode = 1;
    }

    if (g_main_move_mode)
    {
        q_main = KF_Q_MAIN_MOVE;
        r_main = KF_R_MAIN_MOVE;
    }
    else
    {
        q_main = KF_Q_MAIN_STILL;
        r_main = KF_R_MAIN_STILL;
    }

    mx_filt = kf_update(&g_kf_main_x, mx_raw, q_main, r_main);
    my_filt = kf_update(&g_kf_main_y, my_raw, q_main, r_main);
    fx_filt = kf_update(&g_kf_fob_x,  fx_raw, KF_Q_FOB,  KF_R_FOB);
    fy_filt = kf_update(&g_kf_fob_y,  fy_raw, KF_Q_FOB,  KF_R_FOB);

    if (!g_main_move_mode)
    {
        if (g_main_still_since_ms == 0u)
            g_main_still_since_ms = now;

        if (!g_main_hold_active)
        {
            if ((now - g_main_still_since_ms) >= MAIN_STILL_HOLD_ENTER_MS)
            {
                g_main_hold_x = mx_filt;
                g_main_hold_y = my_filt;
                g_main_hold_active = 1;
                g_main_hold_release_count = 0;
                g_main_hold_active_since_ms = now;
                main_recenter_reset();
            }
        }
        else
        {
            float hold_err_x = mx_filt - g_main_hold_x;
            float hold_err_y = my_filt - g_main_hold_y;
            float hold_err_m = sqrtf(hold_err_x*hold_err_x + hold_err_y*hold_err_y);
            uint8_t req_release_samples = main_hold_required_release_samples(now);

            if (hold_err_m <= MAIN_STILL_HOLD_RADIUS_M)
            {
                g_main_hold_release_count = 0;
                main_recenter_reset();
                mx_filt = g_main_hold_x;
                my_filt = g_main_hold_y;
            }
            else if (hold_err_m < MAIN_STILL_RELEASE_M)
            {
                float t = (hold_err_m - MAIN_STILL_HOLD_RADIUS_M) /
                          (MAIN_STILL_RELEASE_M - MAIN_STILL_HOLD_RADIUS_M);
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;

                g_main_hold_release_count = 0;
                main_recenter_reset();
                mx_filt = g_main_hold_x + t * (mx_filt - g_main_hold_x);
                my_filt = g_main_hold_y + t * (my_filt - g_main_hold_y);
            }
            else
            {
                if (g_main_hold_release_count < 255u)
                    g_main_hold_release_count++;

                if (g_main_hold_release_count < req_release_samples)
                {
                    main_recenter_reset();
                    mx_filt = g_main_hold_x;
                    my_filt = g_main_hold_y;
                }
                else
                {
                    if (!g_main_recenter_active)
                    {
                        g_main_recenter_active = 1;
                        g_main_recenter_x = mx_filt;
                        g_main_recenter_y = my_filt;
                        g_main_recenter_count = 1;
                        mx_filt = g_main_hold_x;
                        my_filt = g_main_hold_y;
                    }
                    else
                    {
                        float cand_dx = mx_filt - g_main_recenter_x;
                        float cand_dy = my_filt - g_main_recenter_y;
                        float cand_err_m = sqrtf(cand_dx*cand_dx + cand_dy*cand_dy);

                        if (cand_err_m <= MAIN_STILL_RECENTER_RADIUS_M)
                        {
                            g_main_recenter_x = g_main_recenter_x +
                                                MAIN_STILL_RECENTER_ALPHA * (mx_filt - g_main_recenter_x);
                            g_main_recenter_y = g_main_recenter_y +
                                                MAIN_STILL_RECENTER_ALPHA * (my_filt - g_main_recenter_y);

                            if (g_main_recenter_count < 255u)
                                g_main_recenter_count++;
                        }
                        else
                        {
                            g_main_recenter_x = mx_filt;
                            g_main_recenter_y = my_filt;
                            g_main_recenter_count = 1;
                        }

                        if (g_main_recenter_count >= MAIN_STILL_RECENTER_CONFIRM_SAMPLES)
                        {
                            g_main_hold_x = g_main_recenter_x;
                            g_main_hold_y = g_main_recenter_y;
                            g_main_hold_active = 1;
                            g_main_hold_release_count = 0;
                            g_main_hold_active_since_ms = now;
                            g_main_still_since_ms = now;
                            main_recenter_reset();

                            mx_filt = g_main_hold_x;
                            my_filt = g_main_hold_y;
                        }
                        else
                        {
                            mx_filt = g_main_hold_x;
                            my_filt = g_main_hold_y;
                        }
                    }
                }
            }
        }
    }
    else
    {
        g_main_still_since_ms = 0u;
        g_main_hold_active = 0;
        g_main_hold_release_count = 0;
        g_main_hold_active_since_ms = 0;
        main_recenter_reset();
    }

    mx_out = mx_filt;
    my_out = my_filt;

    if (!g_main_move_mode)
    {
        if (!g_main_out_hold_active)
        {
            if ((now - g_main_still_since_ms) >= MAIN_STILL_OUT_ENTER_MS)
            {
                g_main_out_x = mx_filt;
                g_main_out_y = my_filt;
                g_main_out_hold_active = 1;
                g_main_out_hold_since_ms = now;
                main_out_recenter_reset();
            }
        }
        else
        {
            float out_dx = mx_filt - g_main_out_x;
            float out_dy = my_filt - g_main_out_y;
            float out_err_m = sqrtf(out_dx*out_dx + out_dy*out_dy);

            if (out_err_m <= MAIN_STILL_OUT_HOLD_RADIUS_M)
            {
                main_out_recenter_reset();
                mx_out = g_main_out_x;
                my_out = g_main_out_y;
            }
            else if (out_err_m < MAIN_STILL_OUT_BLEND_RADIUS_M)
            {
                float t = (out_err_m - MAIN_STILL_OUT_HOLD_RADIUS_M) /
                          (MAIN_STILL_OUT_BLEND_RADIUS_M - MAIN_STILL_OUT_HOLD_RADIUS_M);
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;

                main_out_recenter_reset();
                mx_out = g_main_out_x + t * (mx_filt - g_main_out_x);
                my_out = g_main_out_y + t * (my_filt - g_main_out_y);
            }
            else if (out_err_m < MAIN_STILL_OUT_RECENTER_RADIUS_M)
            {
                main_out_recenter_reset();
                mx_out = g_main_out_x;
                my_out = g_main_out_y;
            }
            else
            {
                if (!g_main_out_recenter_active)
                {
                    g_main_out_recenter_active = 1;
                    g_main_out_recenter_x = mx_filt;
                    g_main_out_recenter_y = my_filt;
                    g_main_out_recenter_count = 1;
                    mx_out = g_main_out_x;
                    my_out = g_main_out_y;
                }
                else
                {
                    float cand_dx = mx_filt - g_main_out_recenter_x;
                    float cand_dy = my_filt - g_main_out_recenter_y;
                    float cand_err_m = sqrtf(cand_dx*cand_dx + cand_dy*cand_dy);

                    if (cand_err_m <= MAIN_STILL_RECENTER_RADIUS_M)
                    {
                        g_main_out_recenter_x = g_main_out_recenter_x +
                                                MAIN_STILL_OUT_RECENTER_ALPHA * (mx_filt - g_main_out_recenter_x);
                        g_main_out_recenter_y = g_main_out_recenter_y +
                                                MAIN_STILL_OUT_RECENTER_ALPHA * (my_filt - g_main_out_recenter_y);

                        if (g_main_out_recenter_count < 255u)
                            g_main_out_recenter_count++;
                    }
                    else
                    {
                        g_main_out_recenter_x = mx_filt;
                        g_main_out_recenter_y = my_filt;
                        g_main_out_recenter_count = 1;
                    }

                    if (g_main_out_recenter_count >= MAIN_STILL_OUT_RECENTER_SAMPLES)
                    {
                        g_main_out_x = g_main_out_recenter_x;
                        g_main_out_y = g_main_out_recenter_y;
                        g_main_out_hold_since_ms = now;
                        main_out_recenter_reset();
                        mx_out = g_main_out_x;
                        my_out = g_main_out_y;
                    }
                    else
                    {
                        mx_out = g_main_out_x;
                        my_out = g_main_out_y;
                    }
                }
            }
        }
    }
    else
    {
        g_main_out_hold_active = 0;
        g_main_out_hold_since_ms = 0;
        main_out_recenter_reset();
        mx_out = mx_filt;
        my_out = my_filt;
    }

    dx = fx_filt - mx_out;
    dy = fy_filt - my_out;

    if (!g_have_rel_filt)
    {
        g_rel_dx_filt = dx;
        g_rel_dy_filt = dy;
        g_have_rel_filt = 1;
    }
    else
    {
        g_rel_dx_filt = g_rel_dx_filt + RELVEC_ALPHA * (dx - g_rel_dx_filt);
        g_rel_dy_filt = g_rel_dy_filt + RELVEC_ALPHA * (dy - g_rel_dy_filt);
    }

    /* For this tuning phase, trust solved geometry for displayed distance */
    dist_m = sqrtf(g_rel_dx_filt*g_rel_dx_filt + g_rel_dy_filt*g_rel_dy_filt);

    alpha_deg = wrap_deg_360(rad2deg(atan2f(g_rel_dy_filt, g_rel_dx_filt)));

    if (g_have_prev_main_filt)
    {
        float mdx = mx_out - g_prev_main_filt_x;
        float mdy = my_out - g_prev_main_filt_y;
        float move_m = sqrtf(mdx*mdx + mdy*mdy);

        if (move_m >= HEADING_MOVE_MIN_M)
        {
            if (!g_have_heading_vec_filt)
            {
                g_heading_vx_filt = mdx;
                g_heading_vy_filt = mdy;
                g_have_heading_vec_filt = 1;
            }
            else
            {
                g_heading_vx_filt = g_heading_vx_filt + HEADING_VEC_ALPHA * (mdx - g_heading_vx_filt);
                g_heading_vy_filt = g_heading_vy_filt + HEADING_VEC_ALPHA * (mdy - g_heading_vy_filt);
            }

            if (isfinite(g_heading_vx_filt) && isfinite(g_heading_vy_filt))
            {
                float hv_mag = sqrtf(g_heading_vx_filt*g_heading_vx_filt +
                                     g_heading_vy_filt*g_heading_vy_filt);

                if (hv_mag >= (0.5f * HEADING_MOVE_MIN_M))
                {
                    g_rover_heading_deg = wrap_deg_360(rad2deg(atan2f(g_heading_vy_filt,
                                                                     g_heading_vx_filt)));
                    g_have_rover_heading = 1;
                }
            }
        }
    }

    g_prev_main_filt_x = mx_out;
    g_prev_main_filt_y = my_out;
    g_have_prev_main_filt = 1;

    psi_deg = g_have_rover_heading ? g_rover_heading_deg : ROVER_HEADING_INIT_DEG;
    theta_deg = wrap_deg_180(alpha_deg - psi_deg);

    dbg_printf("UWB,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
               (double)mx_out,
               (double)my_out,
               (double)theta_deg*0.0174532925f,
               (double)dist_m,
               (double)alpha_deg*0.0174532925f);
}
