#include "pendulum_dsp.h"

#ifdef SWING_HOST_TEST
#include <math.h>
#include <string.h>
#undef M_PI
#define M_PI 3.14159265358979323846f
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
#else
#include "tapp_api.h"
#endif

#define SWING_GRAVITY       9.80665f
#define SWING_SOUND_SPEED   343.0f
#define SWING_TWO_PI        6.28318530717958647692f
#define SWING_DEG_TO_RAD    0.01745329251994329577f
#define SWING_RAD_TO_DEG    57.2957795130823208768f
#define SWING_MAX_ARC_DEG   55.0f
#define SWING_LIMIT_RAD     1.08210414f             /* 62 deg safety ceiling (< pi/2) */
#define SWING_COAST_DAMP    0.075f                  /* natural free-pendulum air/pivot damping */
#define SWING_PHYS_SUB      32u                     /* 1.5 kHz Velocity-Verlet rate at 48 kHz */

typedef struct {
    const char* name;
    const char* short_name;
    float d_mic;            /* Spaced hall mic pair width (m) */
    float y_mic;            /* Forward audience mic offset (m) */
    float z_mic;            /* Mic height relative to nadir (m, negative = below arc) */
    float half_wall;        /* Distance from center line to lateral hall walls (m) */
    float y_sway_ratio;     /* Depth sway amplitude relative to lateral arc */
    float ly_ratio;         /* Depth cable length ratio L_y / L */
    float size_scale;       /* Concert hall FDN size multiplier */
} swing_rig_cfg_t;

static const swing_rig_cfg_t kRigs[SWING_RIG_COUNT] = {
    [SWING_RIG_BARBICAN] = {
        .name = "Barbican 10m",
        .short_name = "BARB",
        .d_mic = 1.30f,
        .y_mic = 0.72f,
        .z_mic = -1.05f,
        .half_wall = 11.50f,
        .y_sway_ratio = 0.08f,
        .ly_ratio = 1.00f,
        .size_scale = 1.00f,
    },
    [SWING_RIG_ELLIPTICAL] = {
        .name = "Elliptical 2D",
        .short_name = "2D",
        .d_mic = 1.36f,
        .y_mic = 0.30f,
        .z_mic = -0.95f,
        .half_wall = 10.50f,
        .y_sway_ratio = 0.58f,
        .ly_ratio = 1.00f,
        .size_scale = 0.96f,
    },
    [SWING_RIG_CHAPEL] = {
        .name = "Chapel 5m",
        .short_name = "CHPL",
        .d_mic = 0.96f,
        .y_mic = 0.58f,
        .z_mic = -0.88f,
        .half_wall = 6.20f,
        .y_sway_ratio = 0.14f,
        .ly_ratio = 0.94f,
        .size_scale = 0.88f,
    },
    [SWING_RIG_CLOSE] = {
        .name = "Close 2.4m",
        .short_name = "CLOS",
        .d_mic = 1.45f,
        .y_mic = 0.22f,
        .z_mic = -0.58f,
        .half_wall = 4.40f,
        .y_sway_ratio = 0.06f,
        .ly_ratio = 1.00f,
        .size_scale = 0.74f,
    },
};

/*
 * Branchless minimax sin/cos on [-pi/2, +pi/2] (zero external libm calls!).
 * Pendulum angles are strictly bounded to +-1.082 rad (+-62 deg < 1.5708 rad).
 */
static inline float sin_halfpi(float r) {
    const float r2 = r * r;
    return r * (1.0f - r2 * (0.16666657f - r2 * (0.00833216f - r2 * 0.00019515f)));
}

static inline float cos_halfpi(float r) {
    const float r2 = r * r;
    return 1.0f - r2 * (0.5f - r2 * (0.04166646f - r2 * 0.00138873f));
}

static inline float fast_sin_u32(uint32_t ph) {
    const int32_t sp = (int32_t)ph;
    const float x = (float)sp * (1.0f / 2147483648.0f);
    const float y = 4.0f * x * (1.0f - fabsf(x));
    return y * (0.775f + 0.225f * fabsf(y));
}

float pendulum_clip_guard(float x) {
    return clampf(x, -1.25f, 1.25f);
}

/* Zero-division odd-symmetric C1 cubic soft limiter with exact ceiling +-0.98f */
float pendulum_soft_limit(float x) {
    const float c = clampf(x, -1.47f, 1.47f);
    return c * (1.0f - 0.15429503f * c * c);
}

/*
 * Unclamped 4-point Hermite cubic fractional delay reader on interleaved `src_ring`.
 * `delay_samp` is guaranteed in [4.0f, SWING_SRC_LEN - 16] by control-rate bounds.
 */
static inline float hermite_read_src_l(const swing_frame_t* restrict ring, uint32_t wr, float delay_samp) {
    const int32_t di = (int32_t)delay_samp;
    const float f = delay_samp - (float)di;
    const uint32_t base = wr - (uint32_t)di;

    const float ym1 = ring[(base + 1u) & SWING_SRC_MASK].l;
    const float y0  = ring[base        & SWING_SRC_MASK].l;
    const float y1  = ring[(base - 1u) & SWING_SRC_MASK].l;
    const float y2  = ring[(base - 2u) & SWING_SRC_MASK].l;

    const float c1 = 0.5f * (y1 - ym1);
    const float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
    return ((c3 * f + c2) * f + c1) * f + y0;
}

static inline float hermite_read_src_r(const swing_frame_t* restrict ring, uint32_t wr, float delay_samp) {
    const int32_t di = (int32_t)delay_samp;
    const float f = delay_samp - (float)di;
    const uint32_t base = wr - (uint32_t)di;

    const float ym1 = ring[(base + 1u) & SWING_SRC_MASK].r;
    const float y0  = ring[base        & SWING_SRC_MASK].r;
    const float y1  = ring[(base - 1u) & SWING_SRC_MASK].r;
    const float y2  = ring[(base - 2u) & SWING_SRC_MASK].r;

    const float c1 = 0.5f * (y1 - ym1);
    const float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
    return ((c3 * f + c2) * f + c1) * f + y0;
}

/* Fast 2-point linear fractional delay readers for diffuse wall reflections & FDN */
static inline float linear_read_src_l(const swing_frame_t* restrict ring, uint32_t wr, float delay_samp) {
    const int32_t di = (int32_t)delay_samp;
    const float f = delay_samp - (float)di;
    const uint32_t base = wr - (uint32_t)di;
    const float y0 = ring[base        & SWING_SRC_MASK].l;
    const float y1 = ring[(base - 1u) & SWING_SRC_MASK].l;
    return y0 + f * (y1 - y0);
}

static inline float linear_read_src_r(const swing_frame_t* restrict ring, uint32_t wr, float delay_samp) {
    const int32_t di = (int32_t)delay_samp;
    const float f = delay_samp - (float)di;
    const uint32_t base = wr - (uint32_t)di;
    const float y0 = ring[base        & SWING_SRC_MASK].r;
    const float y1 = ring[(base - 1u) & SWING_SRC_MASK].r;
    return y0 + f * (y1 - y0);
}

static inline float linear_read_ring(const float* restrict buf, uint32_t wr, uint32_t mask, float delay_samp) {
    const int32_t di = (int32_t)delay_samp;
    const float f = delay_samp - (float)di;
    const uint32_t base = wr - (uint32_t)di;
    const float y0 = buf[base        & mask];
    const float y1 = buf[(base - 1u) & mask];
    return y0 + f * (y1 - y0);
}

static inline swing_frame_t loop_read_hermite_fast(const swing_frame_t* restrict buf,
                                                   int32_t len, int32_t idx, float f) {
    int32_t im1 = idx - 1; if (im1 < 0) im1 += len;
    int32_t ip1 = idx + 1; if (ip1 >= len) ip1 -= len;
    int32_t ip2 = idx + 2; if (ip2 >= len) ip2 -= len;

    const swing_frame_t s_m1 = buf[im1];
    const swing_frame_t s_0  = buf[idx];
    const swing_frame_t s_1  = buf[ip1];
    const swing_frame_t s_2  = buf[ip2];

    const float c1l = 0.5f * (s_1.l - s_m1.l);
    const float c2l = s_m1.l - 2.5f * s_0.l + 2.0f * s_1.l - 0.5f * s_2.l;
    const float c3l = 0.5f * (s_2.l - s_m1.l) + 1.5f * (s_0.l - s_1.l);

    const float c1r = 0.5f * (s_1.r - s_m1.r);
    const float c2r = s_m1.r - 2.5f * s_0.r + 2.0f * s_1.r - 0.5f * s_2.r;
    const float c3r = 0.5f * (s_2.r - s_m1.r) + 1.5f * (s_0.r - s_1.r);

    swing_frame_t res = {
        ((c3l * f + c2l) * f + c1l) * f + s_0.l,
        ((c3r * f + c2r) * f + c1r) * f + s_0.r,
    };
    return res;
}

const char* pendulum_rig_name(uint8_t rig_mode) {
    return kRigs[rig_mode % SWING_RIG_COUNT].name;
}

const char* pendulum_rig_short_name(uint8_t rig_mode) {
    return kRigs[rig_mode % SWING_RIG_COUNT].short_name;
}

static void update_hall_coeffs(pendulum_engine_t* d) {
    const float h = clampf(d->hall_norm, 0.0f, 1.0f);
    const swing_rig_cfg_t* rig = &kRigs[d->rig_mode % SWING_RIG_COUNT];

    d->rig_size_scale = rig->size_scale;
    d->er_gain = h * (0.28f - 0.06f * h);
    d->hall_wet = h * (0.42f + 0.16f * h);

    const float hc = sqrtf(h + 1.0e-12f);
    float fb = 0.36f + 0.53f * hc;
    if (d->rig_mode == SWING_RIG_CHAPEL) fb += 0.02f;
    if (d->rig_mode == SWING_RIG_CLOSE)  fb -= 0.05f;
    d->fdn_fb = clampf(fb, 0.25f, 0.91f);

    const float fc = 4200.0f - 1200.0f * h;
    d->fdn_damp = clampf(SWING_TWO_PI * fc * d->inv_fs, 0.12f, 0.65f);
}

/*
 * Recompute 3D soundboard Cartesian coordinates (x_p, y_p, z_p), Euclidean
 * distances to Mic L, Mic R, and opposite lateral walls (zero external libm calls!).
 */
static void update_acoustic_targets(pendulum_engine_t* d, bool snap_immediate) {
    const swing_rig_cfg_t* rig = &kRigs[d->rig_mode % SWING_RIG_COUNT];
    const float L = d->rope_len;
    const float L_y = L * rig->ly_ratio;

    const float tx = clampf(d->theta_x, -SWING_LIMIT_RAD, SWING_LIMIT_RAD);
    const float ty = clampf(d->theta_y, -0.75f, 0.75f);
    const float sin_tx = sin_halfpi(tx);
    const float cos_tx = cos_halfpi(tx);
    const float sin_ty = sin_halfpi(ty);
    const float cos_ty = cos_halfpi(ty);

    const float xp = L * sin_tx;
    const float yp = L_y * sin_ty;
    const float zp = L * (1.0f - cos_tx) + 0.5f * L_y * (1.0f - cos_ty);

    d->x_p = xp;
    d->y_p = yp;
    d->z_p = zp;
    d->live_deg_x = tx * SWING_RAD_TO_DEG;
    d->live_deg_y = ty * SWING_RAD_TO_DEG;

    const float half_m = 0.5f * rig->d_mic;
    const float dx_l = xp + half_m;
    const float dx_r = xp - half_m;
    const float dy   = yp - rig->y_mic;
    const float dz   = zp - rig->z_mic;

    const float dyz_sq = dy * dy + dz * dz;
    const float r_l = sqrtf(dx_l * dx_l + dyz_sq);
    const float r_r = sqrtf(dx_r * dx_r + dyz_sq);

    const float r_nadir = sqrtf(half_m * half_m + rig->y_mic * rig->y_mic + rig->z_mic * rig->z_mic);
    const float r_floor = 0.75f * fabsf(rig->z_mic);

    const float dn = clampf(d->doppler_norm, 0.0f, 1.0f);
    const float doppler_scale = dn * (1.20f + 1.60f * dn);
    const float samp_per_m = d->sample_rate * (1.0f / SWING_SOUND_SPEED);
    const float max_del = (float)(SWING_SRC_LEN - 16u);

    const float del_l = 12.0f + fmaxf(0.0f, r_l - r_floor) * samp_per_m * doppler_scale;
    const float del_r = 12.0f + fmaxf(0.0f, r_r - r_floor) * samp_per_m * doppler_scale;
    d->del_dir_l_tgt = clampf(del_l, 4.0f, max_del);
    d->del_dir_r_tgt = clampf(del_r, 4.0f, max_del);

    const float two_w = 2.0f * rig->half_wall;
    const float dx_wl = two_w + xp - half_m;
    const float dx_wr = two_w - xp - half_m;
    const float r_wl  = sqrtf(dx_wl * dx_wl + dyz_sq);
    const float r_wr  = sqrtf(dx_wr * dx_wr + dyz_sq);

    const float er_base = clampf((two_w - r_floor) * 88.0f, 360.0f, 2200.0f);
    const float wall_dop = clampf(doppler_scale, 0.0f, 1.35f);
    const float del_wl = er_base + (r_wl - two_w) * samp_per_m * wall_dop;
    const float del_wr = er_base + (r_wr - two_w) * samp_per_m * wall_dop;
    d->del_wall_l_tgt = clampf(del_wl, 120.0f, max_del);
    d->del_wall_r_tgt = clampf(del_wr, 120.0f, max_del);

    const float k_prox = 0.14f + 0.18f * dn;
    const float prox_norm = 1.14f;
    const float prox_l = prox_norm / (1.0f + k_prox * fmaxf(0.0f, r_l - r_nadir));
    const float prox_r = prox_norm / (1.0f + k_prox * fmaxf(0.0f, r_r - r_nadir));

    const float r_center = sqrtf(xp * xp + dyz_sq + 0.04f);
    const float az = clampf(xp / r_center, -1.0f, 1.0f);

    const float card_l = clampf((0.52f + 0.48f * (-0.85f * az + 0.35f)) * 1.453f, 0.22f, 1.42f);
    const float card_r = clampf((0.52f + 0.48f * (+0.85f * az + 0.35f)) * 1.453f, 0.22f, 1.42f);

    d->gain_l_tgt = clampf(prox_l * card_l * 0.84f, 0.08f, 1.35f);
    d->gain_r_tgt = clampf(prox_r * card_r * 0.84f, 0.08f, 1.35f);

    const float dist_l = clampf((r_l - r_nadir) * 0.11f, 0.0f, 1.0f);
    const float dist_r = clampf((r_r - r_nadir) * 0.11f, 0.0f, 1.0f);
    const float shadow_l = clampf(0.5f * (az + 1.0f), 0.0f, 1.0f);
    const float shadow_r = clampf(0.5f * (1.0f - az), 0.0f, 1.0f);

    const float fc_l = 11800.0f - 6200.0f * dist_l - 2800.0f * shadow_l * dist_l;
    const float fc_r = 11800.0f - 6200.0f * dist_r - 2800.0f * shadow_r * dist_r;
    d->lp_a_l_tgt = clampf(SWING_TWO_PI * fc_l * d->inv_fs, 0.08f, 0.88f);
    d->lp_a_r_tgt = clampf(SWING_TWO_PI * fc_r * d->inv_fs, 0.08f, 0.88f);

    if (snap_immediate) {
        d->del_dir_l_s1  = d->del_dir_l  = d->del_dir_l_tgt;
        d->del_dir_r_s1  = d->del_dir_r  = d->del_dir_r_tgt;
        d->del_wall_l_s1 = d->del_wall_l = d->del_wall_l_tgt;
        d->del_wall_r_s1 = d->del_wall_r = d->del_wall_r_tgt;
        d->gain_l = d->gain_l_tgt;
        d->gain_r = d->gain_r_tgt;
        d->lp_a_l = d->lp_a_l_tgt;
        d->lp_a_r = d->lp_a_r_tgt;
    }
}

/*
 * Advance 2D nonlinear gravitational pendulum by `dt` seconds via Velocity-Verlet
 * (zero external libm calls!).
 */
static void step_pendulum_verlet(pendulum_engine_t* d, float dt) {
    if (dt <= 0.0f || dt > 0.05f) return;

    d->rope_len += clampf(4.0f * dt, 0.0f, 1.0f) * (d->rope_len_target - d->rope_len);
    const float L = clampf(d->rope_len, 2.0f, 15.0f);
    const swing_rig_cfg_t* rig = &kRigs[d->rig_mode % SWING_RIG_COUNT];
    const float L_y = fmaxf(1.8f, L * rig->ly_ratio);

    const float g_L  = SWING_GRAVITY / L;
    const float g_Ly = SWING_GRAVITY / L_y;
    const float omega0 = sqrtf(g_L);

    float tx = clampf(d->theta_x, -SWING_LIMIT_RAD, SWING_LIMIT_RAD);
    float ox = d->omega_x;
    float ty = clampf(d->theta_y, -0.75f, 0.75f);
    float oy = d->omega_y;

    const float sin_tx0 = sin_halfpi(tx);
    const float sin_ty0 = sin_halfpi(ty);

    float drive_x = 0.0f;
    float drive_y = 0.0f;
    const float damp = SWING_COAST_DAMP;

    if (d->swing_on) {
        const float th_max = clampf(d->theta_max, 0.0f, SWING_LIMIT_RAD);
        if (th_max > 0.003f) {
            const float e_cur = 0.5f * ox * ox + g_L * (1.0f - cos_halfpi(tx));
            const float e_tgt = g_L * (1.0f - cos_halfpi(th_max));

            if (e_cur < 1.0e-5f * g_L) {
                ox += 0.28f * omega0 * th_max;
            }

            const float k_esc = clampf(damp + 1.85f * (e_tgt - e_cur) / (e_tgt + 0.04f),
                                       -1.20f, 1.85f);
            drive_x = k_esc * ox;

            const float ty_max = clampf(th_max * rig->y_sway_ratio, 0.0f, 0.75f);
            const float oy_ref = -ty_max * omega0 * sin_tx0 / fmaxf(0.12f, sin_halfpi(th_max));
            const float ey_cur = 0.5f * oy * oy + g_Ly * (1.0f - cos_halfpi(ty));
            const float ey_tgt = g_Ly * (1.0f - cos_halfpi(ty_max));
            const float ky_esc = clampf(damp + 1.6f * (ey_tgt - ey_cur) / (ey_tgt + 0.02f),
                                        -1.2f, 1.6f);
            drive_y = ky_esc * oy + 0.35f * (oy_ref - oy);
        } else {
            drive_x = -1.10f * ox;
            drive_y = -1.10f * oy;
        }
    }

    const float ax0 = -g_L * sin_tx0 - damp * ox + drive_x;
    const float ox_half = ox + 0.5f * dt * ax0;
    float tx_next = clampf(tx + dt * ox_half, -SWING_LIMIT_RAD, SWING_LIMIT_RAD);
    const float ax1 = -g_L * sin_halfpi(tx_next) - damp * ox_half + drive_x;
    float ox_next = ox_half + 0.5f * dt * ax1;
    if (fabsf(tx_next) >= SWING_LIMIT_RAD && (tx_next * ox_next) > 0.0f) {
        ox_next = 0.0f;
    }

    const float ay0 = -g_Ly * sin_ty0 - damp * oy + drive_y;
    const float oy_half = oy + 0.5f * dt * ay0;
    float ty_next = clampf(ty + dt * oy_half, -0.75f, 0.75f);
    const float ay1 = -g_Ly * sin_halfpi(ty_next) - damp * oy_half + drive_y;
    float oy_next = oy_half + 0.5f * dt * ay1;

    d->theta_x = tx_next;
    d->omega_x = ox_next;
    d->theta_y = ty_next;
    d->omega_y = oy_next;

    update_acoustic_targets(d, false);
}

void pendulum_init(pendulum_engine_t* d, float sample_rate) {
    if (!d) return;
    memset(d, 0, sizeof(*d));

    d->sample_rate = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    d->inv_fs = 1.0f / d->sample_rate;

    d->dc_r = clampf(1.0f - (SWING_TWO_PI * 18.0f * d->inv_fs), 0.990f, 0.9995f);
    d->mod_inc_a = (uint32_t)(0.31f * d->inv_fs * 4294967296.0f + 0.5f);
    d->mod_inc_b = (uint32_t)(0.47f * d->inv_fs * 4294967296.0f + 0.5f);

    d->rig_mode = SWING_RIG_BARBICAN;
    d->swing_on = true;

    pendulum_set_rope(d, 0.45f);
    d->rope_len = d->rope_len_target;
    pendulum_set_arc(d, 0.55f);
    pendulum_set_doppler(d, 0.50f);
    pendulum_set_hall(d, 0.42f);

    d->theta_x = -d->theta_max;
    d->omega_x = 0.0f;
    d->theta_y = 0.0f;
    d->omega_y = sqrtf(SWING_GRAVITY / d->rope_len) * d->theta_max *
                 kRigs[d->rig_mode].y_sway_ratio;
    d->phys_rem = SWING_PHYS_SUB;

    update_acoustic_targets(d, true);
}

void pendulum_attach_space(pendulum_engine_t* d, swing_space_t* space) {
    if (!d) return;
    d->space = space;
    pendulum_clear_audio(d);
}

void pendulum_clear_audio(pendulum_engine_t* d) {
    if (!d) return;
    if (d->space) {
        memset(d->space, 0, sizeof(*d->space));
    }
    d->src_wr = 0u;
    d->fdn0_wr = 0u;
    d->fdn_wr = 0u;
    d->ap_wr = 0u;
    for (uint32_t k = 0u; k < 4u; ++k) d->fdn_lp[k] = 0.0f;
    d->air_lp_l = 0.0f;
    d->air_lp_r = 0.0f;
    d->wall_lp_l = 0.0f;
    d->wall_lp_r = 0.0f;
    d->dc_x_l = 0.0f;
    d->dc_y_l = 0.0f;
    d->dc_x_r = 0.0f;
    d->dc_y_r = 0.0f;
    d->prox_env_l = 0.0f;
    d->prox_env_r = 0.0f;
    d->hall_env = 0.0f;
    d->phys_rem = SWING_PHYS_SUB;
    update_acoustic_targets(d, true);
}

void pendulum_set_rope(pendulum_engine_t* d, float norm) {
    if (!d) return;
    d->rope_norm = clampf(norm, 0.0f, 1.0f);
    d->rope_len_target = 2.20f + 11.80f * d->rope_norm;
    if (d->rope_len < 1.0f) d->rope_len = d->rope_len_target;
}

void pendulum_set_arc(pendulum_engine_t* d, float norm) {
    if (!d) return;
    d->arc_norm = clampf(norm, 0.0f, 1.0f);
    d->theta_max = d->arc_norm * (SWING_MAX_ARC_DEG * SWING_DEG_TO_RAD);
}

void pendulum_set_doppler(pendulum_engine_t* d, float norm) {
    if (!d) return;
    d->doppler_norm = clampf(norm, 0.0f, 1.0f);
    update_acoustic_targets(d, false);
}

void pendulum_set_hall(pendulum_engine_t* d, float norm) {
    if (!d) return;
    d->hall_norm = clampf(norm, 0.0f, 1.0f);
    update_hall_coeffs(d);
}

void pendulum_set_rig(pendulum_engine_t* d, uint8_t rig_mode) {
    if (!d) return;
    d->rig_mode = (uint8_t)(rig_mode % SWING_RIG_COUNT);
    update_hall_coeffs(d);
    update_acoustic_targets(d, false);
}

void pendulum_set_swing_on(pendulum_engine_t* d, bool swing_on) {
    if (!d) return;
    const bool was_off = !d->swing_on;
    d->swing_on = swing_on;
    if (swing_on && was_off && d->theta_max > 0.02f) {
        const float e = pendulum_get_energy(d);
        const float g_L = SWING_GRAVITY / fmaxf(2.0f, d->rope_len);
        if (e < 0.02f * g_L * (1.0f - cos_halfpi(d->theta_max))) {
            pendulum_push(d, 0.45f);
        }
    }
}

void pendulum_push(pendulum_engine_t* d, float impulse_norm) {
    if (!d) return;
    const float imp = clampf(impulse_norm, 0.15f, 1.0f);
    const float L = fmaxf(2.0f, d->rope_len);
    const float omega0 = sqrtf(SWING_GRAVITY / L);
    const float ref_ang = clampf(fmaxf(d->theta_max, 38.0f * SWING_DEG_TO_RAD), 0.0f, SWING_LIMIT_RAD);
    const float v_kick = imp * omega0 * (1.15f * sin_halfpi(0.5f * ref_ang) * 2.0f);

    const float dir = (fabsf(d->omega_x) > 0.01f)
                          ? ((d->omega_x >= 0.0f) ? 1.0f : -1.0f)
                          : ((d->theta_x <= 0.0f) ? 1.0f : -1.0f);

    const float tx = clampf(d->theta_x, -SWING_LIMIT_RAD, SWING_LIMIT_RAD);
    float new_ox = d->omega_x + dir * v_kick;
    const float max_e = (SWING_GRAVITY / L) * (1.0f - cos_halfpi(58.0f * SWING_DEG_TO_RAD));
    const float pot_e = (SWING_GRAVITY / L) * (1.0f - cos_halfpi(tx));
    if (pot_e < max_e) {
        const float max_ox = sqrtf(2.0f * (max_e - pot_e));
        new_ox = clampf(new_ox, -max_ox, max_ox);
    } else {
        new_ox = 0.0f;
    }
    d->omega_x = new_ox;

    const swing_rig_cfg_t* rig = &kRigs[d->rig_mode % SWING_RIG_COUNT];
    d->omega_y = clampf(d->omega_y + dir * 0.4f * v_kick * rig->y_sway_ratio, -1.2f, 1.2f);
    update_acoustic_targets(d, false);
}

void pendulum_center_still(pendulum_engine_t* d) {
    if (!d) return;
    d->theta_x = 0.0f;
    d->omega_x = 0.0f;
    d->theta_y = 0.0f;
    d->omega_y = 0.0f;
    d->live_cents = 0.0f;
    update_acoustic_targets(d, false);
}

float pendulum_get_energy(const pendulum_engine_t* d) {
    if (!d) return 0.0f;
    const float L = fmaxf(2.0f, d->rope_len);
    const float tx = clampf(d->theta_x, -SWING_LIMIT_RAD, SWING_LIMIT_RAD);
    return 0.5f * d->omega_x * d->omega_x + (SWING_GRAVITY / L) * (1.0f - cos_halfpi(tx));
}

void pendulum_step_idle(pendulum_engine_t* d, float dt_sec) {
    if (!d) return;
    const float step = clampf(dt_sec, 0.0f, 0.05f);
    const float sub_dt = 0.5f * step;
    step_pendulum_verlet(d, sub_dt);
    step_pendulum_verlet(d, sub_dt);
    d->live_cents *= 0.85f;
}

static inline float schroeder_allpass(float* restrict buf, uint32_t wr, uint32_t mask,
                                      uint32_t del, float g, float x) {
    const float delayed = buf[(wr - del) & mask];
    const float v = x + g * delayed;
    buf[wr] = v;
    return delayed - g * v;
}

/*
 * Fast block-fused 48 kHz swinging-piano + concert-hall FDN processor.
 * Zero external libm calls, zero float divisions, zero denormal-branch stalls!
 */
void pendulum_process_block(pendulum_engine_t* restrict d,
                            const swing_frame_t* restrict loop_buf,
                            uint32_t loop_len,
                            volatile int32_t* restrict io_head_i,
                            volatile float* restrict io_head_f,
                            float speed_target,
                            float* restrict io_speed_cur,
                            float play_target,
                            float play_step,
                            float* restrict io_play_gain,
                            float* restrict out_bus,
                            uint32_t frames) {
    if (!d || !out_bus || frames == 0u) return;

    if (!loop_buf || loop_len < 8u) {
        memset(out_bus, 0, (size_t)frames * 2u * sizeof(float));
        return;
    }

    const int32_t size = (int32_t)loop_len;
    int32_t head_i = *io_head_i;
    if (head_i < 0 || head_i >= size) head_i = 0;
    float head_f = *io_head_f;
    float speed_cur = *io_speed_cur;
    float pg = *io_play_gain;

    swing_space_t* restrict sp = d->space;
    uint32_t src_wr  = d->src_wr;
    uint32_t ap_wr   = d->ap_wr;
    uint32_t fdn0_wr = d->fdn0_wr;
    uint32_t fdn_wr  = d->fdn_wr;
    uint32_t mod_a   = d->mod_ph_a;
    uint32_t mod_b   = d->mod_ph_b;
    const uint32_t inc_a = d->mod_inc_a;
    const uint32_t inc_b = d->mod_inc_b;

    const float ds_a     = 0.0020f;
    const float dc_r     = d->dc_r;
    const float er_gain  = d->er_gain;
    const float hall_wet = d->hall_wet;
    const float fdn_fb   = d->fdn_fb;
    const float fdn_damp = d->fdn_damp;
    const float sc       = d->rig_size_scale;
    const uint32_t tap0  = (uint32_t)(997.0f * sc);
    const uint32_t tap1  = (uint32_t)(1493.0f * sc);
    const float tap2_b   = 1789.0f * sc;
    const float tap3_b   = 2017.0f * sc;
    const bool do_er     = (sp != NULL) && (er_gain > 0.0005f);
    const bool do_hall   = (sp != NULL) && (hall_wet > 0.0005f);

    uint32_t rem = frames;
    float* restrict dst = out_bus;
    float d_del_last = 0.0f;

    while (rem > 0u) {
        if (d->phys_rem == 0u) {
            step_pendulum_verlet(d, (float)SWING_PHYS_SUB * d->inv_fs);
            d->phys_rem = SWING_PHYS_SUB;
        }
        const uint32_t chunk = (rem < d->phys_rem) ? rem : d->phys_rem;
        d->phys_rem = (uint8_t)(d->phys_rem - chunk);
        rem -= chunk;

        for (uint32_t n = 0u; n < chunk; ++n) {
            speed_cur += 0.002f * (speed_target - speed_cur);
            if (pg < play_target) {
                pg += play_step;
                if (pg > play_target) pg = play_target;
            } else if (pg > play_target) {
                pg -= play_step;
                if (pg < play_target) pg = play_target;
            }

            swing_frame_t dry = { 0.0f, 0.0f };
            if (pg > 0.0001f) {
                dry = loop_read_hermite_fast(loop_buf, size, head_i, head_f);
                dry.l *= pg;
                dry.r *= pg;

                float nf = head_f + speed_cur;
                const int32_t is = (int32_t)floorf(nf);
                nf -= (float)is;
                int32_t next_i = head_i + is;
                if (next_i >= size) {
                    next_i %= size;
                } else if (next_i < 0) {
                    next_i = size - 1 - ((-1 - next_i) % size);
                }
                head_i = next_i;
                head_f = nf;
            }

            const float prev_dl = d->del_dir_l;
            const float prev_dr = d->del_dir_r;

            d->del_dir_l_s1 += ds_a * (d->del_dir_l_tgt - d->del_dir_l_s1);
            d->del_dir_l    += ds_a * (d->del_dir_l_s1  - d->del_dir_l);
            d->del_dir_r_s1 += ds_a * (d->del_dir_r_tgt - d->del_dir_r_s1);
            d->del_dir_r    += ds_a * (d->del_dir_r_s1  - d->del_dir_r);

            d->del_wall_l_s1 += ds_a * (d->del_wall_l_tgt - d->del_wall_l_s1);
            d->del_wall_l    += ds_a * (d->del_wall_l_s1  - d->del_wall_l);
            d->del_wall_r_s1 += ds_a * (d->del_wall_r_tgt - d->del_wall_r_s1);
            d->del_wall_r    += ds_a * (d->del_wall_r_s1  - d->del_wall_r);

            d->gain_l += 0.0025f * (d->gain_l_tgt - d->gain_l);
            d->gain_r += 0.0025f * (d->gain_r_tgt - d->gain_r);
            d->lp_a_l += 0.0025f * (d->lp_a_l_tgt - d->lp_a_l);
            d->lp_a_r += 0.0025f * (d->lp_a_r_tgt - d->lp_a_r);

            d_del_last = 0.5f * ((d->del_dir_l - prev_dl) + (d->del_dir_r - prev_dr));

            const float anti_dn = ((n & 1u) != 0u) ? 1.0e-20f : -1.0e-20f;
            const float sb_l = 0.80f * dry.l + 0.20f * dry.r + anti_dn;
            const float sb_r = 0.20f * dry.l + 0.80f * dry.r - anti_dn;

            float dir_l = sb_l;
            float dir_r = sb_r;
            float er_l  = 0.0f;
            float er_r  = 0.0f;

            if (sp) {
                sp->src_ring[src_wr].l = sb_l;
                sp->src_ring[src_wr].r = sb_r;

                dir_l = hermite_read_src_l(sp->src_ring, src_wr, d->del_dir_l);
                dir_r = hermite_read_src_r(sp->src_ring, src_wr, d->del_dir_r);

                if (do_er) {
                    const float wl_raw = linear_read_src_l(sp->src_ring, src_wr, d->del_wall_l);
                    const float wr_raw = linear_read_src_r(sp->src_ring, src_wr, d->del_wall_r);
                    d->wall_lp_l += 0.34f * (wl_raw - d->wall_lp_l);
                    d->wall_lp_r += 0.34f * (wr_raw - d->wall_lp_r);
                    er_l = er_gain * (0.82f * d->wall_lp_l + 0.28f * d->wall_lp_r);
                    er_r = er_gain * (0.82f * d->wall_lp_r + 0.28f * d->wall_lp_l);
                }

                src_wr = (src_wr + 1u) & SWING_SRC_MASK;
            }

            d->air_lp_l += d->lp_a_l * (dir_l - d->air_lp_l);
            d->air_lp_r += d->lp_a_r * (dir_r - d->air_lp_r);

            const float mic_l = d->air_lp_l * d->gain_l;
            const float mic_r = d->air_lp_r * d->gain_r;

            float rev_l = 0.0f;
            float rev_r = 0.0f;
            if (do_hall) {
                const float diff_in_l = 0.45f * (mic_l + er_l);
                const float diff_in_r = 0.45f * (mic_r + er_r);

                const float diff_l = schroeder_allpass(sp->ap_l, ap_wr, SWING_AP_MASK, 173u, 0.58f, diff_in_l);
                const float diff_r = schroeder_allpass(sp->ap_r, ap_wr, SWING_AP_MASK, 229u, 0.58f, diff_in_r);
                ap_wr = (ap_wr + 1u) & SWING_AP_MASK;

                mod_a += inc_a;
                mod_b += inc_b;
                const float lfo_a = 3.2f * fast_sin_u32(mod_a);
                const float lfo_b = 3.2f * fast_sin_u32(mod_b);

                const float r0 = sp->fdn0[(fdn0_wr - tap0) & SWING_FDN0_MASK];
                const float r1 = sp->fdn1[(fdn_wr  - tap1) & SWING_FDN_MASK];
                const float r2 = linear_read_ring(sp->fdn2, fdn_wr, SWING_FDN_MASK, tap2_b + lfo_a);
                const float r3 = linear_read_ring(sp->fdn3, fdn_wr, SWING_FDN_MASK, tap3_b + lfo_b);

                d->fdn_lp[0] += fdn_damp * (r0 - d->fdn_lp[0]);
                d->fdn_lp[1] += fdn_damp * (r1 - d->fdn_lp[1]);
                d->fdn_lp[2] += fdn_damp * (r2 - d->fdn_lp[2]);
                d->fdn_lp[3] += fdn_damp * (r3 - d->fdn_lp[3]);

                const float u0 = d->fdn_lp[0] * fdn_fb;
                const float u1 = d->fdn_lp[1] * fdn_fb;
                const float u2 = d->fdn_lp[2] * fdn_fb;
                const float u3 = d->fdn_lp[3] * fdn_fb;

                const float hsum = 0.5f * (u0 + u1 + u2 + u3);
                sp->fdn0[fdn0_wr] = (u0 - hsum) + diff_l;
                sp->fdn1[fdn_wr]  = (u1 - hsum) + diff_r;
                sp->fdn2[fdn_wr]  = (u2 - hsum) + 0.7f * (diff_l - diff_r);
                sp->fdn3[fdn_wr]  = (u3 - hsum) + 0.7f * (diff_r + diff_l);

                fdn0_wr = (fdn0_wr + 1u) & SWING_FDN0_MASK;
                fdn_wr  = (fdn_wr  + 1u) & SWING_FDN_MASK;

                rev_l = hall_wet * 0.52f * (d->fdn_lp[0] + d->fdn_lp[2] - 0.45f * d->fdn_lp[1]);
                rev_r = hall_wet * 0.52f * (d->fdn_lp[1] + d->fdn_lp[3] - 0.45f * d->fdn_lp[2]);
            }

            const float sum_l = mic_l + er_l + rev_l;
            const float sum_r = mic_r + er_r + rev_r;

            const float al = fabsf(mic_l);
            const float ar = fabsf(mic_r);
            const float ah = 0.5f * (fabsf(er_l + rev_l) + fabsf(er_r + rev_r));
            d->prox_env_l = (al > d->prox_env_l) ? (d->prox_env_l + 0.02f * (al - d->prox_env_l))
                                                 : (d->prox_env_l * 0.9992f);
            d->prox_env_r = (ar > d->prox_env_r) ? (d->prox_env_r + 0.02f * (ar - d->prox_env_r))
                                                 : (d->prox_env_r * 0.9992f);
            d->hall_env   = (ah > d->hall_env)   ? (d->hall_env   + 0.01f * (ah - d->hall_env))
                                                 : (d->hall_env   * 0.9995f);

            const float y_dc_l = sum_l - d->dc_x_l + dc_r * d->dc_y_l;
            d->dc_x_l = sum_l;
            d->dc_y_l = y_dc_l;

            const float y_dc_r = sum_r - d->dc_x_r + dc_r * d->dc_y_r;
            d->dc_x_r = sum_r;
            d->dc_y_r = y_dc_r;

            dst[0] = pendulum_soft_limit(y_dc_l);
            dst[1] = pendulum_soft_limit(y_dc_r);
            dst += 2;
        }

        const float inst_cents = clampf(-d_del_last * 1731.234f, -199.0f, 199.0f);
        d->live_cents += 0.08f * (inst_cents - d->live_cents);
    }

    d->src_wr   = src_wr;
    d->ap_wr    = ap_wr;
    d->fdn0_wr  = fdn0_wr;
    d->fdn_wr   = fdn_wr;
    d->mod_ph_a = mod_a;
    d->mod_ph_b = mod_b;
    *io_head_i    = head_i;
    *io_head_f    = head_f;
    *io_speed_cur = speed_cur;
    *io_play_gain = pg;
}

swing_frame_t pendulum_process_frame(pendulum_engine_t* d, swing_frame_t dry) {
    swing_frame_t loop_one[8] = {
        dry, dry, dry, dry, dry, dry, dry, dry,
    };
    volatile int32_t hi = 0;
    volatile float hf = 0.0f;
    float sp_cur = 0.0f;
    float pg = 1.0f;
    float out2[2] = { 0.0f, 0.0f };
    pendulum_process_block(d, loop_one, 8u, &hi, &hf, 0.0f, &sp_cur,
                           1.0f, 0.0f, &pg, out2, 1u);
    swing_frame_t res = { out2[0], out2[1] };
    return res;
}

/* ----------------------------------------------------------------------------
 * Built-In "Seed Felt Piano" Generator (chunked & fast aisatsana [102] motif)
 * ------------------------------------------------------------------------- */
typedef struct {
    float beat;     /* Note onset in beats [0 .. 16) */
    uint8_t midi;   /* MIDI pitch (60 = C4)          */
    float vel;      /* Soft felt dynamic [0.2 .. 0.8] */
    float dur_b;    /* Ring duration in beats         */
} felt_note_t;

static const felt_note_t kAisatsanaNotes[PEND_FELT_NOTES_COUNT] = {
    /* Bar 1: Cmaj9 */
    {  0.00f, 48u, 0.56f, 5.2f },   /* C3  */
    {  0.04f, 55u, 0.44f, 4.8f },   /* G3  */
    {  0.82f, 64u, 0.52f, 4.2f },   /* E4  */
    {  1.66f, 62u, 0.46f, 3.8f },   /* D4  */
    {  2.50f, 59u, 0.42f, 3.6f },   /* B3  */
    {  3.28f, 60u, 0.48f, 3.4f },   /* C4  */

    /* Bar 2: Am11 */
    {  4.00f, 45u, 0.55f, 5.2f },   /* A2  */
    {  4.05f, 52u, 0.43f, 4.6f },   /* E3  */
    {  4.84f, 60u, 0.50f, 4.0f },   /* C4  */
    {  5.68f, 71u, 0.42f, 3.8f },   /* B4  */
    {  6.52f, 67u, 0.46f, 3.6f },   /* G4  */
    {  7.30f, 64u, 0.44f, 3.4f },   /* E4  */

    /* Bar 3: Fmaj9(#11) */
    {  8.00f, 41u, 0.54f, 5.2f },   /* F2  */
    {  8.05f, 48u, 0.44f, 4.6f },   /* C3  */
    {  8.84f, 57u, 0.50f, 4.2f },   /* A3  */
    {  9.68f, 67u, 0.45f, 3.8f },   /* G4  */
    { 10.50f, 64u, 0.48f, 3.6f },   /* E4  */
    { 11.30f, 60u, 0.42f, 3.4f },   /* C4  */

    /* Bar 4: G6sus4(add9) -> resolves into Bar 1 Cmaj9 */
    { 12.00f, 43u, 0.54f, 5.2f },   /* G2  */
    { 12.04f, 50u, 0.43f, 4.6f },   /* D3  */
    { 12.85f, 57u, 0.47f, 4.0f },   /* A3  */
    { 13.68f, 65u, 0.46f, 3.6f },   /* F4  */
    { 14.52f, 64u, 0.44f, 3.4f },   /* E4  */
    { 15.28f, 62u, 0.42f, 3.4f },   /* D4  */
};

static float midi_to_hz(uint8_t midi) {
    const float semitones = ((float)midi - 69.0f) * (1.0f / 12.0f);
    return 440.0f * expf(semitones * 0.69314718f);
}

uint32_t pendulum_seed_felt_begin(swing_frame_t* buf, uint32_t max_frames, float sample_rate) {
    if (!buf || max_frames < 1024u) return 0u;
    const float fs = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;

    uint32_t target_len = (uint32_t)(9.60f * fs);
    if (target_len > max_frames) target_len = max_frames;
    target_len &= ~15u;
    if (target_len < 1024u) return 0u;

    memset(buf, 0, (size_t)target_len * sizeof(swing_frame_t));
    return target_len;
}

void pendulum_seed_felt_notes(swing_frame_t* buf, uint32_t target_len, float sample_rate,
                              uint32_t note_start, uint32_t note_count) {
    if (!buf || target_len < 1024u) return;
    const float fs = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    const float inv_fs = 1.0f / fs;

    const float frames_per_beat = (float)target_len * (1.0f / 16.0f);
    const uint32_t attack_fr = (uint32_t)(0.0042f * fs) + 4u;
    const uint32_t rel_fr    = (uint32_t)(0.0850f * fs) + 8u;
    const float inv_atk = 1.0f / (float)attack_fr;
    const float inv_rel = 1.0f / (float)rel_fr;

    /* 4 warm wool-felt partials (partials >= 5 are damped out by felt blanket) */
    static const float kPartAmp[4] = { 1.00f, 0.42f, 0.17f, 0.075f };

    uint32_t end_note = note_start + note_count;
    if (end_note > PEND_FELT_NOTES_COUNT) end_note = PEND_FELT_NOTES_COUNT;

    for (uint32_t ni = note_start; ni < end_note; ++ni) {
        const felt_note_t nt = kAisatsanaNotes[ni];
        const float f0 = midi_to_hz(nt.midi);
        const uint32_t onset = ((uint32_t)(nt.beat * frames_per_beat)) % target_len;
        uint32_t dur = (uint32_t)(nt.dur_b * frames_per_beat);
        if (dur > target_len) dur = target_len;
        if (dur < attack_fr + rel_fr + 16u) continue;

        const float key_pos = clampf(((float)nt.midi - 40.0f) * (1.0f / 36.0f), 0.0f, 1.0f);
        const float pan_l = 0.72f - 0.28f * key_pos;
        const float pan_r = 0.44f + 0.28f * key_pos;

        float c_re[4], c_im[4], st_re[4], st_im[4];
        const float inharm_b = 0.00028f;

        for (uint32_t p = 0u; p < 4u; ++p) {
            const float k = (float)(p + 1u);
            const float fk = f0 * k * sqrtf(1.0f + inharm_b * k * k);
            if (fk > 0.42f * fs) {
                st_re[p] = 0.0f; st_im[p] = 0.0f;
                c_re[p]  = 1.0f; c_im[p]  = 0.0f;
                continue;
            }
            const float tau = 2.90f / (1.0f + 0.38f * (k - 1.0f) * (k - 1.0f) + 0.0015f * f0);
            const float r_dec = expf(-inv_fs / tau);
            const float w = SWING_TWO_PI * fk * inv_fs;
            c_re[p] = r_dec * cosf(w);
            c_im[p] = r_dec * sinf(w);
            st_re[p] = 0.0f;
            st_im[p] = kPartAmp[p] * nt.vel * 0.27f;
        }

        const float f_uni = f0 * 1.000318f;
        const float r_uni = expf(-inv_fs / 2.75f);
        const float w_uni = SWING_TWO_PI * f_uni * inv_fs;
        const float b_re  = r_uni * cosf(w_uni);
        const float b_im  = r_uni * sinf(w_uni);
        float bst_re = 0.0f;
        float bst_im = 0.34f * nt.vel * 0.27f;

        const float r_th = expf(-inv_fs / 0.018f);
        const float w_th = SWING_TWO_PI * 148.0f * inv_fs;
        const float th_c_re = r_th * cosf(w_th);
        const float th_c_im = r_th * sinf(w_th);
        float th_re = 0.0f;
        float th_im = 0.045f * nt.vel;

        const uint32_t thump_end = (uint32_t)(0.12f * fs);
        uint32_t idx = onset;
        const uint32_t rel_start = dur - rel_fr;

        for (uint32_t n = 0u; n < dur; ++n) {
            const float s0 = st_im[0];
            const float nr0 = st_re[0] * c_re[0] - s0 * c_im[0];
            st_im[0] = st_re[0] * c_im[0] + s0 * c_re[0];
            st_re[0] = nr0;

            const float s1 = st_im[1];
            const float nr1 = st_re[1] * c_re[1] - s1 * c_im[1];
            st_im[1] = st_re[1] * c_im[1] + s1 * c_re[1];
            st_re[1] = nr1;

            const float s2 = st_im[2];
            const float nr2 = st_re[2] * c_re[2] - s2 * c_im[2];
            st_im[2] = st_re[2] * c_im[2] + s2 * c_re[2];
            st_re[2] = nr2;

            const float s3 = st_im[3];
            const float nr3 = st_re[3] * c_re[3] - s3 * c_im[3];
            st_im[3] = st_re[3] * c_im[3] + s3 * c_re[3];
            st_re[3] = nr3;

            const float s_uni = bst_im;
            const float nrb = bst_re * b_re - s_uni * b_im;
            bst_im = bst_re * b_im + s_uni * b_re;
            bst_re = nrb;

            float s_th = 0.0f;
            if (n < thump_end) {
                s_th = th_im;
                const float nrt = th_re * th_c_re - s_th * th_c_im;
                th_im = th_re * th_c_im + s_th * th_c_re;
                th_re = nrt;
            }

            const float s_main = s0 + s1 + s2 + s3 + s_th;

            float env = 1.0f;
            if (n < attack_fr) {
                const float t = (float)n * inv_atk;
                env = t * t * (3.0f - 2.0f * t);
            } else if (n >= rel_start) {
                const float t = (float)(dur - n) * inv_rel;
                env = t * t * (3.0f - 2.0f * t);
            }

            buf[idx].l += env * (s_main * pan_l + s_uni * pan_r);
            buf[idx].r += env * (s_main * pan_r + s_uni * pan_l);

            idx++;
            if (idx >= target_len) idx = 0u;
        }
    }
}

void pendulum_seed_felt_finish(swing_frame_t* buf, uint32_t target_len, float sample_rate) {
    if (!buf || target_len < 16u) return;
    const float fs = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    const float inv_fs = 1.0f / fs;

    float lp_l = buf[target_len - 1u].l;
    float lp_r = buf[target_len - 1u].r;
    const float felt_a = clampf(SWING_TWO_PI * 3600.0f * inv_fs, 0.1f, 0.7f);
    for (uint32_t i = 0u; i < target_len; ++i) {
        lp_l += felt_a * (buf[i].l - lp_l);
        lp_r += felt_a * (buf[i].r - lp_r);
        buf[i].l = pendulum_soft_limit(lp_l * 1.15f);
        buf[i].r = pendulum_soft_limit(lp_r * 1.15f);
    }
}

uint32_t pendulum_seed_felt_piano(swing_frame_t* buf, uint32_t max_frames, float sample_rate) {
    const uint32_t target_len = pendulum_seed_felt_begin(buf, max_frames, sample_rate);
    if (target_len == 0u) return 0u;
    pendulum_seed_felt_notes(buf, target_len, sample_rate, 0u, PEND_FELT_NOTES_COUNT);
    pendulum_seed_felt_finish(buf, target_len, sample_rate);
    return target_len;
}
