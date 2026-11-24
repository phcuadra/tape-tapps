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
#define SWING_LIMIT_RAD     1.08210414f             /* 62 deg safety ceiling */
#define SWING_COAST_DAMP    0.075f                  /* natural free-pendulum air/pivot damping */
#define SWING_DENORM_FLOOR  1.0e-18f
#define SWING_PHYS_SUB      8u                      /* update Verlet ODE every 8 frames (6 kHz) */

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

static inline float flush_denorm(float x) {
    return (fabsf(x) < SWING_DENORM_FLOOR) ? 0.0f : x;
}

float pendulum_clip_guard(float x) {
    if (!(x > -4.0f && x < 4.0f)) return 0.0f;
    return clampf(x, -1.25f, 1.25f);
}

/*
 * Odd-symmetric C1 Pade rational tanh soft limiter with exact ceiling +-0.98f.
 * At |s| = 3.0f, s*(27 + s^2)/(27 + 9*s^2) == +-1.0f and d/ds == 0.0f.
 */
float pendulum_soft_limit(float x) {
    if (!(x > -8.0f && x < 8.0f)) {
        return (x >= 0.0f) ? 0.98f : -0.98f;
    }
    const float ceiling = 0.98f;
    const float s = x * (1.0f / 0.98f);
    if (s >= 3.0f) return ceiling;
    if (s <= -3.0f) return -ceiling;
    const float s2 = s * s;
    return ceiling * (s * (27.0f + s2)) / (27.0f + 9.0f * s2);
}

/*
 * 4-point, 3rd-order Catmull-Rom / Hermite cubic fractional delay reader.
 * Assumes `wr` is the index most recently written; `delay_samp` in [2.0f .. mask - 4.0f].
 */
static inline float hermite_read_ring(const float* buf, uint32_t wr, uint32_t mask, float delay_samp) {
    const float d = clampf(delay_samp, 2.0f, (float)(mask - 4u));
    const int32_t di = (int32_t)floorf(d);
    const float f = d - (float)di;

    const uint32_t i_m1 = (wr - (uint32_t)di + 1u) & mask;
    const uint32_t i_0  = (wr - (uint32_t)di)      & mask;
    const uint32_t i_1  = (wr - (uint32_t)di - 1u) & mask;
    const uint32_t i_2  = (wr - (uint32_t)di - 2u) & mask;

    const float ym1 = buf[i_m1];
    const float y0  = buf[i_0];
    const float y1  = buf[i_1];
    const float y2  = buf[i_2];

    const float c0 = y0;
    const float c1 = 0.5f * (y1 - ym1);
    const float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);

    return ((c3 * f + c2) * f + c1) * f + c0;
}

static inline float linear_read_ring(const float* buf, uint32_t wr, uint32_t mask, float delay_samp) {
    const float d = clampf(delay_samp, 1.0f, (float)(mask - 2u));
    const int32_t di = (int32_t)floorf(d);
    const float f = d - (float)di;
    const float y0 = buf[(wr - (uint32_t)di) & mask];
    const float y1 = buf[(wr - (uint32_t)di - 1u) & mask];
    return y0 + f * (y1 - y0);
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

    const float hc = sqrtf(h);
    float fb = 0.36f + 0.53f * hc;
    if (d->rig_mode == SWING_RIG_CHAPEL) fb += 0.02f;
    if (d->rig_mode == SWING_RIG_CLOSE)  fb -= 0.05f;
    d->fdn_fb = clampf(fb, 0.25f, 0.91f);

    /* Warm felt-hall HF damping (~3.4 kHz at 48 kHz) */
    const float fc = 4200.0f - 1200.0f * h;
    d->fdn_damp = clampf(SWING_TWO_PI * fc * d->inv_fs, 0.12f, 0.65f);
}

/*
 * Recompute 3D soundboard Cartesian coordinates (x_p, y_p, z_p), Euclidean
 * distances to Mic L, Mic R, and opposite lateral walls, plus inverse-distance
 * proximity swell, ORTF directional weighting, and off-axis air-absorption LP.
 */
static void update_acoustic_targets(pendulum_engine_t* d, bool snap_immediate) {
    const swing_rig_cfg_t* rig = &kRigs[d->rig_mode % SWING_RIG_COUNT];
    const float L = d->rope_len;
    const float L_y = L * rig->ly_ratio;

    const float sin_tx = sinf(d->theta_x);
    const float cos_tx = cosf(d->theta_x);
    const float sin_ty = sinf(d->theta_y);
    const float cos_ty = cosf(d->theta_y);

    /* 3D Cartesian soundboard center (nadir = (0, 0, 0), +z upward toward fly-tower) */
    const float xp = L * sin_tx;
    const float yp = L_y * sin_ty;
    const float zp = L * (1.0f - cos_tx) + 0.5f * L_y * (1.0f - cos_ty);

    d->x_p = xp;
    d->y_p = yp;
    d->z_p = zp;
    d->live_deg_x = d->theta_x * SWING_RAD_TO_DEG;
    d->live_deg_y = d->theta_y * SWING_RAD_TO_DEG;

    /* Stationary stereo hall mic pair at (-d_mic/2, y_mic, z_mic) and (+d_mic/2, y_mic, z_mic) */
    const float half_m = 0.5f * rig->d_mic;
    const float dx_l = xp - (-half_m);
    const float dx_r = xp - (+half_m);
    const float dy   = yp - rig->y_mic;
    const float dz   = zp - rig->z_mic;

    const float dyz_sq = dy * dy + dz * dz;
    const float r_l = sqrtf(dx_l * dx_l + dyz_sq);
    const float r_r = sqrtf(dx_r * dx_r + dyz_sq);

    /* Minimum reference distance at nadir (0, 0, 0) */
    const float r_nadir = sqrtf(half_m * half_m + rig->y_mic * rig->y_mic + rig->z_mic * rig->z_mic);
    const float r_floor = 0.75f * fabsf(rig->z_mic);

    /*
     * DOPPLER scaling:
     *   0%   -> 0.00x (zero pitch shift; pure spatial panning/proximity)
     *   50%  -> 1.00x (exact 343.0 m/s Barbican Hall physical propagation!)
     *   100% -> 2.80x (dramatic close-mic Aphex swoop)
     */
    const float dn = clampf(d->doppler_norm, 0.0f, 1.0f);
    const float doppler_scale = dn * (1.20f + 1.60f * dn);
    const float samp_per_m = d->sample_rate * (1.0f / SWING_SOUND_SPEED);

    const float del_l = 12.0f + fmaxf(0.0f, r_l - r_floor) * samp_per_m * doppler_scale;
    const float del_r = 12.0f + fmaxf(0.0f, r_r - r_floor) * samp_per_m * doppler_scale;
    d->del_dir_l_tgt = clampf(del_l, 4.0f, (float)(SWING_SRC_LEN - 16u));
    d->del_dir_r_tgt = clampf(del_r, 4.0f, (float)(SWING_SRC_LEN - 16u));

    /*
     * Opposite-wall lateral Barbican image sources at x_img_L = -2*W - x_p and x_img_R = +2*W - x_p:
     * When piano swings left (dx_p/dt < 0), r_wall_l shortens (pitching slightly UP) while
     * r_wall_r lengthens (pitching slightly DOWN).
     */
    const float two_w = 2.0f * rig->half_wall;
    const float dx_wl = two_w + xp - half_m;
    const float dx_wr = two_w - xp - half_m;
    const float r_wl  = sqrtf(dx_wl * dx_wl + dyz_sq);
    const float r_wr  = sqrtf(dx_wr * dx_wr + dyz_sq);

    const float er_base = clampf((two_w - r_floor) * 88.0f, 360.0f, 2200.0f);
    const float wall_dop = clampf(doppler_scale, 0.0f, 1.35f);
    const float del_wl = er_base + (r_wl - two_w) * samp_per_m * wall_dop;
    const float del_wr = er_base + (r_wr - two_w) * samp_per_m * wall_dop;
    d->del_wall_l_tgt = clampf(del_wl, 200.0f, (float)(SWING_SRC_LEN - 16u));
    d->del_wall_r_tgt = clampf(del_wr, 200.0f, (float)(SWING_SRC_LEN - 16u));

    /* Inverse-distance proximity swell + outward ORTF cardioid stereo directivity */
    const float k_prox = 0.14f + 0.18f * dn;
    const float prox_norm = 1.14f;
    const float prox_l = prox_norm / (1.0f + k_prox * fmaxf(0.0f, r_l - r_nadir));
    const float prox_r = prox_norm / (1.0f + k_prox * fmaxf(0.0f, r_r - r_nadir));

    const float r_center = sqrtf(xp * xp + dyz_sq + 0.04f);
    const float az = clampf(xp / r_center, -1.0f, 1.0f); /* -1 = far left, +1 = far right */

    /* Normalized ORTF outward cardioid response (1.00 at center, ~3:1 L/R ratio at apexes) */
    const float card_l = clampf((0.52f + 0.48f * (-0.85f * az + 0.35f)) * 1.453f, 0.22f, 1.42f);
    const float card_r = clampf((0.52f + 0.48f * (+0.85f * az + 0.35f)) * 1.453f, 0.22f, 1.42f);

    d->gain_l_tgt = clampf(prox_l * card_l * 0.84f, 0.08f, 1.35f);
    d->gain_r_tgt = clampf(prox_r * card_r * 0.84f, 0.08f, 1.35f);

    /*
     * Dynamic soundboard radiation + air-absorption low-pass cutoff per ear:
     * ~11.8 kHz directly overhead at nadir -> ~2.6 kHz when high & off-axis at opposite apex.
     */
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
 * Advance 2D nonlinear gravitational pendulum by `dt` seconds via Velocity-Verlet.
 */
static void step_pendulum_verlet(pendulum_engine_t* d, float dt) {
    if (dt <= 0.0f || dt > 0.05f) return;

    /* Smooth suspension cable length L so live ROPE tweaks never jerk velocity */
    d->rope_len += clampf(4.0f * dt, 0.0f, 1.0f) * (d->rope_len_target - d->rope_len);
    const float L = clampf(d->rope_len, 2.0f, 15.0f);
    const swing_rig_cfg_t* rig = &kRigs[d->rig_mode % SWING_RIG_COUNT];
    const float L_y = fmaxf(1.8f, L * rig->ly_ratio);

    const float g_L  = SWING_GRAVITY / L;
    const float g_Ly = SWING_GRAVITY / L_y;
    const float omega0 = sqrtf(g_L);

    float tx = d->theta_x;
    float ox = d->omega_x;
    float ty = d->theta_y;
    float oy = d->omega_y;

    float drive_x = 0.0f;
    float drive_y = 0.0f;
    const float damp = SWING_COAST_DAMP;

    if (d->swing_on) {
        const float th_max = d->theta_max;
        if (th_max > 0.003f) {
            const float e_cur = 0.5f * ox * ox + g_L * (1.0f - cosf(tx));
            const float e_tgt = g_L * (1.0f - cosf(th_max));

            /* Gentle self-starting nudge if piano is completely still at nadir */
            if (e_cur < 1.0e-5f * g_L) {
                ox += 0.28f * omega0 * th_max;
            }

            /* Phase-coherent energy-regulating escapement: vanishes on target nonlinear orbit */
            const float k_esc = clampf(damp + 1.85f * (e_tgt - e_cur) / (e_tgt + 0.04f),
                                       -1.20f, 1.85f);
            drive_x = k_esc * ox;

            /* Quadrature 2D elliptical depth coupling for theta_y */
            const float ty_max = th_max * rig->y_sway_ratio;
            const float oy_ref = -ty_max * omega0 * sinf(tx) / fmaxf(0.12f, sinf(th_max));
            const float ey_cur = 0.5f * oy * oy + g_Ly * (1.0f - cosf(ty));
            const float ey_tgt = g_Ly * (1.0f - cosf(ty_max));
            const float ky_esc = clampf(damp + 1.6f * (ey_tgt - ey_cur) / (ey_tgt + 0.02f),
                                        -1.2f, 1.6f);
            drive_y = ky_esc * oy + 0.35f * (oy_ref - oy);
        } else {
            /* ARC == 0%: smooth active braking to stationary center */
            drive_x = -1.10f * ox;
            drive_y = -1.10f * oy;
        }
    }

    /* 2nd-order Velocity-Verlet step on X pendulum */
    const float ax0 = -g_L * sinf(tx) - damp * ox + drive_x;
    const float ox_half = ox + 0.5f * dt * ax0;
    float tx_next = clampf(tx + dt * ox_half, -SWING_LIMIT_RAD, SWING_LIMIT_RAD);
    const float ax1 = -g_L * sinf(tx_next) - damp * ox_half + drive_x;
    float ox_next = ox_half + 0.5f * dt * ax1;
    if (fabsf(tx_next) >= SWING_LIMIT_RAD && (tx_next * ox_next) > 0.0f) {
        ox_next = 0.0f;
    }

    /* 2nd-order Velocity-Verlet step on Y depth pendulum */
    const float ay0 = -g_Ly * sinf(ty) - damp * oy + drive_y;
    const float oy_half = oy + 0.5f * dt * ay0;
    float ty_next = clampf(ty + dt * oy_half, -0.75f, 0.75f);
    const float ay1 = -g_Ly * sinf(ty_next) - damp * oy_half + drive_y;
    float oy_next = oy_half + 0.5f * dt * ay1;

    d->theta_x = flush_denorm(tx_next);
    d->omega_x = flush_denorm(ox_next);
    d->theta_y = flush_denorm(ty_next);
    d->omega_y = flush_denorm(oy_next);

    update_acoustic_targets(d, false);
}

void pendulum_init(pendulum_engine_t* d, float sample_rate) {
    if (!d) return;
    memset(d, 0, sizeof(*d));

    d->sample_rate = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    d->inv_fs = 1.0f / d->sample_rate;

    /* 18 Hz 1-pole DC blocker pole */
    d->dc_r = clampf(1.0f - (SWING_TWO_PI * 18.0f * d->inv_fs), 0.990f, 0.9995f);

    d->rig_mode = SWING_RIG_BARBICAN;
    d->swing_on = true;

    pendulum_set_rope(d, 0.45f);
    d->rope_len = d->rope_len_target;
    pendulum_set_arc(d, 0.55f);
    pendulum_set_doppler(d, 0.50f);
    pendulum_set_hall(d, 0.42f);

    /* Start poised at left turning point so first playback swoops gracefully across hall */
    d->theta_x = -d->theta_max;
    d->omega_x = 0.0f;
    d->theta_y = 0.0f;
    d->omega_y = sqrtf(SWING_GRAVITY / d->rope_len) * d->theta_max *
                 kRigs[d->rig_mode].y_sway_ratio;

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
    update_acoustic_targets(d, true);
}

void pendulum_set_rope(pendulum_engine_t* d, float norm) {
    if (!d) return;
    d->rope_norm = clampf(norm, 0.0f, 1.0f);
    /* 0% = 2.20m (~2.98s studio rig) -> 100% = 14.00m (~7.51s Barbican fly-tower) */
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
        if (e < 0.02f * g_L * (1.0f - cosf(d->theta_max))) {
            pendulum_push(d, 0.45f);
        }
    }
}

void pendulum_push(pendulum_engine_t* d, float impulse_norm) {
    if (!d) return;
    const float imp = clampf(impulse_norm, 0.15f, 1.0f);
    const float L = fmaxf(2.0f, d->rope_len);
    const float omega0 = sqrtf(SWING_GRAVITY / L);
    const float ref_ang = fmaxf(d->theta_max, 38.0f * SWING_DEG_TO_RAD);
    const float v_kick = imp * omega0 * (1.15f * sinf(0.5f * ref_ang) * 2.0f);

    const float dir = (fabsf(d->omega_x) > 0.01f)
                          ? ((d->omega_x >= 0.0f) ? 1.0f : -1.0f)
                          : ((d->theta_x <= 0.0f) ? 1.0f : -1.0f);

    float new_ox = d->omega_x + dir * v_kick;
    /* Cap mechanical energy safely below 60 deg ceiling */
    const float max_e = (SWING_GRAVITY / L) * (1.0f - cosf(58.0f * SWING_DEG_TO_RAD));
    const float pot_e = (SWING_GRAVITY / L) * (1.0f - cosf(d->theta_x));
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
    /* Keep snap_immediate = false so 2-pole delay smoothers glide smoothly to center! */
    update_acoustic_targets(d, false);
}

float pendulum_get_energy(const pendulum_engine_t* d) {
    if (!d) return 0.0f;
    const float L = fmaxf(2.0f, d->rope_len);
    return 0.5f * d->omega_x * d->omega_x + (SWING_GRAVITY / L) * (1.0f - cosf(d->theta_x));
}

void pendulum_step_idle(pendulum_engine_t* d, float dt_sec) {
    if (!d) return;
    const float step = clampf(dt_sec, 0.0f, 0.05f);
    /* Subdivide into 4 steps for crisp Verlet accuracy during idle UI preview */
    const float sub_dt = 0.25f * step;
    for (uint32_t i = 0u; i < 4u; ++i) {
        step_pendulum_verlet(d, sub_dt);
    }
    d->live_cents *= 0.85f;
}

static inline float schroeder_allpass(float* buf, uint32_t wr, uint32_t mask,
                                      uint32_t del, float g, float x) {
    const float delayed = buf[(wr - del) & mask];
    const float v = flush_denorm(x + g * delayed);
    buf[wr] = v;
    return delayed - g * v;
}

swing_frame_t pendulum_process_frame(pendulum_engine_t* d, swing_frame_t dry) {
    const float in_l = pendulum_clip_guard(dry.l);
    const float in_r = pendulum_clip_guard(dry.r);

    /* Step nonlinear 2D pendulum ODE every 8 samples (6,000 Hz) */
    if (d->phys_sub == 0u) {
        step_pendulum_verlet(d, (float)SWING_PHYS_SUB * d->inv_fs);
    }
    d->phys_sub = (uint8_t)((d->phys_sub + 1u) & (SWING_PHYS_SUB - 1u));

    /*
     * Cascaded 2-pole per-sample delay smoothing (a = 0.0020f -> tau ~10.4 ms):
     * Guarantees C1-continuous delay & pitch across knob turns while preserving
     * 99.95% of 0.13-0.35 Hz pendulum Doppler excursion.
     */
    const float ds_a = 0.0020f;
    const float prev_del_l = d->del_dir_l;
    const float prev_del_r = d->del_dir_r;

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

    /* Instantaneous Doppler pitch-shift telemetry in cents (~1731.23 * (ratio - 1)) */
    const float d_del_mean = 0.5f * ((d->del_dir_l - prev_del_l) + (d->del_dir_r - prev_del_r));
    const float inst_cents = clampf(-d_del_mean * 1731.234f, -199.0f, 199.0f);
    d->live_cents += 0.003f * (inst_cents - d->live_cents);

    /* Soundboard acoustic radiation: blend L/R slightly so point/soundboard source radiates coherently */
    const float sb_l = 0.80f * in_l + 0.20f * in_r;
    const float sb_r = 0.20f * in_l + 0.80f * in_r;

    float dir_l = sb_l;
    float dir_r = sb_r;
    float er_l  = 0.0f;
    float er_r  = 0.0f;

    swing_space_t* sp = d->space;
    if (sp) {
        const uint32_t wr = d->src_wr;
        sp->src_l[wr] = sb_l;
        sp->src_r[wr] = sb_r;

        /* Direct Hermite-interpolated soundboard -> stationary hall mic pair */
        dir_l = hermite_read_ring(sp->src_l, wr, SWING_SRC_MASK, d->del_dir_l);
        dir_r = hermite_read_ring(sp->src_r, wr, SWING_SRC_MASK, d->del_dir_r);

        /* Opposite-wall early reflections with counter-Doppler chorus */
        if (d->er_gain > 0.0005f) {
            const float wl_raw = hermite_read_ring(sp->src_l, wr, SWING_SRC_MASK, d->del_wall_l);
            const float wr_raw = hermite_read_ring(sp->src_r, wr, SWING_SRC_MASK, d->del_wall_r);

            d->wall_lp_l = flush_denorm(d->wall_lp_l + 0.34f * (wl_raw - d->wall_lp_l));
            d->wall_lp_r = flush_denorm(d->wall_lp_r + 0.34f * (wr_raw - d->wall_lp_r));

            er_l = d->er_gain * (0.82f * d->wall_lp_l + 0.28f * d->wall_lp_r);
            er_r = d->er_gain * (0.82f * d->wall_lp_r + 0.28f * d->wall_lp_l);
        }

        d->src_wr = (wr + 1u) & SWING_SRC_MASK;
    }

    /* Apply distance/off-axis soundboard radiation low-pass & proximity/ORTF gains */
    d->air_lp_l = flush_denorm(d->air_lp_l + d->lp_a_l * (dir_l - d->air_lp_l));
    d->air_lp_r = flush_denorm(d->air_lp_r + d->lp_a_r * (dir_r - d->air_lp_r));

    const float mic_l = d->air_lp_l * d->gain_l;
    const float mic_r = d->air_lp_r * d->gain_r;

    /* Concert Hall 4-Line Modulated Householder FDN Reverb */
    float rev_l = 0.0f;
    float rev_r = 0.0f;
    if (sp && d->hall_wet > 0.0005f) {
        const float sc = d->rig_size_scale;
        const uint32_t apw = d->ap_wr;
        const float diff_in_l = 0.45f * (mic_l + er_l);
        const float diff_in_r = 0.45f * (mic_r + er_r);

        const float diff_l = schroeder_allpass(sp->ap_l, apw, SWING_AP_MASK, 347u, 0.58f, diff_in_l);
        const float diff_r = schroeder_allpass(sp->ap_r, apw, SWING_AP_MASK, 463u, 0.58f, diff_in_r);
        d->ap_wr = (apw + 1u) & SWING_AP_MASK;

        /* Subtle LFO chorus modulation on FDN lines 2 & 3 (~0.31 Hz and ~0.47 Hz) */
        d->mod_ph_a += 0.31f * d->inv_fs;
        if (d->mod_ph_a >= 1.0f) d->mod_ph_a -= 1.0f;
        d->mod_ph_b += 0.47f * d->inv_fs;
        if (d->mod_ph_b >= 1.0f) d->mod_ph_b -= 1.0f;

        const float mod_a = 3.2f * sinf(SWING_TWO_PI * d->mod_ph_a);
        const float mod_b = 3.2f * sinf(SWING_TWO_PI * d->mod_ph_b);

        const uint32_t w0 = d->fdn0_wr;
        const uint32_t wf = d->fdn_wr;

        const uint32_t tap0 = (uint32_t)(997.0f * sc);
        const uint32_t tap1 = (uint32_t)(1493.0f * sc);
        const float    tap2 = 1789.0f * sc + mod_a;
        const float    tap3 = 2017.0f * sc + mod_b;

        const float r0 = sp->fdn0[(w0 - tap0) & SWING_FDN0_MASK];
        const float r1 = sp->fdn1[(wf - tap1) & SWING_FDN_MASK];
        const float r2 = linear_read_ring(sp->fdn2, wf, SWING_FDN_MASK, tap2);
        const float r3 = linear_read_ring(sp->fdn3, wf, SWING_FDN_MASK, tap3);

        /* Warm felt-hall HF damping on each FDN line */
        const float damp_a = d->fdn_damp;
        d->fdn_lp[0] = flush_denorm(d->fdn_lp[0] + damp_a * (r0 - d->fdn_lp[0]));
        d->fdn_lp[1] = flush_denorm(d->fdn_lp[1] + damp_a * (r1 - d->fdn_lp[1]));
        d->fdn_lp[2] = flush_denorm(d->fdn_lp[2] + damp_a * (r2 - d->fdn_lp[2]));
        d->fdn_lp[3] = flush_denorm(d->fdn_lp[3] + damp_a * (r3 - d->fdn_lp[3]));

        const float fb = d->fdn_fb;
        const float u0 = d->fdn_lp[0] * fb;
        const float u1 = d->fdn_lp[1] * fb;
        const float u2 = d->fdn_lp[2] * fb;
        const float u3 = d->fdn_lp[3] * fb;

        /* Orthogonal 4x4 Householder reflection matrix: H*u = u - 0.5*(u0+u1+u2+u3) */
        const float hsum = 0.5f * (u0 + u1 + u2 + u3);
        sp->fdn0[w0] = flush_denorm((u0 - hsum) + diff_l);
        sp->fdn1[wf] = flush_denorm((u1 - hsum) + diff_r);
        sp->fdn2[wf] = flush_denorm((u2 - hsum) + 0.7f * (diff_l - diff_r));
        sp->fdn3[wf] = flush_denorm((u3 - hsum) + 0.7f * (diff_r + diff_l));

        d->fdn0_wr = (w0 + 1u) & SWING_FDN0_MASK;
        d->fdn_wr  = (wf + 1u) & SWING_FDN_MASK;

        rev_l = d->hall_wet * 0.52f * (d->fdn_lp[0] + d->fdn_lp[2] - 0.45f * d->fdn_lp[1]);
        rev_r = d->hall_wet * 0.52f * (d->fdn_lp[1] + d->fdn_lp[3] - 0.45f * d->fdn_lp[2]);
    }

    const float sum_l = mic_l + er_l + rev_l;
    const float sum_r = mic_r + er_r + rev_r;

    /* Update live proximity & hall bloom envelopes for OLED visualizer */
    const float al = fabsf(mic_l);
    const float ar = fabsf(mic_r);
    const float ah = 0.5f * (fabsf(er_l + rev_l) + fabsf(er_r + rev_r));
    d->prox_env_l = (al > d->prox_env_l) ? (d->prox_env_l + 0.02f * (al - d->prox_env_l))
                                         : (d->prox_env_l * 0.9992f);
    d->prox_env_r = (ar > d->prox_env_r) ? (d->prox_env_r + 0.02f * (ar - d->prox_env_r))
                                         : (d->prox_env_r * 0.9992f);
    d->hall_env   = (ah > d->hall_env)   ? (d->hall_env   + 0.01f * (ah - d->hall_env))
                                         : (d->hall_env   * 0.9995f);

    /* 18 Hz stereo DC blocker + odd-symmetric C1 Pade rational tanh soft limiter */
    const float y_dc_l = flush_denorm(sum_l - d->dc_x_l + d->dc_r * d->dc_y_l);
    d->dc_x_l = sum_l;
    d->dc_y_l = y_dc_l;

    const float y_dc_r = flush_denorm(sum_r - d->dc_x_r + d->dc_r * d->dc_y_r);
    d->dc_x_r = sum_r;
    d->dc_y_r = y_dc_r;

    swing_frame_t out = {
        .l = pendulum_soft_limit(y_dc_l),
        .r = pendulum_soft_limit(y_dc_r),
    };
    return out;
}

/* ----------------------------------------------------------------------------
 * Built-In "Seed Felt Piano" Generator (aisatsana [102]-inspired motif)
 * ------------------------------------------------------------------------- */
typedef struct {
    float beat;     /* Note onset in beats [0 .. 16) */
    uint8_t midi;   /* MIDI pitch (60 = C4)          */
    float vel;      /* Soft felt dynamic [0.2 .. 0.8] */
    float dur_b;    /* Ring duration in beats         */
} felt_note_t;

/*
 * 4-bar contemplative Cmaj9 -> Am9 -> Fmaj9(#11) -> G6sus4 felt-piano voicing
 * with subtle human rolled micro-timing, wrapping seamlessly across 16 beats.
 */
static const felt_note_t kAisatsanaNotes[] = {
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
    /* 440 * 2^((midi - 69)/12) via expf(x * ln(2)) */
    const float semitones = ((float)midi - 69.0f) * (1.0f / 12.0f);
    return 440.0f * expf(semitones * 0.69314718f);
}

uint32_t pendulum_seed_felt_piano(swing_frame_t* buf, uint32_t max_frames, float sample_rate) {
    if (!buf || max_frames < 1024u) return 0u;
    const float fs = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    const float inv_fs = 1.0f / fs;

    /* Target 9.60s (16 beats @ 100 BPM = 0.60s/beat); scale cleanly if RAM < 9.6s */
    uint32_t target_len = (uint32_t)(9.60f * fs);
    if (target_len > max_frames) target_len = max_frames;
    /* Align to 16-frame boundary */
    target_len &= ~15u;
    if (target_len < 1024u) return 0u;

    memset(buf, 0, (size_t)target_len * sizeof(swing_frame_t));

    const float frames_per_beat = (float)target_len * (1.0f / 16.0f);
    const uint32_t attack_fr = (uint32_t)(0.0042f * fs) + 4u;    /* ~4.2 ms soft wool hammer */
    const uint32_t rel_fr    = (uint32_t)(0.0850f * fs) + 8u;    /* 85 ms damper release     */
    const float inv_atk = 1.0f / (float)attack_fr;
    const float inv_rel = 1.0f / (float)rel_fr;

    static const float kPartAmp[6] = { 1.00f, 0.42f, 0.17f, 0.075f, 0.032f, 0.014f };

    const uint32_t num_notes = (uint32_t)(sizeof(kAisatsanaNotes) / sizeof(kAisatsanaNotes[0]));
    for (uint32_t ni = 0u; ni < num_notes; ++ni) {
        const felt_note_t nt = kAisatsanaNotes[ni];
        const float f0 = midi_to_hz(nt.midi);
        const uint32_t onset = ((uint32_t)(nt.beat * frames_per_beat)) % target_len;
        uint32_t dur = (uint32_t)(nt.dur_b * frames_per_beat);
        if (dur > target_len) dur = target_len;
        if (dur < attack_fr + rel_fr + 16u) continue;

        /* Soundboard note placement (subtle bass-left to treble-right acoustic layout) */
        const float key_pos = clampf(((float)nt.midi - 40.0f) * (1.0f / 36.0f), 0.0f, 1.0f);
        const float pan_l = 0.72f - 0.28f * key_pos;
        const float pan_r = 0.44f + 0.28f * key_pos;

        /* Precompute damped complexphasor multipliers for 6 stiff-string partials + bichord beat */
        float c_re[6], c_im[6], st_re[6], st_im[6];
        float b_re, b_im, bst_re, bst_im;
        const float inharm_b = 0.00028f;

        for (uint32_t p = 0u; p < 6u; ++p) {
            const float k = (float)(p + 1u);
            const float fk = f0 * k * sqrtf(1.0f + inharm_b * k * k);
            if (fk > 0.42f * fs) {
                st_re[p] = 0.0f;
                st_im[p] = 0.0f;
                c_re[p] = 1.0f;
                c_im[p] = 0.0f;
                continue;
            }
            /* Higher partials & higher notes decay faster (warm felt-hammer damping) */
            const float tau = 2.90f / (1.0f + 0.38f * (k - 1.0f) * (k - 1.0f) + 0.0015f * f0);
            const float r_dec = expf(-inv_fs / tau);
            const float w = SWING_TWO_PI * fk * inv_fs;
            c_re[p] = r_dec * cosf(w);
            c_im[p] = r_dec * sinf(w);
            st_re[p] = 0.0f;
            st_im[p] = kPartAmp[p] * nt.vel * 0.26f;
        }

        /* Unison second string detuned by +0.55 cents for intimate acoustic piano shimmer */
        {
            const float f_uni = f0 * 1.000318f;
            const float r_uni = expf(-inv_fs / 2.75f);
            const float w_uni = SWING_TWO_PI * f_uni * inv_fs;
            b_re = r_uni * cosf(w_uni);
            b_im = r_uni * sinf(w_uni);
            bst_re = 0.0f;
            bst_im = 0.34f * nt.vel * 0.26f;
        }

        /* Subtle woody felt-hammer keybed thump (~148 Hz, ~22 ms decay) */
        const float r_th = expf(-inv_fs / 0.018f);
        const float w_th = SWING_TWO_PI * 148.0f * inv_fs;
        const float th_c_re = r_th * cosf(w_th);
        const float th_c_im = r_th * sinf(w_th);
        float th_re = 0.0f;
        float th_im = 0.045f * nt.vel;

        uint32_t idx = onset;
        const uint32_t rel_start = dur - rel_fr;

        for (uint32_t n = 0u; n < dur; ++n) {
            float s_main = 0.0f;
            for (uint32_t p = 0u; p < 6u; ++p) {
                s_main += st_im[p];
                const float nr = st_re[p] * c_re[p] - st_im[p] * c_im[p];
                const float ni_im = st_re[p] * c_im[p] + st_im[p] * c_re[p];
                st_re[p] = nr;
                st_im[p] = ni_im;
            }

            const float s_uni = bst_im;
            {
                const float nr = bst_re * b_re - bst_im * b_im;
                const float ni_im = bst_re * b_im + bst_im * b_re;
                bst_re = nr;
                bst_im = ni_im;
            }

            const float s_th = th_im;
            {
                const float nr = th_re * th_c_re - th_im * th_c_im;
                const float ni_im = th_re * th_c_im + th_im * th_c_re;
                th_re = nr;
                th_im = ni_im;
            }

            float env = 1.0f;
            if (n < attack_fr) {
                const float t = (float)n * inv_atk;
                env = t * t * (3.0f - 2.0f * t);
            } else if (n >= rel_start) {
                const float t = (float)(dur - n) * inv_rel;
                env = t * t * (3.0f - 2.0f * t);
            }

            const float sl = env * ((s_main + s_th) * pan_l + s_uni * pan_r);
            const float sr = env * ((s_main + s_th) * pan_r + s_uni * pan_l);

            buf[idx].l += sl;
            buf[idx].r += sr;

            idx++;
            if (idx >= target_len) idx = 0u;
        }
    }

    /* Gentle warm felt low-pass polish & soft ceiling normalization */
    float lp_l = buf[target_len - 1u].l;
    float lp_r = buf[target_len - 1u].r;
    const float felt_a = clampf(SWING_TWO_PI * 3600.0f * inv_fs, 0.1f, 0.7f);
    for (uint32_t i = 0u; i < target_len; ++i) {
        lp_l += felt_a * (buf[i].l - lp_l);
        lp_r += felt_a * (buf[i].r - lp_r);
        buf[i].l = pendulum_soft_limit(lp_l * 1.15f);
        buf[i].r = pendulum_soft_limit(lp_r * 1.15f);
    }

    return target_len;
}
