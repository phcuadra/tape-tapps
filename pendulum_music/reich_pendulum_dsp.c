#include "reich_pendulum_dsp.h"

#ifdef PM_HOST_TEST
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

#define PM_GRAVITY          9.80665f
#define PM_SOUND_SPEED      343.0f
#define PM_TWO_PI           6.28318530717958647692f
#define PM_DEG_TO_RAD       0.01745329251994329577f
#define PM_RAD_TO_DEG       57.2957795130823208768f
#define PM_LIMIT_RAD        1.04719755f             /* 60 deg safety ceiling */
#define PM_RELEASE_RAD      0.80285146f             /* 46.0 deg Reich pull-back angle */
#define PM_BASE_CABLE_M     2.25f                   /* ~3.01 s base pendulum period */
#define PM_H_GAP_M          0.14f                   /* 14 cm mic-to-cone clearance at nadir */
#define PM_PHYS_SUB         64u                     /* 375 Hz symplectic Verlet rate at 24 kHz */
#define PM_INV_PHYS_SUB     (1.0f / 64.0f)

/* Fixed gallery bay air transit delays at 24 kHz (0.80 m, 1.60 m, 2.40 m @ 343 m/s) */
#define PM_BAY1_DELAY       56u
#define PM_BAY2_DELAY       112u
#define PM_BAY3_DELAY       168u

/* 4 upward-facing floor speaker X coordinates along the gallery stage (meters) */
static const float kSpkX[PM_NUM_MICS] = { -1.20f, -0.40f, +0.40f, +1.20f };

/* Pre-baked Mode 1 floor-speaker stereo pan weights: 0.31 * (1 -+ pan) */
static const float kSpkGainL[PM_NUM_MICS] = {
    0.31f * (1.0f + 0.82f),
    0.31f * (1.0f + 0.27f),
    0.31f * (1.0f - 0.27f),
    0.31f * (1.0f - 0.82f),
};
static const float kSpkGainR[PM_NUM_MICS] = {
    0.31f * (1.0f - 0.82f),
    0.31f * (1.0f - 0.27f),
    0.31f * (1.0f + 0.27f),
    0.31f * (1.0f + 0.82f),
};

/* Non-commensurate cable-length offsets controlled by SPREAD (~1.90m .. 2.62m at 100%) */
static const float kCableOffsets[PM_NUM_MICS] = { -0.155f, -0.052f, +0.058f, +0.165f };

/* 3 speaker/room resonance tuning voicings (Hz) */
static const float kTuningFreqs[PM_TUNING_COUNT][PM_NUM_MICS] = {
    /* Gallery Fifths: G3 - D4 - A4 - D5 (open stacked Reich fifths) */
    [PM_TUNING_FIFTHS]   = { 196.00f, 293.66f, 440.00f, 587.33f },
    /* Warm Maj9:      C4 - E4 - G4 - D5 (luminous gallery chord)    */
    [PM_TUNING_MAJ9]     = { 261.63f, 329.63f, 392.00f, 587.33f },
    /* Raw 1968 PA:    close microtonal PA cluster around ~422 Hz     */
    [PM_TUNING_RAW_1968] = { 398.00f, 417.00f, 426.00f, 444.00f },
};

static const char* const kTuningNames[PM_TUNING_COUNT] = {
    "Gallery Fifths",
    "Warm Maj9",
    "Raw 1968 PA",
};

static const char* const kBankNames[PM_BANK_COUNT] = {
    "Organ Quartet",
    "Marimba & Bells",
    "Tape Choir",
    "Bowed Glass",
};

static const char* const kBankTags[PM_BANK_COUNT][PM_NUM_MICS] = {
    { "ORG1", "ORG2", "ORG3", "ORG4" },
    { "MRB1", "MRB2", "MRB3", "MRB4" },
    { "CHR1", "CHR2", "CHR3", "CHR4" },
    { "GLS1", "GLS2", "GLS3", "GLS4" },
};

/*
 * Branchless minimax sin/cos on [-pi/2, +pi/2] (zero external libm calls!).
 * Pendulum angles are strictly bounded to +-1.0472 rad (+-60 deg) and pan /
 * filter tuning angles lie well inside [-1.57, +1.57].
 */
static inline float sin_halfpi(float r) {
    const float r2 = r * r;
    return r * (1.0f - r2 * (0.16666657f - r2 * (0.00833216f - r2 * 0.00019515f)));
}

static inline float cos_halfpi(float r) {
    const float r2 = r * r;
    return 1.0f - r2 * (0.5f - r2 * (0.04166646f - r2 * 0.00138873f));
}

/*
 * Ultra-fast register-only sine from 32-bit unsigned phase [0 .. 2^32 - 1].
 * 4 FPU ops, zero memory loads, < 0.0009 peak error.
 */
static inline float fast_sin_u32(uint32_t ph) {
    const int32_t sp = (int32_t)ph;
    const float x = (float)sp * (1.0f / 2147483648.0f);
    const float y = 4.0f * x * (1.0f - fabsf(x));
    return y * (0.775f + 0.225f * fabsf(y));
}

static inline uint32_t hz_to_phase_inc(float hz, float inv_fs) {
    const float norm = clampf(hz * inv_fs, 0.0f, 0.45f);
    return (uint32_t)(norm * 4294967296.0f + 0.5f);
}

/* Fast deterministic xorshift32 PRNG returning uniform [-1.0f, +1.0f] */
static inline float next_noise_bi(uint32_t* state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    const int32_t s = (int32_t)(x >> 9) - 4194304;
    return (float)s * (1.0f / 4194304.0f);
}

float pm_clip_guard(float x) {
    return clampf(x, -1.25f, 1.25f);
}

/*
 * Zero-division, zero-sqrt odd-symmetric C1 cubic PA saturator:
 * Exact small-signal slope f'(0) = 1.0000 (preserves Barkhausen oscillation threshold!)
 * and strictly odd f(-x) = -f(x) (zero DC generation!), ceiling +-0.86f.
 */
static inline float fast_pa_sat(float x) {
    const float u = clampf(x * (1.0f / 0.86f), -1.5f, 1.5f);
    return 0.86f * u * (1.0f - (4.0f / 27.0f) * u * u);
}

/* Zero-division C1 cubic output soft limiter with exact ceiling +-0.98f */
float pm_soft_limit(float x) {
    const float c = clampf(x, -1.47f, 1.47f);
    return c * (1.0f - 0.15429503f * c * c);
}

/*
 * Unclamped 2-point linear fractional read on interleaved `spk_ring`.
 * `delay_samp` is guaranteed in [4.0f, PM_SPK_DELAY_LEN - 8] by control-rate bounds.
 */
static inline float linear_read_quad(const pm_quad_t* restrict ring,
                                     uint32_t wr,
                                     uint32_t ch,
                                     float delay_samp) {
    const int32_t di = (int32_t)delay_samp;
    const float f = delay_samp - (float)di;
    const uint32_t base = wr - (uint32_t)di;
    const float y0 = ring[base        & PM_SPK_DELAY_MASK].ch[ch];
    const float y1 = ring[(base - 1u) & PM_SPK_DELAY_MASK].ch[ch];
    return y0 + f * (y1 - y0);
}

const char* pm_tuning_name(uint8_t tuning_mode) {
    return kTuningNames[tuning_mode % PM_TUNING_COUNT];
}

const char* pm_bank_name(uint8_t bank_idx) {
    return kBankNames[bank_idx % PM_BANK_COUNT];
}

const char* pm_bank_slot_tag(uint8_t bank_idx, uint8_t slot_idx) {
    return kBankTags[bank_idx % PM_BANK_COUNT][slot_idx & 3u];
}

static void refresh_cable_reciprocals(reich_pendulum_dsp_t* d, uint32_t i) {
    const float L = clampf(d->cable_len[i], 1.60f, 3.10f);
    d->cable_len[i] = L;
    const float invL = 1.0f / L;
    d->inv_cable[i] = invL;
    d->inv_w0[i] = 1.0f / sqrtf(PM_GRAVITY * invL);
}

static void update_cable_targets(reich_pendulum_dsp_t* d, bool snap) {
    const float sp = clampf(d->spread_norm, 0.0f, 1.0f);
    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const float L = clampf(PM_BASE_CABLE_M * (1.0f + sp * kCableOffsets[i]), 1.60f, 3.10f);
        d->cable_tgt[i] = L;
        if (snap || d->cable_len[i] < 1.0f) {
            d->cable_len[i] = L;
            refresh_cable_reciprocals(d, i);
        }
    }
}

static void update_synth_bank_freqs(reich_pendulum_dsp_t* d) {
    const uint8_t b = d->bank_idx % PM_BANK_COUNT;
    const float inv_fs = d->inv_fs;

    static const float kBankBaseHz[PM_BANK_COUNT][PM_NUM_MICS] = {
        /* Organ Quartet: C3, G3, D4, A4 */
        { 130.81f, 196.00f, 293.66f, 440.00f },
        /* Marimba & Bells: F3, C4, G4, D5 */
        { 174.61f, 261.63f, 392.00f, 587.33f },
        /* Tape Choir: D3, A3, E4, F#4 */
        { 146.83f, 220.00f, 329.63f, 369.99f },
        /* Bowed Glass: E3, B3, F#4, G#4 */
        { 164.81f, 246.94f, 369.99f, 415.30f },
    };

    static const float kLfoHz[PM_BANK_COUNT][PM_NUM_MICS] = {
        { 1.25f, 1.65f, 2.10f, 2.55f },
        { 4.15f, 5.00f, 3.35f, 2.50f },
        { 0.85f, 1.15f, 1.45f, 1.80f },
        { 0.90f, 1.20f, 1.55f, 1.95f },
    };

    for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
        const float f0 = kBankBaseHz[b][s];
        float m1 = 2.00f;
        float m2 = 3.00f;
        if (b == PM_BANK_ORGANS) {
            m1 = (s == 1u) ? 3.00f : 2.00f;
            m2 = (s == 3u) ? 1.50f : 4.00f;
        } else if (b == PM_BANK_MARIMBA) {
            m1 = (s < 2u) ? 3.98f : 2.00f;
            m2 = (s == 3u) ? 1.50f : 2.99f;
        } else if (b == PM_BANK_CHOIR) {
            m1 = 2.003f;
            m2 = 3.005f;
        } else {
            m1 = 2.000f;
            m2 = 2.320f;
        }

        d->synth_inc[s][0] = hz_to_phase_inc(f0, inv_fs);
        d->synth_inc[s][1] = hz_to_phase_inc(f0 * m1, inv_fs);
        d->synth_inc[s][2] = hz_to_phase_inc(f0 * m2, inv_fs);
        d->lfo_inc[s]      = hz_to_phase_inc(kLfoHz[b][s], inv_fs);
    }
    d->bank_dirty = false;
}

static void update_svf_voicing(reich_pendulum_dsp_t* d) {
    const uint8_t tm = d->tuning_mode % PM_TUNING_COUNT;
    const float q_voice = (tm == PM_TUNING_RAW_1968) ? 4.4f : 3.8f;
    const float k_voice = 1.0f / q_voice;

    /* 1-pole cabinet cone lowpass coefficient (~3.1 kHz at 24 kHz) */
    const float lp_a = clampf(0.56f * (24000.0f * d->inv_fs), 0.25f, 0.70f);
    const float lp_b = 1.0f - lp_a;
    const float tau0 = 7.0f * (d->sample_rate * (1.0f / 24000.0f));

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const float fc = kTuningFreqs[tm][i];
        /* hw = pi * fc / fs <= 0.077 rad (< 4.5 deg) -> sin_halfpi/cos_halfpi exact to 1e-7 */
        const float hw = clampf(M_PI * fc * d->inv_fs, 0.002f, 0.45f);
        const float g = sin_halfpi(hw) / cos_halfpi(hw);
        const float a1 = 1.0f / (1.0f + g * (g + k_voice));

        d->tau_trim[i] = clampf(tau0, 4.0f, 32.0f);

        /* Exact Barkhausen closed-loop unity-gain threshold calibration at fc (zero libm imports) */
        const float w = clampf(2.0f * hw, 0.004f, 0.90f);
        const float cw = cos_halfpi(w);
        const float sw = sin_halfpi(w);
        const float re_d = 1.0f - lp_b * cw;
        const float im_d = lp_b * sw;
        const float mag_lp = lp_a / sqrtf(re_d * re_d + im_d * im_d);
        const float ratio = im_d / fmaxf(0.05f, re_d);
        const float atan_apx = ratio / (1.0f + 0.28125f * ratio * ratio);
        const float phi_lag = clampf(w * d->tau_trim[i] + atan_apx, -1.15f, 1.15f);
        const float cos_lag = fmaxf(0.42f, cos_halfpi(phi_lag));

        d->svf_g[i]      = g;
        d->svf_k_gain[i] = k_voice / fmaxf(0.30f, mag_lp * cos_lag);
        d->svf_a1[i]     = a1;
        d->svf_a2[i]     = g * a1;
        d->svf_a3[i]     = g * g * a1;
    }
    d->voicing_dirty = false;
}

/*
 * Recompute 3D capsule positions, self propagation delays, smooth gallery air
 * coupling gains, Mode 2 pan gains, and inter-mic phase coherence (at 375 Hz,
 * exclusively on the audio thread or during single-threaded init/reset).
 */
static void update_acoustic_matrix(reich_pendulum_dsp_t* d, bool snap) {
    const float samp_per_m = d->sample_rate * (1.0f / PM_SOUND_SPEED);
    const float pa_base = 0.88f + 1.95f * clampf(d->gain_norm, 0.0f, 1.0f);
    const float cpl_amt = clampf(d->couple_norm, 0.0f, 1.0f);
    const float max_self_del = (float)(PM_SPK_DELAY_LEN - 8u);

    float g_self[PM_NUM_MICS];
    float r_amp[PM_NUM_MICS];

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const float L = d->cable_len[i];
        const float th = clampf(d->theta[i], -PM_LIMIT_RAD, PM_LIMIT_RAD);
        const float st = sin_halfpi(th);
        const float ct = cos_halfpi(th);

        const float dx_self = L * st;
        const float px = kSpkX[i] + dx_self;
        const float pz = PM_H_GAP_M + L * (1.0f - ct);

        d->px[i] = px;
        d->pz[i] = pz;
        d->deg[i] = th * PM_RAD_TO_DEG;

        /* Precompute constant-power Mode 2 pan gains (ang in [0, pi/2]) */
        const float pan = clampf(px * 0.52f, -0.86f, 0.86f);
        const float ang = (pan + 1.0f) * (0.25f * M_PI);
        d->vca_pan_l[i] = cos_halfpi(ang);
        d->vca_pan_r[i] = sin_halfpi(ang);

        const float u = st;
        const float v = d->omega[i] * d->inv_w0[i];
        d->norm_u[i] = u;
        d->norm_v[i] = v;
        r_amp[i] = sqrtf(u * u + v * v + 1.0e-12f);

        const float dx_s = dx_self * 2.15f;
        const float beam = 1.0f / (1.0f + dx_s * dx_s + 2.10f * (pz - PM_H_GAP_M));
        g_self[i] = beam;
        d->self_gain_tgt[i] = beam;

        const float self_dist = sqrtf(dx_self * dx_self + pz * pz);
        const float extra_m = fmaxf(0.0f, self_dist - PM_H_GAP_M);
        d->self_del_tgt[i] = clampf(d->tau_trim[i] + extra_m * samp_per_m, 4.0f, max_self_del);
    }

    float sync_num = 0.0f;
    float sync_den = 0.0f;
    static const float kDistWeight[4] = { 0.0f, 0.56f, 0.36f, 0.26f };

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        float interf_sum = 0.0f;

        if (cpl_amt > 0.001f) {
            for (uint32_t j = 0u; j < PM_NUM_MICS; ++j) {
                if (j == i) continue;

                const uint32_t bay_diff = (i > j) ? (i - j) : (j - i);
                const float dist_w = kDistWeight[bay_diff & 3u];

                const float prod_r = r_amp[i] * r_amp[j];
                const float act = clampf(prod_r * 14.0f, 0.0f, 1.0f);
                const float raw_cos = (d->norm_u[i] * d->norm_u[j] +
                                       d->norm_v[i] * d->norm_v[j]) /
                                      (prod_r + 1.0e-5f);
                const float coh = act * clampf(raw_cos, -1.0f, 1.0f) + (1.0f - act) * 0.24f;

                const float pair_act = g_self[i] * g_self[j] * dist_w;
                interf_sum += cpl_amt * 0.42f * pair_act * coh;

                if (j == i + 1u) {
                    d->pair_coh[i] = coh * cpl_amt;
                }
                if (j > i) {
                    const float w_pair = 0.25f + pair_act;
                    sync_num += coh * w_pair;
                    sync_den += w_pair;
                }
            }
        } else if (i < 3u) {
            d->pair_coh[i] = 0.0f;
        }

        /* Continuous smooth gallery cross-pickup & loop gain targets (zero threshold steps!) */
        d->cross_gain_tgt[i] = cpl_amt * 0.24f * (0.20f + 0.80f * g_self[i]);
        const float mod = clampf(1.0f + interf_sum, 0.62f, 1.52f);
        d->loop_gain_tgt[i] = pa_base * mod;

        if (snap || d->self_del_cur[i] < 4.0f) {
            d->self_del_cur[i]   = d->self_del_tgt[i];
            d->self_gain_cur[i]  = d->self_gain_tgt[i];
            d->cross_gain_cur[i] = d->cross_gain_tgt[i];
            d->loop_gain_cur[i]  = d->loop_gain_tgt[i];
            d->self_del_inc[i]   = 0.0f;
            d->self_gain_inc[i]  = 0.0f;
            d->cross_gain_inc[i] = 0.0f;
            d->loop_gain_inc[i]  = 0.0f;
        } else {
            d->self_del_inc[i]   = (d->self_del_tgt[i]   - d->self_del_cur[i])   * PM_INV_PHYS_SUB;
            d->self_gain_inc[i]  = (d->self_gain_tgt[i]  - d->self_gain_cur[i])  * PM_INV_PHYS_SUB;
            d->cross_gain_inc[i] = (d->cross_gain_tgt[i] - d->cross_gain_cur[i]) * PM_INV_PHYS_SUB;
            d->loop_gain_inc[i]  = (d->loop_gain_tgt[i]  - d->loop_gain_cur[i])  * PM_INV_PHYS_SUB;
        }
    }

    const float target_sync = (sync_den > 1.0e-4f)
                                  ? clampf((sync_num / sync_den) * cpl_amt, -1.0f, 1.0f)
                                  : 0.0f;
    d->phase_sync += 0.22f * (target_sync - d->phase_sync);
}

static void step_pendulums_verlet(reich_pendulum_dsp_t* d, float dt) {
    if (dt <= 0.0f || dt > 0.05f) return;

    /* Refresh cable length targets on the audio thread from volatile `spread_norm` */
    update_cable_targets(d, false);

    const float dec = clampf(d->decay_norm, 0.0f, 1.0f);
    const float gamma = (dec < 0.005f) ? 0.0f : (dec * dec * 0.44f + dec * 0.055f);
    const float cable_slew = clampf(5.0f * dt, 0.0f, 1.0f);

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const float diff_L = d->cable_tgt[i] - d->cable_len[i];
        if (fabsf(diff_L) > 1.0e-5f) {
            d->cable_len[i] += cable_slew * diff_L;
            refresh_cable_reciprocals(d, i);
        }
        const float g_L = PM_GRAVITY * d->inv_cable[i];

        const float th0 = clampf(d->theta[i], -PM_LIMIT_RAD, PM_LIMIT_RAD);
        const float om0 = d->omega[i];

        const float a0 = -g_L * sin_halfpi(th0) - gamma * om0;
        const float om_half = om0 + 0.5f * dt * a0;
        float th1 = clampf(th0 + dt * om_half, -PM_LIMIT_RAD, PM_LIMIT_RAD);
        const float a1 = -g_L * sin_halfpi(th1) - gamma * om_half;
        float om1 = om_half + 0.5f * dt * a1;

        if (fabsf(th1) >= PM_LIMIT_RAD && (th1 * om1) > 0.0f) {
            om1 = 0.0f;
        }

        d->theta[i] = th1;
        d->omega[i] = om1;
    }

    update_acoustic_matrix(d, false);
}

void pm_dsp_init(reich_pendulum_dsp_t* d, float sample_rate) {
    if (!d) return;
    memset(d, 0, sizeof(*d));

    d->sample_rate = (sample_rate > 8000.0f) ? sample_rate : (float)PM_INTERNAL_SR;
    d->inv_fs = 1.0f / d->sample_rate;
    d->dc_r = clampf(1.0f - (PM_TWO_PI * 18.0f * d->inv_fs), 0.990f, 0.9995f);
    d->rng_state = 0x19685245u;

    d->gain_norm       = 0.58f;
    d->couple_norm     = 0.50f;
    d->spread_norm     = 0.42f;
    d->decay_norm      = 0.35f;
    d->tuning_mode     = PM_TUNING_FIFTHS;
    d->bank_idx        = PM_BANK_ORGANS;
    d->release_stagger = false;
    d->voicing_dirty   = false;
    d->bank_dirty      = false;
    d->phys_rem        = 0u;

    for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
        d->synth_ph[s][0] = s * 0x29000000u;
        d->synth_ph[s][1] = s * 0x51000000u + 0x13000000u;
        d->synth_ph[s][2] = s * 0x77000000u + 0x3B000000u;
        d->lfo_ph[s]      = s * 0x40000000u;
    }

    update_cable_targets(d, true);
    update_svf_voicing(d);
    update_synth_bank_freqs(d);
    pm_pull_and_release(d);
}

void pm_dsp_attach_space(reich_pendulum_dsp_t* d, pend_space_t* space) {
    if (!d) return;
    d->space = space;
    pm_dsp_clear_audio(d);
}

void pm_dsp_clear_audio(reich_pendulum_dsp_t* d) {
    if (!d) return;
    if (d->space) {
        memset(d->space, 0, sizeof(*d->space));
    }
    d->spk_wr = 0u;
    d->room_wr = 0u;
    d->ap_wr = 0u;
    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        d->lp_y[i] = 0.0f;
        d->svf_ic1[i] = 0.0f;
        d->svf_ic2[i] = 0.0f;
        d->last_spk[i] = 0.0f;
        d->env_raw[i] = 0.0f;
        d->env[i] = 0.0f;
        d->vca_gain[i] = 0.0f;
    }
    d->room_lp_l = 0.0f;
    d->room_lp_r = 0.0f;
    d->out_dc_xl = 0.0f;
    d->out_dc_yl = 0.0f;
    d->out_dc_xr = 0.0f;
    d->out_dc_yr = 0.0f;
    d->phys_rem  = PM_PHYS_SUB;
    update_acoustic_matrix(d, true);
}

/* Lock-free UI setters: write volatile targets only; audio thread applies smoothly */
void pm_set_gain(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->gain_norm = clampf(norm, 0.0f, 1.0f);
}

void pm_set_couple(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->couple_norm = clampf(norm, 0.0f, 1.0f);
}

void pm_set_spread(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->spread_norm = clampf(norm, 0.0f, 1.0f);
}

void pm_set_decay(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->decay_norm = clampf(norm, 0.0f, 1.0f);
}

void pm_set_tuning(reich_pendulum_dsp_t* d, uint8_t tuning_mode) {
    if (!d) return;
    d->tuning_mode = tuning_mode % PM_TUNING_COUNT;
    d->voicing_dirty = true;
}

void pm_set_bank(reich_pendulum_dsp_t* d, uint8_t bank_idx) {
    if (!d) return;
    d->bank_idx = bank_idx % PM_BANK_COUNT;
    d->bank_dirty = true;
}

void pm_set_release_stagger(reich_pendulum_dsp_t* d, bool stagger) {
    if (!d) return;
    d->release_stagger = stagger;
}

void pm_pull_and_release(reich_pendulum_dsp_t* d) {
    if (!d) return;
    update_cable_targets(d, true);

    if (!d->release_stagger) {
        for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
            d->theta[i] = -PM_RELEASE_RAD;
            d->omega[i] = 0.0f;
        }
    } else {
        static const float kStagDeg[PM_NUM_MICS] = { -47.0f, -38.0f, +43.0f, -31.0f };
        static const float kStagVel[PM_NUM_MICS] = {  0.00f, +0.42f, -0.15f, -0.55f };
        for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
            d->theta[i] = kStagDeg[i] * PM_DEG_TO_RAD;
            d->omega[i] = kStagVel[i];
        }
    }

    d->phys_rem = PM_PHYS_SUB;
    update_acoustic_matrix(d, true);
}

void pm_set_pendulum_state(reich_pendulum_dsp_t* d, uint32_t idx, float theta_rad, float omega_rad_s) {
    if (!d || idx >= PM_NUM_MICS) return;
    d->theta[idx] = clampf(theta_rad, -PM_LIMIT_RAD, PM_LIMIT_RAD);
    d->omega[idx] = clampf(omega_rad_s, -12.0f, 12.0f);
    d->phys_rem = PM_PHYS_SUB;
    update_acoustic_matrix(d, true);
}

bool pm_all_nearly_still(const reich_pendulum_dsp_t* d, float deg_thresh) {
    if (!d) return true;
    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        if (fabsf(d->deg[i]) > deg_thresh || fabsf(d->omega[i]) > 0.08f) {
            return false;
        }
    }
    return true;
}

/*
 * Synthesize one stereo frame for built-in voice `s` directly in registers
 * (~14 FPU ops, zero RAM reads, skipped when VCA closed).
 */
static inline pm_frame_t synth_voice_sample(const reich_pendulum_dsp_t* d, uint32_t s) {
    const uint8_t b = d->bank_idx;
    const uint32_t p0 = d->synth_ph[s][0];
    const uint32_t p1 = d->synth_ph[s][1];
    const uint32_t p2 = d->synth_ph[s][2];
    const uint32_t lp = d->lfo_ph[s];

    const float lfo = fast_sin_u32(lp);
    pm_frame_t res;

    if (b == PM_BANK_ORGANS) {
        const uint32_t vib = (uint32_t)((int32_t)(lfo * 4800000.0f));
        const float w0l = fast_sin_u32(p0 + vib);
        const float w0r = fast_sin_u32(p0 - vib);
        const float w1  = fast_sin_u32(p1 + (vib << 1));
        const float w2  = fast_sin_u32(p2 - (vib << 1));
        const float trem = 0.90f + 0.10f * lfo;
        res.l = (0.56f * w0l + 0.28f * w1 + 0.16f * w2) * trem;
        res.r = (0.56f * w0r + 0.26f * w1 + 0.18f * w2) * trem;
    } else if (b == PM_BANK_MARIMBA) {
        const float saw = (float)(lp >> 16) * (1.0f / 65536.0f);
        const float strike = 1.0f - saw;
        const float pulse = (s < 2u) ? (0.30f + 0.70f * strike * strike)
                                     : (0.64f + 0.36f * lfo);
        const float w0 = fast_sin_u32(p0);
        const float w1 = fast_sin_u32(p1) * strike;
        const float w2 = fast_sin_u32(p2) * strike * strike;
        const float mono = (0.68f * w0 + 0.22f * w1 + 0.10f * w2) * pulse;
        res.l = mono + 0.06f * w1;
        res.r = mono - 0.06f * w1;
    } else if (b == PM_BANK_CHOIR) {
        const uint32_t det = (uint32_t)((int32_t)(lfo * 7200000.0f));
        const float morph = 0.80f + 0.20f * lfo;
        const float w0l = fast_sin_u32(p0 + det);
        const float w0r = fast_sin_u32(p0 - det + 0x08000000u);
        const float w1  = fast_sin_u32(p1 - det) * morph;
        const float w2  = fast_sin_u32(p2 + det) * morph;
        res.l = 0.54f * w0l + 0.30f * w1 + 0.16f * w2;
        res.r = 0.54f * w0r + 0.28f * w1 + 0.18f * w2;
    } else {
        const float rim = 0.76f + 0.24f * lfo;
        const float w0 = fast_sin_u32(p0);
        const float w1 = fast_sin_u32(p1);
        const float w2 = fast_sin_u32(p2);
        const float core = (0.68f * w0 + 0.18f * w1 + 0.14f * w2) * rim;
        const float sp_mod = 0.12f * lfo;
        res.l = core * (0.85f + sp_mod);
        res.r = core * (0.85f - sp_mod);
    }
    return res;
}

/*
 * Fast block-fused 24 kHz renderer: processes `frames24` stereo frames into
 * interleaved `out24[0 .. 2*frames24 - 1]` with zero UI/audio races, zero
 * discontinuous tap steps, and internal-SRAM delay access.
 */
void pm_dsp_render_block24(reich_pendulum_dsp_t* restrict d,
                           float* restrict out24,
                           uint32_t frames24,
                           float play_target,
                           float play_step,
                           float* restrict io_play_gain,
                           float mode_target,
                           float mode_step,
                           float* restrict io_mode_mix,
                           pm_frame_t* const slot_buf[PM_NUM_MICS],
                           const volatile uint32_t slot_len[PM_NUM_MICS],
                           volatile uint32_t head_io[PM_NUM_MICS],
                           uint8_t custom_mask) {
    if (!d || !out24 || frames24 == 0u) return;

    if (d->voicing_dirty) {
        update_svf_voicing(d);
    }
    if (d->bank_dirty) {
        update_synth_bank_freqs(d);
    }

    float pg = *io_play_gain;
    float mm = *io_mode_mix;

    const float rate_scale = 24000.0f * d->inv_fs;
    const float lp_a       = clampf(0.56f * rate_scale, 0.25f, 0.70f);
    const float env_atk    = 0.0084f * rate_scale;
    const float env_rel    = 0.00104f * rate_scale;
    const float vca_slew   = 0.0064f * rate_scale;
    const float dc_r       = d->dc_r;

    pend_space_t* restrict sp = d->space;
    uint32_t wr  = d->spk_wr;
    uint32_t awr = d->ap_wr;
    uint32_t rwr = d->room_wr;
    uint32_t rng = d->rng_state ? d->rng_state : 0x19685245u;

    uint32_t local_len[PM_NUM_MICS]  = { 0u, 0u, 0u, 0u };
    uint32_t local_head[PM_NUM_MICS] = { 0u, 0u, 0u, 0u };
    uint8_t  valid_custom = 0u;
    if (custom_mask != 0u && slot_buf && slot_len && head_io) {
        for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
            if ((custom_mask & (1u << s)) != 0u && slot_buf[s] != NULL) {
                const uint32_t sl = slot_len[s];
                if (sl >= 8u) {
                    local_len[s] = sl;
                    uint32_t hi = head_io[s];
                    local_head[s] = (hi < sl) ? hi : 0u;
                    valid_custom |= (uint8_t)(1u << s);
                }
            }
        }
    }

    uint32_t rem = frames24;
    float* restrict dst = out24;

    while (rem > 0u) {
        if (d->phys_rem == 0u) {
            d->rng_state = rng;
            step_pendulums_verlet(d, (float)PM_PHYS_SUB * d->inv_fs);
            d->phys_rem = PM_PHYS_SUB;
        }

        const uint32_t chunk = (rem < d->phys_rem) ? rem : d->phys_rem;
        d->phys_rem = (uint8_t)(d->phys_rem - chunk);
        rem -= chunk;

        bool skipped_vca_all = true;

        for (uint32_t n = 0u; n < chunk; ++n) {
            if (pg < play_target) {
                pg += play_step;
                if (pg > play_target) pg = play_target;
            } else if (pg > play_target) {
                pg -= play_step;
                if (pg < play_target) pg = play_target;
            }

            if (mm < mode_target) {
                mm += mode_step;
                if (mm > mode_target) mm = mode_target;
            } else if (mm > mode_target) {
                mm -= mode_step;
                if (mm < mode_target) mm = mode_target;
            }

            /* 1. Continuous 3-load gallery speaker cross-air field (zero tap-step clicks!) */
            float air_cross[PM_NUM_MICS] = { 0.0f, 0.0f, 0.0f, 0.0f };
            if (sp) {
                const pm_quad_t q1 = sp->spk_ring[(wr - PM_BAY1_DELAY) & PM_SPK_DELAY_MASK];
                const pm_quad_t q2 = sp->spk_ring[(wr - PM_BAY2_DELAY) & PM_SPK_DELAY_MASK];
                const pm_quad_t q3 = sp->spk_ring[(wr - PM_BAY3_DELAY) & PM_SPK_DELAY_MASK];
                air_cross[0] = 0.54f * q1.ch[1] + 0.28f * q2.ch[2] + 0.16f * q3.ch[3];
                air_cross[1] = 0.48f * (q1.ch[0] + q1.ch[2]) + 0.26f * q2.ch[3];
                air_cross[2] = 0.48f * (q1.ch[1] + q1.ch[3]) + 0.26f * q2.ch[0];
                air_cross[3] = 0.54f * q1.ch[2] + 0.28f * q2.ch[1] + 0.16f * q3.ch[0];
            }

            const float room_air = next_noise_bi(&rng) * 0.00042f + 1.0e-20f;
            pm_quad_t next_q;
            float mix_l = 0.0f;
            float mix_r = 0.0f;

            /* 2. 4 swinging microphone -> speaker PA resonant feedback channels */
            for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
                const float sg = d->self_gain_cur[i]  + d->self_gain_inc[i];
                const float cg = d->cross_gain_cur[i] + d->cross_gain_inc[i];
                const float lg = d->loop_gain_cur[i]  + d->loop_gain_inc[i];
                const float sd = d->self_del_cur[i]   + d->self_del_inc[i];
                d->self_gain_cur[i]  = sg;
                d->cross_gain_cur[i] = cg;
                d->loop_gain_cur[i]  = lg;
                d->self_del_cur[i]   = sd;

                const float w_self = sp ? linear_read_quad(sp->spk_ring, wr, i, sd)
                                        : d->last_spk[i];

                const float self_nz = next_noise_bi(&rng) * 0.00026f;
                const float air_seed = room_air + self_nz + 0.00014f * d->omega[i];
                const float mic_in = sg * w_self + cg * air_cross[i] + air_seed;

                /* Paper-cone 1-pole cabinet rolloff + Cytomic ZDF bandpass SVF (zero DC gain) */
                const float lpy = d->lp_y[i] + lp_a * (mic_in - d->lp_y[i]);
                d->lp_y[i] = lpy;

                const float ic1 = d->svf_ic1[i];
                const float ic2 = d->svf_ic2[i];
                const float v3  = lpy - ic2;
                const float v1  = d->svf_a1[i] * ic1 + d->svf_a2[i] * v3;
                const float v2  = ic2 + d->svf_a2[i] * ic1 + d->svf_a3[i] * v3;
                d->svf_ic1[i] = 2.0f * v1 - ic1;
                d->svf_ic2[i] = 2.0f * v2 - ic2;

                const float pre = (d->svf_k_gain[i] * v1) * lg;
                const float spk_y = fast_pa_sat(pre);
                next_q.ch[i] = spk_y;
                d->last_spk[i] = spk_y;

                /* Per-mic post-interference feedback envelope follower */
                const float rect = clampf(fabsf(spk_y) * 1.14f, 0.0f, 1.0f);
                float er = d->env_raw[i];
                er += ((rect > er) ? env_atk : env_rel) * (rect - er);
                d->env_raw[i] = er;

                const float clean_env = (er > 0.0035f)
                                            ? clampf((er - 0.0035f) * (1.0f / 0.9965f), 0.0f, 1.0f)
                                            : 0.0f;
                d->env[i] = clean_env;
                float vg = d->vca_gain[i] + vca_slew * (clean_env - d->vca_gain[i]);
                if (vg < 1.0e-5f) vg = 0.0f;
                d->vca_gain[i] = vg;

                mix_l += spk_y * kSpkGainL[i];
                mix_r += spk_y * kSpkGainR[i];
            }

            /* 3. Single-cache-line speaker ring commit + stereo gallery room */
            if (sp) {
                wr = (wr + 1u) & PM_SPK_DELAY_MASK;
                sp->spk_ring[wr] = next_q;

                const float apl_d = sp->ap_l[(awr - 43u) & PM_AP_MASK];
                const float apr_d = sp->ap_r[(awr - 57u) & PM_AP_MASK];
                const float in_l = mix_l + 0.5f * apl_d;
                const float in_r = mix_r + 0.5f * apr_d;
                sp->ap_l[awr] = in_l;
                sp->ap_r[awr] = in_r;
                const float diff_l = apl_d - 0.5f * in_l;
                const float diff_r = apr_d - 0.5f * in_r;
                awr = (awr + 1u) & PM_AP_MASK;

                const float rl_raw = sp->room_l[(rwr - 193u) & PM_ROOM_MASK];
                const float rr_raw = sp->room_r[(rwr - 249u) & PM_ROOM_MASK];
                d->room_lp_l += 0.52f * (rl_raw - d->room_lp_l);
                d->room_lp_r += 0.52f * (rr_raw - d->room_lp_r);

                rwr = (rwr + 1u) & PM_ROOM_MASK;
                sp->room_l[rwr] = diff_l + 0.55f * d->room_lp_l - 0.22f * d->room_lp_r;
                sp->room_r[rwr] = diff_r + 0.55f * d->room_lp_r - 0.22f * d->room_lp_l;

                mix_l += 0.20f * d->room_lp_l;
                mix_r += 0.20f * d->room_lp_r;
            }

            const float y_l = mix_l - d->out_dc_xl + dc_r * d->out_dc_yl;
            d->out_dc_xl = mix_l;
            d->out_dc_yl = y_l;

            const float y_r = mix_r - d->out_dc_xr + dc_r * d->out_dc_yr;
            d->out_dc_xr = mix_r;
            d->out_dc_yr = y_r;

            const float fbk_l = pm_soft_limit(y_l);
            const float fbk_r = pm_soft_limit(y_r);

            /* 4. Fused Mode 2 ("SOUND VCA") only when audible (mm > 0.0001f) */
            float vca_l = 0.0f;
            float vca_r = 0.0f;
            if (mm > 0.0001f) {
                skipped_vca_all = false;
                float sum_l = 0.0f;
                float sum_r = 0.0f;
                for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
                    d->synth_ph[s][0] += d->synth_inc[s][0];
                    d->synth_ph[s][1] += d->synth_inc[s][1];
                    d->synth_ph[s][2] += d->synth_inc[s][2];
                    d->lfo_ph[s]      += d->lfo_inc[s];

                    const float g = d->vca_gain[s];
                    pm_frame_t src;
                    if ((valid_custom & (1u << s)) != 0u) {
                        uint32_t hi = local_head[s];
                        src = slot_buf[s][hi];
                        hi++;
                        if (hi >= local_len[s]) hi = 0u;
                        local_head[s] = hi;
                    } else {
                        if (g <= 1.0e-5f) continue;
                        src = synth_voice_sample(d, s);
                    }

                    if (g > 1.0e-5f) {
                        const float v = g * 0.34f;
                        sum_l += src.l * v * d->vca_pan_l[s];
                        sum_r += src.r * v * d->vca_pan_r[s];
                    }
                }
                vca_l = pm_soft_limit(sum_l);
                vca_r = pm_soft_limit(sum_r);
            }

            const float dry_mix = 1.0f - mm;
            dst[0] = (dry_mix * fbk_l + mm * vca_l) * pg;
            dst[1] = (dry_mix * fbk_r + mm * vca_r) * pg;
            dst += 2;
        }

        /* If pure Mode 1 for the entire sub-chunk, advance phase accumulators in O(1)! */
        if (skipped_vca_all) {
            for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
                d->synth_ph[s][0] += d->synth_inc[s][0] * chunk;
                d->synth_ph[s][1] += d->synth_inc[s][1] * chunk;
                d->synth_ph[s][2] += d->synth_inc[s][2] * chunk;
                d->lfo_ph[s]      += d->lfo_inc[s]      * chunk;
            }
        }
    }

    if (valid_custom != 0u && head_io) {
        for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
            if ((valid_custom & (1u << s)) != 0u) {
                head_io[s] = local_head[s];
            }
        }
    }

    d->spk_wr    = wr;
    d->ap_wr     = awr;
    d->room_wr   = rwr;
    d->rng_state = rng;
    *io_play_gain = pg;
    *io_mode_mix  = mm;
}

/* Single-sample wrappers for standalone unit tests */
void pm_dsp_step(reich_pendulum_dsp_t* d, pm_frame_t* out_fbk) {
    if (!d) {
        if (out_fbk) { out_fbk->l = 0.0f; out_fbk->r = 0.0f; }
        return;
    }
    float buf[2] = { 0.0f, 0.0f };
    float pg = 1.0f;
    float mm = 0.0f;
    pm_dsp_render_block24(d, buf, 1u, 1.0f, 0.0f, &pg, 0.0f, 0.0f, &mm,
                          NULL, NULL, NULL, 0u);
    if (out_fbk) {
        out_fbk->l = buf[0];
        out_fbk->r = buf[1];
    }
}

pm_frame_t pm_render_sound_vca(reich_pendulum_dsp_t* d,
                               const pm_frame_t custom_in[PM_NUM_MICS],
                               uint8_t custom_mask) {
    pm_frame_t out = { 0.0f, 0.0f };
    if (!d) return out;

    float sum_l = 0.0f;
    float sum_r = 0.0f;
    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        d->synth_ph[i][0] += d->synth_inc[i][0];
        d->synth_ph[i][1] += d->synth_inc[i][1];
        d->synth_ph[i][2] += d->synth_inc[i][2];
        d->lfo_ph[i]      += d->lfo_inc[i];

        const float g = d->vca_gain[i];
        if (g <= 1.0e-5f) continue;

        pm_frame_t src;
        if ((custom_mask & (1u << i)) != 0u && custom_in != NULL) {
            src = custom_in[i];
        } else {
            src = synth_voice_sample(d, i);
        }
        const float v = g * 0.34f;
        sum_l += src.l * v * d->vca_pan_l[i];
        sum_r += src.r * v * d->vca_pan_r[i];
    }
    out.l = pm_soft_limit(sum_l);
    out.r = pm_soft_limit(sum_r);
    return out;
}
