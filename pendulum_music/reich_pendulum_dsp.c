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
#define PM_INV_TWO_PI       0.15915494309189533577f
#define PM_DEG_TO_RAD       0.01745329251994329577f
#define PM_RAD_TO_DEG       57.2957795130823208768f
#define PM_LIMIT_RAD        1.04719755f             /* 60 deg safety ceiling */
#define PM_RELEASE_RAD      0.80285146f             /* 46.0 deg Reich pull-back angle */
#define PM_BASE_CABLE_M     2.25f                   /* ~3.01 s base pendulum period */
#define PM_H_GAP_M          0.14f                   /* 14 cm mic-to-cone clearance at nadir */
#define PM_SHARED_CONE_HZ   422.33f                 /* 1.0 wavelength across 0.80m speaker spacing */
#define PM_DENORM_FLOOR     1.0e-18f
#define PM_PHYS_SUB         32u                     /* 1.5 kHz symplectic Verlet rate at 48 kHz */

/* 4 upward-facing floor speaker X coordinates along the gallery stage (meters) */
static const float kSpkX[PM_NUM_MICS] = { -1.20f, -0.40f, +0.40f, +1.20f };

/* Stereo stage panning positions for Mode 1 floor speakers */
static const float kSpkPan[PM_NUM_MICS] = { -0.82f, -0.27f, +0.27f, +0.82f };

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

static inline float flush_denorm(float x) {
    return (fabsf(x) < PM_DENORM_FLOOR) ? 0.0f : x;
}

/*
 * Fast inline minimax sin/cos on [-pi, +pi] (zero external libm calls in audio ISR!).
 * Accurate to < 2e-6 across [-1.6, +1.6] rad (pendulums bounded to +-1.05 rad).
 */
static inline float fast_sin_rad(float x) {
    float k = roundf(x * PM_INV_TWO_PI);
    float r = x - k * PM_TWO_PI;
    if (r > 1.57079633f)       r =  3.14159265f - r;
    else if (r < -1.57079633f) r = -3.14159265f - r;
    const float r2 = r * r;
    return r * (1.0f - r2 * (0.16666657f - r2 * (0.00833216f - r2 * 0.00019515f)));
}

static inline float fast_cos_rad(float x) {
    return fast_sin_rad(x + 1.57079633f);
}

/*
 * Ultra-fast register-only sine from 32-bit unsigned phase [0 .. 2^32 - 1].
 * 4 FPU ops, zero memory loads, < 0.0009 peak error.
 */
static inline float fast_sin_u32(uint32_t ph) {
    /* Map ph to signed [-1.0f, +1.0f) */
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
    if (x == 0u) x = 0x9E3779B9u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    const int32_t s = (int32_t)(x >> 9) - 4194304;
    return (float)s * (1.0f / 4194304.0f);
}

float pm_clip_guard(float x) {
    if (!(x > -4.0f && x < 4.0f)) return 0.0f;
    return clampf(x, -1.25f, 1.25f);
}

/*
 * Zero-division, zero-sqrt C1 cubic PA saturator:
 * Exact small-signal slope f'(0) = 1.0000 (preserves Barkhausen oscillation threshold!)
 * with smooth C1 tube-rail ceiling +-0.86f so constructive in-phase surges have
 * natural dynamic headroom above single/out-of-phase swings.
 */
static inline float fast_pa_sat(float x) {
    const float u = clampf(x * (1.0f / 0.86f), -1.5f, 1.5f);
    return 0.86f * u * (1.0f - (4.0f / 27.0f) * u * u);
}

/*
 * Zero-division C1 cubic output soft limiter with exact ceiling +-0.98f.
 */
float pm_soft_limit(float x) {
    if (!(x > -8.0f && x < 8.0f)) {
        return (x >= 0.0f) ? 0.98f : -0.98f;
    }
    const float c = clampf(x, -1.47f, 1.47f); /* 1.5 * 0.98 = 1.47 */
    return c * (1.0f - 0.15429503f * c * c);  /* 4 / (27 * 0.98^2) */
}

/*
 * 4-point, 3rd-order Hermite cubic fractional delay reader for primary self-paths.
 * Uses single-cycle `(int32_t)d` since `d >= 2.0f > 0.0f`.
 */
static inline float hermite_read_ring(const float* buf, uint32_t wr, uint32_t mask, float delay_samp) {
    const float d = clampf(delay_samp, 2.0f, (float)(mask - 4u));
    const int32_t di = (int32_t)d;
    const float f = d - (float)di;

    const uint32_t udi  = (uint32_t)di;
    const uint32_t i_m1 = (wr - udi + 1u) & mask;
    const uint32_t i_0  = (wr - udi)      & mask;
    const uint32_t i_1  = (wr - udi - 1u) & mask;
    const uint32_t i_2  = (wr - udi - 2u) & mask;

    const float ym1 = buf[i_m1];
    const float y0  = buf[i_0];
    const float y1  = buf[i_1];
    const float y2  = buf[i_2];

    const float c1 = 0.5f * (y1 - ym1);
    const float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);

    return ((c3 * f + c2) * f + c1) * f + y0;
}

/*
 * Fast 2-point linear fractional delay reader for cross-speaker room air bleed.
 */
static inline float linear_read_ring(const float* buf, uint32_t wr, uint32_t mask, float delay_samp) {
    const float d = clampf(delay_samp, 1.0f, (float)(mask - 2u));
    const int32_t di = (int32_t)d;
    const float f = d - (float)di;
    const uint32_t i0 = (wr - (uint32_t)di)      & mask;
    const uint32_t i1 = (wr - (uint32_t)di - 1u) & mask;
    const float y0 = buf[i0];
    return y0 + f * (buf[i1] - y0);
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

static void update_cable_targets(reich_pendulum_dsp_t* d, bool snap) {
    const float sp = clampf(d->spread_norm, 0.0f, 1.0f);
    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const float L = PM_BASE_CABLE_M * (1.0f + sp * kCableOffsets[i]);
        d->cable_tgt[i] = clampf(L, 1.60f, 3.10f);
        if (snap || d->cable_len[i] < 1.0f) {
            d->cable_len[i] = d->cable_tgt[i];
        }
    }
}

static void update_synth_bank_freqs(reich_pendulum_dsp_t* d) {
    const uint8_t b = d->bank_idx % PM_BANK_COUNT;
    const float inv_fs = d->inv_fs;

    /* Base frequencies & overtone ratios for the 4 built-in Reichian sound banks */
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
        { 1.25f, 1.65f, 2.10f, 2.55f },   /* Organ rotary / chorus shimmer */
        { 4.15f, 5.00f, 3.35f, 2.50f },   /* Marimba mallet roll / vibe motor */
        { 0.85f, 1.15f, 1.45f, 1.80f },   /* Choir tape wow / vowel breathe */
        { 0.90f, 1.20f, 1.55f, 1.95f },   /* Singing glass rim undulation */
    };

    for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
        const float f0 = kBankBaseHz[b][s];
        float m1 = 2.00f;
        float m2 = 3.00f;
        if (b == PM_BANK_ORGANS) {
            m1 = (s == 1u) ? 3.00f : 2.00f;
            m2 = (s == 3u) ? 1.50f : 4.00f;
        } else if (b == PM_BANK_MARIMBA) {
            m1 = (s < 2u) ? 3.98f : 2.00f;  /* Rosewood bar 4x overtone / bell octave */
            m2 = (s == 3u) ? 1.50f : 2.99f;
        } else if (b == PM_BANK_CHOIR) {
            m1 = 2.003f;                    /* Warm detuned ensemble octave */
            m2 = 3.005f;                    /* Formant twelfth */
        } else {
            m1 = 2.000f;
            m2 = 2.320f;                    /* Wine-glass (3,0) crystalline rim mode */
        }

        d->synth_inc[s][0] = hz_to_phase_inc(f0, inv_fs);
        d->synth_inc[s][1] = hz_to_phase_inc(f0 * m1, inv_fs);
        d->synth_inc[s][2] = hz_to_phase_inc(f0 * m2, inv_fs);
        d->lfo_inc[s]      = hz_to_phase_inc(kLfoHz[b][s], inv_fs);
    }
}

static void update_svf_voicing(reich_pendulum_dsp_t* d) {
    const uint8_t tm = d->tuning_mode % PM_TUNING_COUNT;
    const float q_voice = (tm == PM_TUNING_RAW_1968) ? 4.4f : 3.8f;
    const float k_voice = 1.0f / q_voice;

    const float q_cone = 3.0f;
    const float k_cone = 1.0f / q_cone;
    const float gc = tanf(M_PI * PM_SHARED_CONE_HZ * d->inv_fs);
    const float ca1 = 1.0f / (1.0f + gc * (gc + k_cone));
    d->svf2_g  = gc;
    d->svf2_k  = k_cone;
    d->svf2_a1 = ca1;
    d->svf2_a2 = gc * ca1;
    d->svf2_a3 = gc * gc * ca1;

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const float fc = kTuningFreqs[tm][i];
        const float g = tanf(M_PI * fc * d->inv_fs);
        const float a1 = 1.0f / (1.0f + g * (g + k_voice));

        const float tau0 = 14.0f;
        d->tau_trim[i] = tau0;

        /* Closed-loop Barkhausen unity-gain threshold equalization across M1..M4 */
        const float r_hp = 85.0f / fc;
        const float r_lp = fc / 3200.0f;
        const float mag_pb = 1.0f / sqrtf((1.0f + r_hp * r_hp) * (1.0f + r_lp * r_lp));
        const float phi_lag = PM_TWO_PI * fc * tau0 * d->inv_fs +
                              atan2f(r_lp, 1.0f) - atan2f(r_hp, 1.0f);
        const float cos_lag = fmaxf(0.42f, cosf(clampf(phi_lag, -1.15f, 1.15f)));

        const float det = q_cone * (fc / PM_SHARED_CONE_HZ - PM_SHARED_CONE_HZ / fc);
        const float cone_re = 1.0f / (1.0f + det * det);
        const float ch_comp = 1.0f / fmaxf(0.30f, mag_pb * cos_lag * (0.82f + 0.18f * cone_re));

        d->svf1_g[i]  = g;
        d->svf1_k[i]  = k_voice * ch_comp;
        d->svf1_a1[i] = a1;
        d->svf1_a2[i] = g * a1;
        d->svf1_a3[i] = g * g * a1;
    }
}

/*
 * Recompute 3D capsule positions, all 16 speaker-to-mic distances & delays,
 * precomputed VCA stereo pan gains, and inter-mic phase coherence (at 1.5 kHz).
 * Uses inline `fast_sin_rad` / `fast_cos_rad` so ZERO external libm calls run in audio!
 */
static void update_acoustic_matrix(reich_pendulum_dsp_t* d, bool snap) {
    const float samp_per_m = d->sample_rate * (1.0f / PM_SOUND_SPEED);
    const float pa_base = (0.88f + 1.95f * clampf(d->gain_norm, 0.0f, 1.0f)) * d->pa_power;
    const float cpl_amt = clampf(d->couple_norm, 0.0f, 1.0f);

    float g_self[PM_NUM_MICS];
    float r_amp[PM_NUM_MICS];

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const float L = d->cable_len[i];
        const float th = d->theta[i];
        const float st = fast_sin_rad(th);
        const float ct = fast_cos_rad(th);
        const float w0 = sqrtf(PM_GRAVITY / L);

        const float dx_self = L * st;
        const float px = kSpkX[i] + dx_self;
        const float pz = PM_H_GAP_M + L * (1.0f - ct);

        d->px[i] = px;
        d->pz[i] = pz;
        d->deg[i] = th * PM_RAD_TO_DEG;

        /* Precompute constant-power Mode 2 pan gains here (1.5 kHz) instead of 48 kHz! */
        const float pan = clampf(px * 0.52f, -0.86f, 0.86f);
        const float ang = (pan + 1.0f) * (0.25f * M_PI);
        d->vca_pan_l[i] = fast_cos_rad(ang);
        d->vca_pan_r[i] = fast_sin_rad(ang);

        const float u = st;
        const float v = d->omega[i] / fmaxf(0.5f, w0);
        d->norm_u[i] = u;
        d->norm_v[i] = v;
        r_amp[i] = sqrtf(u * u + v * v);

        const float beam = 1.0f / (1.0f + (dx_self * 2.15f) * (dx_self * 2.15f) +
                                   2.10f * (pz - PM_H_GAP_M));
        g_self[i] = beam;
        d->self_gain_tgt[i] = beam;
    }

    float sync_num = 0.0f;
    float sync_den = 0.0f;
    const float max_del = (float)(PM_SPK_DELAY_LEN - 8u);

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        float interf_sum = 0.0f;

        for (uint32_t j = 0u; j < PM_NUM_MICS; ++j) {
            const float dx = d->px[i] - kSpkX[j];
            const float dz = d->pz[i];
            const float dist = sqrtf(dx * dx + dz * dz);

            if (i == j) {
                const float extra_m = fmaxf(0.0f, dist - PM_H_GAP_M);
                const float del = d->tau_trim[i] + extra_m * samp_per_m;
                d->del_tgt[i][i] = clampf(del, 8.0f, max_del);
                d->cpl_tgt[i][i] = 0.0f;
            } else {
                const float del = 16.0f + dist * samp_per_m;
                d->del_tgt[i][j] = clampf(del, 8.0f, max_del);

                const float prod_r = r_amp[i] * r_amp[j];
                const float act = clampf(prod_r * 14.0f, 0.0f, 1.0f);
                const float raw_cos = (d->norm_u[i] * d->norm_u[j] +
                                       d->norm_v[i] * d->norm_v[j]) /
                                      (prod_r + 1.0e-5f);
                const float coh = act * clampf(raw_cos, -1.0f, 1.0f) + (1.0f - act) * 0.24f;

                const float excess_d = fmaxf(0.0f, dist - 0.75f);
                const float dist_w = 1.0f / (1.0f + 0.95f * excess_d);

                d->cpl_tgt[i][j] = cpl_amt * 0.30f * dist_w *
                                   (0.16f + 0.84f * g_self[i]) * coh;

                const float pair_act = g_self[i] * g_self[j] * dist_w;
                interf_sum += cpl_amt * 0.36f * pair_act * coh;

                if (j == i + 1u) {
                    d->pair_coh[i] = coh * cpl_amt;
                }
                if (j > i) {
                    const float w_pair = 0.25f + pair_act;
                    sync_num += coh * w_pair;
                    sync_den += w_pair;
                }
            }
        }

        const float mod = clampf(1.0f + interf_sum, 0.62f, 1.52f);
        d->loop_gain_tgt[i] = pa_base * mod;

        if (snap) {
            d->self_gain_cur[i] = d->self_gain_tgt[i];
            d->loop_gain_cur[i] = d->loop_gain_tgt[i];
            for (uint32_t j = 0u; j < PM_NUM_MICS; ++j) {
                d->del_cur[i][j] = d->del_tgt[i][j];
                d->cpl_cur[i][j] = d->cpl_tgt[i][j];
            }
        }
    }

    const float target_sync = (sync_den > 1.0e-4f)
                                  ? clampf((sync_num / sync_den) * cpl_amt, -1.0f, 1.0f)
                                  : 0.0f;
    d->phase_sync += 0.18f * (target_sync - d->phase_sync);
}

static void step_pendulums_verlet(reich_pendulum_dsp_t* d, float dt) {
    if (dt <= 0.0f || dt > 0.05f) return;

    const float dec = clampf(d->decay_norm, 0.0f, 1.0f);
    const float gamma = (dec < 0.005f) ? 0.0f : (dec * dec * 0.44f + dec * 0.055f);
    const float cable_slew = clampf(5.0f * dt, 0.0f, 1.0f);

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        d->cable_len[i] += cable_slew * (d->cable_tgt[i] - d->cable_len[i]);
        const float L = clampf(d->cable_len[i], 1.60f, 3.10f);
        const float g_L = PM_GRAVITY / L;

        const float th0 = d->theta[i];
        const float om0 = d->omega[i];

        const float a0 = -g_L * fast_sin_rad(th0) - gamma * om0;
        const float om_half = om0 + 0.5f * dt * a0;
        float th1 = clampf(th0 + dt * om_half, -PM_LIMIT_RAD, PM_LIMIT_RAD);
        const float a1 = -g_L * fast_sin_rad(th1) - gamma * om_half;
        float om1 = om_half + 0.5f * dt * a1;

        if (fabsf(th1) >= PM_LIMIT_RAD && (th1 * om1) > 0.0f) {
            om1 = 0.0f;
        }

        d->theta[i] = flush_denorm(th1);
        d->omega[i] = flush_denorm(om1);
    }

    update_acoustic_matrix(d, false);
}

void pm_dsp_init(reich_pendulum_dsp_t* d, float sample_rate) {
    if (!d) return;
    memset(d, 0, sizeof(*d));

    d->sample_rate = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    d->inv_fs = 1.0f / d->sample_rate;
    d->dc_r = clampf(1.0f - (PM_TWO_PI * 18.0f * d->inv_fs), 0.990f, 0.9995f);
    d->rng_state = 0x19685245u;

    d->gain_norm    = 0.58f;
    d->couple_norm  = 0.50f;
    d->spread_norm  = 0.42f;
    d->decay_norm   = 0.35f;
    d->tuning_mode  = PM_TUNING_FIFTHS;
    d->bank_idx     = PM_BANK_ORGANS;
    d->release_stagger = false;
    d->plug_pulled  = false;
    d->pa_power     = 1.0f;
    d->pa_power_tgt = 1.0f;

    /* Stagger initial voice phases so voices never sum with coherent transient click */
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
        d->hp_x[i] = 0.0f;
        d->hp_y[i] = 0.0f;
        d->lp_y[i] = 0.0f;
        d->svf1_ic1[i] = 0.0f;
        d->svf1_ic2[i] = 0.0f;
        d->svf2_ic1[i] = 0.0f;
        d->svf2_ic2[i] = 0.0f;
        d->dc_x[i] = 0.0f;
        d->dc_y[i] = 0.0f;
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
    update_acoustic_matrix(d, true);
}

void pm_set_gain(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->gain_norm = clampf(norm, 0.0f, 1.0f);
    update_acoustic_matrix(d, false);
}

void pm_set_couple(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->couple_norm = clampf(norm, 0.0f, 1.0f);
    update_acoustic_matrix(d, false);
}

void pm_set_spread(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->spread_norm = clampf(norm, 0.0f, 1.0f);
    update_cable_targets(d, false);
}

void pm_set_decay(reich_pendulum_dsp_t* d, float norm) {
    if (!d) return;
    d->decay_norm = clampf(norm, 0.0f, 1.0f);
}

void pm_set_tuning(reich_pendulum_dsp_t* d, uint8_t tuning_mode) {
    if (!d) return;
    d->tuning_mode = tuning_mode % PM_TUNING_COUNT;
    update_svf_voicing(d);
    update_acoustic_matrix(d, false);
}

void pm_set_bank(reich_pendulum_dsp_t* d, uint8_t bank_idx) {
    if (!d) return;
    d->bank_idx = bank_idx % PM_BANK_COUNT;
    update_synth_bank_freqs(d);
}

void pm_set_release_stagger(reich_pendulum_dsp_t* d, bool stagger) {
    if (!d) return;
    d->release_stagger = stagger;
}

void pm_pull_and_release(reich_pendulum_dsp_t* d) {
    if (!d) return;
    d->plug_pulled = false;
    d->pa_power_tgt = 1.0f;
    if (d->pa_power < 0.15f) d->pa_power = 0.15f;

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

    update_acoustic_matrix(d, true);
}

void pm_set_pendulum_state(reich_pendulum_dsp_t* d, uint32_t idx, float theta_rad, float omega_rad_s) {
    if (!d || idx >= PM_NUM_MICS) return;
    d->theta[idx] = clampf(theta_rad, -PM_LIMIT_RAD, PM_LIMIT_RAD);
    d->omega[idx] = clampf(omega_rad_s, -12.0f, 12.0f);
    update_acoustic_matrix(d, true);
}

void pm_pull_plug(reich_pendulum_dsp_t* d) {
    if (!d) return;
    d->plug_pulled = true;
    d->pa_power_tgt = 0.0f;
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
 * Step 4-pendulum Verlet kinematics + 4x4 coupled acoustic feedback matrix.
 * Zero divisions, zero sqrtf, and zero external libm calls in the 48 kHz loop!
 */
void pm_dsp_step(reich_pendulum_dsp_t* d, pm_frame_t* out_fbk) {
    if (!d) {
        if (out_fbk) { out_fbk->l = 0.0f; out_fbk->r = 0.0f; }
        return;
    }

    if (++d->phys_sub >= PM_PHYS_SUB) {
        d->phys_sub = 0u;
        step_pendulums_verlet(d, (float)PM_PHYS_SUB * d->inv_fs);
    }

    const float pwr_slew = (d->pa_power_tgt > d->pa_power) ? 0.0025f : 0.000075f;
    d->pa_power += pwr_slew * (d->pa_power_tgt - d->pa_power);
    if (d->pa_power < 1.0e-5f) d->pa_power = 0.0f;

    pend_space_t* sp = d->space;
    const uint32_t wr = d->spk_wr;
    const bool has_couple = (sp != NULL) && (d->couple_norm > 0.002f);

    const float hp_r = 0.989f;
    const float lp_a = 0.325f;
    const float env_atk = 0.0042f;
    const float env_rel = 0.00052f;
    const float param_slew = 0.0035f;

    float new_spk[PM_NUM_MICS];
    float mix_l = 0.0f;
    float mix_r = 0.0f;

    const float room_air = next_noise_bi(&d->rng_state) * 0.00038f;

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        d->self_gain_cur[i] += param_slew * (d->self_gain_tgt[i] - d->self_gain_cur[i]);
        d->loop_gain_cur[i] += param_slew * (d->loop_gain_tgt[i] - d->loop_gain_cur[i]);
        d->del_cur[i][i]    += param_slew * (d->del_tgt[i][i]    - d->del_cur[i][i]);

        /* 1. Self-speaker path: 4-point Hermite cubic interpolation for smooth pitch glide */
        const float w_self = sp ? hermite_read_ring(sp->spk_line[i], wr, PM_SPK_DELAY_MASK,
                                                    d->del_cur[i][i])
                                : d->last_spk[i];

        /* 2. Cross-speaker room bleed (j != i): fast 2-point linear interpolation */
        float cross_sum = 0.0f;
        if (has_couple) {
            for (uint32_t j = 0u; j < PM_NUM_MICS; ++j) {
                if (j == i) continue;
                d->del_cur[i][j] += param_slew * (d->del_tgt[i][j] - d->del_cur[i][j]);
                d->cpl_cur[i][j] += param_slew * (d->cpl_tgt[i][j] - d->cpl_cur[i][j]);
                const float c = d->cpl_cur[i][j];
                if (fabsf(c) > 1.0e-4f) {
                    cross_sum += c * linear_read_ring(sp->spk_line[j], wr, PM_SPK_DELAY_MASK,
                                                      d->del_cur[i][j]);
                }
            }
        }

        /* 3. Capsule thermal noise + swing-air excitation */
        const float self_noise = next_noise_bi(&d->rng_state) * 0.00028f;
        const float air_seed = (room_air + self_noise + 0.00014f * d->omega[i]) * d->pa_power;

        const float mic_in = d->self_gain_cur[i] * w_self + cross_sum + air_seed;

        /* 4. Speaker cabinet passband (85 Hz HP -> 3.0 kHz LP) */
        const float hpy = flush_denorm(mic_in - d->hp_x[i] + hp_r * d->hp_y[i]);
        d->hp_x[i] = mic_in;
        d->hp_y[i] = hpy;
        const float lpy = flush_denorm(d->lp_y[i] + lp_a * (hpy - d->lp_y[i]));
        d->lp_y[i] = lpy;

        /* 5. 2-mode Cytomic ZDF SVF resonant voicing */
        float ic1 = d->svf1_ic1[i];
        float ic2 = d->svf1_ic2[i];
        const float v3 = lpy - ic2;
        const float v1 = d->svf1_a1[i] * ic1 + d->svf1_a2[i] * v3;
        const float v2 = ic2 + d->svf1_a2[i] * ic1 + d->svf1_a3[i] * v3;
        d->svf1_ic1[i] = flush_denorm(2.0f * v1 - ic1);
        d->svf1_ic2[i] = flush_denorm(2.0f * v2 - ic2);
        const float bp_voice = d->svf1_k[i] * v1;

        float c_ic1 = d->svf2_ic1[i];
        float c_ic2 = d->svf2_ic2[i];
        const float cv3 = lpy - c_ic2;
        const float cv1 = d->svf2_a1 * c_ic1 + d->svf2_a2 * cv3;
        const float cv2 = c_ic2 + d->svf2_a2 * c_ic1 + d->svf2_a3 * cv3;
        d->svf2_ic1[i] = flush_denorm(2.0f * cv1 - c_ic1);
        d->svf2_ic2[i] = flush_denorm(2.0f * cv2 - c_ic2);
        const float bp_cone = d->svf2_k * cv1;

        const float voiced = 0.82f * bp_voice + 0.18f * bp_cone;

        /* 6. PA Preamp gain + 18 Hz DC blocker + zero-division C1 cubic PA saturator */
        const float pre = voiced * d->loop_gain_cur[i];
        const float dcy = flush_denorm(pre - d->dc_x[i] + d->dc_r * d->dc_y[i]);
        d->dc_x[i] = pre;
        d->dc_y[i] = dcy;

        const float spk_y = d->pa_power * fast_pa_sat(dcy);
        new_spk[i] = spk_y;
        d->last_spk[i] = spk_y;

        /* 7. Per-mic post-interference feedback envelope follower [0.0f .. 1.0f] */
        const float rect = clampf(fabsf(spk_y) * 1.12f, 0.0f, 1.0f);
        float er = d->env_raw[i];
        er += ((rect > er) ? env_atk : env_rel) * (rect - er);
        er = flush_denorm(er);
        d->env_raw[i] = er;

        const float clean_env = (er > 0.0035f)
                                    ? clampf((er - 0.0035f) * (1.0f / 0.9965f), 0.0f, 1.0f)
                                    : 0.0f;
        d->env[i] = clean_env;
        d->vca_gain[i] += 0.0032f * (clean_env - d->vca_gain[i]);
        if (d->vca_gain[i] < 1.0e-5f) d->vca_gain[i] = 0.0f;

        /* 8. Stereo gallery panning of the 4 floor speakers */
        const float pan = kSpkPan[i];
        mix_l += spk_y * (0.31f * (1.0f - pan));
        mix_r += spk_y * (0.31f * (1.0f + pan));
    }

    /* Commit speaker outputs & compact L1-cache-resident stereo gallery room */
    if (sp) {
        const uint32_t next_wr = (wr + 1u) & PM_SPK_DELAY_MASK;
        sp->spk_line[0][next_wr] = new_spk[0];
        sp->spk_line[1][next_wr] = new_spk[1];
        sp->spk_line[2][next_wr] = new_spk[2];
        sp->spk_line[3][next_wr] = new_spk[3];
        d->spk_wr = next_wr;

        const uint32_t awr = d->ap_wr;
        const float apl_d = sp->ap_l[(awr - 89u) & PM_AP_MASK];
        const float apr_d = sp->ap_r[(awr - 113u) & PM_AP_MASK];
        const float in_l = mix_l + 0.5f * apl_d;
        const float in_r = mix_r + 0.5f * apr_d;
        sp->ap_l[awr] = flush_denorm(in_l);
        sp->ap_r[awr] = flush_denorm(in_r);
        const float diff_l = apl_d - 0.5f * in_l;
        const float diff_r = apr_d - 0.5f * in_r;
        d->ap_wr = (awr + 1u) & PM_AP_MASK;

        const uint32_t rwr = d->room_wr;
        const float rl_raw = sp->room_l[(rwr - 389u) & PM_ROOM_MASK];
        const float rr_raw = sp->room_r[(rwr - 499u) & PM_ROOM_MASK];
        d->room_lp_l = flush_denorm(d->room_lp_l + 0.34f * (rl_raw - d->room_lp_l));
        d->room_lp_r = flush_denorm(d->room_lp_r + 0.34f * (rr_raw - d->room_lp_r));

        const uint32_t n_rwr = (rwr + 1u) & PM_ROOM_MASK;
        sp->room_l[n_rwr] = flush_denorm(diff_l + 0.56f * d->room_lp_l - 0.22f * d->room_lp_r);
        sp->room_r[n_rwr] = flush_denorm(diff_r + 0.56f * d->room_lp_r - 0.22f * d->room_lp_l);
        d->room_wr = n_rwr;

        mix_l += 0.20f * d->room_lp_l;
        mix_r += 0.20f * d->room_lp_r;
    }

    const float y_l = flush_denorm(mix_l - d->out_dc_xl + d->dc_r * d->out_dc_yl);
    d->out_dc_xl = mix_l;
    d->out_dc_yl = y_l;

    const float y_r = flush_denorm(mix_r - d->out_dc_xr + d->dc_r * d->out_dc_yr);
    d->out_dc_xr = mix_r;
    d->out_dc_yr = y_r;

    if (out_fbk) {
        out_fbk->l = pm_soft_limit(y_l);
        out_fbk->r = pm_soft_limit(y_r);
    }
}

/*
 * Synthesize one stereo frame for built-in voice `s` directly in registers
 * (zero PSRAM reads, zero stack arrays, ~15 FPU ops, skipped when VCA closed!).
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
        /* Farfisa / reed stop with subtle stereo rotary phase offset */
        const uint32_t vib = (uint32_t)((int32_t)(lfo * 4800000.0f));
        const float w0l = fast_sin_u32(p0 + vib);
        const float w0r = fast_sin_u32(p0 - vib);
        const float w1  = fast_sin_u32(p1 + (vib << 1));
        const float w2  = fast_sin_u32(p2 - (vib << 1));
        const float trem = 0.90f + 0.10f * lfo;
        res.l = (0.56f * w0l + 0.28f * w1 + 0.16f * w2) * trem;
        res.r = (0.56f * w0r + 0.26f * w1 + 0.18f * w2) * trem;
    } else if (b == PM_BANK_MARIMBA) {
        /* Mallet tremolo / vibraphone motor pulsation */
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
        /* Warm cello + vocal vowel formant chorus */
        const uint32_t det = (uint32_t)((int32_t)(lfo * 7200000.0f));
        const float morph = 0.80f + 0.20f * lfo;
        const float w0l = fast_sin_u32(p0 + det);
        const float w0r = fast_sin_u32(p0 - det + 0x08000000u);
        const float w1  = fast_sin_u32(p1 - det) * morph;
        const float w2  = fast_sin_u32(p2 + det) * morph;
        res.l = 0.54f * w0l + 0.30f * w1 + 0.16f * w2;
        res.r = 0.54f * w0r + 0.28f * w1 + 0.18f * w2;
    } else {
        /* Bowed Glass armonica with (3,0) crystalline rim shimmer */
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

pm_frame_t pm_render_sound_vca(reich_pendulum_dsp_t* d,
                               const pm_frame_t custom_in[PM_NUM_MICS],
                               uint8_t custom_mask) {
    pm_frame_t out = { 0.0f, 0.0f };
    if (!d) return out;

    float sum_l = 0.0f;
    float sum_r = 0.0f;

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        /* Always advance integer phase accumulators (4 cycles) so rhythm stays phase-locked */
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
