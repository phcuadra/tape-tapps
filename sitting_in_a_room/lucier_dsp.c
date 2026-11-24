#include "lucier_dsp.h"

#ifdef LUCIER_HOST_TEST
#include <math.h>
#include <string.h>
#undef M_PI
#define M_PI 3.14159265358979323846f
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
#else
#include "tapp_api.h"
#endif

#define LUCIER_MIN_PASS       0.005f
#define LUCIER_DENORM_FLOOR   1.0e-15f
#define LUCIER_SILENCE_FLOOR  1.0e-6f
#define LUCIER_MAX_STEP_FR    8

/*
 * Four architectural room modal ratio tables (8 axial/tangential eigenmodes each):
 *   0: 1969 Room     — Bolt/Sepmeyer asymmetric living/lecture room (1 : 1.28 : 1.54)
 *   1: Tiled Chamber — Tight reflective alcove with quartal/minor-7th standing waves
 *   2: Stone Chapel  — Pure harmonic architectural proportions (fifth, maj7, octave, twelfth)
 *   3: Cistern       — Deep cylindrical concrete space with Bessel-like incommensurate modes
 */
static const float kGeomRatios[LUCIER_GEOM_COUNT][LUCIER_NUM_MODES] = {
    { 1.000f, 1.281f, 1.538f, 1.789f, 2.000f, 2.354f, 2.692f, 3.162f },
    { 1.000f, 1.189f, 1.498f, 1.782f, 2.245f, 2.667f, 2.996f, 3.564f },
    { 1.000f, 1.335f, 1.498f, 1.875f, 2.000f, 2.500f, 2.996f, 3.750f },
    { 1.000f, 1.382f, 1.839f, 2.145f, 2.618f, 3.078f, 3.646f, 4.236f },
};

/* Natural room modal coupling weights (mid-band modes couple slightly stronger than extremes) */
static const float kBaseWeights[LUCIER_NUM_MODES] = {
    0.88f, 0.96f, 1.00f, 0.98f, 0.94f, 0.88f, 0.80f, 0.72f
};

/* Subtle stereo microphone spatial asymmetry (~0 to +/-5.5 cents) for 3D standing-wave width */
static const float kStereoDetuneL[LUCIER_NUM_MODES] = {
    1.0000f, 0.9986f, 1.0018f, 0.9979f, 1.0024f, 0.9974f, 1.0029f, 0.9971f
};
static const float kStereoDetuneR[LUCIER_NUM_MODES] = {
    1.0000f, 1.0014f, 0.9982f, 1.0021f, 0.9976f, 1.0026f, 0.9971f, 1.0029f
};

static const char* const kGeomNames[LUCIER_GEOM_COUNT] = {
    "1969 Room",
    "Tiled Chamber",
    "Stone Chapel",
    "Cistern",
};

static const char* const kGeomShortNames[LUCIER_GEOM_COUNT] = {
    "ROOM",
    "TILE",
    "CHAPEL",
    "CISTRN",
};

static inline float luc_flush(float x) {
    return (fabsf(x) < LUCIER_DENORM_FLOOR) ? 0.0f : x;
}

float lucier_clip_guard(float x) {
    if (x > 1.2f) return 1.2f;
    if (x < -1.2f) return -1.2f;
    return x;
}

/*
 * C1-continuous odd-symmetric Pade rational tanh tape limiter:
 *   f(0) = 0, f'(0) = 1, f'(+/-2.85) = 0, strictly bounded in [-0.95, +0.95].
 *   Zero even-order harmonics -> zero DC offset generation across 100+ passes.
 */
float lucier_tape_limit(float x) {
    const float ceiling = 0.95f;
    const float inv_c = 1.0f / 0.95f;
    const float s = clampf(x * inv_c, -3.0f, 3.0f);
    const float s2 = s * s;
    return ceiling * (s * (27.0f + s2)) / (27.0f + 9.0f * s2);
}

const char* lucier_geom_name(uint8_t geom_idx) {
    return kGeomNames[geom_idx % LUCIER_GEOM_COUNT];
}

const char* lucier_geom_short_name(uint8_t geom_idx) {
    return kGeomShortNames[geom_idx % LUCIER_GEOM_COUNT];
}

static inline float one_pole_alpha(float fc, float sr) {
    const float c = clampf(fc, 5.0f, 0.45f * sr);
    return 1.0f - expf((-2.0f * M_PI * c) / sr);
}

static void lucier_update_coeffs(lucier_engine_t* d) {
    const float sr = d->sample_rate;
    const uint8_t gi = d->geom_idx % LUCIER_GEOM_COUNT;

    /* 1. Per-pass wet transformation depth:
     *    At PASS = 40%, pass_mix ~ 0.30 -> ~25-40 passes from speech to pure resonant chord
     *    At PASS = 100%, pass_mix = 1.00 -> ~6-10 passes into singing room drones */
    if (d->pass_amt < LUCIER_MIN_PASS) {
        d->pass_mix = 0.0f;
    } else {
        const float p = d->pass_amt;
        d->pass_mix = clampf(0.45f * p + 0.55f * (p * p), 0.0f, 1.0f);
    }

    /* 2. Physical room dimension scaling:
     *    ROOM = 0.0 -> small chamber (f_base = 240 Hz)
     *    ROOM = 0.5 -> medium room   (f_base = 118 Hz)
     *    ROOM = 1.0 -> vast hall     (f_base = 58 Hz) */
    d->f_base = 240.0f * expf(-1.4213857f * d->room_size);

    /* 3. Modal resonance Q (spans Q = 7 at RESO=0 to Q = 72 at RESO=1) */
    const float r = d->resonance;
    d->modal_q = 7.0f + 65.0f * (r * r);
    d->direct_floor = 0.11f * (1.0f - 0.85f * r);

    /* 4. Early reflection tap lengths & Schroeder allpass diffusion scaled by ROOM & RESO */
    const float scale = 0.32f + 0.68f * d->room_size;
    d->tap_l[0] = (uint16_t)(311.0f * scale);
    d->tap_l[1] = (uint16_t)(769.0f * scale);
    d->tap_l[2] = (uint16_t)(1523.0f * scale);
    d->tap_r[0] = (uint16_t)(353.0f * scale);
    d->tap_r[1] = (uint16_t)(857.0f * scale);
    d->tap_r[2] = (uint16_t)(1693.0f * scale);

    d->ap1_del_l = (uint16_t)(157.0f + 220.0f * scale);
    d->ap1_del_r = (uint16_t)(179.0f + 240.0f * scale);
    d->ap2_del_l = (uint16_t)(331.0f + 460.0f * scale);
    d->ap2_del_r = (uint16_t)(367.0f + 500.0f * scale);
    d->ap_fb = 0.38f + 0.24f * r;
    d->refl_mix = 0.24f + 0.48f * r;

    /* 5. Per-pass speaker/mic EQ (sub-rumble HPF + speaker roll-off LPF + spectral tilt) */
    const float tilt = d->tone; /* [-1..+1] */
    const float hpf_fc = 52.0f + 26.0f * clampf(tilt, -1.0f, 1.0f);
    const float lpf_fc = 4600.0f * expf(0.54f * tilt); /* ~2.68 kHz .. ~7.89 kHz */
    d->hpf_a = one_pole_alpha(hpf_fc, sr);
    d->lpf_a = one_pole_alpha(lpf_fc, sr);
    d->piv_a = one_pole_alpha(380.0f, sr);
    d->tilt_low_g  = expf(-0.48f * tilt);
    d->tilt_high_g = expf( 0.48f * tilt);

    /* 6. 8-mode ZDF trapezoidal SVF resonator coefficients */
    const float nyq_lim = 0.40f * sr;
    /* log10(2.0) = 0.30103f -> octaves relative to 380 Hz = log10f(f / 380) * 3.321928f */
    for (uint32_t m = 0; m < LUCIER_NUM_MODES; ++m) {
        const float fc = clampf(d->f_base * kGeomRatios[gi][m], 45.0f, nyq_lim);
        const float fl = clampf(fc * kStereoDetuneL[m], 45.0f, nyq_lim);
        const float fr = clampf(fc * kStereoDetuneR[m], 45.0f, nyq_lim);
        d->mode_freq[m] = fc;

        /* Slight natural Q variation across architectural modes */
        const float qm = d->modal_q * (0.88f + 0.04f * (float)m);
        const float km = 1.0f / qm;
        d->damp_k[m] = km;

        const float gl = tanf((M_PI * fl) / sr);
        const float a1l = 1.0f / (1.0f + gl * (gl + km));
        d->a1_l[m] = a1l;
        d->a2_l[m] = gl * a1l;
        d->a3_l[m] = gl * d->a2_l[m];

        const float gr = tanf((M_PI * fr) / sr);
        const float a1r = 1.0f / (1.0f + gr * (gr + km));
        d->a1_r[m] = a1r;
        d->a2_r[m] = gr * a1r;
        d->a3_r[m] = gr * d->a2_r[m];

        /* Tilt emphasis per mode around 380 Hz pivot */
        const float oct = log10f(fc * (1.0f / 380.0f)) * 3.321928f;
        const float tilt_w = expf(clampf(tilt * 0.42f * oct, -1.4f, 1.4f));
        d->mode_weight[m] = kBaseWeights[m] * tilt_w;
    }
}

void lucier_init(lucier_engine_t* d, float sample_rate) {
    memset(d, 0, sizeof(*d));
    d->sample_rate = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    d->pass_amt    = 0.42f;
    d->room_size   = 0.48f;
    d->resonance   = 0.62f;
    d->tone        = 0.08f;
    d->gate        = 0.0f;
    d->geom_idx    = LUCIER_GEOM_ROOM_1969;

    /* ~18 Hz DC blocker pole: R = 1 - 2*pi*18/sr */
    d->dc_r        = 1.0f - (2.0f * M_PI * 18.0f) / d->sample_rate;
    /* Per-mode amplitude envelope (~220 ms) & slow loop-scale AGC (~450 ms) */
    d->mode_env_a  = 1.0f / (0.22f * d->sample_rate);
    d->agc_a       = 1.0f / (0.45f * d->sample_rate);
    d->gain_slew   = 1.0f / (0.08f * d->sample_rate);

    d->target_rms  = 0.20f;
    d->dry_env_sq  = 0.04f;
    d->wet_env_sq  = 0.01f;
    d->agc_gain    = 1.8f;

    lucier_update_coeffs(d);
}

void lucier_attach_space(lucier_engine_t* d, lucier_space_t* space) {
    d->space = space;
    if (space) {
        memset(space, 0, sizeof(*space));
    }
    d->refl_wr = 0u;
    d->ap1_wr  = 0u;
    d->ap2_wr  = 0u;
}

void lucier_reset_state(lucier_engine_t* d, float initial_rms) {
    for (uint32_t m = 0; m < LUCIER_NUM_MODES; ++m) {
        d->ic1_l[m] = 0.0f;
        d->ic2_l[m] = 0.0f;
        d->ic1_r[m] = 0.0f;
        d->ic2_r[m] = 0.0f;
        d->mode_env[m] = 0.0f;
    }
    d->modal_focus = 0.0f;
    d->sub_lp_l = 0.0f;
    d->sub_lp_r = 0.0f;
    d->spk_lp_l = 0.0f;
    d->spk_lp_r = 0.0f;
    d->piv_lp_l = 0.0f;
    d->piv_lp_r = 0.0f;
    d->dc_x_l   = 0.0f;
    d->dc_y_l   = 0.0f;
    d->dc_x_r   = 0.0f;
    d->dc_y_r   = 0.0f;

    if (d->space) {
        memset(d->space, 0, sizeof(*d->space));
    }
    d->refl_wr = 0u;
    d->ap1_wr  = 0u;
    d->ap2_wr  = 0u;

    /* Anchor target loop loudness to initial loop RMS within a comfortable musical ceiling */
    const float tgt = (initial_rms > 0.02f) ? clampf(initial_rms, 0.12f, 0.26f) : 0.18f;
    d->target_rms = tgt;
    d->dry_env_sq = tgt * tgt;
    d->wet_env_sq = 0.25f * tgt * tgt;
    d->agc_gain   = 1.8f;
}

void lucier_set_pass(lucier_engine_t* d, float norm) {
    d->pass_amt = clampf(norm, 0.0f, 1.0f);
    lucier_update_coeffs(d);
}

void lucier_set_room(lucier_engine_t* d, float norm) {
    d->room_size = clampf(norm, 0.0f, 1.0f);
    lucier_update_coeffs(d);
}

void lucier_set_reso(lucier_engine_t* d, float norm) {
    d->resonance = clampf(norm, 0.0f, 1.0f);
    lucier_update_coeffs(d);
}

void lucier_set_tone(lucier_engine_t* d, float bipolar) {
    d->tone = clampf(bipolar, -1.0f, 1.0f);
    lucier_update_coeffs(d);
}

void lucier_set_gate(lucier_engine_t* d, float gate) {
    d->gate = clampf(gate, 0.0f, 1.0f);
}

void lucier_set_geometry(lucier_engine_t* d, uint8_t geom_idx) {
    d->geom_idx = geom_idx % LUCIER_GEOM_COUNT;
    lucier_update_coeffs(d);
}

bool lucier_is_active(const lucier_engine_t* d) {
    return d->gate > 0.0001f && d->pass_mix > 0.0001f;
}

static inline float schroeder_allpass(float* buf, uint32_t wr, uint32_t del,
                                      uint32_t mask, float g, float x) {
    const uint32_t rd = (wr - del) & mask;
    const float d_out = buf[rd];
    const float v = luc_flush(x + g * d_out);
    buf[wr] = v;
    return d_out - g * v;
}

lucier_frame_t lucier_process_frame(lucier_engine_t* d, lucier_frame_t in, float write_enable) {
    const float xl = lucier_clip_guard(in.l);
    const float xr = lucier_clip_guard(in.r);

    /* Stage A: Early Wall Reflections + Stereo Schroeder Allpass Air Diffusion */
    float air_l = xl;
    float air_r = xr;
    lucier_space_t* sp = d->space;
    if (sp) {
        const uint32_t rw = d->refl_wr;
        sp->refl_l[rw] = xl;
        sp->refl_r[rw] = xr;

        const float tl = 0.46f * sp->refl_l[(rw - d->tap_l[0]) & LUCIER_REFL_MASK]
                       + 0.33f * sp->refl_l[(rw - d->tap_l[1]) & LUCIER_REFL_MASK]
                       + 0.21f * sp->refl_l[(rw - d->tap_l[2]) & LUCIER_REFL_MASK];
        const float tr = 0.46f * sp->refl_r[(rw - d->tap_r[0]) & LUCIER_REFL_MASK]
                       + 0.33f * sp->refl_r[(rw - d->tap_r[1]) & LUCIER_REFL_MASK]
                       + 0.21f * sp->refl_r[(rw - d->tap_r[2]) & LUCIER_REFL_MASK];
        d->refl_wr = (rw + 1u) & LUCIER_REFL_MASK;

        /* Subtle stereo wall cross-reflection (~18%) widens mono voice into 3D space */
        const float x_refl_l = tl + 0.18f * tr;
        const float x_refl_r = tr + 0.18f * tl;

        const uint32_t w1 = d->ap1_wr;
        const float a1l = schroeder_allpass(sp->ap1_l, w1, d->ap1_del_l, LUCIER_AP1_MASK, d->ap_fb, x_refl_l);
        const float a1r = schroeder_allpass(sp->ap1_r, w1, d->ap1_del_r, LUCIER_AP1_MASK, d->ap_fb, x_refl_r);
        d->ap1_wr = (w1 + 1u) & LUCIER_AP1_MASK;

        const uint32_t w2 = d->ap2_wr;
        const float a2l = schroeder_allpass(sp->ap2_l, w2, d->ap2_del_l, LUCIER_AP2_MASK, d->ap_fb, a1l);
        const float a2r = schroeder_allpass(sp->ap2_r, w2, d->ap2_del_r, LUCIER_AP2_MASK, d->ap_fb, a1r);
        d->ap2_wr = (w2 + 1u) & LUCIER_AP2_MASK;

        air_l = xl + d->refl_mix * a2l;
        air_r = xr + d->refl_mix * a2r;
    }

    /* Stage B: Per-Pass Loudspeaker + Microphone EQ (Sub HPF + Roll-Off LPF + Tilt Shelf) */
    d->sub_lp_l = luc_flush(d->sub_lp_l + d->hpf_a * (air_l - d->sub_lp_l));
    d->sub_lp_r = luc_flush(d->sub_lp_r + d->hpf_a * (air_r - d->sub_lp_r));
    const float hp_l = air_l - d->sub_lp_l;
    const float hp_r = air_r - d->sub_lp_r;

    d->spk_lp_l = luc_flush(d->spk_lp_l + d->lpf_a * (hp_l - d->spk_lp_l));
    d->spk_lp_r = luc_flush(d->spk_lp_r + d->lpf_a * (hp_r - d->spk_lp_r));

    d->piv_lp_l = luc_flush(d->piv_lp_l + d->piv_a * (d->spk_lp_l - d->piv_lp_l));
    d->piv_lp_r = luc_flush(d->piv_lp_r + d->piv_a * (d->spk_lp_r - d->piv_lp_r));
    const float piv_hp_l = d->spk_lp_l - d->piv_lp_l;
    const float piv_hp_r = d->spk_lp_r - d->piv_lp_r;

    const float shaped_l = d->tilt_low_g * d->piv_lp_l + d->tilt_high_g * piv_hp_l;
    const float shaped_r = d->tilt_low_g * d->piv_lp_r + d->tilt_high_g * piv_hp_r;

    /* Stage C: 8-Mode Architectural Room Resonator Bank (Unity-Peak ZDF Bandpass SVFs) */
    float sum_l = 0.0f;
    float sum_r = 0.0f;

    for (uint32_t m = 0; m < LUCIER_NUM_MODES; ++m) {
        const float km = d->damp_k[m];

        const float v3l = shaped_l - d->ic2_l[m];
        const float v1l = d->a1_l[m] * d->ic1_l[m] + d->a2_l[m] * v3l;
        const float v2l = d->ic2_l[m] + d->a2_l[m] * d->ic1_l[m] + d->a3_l[m] * v3l;
        d->ic1_l[m] = luc_flush(2.0f * v1l - d->ic1_l[m]);
        d->ic2_l[m] = luc_flush(2.0f * v2l - d->ic2_l[m]);
        const float bpl = km * v1l;

        const float v3r = shaped_r - d->ic2_r[m];
        const float v1r = d->a1_r[m] * d->ic1_r[m] + d->a2_r[m] * v3r;
        const float v2r = d->ic2_r[m] + d->a2_r[m] * d->ic1_r[m] + d->a3_r[m] * v3r;
        d->ic1_r[m] = luc_flush(2.0f * v1r - d->ic1_r[m]);
        d->ic2_r[m] = luc_flush(2.0f * v2r - d->ic2_r[m]);
        const float bpr = km * v1r;

        const float mag = 0.5f * (fabsf(bpl) + fabsf(bpr));
        const float env = d->mode_env[m] + d->mode_env_a * (mag - d->mode_env[m]);
        d->mode_env[m] = env;

        /* Gentle per-mode envelope compression keeps 3-5 excited modes singing as a chord */
        const float mg = d->mode_weight[m] / (1.0f + 2.4f * env);
        sum_l += mg * bpl;
        sum_r += mg * bpr;
    }

    const float raw_l = sum_l + d->direct_floor * shaped_l;
    const float raw_r = sum_r + d->direct_floor * shaped_r;

    /* Stage D: Matched-Envelope AGC + Loop Loudness Governor */
    const float in_pow  = 0.5f * (xl * xl + xr * xr);
    const float raw_pow = 0.5f * (raw_l * raw_l + raw_r * raw_r);
    d->dry_env_sq += d->agc_a * (in_pow - d->dry_env_sq);
    d->wet_env_sq += d->agc_a * (raw_pow - d->wet_env_sq);

    if (d->dry_env_sq > 1.0e-6f) {
        /* 1) Compensate insertion loss / modal peak gain between input and room bank */
        const float match = clampf(sqrtf((d->dry_env_sq + 1.0e-7f) / (d->wet_env_sq + 1.0e-7f)),
                                   0.15f, 6.50f);
        /* 2) Gentle pull toward target_rms so 100+ passes neither blow up nor fade away */
        const float dry_rms = sqrtf(d->dry_env_sq);
        const float gov = clampf(sqrtf(d->target_rms / (dry_rms + 1.0e-5f)), 0.70f, 1.42f);
        const float desired = clampf(match * gov, 0.12f, 7.50f);
        d->agc_gain += d->gain_slew * (desired - d->agc_gain);
    }

    /* Update spectral modal focus metric for UI readout */
    if (d->dry_env_sq > 1.0e-6f) {
        const float focus_inst = clampf(sqrtf(d->wet_env_sq / (d->dry_env_sq + 1.0e-7f)) * 1.15f,
                                        0.0f, 1.0f);
        d->modal_focus += 0.0005f * (focus_inst - d->modal_focus);
    }

    float wet_l = raw_l * d->agc_gain;
    float wet_r = raw_r * d->agc_gain;

    /* 1-pole zero-DC blocker (~18 Hz) on wet room signal */
    const float dc_l = luc_flush(wet_l - d->dc_x_l + d->dc_r * d->dc_y_l);
    const float dc_r_out = luc_flush(wet_r - d->dc_x_r + d->dc_r * d->dc_y_r);
    d->dc_x_l = wet_l;
    d->dc_y_l = dc_l;
    d->dc_x_r = wet_r;
    d->dc_y_r = dc_r_out;

    /* Blend dry generation N with room-processed signal according to PASS & SoftSwitch gate */
    const float eff = clampf(write_enable * d->gate * d->pass_mix, 0.0f, 1.0f);
    if (eff <= 0.0001f) {
        lucier_frame_t dry_out = { xl, xr };
        return dry_out;
    }

    float next_l = lucier_tape_limit(xl + eff * (dc_l - xl));
    float next_r = lucier_tape_limit(xr + eff * (dc_r_out - xr));

    if (fabsf(next_l) < LUCIER_SILENCE_FLOOR) next_l = 0.0f;
    if (fabsf(next_r) < LUCIER_SILENCE_FLOOR) next_r = 0.0f;

    lucier_frame_t res = { next_l, next_r };
    return res;
}

void lucier_commit_step(const lucier_engine_t* d, lucier_frame_t* buf, uint32_t rec_size,
                        int32_t prev_i, int32_t step, lucier_frame_t next_gen, float gate) {
    if (!buf || rec_size < 8u || d->pass_mix <= 0.0001f || gate <= 0.0001f) return;

    const int32_t size = (int32_t)rec_size;
    if (step > 0 && step <= LUCIER_MAX_STEP_FR) {
        for (int32_t k = 0; k < step; ++k) {
            int32_t idx = (prev_i + k) % size;
            if (idx < 0) idx += size;
            buf[idx] = next_gen;
        }
    } else if (step < 0 && -step <= LUCIER_MAX_STEP_FR) {
        const int32_t n = -step;
        for (int32_t k = 0; k < n; ++k) {
            int32_t idx = (prev_i + 1 - k) % size;
            if (idx < 0) idx += size;
            buf[idx] = next_gen;
        }
    }
}
