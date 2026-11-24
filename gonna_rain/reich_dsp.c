#include "reich_dsp.h"

#ifdef REICH_HOST_TEST
#include <math.h>
#include <string.h>
#undef M_PI
#undef M_PI_4
#define M_PI   3.14159265358979323846f
#define M_PI_4 0.78539816339744830962f
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
#else
#include "tapp_api.h"
#endif

#define REICH_DENORM_FLOOR  1.0e-15f
#define REICH_TWO_PI        6.28318530717958647692f
#define REICH_SUB_SPLICE_FR 384u

typedef struct {
    float deg;
    const char* label;
} reich_canon_mark_t;

static const reich_canon_mark_t kCanonMarks[] = {
    {   0.0f, "0 deg (Unison)"   },
    {  45.0f, "45 deg (1/8)"     },
    {  60.0f, "60 deg (1/6)"     },
    {  90.0f, "90 deg (1/4)"     },
    { 120.0f, "120 deg (1/3)"    },
    { 135.0f, "135 deg (3/8)"    },
    { 180.0f, "180 deg (1/2)"    },
    { 225.0f, "225 deg (5/8)"    },
    { 240.0f, "240 deg (2/3)"    },
    { 270.0f, "270 deg (3/4)"    },
    { 300.0f, "300 deg (5/6)"    },
    { 315.0f, "315 deg (7/8)"    },
    { 360.0f, "0 deg (Unison)"   },
};

static inline float reich_flush_denorm(float x) {
    return (fabsf(x) < REICH_DENORM_FLOOR) ? 0.0f : x;
}

static inline float reich_smoothstep01(float t) {
    const float c = clampf(t, 0.0f, 1.0f);
    return c * c * (3.0f - 2.0f * c);
}

float reich_clip_guard(float x) {
    if (!(x > -4.0f && x < 4.0f)) return 0.0f;
    return clampf(x, -1.25f, 1.25f);
}

float reich_tape_limit(float x) {
    if (!(x > -8.0f && x < 8.0f)) {
        return (x >= 0.0f) ? 0.98f : -0.98f;
    }
    const float ceiling = 0.98f;
    const float u = clampf(x * (1.0f / 0.98f), -3.0f, 3.0f);
    const float u2 = u * u;
    const float y = u * (27.0f + u2) / (27.0f + 9.0f * u2);
    return reich_flush_denorm(y * ceiling);
}

/* 4-point, 3rd-order Catmull-Rom / Hermite cubic spline interpolation */
static inline float hermite4(float frac, float ym1, float y0, float y1, float y2) {
    const float c0 = y0;
    const float c1 = 0.5f * (y1 - ym1);
    const float c2 = ym1 - 2.5f * y0 + 2.0f * y1 - 0.5f * y2;
    const float c3 = 0.5f * (y2 - ym1) + 1.5f * (y0 - y1);
    return ((c3 * frac + c2) * frac + c1) * frac + c0;
}

/*
 * Read sample `k` within [0 .. win_len) from `loop_buf`, applying a C1-seamless
 * crossfade splice at the sub-loop boundary whenever `win_len < full_len`.
 */
static inline reich_frame_t read_window_sample(const reich_frame_t* buf,
                                               uint32_t full_len,
                                               uint32_t win_start,
                                               uint32_t win_len,
                                               int32_t k) {
    const uint32_t idx = win_start + (uint32_t)k;
    if (win_len >= full_len) {
        return buf[idx];
    }

    uint32_t fade = REICH_SUB_SPLICE_FR;
    if (fade > (win_len >> 3)) fade = win_len >> 3;

    /* If audio exists immediately after the cropped window, crossfade seamlessly across wrap */
    if (fade >= 8u && (uint32_t)k < fade && (win_start + win_len + fade) <= full_len) {
        const float w = reich_smoothstep01((float)k / (float)fade);
        const reich_frame_t tail_cont = buf[win_start + win_len + (uint32_t)k];
        const reich_frame_t head_orig = buf[idx];
        reich_frame_t out = {
            tail_cont.l + w * (head_orig.l - tail_cont.l),
            tail_cont.r + w * (head_orig.r - tail_cont.r),
        };
        return out;
    }

    /* Fallback micro-taper if cropped right against the end of full_len */
    if (fade >= 8u && ((uint32_t)k < fade || (uint32_t)k >= (win_len - fade))) {
        const uint32_t dist = ((uint32_t)k < fade) ? (uint32_t)k : (win_len - 1u - (uint32_t)k);
        const float w = reich_smoothstep01((float)dist / (float)fade);
        reich_frame_t out = { buf[idx].l * w, buf[idx].r * w };
        return out;
    }

    return buf[idx];
}

static inline reich_frame_t read_hermite_wrapped(const reich_frame_t* buf,
                                                 uint32_t full_len,
                                                 uint32_t win_start,
                                                 int32_t L,
                                                 int32_t i0,
                                                 float frac) {
    const int32_t im1 = (i0 == 0) ? (L - 1) : (i0 - 1);
    const int32_t ip1 = (i0 + 1 >= L) ? (i0 + 1 - L) : (i0 + 1);
    const int32_t ip2 = (i0 + 2 >= L) ? (i0 + 2 - L) : (i0 + 2);

    const uint32_t uL = (uint32_t)L;
    const reich_frame_t sm1 = read_window_sample(buf, full_len, win_start, uL, im1);
    const reich_frame_t s0  = read_window_sample(buf, full_len, win_start, uL, i0);
    const reich_frame_t sp1 = read_window_sample(buf, full_len, win_start, uL, ip1);
    const reich_frame_t sp2 = read_window_sample(buf, full_len, win_start, uL, ip2);

    reich_frame_t out = {
        hermite4(frac, sm1.l, s0.l, sp1.l, sp2.l),
        hermite4(frac, sm1.r, s0.r, sp1.r, sp2.r),
    };
    return out;
}

static void reich_recalc(reich_engine_t* d) {
    const float sr = (d->sample_rate > 8000.0f) ? d->sample_rate : 48000.0f;

    /*
     * Musical exponential DRIFT curve:
     *   0.0%   -> 0.00000 (0% slip)
     *   5.0%   -> 0.00105 (~3 min cycle on a 2s loop: classic glacial Reich flanging)
     *  35.0%   -> 0.00600 (~33s cycle on a 2s loop: concert phasing)
     *  65.0%   -> 0.03350 (~6s cycle)
     * 100.0%   -> 0.25000 (dramatic reel-flange sweep)
     */
    const float dr = d->drift_norm;
    if (dr < 0.003f) {
        d->drift_ratio = 0.0f;
    } else {
        float ramp = 1.0f;
        if (dr < 0.04f) {
            ramp = dr * (1.0f / 0.04f);
        }
        d->drift_ratio = ramp * 0.0008f * expf(5.745f * dr);
    }

    /* Constant-power stereo spread across 2 (Part I) or 4 (Part II) heads */
    const float sp = d->spread_norm;
    const float pans_2[4] = { -sp, +sp, 0.0f, 0.0f };
    const float pans_4[4] = { -sp, +sp, -0.45f * sp, +0.45f * sp };
    const float* pans = (d->num_decks >= 4u) ? pans_4 : pans_2;

    for (uint32_t v = 0; v < REICH_MAX_VOICES; ++v) {
        const float angle = (pans[v] + 1.0f) * M_PI_4;
        d->pan_l[v] = cosf(angle);
        d->pan_r[v] = sinf(angle);
    }

    if (d->num_decks >= 4u) {
        d->voice_scale = 0.34f + 0.11f * sp;
    } else {
        d->voice_scale = 0.66f + 0.22f * sp;
    }

    /*
     * Analog Wollensak capstan character (TAPE = 0..1):
     * At 0.0f: strictly zero wow/flutter & transparent unity EQ/gain for exact phase tests.
     * Above 0: independent capstan micro-wow/flutter (up to +/-0.29 ms = +/-14 samples at 48 kHz),
     * warm ~125 Hz playback head bump, gentle oxide HF smoothing, and soft saturation drive.
     */
    const float tp = d->tape_norm;
    if (tp < 0.002f) {
        d->wow_depth_samples = 0.0f;
        d->bump_gain = 0.0f;
        d->hf_a = 1.0f;
        d->drive_gain = 1.0f;
    } else {
        d->wow_depth_samples = (1.5f * tp + 12.5f * tp * tp) * (sr * (1.0f / 48000.0f));
        d->bump_gain = 0.26f * tp;
        const float fc_hf = 19000.0f * expf(-0.98f * tp); /* ~19 kHz -> ~7.1 kHz */
        d->hf_a = 1.0f - expf(-REICH_TWO_PI * fc_hf / sr);
        d->drive_gain = 1.0f + 0.32f * tp;
    }
    d->bump_a = 1.0f - expf(-REICH_TWO_PI * 125.0f / sr);
}

void reich_init(reich_engine_t* d, float sample_rate) {
    memset(d, 0, sizeof(*d));
    d->sample_rate = (sample_rate > 8000.0f) ? sample_rate : 48000.0f;
    d->drift_norm  = 0.35f;
    d->spread_norm = 0.70f;
    d->tape_norm   = 0.28f;
    d->num_decks   = 2u;
    d->motion_mode = REICH_MOTION_CONTINUOUS;
    d->drifting    = true;
    d->inv_win_len = 1.0f;

    /* Offset initial motor LFO phases so Decks A/B/C/D breathe independently */
    d->wow_ph_a  = 0.00f;
    d->wow_ph_b  = 0.37f;
    d->wow_ph_c  = 0.61f;
    d->wow_ph_d  = 0.83f;
    d->flut_ph_a = 0.13f;
    d->flut_ph_b = 0.59f;

    reich_recalc(d);
}

void reich_reset_loop(reich_engine_t* d, uint32_t full_len, uint32_t win_start, uint32_t win_len) {
    if (win_len < 8u) win_len = (full_len >= 8u) ? full_len : 8u;
    if (full_len < win_len) full_len = win_len;
    if (win_start + win_len > full_len) win_start = 0u;

    d->full_len     = full_len;
    d->win_start    = win_start;
    d->win_len      = win_len;
    d->inv_win_len  = 1.0f / (float)win_len;

    d->head_a_i     = 0;
    d->head_a_f     = 0.0f;
    d->phase_i      = 0;
    d->phase_f      = 0.0f;
    d->slew_samples = 0.0f;

    d->lap_count    = 1u;
    d->phase_cycles = 0u;
    d->prev_octant  = 0u;
    d->dwell_rem    = 0u;
    d->is_dwelling  = false;

    d->bump_lp_l    = 0.0f;
    d->bump_lp_r    = 0.0f;
    d->hf_lp_l      = 0.0f;
    d->hf_lp_r      = 0.0f;

    for (uint32_t v = 0; v < REICH_MAX_VOICES; ++v) {
        d->voice_pos_i[v] = 0;
        d->voice_env[v] = 0.0f;
    }
}

void reich_set_window(reich_engine_t* d, uint32_t full_len, uint32_t win_start, uint32_t win_len) {
    if (win_len < 8u || full_len < 8u) return;
    if (win_len > full_len) win_len = full_len;
    if (win_start + win_len > full_len) win_start = full_len - win_len;

    /* Preserve normalized phase angle across sub-loop crop changes */
    const float cur_deg = reich_get_phase_deg(d);

    d->full_len    = full_len;
    d->win_start   = win_start;
    d->win_len     = win_len;
    d->inv_win_len = 1.0f / (float)win_len;

    if (d->head_a_i >= (int32_t)win_len || d->head_a_i < 0) {
        d->head_a_i = d->head_a_i % (int32_t)win_len;
        if (d->head_a_i < 0) d->head_a_i += (int32_t)win_len;
    }

    reich_set_phase_deg_immediate(d, cur_deg);
    d->dwell_rem = 0u;
    d->is_dwelling = false;
}

void reich_set_drift(reich_engine_t* d, float norm) {
    d->drift_norm = clampf(norm, 0.0f, 1.0f);
    reich_recalc(d);
}

void reich_set_spread(reich_engine_t* d, float norm) {
    d->spread_norm = clampf(norm, 0.0f, 1.0f);
    reich_recalc(d);
}

void reich_set_tape(reich_engine_t* d, float norm) {
    d->tape_norm = clampf(norm, 0.0f, 1.0f);
    reich_recalc(d);
}

void reich_set_decks(reich_engine_t* d, uint8_t num_decks) {
    d->num_decks = (num_decks >= 4u) ? 4u : 2u;
    reich_recalc(d);
}

void reich_set_motion_mode(reich_engine_t* d, uint8_t mode) {
    d->motion_mode = (mode == REICH_MOTION_CANON_DWELL) ? REICH_MOTION_CANON_DWELL
                                                        : REICH_MOTION_CONTINUOUS;
    d->dwell_rem = 0u;
    d->is_dwelling = false;
    d->prev_octant = (uint8_t)((uint32_t)(reich_get_phase_turns(d) * 8.0f) & 7u);
}

void reich_set_drifting(reich_engine_t* d, bool drifting) {
    d->drifting = drifting;
    if (!drifting) {
        d->dwell_rem = 0u;
        d->is_dwelling = false;
    }
}

float reich_get_phase_turns(const reich_engine_t* d) {
    if (d->win_len < 8u) return 0.0f;
    float t = ((float)d->phase_i + d->phase_f) * d->inv_win_len;
    if (t >= 1.0f) t -= floorf(t);
    if (t < 0.0f) t = 0.0f;
    return t;
}

float reich_get_phase_deg(const reich_engine_t* d) {
    return reich_get_phase_turns(d) * 360.0f;
}

float reich_get_offset_ms(const reich_engine_t* d) {
    if (d->win_len < 8u || d->sample_rate < 1000.0f) return 0.0f;
    const float samples = (float)d->phase_i + d->phase_f;
    return (samples * 1000.0f) / d->sample_rate;
}

void reich_set_phase_deg_immediate(reich_engine_t* d, float deg) {
    if (d->win_len < 8u) return;
    float wrapped = fmodf(deg, 360.0f);
    if (wrapped < 0.0f) wrapped += 360.0f;
    if (wrapped >= 360.0f) wrapped = 0.0f;

    const int32_t L = (int32_t)d->win_len;
    const float exact = (wrapped * (1.0f / 360.0f)) * (float)L;
    int32_t pi = (int32_t)floorf(exact);
    float pf = exact - (float)pi;

    if (pf > 0.99995f) {
        pf = 0.0f;
        pi++;
    } else if (pf < 0.00005f) {
        pf = 0.0f;
    }
    if (pi >= L || pi < 0) pi = 0;

    d->phase_i = pi;
    d->phase_f = pf;
    d->slew_samples = 0.0f;
    d->prev_octant = (uint8_t)((uint32_t)(reich_get_phase_turns(d) * 8.0f) & 7u);
}

void reich_nudge_phase_deg(reich_engine_t* d, float delta_deg) {
    if (d->win_len < 8u) return;
    const float delta_samples = (delta_deg * (1.0f / 360.0f)) * (float)d->win_len;
    const float max_queue = (float)d->win_len;
    d->slew_samples = clampf(d->slew_samples + delta_samples, -max_queue, max_queue);
    d->dwell_rem = 0u;
    d->is_dwelling = false;
}

void reich_slew_to_phase_deg(reich_engine_t* d, float target_deg) {
    if (d->win_len < 8u) return;
    float tgt = fmodf(target_deg, 360.0f);
    if (tgt < 0.0f) tgt += 360.0f;

    const float cur = reich_get_phase_deg(d);
    float diff = tgt - cur;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    d->slew_samples = (diff * (1.0f / 360.0f)) * (float)d->win_len;
    d->dwell_rem = 0u;
    d->is_dwelling = false;
}

float reich_snap_nearest_canon(reich_engine_t* d, const char** out_label) {
    const float cur = reich_get_phase_deg(d);
    uint32_t best_idx = 0u;
    float best_dist = 999.0f;

    const uint32_t n = (uint32_t)(sizeof(kCanonMarks) / sizeof(kCanonMarks[0]));
    for (uint32_t i = 0u; i < n; ++i) {
        const float dist = fabsf(cur - kCanonMarks[i].deg);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx = i;
        }
    }

    const float snap_deg = (kCanonMarks[best_idx].deg >= 360.0f) ? 0.0f : kCanonMarks[best_idx].deg;
    if (out_label) *out_label = kCanonMarks[best_idx].label;
    reich_slew_to_phase_deg(d, snap_deg);
    return snap_deg;
}

const char* reich_describe_zone(const reich_engine_t* d) {
    const float deg = reich_get_phase_deg(d);
    const float ms = reich_get_offset_ms(d);
    const float tail_ms = ((float)d->win_len * 1000.0f / d->sample_rate) - ms;
    const float min_ms = (ms < tail_ms) ? ms : tail_ms;

    if (deg < 1.5f || deg > 358.5f) return "UNISON 0";
    if (min_ms < 16.0f)             return "COMB FLANGE";
    if (min_ms < 75.0f)             return "ECHO DOUBLE";
    if (fabsf(deg - 90.0f)  < 3.5f) return "1/4 CANON";
    if (fabsf(deg - 120.0f) < 3.5f) return "1/3 CANON";
    if (fabsf(deg - 180.0f) < 4.0f) return "180 HOCKET";
    if (fabsf(deg - 240.0f) < 3.5f) return "2/3 CANON";
    if (fabsf(deg - 270.0f) < 3.5f) return "3/4 CANON";
    return "POLY CANON";
}

static inline void wrap_add_samples(int32_t L, int32_t* io_i, float* io_f,
                                    float delta, bool* out_wrapped) {
    float nf = *io_f + delta;
    int32_t step = (int32_t)floorf(nf);
    nf -= (float)step;
    int32_t ni = *io_i + step;

    bool wrapped = false;
    while (ni >= L) {
        ni -= L;
        wrapped = true;
    }
    while (ni < 0) {
        ni += L;
        wrapped = true;
    }
    *io_i = ni;
    *io_f = nf;
    if (out_wrapped) *out_wrapped = wrapped;
}

reich_frame_t reich_process_frame(reich_engine_t* d, const reich_frame_t* loop_buf,
                                  float speed, bool allow_drift) {
    reich_frame_t zero = { 0.0f, 0.0f };
    if (!loop_buf || d->win_len < 8u || d->full_len < d->win_len) {
        return zero;
    }

    const int32_t L = (int32_t)d->win_len;
    const float abs_speed = fabsf(speed);

    /* 1. Compute zero-mean analog capstan micro-wow & pinch-roller flutter offsets */
    float wow_off[REICH_MAX_VOICES] = { 0.0f, 0.0f, 0.0f, 0.0f };
    if (d->wow_depth_samples > 0.0f && abs_speed > 0.001f) {
        const float inv_sr = 1.0f / d->sample_rate;
        d->wow_ph_a  += 0.43f * inv_sr;
        d->wow_ph_b  += 0.61f * inv_sr;
        d->wow_ph_c  += 0.51f * inv_sr;
        d->wow_ph_d  += 0.73f * inv_sr;
        d->flut_ph_a += 5.70f * inv_sr;
        d->flut_ph_b += 6.40f * inv_sr;

        if (d->wow_ph_a  >= 1.0f) d->wow_ph_a  -= 1.0f;
        if (d->wow_ph_b  >= 1.0f) d->wow_ph_b  -= 1.0f;
        if (d->wow_ph_c  >= 1.0f) d->wow_ph_c  -= 1.0f;
        if (d->wow_ph_d  >= 1.0f) d->wow_ph_d  -= 1.0f;
        if (d->flut_ph_a >= 1.0f) d->flut_ph_a -= 1.0f;
        if (d->flut_ph_b >= 1.0f) d->flut_ph_b -= 1.0f;

        const float dep = d->wow_depth_samples;
        wow_off[0] = dep * (0.80f * sinf(REICH_TWO_PI * d->wow_ph_a) +
                            0.20f * sinf(REICH_TWO_PI * d->flut_ph_a));
        wow_off[1] = dep * (0.80f * sinf(REICH_TWO_PI * d->wow_ph_b) +
                            0.20f * sinf(REICH_TWO_PI * d->flut_ph_b));
        if (d->num_decks >= 4u) {
            wow_off[2] = dep * (0.82f * sinf(REICH_TWO_PI * d->wow_ph_c) -
                                0.18f * sinf(REICH_TWO_PI * d->flut_ph_a));
            wow_off[3] = dep * (0.82f * sinf(REICH_TWO_PI * d->wow_ph_d) -
                                0.18f * sinf(REICH_TWO_PI * d->flut_ph_b));
        }
    }

    /* 2. Read 2 (Part I) or 4 (Part II) Hermite-interpolated virtual tape heads */
    const uint32_t voices = (d->num_decks >= 4u) ? 4u : 2u;
    float sum_l = 0.0f;
    float sum_r = 0.0f;

    for (uint32_t v = 0u; v < voices; ++v) {
        /* Head v sits at exact harmonic phase multiple (v * phase) ahead of Deck A */
        int32_t mul_i = d->phase_i * (int32_t)v;
        float mul_f = d->phase_f * (float)v;
        while (mul_f >= 1.0f) {
            mul_f -= 1.0f;
            mul_i++;
        }
        while (mul_i >= L) {
            mul_i -= L;
        }

        int32_t vi = d->head_a_i + mul_i;
        float vf = d->head_a_f + mul_f;
        if (vf >= 1.0f) {
            vf -= 1.0f;
            vi++;
        }
        if (vi >= L) {
            vi -= L;
        }

        if (wow_off[v] != 0.0f) {
            const float wf = vf + wow_off[v];
            const int32_t ws = (int32_t)floorf(wf);
            vf = wf - (float)ws;
            vi += ws;
            while (vi >= L) vi -= L;
            while (vi < 0)  vi += L;
        }

        d->voice_pos_i[v] = vi;

        const reich_frame_t raw = read_hermite_wrapped(
            loop_buf, d->full_len, d->win_start, L, vi, vf);

        const float abs_v = 0.5f * (fabsf(raw.l) + fabsf(raw.r));
        d->voice_env[v] = (abs_v > d->voice_env[v])
                              ? (d->voice_env[v] + 0.08f * (abs_v - d->voice_env[v]))
                              : (d->voice_env[v] * 0.9992f);

        /*
         * Fold stereo source gently toward deck center as SPREAD increases so hard L/R
         * panning places each complete tape deck cleanly on its stereo speaker, while at
         * SPREAD = 0% stereo channels stay intact and Deck A + Deck B sum coherently.
         */
        const float fold = 0.5f * d->spread_norm;
        const float deck_l = raw.l * (1.0f - fold) + raw.r * fold;
        const float deck_r = raw.r * (1.0f - fold) + raw.l * fold;

        sum_l += deck_l * d->pan_l[v];
        sum_r += deck_r * d->pan_r[v];
    }

    sum_l *= d->voice_scale;
    sum_r *= d->voice_scale;

    /* 3. Optional Wollensak tape head bump (~125 Hz) + oxide HF roll-off + saturation */
    if (d->tape_norm >= 0.002f) {
        d->bump_lp_l = reich_flush_denorm(d->bump_lp_l + d->bump_a * (sum_l - d->bump_lp_l));
        d->bump_lp_r = reich_flush_denorm(d->bump_lp_r + d->bump_a * (sum_r - d->bump_lp_r));
        sum_l += d->bump_gain * d->bump_lp_l;
        sum_r += d->bump_gain * d->bump_lp_r;

        d->hf_lp_l = reich_flush_denorm(d->hf_lp_l + d->hf_a * (sum_l - d->hf_lp_l));
        d->hf_lp_r = reich_flush_denorm(d->hf_lp_r + d->hf_a * (sum_r - d->hf_lp_r));
        sum_l = d->hf_lp_l * d->drive_gain;
        sum_r = d->hf_lp_r * d->drive_gain;
    }

    reich_frame_t out = {
        reich_tape_limit(sum_l),
        reich_tape_limit(sum_r),
    };

    /* 4. Advance Deck A reference playhead */
    bool a_wrapped = false;
    wrap_add_samples(L, &d->head_a_i, &d->head_a_f, speed, &a_wrapped);
    if (a_wrapped) {
        d->lap_count++;
    }

    /* 5. Advance Deck B relative phase (continuous drift / canon dwell + reel-flange servo) */
    float phase_delta = 0.0f;
    const bool drift_on = allow_drift && d->drifting && (d->drift_ratio > 0.0f) && (abs_speed > 0.0001f);

    if (drift_on) {
        if (d->motion_mode == REICH_MOTION_CANON_DWELL) {
            if (d->dwell_rem > 0u) {
                d->dwell_rem--;
                d->is_dwelling = (d->dwell_rem > 0u);
            } else {
                d->is_dwelling = false;
                phase_delta += abs_speed * d->drift_ratio;
            }
        } else {
            d->is_dwelling = false;
            phase_delta += abs_speed * d->drift_ratio;
        }
    } else {
        d->is_dwelling = false;
    }

    /* Smooth reel-flange servo slew (drains manual PHASE nudges & snap jumps without clicks) */
    if (d->slew_samples != 0.0f) {
        if (fabsf(d->slew_samples) <= 0.004f) {
            phase_delta += d->slew_samples;
            d->slew_samples = 0.0f;
        } else {
            const float max_slew = clampf((float)L * (1.0f / 48000.0f) * 0.38f, 0.25f, 2.5f);
            float servo = d->slew_samples * 0.00022f;
            if (fabsf(servo) < 0.004f) {
                servo = (d->slew_samples > 0.0f) ? 0.004f : -0.004f;
            }
            servo = clampf(servo, -max_slew, max_slew);
            phase_delta += servo;
            d->slew_samples -= servo;
        }
    }

    if (phase_delta != 0.0f) {
        bool ph_wrapped = false;
        wrap_add_samples(L, &d->phase_i, &d->phase_f, phase_delta, &ph_wrapped);
        if (ph_wrapped && phase_delta > 0.0f) {
            d->phase_cycles++;
        }

        /* Check if Canon Dwell just crossed one of the 8 rational 45-degree landmarks */
        if (drift_on && d->motion_mode == REICH_MOTION_CANON_DWELL &&
            d->dwell_rem == 0u && d->slew_samples == 0.0f) {
            const uint8_t oct = (uint8_t)((uint32_t)(reich_get_phase_turns(d) * 8.0f) & 7u);
            if (oct != d->prev_octant) {
                d->prev_octant = oct;
                reich_set_phase_deg_immediate(d, (float)oct * 45.0f);
                d->dwell_rem = 2u * (uint32_t)L; /* Lock groove for 2 full bars */
                d->is_dwelling = true;
            }
        } else {
            d->prev_octant = (uint8_t)((uint32_t)(reich_get_phase_turns(d) * 8.0f) & 7u);
        }
    }

    return out;
}
