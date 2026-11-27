#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * lucier_dsp.h — Alvin Lucier "I Am Sitting in a Room" (1969) Iterative Room-Resonance Engine
 *
 * Models generation-by-generation acoustic re-recording of a tape loop through a physical
 * room's modal transfer function + loudspeaker/microphone spectral tilt EQ + early wall
 * reflections + energy-governed AGC & soft tape limiting (ARM Cortex-M7, float32 only).
 */

#define LUCIER_NUM_MODES   8u
#define LUCIER_GEOM_COUNT  4u

#define LUCIER_REFL_LEN    2048u
#define LUCIER_REFL_MASK   (LUCIER_REFL_LEN - 1u)
#define LUCIER_AP1_LEN     512u
#define LUCIER_AP1_MASK    (LUCIER_AP1_LEN - 1u)
#define LUCIER_AP2_LEN     1024u
#define LUCIER_AP2_MASK    (LUCIER_AP2_LEN - 1u)

typedef struct {
    float l;
    float r;
} lucier_frame_t;

/*
 * 4-byte 32-bit-aligned packed stereo loop frame (192 KB/sec at 48 kHz — 3x smaller
 * than float32 + Q15 backup, enabling 8–10s loops in *tape!* hardware PSRAM).
 * Full scale is ±1.2000f (+1.58 dBFS analog headroom matching lucier_clip_guard),
 * accessed strictly as 32-bit words so OCTOSPI PSRAM never sees halfword accesses.
 */
#define LUCIER_Q_SCALE  27306.0f
#define LUCIER_Q_INV    (1.0f / 27306.0f)

typedef struct __attribute__((aligned(4))) {
    uint32_t lr;    /* low 16 bits: int16_t L, high 16 bits: int16_t R */
} lucier_pframe_t;

static inline int16_t lucier_f32_to_q(float x) {
    if (x >  1.2f) x =  1.2f;
    if (x < -1.2f) x = -1.2f;
    const float s = x * LUCIER_Q_SCALE;
    int32_t q = (int32_t)(s >= 0.0f ? (s + 0.5f) : (s - 0.5f));
    if (q >  32767) q =  32767;
    if (q < -32767) q = -32767;
    return (int16_t)q;
}

static inline lucier_pframe_t lucier_frame_pack(float l, float r) {
    const uint16_t ul = (uint16_t)lucier_f32_to_q(l);
    const uint16_t ur = (uint16_t)lucier_f32_to_q(r);
    lucier_pframe_t f = { (uint32_t)ul | ((uint32_t)ur << 16) };
    return f;
}

static inline lucier_frame_t lucier_frame_unpack(lucier_pframe_t f) {
    const int16_t ql = (int16_t)(uint16_t)(f.lr & 0xFFFFu);
    const int16_t qr = (int16_t)(uint16_t)(f.lr >> 16);
    lucier_frame_t out = { (float)ql * LUCIER_Q_INV, (float)qr * LUCIER_Q_INV };
    return out;
}

/*
 * External room reflection & Schroeder allpass delay memory (~28 KB).
 * Allocate via os_malloc(sizeof(lucier_space_t)) in init() so the TAPP model stays small.
 */
typedef struct {
    float refl_l[LUCIER_REFL_LEN];
    float refl_r[LUCIER_REFL_LEN];
    float ap1_l[LUCIER_AP1_LEN];
    float ap1_r[LUCIER_AP1_LEN];
    float ap2_l[LUCIER_AP2_LEN];
    float ap2_r[LUCIER_AP2_LEN];
} lucier_space_t;

enum {
    LUCIER_GEOM_ROOM_1969 = 0,
    LUCIER_GEOM_TILED_CHAMBER,
    LUCIER_GEOM_STONE_CHAPEL,
    LUCIER_GEOM_CISTERN,
};

typedef struct {
    float sample_rate;

    /* Normalized user controls */
    float pass_amt;      /* [0..1]   PASS: per-generation room + EQ re-recording depth */
    float room_size;     /* [0..1]   ROOM: physical scale (small chamber -> vast hall) */
    float resonance;     /* [0..1]   RESO: modal Q sharpness & reflection diffusion */
    float tone;          /* [-1..+1] TONE: speaker/mic spectral tilt (warm <-> bright) */
    float gate;          /* [0..1]   Smooth SoftSwitch re-recording gate */
    uint8_t geom_idx;    /* [0..3]   Architectural room modal ratio preset */

    /* Derived coefficients */
    float pass_mix;
    float f_base;
    float modal_q;
    float damp_k[LUCIER_NUM_MODES];
    float mode_freq[LUCIER_NUM_MODES];
    float mode_weight[LUCIER_NUM_MODES];

    /* Left/Right ZDF trapezoidal SVF coefficients */
    float a1_l[LUCIER_NUM_MODES];
    float a2_l[LUCIER_NUM_MODES];
    float a3_l[LUCIER_NUM_MODES];
    float a1_r[LUCIER_NUM_MODES];
    float a2_r[LUCIER_NUM_MODES];
    float a3_r[LUCIER_NUM_MODES];

    /* ZDF SVF integrator states */
    float ic1_l[LUCIER_NUM_MODES];
    float ic2_l[LUCIER_NUM_MODES];
    float ic1_r[LUCIER_NUM_MODES];
    float ic2_r[LUCIER_NUM_MODES];

    /* Per-mode live amplitude envelopes (drives equilibrium + OLED visualizer + LEDs) */
    float mode_env[LUCIER_NUM_MODES];
    float mode_env_a;
    float modal_focus;   /* [0..1] smoothed spectral concentration metric */

    /* Per-pass speaker/mic EQ filter states & coeffs */
    float hpf_a;
    float lpf_a;
    float piv_a;
    float tilt_low_g;
    float tilt_high_g;
    float direct_floor;
    float sub_lp_l;
    float sub_lp_r;
    float spk_lp_l;
    float spk_lp_r;
    float piv_lp_l;
    float piv_lp_r;

    /* 1-pole output DC blocker (~18 Hz) */
    float dc_r;
    float dc_x_l;
    float dc_y_l;
    float dc_x_r;
    float dc_y_r;

    /* Early reflections & allpass diffusion state */
    lucier_space_t* space;
    uint32_t refl_wr;
    uint32_t ap1_wr;
    uint32_t ap2_wr;
    uint16_t tap_l[3];
    uint16_t tap_r[3];
    uint16_t ap1_del_l;
    uint16_t ap1_del_r;
    uint16_t ap2_del_l;
    uint16_t ap2_del_r;
    float ap_fb;
    float refl_mix;

    /* Matched-envelope AGC & energy governor */
    float target_rms;
    float dry_env_sq;
    float wet_env_sq;
    float agc_gain;
    float agc_a;
    float gain_slew;
    uint32_t rng_state;
} lucier_engine_t;

/* Initialize engine with sample rate (48000.0f) and default Middletown 1969 room parameters */
void lucier_init(lucier_engine_t* d, float sample_rate);

/* Attach optional os_malloc'd delay workspace for early reflections & allpass diffusion */
void lucier_attach_space(lucier_engine_t* d, lucier_space_t* space);

/* Clear filter, reflection, DC blocker, and AGC states for a newly captured/restored loop */
void lucier_reset_state(lucier_engine_t* d, float initial_rms);

/* Human-readable name for room geometry preset [0..3] */
const char* lucier_geom_name(uint8_t geom_idx);
const char* lucier_geom_short_name(uint8_t geom_idx);

/* Parameter setters */
void lucier_set_pass(lucier_engine_t* d, float norm);
void lucier_set_room(lucier_engine_t* d, float norm);
void lucier_set_reso(lucier_engine_t* d, float norm);
void lucier_set_tone(lucier_engine_t* d, float bipolar);
void lucier_set_gate(lucier_engine_t* d, float gate);
void lucier_set_geometry(lucier_engine_t* d, uint8_t geom_idx);

/* True when gate > 0 and pass_amt above active threshold */
bool lucier_is_active(const lucier_engine_t* d);

/* Smooth C1 rational tanh tape limiter in [-0.95, +0.95] */
float lucier_tape_limit(float x);

/* Hard safety guard against out-of-range input */
float lucier_clip_guard(float x);

/*
 * Run one stereo frame through the acoustic room + EQ + AGC chain at sample rate.
 * When `write_enable` > 0, returns the blended next-generation stereo sample;
 * also updates modal envelopes & AGC priming smoothly even when `write_enable` == 0.
 */
lucier_frame_t lucier_process_frame(lucier_engine_t* d, lucier_frame_t in, float write_enable);

/*
 * Step-safe in-place loop re-recorder: writes `next_gen` back into 32-bit packed `buf`
 * using unbiased stochastic rounding only on frames the playhead has just stepped past.
 */
void lucier_commit_step(lucier_engine_t* d, lucier_pframe_t* buf, uint32_t rec_size,
                        int32_t prev_i, int32_t step, lucier_frame_t next_gen, float gate);
