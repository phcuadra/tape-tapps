#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * disintegrate_dsp.h — Destructive Basinski-style magnetic tape oxide erosion engine
 *
 * Faithful C port of phcuadra/Spotykach's Disintegrate Grit mode (feat/disintegrate-mode),
 * adapted for tape! (ARM Cortex-M7, single-precision float32 only).
 */

/*
 * 4-byte 32-bit-aligned packed stereo frame (192 KB/sec at 48 kHz — 3x smaller
 * than float32 + Q15 backup, enabling 8–10s loops in *tape!* hardware PSRAM).
 * Full scale is ±1.2000f (+1.58 dBFS analog headroom matching dis_soft_clip_guard),
 * accessed strictly as 32-bit words so OCTOSPI PSRAM never sees halfword accesses.
 */
#define DIS_Q_SCALE     27306.0f
#define DIS_Q_INV       (1.0f / 27306.0f)

typedef struct __attribute__((aligned(4))) {
    uint32_t lr;    /* low 16 bits: int16_t L, high 16 bits: int16_t R */
} dis_frame_t;

static inline int16_t dis_f32_to_q(float x) {
    if (x >  1.2f) x =  1.2f;
    if (x < -1.2f) x = -1.2f;
    const float s = x * DIS_Q_SCALE;
    int32_t q = (int32_t)(s >= 0.0f ? (s + 0.5f) : (s - 0.5f));
    if (q >  32767) q =  32767;
    if (q < -32767) q = -32767;
    return (int16_t)q;
}

static inline dis_frame_t dis_frame_pack(float l, float r) {
    const uint16_t ul = (uint16_t)dis_f32_to_q(l);
    const uint16_t ur = (uint16_t)dis_f32_to_q(r);
    dis_frame_t f = { (uint32_t)ul | ((uint32_t)ur << 16) };
    return f;
}

static inline void dis_frame_unpack(dis_frame_t f, float* out_l, float* out_r) {
    const int16_t ql = (int16_t)(uint16_t)(f.lr & 0xFFFFu);
    const int16_t qr = (int16_t)(uint16_t)(f.lr >> 16);
    *out_l = (float)ql * DIS_Q_INV;
    *out_r = (float)qr * DIS_Q_INV;
}

typedef struct {
    float sample_rate;
    float intensity;     /* [0..1] Erosion rate (Spotykach: intensity) */
    float character;     /* [0..1] Oxide dropout patch size: fine pitting -> large sheets */
    float mix;           /* [0..1] HF tape wear, saturation warmth & micro-crackle */
    float gate;          /* [0..1] Smooth on/off SoftSwitch gate */

    /* Precomputed DSP parameters */
    float rate;
    float spatial_freq;
    float drift_inc;
    float stereo_link;

    /* Slowly evolving tape-defect drift & band-limited oxide crackle state */
    float drift_phase;
    float detail_phase;
    float dust_lp_l;
    float dust_lp_r;
    float activity;

    uint32_t rng_state;
    uint32_t tape_seed_l;
    uint32_t tape_seed_r;
} dis_engine_t;

/* Initialize engine with sample rate (typically 48000.0f) and default parameters */
void dis_init(dis_engine_t* d, float sample_rate);

/* Re-roll physical tape oxide defect seeds & reset drift for a fresh loop */
void dis_reset_tape(dis_engine_t* d);

/* Parameter setters (all normalized [0..1]) */
void dis_set_intensity(dis_engine_t* d, float norm);
void dis_set_character(dis_engine_t* d, float norm);
void dis_set_mix(dis_engine_t* d, float norm);
void dis_set_gate(dis_engine_t* d, float gate);

/* True when gate > 0 and intensity above active threshold */
bool dis_is_active(const dis_engine_t* d);

/* Output safety clamp matching Spotykach Disintegrate::process */
float dis_soft_clip_guard(float x);

/*
 * Destructive per-frame tape wear applied directly to the stereo float loop buffer
 * as the playhead steps past frame `idx`.
 */
void dis_erode_frame(dis_engine_t* d, dis_frame_t* buf, uint32_t rec_size,
                     int32_t idx, float weight);

/*
 * Step-based erosion helper (ported from Spotykach Buffer::disintegrate_step):
 * erodes only frames the readhead has just stepped past (up to 8 frames per sample)
 * so active linear interpolation [curr_i, curr_i + 1] never reads a mid-stepped boundary.
 */
void dis_step_erode(dis_engine_t* d, dis_frame_t* buf, uint32_t rec_size,
                    int32_t prev_i, int32_t step, float weight);

/*
 * Read-only query of the current C1-periodic oxide flaking field [0..1] at frame `idx`
 * (used by the UI visualizer to render where weak spots and peeling sheets sit on the tape).
 */
float dis_sample_flake_vis(const dis_engine_t* d, uint32_t rec_size, uint32_t idx);
