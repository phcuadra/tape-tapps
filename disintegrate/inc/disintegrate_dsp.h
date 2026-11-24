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

typedef struct {
    float l;
    float r;
} dis_frame_t;

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
