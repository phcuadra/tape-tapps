#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * reich_dsp.h — Steve Reich "It's Gonna Rain" (1965) Tape-Phasing Engine
 *
 * Models two (Part I) or four (Part II) virtual Wollensak reel-to-reel tape decks
 * playing a shared magnetic loop from unison (0 deg) through resonant comb-filtering,
 * slapback doubling, polyrhythmic hockets, and contrapuntal canons all the way back
 * around to unison (360 deg).
 *
 * Strictly single-precision float32 & 32-bit integer arithmetic for ARM Cortex-M7.
 */

#define REICH_MAX_VOICES 4u

typedef struct {
    float l;
    float r;
} reich_frame_t;

enum {
    REICH_MOTION_CONTINUOUS = 0, /* Classic 1965 "It's Gonna Rain" continuous capstan slip */
    REICH_MOTION_CANON_DWELL = 1 /* "Piano Phase" stepped glide + 2-bar canon lock dwell  */
};

typedef struct {
    float sample_rate;

    /* Normalized user controls */
    float drift_norm;     /* [0..1] DRIFT: relative capstan speed differential */
    float spread_norm;    /* [0..1] SPREAD: 0% mono comb-flange -> 100% L/R stereo hocket */
    float tape_norm;      /* [0..1] TAPE: analog capstan micro-wow/flutter & head warmth */

    uint8_t num_decks;    /* 2 (Part I) or 4 (Part II) */
    uint8_t motion_mode;  /* REICH_MOTION_CONTINUOUS or REICH_MOTION_CANON_DWELL */
    bool drifting;        /* true = PHASING (slip active), false = LOCKED (0.000% slip) */
    bool is_dwelling;     /* true while pausing on a stepped canon division */

    /* Active loop & non-destructive sub-loop window geometry */
    uint32_t full_len;
    uint32_t win_start;
    uint32_t win_len;
    float inv_win_len;

    /* Deck A reference capstan playhead: [0 .. win_len) */
    int32_t head_a_i;
    float head_a_f;

    /*
     * Deck B relative phase lead ahead of Deck A in sample-accurate (int + frac) form.
     * Keeping phase in [0 .. win_len) integer + [0 .. 1) float samples avoids float32
     * ULP underflow even at ultra-slow 0.05% Reich drift on 24-second loops, and holds
     * 0.0000 deg relative lock indefinitely when LOCKED.
     */
    int32_t phase_i;
    float phase_f;

    /* Smooth "thumb-on-reel-flange" capstan servo slew accumulator (in samples) */
    float slew_samples;

    /* Cycle / lap counters */
    uint32_t lap_count;       /* Deck A loop revolutions */
    uint32_t phase_cycles;    /* Full 360 deg phase revolutions completed by Deck B */

    /* Canon Dwell state ("Piano Phase" mode) */
    uint8_t prev_octant;
    uint32_t dwell_rem;

    /* Derived DSP coefficients */
    float drift_ratio;        /* Relative speed slip (e.g. 0.0008 .. 0.25) */
    float wow_depth_samples;  /* Zero-mean capstan wow/flutter excursion in samples */
    float voice_scale;        /* Per-voice gain normalization */
    float pan_l[REICH_MAX_VOICES];
    float pan_r[REICH_MAX_VOICES];

    /* Independent capstan motor LFO phases [0 .. 1) turns */
    float wow_ph_a;
    float wow_ph_b;
    float wow_ph_c;
    float wow_ph_d;
    float flut_ph_a;
    float flut_ph_b;

    /* Analog Wollensak tape head bump (~120 Hz) & oxide HF roll-off filter states */
    float bump_a;
    float bump_gain;
    float hf_a;
    float drive_gain;
    float bump_lp_l;
    float bump_lp_r;
    float hf_lp_l;
    float hf_lp_r;

    /* Live per-head positions (within [0 .. win_len)) and amplitude envelopes for UI/LEDs */
    int32_t voice_pos_i[REICH_MAX_VOICES];
    float voice_env[REICH_MAX_VOICES];
} reich_engine_t;

/* Initialize engine at sample_rate (48000.0f) with musical defaults */
void reich_init(reich_engine_t* d, float sample_rate);

/* Reset playheads, phase (0.0 deg unison), filter states, and loop geometry */
void reich_reset_loop(reich_engine_t* d, uint32_t full_len, uint32_t win_start, uint32_t win_len);

/* Non-destructively update sub-loop trim window while preserving normalized phase angle */
void reich_set_window(reich_engine_t* d, uint32_t full_len, uint32_t win_start, uint32_t win_len);

/* Parameter setters */
void reich_set_drift(reich_engine_t* d, float norm);
void reich_set_spread(reich_engine_t* d, float norm);
void reich_set_tape(reich_engine_t* d, float norm);
void reich_set_decks(reich_engine_t* d, uint8_t num_decks);
void reich_set_motion_mode(reich_engine_t* d, uint8_t mode);
void reich_set_drifting(reich_engine_t* d, bool drifting);

/* Phase angle queries & reel-flange controls */
float reich_get_phase_turns(const reich_engine_t* d);
float reich_get_phase_deg(const reich_engine_t* d);
float reich_get_offset_ms(const reich_engine_t* d);

/* Set exact phase angle in [0 .. 360) immediately (no slew) */
void reich_set_phase_deg_immediate(reich_engine_t* d, float deg);

/* Smoothly nudge target phase angle by delta_deg (simulates thumb/finger on reel flange) */
void reich_nudge_phase_deg(reich_engine_t* d, float delta_deg);

/* Smoothly slew along shortest arc [-180 .. +180] to target_deg */
void reich_slew_to_phase_deg(reich_engine_t* d, float target_deg);

/* Smoothly slew to nearest canonical Reich subdivision (0, 45, 60, 90, 120, 135, 180, ...) */
float reich_snap_nearest_canon(reich_engine_t* d, const char** out_label);

/* Short acoustic zone description for OLED status strip ("UNISON", "FLANGE", "180 HOCKET", ...) */
const char* reich_describe_zone(const reich_engine_t* d);

/* Odd-symmetric C1 Padé rational tanh soft tape limiter in [-0.98, +0.98] */
float reich_tape_limit(float x);

/* Hard input safety clamp against out-of-range float inputs */
float reich_clip_guard(float x);

/*
 * Render one stereo frame from `loop_buf` across 2 (Part I) or 4 (Part II) Hermite-interpolated
 * virtual tape heads, advancing Deck A by `speed` and Deck B's phase by capstan drift + slew.
 */
reich_frame_t reich_process_frame(reich_engine_t* d, const reich_frame_t* loop_buf,
                                  float speed, bool allow_drift);
