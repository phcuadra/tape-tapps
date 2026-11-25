#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * pendulum_dsp.h — Aphex Twin Barbican Hall "Swinging Piano" Acoustic Engine
 *
 * Physically grounded 48 kHz stereo float32 simulation of a suspended concert
 * grand piano swinging in a 1D stage-wide arc or 2D elliptical Lissajous orbit
 * above a stationary stereo hall microphone pair:
 *   1. Nonlinear gravitational pendulum ODE (Velocity-Verlet integration at
 *      1.5 kHz, zero external libm calls) with phase-coherent escapement drive
 *      (SWINGING) vs. natural free-decay coasting (COASTING) + impulse push.
 *   2. Frame-interleaved 3D Euclidean soundboard-to-microphone propagation read
 *      via 4-point Hermite cubic fractional delay lines (authentic Doppler pitch
 *      bending, Interaural Time Difference, inverse-distance proximity swell,
 *      and dynamic soundboard/air-absorption low-pass filtering).
 *   3. Opposite-wall lateral Barbican early reflections feeding a modulated
 *      4-line Householder FDN concert-hall reverb, 18 Hz DC blocker, and
 *      zero-division odd-symmetric C1 cubic soft limiter.
 *   4. Chunked & single-shot built-in "Seed Felt Piano" generator rendering a
 *      seamless stereo 48 kHz aisatsana [102]-inspired felt-piano loop.
 */

#define SWING_SRC_LEN          4096u   /* 85.3 ms @ 48 kHz = 29.3 m propagation */
#define SWING_SRC_MASK         (SWING_SRC_LEN - 1u)

#define SWING_FDN0_LEN         1024u
#define SWING_FDN0_MASK        (SWING_FDN0_LEN - 1u)
#define SWING_FDN_LEN          2048u
#define SWING_FDN_MASK         (SWING_FDN_LEN - 1u)

#define SWING_AP_LEN           256u
#define SWING_AP_MASK          (SWING_AP_LEN - 1u)

#define SWING_RIG_COUNT        4u
#define PEND_FELT_NOTES_COUNT  24u

typedef struct {
    float l;
    float r;
} swing_frame_t;

enum {
    SWING_RIG_BARBICAN = 0,   /* Barbican 10m: wide concert stage, spaced hall pair */
    SWING_RIG_ELLIPTICAL,     /* Elliptical 2D: quadrature X-Y orbit over mics      */
    SWING_RIG_CHAPEL,         /* Chapel 5m: intimate stone walls, warm decay        */
    SWING_RIG_CLOSE,          /* Close 2.4m: dramatic overhead close-mic swoop      */
};

/*
 * Compact 62.0 KB interleaved delay & concert-hall FDN workspace (63,488 B).
 * Frame-interleaved `src_ring` keeps L & R soundboard radiation in the same
 * L1 D-Cache line with zero 32 KB set-conflict aliasing.
 */
typedef struct {
    swing_frame_t src_ring[SWING_SRC_LEN]; /* 4096 * 8 B = 32 KB interleaved L/R */
    float         fdn0[SWING_FDN0_LEN];    /* 1024 * 4 B =  4 KB                 */
    float         fdn1[SWING_FDN_LEN];     /* 2048 * 4 B =  8 KB                 */
    float         fdn2[SWING_FDN_LEN];     /* 2048 * 4 B =  8 KB                 */
    float         fdn3[SWING_FDN_LEN];     /* 2048 * 4 B =  8 KB                 */
    float         ap_l[SWING_AP_LEN];      /*  256 * 4 B =  1 KB                 */
    float         ap_r[SWING_AP_LEN];      /*  256 * 4 B =  1 KB                 */
} swing_space_t;

typedef struct {
    float sample_rate;
    float inv_fs;

    /* Normalized user controls [0..1] */
    float rope_norm;
    float arc_norm;
    float doppler_norm;
    float hall_norm;

    uint8_t rig_mode;
    bool swing_on;

    /* Physical pendulum state (radians & rad/s) */
    float rope_len_target;
    float rope_len;
    float theta_max;
    float theta_x;
    float omega_x;
    float theta_y;
    float omega_y;

    /* Sub-block physics decimation counter (1.5 kHz Velocity-Verlet updates) */
    uint8_t phys_rem;

    /* Current 3D soundboard Cartesian coordinates (meters) */
    float x_p;
    float y_p;
    float z_p;

    /* Target & cascaded 2-pole smoothed propagation delays (samples) */
    float del_dir_l_tgt;
    float del_dir_r_tgt;
    float del_wall_l_tgt;
    float del_wall_r_tgt;

    float del_dir_l_s1;
    float del_dir_l;
    float del_dir_r_s1;
    float del_dir_r;

    float del_wall_l_s1;
    float del_wall_l;
    float del_wall_r_s1;
    float del_wall_r;

    /* Target & smoothed per-mic direct gains and air-absorption LP coeffs */
    float gain_l_tgt;
    float gain_r_tgt;
    float gain_l;
    float gain_r;
    float lp_a_l_tgt;
    float lp_a_r_tgt;
    float lp_a_l;
    float lp_a_r;

    /* Soundboard radiation & wall absorption 1-pole low-pass states */
    float air_lp_l;
    float air_lp_r;
    float wall_lp_l;
    float wall_lp_r;

    /* Concert hall FDN & diffuser states */
    swing_space_t* space;
    uint32_t src_wr;
    uint32_t fdn0_wr;
    uint32_t fdn_wr;
    uint32_t ap_wr;
    float fdn_lp[4];
    uint32_t mod_ph_a;
    uint32_t mod_ph_b;
    uint32_t mod_inc_a;
    uint32_t mod_inc_b;

    /* Derived hall coefficients */
    float er_gain;
    float hall_wet;
    float fdn_fb;
    float fdn_damp;
    float rig_size_scale;

    /* 18 Hz 1-pole stereo output DC blocker */
    float dc_r;
    float dc_x_l;
    float dc_y_l;
    float dc_x_r;
    float dc_y_r;

    /* Live telemetry for OLED visualizer, RGB LEDs & tests */
    float live_deg_x;
    float live_deg_y;
    float live_cents;
    float prox_env_l;
    float prox_env_r;
    float hall_env;
} pendulum_engine_t;

void pendulum_init(pendulum_engine_t* d, float sample_rate);
void pendulum_attach_space(pendulum_engine_t* d, swing_space_t* space);
void pendulum_clear_audio(pendulum_engine_t* d);

void pendulum_set_rope(pendulum_engine_t* d, float norm);
void pendulum_set_arc(pendulum_engine_t* d, float norm);
void pendulum_set_doppler(pendulum_engine_t* d, float norm);
void pendulum_set_hall(pendulum_engine_t* d, float norm);
void pendulum_set_rig(pendulum_engine_t* d, uint8_t rig_mode);
void pendulum_set_swing_on(pendulum_engine_t* d, bool swing_on);

void pendulum_push(pendulum_engine_t* d, float impulse_norm);
void pendulum_center_still(pendulum_engine_t* d);
float pendulum_get_energy(const pendulum_engine_t* d);

const char* pendulum_rig_name(uint8_t rig_mode);
const char* pendulum_rig_short_name(uint8_t rig_mode);

void pendulum_step_idle(pendulum_engine_t* d, float dt_sec);
float pendulum_soft_limit(float x);
float pendulum_clip_guard(float x);

/* Single-frame wrapper & fast fused block processor */
swing_frame_t pendulum_process_frame(pendulum_engine_t* d, swing_frame_t dry);

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
                            uint32_t frames);

/* Chunked & full-shot built-in felt-piano generator */
uint32_t pendulum_seed_felt_begin(swing_frame_t* buf, uint32_t max_frames, float sample_rate);
void pendulum_seed_felt_notes(swing_frame_t* buf, uint32_t target_len, float sample_rate,
                              uint32_t note_start, uint32_t note_count);
void pendulum_seed_felt_finish(swing_frame_t* buf, uint32_t target_len, float sample_rate);
uint32_t pendulum_seed_felt_piano(swing_frame_t* buf, uint32_t max_frames, float sample_rate);
