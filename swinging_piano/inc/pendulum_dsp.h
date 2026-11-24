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
 *   1. Nonlinear gravitational pendulum ODE (Velocity-Verlet integration) with
 *      phase-coherent escapement drive (SWINGING) vs. natural free-decay coasting
 *      (COASTING) + angular-momentum impulse push.
 *   2. Exact 3D Euclidean soundboard-to-microphone propagation read via 4-point,
 *      3rd-order Hermite cubic fractional delay lines (authentic Doppler pitch
 *      bending, true Interaural Time Difference, inverse-distance proximity swell,
 *      and dynamic soundboard/air-absorption low-pass filtering).
 *   3. Opposite-wall lateral Barbican early reflections (counter-Doppler spatial
 *      chorus) feeding a modulated 4-line Householder FDN concert-hall reverb,
 *      18 Hz DC blocker, and odd-symmetric C1 Pade rational tanh soft limiter.
 *   4. Built-in "Seed Felt Piano" generator rendering a seamless stereo 48 kHz
 *      aisatsana [102]-inspired felt-piano progression into loop RAM.
 *
 * Strictly single-precision float32 & 32-bit integer arithmetic (zero doubles,
 * zero 64-bit integer divisions).
 */

#define SWING_SRC_LEN     8192u
#define SWING_SRC_MASK    (SWING_SRC_LEN - 1u)

#define SWING_FDN0_LEN    1024u
#define SWING_FDN0_MASK   (SWING_FDN0_LEN - 1u)
#define SWING_FDN_LEN     2048u
#define SWING_FDN_MASK    (SWING_FDN_LEN - 1u)

#define SWING_AP_LEN      512u
#define SWING_AP_MASK     (SWING_AP_LEN - 1u)

#define SWING_RIG_COUNT   4u

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
 * External PSRAM delay & concert-hall FDN workspace (exact 96.0 KB = 98,304 B).
 * Allocate via os_malloc(sizeof(swing_space_t)) in app_init(); engine degrades
 * gracefully to zero-delay panning/proximity/filtering if space == NULL.
 */
typedef struct {
    float src_l[SWING_SRC_LEN];     /* 8192 floats: Left soundboard propagation line  */
    float src_r[SWING_SRC_LEN];     /* 8192 floats: Right soundboard propagation line */
    float fdn0[SWING_FDN0_LEN];     /* 1024 floats: Hall FDN delay line 0             */
    float fdn1[SWING_FDN_LEN];      /* 2048 floats: Hall FDN delay line 1             */
    float fdn2[SWING_FDN_LEN];      /* 2048 floats: Hall FDN delay line 2 (modulated) */
    float fdn3[SWING_FDN_LEN];      /* 2048 floats: Hall FDN delay line 3 (modulated) */
    float ap_l[SWING_AP_LEN];       /* 512 floats:  Left Schroeder input diffuser     */
    float ap_r[SWING_AP_LEN];       /* 512 floats:  Right Schroeder input diffuser    */
} swing_space_t;

typedef struct {
    float sample_rate;
    float inv_fs;

    /* Normalized user controls [0..1] */
    float rope_norm;        /* ROPE: 2.2m (0%) -> 14.0m (100%) suspension cable     */
    float arc_norm;         /* ARC:  0 deg (0%) -> +-55 deg (100%) excursion        */
    float doppler_norm;     /* DOPPLER: 0% zero pitch -> 50% 343 m/s -> 100% close  */
    float hall_norm;        /* HALL: opposite-wall early refl + Barbican FDN tail   */

    uint8_t rig_mode;       /* SWING_RIG_BARBICAN .. SWING_RIG_CLOSE                */
    bool swing_on;          /* true = sustained escapement drive, false = coasting  */

    /* Physical pendulum state (radians & rad/s) */
    float rope_len_target;  /* Target cable length L in meters                      */
    float rope_len;         /* Smoothed active cable length L in meters             */
    float theta_max;        /* Target excursion angle in radians [0 .. 0.960]       */
    float theta_x;          /* Lateral stage swing angle (rad, <0 Left, >0 Right)   */
    float omega_x;          /* Lateral angular velocity d(theta_x)/dt (rad/s)       */
    float theta_y;          /* Front-to-back depth angle (rad)                      */
    float omega_y;          /* Depth angular velocity d(theta_y)/dt (rad/s)         */

    /* Sub-block physics decimation counter (6 kHz Velocity-Verlet updates) */
    uint8_t phys_sub;

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
    float mod_ph_a;
    float mod_ph_b;

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
    float live_deg_x;       /* Current lateral angle in degrees [-62 .. +62]        */
    float live_deg_y;       /* Current depth angle in degrees                       */
    float live_cents;       /* Instantaneous stereo-mean Doppler shift in cents     */
    float prox_env_l;       /* [0..1] Mic L proximity + acoustic energy envelope    */
    float prox_env_r;       /* [0..1] Mic R proximity + acoustic energy envelope    */
    float hall_env;         /* [0..1] Concert hall diffuse tail bloom envelope      */
} pendulum_engine_t;

/* Initialize engine at sample_rate (48000.0f) with Barbican defaults */
void pendulum_init(pendulum_engine_t* d, float sample_rate);

/* Attach PSRAM delay/FDN workspace (NULL-safe) */
void pendulum_attach_space(pendulum_engine_t* d, swing_space_t* space);

/* Clear delay/reverb audio history while optionally keeping pendulum motion */
void pendulum_clear_audio(pendulum_engine_t* d);

/* Parameter setters */
void pendulum_set_rope(pendulum_engine_t* d, float norm);
void pendulum_set_arc(pendulum_engine_t* d, float norm);
void pendulum_set_doppler(pendulum_engine_t* d, float norm);
void pendulum_set_hall(pendulum_engine_t* d, float norm);
void pendulum_set_rig(pendulum_engine_t* d, uint8_t rig_mode);
void pendulum_set_swing_on(pendulum_engine_t* d, bool swing_on);

/* Give the suspended piano an angular-momentum push (impulse_norm in [0.2 .. 1.0]) */
void pendulum_push(pendulum_engine_t* d, float impulse_norm);

/* Gently bring the pendulum to 0 deg center rest */
void pendulum_center_still(pendulum_engine_t* d);

/* Specific mechanical energy E = 0.5*omega^2 + (g/L)*(1 - cos(theta)) (m/s^2 * rad) */
float pendulum_get_energy(const pendulum_engine_t* d);

/* Rig display names */
const char* pendulum_rig_name(uint8_t rig_mode);
const char* pendulum_rig_short_name(uint8_t rig_mode);

/* Step pendulum kinematics during idle UI preview (when audio engine is stopped) */
void pendulum_step_idle(pendulum_engine_t* d, float dt_sec);

/* Odd-symmetric C1 Pade rational tanh soft limiter in [-0.98f, +0.98f] */
float pendulum_soft_limit(float x);

/* Input safety clamp against NaN / out-of-range float samples */
float pendulum_clip_guard(float x);

/* Process one 48 kHz stereo frame through the swinging piano + hall engine */
swing_frame_t pendulum_process_frame(pendulum_engine_t* d, swing_frame_t dry);

/*
 * Render an intimate ~9.6s (or max_frames) stereo 48 kHz felt-piano loop
 * inspired by Aphex Twin's "aisatsana [102]" directly into `buf` with seamless
 * wrap-around damper-off sustain resonance. Returns rendered loop length in frames.
 */
uint32_t pendulum_seed_felt_piano(swing_frame_t* buf, uint32_t max_frames, float sample_rate);
