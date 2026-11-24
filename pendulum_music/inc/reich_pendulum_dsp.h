#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * reich_pendulum_dsp.h — Steve Reich 1968 "Pendulum Music" Coupled Feedback Engine
 *
 * Physically grounded, L1-cache-resident 48 kHz float32 simulation of 4 suspended
 * microphones (i = 0..3) swinging on staggered overhead cables above 4 upward-facing
 * floor speakers (j = 0..3) in a shared gallery space:
 *   1. Symplectic Velocity-Verlet nonlinear pendulum kinematics (1.5 kHz sub-rate):
 *        d2_theta_i/dt^2 = -(g / L_i) * sin(theta_i) - gamma * d_theta_i/dt
 *      capturing amplitude-dependent period dispersion, cable-length phasing (SPREAD),
 *      perpetual vs gradual decay into a resting 4-voice feedback chord (DECAY),
 *      Unison/Staggered Pull & Release, and the 1968 "Pull Power Plug" ending.
 *   2. Full 4x4 speaker-to-microphone 3D Euclidean acoustic propagation matrix
 *      (4-point Hermite cubic on self-feedback paths, linear on cross-bleed paths,
 *      inside a compact 21 KB workspace whose active window fits inside STM32H7
 *      16 KB L1 D-Cache), producing Barkhausen/Doppler pitch sweeps, constructive
 *      in-phase surges, and destructive out-of-phase cancellation (COUPLE).
 *   3. Warm 1968 PA preamp + 2-mode Cytomic ZDF SVF speaker/cabinet voicings with
 *      zero-division C1 cubic soft saturation and internal DC blocking.
 *   4. Per-mic post-interference feedback envelope followers env[0..3] in [0, 1]
 *      driving Mode 2 ("SOUND VCA") with zero-RAM-traffic real-time register
 *      synthesis for 4 built-in sound banks + custom per-slot recorded/loaded loops.
 *
 * Strictly single-precision float32 & 32-bit integer arithmetic (zero doubles,
 * zero 64-bit integer divisions, zero audio-rate libm calls).
 */

#define PM_NUM_MICS        4u

#define PM_SPK_DELAY_LEN   1024u
#define PM_SPK_DELAY_MASK  (PM_SPK_DELAY_LEN - 1u)

#define PM_ROOM_LEN        512u
#define PM_ROOM_MASK       (PM_ROOM_LEN - 1u)

#define PM_AP_LEN          128u
#define PM_AP_MASK         (PM_AP_LEN - 1u)

typedef struct {
    float l;
    float r;
} pm_frame_t;

enum {
    PM_TUNING_FIFTHS = 0,   /* Gallery Fifths: G3 - D4 - A4 - D5 open Reich fifths */
    PM_TUNING_MAJ9,         /* Warm Maj9:      C4 - E4 - G4 - D5 luminous gallery  */
    PM_TUNING_RAW_1968,     /* Raw 1968 PA:    beating close unison PA cluster     */
    PM_TUNING_COUNT,
};

enum {
    PM_BANK_ORGANS = 0,     /* Organ Quartet (Four Organs reed/pipe stops)         */
    PM_BANK_MARIMBA,        /* Marimba & Bells (Six Marimbas / Drumming rolls)     */
    PM_BANK_CHOIR,          /* Tape Choir (cello + vowel formants + triangle)      */
    PM_BANK_GLASS,          /* Bowed Glass (crystalline singing glass armonica)    */
    PM_BANK_COUNT,
};

/*
 * Compact delay & gallery reflection workspace (21,504 bytes = 21.0 KB;
 * active delay window < 13 KB so it stays 100% resident in 16 KB L1 D-Cache).
 */
typedef struct {
    float spk_line[PM_NUM_MICS][PM_SPK_DELAY_LEN];  /* 4 * 1024 floats = 16 KB */
    float room_l[PM_ROOM_LEN];                      /* 512 floats      =  2 KB */
    float room_r[PM_ROOM_LEN];                      /* 512 floats      =  2 KB */
    float ap_l[PM_AP_LEN];                          /* 128 floats      = 0.5KB */
    float ap_r[PM_AP_LEN];                          /* 128 floats      = 0.5KB */
} pend_space_t;

typedef struct {
    float sample_rate;
    float inv_fs;

    /* Normalized performance controls [0..1] */
    float gain_norm;
    float couple_norm;
    float spread_norm;
    float decay_norm;

    uint8_t tuning_mode;
    uint8_t bank_idx;
    bool    release_stagger;
    bool    plug_pulled;
    uint8_t phys_sub;

    /* PA power supply rail [0..1] */
    float pa_power;
    float pa_power_tgt;

    /* Per-pendulum nonlinear kinematics & 3D capsule geometry */
    float theta[PM_NUM_MICS];
    float omega[PM_NUM_MICS];
    float cable_len[PM_NUM_MICS];
    float cable_tgt[PM_NUM_MICS];
    float px[PM_NUM_MICS];
    float pz[PM_NUM_MICS];
    float deg[PM_NUM_MICS];
    float norm_u[PM_NUM_MICS];
    float norm_v[PM_NUM_MICS];

    /* Precomputed Mode 2 constant-power stereo pan gains (updated at 1.5 kHz) */
    float vca_pan_l[PM_NUM_MICS];
    float vca_pan_r[PM_NUM_MICS];

    /* 4x4 acoustic propagation delays (samples) & cross-coupling gains */
    float del_tgt[PM_NUM_MICS][PM_NUM_MICS];
    float del_cur[PM_NUM_MICS][PM_NUM_MICS];
    float cpl_tgt[PM_NUM_MICS][PM_NUM_MICS];
    float cpl_cur[PM_NUM_MICS][PM_NUM_MICS];
    float self_gain_tgt[PM_NUM_MICS];
    float self_gain_cur[PM_NUM_MICS];
    float loop_gain_tgt[PM_NUM_MICS];
    float loop_gain_cur[PM_NUM_MICS];

    /* Per-channel PA speaker passband, 2-mode ZDF SVF body & DC blocker */
    float hp_x[PM_NUM_MICS];
    float hp_y[PM_NUM_MICS];
    float lp_y[PM_NUM_MICS];
    float svf1_g[PM_NUM_MICS];
    float svf1_k[PM_NUM_MICS];
    float svf1_a1[PM_NUM_MICS];
    float svf1_a2[PM_NUM_MICS];
    float svf1_a3[PM_NUM_MICS];
    float svf1_ic1[PM_NUM_MICS];
    float svf1_ic2[PM_NUM_MICS];

    float svf2_g;
    float svf2_k;
    float svf2_a1;
    float svf2_a2;
    float svf2_a3;
    float svf2_ic1[PM_NUM_MICS];
    float svf2_ic2[PM_NUM_MICS];

    float tau_trim[PM_NUM_MICS];
    float dc_r;
    float dc_x[PM_NUM_MICS];
    float dc_y[PM_NUM_MICS];
    float last_spk[PM_NUM_MICS];

    /* Per-mic feedback level followers & anti-zipper VCA gains [0..1] */
    float env_raw[PM_NUM_MICS];
    float env[PM_NUM_MICS];
    float vca_gain[PM_NUM_MICS];

    /* Real-time zero-RAM-traffic synthesizer state for built-in Mode 2 banks */
    uint32_t synth_ph[PM_NUM_MICS][3];
    uint32_t synth_inc[PM_NUM_MICS][3];
    uint32_t lfo_ph[PM_NUM_MICS];
    uint32_t lfo_inc[PM_NUM_MICS];

    /* Live inter-mic phase coherence telemetry for OLED */
    float pair_coh[3];
    float phase_sync;

    /* Acoustic delay & gallery room reflection workspace */
    pend_space_t* space;
    uint32_t spk_wr;
    uint32_t room_wr;
    uint32_t ap_wr;
    float room_lp_l;
    float room_lp_r;
    float out_dc_xl;
    float out_dc_yl;
    float out_dc_xr;
    float out_dc_yr;

    /* Deterministic PRNG for gallery thermal air excitation floor */
    uint32_t rng_state;
} reich_pendulum_dsp_t;

void pm_dsp_init(reich_pendulum_dsp_t* d, float sample_rate);
void pm_dsp_attach_space(reich_pendulum_dsp_t* d, pend_space_t* space);
void pm_dsp_clear_audio(reich_pendulum_dsp_t* d);

void pm_set_gain(reich_pendulum_dsp_t* d, float norm);
void pm_set_couple(reich_pendulum_dsp_t* d, float norm);
void pm_set_spread(reich_pendulum_dsp_t* d, float norm);
void pm_set_decay(reich_pendulum_dsp_t* d, float norm);
void pm_set_tuning(reich_pendulum_dsp_t* d, uint8_t tuning_mode);
void pm_set_bank(reich_pendulum_dsp_t* d, uint8_t bank_idx);
void pm_set_release_stagger(reich_pendulum_dsp_t* d, bool stagger);

void pm_pull_and_release(reich_pendulum_dsp_t* d);
void pm_set_pendulum_state(reich_pendulum_dsp_t* d, uint32_t idx, float theta_rad, float omega_rad_s);
void pm_pull_plug(reich_pendulum_dsp_t* d);
bool pm_all_nearly_still(const reich_pendulum_dsp_t* d, float deg_thresh);

const char* pm_tuning_name(uint8_t tuning_mode);
const char* pm_bank_name(uint8_t bank_idx);
const char* pm_bank_slot_tag(uint8_t bank_idx, uint8_t slot_idx);

float pm_clip_guard(float x);
float pm_soft_limit(float x);

void pm_dsp_step(reich_pendulum_dsp_t* d, pm_frame_t* out_fbk);

/*
 * Render Mode 2 ("SOUND VCA"):
 * For each mic i = 0..3, if bit `i` of `custom_mask` is set and `custom_in` is non-NULL,
 * uses `custom_in[i]` (recorded/loaded audio); otherwise synthesizes the active built-in
 * sound bank voice in registers with zero PSRAM traffic.
 */
pm_frame_t pm_render_sound_vca(reich_pendulum_dsp_t* d,
                               const pm_frame_t custom_in[PM_NUM_MICS],
                               uint8_t custom_mask);
