#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * reich_pendulum_dsp.h — Steve Reich 1968 "Pendulum Music" Coupled Feedback Engine
 *
 * Ultra-lightweight, internal-SRAM-resident 24 kHz float32 simulation (upsampled
 * to 48 kHz via firmware `ups2x_f32_add`) of 4 suspended microphones (i = 0..3)
 * swinging on staggered overhead cables above 4 upward-facing floor speakers
 * (j = 0..3) in a shared gallery space:
 *   1. Symplectic Velocity-Verlet nonlinear pendulum kinematics (375 Hz control
 *      sub-rate, every 64 samples at 24 kHz):
 *        d2_theta_i/dt^2 = -(g / L_i) * sin(theta_i) - gamma * d_theta_i/dt
 *      capturing amplitude-dependent period dispersion, cable-length phasing
 *      (SPREAD), perpetual vs gradual decay into a resting 4-voice feedback
 *      chord (DECAY), and Unison/Staggered Pull & Release.
 *   2. Frame-interleaved speaker delay ring (`pm_quad_t spk_ring[256]`, 4 KB in
 *      internal SRAM) with smooth per-sample-interpolated self-Doppler paths and
 *      continuous 3-tap gallery speaker air cross-coupling (zero tap-switching clicks).
 *   3. Lock-free parameter handoff & single-pass block renderer (`pm_dsp_render_block24`)
 *      fusing Mode 1 ("1:FEEDBACK") and Mode 2 ("2:SOUND VCA") with zero UI/audio
 *      thread races and zero audio-rate libm calls.
 */

#define PM_NUM_MICS        4u
#define PM_INTERNAL_SR     24000u

/* 24 kHz delay lengths (10.7 ms speaker air path = 3.66 m; 10.7 ms gallery room) */
#define PM_SPK_DELAY_LEN   256u
#define PM_SPK_DELAY_MASK  (PM_SPK_DELAY_LEN - 1u)

#define PM_ROOM_LEN        256u
#define PM_ROOM_MASK       (PM_ROOM_LEN - 1u)

#define PM_AP_LEN          64u
#define PM_AP_MASK         (PM_AP_LEN - 1u)

/* Max interleaved float32 samples per 24 kHz render chunk (128 stereo 24k frames) */
#define PM_SCRATCH24       256u

typedef struct {
    float l;
    float r;
} pm_frame_t;

/* 16-byte frame-interleaved speaker quad: all 4 speakers share a single cache line */
typedef struct {
    float ch[PM_NUM_MICS];
} pm_quad_t;

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
 * Compact 7.5 KB (7,680-byte) hot acoustic workspace embedded directly inside
 * `pm_model_t` so the firmware places it in zero-wait-state internal SRAM.
 */
typedef struct {
    pm_quad_t spk_ring[PM_SPK_DELAY_LEN];   /* 256 * 16 B = 4,096 B (4.0 KB) */
    float     room_l[PM_ROOM_LEN];          /* 256 *  4 B = 1,024 B (1.0 KB) */
    float     room_r[PM_ROOM_LEN];          /* 256 *  4 B = 1,024 B (1.0 KB) */
    float     ap_l[PM_AP_LEN];              /*  64 *  4 B =   256 B          */
    float     ap_r[PM_AP_LEN];              /*  64 *  4 B =   256 B          */
    float     scratch24[PM_SCRATCH24];      /* 256 *  4 B = 1,024 B (1.0 KB) */
} pend_space_t;

typedef struct {
    float sample_rate;
    float inv_fs;

    /* Lock-free normalized performance controls [0..1] written by UI, read by audio */
    volatile float   gain_norm;
    volatile float   couple_norm;
    volatile float   spread_norm;
    volatile float   decay_norm;
    volatile uint8_t tuning_mode;
    volatile uint8_t bank_idx;
    volatile bool    release_stagger;
    volatile bool    voicing_dirty;
    volatile bool    bank_dirty;

    uint8_t phys_rem;

    /* Per-pendulum nonlinear kinematics & cached reciprocals */
    float theta[PM_NUM_MICS];
    float omega[PM_NUM_MICS];
    float cable_len[PM_NUM_MICS];
    float cable_tgt[PM_NUM_MICS];
    float inv_cable[PM_NUM_MICS];
    float inv_w0[PM_NUM_MICS];
    float px[PM_NUM_MICS];
    float pz[PM_NUM_MICS];
    float deg[PM_NUM_MICS];
    float norm_u[PM_NUM_MICS];
    float norm_v[PM_NUM_MICS];

    /* Precomputed Mode 2 constant-power stereo pan gains (updated at 375 Hz) */
    float vca_pan_l[PM_NUM_MICS];
    float vca_pan_r[PM_NUM_MICS];

    /* Continuous per-sample-ramped delays & gains (zero staircase jumps) */
    float self_del_cur[PM_NUM_MICS];
    float self_del_tgt[PM_NUM_MICS];
    float self_del_inc[PM_NUM_MICS];

    float self_gain_cur[PM_NUM_MICS];
    float self_gain_tgt[PM_NUM_MICS];
    float self_gain_inc[PM_NUM_MICS];

    float cross_gain_cur[PM_NUM_MICS];
    float cross_gain_tgt[PM_NUM_MICS];
    float cross_gain_inc[PM_NUM_MICS];

    float loop_gain_cur[PM_NUM_MICS];
    float loop_gain_tgt[PM_NUM_MICS];
    float loop_gain_inc[PM_NUM_MICS];

    /* Per-channel PA cabinet LPF + resonant ZDF Bandpass SVF (zero DC gain) */
    float lp_y[PM_NUM_MICS];
    float svf_g[PM_NUM_MICS];
    float svf_k_gain[PM_NUM_MICS];
    float svf_a1[PM_NUM_MICS];
    float svf_a2[PM_NUM_MICS];
    float svf_a3[PM_NUM_MICS];
    float svf_ic1[PM_NUM_MICS];
    float svf_ic2[PM_NUM_MICS];
    float tau_trim[PM_NUM_MICS];
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
    float dc_r;
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
bool pm_all_nearly_still(const reich_pendulum_dsp_t* d, float deg_thresh);

const char* pm_tuning_name(uint8_t tuning_mode);
const char* pm_bank_name(uint8_t bank_idx);
const char* pm_bank_slot_tag(uint8_t bank_idx, uint8_t slot_idx);

float pm_clip_guard(float x);
float pm_soft_limit(float x);

/* Single-sample helpers retained for host unit tests & inspection */
void pm_dsp_step(reich_pendulum_dsp_t* d, pm_frame_t* out_fbk);
pm_frame_t pm_render_sound_vca(reich_pendulum_dsp_t* d,
                               const pm_frame_t custom_in[PM_NUM_MICS],
                               uint8_t custom_mask);

/*
 * Fast block-fused 24 kHz renderer: processes `frames24` stereo frames into
 * interleaved `out24[0 .. 2*frames24 - 1]` in a single call per block.
 */
void pm_dsp_render_block24(reich_pendulum_dsp_t* restrict d,
                           float* restrict out24,
                           uint32_t frames24,
                           float play_target,
                           float play_step,
                           float* restrict io_play_gain,
                           float mode_target,
                           float mode_step,
                           float* restrict io_mode_mix,
                           pm_frame_t* const slot_buf[PM_NUM_MICS],
                           const volatile uint32_t slot_len[PM_NUM_MICS],
                           volatile uint32_t head_io[PM_NUM_MICS],
                           uint8_t custom_mask);
