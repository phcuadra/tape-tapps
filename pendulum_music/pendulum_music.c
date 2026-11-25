/*
 * pendulum_music.c — Steve Reich 1968 "Pendulum Music" Standalone TAPP
 *
 * Modeled after Steve Reich's August 1968 score "Pendulum Music — For
 * Microphones, Amplifiers, Speakers and Performers".
 *
 * Simulates 4 suspended microphones (M1..M4) swinging on overhead cables
 * above 4 upward-facing floor speakers in a shared gallery space with a
 * frame-interleaved 4x4 3D acoustic propagation matrix, inter-mic
 * constructive/destructive phase interference, Barkhausen/Doppler pitch glides,
 * and gradual decay into a continuous 4-voice resting feedback chord.
 *
 * Two Live Performance Modes (instant 20 ms crossfade via BTN3 press):
 *   1. 1:FEEDBACK  — Output IS the stereo gallery mix of the 4 coupled
 *                    microphone-speaker feedback loops.
 *   2. 2:SOUND VCA — The 4 coupled mic feedback envelopes env[0..3] shape the
 *                    output levels of 4 distinct stereo sounds (built-in Reich
 *                    sound banks, live Mic/Line captures per pendulum, or SD
 *                    tape takes loaded onto M1..M4).
 *
 * Controls:
 *   BTN1 press : Play / Stop (auto-pulls & releases mics if nearly still)
 *   BTN1 hold  : Volume overlay (out @ 25% default / fx send / in / mic trim)
 *   BTN2 press : Record live input into target pendulum slot M1..M4 (REC-only monitor)
 *   BTN2 hold  : Bounce stereo gallery output to free SD tape space (DMA windows)
 *   BTN3 press : Toggle Mode 1 (fbk: 1:FEEDBACK) <-> Mode 2 (vca: 2:SOUND VCA)
 *   BTN3 hold  : Menu (Pull & Release, Mode, Sound Bank, Rec Target,
 *                Load Tape Take -> Mic X, Load Takes 1-4 -> M1-M4,
 *                Release Unison/Staggered, Tuning, Input Line/Mic, Mic Trim, Re-Seed)
 *   BTN4 / 5   : Select encoder parameter (GAIN, COUPLE, SPREAD, DECAY)
 *   ENCODER    : Adjust selected parameter
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tapp_api.h"
#include "reich_pendulum_dsp.h"

TAPP_DECLARE_TYPE(AppTypeStandalone);

/* ----------------------------------------------------------------------------
 * Constants & Configuration
 * ------------------------------------------------------------------------- */
#define FRAMES_PER_POS        64u
#define WIN_NONE              0xFFFFFFFFu

/* Custom slot audio runs at PM_INTERNAL_SR (24 kHz), matching the DSP engine */
#define MIN_LOOP_FR24         (PM_INTERNAL_SR / 5u)     /* 0.2 s minimum slot loop */
#define SPLICE_FADE_FR24      480u                      /* 20 ms smoothstep loop splice */
#define REC_FADE_IN_FR24      600u                      /* 25 ms smoothstep capture fade-in */
#define MIC_START_SETTLE_FR24 2040u                     /* 85 ms post-BTN2-release settle */
#define MIC_BTN_HELD_MAX_FR24 6000u                     /* 250 ms max holdoff while BTN2 down */
#define MIC_STOP_TRIM_FR24    2640u                     /* 110 ms stop-button click tail trim */

/* 48 kHz Tape I/O constants */
#define BOUNCE_FADE_FR        1440u                     /* 30 ms bounce fade in/out */
#define MIN_BOUNCE_FR         (2u * TAPP_SAMPLE_RATE)   /* require >= 2 s free tape */
#define BOUNCE_GUARD_FR       (128u * FRAMES_PER_POS)
#define COMMIT_TRIM_POS       64u
#define LOAD_CHUNK_FR48       2048u                     /* ~42 ms per tick during tape_read */

#define UI_FRAME_MS           40u                       /* 25 fps smooth frame-paced UI */
#define UI_VOL_FRAME_MS       80u                       /* 12.5 fps while volume bar animates */
#define TRAIL_LEN             2u
#define MAX_MENU_TAKES        20u

enum {
    ST_STOP = 0,
    ST_PLAY,
    ST_REC,
    ST_LOADING,
};

enum {
    PM_MODE_FEEDBACK = 0,
    PM_MODE_SOUND_VCA = 1,
};

enum {
    ENC_GAIN = 0,
    ENC_COUPLE,
    ENC_SPREAD,
    ENC_DECAY,
    ENC_TOTAL,
};

/* ----------------------------------------------------------------------------
 * App Model (~10.4 KB in zero-wait-state internal SRAM; custom loops in PSRAM)
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Transport & cross-thread request flags */
    volatile uint8_t  state;
    volatile uint8_t  mode;             /* PM_MODE_FEEDBACK or PM_MODE_SOUND_VCA */
    volatile bool     buf_locked;
    volatile bool     rec_done_req;
    volatile bool     mon_arm_req;      /* set by audio when post-BTN2 holdoff finishes */
    volatile bool     tape_swapped;
    volatile bool     pull_req;
    volatile bool     seed_req;
    volatile bool     eng_active;

    /* Hot 7.5 KB delay/room workspace embedded directly in fast internal SRAM! */
    pend_space_t      space;

    /* 4 custom 24 kHz sound slots in PSRAM + SD read staging */
    pm_frame_t*       slots_mem;
    pm_frame_t*       slot_buf[PM_NUM_MICS];
    float*            read_stage;
    uint32_t          max_slot_frames;  /* in 24 kHz stereo frames */
    volatile uint32_t slot_len[PM_NUM_MICS];
    volatile uint32_t head_i[PM_NUM_MICS];

    /* 24 kHz -> 48 kHz firmware upsampler state */
    ups2x_f32_t       ups;

    /* Slot metadata, recording target & button-click pop suppression state */
    uint8_t           rec_slot;         /* Active target slot 0..3 (M1..M4) */
    uint8_t           bank_idx;         /* Active built-in bank 0..3 */
    volatile uint8_t  slot_custom;      /* Bitmask (bits 0..3): 1 = custom USER/TK audio */
    char              slot_tag[PM_NUM_MICS][6];
    volatile uint32_t rec_frames;       /* captured 24 kHz frames */
    volatile bool     rec_btn_down;
    volatile bool     rec_manual_stop;
    volatile uint32_t rec_holdoff_fr;
    volatile uint32_t rec_held_fr;
    float             rec_hp_xl;
    float             rec_hp_yl;
    float             rec_hp_xr;
    float             rec_hp_yr;

    /* Smooth audio crossfaders & per-sample anti-zipper volume followers */
    float             play_gain;
    float             mode_mix;         /* 0.0 = Mode 1 Feedback, 1.0 = Mode 2 VCA */
    float             vout_cur;
    float             vfx_cur;

    /* Core 4-pendulum coupled acoustic feedback DSP engine (runs at 24 kHz) */
    reich_pendulum_dsp_t dsp;

    /* Chunked SD tape-take loader (48 kHz tape -> 24 kHz slot) */
    uint32_t          load_src_frame48;
    uint32_t          load_total48;
    uint32_t          load_done48;
    uint32_t          load_written24;
    uint8_t           load_dst_slot;
    uint8_t           load_batch_idx;
    uint8_t           load_batch_cnt;
    bool              load_batch_mode;
    uint32_t          load_take_idx;

    /* Bounce-to-tape DMA streaming state (48 kHz) */
    tape_buffer_t*    wb;
    volatile bool     bouncing;
    volatile bool     bounce_stop_req;
    volatile float    bounce_gain;
    volatile uint32_t bounce_frame;
    volatile uint32_t bounce_cur_win;
    uint32_t          bounce_tok;
    uint32_t          bounce_start;
    uint32_t          bounce_limit;

    /* Live metering, frame pacing & pendulum motion trails */
    volatile float    peak_l;
    volatile float    peak_r;
    int8_t            trail_deg[PM_NUM_MICS][TRAIL_LEN];
    uint32_t          last_trail_ms;
    uint32_t          last_draw_ms;
    uint32_t          anim_until_ms;
    bool              ui_dirty;

    /* Cached header & card text so redraw() avoids per-frame snprintf/measure */
    int16_t           cached_phi_pc;
    int16_t           cached_phi_x;
    char              cached_phi_str[16];
    int16_t           cached_prm_pc[ENC_TOTAL];
    int16_t           cached_prm_x[ENC_TOTAL];
    char              cached_prm_str[ENC_TOTAL][8];

    /* 4 live performance encoder parameters */
    params_t          prm[ENC_TOTAL];
    uint8_t           enc_sel;

    /* Volume overlay, mic trim & tape timeline params */
    params_t          vol_out;
    params_t          vol_fx;
    params_t          mic_trim;         /* 0.40f default (-8 dB) when Mic */
    volatile bool     vol_ui;
    bool              mon_saved;
    bool              wet_saved;
    bool              mon_req;
    params_t          tl_pos;
    params_t          tl_start;
    params_t          tl_end;

    /* Menu & cached UI labels */
    ui_menu_t*        menu;
    uint8_t           in_src;
    uint8_t           hint_state;
    uint8_t           hint_mode;
    bool              hint_bounce;
    char              take_labels[MAX_MENU_TAKES][22];
    char              mode_label[26];
    char              bank_label[30];
    char              target_label[24];
    char              load_one_label[28];
    char              rel_label[24];
    char              tune_label[28];
    char              input_label[20];
} pm_model_t;

/* Forward declarations */
static void pm_hints(const pm_model_t* m);
static void pm_finish_capture(pm_model_t* m);
static void pm_bounce_stop(pm_model_t* m, tape_t* t);
static void pm_sync_engine_active(pm_model_t* m);
static void pm_sync_monitor(pm_model_t* m);

/* ----------------------------------------------------------------------------
 * Loop Splice & DSP Sync Helpers
 * ------------------------------------------------------------------------- */
static void apply_loop_splice(pm_frame_t* buf, uint32_t len) {
    if (!buf || len < 16u) return;
    uint32_t fade = SPLICE_FADE_FR24;
    if (fade > (len >> 3)) fade = len >> 3;
    if (fade < 4u) return;

    const float inv = 1.0f / (float)fade;
    for (uint32_t i = 0u; i < fade; ++i) {
        const float t = (float)i * inv;
        const float w = t * t * (3.0f - 2.0f * t);
        buf[i].l *= w;
        buf[i].r *= w;
        const uint32_t tail = len - 1u - i;
        buf[tail].l *= w;
        buf[tail].r *= w;
    }
}

static void sync_params_to_dsp(pm_model_t* m) {
    pm_set_gain(&m->dsp, m->prm[ENC_GAIN].val);
    pm_set_couple(&m->dsp, m->prm[ENC_COUPLE].val);
    pm_set_spread(&m->dsp, m->prm[ENC_SPREAD].val);
    pm_set_decay(&m->dsp, m->prm[ENC_DECAY].val);
}

static void pm_sync_engine_active(pm_model_t* m) {
    const bool need = (m->state == ST_PLAY || m->state == ST_REC ||
                       m->bouncing || m->play_gain > 0.001f);
    if (need != m->eng_active) {
        engine_set_active(need);
    }
}

/*
 * Event-driven monitor & WET gate: keeps live input monitoring strictly OFF
 * unless actively capturing (ST_REC) past button-release settle.
 */
static void pm_sync_monitor(pm_model_t* m) {
    m->in_src = os_audio_get_input();
    const bool want_mon = (m->state == ST_REC) && !m->rec_done_req &&
                          !m->rec_btn_down && (m->rec_holdoff_fr == 0u);
    if (!want_mon) {
        if (m->mon_req || os_audio_get_monitor()) {
            (void)os_audio_set_monitor(false);
            m->mon_req = false;
        }
    } else if (!m->mon_req) {
        (void)os_audio_set_monitor(true);
        m->mon_req = true;
    }

    const bool want_wet = want_mon && m->wet_saved && (m->in_src == 0u);
    if (os_audio_get_wet() != want_wet) {
        (void)os_audio_set_wet(want_wet);
    }
}

static void pm_tame_hot_mic_input(void) {
    if (os_audio_get_input() != 1u) return;
    mixer_t* mx = mixer_get();
    params_t* in_vol = mx ? mixer_get_input_vol(mx) : NULL;
    if (in_vol && in_vol->val > 0.35f) {
        in_vol->target = 0.35f;
        param_update_fast(in_vol, NULL);
    }
}

/* ----------------------------------------------------------------------------
 * Bounce-to-Tape Streaming (DMA windows, full 48 kHz resolution)
 * ------------------------------------------------------------------------- */
static bool bounce_free_region(tape_t* t, uint32_t* start_fr, uint32_t* limit_fr) {
    if (!t) return false;
    const uint32_t cap = tape_size(t) * FRAMES_PER_POS;
    if (cap < MIN_BOUNCE_FR) return false;

    const uint32_t n = tape_marks_count(t);
    uint32_t end = 0u;
    for (uint32_t i = 0u; i < n; ++i) {
        uint32_t s = 0u, e = 0u;
        if (tape_marks_get(t, i, &s, &e) && e > end) end = e;
    }
    const uint32_t start = end ? (end + BOUNCE_GUARD_FR) : 0u;
    if (start >= cap || (cap - start) < MIN_BOUNCE_FR) return false;

    *start_fr = start;
    *limit_fr = cap;
    return true;
}

static void pm_bounce_start(pm_model_t* m) {
    tape_t* t = tape_get();
    if (!t || !m->wb) {
        ui_notify_toast(NotifyWarning, "Tape unavailable", 1800u);
        return;
    }
    if (m->bouncing) return;

    uint32_t start = 0u, limit = 0u;
    if (!bounce_free_region(t, &start, &limit)) {
        ui_notify_toast(NotifyWarning, "No free tape space", 1800u);
        return;
    }

    tape_buffer_clear_all_buffers(m->wb);
    m->bounce_start = start;
    m->bounce_limit = limit;
    m->bounce_frame = start;
    m->bounce_cur_win = WIN_NONE;
    m->bounce_gain = 0.0f;
    m->bounce_stop_req = false;

    m->bounce_tok = (uint32_t)(uintptr_t)tape_rec_begin(
        t, (start + BOUNCE_FADE_FR) / FRAMES_PER_POS, false);

    m->tl_start.val = m->tl_start.target = (float)(start / FRAMES_PER_POS);
    m->tl_end.val = m->tl_end.target = (float)(start / FRAMES_PER_POS);
    tape_timeline_set_loop(false);

    if (m->state == ST_STOP) {
        if (pm_all_nearly_still(&m->dsp, 3.0f)) {
            m->pull_req = true;
        }
        m->state = ST_PLAY;
    }

    m->bouncing = true;
    m->ui_dirty = true;
    m->anim_until_ms = os_tick_get() + 2400u;
    pm_sync_engine_active(m);
    pm_hints(m);
    ui_notify_toast(NotifyInfo, "Bouncing Pendulum Music", 1800u);
}

static void pm_bounce_stop(pm_model_t* m, tape_t* t) {
    if (!m->bouncing) return;
    m->bouncing = false;
    m->bounce_stop_req = false;

    if (m->wb) tape_buffer_flush(m->wb);

    if (m->bounce_tok && t) {
        const uint32_t end_pos = tape_rec_commit(
            (tape_rec_t*)(uintptr_t)m->bounce_tok,
            m->bounce_frame / FRAMES_PER_POS + COMMIT_TRIM_POS);
        const uint32_t start_fr = m->bounce_start + BOUNCE_FADE_FR;
        const uint32_t end_fr = end_pos * FRAMES_PER_POS;
        m->bounce_tok = 0u;

        if (end_fr > start_fr) {
            tape_cue_add(t, start_fr, end_fr);
            m->tl_start.val = m->tl_start.target = (float)(start_fr / FRAMES_PER_POS);
            m->tl_end.val = m->tl_end.target = (float)end_pos;
            tape_timeline_set_loop(true);
        }
        ui_notify_toast(NotifySuccess, "Pendulum bounce saved", 1800u);
    }

    m->ui_dirty = true;
    m->anim_until_ms = os_tick_get() + 2400u;
    pm_sync_engine_active(m);
    pm_hints(m);
}

/* ----------------------------------------------------------------------------
 * Audio Engine (24 kHz internal DSP -> hardware ups2x_f32_add -> 48 kHz bus)
 * ------------------------------------------------------------------------- */
static inline void bounce_win_request(pm_model_t* m, uint32_t win) {
    tape_buffer_update(m->wb, win * TAPE_WINDOW_POS, true);
}

static void pm_process(void* restrict ctx, mixer_t* restrict mix) {
    pm_model_t* m = (pm_model_t*)ctx;
    float* out = mixer_get_out(mix);
    const float* in = mixer_get_record(mix);
    float* send = mixer_get_send(mix);
    const uint32_t fs = mixer_get_fs(mix);
    if (!out || fs == 0u) return;

    const uint8_t st = m->state;
    const bool locked = m->buf_locked;
    const bool bouncing = m->bouncing && (m->wb != NULL);

    if (st == ST_REC || bouncing) {
        mixer_rec_active(mix);
    }

    if (m->pull_req) {
        m->pull_req = false;
        pm_pull_and_release(&m->dsp);
    }

    if (locked) {
        memset(out, 0, (size_t)fs * sizeof(float));
        return;
    }

    const float vout_tgt = m->vol_out.val;
    const float vfx_tgt  = m->vol_fx.val;
    float pl = 0.0f;
    float pr = 0.0f;

    /* Path A: Live 48k -> 24k decimated slot capture (output is muted during ST_REC) */
    if (st == ST_REC) {
        m->vout_cur = vout_tgt;
        m->vfx_cur  = vfx_tgt;
        memset(out, 0, (size_t)fs * sizeof(float));

        const uint8_t rslot = m->rec_slot & 3u;
        pm_frame_t* rec_dst = m->slot_buf[rslot];
        if (rec_dst && in) {
            const float itrim = (m->in_src == 1u) ? m->mic_trim.val : 1.0f;
            for (uint32_t i = 0u; i + 3u < fs; i += 4u) {
                float il = 0.5f * (in[i]      + in[i + 2u]) * itrim;
                float ir = 0.5f * (in[i + 1u] + in[i + 3u]) * itrim;

                if (m->in_src == 1u) {
                    const float hpl = 0.9779f * (m->rec_hp_yl + il - m->rec_hp_xl);
                    const float hpr = 0.9779f * (m->rec_hp_yr + ir - m->rec_hp_xr);
                    m->rec_hp_xl = il;
                    m->rec_hp_yl = hpl;
                    m->rec_hp_xr = ir;
                    m->rec_hp_yr = hpr;
                    il = hpl;
                    ir = hpr;
                }

                if (m->rec_btn_down) {
                    m->rec_held_fr++;
                    if (m->rec_held_fr >= MIC_BTN_HELD_MAX_FR24) {
                        m->rec_btn_down = false;
                    }
                } else if (m->rec_holdoff_fr > 0u) {
                    m->rec_holdoff_fr--;
                    if (m->rec_holdoff_fr == 0u) {
                        m->mon_arm_req = true;
                    }
                } else {
                    const uint32_t rf = m->rec_frames;
                    if (rf < m->max_slot_frames && !m->rec_done_req) {
                        float fg = 1.0f;
                        if (rf < REC_FADE_IN_FR24) {
                            const float t = (float)rf * (1.0f / (float)REC_FADE_IN_FR24);
                            fg = t * t * (3.0f - 2.0f * t);
                        }
                        rec_dst[rf].l = pm_clip_guard(il * fg);
                        rec_dst[rf].r = pm_clip_guard(ir * fg);
                        m->rec_frames = rf + 1u;
                        if (m->rec_frames >= m->max_slot_frames) {
                            m->rec_done_req = true;
                        }
                    }
                    const float al = fabsf(il);
                    const float ar = fabsf(ir);
                    if (al > pl) pl = al;
                    if (ar > pr) pr = ar;
                }
            }
        }
        m->peak_l = (pl > m->peak_l) ? pl : (m->peak_l * 0.85f);
        m->peak_r = (pr > m->peak_r) ? pr : (m->peak_r * 0.85f);
        return;
    }

    /* Path B: 24 kHz coupled feedback + Sound VCA block engine -> 48 kHz upsample */
    const float play_target = (st == ST_PLAY) ? 1.0f : 0.0f;
    const float mode_target = (m->mode == PM_MODE_SOUND_VCA) ? 1.0f : 0.0f;

    if (m->play_gain <= 0.0001f && play_target == 0.0f && !bouncing) {
        m->play_gain = 0.0f;
        m->vout_cur  = vout_tgt;
        m->vfx_cur   = vfx_tgt;
        m->ups.prev_l = 0.0f;
        m->ups.prev_r = 0.0f;
        memset(out, 0, (size_t)fs * sizeof(float));
        m->peak_l *= 0.85f;
        m->peak_r *= 0.85f;
        return;
    }

    memset(out, 0, (size_t)fs * sizeof(float));

    const float play_step24 = 1.0f / 360.0f;    /* 15 ms play/stop ramp at 24 kHz */
    const float mode_step24 = 1.0f / 480.0f;    /* 20 ms mode crossfade at 24 kHz */
    float* const scratch24 = m->space.scratch24;

    uint32_t pos48 = 0u;
    while (pos48 + 3u < fs) {
        uint32_t rem48 = (fs - pos48) & ~3u;
        uint32_t n24 = rem48 >> 1;
        if (n24 > PM_SCRATCH24) {
            n24 = PM_SCRATCH24;
            rem48 = n24 << 1;
        }

        pm_dsp_render_block24(&m->dsp,
                              scratch24,
                              n24 >> 1,
                              play_target,
                              play_step24,
                              &m->play_gain,
                              mode_target,
                              mode_step24,
                              &m->mode_mix,
                              m->slot_buf,
                              m->slot_len,
                              m->head_i,
                              m->slot_custom);

        /* Upsample unity-level 24 kHz stereo render into 48 kHz `out` bus */
        ups2x_f32_add(&m->ups, scratch24, out + pos48, n24);
        pos48 += rem48;
    }

    /* Optional Bounce-to-Tape (unity pre-vout level) + per-sample smoothed headphone trim */
    if (bouncing) {
        float* bbuf = tape_buffer_get_ptr(m->wb);
        const float bounce_target = !m->bounce_stop_req ? 1.0f : 0.0f;
        const float bounce_step = 1.0f / (float)BOUNCE_FADE_FR;
        float bg = m->bounce_gain;

        for (uint32_t i = 0u; i + 1u < fs; i += 2u) {
            const float ul = out[i];
            const float ur = out[i + 1u];

            const uint32_t bf = m->bounce_frame;
            const uint32_t win = bf / TAPE_WINDOW_FR;
            if (win != m->bounce_cur_win) {
                if (m->bounce_cur_win == WIN_NONE || win != m->bounce_cur_win + 1u) {
                    bounce_win_request(m, win);
                }
                bounce_win_request(m, win + 1u);
                m->bounce_cur_win = win;
                bbuf = tape_buffer_get_ptr(m->wb);
            }

            if (bg < bounce_target) {
                bg += bounce_step;
                if (bg > bounce_target) bg = bounce_target;
            } else if (bg > bounce_target) {
                bg -= bounce_step;
                if (bg < bounce_target) bg = bounce_target;
            }

            if (bbuf && (bg > 0.0f || !m->bounce_stop_req) && bf < m->bounce_limit) {
                const uint32_t off = (bf & (TAPE_WINDOW_FR - 1u)) * 2u;
                bbuf[off]      = clampf(ul * bg, -1.0f, 1.0f);
                bbuf[off + 1u] = clampf(ur * bg, -1.0f, 1.0f);
                m->bounce_frame = bf + 1u;
            }
        }
        m->bounce_gain = bg;
    }

    float vo = m->vout_cur;
    float vf = m->vfx_cur;
    const float vol_slew = 0.0045f; /* ~4.6 ms anti-zipper 1-pole at 48 kHz */

    if (send && (vfx_tgt > 0.0001f || vf > 0.0001f)) {
        for (uint32_t i = 0u; i + 1u < fs; i += 2u) {
            vo += vol_slew * (vout_tgt - vo);
            vf += vol_slew * (vfx_tgt  - vf);
            const float ul = out[i];
            const float ur = out[i + 1u];
            const float al = fabsf(ul);
            const float ar = fabsf(ur);
            if (al > pl) pl = al;
            if (ar > pr) pr = ar;
            const float out_l = ul * vo;
            const float out_r = ur * vo;
            out[i]       = out_l;
            out[i + 1u]  = out_r;
            send[i]      += out_l * vf;
            send[i + 1u] += out_r * vf;
        }
    } else {
        vf = vfx_tgt;
        for (uint32_t i = 0u; i + 1u < fs; i += 2u) {
            vo += vol_slew * (vout_tgt - vo);
            const float ul = out[i];
            const float ur = out[i + 1u];
            const float al = fabsf(ul);
            const float ar = fabsf(ur);
            if (al > pl) pl = al;
            if (ar > pr) pr = ar;
            out[i]      = ul * vo;
            out[i + 1u] = ur * vo;
        }
    }

    if (fabsf(vout_tgt - vo) < 1.0e-5f) vo = vout_tgt;
    if (fabsf(vfx_tgt  - vf) < 1.0e-5f) vf = vfx_tgt;
    m->vout_cur = vo;
    m->vfx_cur  = vf;

    m->peak_l = (pl > m->peak_l) ? pl : (m->peak_l * 0.85f);
    m->peak_r = (pr > m->peak_r) ? pr : (m->peak_r * 0.85f);
}

static void pm_eng_active(void* ctx, bool state) {
    ((pm_model_t*)ctx)->eng_active = state;
}

static uint_fast8_t pm_eng_is_active(void* ctx) {
    const pm_model_t* m = (const pm_model_t*)ctx;
    return (m->state == ST_PLAY || m->state == ST_REC ||
            m->bouncing || m->play_gain > 0.001f) ? 1u : 0u;
}

static void pm_tape_changed(void* ctx) {
    ((pm_model_t*)ctx)->tape_swapped = true;
}

static const engine_callbacks_t pm_engine_cb = {
    .process = pm_process,
    .tape_changed = pm_tape_changed,
    .active = pm_eng_active,
    .is_active = pm_eng_is_active,
};

/* ----------------------------------------------------------------------------
 * RGB LEDs (pure integer math on OS LED task, 25 Hz)
 * ------------------------------------------------------------------------- */
static void pm_led_step(const os_led_t* led, const void* ctx) {
    const uint8_t step = os_led_get_step(led);
    const pm_model_t* m = (const pm_model_t*)ctx;
    if (!m) return;

    if (m->state == ST_REC) {
        const int tri = (step & 32u) ? (63 - (int)(step & 63u)) : (int)(step & 63u);
        for (uint8_t i = 0u; i < 4u; ++i) {
            const bool is_tgt = (i == (m->rec_slot & 3u));
            const uint8_t r = is_tgt ? (uint8_t)(48 + tri * 2) : (uint8_t)(8 + (tri >> 1));
            hal_led_set_rgb(i, r, 2u, 2u);
        }
        return;
    }

    const bool playing = (m->state == ST_PLAY);
    for (uint8_t i = 0u; i < 4u; ++i) {
        if (m->bouncing && i == 1u) {
            const uint8_t pulse = ((step & 16u) == 0u) ? 72u : 18u;
            hal_led_set_rgb(1u, pulse, 5u, 2u);
            continue;
        }

        uint32_t br_q8 = 20u; /* ~8% idle glow */
        if (playing) {
            int32_t e_q8 = (int32_t)(m->dsp.env[i] * 240.0f);
            if (e_q8 < 0) e_q8 = 0;
            if (e_q8 > 240) e_q8 = 240;
            br_q8 = 15u + (uint32_t)e_q8;
        } else if (m->mode == PM_MODE_SOUND_VCA && i == (m->rec_slot & 3u)) {
            br_q8 = 72u;
        }

        if (m->mode == PM_MODE_FEEDBACK) {
            hal_led_set_rgb(i,
                            (uint8_t)((72u * br_q8) >> 8),
                            (uint8_t)((44u * br_q8) >> 8),
                            (uint8_t)((10u * br_q8) >> 8));
        } else {
            hal_led_set_rgb(i,
                            (uint8_t)((24u * br_q8) >> 8),
                            (uint8_t)((58u * br_q8) >> 8),
                            (uint8_t)((68u * br_q8) >> 8));
        }
    }
}

static led_pattern_t pm_led_pattern = {
    .callback = pm_led_step,
    .steps = 255u,
    .rate = 20u,
    .loop = true,
};

/* ----------------------------------------------------------------------------
 * Bank Selection, Live Slot Capture & Chunked SD Tape Loader
 * ------------------------------------------------------------------------- */
static void pm_do_seed_bank(pm_model_t* m, bool show_toast) {
    pm_set_bank(&m->dsp, m->bank_idx);
    m->slot_custom = 0u;
    for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
        strncpy(m->slot_tag[s], pm_bank_slot_tag(m->bank_idx, (uint8_t)s), 5u);
        m->slot_tag[s][5] = '\0';
    }
    m->ui_dirty = true;

    if (show_toast) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Bank: %s", pm_bank_name(m->bank_idx));
        m->anim_until_ms = os_tick_get() + 2200u;
        ui_notify_toast(NotifySuccess, msg, 1500u);
    }
}

static void pm_finish_capture(pm_model_t* m) {
    m->buf_locked = true;
    m->rec_done_req = false;
    m->rec_btn_down = false;
    m->rec_holdoff_fr = 0u;
    uint32_t captured = m->rec_frames;
    const uint8_t slot = m->rec_slot & 3u;
    if (m->in_src == 1u && m->rec_manual_stop) {
        if (captured > MIN_LOOP_FR24 + MIC_STOP_TRIM_FR24) {
            captured -= MIC_STOP_TRIM_FR24;
        } else if (captured > MIN_LOOP_FR24) {
            captured = MIN_LOOP_FR24;
        }
    }
    m->rec_manual_stop = false;

    if (captured < MIN_LOOP_FR24) {
        m->state = ST_STOP;
        m->buf_locked = false;
        m->ui_dirty = true;
        m->anim_until_ms = os_tick_get() + 2400u;
        pm_sync_monitor(m);
        pm_sync_engine_active(m);
        pm_hints(m);
        ui_notify_toast(NotifyWarning, "Capture too short", 1800u);
        return;
    }

    apply_loop_splice(m->slot_buf[slot], captured);
    m->slot_len[slot] = captured;
    m->head_i[slot] = 0u;
    m->slot_custom |= (uint8_t)(1u << slot);
    strcpy(m->slot_tag[slot], "USER");

    m->mode = PM_MODE_SOUND_VCA;
    m->rec_slot = (uint8_t)((slot + 1u) & 3u);
    if (pm_all_nearly_still(&m->dsp, 3.0f)) {
        m->pull_req = true;
    }

    m->state = ST_PLAY;
    m->buf_locked = false;
    m->ui_dirty = true;
    m->anim_until_ms = os_tick_get() + 2400u;
    pm_sync_monitor(m);
    pm_sync_engine_active(m);
    pm_hints(m);

    char msg[32];
    snprintf(msg, sizeof(msg), "Saved Mic %u -> Sound VCA", (unsigned)(slot + 1u));
    ui_notify_toast(NotifySuccess, msg, 1800u);
}

static bool pm_prep_single_take(pm_model_t* m, tape_t* t, uint32_t take_idx, uint8_t dst_slot) {
    uint32_t s = 0u, e = 0u;
    if (!tape_marks_get(t, take_idx, &s, &e) || e <= s) return false;
    uint32_t len48 = (e - s) & ~1u;
    const uint32_t max48 = m->max_slot_frames * 2u;
    if (len48 > max48) len48 = max48;
    if ((len48 >> 1) < MIN_LOOP_FR24) return false;

    m->load_src_frame48 = s;
    m->load_total48 = len48;
    m->load_done48 = 0u;
    m->load_written24 = 0u;
    m->load_dst_slot = dst_slot & 3u;
    m->load_take_idx = take_idx;
    return true;
}

static void pm_start_take_load(pm_model_t* m, uint32_t take_idx, bool batch_first_four) {
    tape_t* t = tape_get();
    if (!t || !m->slots_mem || !m->read_stage) {
        m->anim_until_ms = os_tick_get() + 2400u;
        ui_notify_toast(NotifyWarning, "Tape unavailable", 1800u);
        return;
    }
    const uint32_t n = tape_marks_count(t);
    if (n == 0u) {
        m->anim_until_ms = os_tick_get() + 2400u;
        ui_notify_toast(NotifyWarning, "No tape takes", 1800u);
        return;
    }

    m->buf_locked = true;
    if (batch_first_four) {
        const uint8_t cnt = (n < 4u) ? (uint8_t)n : 4u;
        m->load_batch_mode = true;
        m->load_batch_idx = 0u;
        m->load_batch_cnt = cnt;
        if (!pm_prep_single_take(m, t, 0u, 0u)) {
            m->buf_locked = false;
            m->anim_until_ms = os_tick_get() + 2400u;
            ui_notify_toast(NotifyWarning, "Take 1 too short", 1800u);
            return;
        }
    } else {
        m->load_batch_mode = false;
        if (!pm_prep_single_take(m, t, take_idx, m->rec_slot)) {
            m->buf_locked = false;
            m->anim_until_ms = os_tick_get() + 2400u;
            ui_notify_toast(NotifyWarning, "Take too short", 1800u);
            return;
        }
    }

    m->state = ST_LOADING;
    m->ui_dirty = true;
    pm_sync_monitor(m);
    pm_sync_engine_active(m);
    pm_hints(m);
}

static void pm_step_take_load(pm_model_t* m) {
    tape_t* t = tape_get();
    const uint8_t dst = m->load_dst_slot & 3u;
    if (!t || !m->read_stage || !m->slot_buf[dst]) {
        m->state = ST_STOP;
        m->buf_locked = false;
        m->ui_dirty = true;
        return;
    }

    const uint32_t rem48 = m->load_total48 - m->load_done48;
    uint32_t chunk48 = (rem48 > LOAD_CHUNK_FR48) ? LOAD_CHUNK_FR48 : rem48;
    chunk48 &= ~1u;
    if (chunk48 == 0u) chunk48 = rem48;

    const uint32_t got48 = tape_read(t, m->load_src_frame48 + m->load_done48, m->read_stage, chunk48);
    if (got48 < 2u) {
        m->state = ST_STOP;
        m->buf_locked = false;
        m->ui_dirty = true;
        m->anim_until_ms = os_tick_get() + 2400u;
        ui_notify_toast(NotifyError, "Tape read failed", 1800u);
        return;
    }

    pm_frame_t* sbuf = m->slot_buf[dst];
    const uint32_t pairs = got48 >> 1;
    uint32_t wr24 = m->load_written24;
    for (uint32_t p = 0u; p < pairs && wr24 < m->max_slot_frames; ++p) {
        const uint32_t b = p * 4u;
        const float l = 0.5f * (m->read_stage[b]      + m->read_stage[b + 2u]);
        const float r = 0.5f * (m->read_stage[b + 1u] + m->read_stage[b + 3u]);
        sbuf[wr24].l = pm_clip_guard(l);
        sbuf[wr24].r = pm_clip_guard(r);
        wr24++;
    }
    m->load_written24 = wr24;
    m->load_done48 += got48;

    if (m->load_done48 >= m->load_total48 || wr24 >= m->max_slot_frames) {
        apply_loop_splice(sbuf, wr24);
        m->slot_len[dst] = wr24;
        m->head_i[dst] = 0u;
        m->slot_custom |= (uint8_t)(1u << dst);
        snprintf(m->slot_tag[dst], sizeof(m->slot_tag[dst]), "TK%u",
                 (unsigned)((m->load_take_idx + 1u) % 100u));

        if (m->load_batch_mode) {
            m->load_batch_idx++;
            while (m->load_batch_idx < m->load_batch_cnt) {
                if (pm_prep_single_take(m, t, m->load_batch_idx, m->load_batch_idx)) {
                    return;
                }
                m->load_batch_idx++;
            }
            m->mode = PM_MODE_SOUND_VCA;
            if (pm_all_nearly_still(&m->dsp, 3.0f)) {
                m->pull_req = true;
            }
            m->state = ST_PLAY;
            m->buf_locked = false;
            m->ui_dirty = true;
            m->anim_until_ms = os_tick_get() + 2400u;
            pm_sync_monitor(m);
            pm_sync_engine_active(m);
            pm_hints(m);
            ui_notify_toast(NotifySuccess, "Loaded takes -> M1-M4", 1800u);
        } else {
            m->mode = PM_MODE_SOUND_VCA;
            m->rec_slot = (uint8_t)((dst + 1u) & 3u);
            if (pm_all_nearly_still(&m->dsp, 3.0f)) {
                m->pull_req = true;
            }
            m->state = ST_PLAY;
            m->buf_locked = false;
            m->ui_dirty = true;
            m->anim_until_ms = os_tick_get() + 2400u;
            pm_sync_monitor(m);
            pm_sync_engine_active(m);
            pm_hints(m);
            char msg[32];
            snprintf(msg, sizeof(msg), "Loaded -> Mic %u (VCA)", (unsigned)(dst + 1u));
            ui_notify_toast(NotifySuccess, msg, 1800u);
        }
    }
}

/* ----------------------------------------------------------------------------
 * Menu System (BTN3 Hold — "PENDULUM MUSIC")
 * ------------------------------------------------------------------------- */
static void menu_act_pull_release(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    m->pull_req = true;
    m->state = ST_PLAY;
    m->ui_dirty = true;
    m->anim_until_ms = os_tick_get() + 2200u;
    pm_sync_monitor(m);
    pm_sync_engine_active(m);
    pm_hints(m);
    ui_menu_hide(menu);
    ui_notify_toast(NotifyInfo, "Released 4 microphones", 1500u);
}

static void menu_act_toggle_mode(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    m->mode = (m->mode == PM_MODE_FEEDBACK) ? PM_MODE_SOUND_VCA : PM_MODE_FEEDBACK;
    m->ui_dirty = true;
    pm_hints(m);
    ui_menu_rebuild(menu);
}

static void menu_act_cycle_bank(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    m->bank_idx = (uint8_t)((m->bank_idx + 1u) % PM_BANK_COUNT);
    m->mode = PM_MODE_SOUND_VCA;
    m->seed_req = true;
    m->ui_dirty = true;
    ui_menu_hide(menu);
}

static void menu_act_cycle_target(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    m->rec_slot = (uint8_t)((m->rec_slot + 1u) & 3u);
    m->ui_dirty = true;
    ui_menu_rebuild(menu);
}

static void menu_act_load_take_one(ui_menu_t* menu, ui_menu_node_t* entry) {
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    const uint16_t idx = UI_PICK_IDX(entry);
    ui_menu_hide(menu);
    pm_start_take_load(m, (uint32_t)idx, false);
}

static void menu_takes_builder(ui_menu_t* menu) {
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    tape_t* t = tape_get();
    const uint32_t n = t ? tape_marks_count(t) : 0u;

    if (n == 0u) {
        ui_menu_add(menu, UI_ACTION("(no tape takes)", NULL));
        return;
    }

    const uint32_t count = (n > MAX_MENU_TAKES) ? MAX_MENU_TAKES : n;
    const uint32_t max48 = m->max_slot_frames * 2u;
    for (uint32_t i = 0u; i < count; ++i) {
        uint32_t s = 0u, e = 0u;
        uint32_t dur_ds = 0u;
        if (tape_marks_get(t, i, &s, &e) && e > s) {
            uint32_t frames48 = e - s;
            if (frames48 > max48) frames48 = max48;
            dur_ds = frames48 / 4800u;
        }
        snprintf(m->take_labels[i], sizeof(m->take_labels[i]), "Take %u (%u.%us)",
                 (unsigned)(i + 1u), (unsigned)(dur_ds / 10u), (unsigned)(dur_ds % 10u));
        ui_menu_add(menu, UI_PICK(m->take_labels[i], menu_act_load_take_one, i));
    }
}

static void menu_act_load_four(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    ui_menu_hide(menu);
    pm_start_take_load(m, 0u, true);
}

static void menu_act_toggle_release(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    pm_set_release_stagger(&m->dsp, !m->dsp.release_stagger);
    m->ui_dirty = true;
    ui_menu_rebuild(menu);
}

static void menu_act_cycle_tuning(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    pm_set_tuning(&m->dsp, (uint8_t)((m->dsp.tuning_mode + 1u) % PM_TUNING_COUNT));
    m->ui_dirty = true;
    ui_menu_rebuild(menu);
}

static void menu_act_input(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_switch_input();
    m->mon_req = false;
    pm_tame_hot_mic_input();
    pm_sync_monitor(m);
    m->ui_dirty = true;
    ui_menu_rebuild(menu);
}

static void menu_act_reseed(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    m->seed_req = true;
    m->ui_dirty = true;
    ui_menu_hide(menu);
}

static void menu_root_builder(ui_menu_t* menu) {
    pm_model_t* m = (pm_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_get_input();

    snprintf(m->mode_label, sizeof(m->mode_label), "Mode: %s",
             (m->mode == PM_MODE_SOUND_VCA) ? "2 Sound VCA" : "1 Feedback");
    snprintf(m->bank_label, sizeof(m->bank_label), "Bank: %s",
             pm_bank_name(m->bank_idx));
    snprintf(m->target_label, sizeof(m->target_label), "Rec Target: Mic %u",
             (unsigned)((m->rec_slot & 3u) + 1u));
    snprintf(m->load_one_label, sizeof(m->load_one_label), "Load Take -> Mic %u",
             (unsigned)((m->rec_slot & 3u) + 1u));
    snprintf(m->rel_label, sizeof(m->rel_label), "Release: %s",
             m->dsp.release_stagger ? "Staggered" : "Unison");
    snprintf(m->tune_label, sizeof(m->tune_label), "Tuning: %s",
             pm_tuning_name(m->dsp.tuning_mode));
    snprintf(m->input_label, sizeof(m->input_label), "Input: %s",
             (m->in_src == 1u) ? "Mic" : "Line");

    ui_menu_add(menu, UI_ACTION("Pull & Release Mics", menu_act_pull_release));
    ui_menu_add(menu, UI_ACTION(m->mode_label, menu_act_toggle_mode));
    ui_menu_add(menu, UI_ACTION(m->bank_label, menu_act_cycle_bank));
    ui_menu_add(menu, UI_ACTION(m->target_label, menu_act_cycle_target));
    ui_menu_add(menu, UI_ENTRY(m->load_one_label, menu_takes_builder));
    ui_menu_add(menu, UI_ACTION("Load Takes 1-4 -> M1-M4", menu_act_load_four));
    ui_menu_add(menu, UI_ACTION(m->rel_label, menu_act_toggle_release));
    ui_menu_add(menu, UI_ACTION(m->tune_label, menu_act_cycle_tuning));
    ui_menu_add(menu, UI_ACTION(m->input_label, menu_act_input));
    ui_menu_add(menu, UI_PARAM("Mic Trim", MenuNodeParam, &m->mic_trim));
    ui_menu_add(menu, UI_ACTION("Re-Seed Default Sounds", menu_act_reseed));
}

/* ----------------------------------------------------------------------------
 * Hint Band & Volume Overlay
 * ------------------------------------------------------------------------- */
static void pm_hints(const pm_model_t* m) {
    const bool playing = (m->state == ST_PLAY);
    const bool rec = (m->state == ST_REC);
    const bool vca_mode = (m->mode == PM_MODE_SOUND_VCA);
    const tapp_hint_cell_t hints[HintsActionsTotal] = {
        [HintPress1] = {playing ? "stop" : "play",
                        gfx_icon(playing ? GfxIconPause : GfxIconPlay), NULL,
                        playing ? HINT_ACTIVE : 0u},
        [HintHold1]  = {"vol", gfx_icon(GfxIconVolume)},
        [HintPress2] = {rec ? "done" : "rec", gfx_icon(GfxIconRec), NULL,
                        rec ? HINT_ACTIVE : 0u},
        [HintHold2]  = {"bounce", gfx_icon(GfxIconSos), NULL,
                        m->bouncing ? HINT_ACTIVE : 0u},
        [HintPress3] = {vca_mode ? "vca" : "fbk",
                        gfx_icon(vca_mode ? GfxIconFx : GfxIconMic), NULL,
                        vca_mode ? HINT_ACTIVE : 0u},
        [HintHold3]  = {"menu", gfx_icon(GfxIconMenu)},
        [HintPress4] = {NULL, gfx_icon(GfxIconPlayRev)},
        [HintPress5] = {NULL, gfx_icon(GfxIconPlay)},
    };
    ui_hints_set(hints);
}

static void pm_vol_show(pm_model_t* m) {
    params_t* in_vol = mixer_get_input_vol(mixer_get());
    ui_hints_set_volume(1u, 0u, NULL, NULL, 0u);
    ui_hints_set_volume(1u, 1u, "out", &m->vol_out, HintVolOutput);
    ui_hints_set_volume(2u, 0u, "fx send", &m->vol_fx, HintVolFx);
    if (in_vol) ui_hints_set_volume(3u, 0u, "in", in_vol, HintVolInput);
    else        ui_hints_set_volume(3u, 0u, NULL, NULL, 0u);
    ui_hints_set_volume(3u, 1u, "mic trim", &m->mic_trim, HintVolInput);
    ui_hints_set_volume(4u, 0u, "<", NULL, 0u);
    ui_hints_set_volume(5u, 0u, ">", NULL, 0u);
    ui_hints_volume_select(1u, 1u);
}

static void pm_vol_hide(pm_model_t* m) {
    m->vol_ui = false;
    m->ui_dirty = true;
    pm_hints(m);
}

/* ----------------------------------------------------------------------------
 * Drawing (400x240 1-bit OLED, fast cached labels & low-cost vector primitives)
 * ------------------------------------------------------------------------- */
#define GALLERY_X     8
#define GALLERY_Y     71
#define GALLERY_W     384
#define GALLERY_H     88
#define TRUSS_Y       78
#define PIV_Y         79
#define FLOOR_Y       145
#define BAY_W         95

#define CARD_Y        163
#define CARD_H        40
#define CARD_W        93
#define CARD_GAP      4

static const char* const kBayNames[PM_NUM_MICS] = { "M1", "M2", "M3", "M4" };
static const char* const kRecBadges[PM_NUM_MICS] = { "REC M1", "REC M2", "REC M3", "REC M4" };

static void bay_pendulum_pt(int piv_x, int r, float deg, int* out_x, int* out_y) {
    const int32_t q16 = (int32_t)(deg * (65536.0f / 360.0f));
    const uint16_t ang = (uint16_t)(q16 & 0xFFFFu);
    const int sx = (int)lutsin_q15(ang);
    const int cy = (int)lutcos_q15(ang);
    *out_x = piv_x + ((sx * r + (sx >= 0 ? 16384 : -16384)) >> 15);
    *out_y = PIV_Y + ((cy * r + (cy >= 0 ? 16384 : -16384)) >> 15);
}

static void draw_header(gfx_t* gfx, pm_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_set_font(gfx, DepartureMono_Regular_10);

    gfx_draw_str(gfx, 8, 63, "PENDULUM MUSIC");
    int tx = 104;

    /* Inverted Mode Pill Badge: 1:FEEDBACK (68 px) or 2:SOUND VCA (74 px) */
    const bool is_vca = (m->mode == PM_MODE_SOUND_VCA);
    const char* mode_s = is_vca ? "2:SOUND VCA" : "1:FEEDBACK";
    const int mw = is_vca ? 74 : 68;
    gfx_draw_rect_fill_r(gfx, tx, 52, mw, 15, 2);
    gfx_set_color(gfx, GFX_COLOR_CLEAR);
    gfx_draw_str(gfx, tx + 4, 63, mode_s);
    gfx_set_color(gfx, GFX_COLOR_SET);
    tx += mw + 5;

    /* State Badge */
    const char* st_s = "STOPPED";
    int sw = 50;
    bool inv_st = false;
    if (m->state == ST_REC) {
        st_s = kRecBadges[m->rec_slot & 3u];
        sw = 44;
        inv_st = (((os_tick_get() / 280u) & 1u) == 0u);
    } else if (m->state == ST_LOADING) {
        st_s = "LOADING";
        sw = 50;
        inv_st = true;
    } else if (m->state == ST_PLAY) {
        if (pm_all_nearly_still(&m->dsp, 2.5f)) {
            st_s = "REST CHORD";
            sw = 68;
            inv_st = true;
        } else {
            st_s = "SWINGING";
            sw = 56;
        }
    }

    if (inv_st) {
        gfx_draw_rect_fill_r(gfx, tx, 52, sw, 15, 2);
        gfx_set_color(gfx, GFX_COLOR_CLEAR);
        gfx_draw_str(gfx, tx + 4, 63, st_s);
        gfx_set_color(gfx, GFX_COLOR_SET);
    } else {
        gfx_draw_rect_r(gfx, tx, 52, sw, 15, 2);
        gfx_draw_str(gfx, tx + 4, 63, st_s);
    }

    /* Right-aligned bounce timer or cached live inter-mic phase coherence */
    if (m->bouncing) {
        char rtxt[16];
        const uint32_t bsec = (m->bounce_frame - m->bounce_start) / TAPP_SAMPLE_RATE;
        snprintf(rtxt, sizeof(rtxt), "R%02u:%02u",
                 (unsigned)(bsec / 60u), (unsigned)(bsec % 60u));
        gfx_draw_str(gfx, 354, 63, rtxt);
    } else {
        const int16_t sync_pc = (int16_t)(m->dsp.phase_sync * 100.0f +
                                          (m->dsp.phase_sync >= 0.0f ? 0.5f : -0.5f));
        if (sync_pc != m->cached_phi_pc) {
            m->cached_phi_pc = sync_pc;
            snprintf(m->cached_phi_str, sizeof(m->cached_phi_str), "PHI %+d%%", (int)sync_pc);
            m->cached_phi_x = (int16_t)(392 - (int)gfx_get_str_width(gfx, m->cached_phi_str));
        }
        gfx_draw_str(gfx, m->cached_phi_x, 63, m->cached_phi_str);
    }
}

static void draw_gallery_stage(gfx_t* gfx, const pm_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_rect_r(gfx, GALLERY_X, GALLERY_Y, GALLERY_W, GALLERY_H, 3);

    /* Loading overlay */
    if (m->state == ST_LOADING) {
        gfx_set_font(gfx, gfx_nunito_semibold_14);
        const char* msg = m->load_batch_mode ? "LOADING TAKES 1-4 -> M1-M4..."
                                             : "LOADING TAPE TAKE...";
        const int mw = (int)gfx_get_str_width(gfx, msg);
        gfx_draw_str(gfx, GALLERY_X + (GALLERY_W - mw) / 2, GALLERY_Y + 34, msg);

        const float pct = (m->load_total48 > 0u)
                              ? ((float)m->load_done48 / (float)m->load_total48)
                              : 0.0f;
        gfx_draw_rect_r(gfx, GALLERY_X + 36, GALLERY_Y + 48, GALLERY_W - 72, 14, 3);
        const int fw = (int)(clampf(pct, 0.0f, 1.0f) * (float)(GALLERY_W - 76) + 0.5f);
        if (fw > 0) gfx_draw_rect_fill_r(gfx, GALLERY_X + 38, GALLERY_Y + 50, fw, 10, 2);
        return;
    }

    /* Live recording overlay into target slot M1..M4 */
    if (m->state == ST_REC) {
        const float pct = (m->max_slot_frames > 0u)
                              ? ((float)m->rec_frames / (float)m->max_slot_frames)
                              : 0.0f;
        const int fill_w = (int)(clampf(pct, 0.0f, 1.0f) * (float)(GALLERY_W - 8));
        if (fill_w > 0) {
            gfx_fill_rect_dithered(gfx, GALLERY_X + 4, GALLERY_Y + 4, fill_w, GALLERY_H - 8, 2);
        }

        const float pk = (m->peak_l > m->peak_r) ? m->peak_l : m->peak_r;
        const int mh = (int)(clampf(pk, 0.0f, 1.0f) * 26.0f);
        const int mid_y = GALLERY_Y + 40;
        gfx_draw_hline_dithered(gfx, GALLERY_X + 12, mid_y, GALLERY_W - 24, 2);
        if (mh > 0 && fill_w > 4) {
            gfx_draw_rect_fill(gfx, GALLERY_X + 4 + fill_w - 3, mid_y - mh, 3, mh * 2 + 1);
        }

        const uint32_t ds = m->rec_frames / 2400u;
        const uint32_t max_ds = m->max_slot_frames / 2400u;
        gfx_set_font(gfx, DepartureMono_Regular_10);
        gfx_draw_strf(gfx, GALLERY_X + 12, GALLERY_Y + GALLERY_H - 10,
                      "RECORDING MIC %u SOUND: %u.%us / %u.%us  (BTN2 = DONE)",
                      (unsigned)((m->rec_slot & 3u) + 1u),
                      (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                      (unsigned)(max_ds / 10u), (unsigned)(max_ds % 10u));
        return;
    }

    /* Overhead gallery suspension beam & stage floor */
    gfx_draw_hline_dithered(gfx, GALLERY_X + 5, TRUSS_Y, GALLERY_W - 10, 3);
    gfx_draw_hline_dithered(gfx, GALLERY_X + 5, FLOOR_Y + 3, GALLERY_W - 10, 1);

    int mic_x[PM_NUM_MICS];
    int mic_y[PM_NUM_MICS];
    int piv_xs[PM_NUM_MICS];

    gfx_set_font(gfx, DepartureMono_Regular_10);

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        const int bx = 10 + (int)i * BAY_W;
        const int px_piv = bx + 46;
        piv_xs[i] = px_piv;

        if (i > 0u) {
            gfx_draw_vline_dithered(gfx, bx - 1, GALLERY_Y + 4, GALLERY_H - 8, 1);
        }

        gfx_draw_rect_fill(gfx, px_piv - 2, TRUSS_Y - 2, 5, 3);

        const char* m_lbl = kBayNames[i];
        const bool is_tgt = (i == (m->rec_slot & 3u));
        if (is_tgt && m->mode == PM_MODE_SOUND_VCA) {
            gfx_draw_rect_fill_r(gfx, bx + 3, GALLERY_Y + 5, 18, 12, 2);
            gfx_set_color(gfx, GFX_COLOR_CLEAR);
            gfx_draw_str(gfx, bx + 5, GALLERY_Y + 15, m_lbl);
            gfx_set_color(gfx, GFX_COLOR_SET);
        } else {
            gfx_draw_str(gfx, bx + 4, GALLERY_Y + 15, m_lbl);
        }

        const float l_norm = clampf((m->dsp.cable_len[i] - 1.85f) * (1.0f / 0.85f), 0.0f, 1.0f);
        const int r_vis = 37 + (int)(l_norm * 10.0f + 0.5f);

        /* Fast 2-pip motion trail */
        for (uint32_t t = 0u; t < TRAIL_LEN; ++t) {
            const float tdeg = (float)m->trail_deg[i][t];
            if (fabsf(tdeg - m->dsp.deg[i]) < 2.5f) continue;
            int gx = 0, gy = 0;
            bay_pendulum_pt(px_piv, r_vis, tdeg, &gx, &gy);
            gfx_draw_rect_fill(gfx, gx, gy, (int)(t + 1u), (int)(t + 1u));
        }

        /* Live swinging microphone cable + capsule */
        int mx = 0, my = 0;
        bay_pendulum_pt(px_piv, r_vis, m->dsp.deg[i], &mx, &my);
        mic_x[i] = mx;
        mic_y[i] = my;
        gfx_draw_line(gfx, px_piv, PIV_Y, mx, my);
        gfx_draw_rect_fill(gfx, mx - 2, my - 1, 5, 4);

        const float env = clampf(m->dsp.env[i], 0.0f, 1.0f);
        if (env > 0.25f) {
            gfx_draw_rect(gfx, mx - 4, my - 3, 9, 8);
        }

        /* Upward-facing floor speaker cabinet & flared cone */
        gfx_draw_rect_fill(gfx, px_piv - 9, FLOOR_Y, 19, 4);
        gfx_draw_line(gfx, px_piv - 9, FLOOR_Y, px_piv - 13, FLOOR_Y - 4);
        gfx_draw_line(gfx, px_piv + 9, FLOOR_Y, px_piv + 13, FLOOR_Y - 4);
        gfx_draw_hline(gfx, px_piv - 13, FLOOR_Y - 4, 27);

        /* Fast horizontal acoustic wavefront bars rising from speaker cone */
        const int waves = (int)(env * 3.6f);
        for (int w = 1; w <= waves; ++w) {
            const int wy = FLOOR_Y - 4 - w * 4;
            const int half_w = 6 + w * 3;
            gfx_draw_hline(gfx, px_piv - half_w, wy, half_w * 2 + 1);
        }

        /* Right-edge per-bay live feedback / VCA vertical bar meter */
        const int vx = bx + BAY_W - 9;
        const int vy = GALLERY_Y + 8;
        const int vh = 56;
        gfx_draw_rect(gfx, vx, vy, 5, vh);
        const int fill_h = (int)(env * (float)(vh - 2) + 0.5f);
        if (fill_h > 0) {
            gfx_draw_rect_fill(gfx, vx + 1, vy + vh - 1 - fill_h, 3, fill_h);
        }

        const char* tag = (m->mode == PM_MODE_SOUND_VCA) ? m->slot_tag[i] : "FBK";
        gfx_draw_str(gfx, bx + 4, GALLERY_Y + GALLERY_H - 4, tag);
    }

    /* Inter-bay phase-coupling link indicators */
    if (m->prm[ENC_COUPLE].val > 0.04f && m->state == ST_PLAY) {
        for (uint32_t i = 0u; i < 3u; ++i) {
            const float coh = m->dsp.pair_coh[i];
            const float act = m->dsp.env[i] + m->dsp.env[i + 1u];
            if (act < 0.08f) continue;

            if (coh > 0.14f) {
                const int mid_x = (mic_x[i] + mic_x[i + 1u]) >> 1;
                const int mid_y = (mic_y[i] + mic_y[i + 1u]) >> 1;
                gfx_draw_line(gfx, mic_x[i], mic_y[i], mic_x[i + 1u], mic_y[i + 1u]);
                gfx_draw_rect_fill(gfx, mid_x - 1, mid_y - 1, 3, 3);
            } else if (coh < -0.14f) {
                const int lx = (piv_xs[i] + piv_xs[i + 1u]) >> 1;
                gfx_draw_hline_dithered(gfx, lx - 9, FLOOR_Y - 14, 19, 3);
                gfx_draw_line(gfx, lx - 2, FLOOR_Y - 16, lx + 2, FLOOR_Y - 12);
                gfx_draw_line(gfx, lx - 2, FLOOR_Y - 12, lx + 2, FLOOR_Y - 16);
            }
        }
    }
}

static void draw_param_cards(gfx_t* gfx, pm_model_t* m) {
    gfx_set_font(gfx, DepartureMono_Regular_10);
    for (uint8_t idx = 0u; idx < ENC_TOTAL; ++idx) {
        const int x = 8 + (int)idx * (CARD_W + CARD_GAP);
        const bool sel = (m->enc_sel == idx);
        const params_t* p = &m->prm[idx];

        gfx_set_color(gfx, GFX_COLOR_SET);
        ui_draw_frame(gfx, x, CARD_Y, CARD_W, CARD_H, 3, sel);
        gfx_draw_str(gfx, x + 5, CARD_Y + 13, p->name);

        const int16_t pc = (int16_t)(p->val * 100.0f + 0.5f);
        if (pc != m->cached_prm_pc[idx]) {
            m->cached_prm_pc[idx] = pc;
            if (idx == ENC_DECAY && pc == 0) {
                strcpy(m->cached_prm_str[idx], "INF");
            } else {
                snprintf(m->cached_prm_str[idx], sizeof(m->cached_prm_str[idx]), "%d%%", (int)pc);
            }
            m->cached_prm_x[idx] = (int16_t)(x + CARD_W - 5 -
                                             (int)gfx_get_str_width(gfx, m->cached_prm_str[idx]));
        }
        gfx_draw_str(gfx, m->cached_prm_x[idx], CARD_Y + 13, m->cached_prm_str[idx]);

        const int bx = x + 6;
        const int by = CARD_Y + 22;
        const int bw = CARD_W - 12;
        const int bh = 11;
        gfx_draw_rect_r(gfx, bx, by, bw, bh, 2);

        const int fill = (int)(clampf(p->val, 0.0f, 1.0f) * (float)(bw - 4) + 0.5f);
        if (fill > 0) {
            gfx_draw_rect_fill(gfx, bx + 2, by + 2, fill, bh - 4);
        }
    }
}

static void pm_redraw(gfx_t* gfx, const os_app_t* app) {
    pm_model_t* m = (pm_model_t*)os_app_get_model(app);
    if (!m) return;

    draw_header(gfx, m);
    draw_gallery_stage(gfx, m);
    draw_param_cards(gfx, m);
}

/* ----------------------------------------------------------------------------
 * Tick (main UI thread — 30 fps frame-paced while active, zero-cost when idle)
 * ------------------------------------------------------------------------- */
static bool pm_tick(os_app_t* app) {
    pm_model_t* m = (pm_model_t*)os_app_get_model(app);
    tape_t* t = tape_get();
    const uint32_t now_ms = os_tick_get();

    if (m->tape_swapped) {
        m->tape_swapped = false;
        if (m->bouncing) {
            m->bouncing = false;
            m->bounce_tok = 0u;
        }
        if (m->wb) tape_buffer_clear_all_buffers(m->wb);
        m->bounce_cur_win = WIN_NONE;
        tape_timeline_set_loop(false);
        pm_hints(m);
        m->ui_dirty = true;
    }

    if (m->seed_req) {
        m->seed_req = false;
        pm_do_seed_bank(m, true);
    }

    if (m->mon_arm_req) {
        m->mon_arm_req = false;
        pm_sync_monitor(m);
    }

    if (m->state == ST_LOADING) {
        pm_step_take_load(m);
        return true;
    }

    if (m->state == ST_REC && m->rec_done_req) {
        pm_finish_capture(m);
    }

    if (m->bouncing && t) {
        if (!m->bounce_stop_req && m->bounce_frame + BOUNCE_FADE_FR >= m->bounce_limit) {
            m->bounce_stop_req = true;
        }
        if ((m->bounce_stop_req && m->bounce_gain <= 0.001f) ||
            m->bounce_frame >= m->bounce_limit) {
            pm_bounce_stop(m, t);
        }
    }

    pm_sync_engine_active(m);

    /* Update per-pendulum motion trail every ~90 ms only while swinging */
    if (m->state == ST_PLAY && (now_ms - m->last_trail_ms >= 90u)) {
        m->last_trail_ms = now_ms;
        for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
            for (uint32_t k = 0u; k + 1u < TRAIL_LEN; ++k) {
                m->trail_deg[i][k] = m->trail_deg[i][k + 1u];
            }
            const float cd = clampf(m->dsp.deg[i], -60.0f, 60.0f);
            m->trail_deg[i][TRAIL_LEN - 1u] = (int8_t)(cd >= 0.0f ? (cd + 0.5f) : (cd - 0.5f));
        }
    }

    /* Encoder input when neither volume overlay nor menu is open */
    if (!m->vol_ui && !(m->menu && ui_menu_is_visible(m->menu))) {
        const int32_t d = os_controls_encoder_get_delta();
        if (d != 0) {
            params_t* p = &m->prm[m->enc_sel];
            param_write_delta_val(p, d);
            param_update_fast(p, m);
            sync_params_to_dsp(m);
            m->ui_dirty = true;
        }
    }

    if (m->bouncing) {
        const float pos = (float)(m->bounce_frame / FRAMES_PER_POS);
        m->tl_pos.prev = m->tl_pos.val;
        m->tl_pos.val = m->tl_pos.target = pos;
        m->tl_end.val = m->tl_end.target = pos;
    }

    if (!m->vol_ui && (m->hint_state != m->state ||
                       m->hint_mode != m->mode ||
                       m->hint_bounce != m->bouncing)) {
        m->hint_state = m->state;
        m->hint_mode = m->mode;
        m->hint_bounce = m->bouncing;
        pm_hints(m);
        m->ui_dirty = true;
    }

    const bool toast_win = (m->anim_until_ms != 0u) && ((int32_t)(m->anim_until_ms - now_ms) > 0);
    const bool animating = (m->state == ST_PLAY || m->state == ST_REC ||
                            m->bouncing || toast_win);
    const uint32_t frame_ms = m->vol_ui ? UI_VOL_FRAME_MS : UI_FRAME_MS;
    if ((m->ui_dirty || animating) &&
        (m->last_draw_ms == 0u || (now_ms - m->last_draw_ms >= frame_ms))) {
        m->ui_dirty = false;
        m->last_draw_ms = (now_ms != 0u) ? now_ms : 1u;
        return true;
    }

    return false;
}

/* ----------------------------------------------------------------------------
 * Input & Resume Handlers
 * ------------------------------------------------------------------------- */
static void pm_on_resume(os_app_t* app) {
    pm_model_t* m = (pm_model_t*)os_app_get_model(app);
    if (!m) return;
    pm_sync_monitor(m);
    pm_sync_engine_active(m);
    m->ui_dirty = true;
}

static void pm_input(os_app_t* app, uint8_t btn, KeyStateEnum st) {
    pm_model_t* m = (pm_model_t*)os_app_get_model(app);
    if (btn == BTN_ENCODER) return;

    if (m->vol_ui) {
        if (st == KEY_STATE_RELEASED && btn == BTN1) pm_vol_hide(m);
        return;
    }

    if (st == KEY_STATE_RELEASED && btn == BTN2 && m->rec_btn_down) {
        m->rec_btn_down = false;
        m->rec_holdoff_fr = MIC_START_SETTLE_FR24;
        return;
    }

    if (m->state == ST_LOADING) return;

    if (st == KEY_STATE_PRESSED) {
        switch (btn) {
        case BTN1:
            if (m->state == ST_PLAY) {
                m->state = ST_STOP;
                pm_sync_monitor(m);
                pm_sync_engine_active(m);
            } else if (m->state == ST_STOP) {
                if (pm_all_nearly_still(&m->dsp, 3.0f)) {
                    m->pull_req = true;
                }
                m->state = ST_PLAY;
                pm_sync_monitor(m);
                pm_sync_engine_active(m);
            }
            break;

        case BTN2:
            if (m->state == ST_REC) {
                m->rec_btn_down = false;
                m->rec_holdoff_fr = 0u;
                m->rec_manual_stop = true;
                m->rec_done_req = true;
                pm_sync_monitor(m);
            } else if (m->slots_mem) {
                m->buf_locked = true;
                m->rec_frames = 0u;
                m->rec_done_req = false;
                m->mon_arm_req = false;
                m->rec_manual_stop = false;
                m->rec_held_fr = 0u;
                m->rec_hp_xl = m->rec_hp_yl = 0.0f;
                m->rec_hp_xr = m->rec_hp_yr = 0.0f;
                m->in_src = os_audio_get_input();
                if (m->in_src == 1u) {
                    m->rec_btn_down = true;
                    m->rec_holdoff_fr = MIC_START_SETTLE_FR24;
                } else {
                    m->rec_btn_down = false;
                    m->rec_holdoff_fr = 0u;
                }
                m->state = ST_REC;
                m->buf_locked = false;
                pm_sync_monitor(m);
                pm_sync_engine_active(m);
            }
            break;

        case BTN3:
            m->mode = (m->mode == PM_MODE_FEEDBACK) ? PM_MODE_SOUND_VCA : PM_MODE_FEEDBACK;
            break;

        case BTN4:
            m->enc_sel = (uint8_t)((m->enc_sel + ENC_TOTAL - 1u) % ENC_TOTAL);
            ui_statusbar_hint_event(HintPress4);
            break;

        case BTN5:
            m->enc_sel = (uint8_t)((m->enc_sel + 1u) % ENC_TOTAL);
            ui_statusbar_hint_event(HintPress5);
            break;

        default:
            break;
        }

        m->ui_dirty = true;
        m->hint_state = m->state;
        m->hint_mode = m->mode;
        m->hint_bounce = m->bouncing;
        pm_hints(m);
    } else if (st == KEY_STATE_HOLD) {
        if (btn == BTN1 && !m->vol_ui) {
            m->vol_ui = true;
            pm_vol_show(m);
        } else if (btn == BTN2 && m->state != ST_REC) {
            if (m->bouncing) {
                m->bounce_stop_req = true;
            } else {
                pm_bounce_start(m);
            }
        } else if (btn == BTN3) {
            if (m->menu) ui_menu_show(m->menu);
        }
    }
}

/* ----------------------------------------------------------------------------
 * Lifecycle (init / deinit)
 * ------------------------------------------------------------------------- */
static bool pm_app_init(os_app_t* app) {
    pm_model_t* m = (pm_model_t*)os_app_get_model(app);

    /*
     * 1. `m->space` (7.5 KB) is already embedded inside `pm_model_t` in fast
     *    internal SRAM! Allocate SD staging, DMA window pair, and menu first.
     */
    m->read_stage = (float*)os_malloc((size_t)LOAD_CHUNK_FR48 * 2u * sizeof(float));

    tape_t* t = tape_get();
    if (t) {
        m->wb = tape_buffer_init(t);
    }
    m->bounce_cur_win = WIN_NONE;

    m->menu = ui_menu_create(m, menu_root_builder);
    if (m->menu) {
        ui_menu_set_name(m->menu, "PENDULUM MUSIC");
        ui_menu_set_header(m->menu, true);
    }

    /*
     * 2. Allocate PSRAM for 4 custom 24 kHz recording/take slots (4s -> 3s -> 2s each),
     *    enforcing >= 512 KB of free OS heap afterward so menus/icons/toasts never starve.
     */
    static const uint32_t kCandSecs[] = { 4u, 3u, 2u, 1u };
    for (uint32_t c = 0u; c < (sizeof(kCandSecs) / sizeof(kCandSecs[0])); ++c) {
        const uint32_t per_slot = kCandSecs[c] * PM_INTERNAL_SR;
        const size_t bytes = (size_t)per_slot * PM_NUM_MICS * sizeof(pm_frame_t);
        pm_frame_t* mem = (pm_frame_t*)os_malloc(bytes);
        if (!mem) continue;
        void* heap_guard = os_malloc(512u * 1024u);
        if (heap_guard) {
            os_free(heap_guard);
            m->slots_mem = mem;
            m->max_slot_frames = per_slot;
            for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
                m->slot_buf[s] = mem + s * per_slot;
            }
            break;
        }
        os_free(mem);
    }

    /* 3. Initialize 24 kHz Reich 4-pendulum coupled acoustic feedback DSP */
    pm_dsp_init(&m->dsp, (float)PM_INTERNAL_SR);
    pm_dsp_attach_space(&m->dsp, &m->space);

    m->mode = PM_MODE_FEEDBACK;
    m->mode_mix = 0.0f;
    m->rec_slot = 0u;
    m->bank_idx = PM_BANK_ORGANS;
    pm_do_seed_bank(m, false);

    /* Invalidate cached UI strings so first redraw populates them */
    m->cached_phi_pc = -32767;
    for (uint8_t idx = 0u; idx < ENC_TOTAL; ++idx) {
        m->cached_prm_pc[idx] = -32767;
    }

    /* 4 live encoder cards: GAIN, COUPLE, SPREAD, DECAY */
    m->prm[ENC_GAIN]   = (params_t)PARAM_RANGE("GAIN",   0.58f, 0.00f, 1.00f, 0.02f);
    m->prm[ENC_COUPLE] = (params_t)PARAM_RANGE("COUPLE", 0.50f, 0.00f, 1.00f, 0.02f);
    m->prm[ENC_SPREAD] = (params_t)PARAM_RANGE("SPREAD", 0.42f, 0.00f, 1.00f, 0.02f);
    m->prm[ENC_DECAY]  = (params_t)PARAM_RANGE("DECAY",  0.35f, 0.00f, 1.00f, 0.02f);
    m->enc_sel = ENC_GAIN;
    sync_params_to_dsp(m);
    pm_pull_and_release(&m->dsp);

    for (uint32_t i = 0u; i < PM_NUM_MICS; ++i) {
        for (uint32_t k = 0u; k < TRAIL_LEN; ++k) {
            m->trail_deg[i][k] = (int8_t)m->dsp.deg[i];
        }
    }

    /* Volume overlay (default 25% = 0.12f / -18.4 dB for headphones) & mic trim */
    m->vol_out = (params_t)PARAM_RANGE("out", 0.12f, 0.0f, 2.0f, 0.02f);
    m->vol_out.type = ParamValDBType;
    m->vol_fx = (params_t)PARAM_RANGE("fx send", 0.0f, 0.0f, 1.0f, 0.02f);
    m->vol_fx.type = ParamValDBType;
    m->mic_trim = (params_t)PARAM_RANGE("mic trim", 0.40f, 0.05f, 1.50f, 0.02f);
    m->mic_trim.type = ParamValDBType;
    m->vout_cur = 0.12f;
    m->vfx_cur  = 0.0f;

    /* Tape timeline strip along bottom edge */
    m->tl_pos   = (params_t)PARAM_RANGE("pos",   0.0f, 0.0f, 1.0e9f, 1.0f);
    m->tl_start = (params_t)PARAM_RANGE("start", 0.0f, 0.0f, 1.0e9f, 1.0f);
    m->tl_end   = (params_t)PARAM_RANGE("end",   0.0f, 0.0f, 1.0e9f, 1.0f);

    if (t) {
        tape_timeline_setup(1u, 8, 208, SCREEN_WIDTH - 16, 28);
        tape_timeline_attach_params(0u, &m->tl_pos, &m->tl_start, &m->tl_end);
        tape_timeline_switch_track(0u);
        tape_timeline_show_time_display(false);
        tape_timeline_set_loop(false);
        tape_timeline_show(true);
        tape_timeline_zoom_out();
    }

    m->in_src = os_audio_get_input();
    m->mon_saved = os_audio_get_monitor();
    m->wet_saved = os_audio_get_wet();
    m->mon_req = false;
    pm_tame_hot_mic_input();
    (void)os_audio_set_monitor(false);
    (void)os_audio_set_wet(false);

    ui_statusbar_show(false);
    ui_hints_show(true);
    m->hint_state = m->state;
    m->hint_mode = m->mode;
    m->hint_bounce = m->bouncing;
    m->ui_dirty = true;
    pm_hints(m);

    os_led_set_pattern(&pm_led_pattern, m);
    engine_set_active(false);
    return true;
}

static void pm_app_deinit(os_app_t* app) {
    pm_model_t* m = (pm_model_t*)os_app_get_model(app);
    if (!m) return;

    (void)os_audio_set_wet(m->wet_saved);
    (void)os_audio_set_monitor(m->mon_saved);

    if (m->bouncing) {
        pm_bounce_stop(m, tape_get());
    }
    if (m->menu) {
        ui_menu_destroy(m->menu);
        m->menu = NULL;
    }

    tape_timeline_set_loop(false);
    tape_timeline_show(false);

    if (m->wb) {
        tape_buffer_deinit(m->wb);
        m->wb = NULL;
    }
    if (m->read_stage) {
        os_free(m->read_stage);
        m->read_stage = NULL;
    }
    if (m->slots_mem) {
        os_free(m->slots_mem);
        m->slots_mem = NULL;
        for (uint32_t s = 0u; s < PM_NUM_MICS; ++s) {
            m->slot_buf[s] = NULL;
        }
    }
}

TAPP_APP("Pendulum Music", pm_model_t, pm_app_init, pm_app_deinit,
         .redraw = pm_redraw,
         .tick = pm_tick,
         .on_input = pm_input,
         .on_resume = pm_on_resume,
         .engine_cb = &pm_engine_cb);
