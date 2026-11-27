/*
 * gonna_rain.c — Steve Reich "It's Gonna Rain" (1965) Standalone TAPP for tape!
 *
 * Dual-deck (Part I) and Quad-deck (Part II) virtual Wollensak reel-to-reel
 * tape-phasing instrument. Captures or loads a loop in 0.0 deg unison, then lets
 * Deck B's capstan drift gradually ahead of Deck A through resonant comb-filtering,
 * slapback echo doubling, rhythmic hockets, and contrapuntal polyrhythms all the
 * way around to 360.0 deg unison.
 *
 * Controls:
 *   BTN1 press : Play / Stop reels
 *   BTN1 hold  : Volume overlay (out / fx send / in)
 *   BTN2 press : Capture live input into RAM loop / Close loop & start in Unison
 *   BTN2 hold  : Bounce phasing performance to free SD tape space (DMA windows)
 *   BTN3 press : Toggle Phase Drift ON (PHASING) / Capstan Thumb-Lock (LOCKED)
 *   BTN3 hold  : Menu (Load Tape Take, Re-Align Unison, Snap Canon, +90 deg,
 *                2/4 Decks, Continuous/Dwell, Sub-Loop Crop, Dir, Speed, Input, Clear)
 *   BTN4 / 5   : Previous / Next encoder parameter (DRIFT, PHASE, SPREAD, TAPE)
 *   ENCODER    : Adjust parameter (on PHASE: smooth thumb-on-reel-flange phase scrub!)
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tapp_api.h"
#include "reich_dsp.h"

TAPP_DECLARE_TYPE(AppTypeStandalone);

/* ----------------------------------------------------------------------------
 * Constants & Configuration
 * ------------------------------------------------------------------------- */
#define FRAMES_PER_POS   64u
#define WIN_NONE         0xFFFFFFFFu
#define MIN_LOOP_FR      (TAPP_SAMPLE_RATE / 5u)    /* 0.2 s minimum loop */
#define SPLICE_FADE_FR       960u                   /* 20 ms smoothstep loop splice */
#define REC_FADE_IN_FR       1200u                  /* 25 ms smoothstep capture fade-in */
#define MIC_START_SETTLE_FR  4080u                  /* 85 ms post-BTN2-release settle */
#define MIC_BTN_HELD_MAX_FR  12000u                 /* 250 ms max holdoff while BTN2 down */
#define MIC_STOP_TRIM_FR     5280u                  /* 110 ms stop-button click tail trim */
#define BOUNCE_FADE_FR       1440u                  /* 30 ms bounce fade in/out */
#define MIN_BOUNCE_FR        (2u * TAPP_SAMPLE_RATE)/* require >= 2 s free tape */
#define BOUNCE_GUARD_FR      (128u * FRAMES_PER_POS)
#define COMMIT_TRIM_POS      64u
#define LOAD_CHUNK_FR        2048u                  /* ~42 ms per tick during tape_read */

#define WAVE_BINS        134u                       /* 134 * 2px = 268px in right stage */
#define MAX_MENU_TAKES   32u

enum {
    ST_STOP = 0,
    ST_PLAY,
    ST_REC,
    ST_LOADING,
};

enum {
    ENC_DRIFT = 0,
    ENC_PHASE,
    ENC_SPREAD,
    ENC_TAPE,
    ENC_TOTAL,
};

enum {
    CROP_FULL = 0,
    CROP_HALF,
    CROP_QUARTER,
    CROP_TIGHT_18, /* 1.80s authentic 1965 "It's Gonna Rain" loop window */
    CROP_COUNT,
};

/* ----------------------------------------------------------------------------
 * Model (~2.4 KB in internal SRAM; audio loop buffer in PSRAM via os_malloc)
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Transport & thread-sync flags */
    volatile uint8_t  state;
    volatile bool     buf_locked;
    volatile bool     drifting;       /* true = PHASING, false = LOCKED */
    volatile bool     unison_intro;   /* play Lap 1 in pure 0.0 deg unison first */
    volatile bool     rec_done_req;
    volatile bool     tape_swapped;
    volatile bool     realign_req;
    volatile bool     snap_req;
    volatile bool     jump90_req;
    volatile bool     clear_req;

    /* Audio engine active flag */
    volatile bool     eng_active;

    /* Non-destructive shared stereo loop buffer in PSRAM */
    reich_frame_t*    loop_buf;
    float*            read_stage;
    uint32_t          max_frames;
    volatile uint32_t loop_len;       /* Full captured/loaded take length */
    uint8_t           crop_mode;      /* CROP_FULL .. CROP_TIGHT_18 */

    /* Live capture & button-click pop suppression state */
    volatile uint32_t rec_frames;
    volatile bool     rec_btn_down;
    volatile bool     rec_manual_stop;
    volatile uint32_t rec_holdoff_fr;
    volatile uint32_t rec_held_fr;
    float             rec_hp_xl;
    float             rec_hp_yl;
    float             rec_hp_xr;
    float             rec_hp_yr;

    /* Smooth varispeed motor & output envelope */
    float             speed_target;
    float             speed_cur;
    float             play_gain;

    /* Core Steve Reich Tape-Phasing DSP engine */
    reich_engine_t    dsp;

    /* Chunked tape-take loader state */
    uint32_t          load_src_frame;
    uint32_t          load_total;
    uint32_t          load_done;

    /* Bounce-to-tape streaming state (DMA windows) */
    tape_buffer_t*    wb;
    volatile bool     bouncing;
    volatile bool     bounce_stop_req;
    volatile float    bounce_gain;
    volatile uint32_t bounce_frame;
    volatile uint32_t bounce_cur_win;
    uint32_t          bounce_tok;
    uint32_t          bounce_start;
    uint32_t          bounce_limit;

    /* Metering & active-window waveform bins */
    volatile float    peak_l;
    volatile float    peak_r;
    uint8_t           wave_bins[WAVE_BINS];

    /* Live performance parameters */
    params_t          prm[ENC_TOTAL];
    uint8_t           enc_sel;

    /* Volume overlay & mic trim params */
    params_t          vol_out;
    params_t          vol_fx;
    params_t          mic_trim;       /* app-local input trim (0.40x default when Mic) */
    float             vout_cur;
    float             vfx_cur;
    volatile bool     vol_ui;
    bool              mon_saved;      /* user's monitor state saved at launch, restored at exit */
    bool              wet_saved;      /* user's WET state saved at launch, restored at exit */
    bool              mon_req;        /* true when monitor ON has been requested for ST_REC */

    /* Menu & cached UI strings */
    ui_menu_t*        menu;
    uint32_t          last_draw_ms;
    uint8_t           in_src;
    uint8_t           hint_state;
    bool              hint_drift;
    bool              hint_bounce;
    char              take_labels[MAX_MENU_TAKES][24];
    char              decks_label[28];
    char              motion_label[28];
    char              crop_label[28];
    char              dir_label[24];
    char              input_label[20];
} rain_model_t;

/* Forward declarations */
static void rain_hints(const rain_model_t* m);
static void rain_finish_capture(rain_model_t* m);
static void rain_bounce_stop(rain_model_t* m, tape_t* t);
static void rain_sync_engine_active(rain_model_t* m);
static void rain_sync_monitor(rain_model_t* m);

/* ----------------------------------------------------------------------------
 * Loop Splice, Window Geometry & Waveform Bin Builder
 * ------------------------------------------------------------------------- */
static void apply_loop_splice(reich_frame_t* buf, uint32_t len) {
    if (!buf || len < 16u) return;
    uint32_t fade = SPLICE_FADE_FR;
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

static const char* crop_mode_name(uint8_t mode) {
    switch (mode) {
    case CROP_HALF:     return "1/2 Loop";
    case CROP_QUARTER:  return "1/4 Loop";
    case CROP_TIGHT_18: return "1.8s Tight";
    default:            return "Full Loop";
    }
}

static void compute_crop_window(uint32_t full_len, uint8_t crop_mode,
                                uint32_t* out_start, uint32_t* out_len) {
    uint32_t wlen = full_len;
    if (crop_mode == CROP_HALF) {
        wlen = full_len >> 1;
    } else if (crop_mode == CROP_QUARTER) {
        wlen = full_len >> 2;
    } else if (crop_mode == CROP_TIGHT_18) {
        const uint32_t tight = (uint32_t)(1.8f * (float)TAPP_SAMPLE_RATE); /* 86,400 fr */
        if (tight < full_len) wlen = tight;
    }
    if (wlen < MIN_LOOP_FR) {
        wlen = (full_len < MIN_LOOP_FR) ? full_len : MIN_LOOP_FR;
    }
    if (wlen > full_len) wlen = full_len;
    *out_start = 0u;
    *out_len = wlen;
}

static void rebuild_waveform_bins(rain_model_t* m) {
    const uint32_t wstart = m->dsp.win_start;
    const uint32_t wlen = m->dsp.win_len;
    if (!m->loop_buf || wlen < 8u || (wstart + wlen) > m->loop_len) {
        memset(m->wave_bins, 0, sizeof(m->wave_bins));
        return;
    }

    for (uint32_t b = 0u; b < WAVE_BINS; ++b) {
        const uint32_t f0 = (b * wlen) / WAVE_BINS;
        uint32_t f1 = ((b + 1u) * wlen) / WAVE_BINS;
        if (f1 <= f0) f1 = f0 + 1u;
        if (f1 > wlen) f1 = wlen;

        uint32_t stride = (f1 - f0) / 32u;
        if (stride < 1u) stride = 1u;

        float pk = 0.0f;
        for (uint32_t i = f0; i < f1; i += stride) {
            const reich_frame_t fr = m->loop_buf[wstart + i];
            const float al = fabsf(fr.l);
            const float ar = fabsf(fr.r);
            const float a = (al > ar) ? al : ar;
            if (a > pk) pk = a;
        }
        m->wave_bins[b] = (uint8_t)(clampf(pk, 0.0f, 1.0f) * 255.0f + 0.5f);
    }
}

static void sync_params_to_dsp(rain_model_t* m) {
    reich_set_drift(&m->dsp, m->prm[ENC_DRIFT].val);
    reich_set_spread(&m->dsp, m->prm[ENC_SPREAD].val);
    reich_set_tape(&m->dsp, m->prm[ENC_TAPE].val);
    reich_set_drifting(&m->dsp, m->drifting);
}

static void rain_sync_engine_active(rain_model_t* m) {
    if (!m->eng_active) {
        m->eng_active = true;
        engine_set_active(true);
    }
}

/*
 * Keep live input monitoring and WET FX-chain pass-through strictly OFF unless
 * actively capturing (ST_REC), preventing mic bleed or feedback out of speakers/headphones.
 */
static void rain_sync_monitor(rain_model_t* m) {
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

static void rain_tame_hot_mic_input(void) {
    if (os_audio_get_input() != 1u) return;
    mixer_t* mx = mixer_get();
    params_t* in_vol = mx ? mixer_get_input_vol(mx) : NULL;
    if (in_vol && in_vol->val > 0.35f) {
        in_vol->target = 0.35f;
        param_update_fast(in_vol, NULL);
    }
}

/* ----------------------------------------------------------------------------
 * Bounce-to-Tape Streaming (DMA windows)
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

static void rain_bounce_start(rain_model_t* m) {
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

    m->bouncing = true;
    rain_sync_engine_active(m);
    rain_hints(m);
    ui_notify_toast(NotifyInfo, "Bouncing phase to tape", 1800u);
}

static void rain_bounce_stop(rain_model_t* m, tape_t* t) {
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
        }
        ui_notify_toast(NotifySuccess, "Phase bounce saved", 1800u);
    }

    rain_sync_engine_active(m);
    rain_hints(m);
}

/* ----------------------------------------------------------------------------
 * Audio Engine (real-time 48 kHz, zero blocking, single-precision f32 only)
 * ------------------------------------------------------------------------- */
static inline void bounce_win_request(rain_model_t* m, uint32_t win) {
    tape_buffer_update(m->wb, win * TAPE_WINDOW_POS, true);
}

static void rain_process(void* restrict ctx, mixer_t* restrict mix) {
    rain_model_t* m = (rain_model_t*)ctx;
    float* out = mixer_get_out(mix);
    const float* in = mixer_get_record(mix);
    float* send = mixer_get_send(mix);
    const uint32_t fs = mixer_get_fs(mix);
    if (!out) return;

    const uint8_t st = m->state;
    const bool locked = m->buf_locked;
    const bool bouncing = m->bouncing && (m->wb != NULL);

    if (st == ST_REC || bouncing) {
        mixer_rec_active(mix);
    }

    if (locked) {
        memset(out, 0, (size_t)fs * sizeof(float));
        return;
    }

    const float vout_tgt = m->vol_out.val;
    const float vfx_tgt  = m->vol_fx.val;
    const float play_target = (st == ST_PLAY && m->loop_len >= 8u) ? 1.0f : 0.0f;

    if (st != ST_REC && play_target == 0.0f && m->play_gain <= 0.0001f && !bouncing) {
        m->play_gain = 0.0f;
        m->vout_cur  = vout_tgt;
        m->vfx_cur   = vfx_tgt;
        memset(out, 0, (size_t)fs * sizeof(float));
        m->peak_l *= 0.85f;
        m->peak_r *= 0.85f;
        return;
    }

    float vo = m->vout_cur;
    float vf = m->vfx_cur;
    const float vol_slew = 0.0045f;
    const float itrim = (m->in_src == 1u) ? m->mic_trim.val : 1.0f;
    const float bounce_target = (bouncing && !m->bounce_stop_req) ? 1.0f : 0.0f;
    const float play_step = 1.0f / 720.0f;       /* ~15 ms anti-click */
    const float bounce_step = 1.0f / (float)BOUNCE_FADE_FR;

    float* bbuf = bouncing ? tape_buffer_get_ptr(m->wb) : NULL;
    float pl = 0.0f;
    float pr = 0.0f;

    reich_frame_t* lbuf = m->loop_buf;
    const uint32_t len = m->loop_len;

    for (uint32_t i = 0u; i < fs; i += 2u) {
        float ol = 0.0f;
        float orr = 0.0f;

        if (st == ST_REC && lbuf && in) {
            float il = in[i] * itrim;
            float ir = in[i + 1u] * itrim;

            if (m->in_src == 1u) {
                const float hpl = 0.9889f * (m->rec_hp_yl + il - m->rec_hp_xl);
                const float hpr = 0.9889f * (m->rec_hp_yr + ir - m->rec_hp_xr);
                m->rec_hp_xl = il;
                m->rec_hp_yl = (fabsf(hpl) < 1.0e-18f) ? 0.0f : hpl;
                m->rec_hp_xr = ir;
                m->rec_hp_yr = (fabsf(hpr) < 1.0e-18f) ? 0.0f : hpr;
                il = m->rec_hp_yl;
                ir = m->rec_hp_yr;
            }

            if (m->rec_btn_down) {
                m->rec_held_fr++;
                if (m->rec_held_fr >= MIC_BTN_HELD_MAX_FR) {
                    m->rec_btn_down = false;
                }
            } else if (m->rec_holdoff_fr > 0u) {
                m->rec_holdoff_fr--;
            } else {
                const uint32_t rf = m->rec_frames;
                if (rf < m->max_frames && !m->rec_done_req) {
                    float fg = 1.0f;
                    if (rf < REC_FADE_IN_FR) {
                        const float t = (float)rf * (1.0f / (float)REC_FADE_IN_FR);
                        fg = t * t * (3.0f - 2.0f * t);
                    }
                    lbuf[rf].l = reich_clip_guard(il * fg);
                    lbuf[rf].r = reich_clip_guard(ir * fg);
                    m->rec_frames = rf + 1u;
                    if (m->rec_frames >= m->max_frames) {
                        m->rec_done_req = true;
                    }
                }
                const float al = fabsf(il);
                const float ar = fabsf(ir);
                if (al > pl) pl = al;
                if (ar > pr) pr = ar;
            }
        } else {
            /* Smooth capstan speed & play envelope */
            m->speed_cur += 0.002f * (m->speed_target - m->speed_cur);
            float pg = m->play_gain;
            if (pg < play_target) {
                pg += play_step;
                if (pg > play_target) pg = play_target;
            } else if (pg > play_target) {
                pg -= play_step;
                if (pg < play_target) pg = play_target;
            }
            m->play_gain = pg;

            if (pg > 0.0001f && lbuf && len >= 8u) {
                /* Hold Lap 1 in 0.0 deg unison statement, then release Deck B to phase */
                if (m->unison_intro && m->dsp.lap_count >= 2u) {
                    m->unison_intro = false;
                }
                const bool allow_drift = (st == ST_PLAY) && !m->unison_intro;
                const reich_frame_t ph = reich_process_frame(&m->dsp, lbuf, m->speed_cur, allow_drift);

                ol  = ph.l * pg;
                orr = ph.r * pg;
            }

            const float al = fabsf(ol);
            const float ar = fabsf(orr);
            if (al > pl) pl = al;
            if (ar > pr) pr = ar;
        }

        /* Optional Bounce-to-Tape streaming write (unity pre-vout level) */
        if (bouncing) {
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

            float bg = m->bounce_gain;
            if (bg < bounce_target) {
                bg += bounce_step;
                if (bg > bounce_target) bg = bounce_target;
            } else if (bg > bounce_target) {
                bg -= bounce_step;
                if (bg < bounce_target) bg = bounce_target;
            }
            m->bounce_gain = bg;

            if (bbuf && (bg > 0.0f || !m->bounce_stop_req) && bf < m->bounce_limit) {
                const uint32_t off = (bf & (TAPE_WINDOW_FR - 1u)) * 2u;
                bbuf[off]      = clampf(ol * bg, -1.0f, 1.0f);
                bbuf[off + 1u] = clampf(orr * bg, -1.0f, 1.0f);
                m->bounce_frame = bf + 1u;
            }
        }

        vo += vol_slew * (vout_tgt - vo);
        vf += vol_slew * (vfx_tgt  - vf);
        const float out_l = ol * vo;
        const float out_r = orr * vo;
        out[i]      = out_l;
        out[i + 1u] = out_r;
        if (send) {
            send[i]      += out_l * vf;
            send[i + 1u] += out_r * vf;
        }
    }

    if (fabsf(vout_tgt - vo) < 1.0e-5f) vo = vout_tgt;
    if (fabsf(vfx_tgt  - vf) < 1.0e-5f) vf = vfx_tgt;
    m->vout_cur = vo;
    m->vfx_cur  = vf;

    m->peak_l = (pl > m->peak_l) ? pl : (m->peak_l * 0.85f);
    m->peak_r = (pr > m->peak_r) ? pr : (m->peak_r * 0.85f);
}

static void rain_eng_active(void* ctx, bool state) {
    ((rain_model_t*)ctx)->eng_active = state;
}

static void rain_tape_changed(void* ctx) {
    ((rain_model_t*)ctx)->tape_swapped = true;
}

static const engine_callbacks_t rain_engine_cb = {
    .process = rain_process,
    .tape_changed = rain_tape_changed,
    .active = rain_eng_active,
};

/* ----------------------------------------------------------------------------
 * RGB LED Pattern (LEDs 0-1 chase Deck A; LEDs 2-3 chase Deck B)
 * Unison warm Wollensak gold morphs into storm-rain cyan/indigo hockets
 * ------------------------------------------------------------------------- */
static void rain_led_step(const os_led_t* led, const void* ctx) {
    const uint8_t step = os_led_get_step(led);
    const rain_model_t* m = (const rain_model_t*)ctx;
    if (!m) return;

    if (m->state == ST_REC) {
        const int tri = (step & 32u) ? (63 - (int)(step & 63u)) : (int)(step & 63u);
        const uint8_t r = (uint8_t)(16 + tri * 2);
        for (uint8_t i = 0u; i < 4u; ++i) hal_led_set_rgb(i, r, 2u, 2u);
        return;
    }

    if (m->state == ST_PLAY && m->dsp.win_len >= 8u) {
        const float inv_w = m->dsp.inv_win_len;
        const float pos_a = clampf((float)m->dsp.voice_pos_i[0] * inv_w, 0.0f, 0.999f);
        const float pos_b = clampf((float)m->dsp.voice_pos_i[1] * inv_w, 0.0f, 0.999f);
        const uint8_t sub_a = (pos_a >= 0.5f) ? 1u : 0u;
        const uint8_t sub_b = (pos_b >= 0.5f) ? 3u : 2u;

        /* Separation metric: 0.0 at 0/360 deg unison -> 1.0 at 180 deg half-loop hocket */
        const float turns = reich_get_phase_turns(&m->dsp);
        const float sep = 1.0f - fabsf(2.0f * turns - 1.0f);

        const float env_a = clampf(0.32f + 1.4f * m->dsp.voice_env[0], 0.30f, 1.0f);
        const float env_b = clampf(0.32f + 1.4f * m->dsp.voice_env[1], 0.30f, 1.0f);

        for (uint8_t i = 0u; i < 4u; ++i) {
            if (m->bouncing && i == 0u) {
                const uint8_t pulse = ((step & 16u) == 0u) ? 58u : 18u;
                hal_led_set_rgb(0u, pulse, (uint8_t)(pulse >> 2), 0u);
                continue;
            }

            if (i < 2u) {
                /* Deck A pair (LEDs 0 & 1): warm tungsten-gold anchor */
                const float br = ((i == sub_a) ? 1.0f : 0.32f) * env_a;
                const uint8_t r = (uint8_t)((58.0f - 20.0f * sep) * br);
                const uint8_t g = (uint8_t)((40.0f + 4.0f * sep) * br);
                const uint8_t b = (uint8_t)((8.0f  + 22.0f * sep) * br);
                hal_led_set_rgb(i, r, g, b);
            } else {
                /* Deck B pair (LEDs 2 & 3): gold at unison -> crisp rain-cyan/indigo out of phase */
                const float br = ((i == sub_b) ? 1.0f : 0.32f) * env_b;
                const uint8_t r = (uint8_t)((58.0f * (1.0f - sep) + 8.0f * sep) * br);
                const uint8_t g = (uint8_t)((40.0f * (1.0f - sep) + 46.0f * sep) * br);
                const uint8_t b = (uint8_t)((8.0f  * (1.0f - sep) + 64.0f * sep) * br);
                hal_led_set_rgb(i, r, g, b);
            }
        }
        return;
    }

    for (uint8_t i = 0u; i < 4u; ++i) {
        if (m->bouncing && i == 0u) {
            hal_led_set_rgb(0u, 36u, 8u, 0u);
        } else {
            hal_led_set_rgb(i, 2u, 3u, 5u);
        }
    }
}

static led_pattern_t rain_led_pattern = {
    .callback = rain_led_step,
    .steps = 255u,
    .rate = 35u,
    .loop = true,
};

/* ----------------------------------------------------------------------------
 * Loop Capture Finalization, Clear, Take Loading & Phase Commands
 * ------------------------------------------------------------------------- */
static void rain_prepare_fresh_loop(rain_model_t* m, uint32_t captured_len) {
    apply_loop_splice(m->loop_buf, captured_len);
    m->loop_len = captured_len;
    m->crop_mode = CROP_FULL;

    uint32_t wstart = 0u, wlen = captured_len;
    compute_crop_window(captured_len, m->crop_mode, &wstart, &wlen);

    reich_reset_loop(&m->dsp, captured_len, wstart, wlen);
    sync_params_to_dsp(m);
    rebuild_waveform_bins(m);

    m->unison_intro = true;
    m->play_gain = 0.0f;
    m->prm[ENC_PHASE].val = m->prm[ENC_PHASE].target = 0.0f;
}

static void rain_finish_capture(rain_model_t* m) {
    m->buf_locked = true;
    m->rec_done_req = false;
    m->rec_btn_down = false;
    m->rec_holdoff_fr = 0u;
    uint32_t captured = m->rec_frames;
    if (m->in_src == 1u && m->rec_manual_stop) {
        if (captured > MIN_LOOP_FR + MIC_STOP_TRIM_FR) {
            captured -= MIC_STOP_TRIM_FR;
        } else if (captured > MIN_LOOP_FR) {
            captured = MIN_LOOP_FR;
        }
    }
    m->rec_manual_stop = false;

    if (captured < MIN_LOOP_FR) {
        m->state = ST_STOP;
        m->buf_locked = false;
        rain_sync_monitor(m);
        rain_sync_engine_active(m);
        rain_hints(m);
        ui_notify_toast(NotifyWarning, "Loop too short", 1800u);
        return;
    }

    rain_prepare_fresh_loop(m, captured);

    m->state = ST_PLAY;
    m->buf_locked = false;
    rain_sync_monitor(m);
    rain_sync_engine_active(m);
    rain_hints(m);
}

static void rain_do_clear(rain_model_t* m) {
    m->buf_locked = true;
    m->state = ST_STOP;
    m->loop_len = 0u;
    m->crop_mode = CROP_FULL;
    m->unison_intro = false;
    memset(m->wave_bins, 0, sizeof(m->wave_bins));
    reich_reset_loop(&m->dsp, 8u, 0u, 8u);
    m->prm[ENC_PHASE].val = m->prm[ENC_PHASE].target = 0.0f;
    m->buf_locked = false;
    rain_sync_engine_active(m);
    rain_hints(m);
    ui_notify_toast(NotifyInfo, "Loop cleared", 1800u);
}

static void rain_start_take_load(rain_model_t* m, uint32_t take_idx) {
    tape_t* t = tape_get();
    if (!t || !m->loop_buf || !m->read_stage) {
        ui_notify_toast(NotifyWarning, "Tape unavailable", 1800u);
        return;
    }
    uint32_t s = 0u, e = 0u;
    if (!tape_marks_get(t, take_idx, &s, &e) || e <= s) {
        ui_notify_toast(NotifyWarning, "Empty take", 1800u);
        return;
    }

    uint32_t len = e - s;
    if (len > m->max_frames) len = m->max_frames;
    if (len < MIN_LOOP_FR) {
        ui_notify_toast(NotifyWarning, "Take too short", 1800u);
        return;
    }

    m->buf_locked = true;
    m->load_src_frame = s;
    m->load_total = len;
    m->load_done = 0u;
    m->state = ST_LOADING;
    rain_sync_engine_active(m);
    rain_hints(m);
}

static void rain_step_take_load(rain_model_t* m) {
    tape_t* t = tape_get();
    if (!t || !m->read_stage || !m->loop_buf) {
        m->state = ST_STOP;
        m->buf_locked = false;
        return;
    }

    const uint32_t rem = m->load_total - m->load_done;
    const uint32_t chunk = (rem > LOAD_CHUNK_FR) ? LOAD_CHUNK_FR : rem;
    const uint32_t got = tape_read(t, m->load_src_frame + m->load_done, m->read_stage, chunk);
    if (got == 0u) {
        m->state = ST_STOP;
        m->buf_locked = false;
        ui_notify_toast(NotifyError, "Tape read failed", 1800u);
        return;
    }

    for (uint32_t i = 0u; i < got; ++i) {
        m->loop_buf[m->load_done + i].l = reich_clip_guard(m->read_stage[i * 2u]);
        m->loop_buf[m->load_done + i].r = reich_clip_guard(m->read_stage[i * 2u + 1u]);
    }
    m->load_done += got;

    if (m->load_done >= m->load_total) {
        rain_prepare_fresh_loop(m, m->load_done);
        m->state = ST_PLAY;
        m->buf_locked = false;
        rain_sync_engine_active(m);
        rain_hints(m);
        ui_notify_toast(NotifySuccess, "Loaded in 0 deg Unison", 1800u);
    }
}

/* ----------------------------------------------------------------------------
 * Menu System (BTN3 Hold — "GONNA RAIN")
 * ------------------------------------------------------------------------- */
static void menu_act_load_take(ui_menu_t* menu, ui_menu_node_t* entry) {
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    const uint16_t idx = UI_PICK_IDX(entry);
    ui_menu_hide(menu);
    rain_start_take_load(m, (uint32_t)idx);
}

static void menu_takes_builder(ui_menu_t* menu) {
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    tape_t* t = tape_get();
    const uint32_t n = t ? tape_marks_count(t) : 0u;

    if (n == 0u) {
        ui_menu_add(menu, UI_ACTION("(no tape takes)", NULL));
        return;
    }

    const uint32_t count = (n > MAX_MENU_TAKES) ? MAX_MENU_TAKES : n;
    for (uint32_t i = 0u; i < count; ++i) {
        uint32_t s = 0u, e = 0u;
        uint32_t dur_ds = 0u;
        if (tape_marks_get(t, i, &s, &e) && e > s) {
            uint32_t frames = e - s;
            if (frames > m->max_frames) frames = m->max_frames;
            dur_ds = frames / 4800u;
        }
        snprintf(m->take_labels[i], sizeof(m->take_labels[i]), "Take %u (%u.%us)",
                 (unsigned)(i + 1u), (unsigned)(dur_ds / 10u), (unsigned)(dur_ds % 10u));
        ui_menu_add(menu, UI_PICK(m->take_labels[i], menu_act_load_take, i));
    }
}

static void menu_act_realign(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->realign_req = true;
    ui_menu_hide(menu);
}

static void menu_act_snap_canon(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->snap_req = true;
    ui_menu_hide(menu);
}

static void menu_act_jump90(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->jump90_req = true;
    ui_menu_hide(menu);
}

static void menu_act_toggle_decks(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    const uint8_t next = (m->dsp.num_decks >= 4u) ? 2u : 4u;
    reich_set_decks(&m->dsp, next);
    ui_menu_rebuild(menu);
}

static void menu_act_toggle_motion(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    const uint8_t next = (m->dsp.motion_mode == REICH_MOTION_CANON_DWELL)
                             ? REICH_MOTION_CONTINUOUS
                             : REICH_MOTION_CANON_DWELL;
    reich_set_motion_mode(&m->dsp, next);
    ui_menu_rebuild(menu);
}

static void menu_act_cycle_crop(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->crop_mode = (uint8_t)((m->crop_mode + 1u) % CROP_COUNT);
    if (m->loop_len >= 8u) {
        uint32_t wstart = 0u, wlen = m->loop_len;
        compute_crop_window(m->loop_len, m->crop_mode, &wstart, &wlen);
        reich_set_window(&m->dsp, m->loop_len, wstart, wlen);
        rebuild_waveform_bins(m);
    }
    ui_menu_rebuild(menu);
}

static void menu_act_flip_dir(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->speed_target = -m->speed_target;
    if (fabsf(m->speed_target) < 0.1f) m->speed_target = -1.0f;
    ui_menu_hide(menu);
}

static void menu_act_speed_1x(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 1.0f;
    ui_menu_hide(menu);
}

static void menu_act_speed_half(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 0.5f;
    ui_menu_hide(menu);
}

static void menu_act_speed_double(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 2.0f;
    ui_menu_hide(menu);
}

static void menu_act_input(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_switch_input();
    m->mon_req = false;
    rain_tame_hot_mic_input();
    rain_sync_monitor(m);
    ui_menu_rebuild(menu);
}

static void menu_act_clear(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->clear_req = true;
    ui_menu_hide(menu);
}

static void menu_root_builder(ui_menu_t* menu) {
    rain_model_t* m = (rain_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_get_input();

    snprintf(m->decks_label, sizeof(m->decks_label), "Decks: %s",
             (m->dsp.num_decks >= 4u) ? "4 (Part II)" : "2 (Part I)");
    snprintf(m->motion_label, sizeof(m->motion_label), "Motion: %s",
             (m->dsp.motion_mode == REICH_MOTION_CANON_DWELL) ? "Canon Dwell" : "Continuous");
    snprintf(m->crop_label, sizeof(m->crop_label), "Crop: %s",
             crop_mode_name(m->crop_mode));
    snprintf(m->dir_label, sizeof(m->dir_label), "Direction: %s",
             (m->speed_target < 0.0f) ? "REV -> FWD" : "FWD -> REV");
    snprintf(m->input_label, sizeof(m->input_label), "Input: %s",
             (m->in_src == 1u) ? "Mic" : "Line");

    ui_menu_add(menu, UI_ENTRY("Load Tape Take", menu_takes_builder));
    ui_menu_add(menu, UI_ACTION("Re-Align Unison (0 deg)", menu_act_realign));
    ui_menu_add(menu, UI_ACTION("Snap Nearest Canon", menu_act_snap_canon));
    ui_menu_add(menu, UI_ACTION("Jump Phase: +90 deg", menu_act_jump90));
    ui_menu_add(menu, UI_ACTION(m->decks_label, menu_act_toggle_decks));
    ui_menu_add(menu, UI_ACTION(m->motion_label, menu_act_toggle_motion));
    ui_menu_add(menu, UI_ACTION(m->crop_label, menu_act_cycle_crop));
    ui_menu_add(menu, UI_ACTION(m->dir_label, menu_act_flip_dir));
    ui_menu_add(menu, UI_ACTION("Speed: 1.00x", menu_act_speed_1x));
    ui_menu_add(menu, UI_ACTION("Speed: 0.50x (Half)", menu_act_speed_half));
    ui_menu_add(menu, UI_ACTION("Speed: 2.00x (Double)", menu_act_speed_double));
    ui_menu_add(menu, UI_ACTION(m->input_label, menu_act_input));
    ui_menu_add(menu, UI_PARAM("Mic Trim", MenuNodeParam, &m->mic_trim));
    ui_menu_add(menu, UI_ACTION("Clear Loop", menu_act_clear));
}

/* ----------------------------------------------------------------------------
 * Hint Band & Volume Overlay
 * ------------------------------------------------------------------------- */
static void rain_hints(const rain_model_t* m) {
    const bool playing = (m->state == ST_PLAY);
    const bool rec = (m->state == ST_REC);
    const tapp_hint_cell_t hints[HintsActionsTotal] = {
        [HintPress1] = {playing ? "stop" : "play",
                        gfx_icon(playing ? GfxIconPause : GfxIconPlay), NULL,
                        playing ? HINT_ACTIVE : 0u},
        [HintHold1]  = {"vol", gfx_icon(GfxIconVolume)},
        [HintPress2] = {rec ? "loop" : "rec", gfx_icon(GfxIconRec), NULL,
                        rec ? HINT_ACTIVE : 0u},
        [HintHold2]  = {"bounce", gfx_icon(GfxIconSos), NULL,
                        m->bouncing ? HINT_ACTIVE : 0u},
        [HintPress3] = {m->drifting ? "phase" : "lock",
                        gfx_icon(m->drifting ? GfxIconSpeed : GfxIconChain), NULL,
                        m->drifting ? HINT_ACTIVE : 0u},
        [HintHold3]  = {"menu", gfx_icon(GfxIconMenu)},
        [HintPress4] = {NULL, gfx_icon(GfxIconPlayRev)},
        [HintPress5] = {NULL, gfx_icon(GfxIconPlay)},
    };
    ui_hints_set(hints);
}

static void rain_vol_show(rain_model_t* m) {
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

static void rain_vol_hide(rain_model_t* m) {
    m->vol_ui = false;
    rain_hints(m);
}

/* ----------------------------------------------------------------------------
 * Drawing (400x240 1-bit monochrome OLED, content y = 48..240)
 * ------------------------------------------------------------------------- */
#define DIAL_BOX_X    8
#define DIAL_BOX_Y    74
#define DIAL_BOX_W    98
#define DIAL_BOX_H    110
#define DIAL_CX       57
#define DIAL_CY       129
#define DIAL_R        40

#define STAGE_X       110
#define STAGE_Y       74
#define STAGE_W       282
#define STAGE_H       110
#define TRACK_X       117
#define TRACK_W       268     /* 134 bins * 2px */
#define TRACK_MID_Y   120
#define TRACK_HALF_H  38
#define FOOTER_Y      168

#define CARD_Y        190
#define CARD_H        42
#define CARD_W        93
#define CARD_GAP      4

/* Crisp 3x3 pixel degree symbol at (x, y_top) — avoids extended-ASCII font dependency */
static void draw_deg_ring(gfx_t* gfx, int x, int y_top) {
    gfx_draw_hline(gfx, x + 1, y_top, 2);
    gfx_draw_hline(gfx, x + 1, y_top + 3, 2);
    gfx_draw_vline(gfx, x,     y_top + 1, 2);
    gfx_draw_vline(gfx, x + 3, y_top + 1, 2);
}

/* Convert Q16 angle (0 = 12 o'clock, 16384 = 3 o'clock, clockwise) to screen (x, y) */
static void polar_cw(int cx, int cy, int r, uint16_t q16_turn, int* out_x, int* out_y) {
    const int sx = (int)lutsin_q15(q16_turn);
    const int cy_cos = (int)lutcos_q15(q16_turn);
    *out_x = cx + ((sx * r + (sx >= 0 ? 16384 : -16384)) >> 15);
    *out_y = cy - ((cy_cos * r + (cy_cos >= 0 ? 16384 : -16384)) >> 15);
}

static void draw_header(gfx_t* gfx, const rain_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);

    const float deg = reich_get_phase_deg(&m->dsp);
    const bool near_unison = (deg < 1.8f || deg > 358.2f);

    const char* badge = "READY";
    bool inv_badge = false;
    if (m->state == ST_REC) {
        badge = "CAPTURE";
        inv_badge = (((os_tick_get() / 300u) & 1u) == 0u);
    } else if (m->state == ST_LOADING) {
        badge = "LOADING";
        inv_badge = true;
    } else if (m->state == ST_PLAY) {
        if (m->unison_intro || near_unison) {
            badge = "UNISON";
            inv_badge = true;
        } else if (!m->drifting) {
            badge = "LOCKED";
        } else if (m->dsp.is_dwelling) {
            badge = "DWELL";
            inv_badge = true;
        } else {
            badge = "PHASING";
            inv_badge = true;
        }
    } else if (m->loop_len == 0u) {
        badge = "EMPTY";
    }

    gfx_set_font(gfx, DepartureMono_Regular_10);
    const int bw = (int)gfx_get_str_width(gfx, badge) + 10;
    if (inv_badge) {
        gfx_draw_rect_fill_r(gfx, 8, 53, bw, 16, 2);
        gfx_set_color(gfx, GFX_COLOR_CLEAR);
        gfx_draw_str(gfx, 13, 64, badge);
        gfx_set_color(gfx, GFX_COLOR_SET);
    } else {
        gfx_draw_rect_r(gfx, 8, 53, bw, 16, 2);
        gfx_draw_str(gfx, 13, 64, badge);
    }

    int tx = 8 + bw + 8;

    /* Bounce indicator pill if actively recording phase performance to tape */
    if (m->bouncing) {
        const uint32_t bsec = (m->bounce_frame - m->bounce_start) / TAPP_SAMPLE_RATE;
        char btxt[16];
        snprintf(btxt, sizeof(btxt), "REC %02u:%02u",
                 (unsigned)(bsec / 60u), (unsigned)(bsec % 60u));
        const int rbw = (int)gfx_get_str_width(gfx, btxt) + 8;
        if (((os_tick_get() / 400u) & 1u) == 0u) {
            gfx_draw_rect_fill_r(gfx, tx, 53, rbw, 16, 2);
            gfx_set_color(gfx, GFX_COLOR_CLEAR);
            gfx_draw_str(gfx, tx + 4, 64, btxt);
            gfx_set_color(gfx, GFX_COLOR_SET);
        } else {
            gfx_draw_rect_r(gfx, tx, 53, rbw, 16, 2);
            gfx_draw_str(gfx, tx + 4, 64, btxt);
        }
        tx += rbw + 8;
    }

    /* Loop window duration + LAP counter */
    if (m->state == ST_REC) {
        const uint32_t ds = m->rec_frames / 4800u;
        const uint32_t max_ds = m->max_frames / 4800u;
        gfx_draw_strf(gfx, tx, 64, "%u.%us / %u.%us",
                      (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                      (unsigned)(max_ds / 10u), (unsigned)(max_ds % 10u));
    } else if (m->loop_len >= 8u) {
        const uint32_t ds = m->dsp.win_len / 4800u;
        const uint32_t lap = (m->dsp.lap_count > 0u) ? m->dsp.lap_count : 1u;
        if (fabsf(m->speed_target - 1.0f) > 0.05f) {
            const char sgn = (m->speed_target < 0.0f) ? '-' : '+';
            const int sp10 = (int)(fabsf(m->speed_target) * 10.0f + 0.5f);
            gfx_draw_strf(gfx, tx, 64, "%u.%us L%u (%c%d.%dx)",
                          (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                          (unsigned)lap, sgn, sp10 / 10, sp10 % 10);
        } else {
            gfx_draw_strf(gfx, tx, 64, "%u.%us  LAP %u",
                          (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                          (unsigned)lap);
        }
    } else {
        const uint32_t max_s = m->max_frames / TAPP_SAMPLE_RATE;
        gfx_draw_strf(gfx, tx, 64, "MAX %us RAM", (unsigned)max_s);
    }

    /* Right-aligned deck badge (2H / 4H) + live phase angle with custom degree ring */
    if (m->state != ST_REC && m->state != ST_LOADING) {
        const int deg10 = (int)(deg * 10.0f + 0.5f);
        const int whole = (deg10 >= 3600) ? 0 : (deg10 / 10);
        const int tenths = (deg10 >= 3600) ? 0 : (deg10 % 10);
        char ptxt[24];
        snprintf(ptxt, sizeof(ptxt), "%uH %3d.%d",
                 (unsigned)m->dsp.num_decks, whole, tenths);
        const int pw = (int)gfx_get_str_width(gfx, ptxt);
        const int px = 386 - pw;
        gfx_draw_str(gfx, px, 64, ptxt);
        draw_deg_ring(gfx, px + pw + 2, 55);
    }
}

static void draw_phase_dial(gfx_t* gfx, const rain_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_rect_r(gfx, DIAL_BOX_X, DIAL_BOX_Y, DIAL_BOX_W, DIAL_BOX_H, 3);

    /* Corner annotations: Part I / II Roman numeral + completed 360-deg cycle count */
    gfx_set_font(gfx, DepartureMono_Regular_10);
    gfx_draw_str(gfx, DIAL_BOX_X + 5, DIAL_BOX_Y + 12,
                 (m->dsp.num_decks >= 4u) ? "II" : "I");
    if (m->loop_len >= 8u && m->dsp.phase_cycles > 0u) {
        char cstr[10];
        snprintf(cstr, sizeof(cstr), "x%u", (unsigned)m->dsp.phase_cycles);
        const int cw = (int)gfx_get_str_width(gfx, cstr);
        gfx_draw_str(gfx, DIAL_BOX_X + DIAL_BOX_W - 4 - cw, DIAL_BOX_Y + 12, cstr);
    }

    /* Outer phase ring + 8 rational canon ticks (0, 45, 90, 135, 180, 225, 270, 315 deg) */
    gfx_draw_circle(gfx, DIAL_CX, DIAL_CY, DIAL_R, GFX_DRAW_ALL);
    for (uint32_t k = 0u; k < 8u; ++k) {
        const uint16_t ang = (uint16_t)(k * 8192u);
        const int inner_r = ((k & 1u) == 0u) ? (DIAL_R - 4) : (DIAL_R - 2);
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
        polar_cw(DIAL_CX, DIAL_CY, inner_r, ang, &x0, &y0);
        polar_cw(DIAL_CX, DIAL_CY, DIAL_R, ang, &x1, &y1);
        gfx_draw_line(gfx, x0, y0, x1, y1);
    }

    /* Rotating Wollensak supply reel hub driven by Deck A's live rotation */
    gfx_draw_circle(gfx, DIAL_CX, DIAL_CY, 9, GFX_DRAW_ALL);
    gfx_draw_disc(gfx, DIAL_CX, DIAL_CY, 2, GFX_DRAW_ALL);

    uint16_t reel_ang = 0u;
    if (m->loop_len >= 8u && m->dsp.win_len >= 8u) {
        const float a_norm = (float)m->dsp.head_a_i * m->dsp.inv_win_len;
        reel_ang = (uint16_t)(a_norm * 65536.0f);
    }
    for (uint32_t sp = 0u; sp < 3u; ++sp) {
        const uint16_t sa = (uint16_t)(reel_ang + sp * 21845u);
        int sx0 = 0, sy0 = 0, sx1 = 0, sy1 = 0;
        polar_cw(DIAL_CX, DIAL_CY, 3, sa, &sx0, &sy0);
        polar_cw(DIAL_CX, DIAL_CY, 8, sa, &sx1, &sy1);
        gfx_draw_line(gfx, sx0, sy0, sx1, sy1);
    }

    /* 12 o'clock Deck A reference marker bead */
    gfx_draw_disc(gfx, DIAL_CX, DIAL_CY - DIAL_R, 2, GFX_DRAW_ALL);

    /* Phase sweep arc + orbital head arms (Deck B, plus Heads C & D in Part II mode) */
    const float turns = (m->loop_len >= 8u)
                            ? reich_get_phase_turns(&m->dsp)
                            : clampf(m->prm[ENC_PHASE].val * (1.0f / 360.0f), 0.0f, 1.0f);
    const uint16_t ph_q16 = (uint16_t)(turns * 65536.0f);

    /* Dotted phase-lead arc at r = 27 from 12 o'clock clockwise to Deck B */
    const uint32_t arc_dots = (uint32_t)(turns * 32.0f);
    for (uint32_t d = 1u; d <= arc_dots; ++d) {
        const uint16_t da = (uint16_t)((d * 65536u) / 32u);
        int ax = 0, ay = 0;
        polar_cw(DIAL_CX, DIAL_CY, 27, da, &ax, &ay);
        gfx_draw_pixel(gfx, ax, ay);
    }

    /* Part II secondary voices (Heads C = 2x, D = 3x) */
    if (m->dsp.num_decks >= 4u) {
        for (uint32_t v = 2u; v <= 3u; ++v) {
            const uint16_t va = (uint16_t)(ph_q16 * v);
            int vx0 = 0, vy0 = 0, vx1 = 0, vy1 = 0;
            polar_cw(DIAL_CX, DIAL_CY, 10, va, &vx0, &vy0);
            polar_cw(DIAL_CX, DIAL_CY, DIAL_R - 2, va, &vx1, &vy1);
            gfx_draw_line(gfx, vx0, vy0, vx1, vy1);
            gfx_draw_disc(gfx, vx1, vy1, 2, GFX_DRAW_ALL);
        }
    }

    /* Primary Deck B phase arm (thick needle + rim disc) */
    {
        int bx0 = 0, by0 = 0, bx1 = 0, by1 = 0;
        polar_cw(DIAL_CX, DIAL_CY, 10, ph_q16, &bx0, &by0);
        polar_cw(DIAL_CX, DIAL_CY, DIAL_R - 1, ph_q16, &bx1, &by1);
        gfx_draw_line_thick(gfx, bx0, by0, bx1, by1, 2);
        gfx_draw_disc(gfx, bx1, by1, 3, GFX_DRAW_ALL);
    }
}

static void draw_footer_strip(gfx_t* gfx, const rain_model_t* m) {
    gfx_draw_hline_dithered(gfx, STAGE_X + 3, FOOTER_Y - 2, STAGE_W - 6, 2);
    gfx_set_font(gfx, DepartureMono_Regular_10);

    if (m->loop_len >= 8u) {
        const char* zone = reich_describe_zone(&m->dsp);
        const int ms = (int)(reich_get_offset_ms(&m->dsp) + 0.5f);
        gfx_draw_strf(gfx, STAGE_X + 6, FOOTER_Y + 11, "%s +%dms", zone, ms);
    } else {
        const char* mode_s = (m->dsp.motion_mode == REICH_MOTION_CANON_DWELL)
                                 ? "CANON DWELL"
                                 : "WOLLENSAK 1965";
        gfx_draw_str(gfx, STAGE_X + 6, FOOTER_Y + 11, mode_s);
    }

    /* Twin L / R stereo hocket peak meters on right of footer */
    const int mx = STAGE_X + STAGE_W - 52;
    const int mw = 44;
    gfx_draw_rect(gfx, mx, FOOTER_Y + 2, mw, 4);
    gfx_draw_rect(gfx, mx, FOOTER_Y + 8, mw, 4);
    const int lw = (int)(clampf(m->peak_l, 0.0f, 1.0f) * (float)(mw - 2) + 0.5f);
    const int rw = (int)(clampf(m->peak_r, 0.0f, 1.0f) * (float)(mw - 2) + 0.5f);
    if (lw > 0) gfx_draw_hline(gfx, mx + 1, FOOTER_Y + 3, lw);
    if (rw > 0) gfx_draw_hline(gfx, mx + 1, FOOTER_Y + 9, rw);
}

static void draw_waveform_stage(gfx_t* gfx, const rain_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_rect_r(gfx, STAGE_X, STAGE_Y, STAGE_W, STAGE_H, 3);

    /* Loading progress view */
    if (m->state == ST_LOADING) {
        gfx_set_font(gfx, gfx_nunito_semibold_14);
        const char* msg = "LOADING LOOP IN UNISON...";
        const int mw = (int)gfx_get_str_width(gfx, msg);
        gfx_draw_str(gfx, STAGE_X + (STAGE_W - mw) / 2, STAGE_Y + 46, msg);

        const float pct = (m->load_total > 0u)
                              ? ((float)m->load_done / (float)m->load_total)
                              : 0.0f;
        gfx_draw_rect_r(gfx, STAGE_X + 24, STAGE_Y + 58, STAGE_W - 48, 14, 3);
        const int fw = (int)(clampf(pct, 0.0f, 1.0f) * (float)(STAGE_W - 52) + 0.5f);
        if (fw > 0) gfx_draw_rect_fill_r(gfx, STAGE_X + 26, STAGE_Y + 60, fw, 10, 2);
        return;
    }

    /* Live capture view */
    if (m->state == ST_REC) {
        const float pct = (m->max_frames > 0u)
                              ? ((float)m->rec_frames / (float)m->max_frames)
                              : 0.0f;
        const int fill_w = (int)(clampf(pct, 0.0f, 1.0f) * (float)(STAGE_W - 8));
        if (fill_w > 0) {
            gfx_fill_rect_dithered(gfx, STAGE_X + 4, STAGE_Y + 4, fill_w, STAGE_H - 8, 2);
        }

        const float pk = (m->peak_l > m->peak_r) ? m->peak_l : m->peak_r;
        const int mh = (int)(clampf(pk, 0.0f, 1.0f) * (float)TRACK_HALF_H);
        gfx_draw_hline_dithered(gfx, TRACK_X, TRACK_MID_Y, TRACK_W, 2);
        if (mh > 0 && fill_w > 2) {
            gfx_draw_rect_fill(gfx, STAGE_X + 4 + fill_w - 3, TRACK_MID_Y - mh, 3, mh * 2 + 1);
        }

        gfx_set_font(gfx, DepartureMono_Regular_10);
        const char* cap = "SPEAK PHRASE  [PRESS LOOP]";
        const int cw = (int)gfx_get_str_width(gfx, cap);
        gfx_draw_str(gfx, STAGE_X + (STAGE_W - cw) / 2, STAGE_Y + STAGE_H - 10, cap);
        return;
    }

    /* Empty state: twin phase-shifted Wollensak waves centered vertically */
    if (m->loop_len < 8u) {
        gfx_draw_hline_dithered(gfx, TRACK_X, TRACK_MID_Y, TRACK_W, 1);

        /* Two phase-shifted sine bursts illustrating Deck A + Deck B phasing */
        for (int i = 0; i < TRACK_W; i += 2) {
            const uint16_t pa = (uint16_t)((i * 65536) / 68);
            const uint16_t pb = (uint16_t)(((i - 22) * 65536) / 68);
            const int ya = ((int)lutsin_q15(pa) * 14) >> 15;
            const int yb = ((int)lutsin_q15(pb) * 14) >> 15;
            gfx_draw_pixel(gfx, TRACK_X + i, TRACK_MID_Y - 6 - ya);
            if ((i & 2) == 0) {
                gfx_draw_pixel(gfx, TRACK_X + i, TRACK_MID_Y + 6 - yb);
            }
        }

        draw_footer_strip(gfx, m);
        return;
    }

    /* Active loop waveform */
    gfx_draw_hline_dithered(gfx, TRACK_X, TRACK_MID_Y, TRACK_W, 1);
    for (uint32_t b = 0u; b < WAVE_BINS; ++b) {
        const int h = ((int)m->wave_bins[b] * TRACK_HALF_H) / 255;
        if (h > 0) {
            const int x = TRACK_X + (int)(b * 2u);
            gfx_draw_vline(gfx, x, TRACK_MID_Y - h, h * 2 + 1);
        }
    }

    const float inv_w = m->dsp.inv_win_len;
    const int xa = TRACK_X + (int)(clampf((float)m->dsp.voice_pos_i[0] * inv_w, 0.0f, 0.9999f) *
                                   (float)(TRACK_W - 1));
    const int xb = TRACK_X + (int)(clampf((float)m->dsp.voice_pos_i[1] * inv_w, 0.0f, 0.9999f) *
                                   (float)(TRACK_W - 1));

    /* Part II secondary playheads (Heads C & D) */
    if (m->dsp.num_decks >= 4u) {
        for (uint32_t v = 2u; v <= 3u; ++v) {
            const int xv = TRACK_X + (int)(clampf((float)m->dsp.voice_pos_i[v] * inv_w, 0.0f, 0.9999f) *
                                           (float)(TRACK_W - 1));
            gfx_draw_vline_dithered(gfx, xv, STAGE_Y + 4, FOOTER_Y - STAGE_Y - 8, 4);
        }
    }

    /* Phase-offset tether bar connecting Deck A -> Deck B along y = FOOTER_Y - 4 */
    const int ty = FOOTER_Y - 4;
    if (xb >= xa) {
        if (xb > xa) gfx_draw_hline(gfx, xa, ty, xb - xa + 1);
    } else {
        gfx_draw_hline(gfx, xa, ty, (TRACK_X + TRACK_W) - xa);
        gfx_draw_hline(gfx, TRACK_X, ty, (xb - TRACK_X) + 1);
    }

    /* Deck B Playhead: crisp dashed vertical cursor + upward triangle marker at bottom */
    gfx_set_color(gfx, GFX_COLOR_XOR);
    for (int y = STAGE_Y + 4; y < FOOTER_Y - 5; y += 4) {
        gfx_draw_vline(gfx, xb, y, 2);
    }
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_line(gfx, xb - 3, FOOTER_Y - 3, xb, FOOTER_Y - 6);
    gfx_draw_line(gfx, xb + 3, FOOTER_Y - 3, xb, FOOTER_Y - 6);
    gfx_draw_hline(gfx, xb - 3, FOOTER_Y - 3, 7);

    /* Deck A Playhead: solid 2px XOR cursor + downward triangle tab at top */
    gfx_set_color(gfx, GFX_COLOR_XOR);
    gfx_draw_vline(gfx, xa, STAGE_Y + 4, FOOTER_Y - STAGE_Y - 8);
    gfx_draw_vline(gfx, xa + 1, STAGE_Y + 4, FOOTER_Y - STAGE_Y - 8);
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_hline(gfx, xa - 3, STAGE_Y + 2, 7);
    gfx_draw_line(gfx, xa - 3, STAGE_Y + 2, xa, STAGE_Y + 5);
    gfx_draw_line(gfx, xa + 3, STAGE_Y + 2, xa, STAGE_Y + 5);

    draw_footer_strip(gfx, m);
}

static void draw_param_cards(gfx_t* gfx, const rain_model_t* m) {
    const float live_deg = (m->loop_len >= 8u)
                               ? reich_get_phase_deg(&m->dsp)
                               : m->prm[ENC_PHASE].val;

    for (uint8_t idx = 0u; idx < ENC_TOTAL; ++idx) {
        const int x = 8 + (int)idx * (CARD_W + CARD_GAP);
        const bool sel = (m->enc_sel == idx);
        const params_t* p = &m->prm[idx];

        gfx_set_color(gfx, GFX_COLOR_SET);
        ui_draw_frame(gfx, x, CARD_Y, CARD_W, CARD_H, 3, sel);

        gfx_set_font(gfx, DepartureMono_Regular_10);
        gfx_draw_str(gfx, x + 6, CARD_Y + 14, p->name);

        char val_s[16];
        if (idx == ENC_PHASE) {
            const int d10 = (int)(live_deg * 10.0f + 0.5f);
            const int wh = (d10 >= 3600) ? 0 : (d10 / 10);
            snprintf(val_s, sizeof(val_s), "%d", wh);
            const int vw = (int)gfx_get_str_width(gfx, val_s);
            const int vx = x + CARD_W - 11 - vw;
            gfx_draw_str(gfx, vx, CARD_Y + 14, val_s);
            draw_deg_ring(gfx, vx + vw + 1, CARD_Y + 5);
        } else {
            const int pc = (int)(p->val * 100.0f + 0.5f);
            snprintf(val_s, sizeof(val_s), "%d%%", pc);
            const int vw = (int)gfx_get_str_width(gfx, val_s);
            gfx_draw_str(gfx, x + CARD_W - 6 - vw, CARD_Y + 14, val_s);
        }

        const int bx = x + 6;
        const int by = CARD_Y + 23;
        const int bw = CARD_W - 12;
        const int bh = 12;
        gfx_draw_rect_r(gfx, bx, by, bw, bh, 2);

        if (idx == ENC_PHASE) {
            /* Mark 90, 180, 270 deg canon ticks inside the PHASE bar */
            for (int q = 1; q <= 3; ++q) {
                const int qx = bx + 2 + ((bw - 4) * q) / 4;
                gfx_draw_vline_dithered(gfx, qx, by + 1, bh - 2, 3);
            }
            const float norm = clampf(live_deg * (1.0f / 360.0f), 0.0f, 1.0f);
            const int fill = (int)(norm * (float)(bw - 4) + 0.5f);
            if (fill > 0) {
                gfx_draw_rect_fill(gfx, bx + 2, by + 2, fill, bh - 4);
            }
        } else {
            const int fill = (int)(clampf(p->val, 0.0f, 1.0f) * (float)(bw - 4) + 0.5f);
            if (fill > 0) {
                gfx_draw_rect_fill(gfx, bx + 2, by + 2, fill, bh - 4);
            }
        }
    }
}

static void rain_redraw(gfx_t* gfx, const os_app_t* app) {
    const rain_model_t* m = (const rain_model_t*)os_app_get_model(app);
    if (!m) return;

    draw_header(gfx, m);
    draw_phase_dial(gfx, m);
    draw_waveform_stage(gfx, m);
    draw_param_cards(gfx, m);
}

/* ----------------------------------------------------------------------------
 * Tick (main UI thread — safe for tape_read, menu commands & UI sync)
 * ------------------------------------------------------------------------- */
static bool rain_tick(os_app_t* app) {
    rain_model_t* m = (rain_model_t*)os_app_get_model(app);
    tape_t* t = tape_get();

    if (m->tape_swapped) {
        m->tape_swapped = false;
        if (m->bouncing) {
            m->bouncing = false;
            m->bounce_tok = 0u;
        }
        if (m->wb) tape_buffer_clear_all_buffers(m->wb);
        m->bounce_cur_win = WIN_NONE;
        rain_hints(m);
    }

    if (m->realign_req) {
        m->realign_req = false;
        m->unison_intro = false;
        if (m->loop_len >= 8u) {
            reich_slew_to_phase_deg(&m->dsp, 0.0f);
            ui_notify_toast(NotifyInfo, "Slewing to 0 deg Unison", 1500u);
        } else {
            m->prm[ENC_PHASE].val = m->prm[ENC_PHASE].target = 0.0f;
        }
    }

    if (m->snap_req) {
        m->snap_req = false;
        m->unison_intro = false;
        if (m->loop_len >= 8u) {
            const char* lbl = NULL;
            (void)reich_snap_nearest_canon(&m->dsp, &lbl);
            if (lbl) {
                ui_notify_toast(NotifyInfo, lbl, 1500u);
            }
        }
    }

    if (m->jump90_req) {
        m->jump90_req = false;
        m->unison_intro = false;
        if (m->loop_len >= 8u) {
            reich_nudge_phase_deg(&m->dsp, 90.0f);
            ui_notify_toast(NotifyInfo, "Phase +90 deg", 1400u);
        } else {
            float nv = fmodf(m->prm[ENC_PHASE].val + 90.0f, 360.0f);
            if (nv < 0.0f) nv += 360.0f;
            m->prm[ENC_PHASE].val = m->prm[ENC_PHASE].target = nv;
        }
    }

    if (m->clear_req) {
        m->clear_req = false;
        rain_do_clear(m);
    }

    if (m->state == ST_LOADING) {
        rain_step_take_load(m);
        return true;
    }

    if (m->state == ST_REC && m->rec_done_req) {
        rain_finish_capture(m);
    }

    if (m->bouncing && t) {
        if (!m->bounce_stop_req && m->bounce_frame + BOUNCE_FADE_FR >= m->bounce_limit) {
            m->bounce_stop_req = true;
        }
        if ((m->bounce_stop_req && m->bounce_gain <= 0.001f) ||
            m->bounce_frame >= m->bounce_limit) {
            rain_bounce_stop(m, t);
        }
    }

    m->in_src = os_audio_get_input();
    rain_sync_monitor(m);
    rain_sync_engine_active(m);

    /* Encoder input (only when neither volume overlay nor modal menu is open) */
    if (!m->vol_ui && !(m->menu && ui_menu_is_visible(m->menu))) {
        const int32_t d = os_controls_encoder_get_delta();
        if (d != 0) {
            if (m->enc_sel == ENC_PHASE) {
                /* Velocity-scaled "thumb-on-reel-flange" phase nudge */
                m->unison_intro = false;
                const float step_deg = (abs((int)d) <= 1) ? 1.0f : 2.5f;
                const float delta_deg = (float)d * step_deg;
                if (m->loop_len >= 8u) {
                    if (m->state == ST_PLAY) {
                        reich_nudge_phase_deg(&m->dsp, delta_deg);
                    } else {
                        reich_set_phase_deg_immediate(&m->dsp,
                            reich_get_phase_deg(&m->dsp) + delta_deg);
                    }
                } else {
                    float nv = fmodf(m->prm[ENC_PHASE].val + delta_deg, 360.0f);
                    if (nv < 0.0f) nv += 360.0f;
                    m->prm[ENC_PHASE].val = m->prm[ENC_PHASE].target = nv;
                }
            } else {
                params_t* p = &m->prm[m->enc_sel];
                param_write_delta_val(p, d);
                param_update_fast(p, m);
                sync_params_to_dsp(m);
            }
        }
    }

    /* Keep PHASE parameter card mirrored to live capstan phase angle */
    if (m->loop_len >= 8u) {
        const float live = reich_get_phase_deg(&m->dsp);
        m->prm[ENC_PHASE].val = m->prm[ENC_PHASE].target = live;
    }

    if (!m->vol_ui && (m->hint_state != m->state ||
                       m->hint_drift != m->drifting ||
                       m->hint_bounce != m->bouncing)) {
        m->hint_state = m->state;
        m->hint_drift = m->drifting;
        m->hint_bounce = m->bouncing;
        rain_hints(m);
    }

    const uint32_t now_ms = os_tick_get();
    const uint32_t frame_ms = m->vol_ui ? 80u : 33u;
    if (m->last_draw_ms == 0u || (now_ms - m->last_draw_ms >= frame_ms)) {
        m->last_draw_ms = (now_ms != 0u) ? now_ms : 1u;
        return true;
    }
    return false;
}

/* ----------------------------------------------------------------------------
 * Input Handler
 * ------------------------------------------------------------------------- */
static void rain_input(os_app_t* app, uint8_t btn, KeyStateEnum st) {
    rain_model_t* m = (rain_model_t*)os_app_get_model(app);
    if (btn == BTN_ENCODER) return;

    if (m->vol_ui) {
        if (st == KEY_STATE_RELEASED && btn == BTN1) rain_vol_hide(m);
        return;
    }

    if (st == KEY_STATE_RELEASED && btn == BTN2 && m->rec_btn_down) {
        m->rec_btn_down = false;
        m->rec_holdoff_fr = MIC_START_SETTLE_FR;
        return;
    }

    if (m->state == ST_LOADING) return;

    if (st == KEY_STATE_PRESSED) {
        switch (btn) {
        case BTN1:
            if (m->state == ST_PLAY) {
                m->state = ST_STOP;
                rain_sync_monitor(m);
                rain_sync_engine_active(m);
            } else if (m->state == ST_STOP && m->loop_len >= 8u) {
                m->state = ST_PLAY;
                rain_sync_monitor(m);
                rain_sync_engine_active(m);
            }
            break;

        case BTN2:
            if (m->state == ST_REC) {
                m->rec_btn_down = false;
                m->rec_holdoff_fr = 0u;
                m->rec_manual_stop = true;
                m->rec_done_req = true;
                rain_sync_monitor(m);
            } else {
                m->buf_locked = true;
                m->rec_frames = 0u;
                m->rec_done_req = false;
                m->rec_manual_stop = false;
                m->rec_held_fr = 0u;
                m->rec_hp_xl = m->rec_hp_yl = 0.0f;
                m->rec_hp_xr = m->rec_hp_yr = 0.0f;
                m->in_src = os_audio_get_input();
                if (m->in_src == 1u) {
                    m->rec_btn_down = true;
                    m->rec_holdoff_fr = MIC_START_SETTLE_FR;
                } else {
                    m->rec_btn_down = false;
                    m->rec_holdoff_fr = 0u;
                }
                m->state = ST_REC;
                m->buf_locked = false;
                rain_sync_monitor(m);
                rain_sync_engine_active(m);
            }
            break;

        case BTN3:
            if (m->unison_intro && m->drifting) {
                /* Pressing phase during opening unison lap starts phasing right away */
                m->unison_intro = false;
            } else {
                m->drifting = !m->drifting;
                reich_set_drifting(&m->dsp, m->drifting);
            }
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

        m->hint_state = m->state;
        m->hint_drift = m->drifting;
        m->hint_bounce = m->bouncing;
        rain_hints(m);
    } else if (st == KEY_STATE_HOLD) {
        if (btn == BTN1 && !m->vol_ui) {
            m->vol_ui = true;
            rain_vol_show(m);
        } else if (btn == BTN2 && m->state != ST_REC) {
            if (m->bouncing) {
                m->bounce_stop_req = true;
            } else {
                rain_bounce_start(m);
            }
        } else if (btn == BTN3) {
            if (m->menu) ui_menu_show(m->menu);
        }
    }
}

/* ----------------------------------------------------------------------------
 * Lifecycle (init / deinit)
 * ------------------------------------------------------------------------- */
static bool rain_app_init(os_app_t* app) {
    rain_model_t* m = (rain_model_t*)os_app_get_model(app);

    /*
     * 1. Initialize Reich DSP & all `params_t` fields FIRST so `ui_menu_create()`
     *    sees valid parameter descriptors when `menu_root_builder` runs.
     */
    reich_init(&m->dsp, (float)TAPP_SAMPLE_RATE);

    m->drifting = true;
    m->speed_cur = 1.0f;
    m->speed_target = 1.0f;

    /* 4 live encoder cards: DRIFT, PHASE, SPREAD, TAPE */
    m->prm[ENC_DRIFT]  = (params_t)PARAM_RANGE("DRIFT",  0.35f, 0.00f,   1.00f, 0.02f);
    m->prm[ENC_PHASE]  = (params_t)PARAM_RANGE("PHASE",  0.00f, 0.00f, 360.00f, 1.00f);
    m->prm[ENC_SPREAD] = (params_t)PARAM_RANGE("SPREAD", 0.70f, 0.00f,   1.00f, 0.02f);
    m->prm[ENC_TAPE]   = (params_t)PARAM_RANGE("TAPE",   0.28f, 0.00f,   1.00f, 0.02f);
    m->enc_sel = ENC_DRIFT;
    sync_params_to_dsp(m);

    /* Volume overlay (default 15% = 0.045f / -26.9 dB for headphones) & mic trim */
    m->vol_out = (params_t)PARAM_RANGE("out", 0.045f, 0.0f, 2.0f, 0.02f);
    m->vol_out.type = ParamValDBType;
    m->vol_fx = (params_t)PARAM_RANGE("fx send", 0.0f, 0.0f, 1.0f, 0.02f);
    m->vol_fx.type = ParamValDBType;
    m->mic_trim = (params_t)PARAM_RANGE("mic trim", 0.40f, 0.05f, 1.50f, 0.02f);
    m->mic_trim.type = ParamValDBType;
    m->vout_cur = 0.045f;
    m->vfx_cur  = 0.0f;

    /*
     * 2. Allocate staging buffer, DMA window pair, and modal menu BEFORE the
     *    large audio loop buffer so menu creation NEVER runs out of heap.
     */
    m->read_stage = (float*)os_malloc((size_t)LOAD_CHUNK_FR * 2u * sizeof(float));

    tape_t* t = tape_get();
    if (t) {
        m->wb = tape_buffer_init(t);
    }
    m->bounce_cur_win = WIN_NONE;

    m->menu = ui_menu_create(m, menu_root_builder);
    if (m->menu) {
        ui_menu_set_name(m->menu, "GONNA RAIN");
        ui_menu_set_header(m->menu, true);
    }

    /*
     * 3. Non-destructive dual/quad head loop buffer in PSRAM (single `os_malloc`
     *    without secondary guard probes so the web emulator's ~4.19 MB bump
     *    allocator never strands memory):
     *    - On 8 MB hardware: 15s (5.76 MB) succeeds, leaving ~1.2 MB free.
     *    - On 4.19 MB emulator: 20s/15s/12s return NULL cleanly and 8s (3.07 MB)
     *      succeeds on first fit, leaving ~1.0 MB untouched!
     */
    static const uint32_t kCandSecs[] = { 20u, 15u, 12u, 8u, 6u, 4u, 2u, 1u };
    const uint32_t n_cands = (uint32_t)(sizeof(kCandSecs) / sizeof(kCandSecs[0]));
    for (uint32_t c = 0u; c < n_cands; ++c) {
        const uint32_t frames = kCandSecs[c] * TAPP_SAMPLE_RATE;
        reich_frame_t* lb = (reich_frame_t*)os_malloc((size_t)frames * sizeof(reich_frame_t));
        if (lb) {
            m->loop_buf = lb;
            m->max_frames = frames;
            break;
        }
    }

    tape_timeline_show(false);

    /* Save user's monitor/WET state, silence idle speaker bleed, and tame hot mic gain */
    m->in_src = os_audio_get_input();
    m->mon_saved = os_audio_get_monitor();
    m->wet_saved = os_audio_get_wet();
    m->mon_req = false;
    rain_tame_hot_mic_input();
    (void)os_audio_set_monitor(false);
    (void)os_audio_set_wet(false);

    ui_statusbar_show(false);
    ui_hints_show(true);
    m->hint_state = m->state;
    m->hint_drift = m->drifting;
    m->hint_bounce = m->bouncing;
    rain_hints(m);

    os_led_set_pattern(&rain_led_pattern, m);
    m->eng_active = true;
    engine_set_active(true);
    return m->loop_buf != NULL;
}

static void rain_app_deinit(os_app_t* app) {
    rain_model_t* m = (rain_model_t*)os_app_get_model(app);
    if (!m) return;

    /* Restore user's WET and live-input monitor preferences on exit */
    (void)os_audio_set_wet(m->wet_saved);
    (void)os_audio_set_monitor(m->mon_saved);

    if (m->bouncing) {
        rain_bounce_stop(m, tape_get());
    }
    if (m->menu) {
        ui_menu_destroy(m->menu);
        m->menu = NULL;
    }

    tape_timeline_show(false);

    if (m->wb) {
        tape_buffer_deinit(m->wb);
        m->wb = NULL;
    }
    if (m->read_stage) {
        os_free(m->read_stage);
        m->read_stage = NULL;
    }
    if (m->loop_buf) {
        os_free(m->loop_buf);
        m->loop_buf = NULL;
    }
}

TAPP_APP("Gonna Rain", rain_model_t, rain_app_init, rain_app_deinit,
         .redraw = rain_redraw,
         .tick = rain_tick,
         .on_input = rain_input,
         .engine_cb = &rain_engine_cb);
