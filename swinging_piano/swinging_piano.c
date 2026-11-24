/*
 * swinging_piano.c — Aphex Twin Barbican Hall "Swinging Piano" Standalone TAPP
 *
 * Inspired by Richard D. James's October 2012 Barbican Hall "Remote Orchestra"
 * suspended swinging grand piano installation and the intimate room-captured
 * felt piano aesthetic of "aisatsana [102]".
 *
 * Suspends a stereo loop (live Mic/Line capture, SD tape take, or built-in
 * "Seed Felt Piano" generator) on a nonlinear gravitational 1D/2D pendulum rig
 * swinging over a stationary stereo hall microphone pair with Hermite cubic
 * fractional delay Doppler pitch bending, true Interaural Time Difference (ITD),
 * inverse-distance proximity swell, soundboard/air-absorption filtering,
 * opposite-wall early reflections, and a 4-line Householder FDN concert hall.
 *
 * Controls:
 *   BTN1 press : Play / Stop loop
 *   BTN1 hold  : Volume overlay (out / fx send / in / mic trim)
 *   BTN2 press : Record live input into PSRAM loop (monitored ONLY during ST_REC)
 *   BTN2 hold  : Bounce swinging hall output to free SD tape space (DMA windows)
 *   BTN3 press : Toggle Sustained Swing Drive (swing) <-> Free-Pendulum Coast (coast)
 *   BTN3 hold  : Menu (Load Tape Take, Seed Felt Piano, Push Piano, Center & Still,
 *                Rig Mode, Direction, Speed 1x/0.5x/2x, Input Line/Mic, Mic Trim, Clear)
 *   BTN4 / 5   : Previous / Next encoder parameter (ROPE, ARC, DOPPLER, HALL)
 *   ENCODER    : Adjust selected parameter
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tapp_api.h"
#include "pendulum_dsp.h"

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

#define WAVE_BINS        116u                       /* 116 * 2px = 232px in right panel */
#define TRAIL_LEN        6u
#define MAX_MENU_TAKES   32u

enum {
    ST_STOP = 0,
    ST_PLAY,
    ST_REC,
    ST_LOADING,
};

enum {
    ENC_ROPE = 0,
    ENC_ARC,
    ENC_DOPPLER,
    ENC_HALL,
    ENC_TOTAL,
};

/* ----------------------------------------------------------------------------
 * Model (~2.1 KB in internal SRAM; audio loop & hall space in PSRAM via os_malloc)
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Transport & thread-sync flags */
    volatile uint8_t  state;
    volatile bool     buf_locked;
    volatile bool     swing_on;       /* true = SWINGING drive, false = COASTING free decay */
    volatile bool     rec_done_req;
    volatile bool     tape_swapped;
    volatile bool     seed_req;
    volatile bool     push_req;
    volatile bool     center_req;
    volatile bool     clear_req;

    /* Audio engine active flag */
    volatile bool     eng_active;

    /* PSRAM stereo loop buffer + hall propagation/FDN workspace */
    swing_frame_t*    loop_buf;
    swing_space_t*    space;
    float*            read_stage;
    uint32_t          max_frames;
    volatile uint32_t loop_len;

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

    /* Loop playhead & smooth varispeed motor */
    volatile int32_t  head_i;
    float             head_f;
    float             speed_target;
    float             speed_cur;
    float             play_gain;

    /* Core Barbican Pendulum + Hall Acoustic DSP engine */
    pendulum_engine_t dsp;

    /* Chunked SD tape-take loader state */
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

    /* Metering, waveform envelope & pendulum motion trail */
    volatile float    peak_l;
    volatile float    peak_r;
    uint8_t           wave_bins[WAVE_BINS];
    int8_t            trail_deg[TRAIL_LEN];
    uint32_t          last_tick_ms;
    uint32_t          last_trail_ms;

    /* 4 live performance encoder parameters */
    params_t          prm[ENC_TOTAL];
    uint8_t           enc_sel;

    /* Volume overlay, mic trim & tape timeline params */
    params_t          vol_out;
    params_t          vol_fx;
    params_t          mic_trim;       /* app-local input trim (0.40f default when Mic) */
    volatile bool     vol_ui;
    bool              mon_saved;      /* user's monitor state saved at init, restored at exit */
    bool              wet_saved;      /* user's WET state saved at init, restored at exit */
    bool              mon_req;        /* true when monitor ON was requested for ST_REC */
    params_t          tl_pos;
    params_t          tl_start;
    params_t          tl_end;

    /* Menu & cached UI strings */
    ui_menu_t*        menu;
    uint8_t           in_src;
    uint8_t           hint_state;
    bool              hint_swing;
    bool              hint_bounce;
    char              take_labels[MAX_MENU_TAKES][24];
    char              rig_label[28];
    char              dir_label[24];
    char              input_label[20];
} swing_model_t;

/* Forward declarations */
static void swing_hints(const swing_model_t* m);
static void swing_finish_capture(swing_model_t* m);
static void swing_bounce_stop(swing_model_t* m, tape_t* t);
static void swing_sync_engine_active(swing_model_t* m);
static void swing_sync_monitor(swing_model_t* m);

/* ----------------------------------------------------------------------------
 * Loop Splice, Hermite Loop Reader & Waveform Bin Builder
 * ------------------------------------------------------------------------- */
static void apply_loop_splice(swing_frame_t* buf, uint32_t len) {
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

static inline swing_frame_t loop_read_hermite(const swing_frame_t* buf, int32_t len,
                                              int32_t i0, float frac) {
    const int32_t im1 = (i0 > 0) ? (i0 - 1) : (len - 1);
    const int32_t i1  = (i0 + 1 < len) ? (i0 + 1) : 0;
    const int32_t i2  = (i1 + 1 < len) ? (i1 + 1) : 0;

    const swing_frame_t ym1 = buf[im1];
    const swing_frame_t y0  = buf[i0];
    const swing_frame_t y1  = buf[i1];
    const swing_frame_t y2  = buf[i2];

    const float c1l = 0.5f * (y1.l - ym1.l);
    const float c2l = ym1.l - 2.5f * y0.l + 2.0f * y1.l - 0.5f * y2.l;
    const float c3l = 0.5f * (y2.l - ym1.l) + 1.5f * (y0.l - y1.l);

    const float c1r = 0.5f * (y1.r - ym1.r);
    const float c2r = ym1.r - 2.5f * y0.r + 2.0f * y1.r - 0.5f * y2.r;
    const float c3r = 0.5f * (y2.r - ym1.r) + 1.5f * (y0.r - y1.r);

    swing_frame_t out = {
        .l = ((c3l * frac + c2l) * frac + c1l) * frac + y0.l,
        .r = ((c3r * frac + c2r) * frac + c1r) * frac + y0.r,
    };
    return out;
}

static void rebuild_waveform_bins(swing_model_t* m) {
    const uint32_t len = m->loop_len;
    if (!m->loop_buf || len < 8u) {
        memset(m->wave_bins, 0, sizeof(m->wave_bins));
        return;
    }

    for (uint32_t b = 0u; b < WAVE_BINS; ++b) {
        const uint32_t f0 = (b * len) / WAVE_BINS;
        uint32_t f1 = ((b + 1u) * len) / WAVE_BINS;
        if (f1 <= f0) f1 = f0 + 1u;
        if (f1 > len) f1 = len;

        uint32_t stride = (f1 - f0) / 32u;
        if (stride < 1u) stride = 1u;

        float pk = 0.0f;
        for (uint32_t i = f0; i < f1; i += stride) {
            const swing_frame_t fr = m->loop_buf[i];
            const float al = fabsf(fr.l);
            const float ar = fabsf(fr.r);
            const float a = (al > ar) ? al : ar;
            if (a > pk) pk = a;
        }
        m->wave_bins[b] = (uint8_t)(clampf(pk, 0.0f, 1.0f) * 255.0f + 0.5f);
    }
}

static void sync_params_to_dsp(swing_model_t* m) {
    pendulum_set_rope(&m->dsp, m->prm[ENC_ROPE].val);
    pendulum_set_arc(&m->dsp, m->prm[ENC_ARC].val);
    pendulum_set_doppler(&m->dsp, m->prm[ENC_DOPPLER].val);
    pendulum_set_hall(&m->dsp, m->prm[ENC_HALL].val);
    pendulum_set_swing_on(&m->dsp, m->swing_on);
}

static void swing_sync_engine_active(swing_model_t* m) {
    const bool need = (m->state == ST_PLAY || m->state == ST_REC ||
                       m->bouncing || m->play_gain > 0.001f);
    if (need != m->eng_active) {
        engine_set_active(need);
    }
}

/*
 * Keep live input monitoring and WET FX-chain pass-through strictly OFF unless
 * actively capturing (ST_REC), preventing mic bleed or feedback out of speakers.
 */
static void swing_sync_monitor(swing_model_t* m) {
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

static void swing_tame_hot_mic_input(void) {
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

static void swing_bounce_start(swing_model_t* m) {
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

    m->bouncing = true;
    swing_sync_engine_active(m);
    swing_hints(m);
    ui_notify_toast(NotifyInfo, "Bouncing swing to tape", 1800u);
}

static void swing_bounce_stop(swing_model_t* m, tape_t* t) {
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
        ui_notify_toast(NotifySuccess, "Swing bounce saved", 1800u);
    }

    swing_sync_engine_active(m);
    swing_hints(m);
}

/* ----------------------------------------------------------------------------
 * Audio Engine (real-time 48 kHz, zero blocking, single-precision f32 only)
 * ------------------------------------------------------------------------- */
static inline void bounce_win_request(swing_model_t* m, uint32_t win) {
    tape_buffer_update(m->wb, win * TAPE_WINDOW_POS, true);
}

static void swing_process(void* restrict ctx, mixer_t* restrict mix) {
    swing_model_t* m = (swing_model_t*)ctx;
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

    if (m->push_req) {
        m->push_req = false;
        pendulum_push(&m->dsp, 0.85f);
    }
    if (m->center_req) {
        m->center_req = false;
        pendulum_center_still(&m->dsp);
    }

    if (locked) {
        for (uint32_t i = 0u; i < fs; i += 2u) {
            out[i] = 0.0f;
            out[i + 1u] = 0.0f;
        }
        return;
    }

    const float vout = m->vol_out.val;
    const float vfx = m->vol_fx.val;
    const float itrim = (m->in_src == 1u) ? m->mic_trim.val : 1.0f;
    const float play_target = (st == ST_PLAY && m->loop_len >= 8u) ? 1.0f : 0.0f;
    const float bounce_target = (bouncing && !m->bounce_stop_req) ? 1.0f : 0.0f;
    const float play_step = 1.0f / 720.0f;       /* ~15 ms anti-click */
    const float bounce_step = 1.0f / (float)BOUNCE_FADE_FR;

    float* bbuf = bouncing ? tape_buffer_get_ptr(m->wb) : NULL;
    float pl = 0.0f;
    float pr = 0.0f;

    swing_frame_t* lbuf = m->loop_buf;
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
                    lbuf[rf].l = pendulum_clip_guard(il * fg);
                    lbuf[rf].r = pendulum_clip_guard(ir * fg);
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
            /* Smooth tape motor speed & play envelope */
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

            if ((pg > 0.0001f || bouncing) && lbuf && len >= 8u) {
                const int32_t size = (int32_t)len;
                int32_t i0 = m->head_i;
                if (i0 < 0 || i0 >= size) i0 = 0;
                const float frac = m->head_f;

                swing_frame_t dry = { 0.0f, 0.0f };
                if (pg > 0.0001f) {
                    dry = loop_read_hermite(lbuf, size, i0, frac);
                    dry.l *= pg;
                    dry.r *= pg;

                    float nf = frac + m->speed_cur;
                    int32_t step = 0;
                    while (nf >= 1.0f) { nf -= 1.0f; step++; }
                    while (nf < 0.0f)  { nf += 1.0f; step--; }

                    int32_t next_i = i0 + step;
                    while (next_i >= size) next_i -= size;
                    while (next_i < 0)     next_i += size;
                    m->head_i = next_i;
                    m->head_f = nf;
                }

                const swing_frame_t wet = pendulum_process_frame(&m->dsp, dry);
                ol  = wet.l * vout;
                orr = wet.r * vout;
            }

            const float al = fabsf(ol);
            const float ar = fabsf(orr);
            if (al > pl) pl = al;
            if (ar > pr) pr = ar;
        }

        /* Optional Bounce-to-Tape streaming write */
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

        out[i]      = ol;
        out[i + 1u] = orr;
        if (send) {
            send[i]      += ol * vfx;
            send[i + 1u] += orr * vfx;
        }
    }

    m->peak_l = (pl > m->peak_l) ? pl : (m->peak_l * 0.85f);
    m->peak_r = (pr > m->peak_r) ? pr : (m->peak_r * 0.85f);
}

static void swing_eng_active(void* ctx, bool state) {
    ((swing_model_t*)ctx)->eng_active = state;
}

static uint_fast8_t swing_eng_is_active(void* ctx) {
    const swing_model_t* m = (const swing_model_t*)ctx;
    return (m->state == ST_PLAY || m->state == ST_REC ||
            m->bouncing || m->play_gain > 0.001f) ? 1u : 0u;
}

static void swing_tape_changed(void* ctx) {
    ((swing_model_t*)ctx)->tape_swapped = true;
}

static const engine_callbacks_t swing_engine_cb = {
    .process = swing_process,
    .tape_changed = swing_tape_changed,
    .active = swing_eng_active,
    .is_active = swing_eng_is_active,
};

/* ----------------------------------------------------------------------------
 * RGB LED Pattern (smooth Left <-> Right warm amber/ivory pendulum panner)
 * BTN2 column (LED 1) pulses red during ST_REC / bouncing.
 * ------------------------------------------------------------------------- */
static void swing_led_step(const os_led_t* led, const void* ctx) {
    const uint8_t step = os_led_get_step(led);
    const swing_model_t* m = (const swing_model_t*)ctx;
    if (!m) return;

    if (m->state == ST_REC) {
        const int tri = (step & 32u) ? (63 - (int)(step & 63u)) : (int)(step & 63u);
        for (uint8_t i = 0u; i < 4u; ++i) {
            const uint8_t r = (i == 1u) ? (uint8_t)(42 + tri * 2) : (uint8_t)(10 + tri);
            hal_led_set_rgb(i, r, 2u, 2u);
        }
        return;
    }

    /* Map lateral piano swing [-45..+45 deg] continuously across LEDs 0.0 .. 3.0 */
    const float norm_x = clampf(m->dsp.live_deg_x * (1.0f / 45.0f), -1.0f, 1.0f);
    const float led_pos = 1.5f + 1.5f * norm_x;
    const bool active_play = (m->state == ST_PLAY && m->loop_len >= 8u);
    const float hall_glow = clampf(m->dsp.hall_env * 2.2f, 0.0f, 0.45f);
    const float base_scale = active_play ? 1.0f : 0.28f;

    for (uint8_t i = 0u; i < 4u; ++i) {
        if (m->bouncing && i == 1u) {
            const uint8_t pulse = ((step & 16u) == 0u) ? 68u : 18u;
            hal_led_set_rgb(1u, pulse, 4u, 2u);
            continue;
        }

        const float dist = fabsf((float)i - led_pos);
        /* Smooth Gaussian-like spatial spotlight across adjacent LEDs */
        float spot = 1.0f - 0.62f * dist;
        if (spot < 0.08f) spot = 0.08f;
        spot = spot * spot;

        const float br = clampf((spot + 0.35f * hall_glow) * base_scale, 0.04f, 1.0f);
        if (m->swing_on) {
            /* Warm Barbican stage tungsten-amber / felt-ivory */
            const uint8_t r = (uint8_t)(64.0f * br);
            const uint8_t g = (uint8_t)(46.0f * br);
            const uint8_t b = (uint8_t)(18.0f * br);
            hal_led_set_rgb(i, r, g, b);
        } else {
            /* Natural free-decay coast: serene twilight ivory-blue */
            const uint8_t r = (uint8_t)(38.0f * br);
            const uint8_t g = (uint8_t)(42.0f * br);
            const uint8_t b = (uint8_t)(54.0f * br);
            hal_led_set_rgb(i, r, g, b);
        }
    }
}

static led_pattern_t swing_led_pattern = {
    .callback = swing_led_step,
    .steps = 255u,
    .rate = 30u,
    .loop = true,
};

/* ----------------------------------------------------------------------------
 * Loop Capture, Felt-Piano Seeding, Clear & Chunked SD Take Loader
 * ------------------------------------------------------------------------- */
static void swing_prepare_fresh_loop(swing_model_t* m, uint32_t len, bool already_seamless) {
    if (!already_seamless) {
        apply_loop_splice(m->loop_buf, len);
    }
    m->loop_len = len;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->play_gain = 0.0f;
    pendulum_clear_audio(&m->dsp);
    sync_params_to_dsp(m);
    rebuild_waveform_bins(m);
}

static void swing_finish_capture(swing_model_t* m) {
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
        swing_sync_monitor(m);
        swing_sync_engine_active(m);
        swing_hints(m);
        ui_notify_toast(NotifyWarning, "Loop too short", 1800u);
        return;
    }

    swing_prepare_fresh_loop(m, captured, false);
    if (!m->swing_on) {
        pendulum_push(&m->dsp, 0.75f);
    }

    m->state = ST_PLAY;
    m->buf_locked = false;
    swing_sync_monitor(m);
    swing_sync_engine_active(m);
    swing_hints(m);
}

static void swing_do_seed_piano(swing_model_t* m) {
    if (!m->loop_buf || m->max_frames < MIN_LOOP_FR) {
        ui_notify_toast(NotifyWarning, "RAM unavailable", 1800u);
        return;
    }

    m->buf_locked = true;
    m->state = ST_STOP;
    swing_sync_monitor(m);

    const uint32_t len = pendulum_seed_felt_piano(m->loop_buf, m->max_frames,
                                                  (float)TAPP_SAMPLE_RATE);
    if (len < MIN_LOOP_FR) {
        m->buf_locked = false;
        return;
    }

    swing_prepare_fresh_loop(m, len, true);
    if (!m->swing_on) {
        pendulum_push(&m->dsp, 0.80f);
    }

    m->state = ST_PLAY;
    m->buf_locked = false;
    swing_sync_monitor(m);
    swing_sync_engine_active(m);
    swing_hints(m);
    ui_notify_toast(NotifySuccess, "Seeded Felt Piano [102]", 1800u);
}

static void swing_do_clear(swing_model_t* m) {
    m->buf_locked = true;
    m->state = ST_STOP;
    m->loop_len = 0u;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->play_gain = 0.0f;
    memset(m->wave_bins, 0, sizeof(m->wave_bins));
    pendulum_clear_audio(&m->dsp);
    m->buf_locked = false;
    swing_sync_monitor(m);
    swing_sync_engine_active(m);
    swing_hints(m);
    ui_notify_toast(NotifyInfo, "Loop cleared", 1800u);
}

static void swing_start_take_load(swing_model_t* m, uint32_t take_idx) {
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
    swing_sync_monitor(m);
    swing_sync_engine_active(m);
    swing_hints(m);
}

static void swing_step_take_load(swing_model_t* m) {
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
        m->loop_buf[m->load_done + i].l = pendulum_clip_guard(m->read_stage[i * 2u]);
        m->loop_buf[m->load_done + i].r = pendulum_clip_guard(m->read_stage[i * 2u + 1u]);
    }
    m->load_done += got;

    if (m->load_done >= m->load_total) {
        swing_prepare_fresh_loop(m, m->load_done, false);
        if (!m->swing_on) {
            pendulum_push(&m->dsp, 0.75f);
        }
        m->state = ST_PLAY;
        m->buf_locked = false;
        swing_sync_monitor(m);
        swing_sync_engine_active(m);
        swing_hints(m);
        ui_notify_toast(NotifySuccess, "Take loaded onto rig", 1800u);
    }
}

/* ----------------------------------------------------------------------------
 * Menu System (BTN3 Hold — "SWINGING PIANO")
 * ------------------------------------------------------------------------- */
static void menu_act_load_take(ui_menu_t* menu, ui_menu_node_t* entry) {
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    const uint16_t idx = UI_PICK_IDX(entry);
    ui_menu_hide(menu);
    swing_start_take_load(m, (uint32_t)idx);
}

static void menu_takes_builder(ui_menu_t* menu) {
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
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

static void menu_act_seed_piano(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    m->seed_req = true;
    ui_menu_hide(menu);
}

static void menu_act_push(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    m->push_req = true;
    ui_menu_hide(menu);
    ui_notify_toast(NotifyInfo, "Pushed piano!", 1400u);
}

static void menu_act_center(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    m->swing_on = false;
    pendulum_set_swing_on(&m->dsp, false);
    m->center_req = true;
    ui_menu_hide(menu);
    ui_notify_toast(NotifyInfo, "Centered & still", 1400u);
}

static void menu_act_cycle_rig(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    const uint8_t next = (uint8_t)((m->dsp.rig_mode + 1u) % SWING_RIG_COUNT);
    pendulum_set_rig(&m->dsp, next);
    ui_menu_rebuild(menu);
}

static void menu_act_flip_dir(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    m->speed_target = -m->speed_target;
    if (fabsf(m->speed_target) < 0.1f) m->speed_target = -1.0f;
    ui_menu_hide(menu);
}

static void menu_act_speed_1x(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 1.0f;
    ui_menu_hide(menu);
}

static void menu_act_speed_half(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 0.5f;
    ui_menu_hide(menu);
}

static void menu_act_speed_double(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 2.0f;
    ui_menu_hide(menu);
}

static void menu_act_input(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_switch_input();
    m->mon_req = false;
    swing_tame_hot_mic_input();
    swing_sync_monitor(m);
    ui_menu_rebuild(menu);
}

static void menu_act_clear(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    m->clear_req = true;
    ui_menu_hide(menu);
}

static void menu_root_builder(ui_menu_t* menu) {
    swing_model_t* m = (swing_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_get_input();

    snprintf(m->rig_label, sizeof(m->rig_label), "Rig: %s",
             pendulum_rig_name(m->dsp.rig_mode));
    snprintf(m->dir_label, sizeof(m->dir_label), "Direction: %s",
             (m->speed_target < 0.0f) ? "REV -> FWD" : "FWD -> REV");
    snprintf(m->input_label, sizeof(m->input_label), "Input: %s",
             (m->in_src == 1u) ? "Mic" : "Line");

    ui_menu_add(menu, UI_ENTRY("Load Tape Take", menu_takes_builder));
    ui_menu_add(menu, UI_ACTION("Seed Felt Piano", menu_act_seed_piano));
    ui_menu_add(menu, UI_ACTION("Push Piano (Impulse)", menu_act_push));
    ui_menu_add(menu, UI_ACTION("Center & Still", menu_act_center));
    ui_menu_add(menu, UI_ACTION(m->rig_label, menu_act_cycle_rig));
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
static void swing_hints(const swing_model_t* m) {
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
        [HintPress3] = {m->swing_on ? "swing" : "coast",
                        gfx_icon(m->swing_on ? GfxIconSpeed : GfxIconFx), NULL,
                        m->swing_on ? HINT_ACTIVE : 0u},
        [HintHold3]  = {"menu", gfx_icon(GfxIconMenu)},
        [HintPress4] = {NULL, gfx_icon(GfxIconPlayRev)},
        [HintPress5] = {NULL, gfx_icon(GfxIconPlay)},
    };
    ui_hints_set(hints);
}

static void swing_vol_show(swing_model_t* m) {
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

static void swing_vol_hide(swing_model_t* m) {
    m->vol_ui = false;
    swing_hints(m);
}

/* ----------------------------------------------------------------------------
 * Drawing (400x240 1-bit monochrome OLED, content y = 48..240)
 * ------------------------------------------------------------------------- */
#define RIG_X         8
#define RIG_Y         74
#define RIG_W         136
#define RIG_H         82
#define PIV_X         76
#define PIV_Y         80
#define FLOOR_Y       151
#define MIC_LX        51
#define MIC_RX        101

#define STAGE_X       148
#define STAGE_Y       74
#define STAGE_W       244
#define STAGE_H       82
#define TRACK_X       154
#define TRACK_W       232       /* 116 bins * 2px */
#define TRACK_MID_Y   104
#define TRACK_HALF_H  22
#define FOOTER_Y      138

#define CARD_Y        162
#define CARD_H        40
#define CARD_W        93
#define CARD_GAP      4

/* Crisp 3x3 pixel degree symbol at (x, y_top) */
static void draw_deg_ring(gfx_t* gfx, int x, int y_top) {
    gfx_draw_hline(gfx, x + 1, y_top, 2);
    gfx_draw_hline(gfx, x + 1, y_top + 3, 2);
    gfx_draw_vline(gfx, x,     y_top + 1, 2);
    gfx_draw_vline(gfx, x + 3, y_top + 1, 2);
}

/* Pendulum coordinates downward from overhead pivot (PIV_X, PIV_Y): deg in [-62..+62] */
static void pendulum_pt(int r, float deg, int* out_x, int* out_y) {
    /* Convert deg to Q16 turn: 0 = 6 o'clock downward; +deg = right */
    const int32_t q16 = (int32_t)(deg * (65536.0f / 360.0f));
    const uint16_t ang = (uint16_t)(q16 & 0xFFFF);
    const int sx = (int)lutsin_q15(ang);
    const int cy = (int)lutcos_q15(ang);
    *out_x = PIV_X + ((sx * r + (sx >= 0 ? 16384 : -16384)) >> 15);
    *out_y = PIV_Y + ((cy * r + (cy >= 0 ? 16384 : -16384)) >> 15);
}

static void draw_header(gfx_t* gfx, const swing_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_set_font(gfx, DepartureMono_Regular_10);

    /* Title on left */
    gfx_draw_str(gfx, 8, 64, "SWINGING PIANO");
    int tx = 8 + (int)gfx_get_str_width(gfx, "SWINGING PIANO") + 6;

    /* State pill badge */
    const char* badge = "STOPPED";
    bool inv_badge = false;
    if (m->state == ST_REC) {
        badge = "CAPTURE";
        inv_badge = (((os_tick_get() / 300u) & 1u) == 0u);
    } else if (m->state == ST_LOADING) {
        badge = "LOADING";
        inv_badge = true;
    } else if (m->state == ST_PLAY) {
        if (m->swing_on) {
            badge = "SWINGING";
            inv_badge = true;
        } else {
            badge = "COASTING";
        }
    }

    const int bw = (int)gfx_get_str_width(gfx, badge) + 8;
    if (inv_badge) {
        gfx_draw_rect_fill_r(gfx, tx, 53, bw, 16, 2);
        gfx_set_color(gfx, GFX_COLOR_CLEAR);
        gfx_draw_str(gfx, tx + 4, 64, badge);
        gfx_set_color(gfx, GFX_COLOR_SET);
    } else {
        gfx_draw_rect_r(gfx, tx, 53, bw, 16, 2);
        gfx_draw_str(gfx, tx + 4, 64, badge);
    }
    tx += bw + 6;

    /* Live cable length (e.g. 7.5m) + optional bounce timer */
    const int rope_dm = (int)(m->dsp.rope_len * 10.0f + 0.5f);
    if (m->bouncing) {
        const uint32_t bsec = (m->bounce_frame - m->bounce_start) / TAPP_SAMPLE_RATE;
        gfx_draw_strf(gfx, tx, 64, "%d.%dm R%02u:%02u",
                      rope_dm / 10, rope_dm % 10,
                      (unsigned)(bsec / 60u), (unsigned)(bsec % 60u));
    } else {
        gfx_draw_strf(gfx, tx, 64, "%d.%dm", rope_dm / 10, rope_dm % 10);
    }

    /* Right-aligned live Doppler cents + pendulum angle readout (e.g. +18c +32 deg) */
    const int cents = (int)(m->dsp.live_cents >= 0.0f ? (m->dsp.live_cents + 0.5f)
                                                      : (m->dsp.live_cents - 0.5f));
    const int ang = (int)(m->dsp.live_deg_x >= 0.0f ? (m->dsp.live_deg_x + 0.5f)
                                                    : (m->dsp.live_deg_x - 0.5f));
    char rtxt[24];
    snprintf(rtxt, sizeof(rtxt), "%+dc %+d", cents, ang);
    const int rw = (int)gfx_get_str_width(gfx, rtxt);
    const int rx = 386 - rw;
    gfx_draw_str(gfx, rx, 64, rtxt);
    draw_deg_ring(gfx, rx + rw + 2, 55);
}

static void draw_pendulum_rig(gfx_t* gfx, const swing_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_rect_r(gfx, RIG_X, RIG_Y, RIG_W, RIG_H, 3);

    /* Overhead Barbican fly-tower truss beam + central pivot shackle */
    gfx_draw_hline_dithered(gfx, RIG_X + 6, PIV_Y - 2, RIG_W - 12, 3);
    gfx_draw_rect_fill(gfx, PIV_X - 3, PIV_Y - 3, 7, 4);
    gfx_draw_disc(gfx, PIV_X, PIV_Y, 2, GFX_DRAW_ALL);

    /* Corner Rig identifier */
    gfx_set_font(gfx, DepartureMono_Regular_10);
    gfx_draw_str(gfx, RIG_X + 5, RIG_Y + 13, pendulum_rig_short_name(m->dsp.rig_mode));

    /* Mini 2D top-down orbital scope in upper-right when in Elliptical 2D mode */
    if (m->dsp.rig_mode == SWING_RIG_ELLIPTICAL) {
        const int ox = RIG_X + RIG_W - 14;
        const int oy = RIG_Y + 11;
        gfx_draw_circle(gfx, ox, oy, 6, GFX_DRAW_ALL);
        const int dx = (int)clampf(m->dsp.live_deg_x * 0.11f, -5.0f, 5.0f);
        const int dy = (int)clampf(m->dsp.live_deg_y * 0.16f, -5.0f, 5.0f);
        gfx_draw_disc(gfx, ox + dx, oy + dy, 1, GFX_DRAW_ALL);
    }

    /* Visual cable radius scales with ROPE (36..51 px) */
    const int r_vis = 36 + (int)(clampf(m->prm[ENC_ROPE].val, 0.0f, 1.0f) * 15.0f + 0.5f);
    const float max_arc_deg = clampf(m->prm[ENC_ARC].val, 0.0f, 1.0f) * 55.0f;

    /* Dotted circular swing arc guide from -55 to +55 deg */
    for (int deg = -55; deg <= 55; deg += 5) {
        int ax = 0, ay = 0;
        pendulum_pt(r_vis, (float)deg, &ax, &ay);
        if ((deg % 10) == 0 || fabsf((float)deg) <= max_arc_deg) {
            gfx_draw_pixel(gfx, ax, ay);
        }
    }

    /* Target ARC excursion stop ticks on the arc */
    if (max_arc_deg > 2.0f) {
        for (int s = -1; s <= 1; s += 2) {
            int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
            pendulum_pt(r_vis - 2, (float)s * max_arc_deg, &x0, &y0);
            pendulum_pt(r_vis + 2, (float)s * max_arc_deg, &x1, &y1);
            gfx_draw_line(gfx, x0, y0, x1, y1);
        }
    }

    /* Dithered motion trail behind the swinging grand piano */
    for (uint32_t k = 0u; k < TRAIL_LEN; ++k) {
        const float tdeg = (float)m->trail_deg[k];
        if (fabsf(tdeg - m->dsp.live_deg_x) < 1.5f) continue;
        int tx = 0, ty = 0;
        pendulum_pt(r_vis, tdeg, &tx, &ty);
        const uint8_t shade = (uint8_t)(1u + k);
        gfx_fill_rect_dithered(gfx, tx - 3, ty - 1, 7, 3, shade);
    }

    /* Live piano position & taut suspension cable */
    int px = 0, py = 0;
    pendulum_pt(r_vis, m->dsp.live_deg_x, &px, &py);
    gfx_draw_line(gfx, PIV_X, PIV_Y, px, py);

    /*
     * Stylized suspended Concert Grand Piano + open lid tilted along arc tangent:
     * Tangent vector u = (cos(theta), -sin(theta)), radial vector n = (sin(theta), cos(theta)).
     */
    const int32_t q16 = (int32_t)(m->dsp.live_deg_x * (65536.0f / 360.0f));
    const uint16_t ang = (uint16_t)(q16 & 0xFFFF);
    const float st = (float)lutsin_q15(ang) * (1.0f / 32768.0f);
    const float ct = (float)lutcos_q15(ang) * (1.0f / 32768.0f);

    /* Soundboard deck (14px wide along tangent) */
    const int x_l = px - (int)(7.0f * ct);
    const int y_l = py + (int)(7.0f * st);
    const int x_r = px + (int)(7.0f * ct);
    const int y_r = py - (int)(7.0f * st);
    gfx_draw_line_thick(gfx, x_l, y_l, x_r, y_r, 3);

    /* Curved bass rim under left side + raised grand piano lid + prop stick */
    const int rim_x = x_l + (int)(3.0f * st);
    const int rim_y = y_l + (int)(3.0f * ct);
    gfx_draw_line(gfx, x_l, y_l, rim_x, rim_y);
    gfx_draw_line(gfx, rim_x, rim_y, px + (int)(2.0f * st), py + (int)(2.0f * ct));

    const int lid_tip_x = px + (int)(5.0f * ct - 6.0f * st);
    const int lid_tip_y = py + (int)(-5.0f * st - 6.0f * ct);
    gfx_draw_line(gfx, x_l, y_l, lid_tip_x, lid_tip_y);
    gfx_draw_line(gfx, px + (int)(3.0f * ct), py - (int)(3.0f * st), lid_tip_x, lid_tip_y);

    /* Barbican stage floor + stationary L and R microphone capsules */
    gfx_draw_hline_dithered(gfx, RIG_X + 6, FLOOR_Y, RIG_W - 12, 2);

    const int mic_xs[2] = { MIC_LX, MIC_RX };
    for (int mi = 0; mi < 2; ++mi) {
        const int mx = mic_xs[mi];
        gfx_draw_vline(gfx, mx, FLOOR_Y - 5, 5);
        gfx_draw_hline(gfx, mx - 2, FLOOR_Y, 5);
        gfx_draw_disc(gfx, mx, FLOOR_Y - 6, 1, GFX_DRAW_ALL);

        /* Proximity ripples as the piano swoops past each floor mic */
        const float geo_prox = clampf(1.0f - fabsf((float)(px - mx)) * (1.0f / 42.0f), 0.0f, 1.0f);
        const float aud_env = (mi == 0) ? m->dsp.prox_env_l : m->dsp.prox_env_r;
        const float rip = clampf(0.45f * geo_prox + 1.35f * aud_env, 0.0f, 1.0f);
        const int rings = (int)(rip * 3.2f);
        for (int r = 1; r <= rings; ++r) {
            gfx_draw_circle(gfx, mx, FLOOR_Y - 6, (uint16_t)(2 + r * 3),
                            GFX_DRAW_UPPER_LEFT | GFX_DRAW_UPPER_RIGHT);
        }
    }

    gfx_draw_str(gfx, MIC_LX - 11, FLOOR_Y - 1, "L");
    gfx_draw_str(gfx, MIC_RX + 6,  FLOOR_Y - 1, "R");
}

static void draw_footer_strip(gfx_t* gfx, const swing_model_t* m) {
    gfx_draw_hline_dithered(gfx, STAGE_X + 3, FOOTER_Y - 2, STAGE_W - 6, 2);
    gfx_set_font(gfx, DepartureMono_Regular_10);

    gfx_draw_str(gfx, STAGE_X + 5, FOOTER_Y + 11, "L");
    gfx_draw_str(gfx, STAGE_X + STAGE_W - 11, FOOTER_Y + 11, "R");

    /* Stereo L/R Proximity & Hall Bloom Meter */
    const int bx = STAGE_X + 15;
    const int bw = STAGE_W - 30;
    const int by = FOOTER_Y + 3;
    const int bh = 8;
    const int mid = bx + bw / 2;
    const int half = (bw - 4) / 2;

    gfx_draw_rect_r(gfx, bx, by, bw, bh, 2);

    /* Dithered diffuse Barbican Hall bloom aura */
    const float hall_vis = clampf(m->dsp.hall_env * 2.4f + 0.22f * m->prm[ENC_HALL].val, 0.0f, 1.0f);
    const int h_span = (int)(hall_vis * (float)half);
    if (h_span > 1) {
        gfx_fill_rect_dithered(gfx, mid - h_span, by + 1, h_span * 2, bh - 2, 2);
    }

    /* Crisp Left / Right direct mic proximity bars */
    const float pl = (m->state == ST_PLAY) ? clampf(m->dsp.prox_env_l * 1.8f, 0.0f, 1.0f)
                                           : clampf(m->dsp.gain_l * 0.55f, 0.0f, 1.0f);
    const float pr = (m->state == ST_PLAY) ? clampf(m->dsp.prox_env_r * 1.8f, 0.0f, 1.0f)
                                           : clampf(m->dsp.gain_r * 0.55f, 0.0f, 1.0f);
    const int lw = (int)(pl * (float)half);
    const int rw = (int)(pr * (float)half);
    if (lw > 0) gfx_draw_rect_fill(gfx, mid - lw, by + 2, lw, bh - 4);
    if (rw > 0) gfx_draw_rect_fill(gfx, mid,      by + 2, rw, bh - 4);

    /* Center nadir tick + live acoustic source position pip */
    gfx_set_color(gfx, GFX_COLOR_XOR);
    gfx_draw_vline(gfx, mid, by + 1, bh - 2);
    const int pip_x = mid + (int)(clampf(m->dsp.live_deg_x * (1.0f / 55.0f), -1.0f, 1.0f) *
                                  (float)(half - 2));
    gfx_draw_vline(gfx, pip_x, by + 1, bh - 2);
    gfx_set_color(gfx, GFX_COLOR_SET);
}

static void draw_waveform_stage(gfx_t* gfx, const swing_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_rect_r(gfx, STAGE_X, STAGE_Y, STAGE_W, STAGE_H, 3);

    /* Loading progress view */
    if (m->state == ST_LOADING) {
        gfx_set_font(gfx, gfx_nunito_semibold_14);
        const char* msg = "RIGGING TAPE TAKE...";
        const int mw = (int)gfx_get_str_width(gfx, msg);
        gfx_draw_str(gfx, STAGE_X + (STAGE_W - mw) / 2, STAGE_Y + 32, msg);

        const float pct = (m->load_total > 0u)
                              ? ((float)m->load_done / (float)m->load_total)
                              : 0.0f;
        gfx_draw_rect_r(gfx, STAGE_X + 20, STAGE_Y + 46, STAGE_W - 40, 14, 3);
        const int fw = (int)(clampf(pct, 0.0f, 1.0f) * (float)(STAGE_W - 44) + 0.5f);
        if (fw > 0) gfx_draw_rect_fill_r(gfx, STAGE_X + 22, STAGE_Y + 48, fw, 10, 2);
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

        const uint32_t ds = m->rec_frames / 4800u;
        const uint32_t max_ds = m->max_frames / 4800u;
        gfx_set_font(gfx, DepartureMono_Regular_10);
        gfx_draw_strf(gfx, STAGE_X + 10, STAGE_Y + STAGE_H - 10,
                      "REC %u.%us/%u.%us [LOOP]",
                      (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                      (unsigned)(max_ds / 10u), (unsigned)(max_ds % 10u));
        return;
    }

    /* Empty state prompt */
    if (m->loop_len < 8u) {
        gfx_draw_hline_dithered(gfx, TRACK_X, TRACK_MID_Y, TRACK_W, 1);

        /* Decaying pendulum harmonic trace */
        for (int i = 0; i < TRACK_W; i += 2) {
            const uint16_t ph = (uint16_t)((i * 65536) / 56);
            const int env = 14 - (i * 11) / TRACK_W;
            const int y = ((int)lutsin_q15(ph) * env) >> 15;
            gfx_draw_pixel(gfx, TRACK_X + i, TRACK_MID_Y - 5 - y);
        }

        gfx_set_font(gfx, DepartureMono_Regular_10);
        const char* hint = "REC OR MENU -> SEED PIANO";
        const int hw = (int)gfx_get_str_width(gfx, hint);
        gfx_draw_str(gfx, STAGE_X + (STAGE_W - hw) / 2, FOOTER_Y - 6, hint);
        draw_footer_strip(gfx, m);
        return;
    }

    /* Active loop waveform + duration badge */
    gfx_draw_hline_dithered(gfx, TRACK_X, TRACK_MID_Y, TRACK_W, 1);
    for (uint32_t b = 0u; b < WAVE_BINS; ++b) {
        const int h = ((int)m->wave_bins[b] * TRACK_HALF_H) / 255;
        if (h > 0) {
            const int x = TRACK_X + (int)(b * 2u);
            gfx_draw_vline(gfx, x, TRACK_MID_Y - h, h * 2 + 1);
        }
    }

    /* Duration & speed tag in top-right of stage */
    gfx_set_font(gfx, DepartureMono_Regular_10);
    {
        const uint32_t ds = m->loop_len / 4800u;
        char dstr[20];
        if (fabsf(m->speed_target - 1.0f) > 0.05f) {
            const char sgn = (m->speed_target < 0.0f) ? '-' : '+';
            const int sp10 = (int)(fabsf(m->speed_target) * 10.0f + 0.5f);
            snprintf(dstr, sizeof(dstr), "%u.%us %c%d.%dx",
                     (unsigned)(ds / 10u), (unsigned)(ds % 10u), sgn, sp10 / 10, sp10 % 10);
        } else {
            snprintf(dstr, sizeof(dstr), "%u.%us", (unsigned)(ds / 10u), (unsigned)(ds % 10u));
        }
        const int dw = (int)gfx_get_str_width(gfx, dstr);
        gfx_draw_str(gfx, STAGE_X + STAGE_W - 6 - dw, STAGE_Y + 12, dstr);
    }

    /* Playhead cursor (2px XOR) + direction chevron */
    const int px = TRACK_X + (int)(((uint32_t)m->head_i * (TRACK_W - 2u)) / m->loop_len);
    gfx_set_color(gfx, GFX_COLOR_XOR);
    gfx_draw_vline(gfx, px, STAGE_Y + 3, FOOTER_Y - STAGE_Y - 6);
    gfx_draw_vline(gfx, px + 1, STAGE_Y + 3, FOOTER_Y - STAGE_Y - 6);
    gfx_set_color(gfx, GFX_COLOR_SET);

    if (m->speed_cur < -0.02f) {
        gfx_draw_line(gfx, px + 3, STAGE_Y + 3, px - 2, STAGE_Y + 6);
        gfx_draw_line(gfx, px + 3, STAGE_Y + 9, px - 2, STAGE_Y + 6);
    } else if (m->speed_cur > 0.02f) {
        gfx_draw_line(gfx, px - 2, STAGE_Y + 3, px + 3, STAGE_Y + 6);
        gfx_draw_line(gfx, px - 2, STAGE_Y + 9, px + 3, STAGE_Y + 6);
    }

    draw_footer_strip(gfx, m);
}

static void draw_param_cards(gfx_t* gfx, const swing_model_t* m) {
    for (uint8_t idx = 0u; idx < ENC_TOTAL; ++idx) {
        const int x = 8 + (int)idx * (CARD_W + CARD_GAP);
        const bool sel = (m->enc_sel == idx);
        const params_t* p = &m->prm[idx];

        gfx_set_color(gfx, GFX_COLOR_SET);
        ui_draw_frame(gfx, x, CARD_Y, CARD_W, CARD_H, 3, sel);

        gfx_set_font(gfx, DepartureMono_Regular_10);
        gfx_draw_str(gfx, x + 5, CARD_Y + 13, p->name);

        char val_s[16];
        if (idx == ENC_ROPE) {
            const int dm = (int)((2.20f + 11.80f * p->val) * 10.0f + 0.5f);
            snprintf(val_s, sizeof(val_s), "%d.%dm", dm / 10, dm % 10);
            const int vw = (int)gfx_get_str_width(gfx, val_s);
            gfx_draw_str(gfx, x + CARD_W - 5 - vw, CARD_Y + 13, val_s);
        } else if (idx == ENC_ARC) {
            const int deg = (int)(p->val * 55.0f + 0.5f);
            snprintf(val_s, sizeof(val_s), "%d", deg);
            const int vw = (int)gfx_get_str_width(gfx, val_s);
            const int vx = x + CARD_W - 10 - vw;
            gfx_draw_str(gfx, vx, CARD_Y + 13, val_s);
            draw_deg_ring(gfx, vx + vw + 1, CARD_Y + 4);
        } else {
            const int pc = (int)(p->val * 100.0f + 0.5f);
            snprintf(val_s, sizeof(val_s), "%d%%", pc);
            const int vw = (int)gfx_get_str_width(gfx, val_s);
            gfx_draw_str(gfx, x + CARD_W - 5 - vw, CARD_Y + 13, val_s);
        }

        const int bx = x + 6;
        const int by = CARD_Y + 22;
        const int bw = CARD_W - 12;
        const int bh = 11;
        gfx_draw_rect_r(gfx, bx, by, bw, bh, 2);

        if (idx == ENC_DOPPLER) {
            /* Mark 50% natural 343 m/s hall reference inside DOPPLER bar */
            const int mid = bx + bw / 2;
            gfx_draw_vline_dithered(gfx, mid, by + 1, bh - 2, 4);
        }

        const int fill = (int)(clampf(p->val, 0.0f, 1.0f) * (float)(bw - 4) + 0.5f);
        if (fill > 0) {
            gfx_draw_rect_fill(gfx, bx + 2, by + 2, fill, bh - 4);
        }
    }
}

static void swing_redraw(gfx_t* gfx, const os_app_t* app) {
    const swing_model_t* m = (const swing_model_t*)os_app_get_model(app);
    if (!m) return;

    draw_header(gfx, m);
    draw_pendulum_rig(gfx, m);
    draw_waveform_stage(gfx, m);
    draw_param_cards(gfx, m);
}

/* ----------------------------------------------------------------------------
 * Tick (main UI thread — safe for tape_read, menu commands & UI sync)
 * ------------------------------------------------------------------------- */
static bool swing_tick(os_app_t* app) {
    swing_model_t* m = (swing_model_t*)os_app_get_model(app);
    tape_t* t = tape_get();
    const uint32_t now_ms = os_tick_get();
    const uint32_t dt_ms = (m->last_tick_ms != 0u && now_ms > m->last_tick_ms)
                               ? (now_ms - m->last_tick_ms)
                               : 33u;
    m->last_tick_ms = now_ms;

    if (m->tape_swapped) {
        m->tape_swapped = false;
        if (m->bouncing) {
            m->bouncing = false;
            m->bounce_tok = 0u;
        }
        if (m->wb) tape_buffer_clear_all_buffers(m->wb);
        m->bounce_cur_win = WIN_NONE;
        tape_timeline_set_loop(false);
        swing_hints(m);
    }

    if (m->seed_req) {
        m->seed_req = false;
        swing_do_seed_piano(m);
    }

    if (m->clear_req) {
        m->clear_req = false;
        swing_do_clear(m);
    }

    if (m->state == ST_LOADING) {
        swing_step_take_load(m);
        return true;
    }

    if (m->state == ST_REC && m->rec_done_req) {
        swing_finish_capture(m);
    }

    if (m->bouncing && t) {
        if (!m->bounce_stop_req && m->bounce_frame + BOUNCE_FADE_FR >= m->bounce_limit) {
            m->bounce_stop_req = true;
        }
        if ((m->bounce_stop_req && m->bounce_gain <= 0.001f) ||
            m->bounce_frame >= m->bounce_limit) {
            swing_bounce_stop(m, t);
        }
    }

    m->in_src = os_audio_get_input();
    swing_sync_monitor(m);
    swing_sync_engine_active(m);

    /* Step idle visual pendulum only when audio engine is parked */
    if (!m->eng_active && m->state == ST_STOP) {
        if (m->push_req) {
            m->push_req = false;
            pendulum_push(&m->dsp, 0.85f);
        }
        if (m->center_req) {
            m->center_req = false;
            pendulum_center_still(&m->dsp);
        }
        pendulum_step_idle(&m->dsp, (float)dt_ms * 0.001f);
    }

    /* Update pendulum motion trail every ~75 ms */
    if (now_ms - m->last_trail_ms >= 75u) {
        m->last_trail_ms = now_ms;
        for (uint32_t k = 0u; k + 1u < TRAIL_LEN; ++k) {
            m->trail_deg[k] = m->trail_deg[k + 1u];
        }
        const float cd = clampf(m->dsp.live_deg_x, -62.0f, 62.0f);
        m->trail_deg[TRAIL_LEN - 1u] = (int8_t)(cd >= 0.0f ? (cd + 0.5f) : (cd - 0.5f));
    }

    /* Encoder input (only when neither volume overlay nor modal menu is open) */
    if (!m->vol_ui && !(m->menu && ui_menu_is_visible(m->menu))) {
        const int32_t d = os_controls_encoder_get_delta();
        if (d != 0) {
            params_t* p = &m->prm[m->enc_sel];
            param_write_delta_val(p, d);
            param_update_fast(p, m);
            sync_params_to_dsp(m);
        }
    }

    if (m->bouncing) {
        const float pos = (float)(m->bounce_frame / FRAMES_PER_POS);
        m->tl_pos.prev = m->tl_pos.val;
        m->tl_pos.val = m->tl_pos.target = pos;
        m->tl_end.val = m->tl_end.target = pos;
    }

    if (!m->vol_ui && (m->hint_state != m->state ||
                       m->hint_swing != m->swing_on ||
                       m->hint_bounce != m->bouncing)) {
        m->hint_state = m->state;
        m->hint_swing = m->swing_on;
        m->hint_bounce = m->bouncing;
        swing_hints(m);
    }

    return true;
}

/* ----------------------------------------------------------------------------
 * Input Handler
 * ------------------------------------------------------------------------- */
static void swing_input(os_app_t* app, uint8_t btn, KeyStateEnum st) {
    swing_model_t* m = (swing_model_t*)os_app_get_model(app);
    if (btn == BTN_ENCODER) return;

    if (m->vol_ui) {
        if (st == KEY_STATE_RELEASED && btn == BTN1) swing_vol_hide(m);
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
                swing_sync_monitor(m);
                swing_sync_engine_active(m);
            } else if (m->state == ST_STOP && m->loop_len >= 8u) {
                m->state = ST_PLAY;
                swing_sync_monitor(m);
                swing_sync_engine_active(m);
            }
            break;

        case BTN2:
            if (m->state == ST_REC) {
                m->rec_btn_down = false;
                m->rec_holdoff_fr = 0u;
                m->rec_manual_stop = true;
                m->rec_done_req = true;
                swing_sync_monitor(m);
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
                swing_sync_monitor(m);
                swing_sync_engine_active(m);
            }
            break;

        case BTN3:
            m->swing_on = !m->swing_on;
            pendulum_set_swing_on(&m->dsp, m->swing_on);
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
        m->hint_swing = m->swing_on;
        m->hint_bounce = m->bouncing;
        swing_hints(m);
    } else if (st == KEY_STATE_HOLD) {
        if (btn == BTN1 && !m->vol_ui) {
            m->vol_ui = true;
            swing_vol_show(m);
        } else if (btn == BTN2 && m->state != ST_REC) {
            if (m->bouncing) {
                m->bounce_stop_req = true;
            } else {
                swing_bounce_start(m);
            }
        } else if (btn == BTN3) {
            if (m->menu) ui_menu_show(m->menu);
        }
    }
}

/* ----------------------------------------------------------------------------
 * Lifecycle (init / deinit)
 * ------------------------------------------------------------------------- */
static bool swing_app_init(os_app_t* app) {
    swing_model_t* m = (swing_model_t*)os_app_get_model(app);

    /* Allocate 96 KB acoustic delay/FDN workspace + tape read staging buffer first */
    m->space = (swing_space_t*)os_malloc(sizeof(swing_space_t));
    m->read_stage = (float*)os_malloc((size_t)LOAD_CHUNK_FR * 2u * sizeof(float));

    /* Allocate PSRAM stereo loop buffer with graceful fallback: 24s -> 16s -> 12s -> 8s -> 4s -> 2s */
    static const uint32_t kCandSecs[] = { 24u, 16u, 12u, 8u, 6u, 4u, 2u };
    for (uint32_t c = 0u; c < (sizeof(kCandSecs) / sizeof(kCandSecs[0])); ++c) {
        const uint32_t frames = kCandSecs[c] * TAPP_SAMPLE_RATE;
        swing_frame_t* lb = (swing_frame_t*)os_malloc((size_t)frames * sizeof(swing_frame_t));
        if (lb) {
            m->loop_buf = lb;
            m->max_frames = frames;
            break;
        }
    }
    if (!m->loop_buf) {
        if (m->read_stage) { os_free(m->read_stage); m->read_stage = NULL; }
        if (m->space)      { os_free(m->space);      m->space = NULL; }
        return false;
    }

    tape_t* t = tape_get();
    if (t) {
        m->wb = tape_buffer_init(t);
    }
    m->bounce_cur_win = WIN_NONE;

    /* Initialize Barbican Pendulum + Hall Acoustic DSP engine */
    pendulum_init(&m->dsp, (float)TAPP_SAMPLE_RATE);
    pendulum_attach_space(&m->dsp, m->space);

    m->swing_on = true;
    m->speed_cur = 1.0f;
    m->speed_target = 1.0f;

    /* 4 live encoder cards: ROPE, ARC, DOPPLER, HALL */
    m->prm[ENC_ROPE]    = (params_t)PARAM_RANGE("ROPE",    0.45f, 0.00f, 1.00f, 0.02f);
    m->prm[ENC_ARC]     = (params_t)PARAM_RANGE("ARC",     0.55f, 0.00f, 1.00f, 0.02f);
    m->prm[ENC_DOPPLER] = (params_t)PARAM_RANGE("DOPPLER", 0.50f, 0.00f, 1.00f, 0.02f);
    m->prm[ENC_HALL]    = (params_t)PARAM_RANGE("HALL",    0.42f, 0.00f, 1.00f, 0.02f);
    m->enc_sel = ENC_ROPE;
    sync_params_to_dsp(m);
    m->dsp.rope_len = m->dsp.rope_len_target;

    for (uint32_t k = 0u; k < TRAIL_LEN; ++k) {
        m->trail_deg[k] = (int8_t)m->dsp.live_deg_x;
    }

    /* Volume overlay & mic capture trim params */
    m->vol_out = (params_t)PARAM_RANGE("out", 1.0f, 0.0f, 2.0f, 0.02f);
    m->vol_out.type = ParamValDBType;
    m->vol_fx = (params_t)PARAM_RANGE("fx send", 0.0f, 0.0f, 1.0f, 0.02f);
    m->vol_fx.type = ParamValDBType;
    m->mic_trim = (params_t)PARAM_RANGE("mic trim", 0.40f, 0.05f, 1.50f, 0.02f);
    m->mic_trim.type = ParamValDBType;

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

    m->menu = ui_menu_create(m, menu_root_builder);
    if (m->menu) {
        ui_menu_set_name(m->menu, "SWINGING PIANO");
        ui_menu_set_header(m->menu, true);
    }

    /* Save user's monitor/WET state, silence idle speaker bleed, and tame hot mic gain */
    m->in_src = os_audio_get_input();
    m->mon_saved = os_audio_get_monitor();
    m->wet_saved = os_audio_get_wet();
    m->mon_req = false;
    swing_tame_hot_mic_input();
    (void)os_audio_set_monitor(false);
    (void)os_audio_set_wet(false);

    ui_statusbar_show(false);
    ui_hints_show(true);
    m->hint_state = m->state;
    m->hint_swing = m->swing_on;
    m->hint_bounce = m->bouncing;
    swing_hints(m);

    os_led_set_pattern(&swing_led_pattern, m);
    engine_set_active(false);
    return true;
}

static void swing_app_deinit(os_app_t* app) {
    swing_model_t* m = (swing_model_t*)os_app_get_model(app);
    if (!m) return;

    /* Restore user's WET and live-input monitor preferences on exit */
    (void)os_audio_set_wet(m->wet_saved);
    (void)os_audio_set_monitor(m->mon_saved);

    if (m->bouncing) {
        swing_bounce_stop(m, tape_get());
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
    if (m->space) {
        os_free(m->space);
        m->space = NULL;
    }
    if (m->loop_buf) {
        os_free(m->loop_buf);
        m->loop_buf = NULL;
    }
}

TAPP_APP("Swinging Piano", swing_model_t, swing_app_init, swing_app_deinit,
         .redraw = swing_redraw,
         .tick = swing_tick,
         .on_input = swing_input,
         .engine_cb = &swing_engine_cb);
