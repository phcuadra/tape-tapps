/*
 * disintegrate.c — William Basinski-inspired Disintegration Loop Standalone TAPP for tape!
 *
 * Ports phcuadra/Spotykach's destructive Disintegrate oxide-shedding algorithm to a
 * dedicated standalone tape! instrument:
 *   - Capture loops from live input (mic/line/WET/source) OR load recorded takes from tape
 *   - Destructive per-sample oxide erosion (C1-periodic flaking field, stride-1+3 HF diffusion,
 *     amplitude-dependent oxide shedding, odd-symmetric tape saturation, micro-crackle)
 *   - Varispeed & reverse playback (-2.00x .. +2.00x) with tape-motor inertia
 *   - Optional real-time Bounce-to-Tape onto free SD tape space (commits mark + cue)
 *   - Clean Q15 backup snapshot for instant fresh-loop restoration
 *   - Live ghost-vs-surviving waveform visualizer, oxide flake map, falling dust & RGB LEDs
 *
 * Controls:
 *   BTN1 press : Play / Stop loop
 *   BTN1 hold  : Volume overlay (out / fx send / in)
 *   BTN2 press : Capture live input into RAM loop / close loop & play
 *   BTN2 hold  : Bounce disintegrating output to free tape space (start / stop & commit)
 *   BTN3 press : Disintegrate erosion ON / OFF (smooth 30ms gate)
 *   BTN3 hold  : Menu (Load Tape Take, Restore Fresh Loop, Flip Dir, Speed presets, Seed, Input, Clear)
 *   BTN4 / 5   : Previous / Next encoder parameter (RATE, FLAKE, WEAR, SPEED)
 *   ENCODER    : Adjust selected parameter
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tapp_api.h"
#include "disintegrate_dsp.h"

TAPP_DECLARE_TYPE(AppTypeStandalone);

/* ----------------------------------------------------------------------------
 * Constants & Configuration
 * ------------------------------------------------------------------------- */
#define FRAMES_PER_POS   64u
#define WIN_NONE         0xFFFFFFFFu
#define MIN_LOOP_FR      (TAPP_SAMPLE_RATE / 10u)   /* 0.1 s minimum loop */
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

#define WAVE_BINS        192u                       /* 2px per bin across 384px */
#define FLAKE_BINS       64u
#define MAX_MENU_TAKES   32u
#define DUST_COUNT       12u

enum {
    ST_STOP = 0,
    ST_PLAY,
    ST_REC,
    ST_LOADING,
};

enum {
    ENC_RATE = 0,
    ENC_FLAKE,
    ENC_WEAR,
    ENC_SPEED,
    ENC_TOTAL,
};

#define DIS_BACKUP_PATH  "/fresh_loop.bin"

enum {
    BACKUP_NONE = 0,
    BACKUP_STORAGE_FILE,
    BACKUP_TAPE_SPAN,
};

/* ----------------------------------------------------------------------------
 * Model (kept small in internal RAM; audio buffers live in os_malloc / PSRAM)
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Transport & sync flags */
    volatile uint8_t  state;
    volatile bool     buf_locked;     /* audio thread mutes while UI mutates loop_buf */
    volatile bool     erode_on;       /* user disintegration gate toggle */
    volatile bool     rec_done_req;   /* capture reached max_frames or user stopped */
    volatile bool     tape_swapped;   /* set by tape_changed callback */
    volatile bool     restore_req;
    volatile bool     clear_req;
    volatile bool     new_seed_req;

    /* Audio engine active flag */
    volatile bool     eng_active;

    /* RAM loop buffer (4 B/frame aligned 32-bit stereo Q2.14 in PSRAM via os_malloc) */
    dis_frame_t*      loop_buf;
    float*            read_stage;     /* LOAD_CHUNK_FR * 2 floats for tape_read */
    uint32_t          max_frames;     /* allocated capacity in stereo frames */
    volatile uint32_t loop_len;       /* active loop length in stereo frames */
    uint32_t          backup_len;
    uint32_t          backup_src_frame;
    uint8_t           backup_mode;    /* BACKUP_NONE / BACKUP_STORAGE_FILE / BACKUP_TAPE_SPAN */
    bool              load_is_restore;
    uint8_t           restore_prev_state;

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

    /* Playhead (integer + fractional for drift-free varispeed) */
    volatile int32_t  head_i;
    float             head_f;
    volatile uint32_t passes;
    float             speed_target;
    float             speed_cur;
    float             play_gain;
    float             gate_cur;

    /* Core Spotykach Disintegrate DSP engine */
    dis_engine_t      dsp;

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

    /* Metering & LEDs */
    volatile float    peak_l;
    volatile float    peak_r;
    float             orig_rms;
    float             curr_rms;
    float             sweep_rms_acc;
    uint8_t           integrity_pct;

    /* Visualizer caches */
    uint8_t           wave_orig[WAVE_BINS];
    uint8_t           wave_curr[WAVE_BINS];
    uint8_t           flake_vis[FLAKE_BINS];
    uint16_t          wave_cursor;
    uint8_t           flake_cursor;

    /* Falling oxide dust particles (purely visual) */
    int16_t           dust_x[DUST_COUNT];
    int16_t           dust_y[DUST_COUNT];
    uint8_t           dust_life[DUST_COUNT];
    uint32_t          vis_rng;
    uint32_t          last_draw_ms;

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

    /* Menu & cached UI state */
    ui_menu_t*        menu;
    uint8_t           in_src;
    uint8_t           hint_state;
    bool              hint_erode;
    bool              hint_bounce;
    char              take_labels[MAX_MENU_TAKES][24];
    char              input_label[20];
    char              dir_label[24];
} dis_model_t;

/* Forward declarations */
static void dis_hints(const dis_model_t* m);
static void dis_finish_capture(dis_model_t* m);
static void dis_bounce_stop(dis_model_t* m, tape_t* t);
static void dis_sync_engine_active(dis_model_t* m);
static void dis_sync_monitor(dis_model_t* m);

/* ----------------------------------------------------------------------------
 * Small helpers (f32 only)
 * ------------------------------------------------------------------------- */
static inline uint32_t vis_rand(dis_model_t* m) {
    uint32_t x = m->vis_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    m->vis_rng = x ? x : 0xA3C59AC3u;
    return m->vis_rng;
}

static void apply_loop_splice(dis_frame_t* buf, uint32_t len) {
    if (!buf || len < 16u) return;
    uint32_t fade = SPLICE_FADE_FR;
    if (fade > (len >> 3)) fade = len >> 3;
    if (fade < 4u) return;

    const float inv = 1.0f / (float)fade;
    for (uint32_t i = 0; i < fade; ++i) {
        const float t = (float)i * inv;
        const float w = t * t * (3.0f - 2.0f * t); /* smoothstep */
        float hl = 0.0f, hr = 0.0f;
        dis_frame_unpack(buf[i], &hl, &hr);
        buf[i] = dis_frame_pack(hl * w, hr * w);

        const uint32_t tail = len - 1u - i;
        float tl = 0.0f, tr = 0.0f;
        dis_frame_unpack(buf[tail], &tl, &tr);
        buf[tail] = dis_frame_pack(tl * w, tr * w);
    }
}

static void build_waves_and_rms(dis_model_t* m) {
    const uint32_t len = m->loop_len;
    if (!m->loop_buf || len < 8u) return;

    /* Build initial waveform envelope & reference RMS */
    float energy = 0.0f;
    uint32_t energy_cnt = 0u;
    for (uint32_t b = 0; b < WAVE_BINS; ++b) {
        const uint32_t f0 = (uint32_t)(((uint32_t)b * len) / WAVE_BINS);
        uint32_t f1 = (uint32_t)(((uint32_t)(b + 1u) * len) / WAVE_BINS);
        if (f1 <= f0) f1 = f0 + 1u;
        if (f1 > len) f1 = len;

        uint32_t step = (f1 - f0) / 32u;
        if (step < 1u) step = 1u;

        float pk = 0.0f;
        for (uint32_t i = f0; i < f1; i += step) {
            float sl = 0.0f, sr = 0.0f;
            dis_frame_unpack(m->loop_buf[i], &sl, &sr);
            const float al = fabsf(sl);
            const float ar = fabsf(sr);
            const float a = (al > ar) ? al : ar;
            if (a > pk) pk = a;
            energy += 0.5f * (al * al + ar * ar);
            energy_cnt++;
        }
        uint32_t h = (uint32_t)(clampf(pk, 0.0f, 1.0f) * 255.0f);
        m->wave_orig[b] = (uint8_t)h;
        m->wave_curr[b] = (uint8_t)h;
    }

    m->orig_rms = (energy_cnt > 0u) ? sqrtf(energy / (float)energy_cnt) : 0.0f;
    m->curr_rms = m->orig_rms;
    m->integrity_pct = 100u;
    m->wave_cursor = 0u;
    m->sweep_rms_acc = 0.0f;

    /* Populate initial spatial oxide flake map */
    for (uint32_t k = 0; k < FLAKE_BINS; ++k) {
        const uint32_t idx = (uint32_t)(((uint32_t)k * len) / FLAKE_BINS);
        const float fl = dis_sample_flake_vis(&m->dsp, len, idx);
        m->flake_vis[k] = (uint8_t)(clampf(fl, 0.0f, 1.0f) * 255.0f);
    }
    m->flake_cursor = 0u;
}

static void sync_params_to_dsp(dis_model_t* m) {
    dis_set_intensity(&m->dsp, m->prm[ENC_RATE].val);
    dis_set_character(&m->dsp, m->prm[ENC_FLAKE].val);
    dis_set_mix(&m->dsp, m->prm[ENC_WEAR].val);

    float sp = m->prm[ENC_SPEED].val;
    if (fabsf(sp) < 0.02f) sp = 0.0f;
    else if (fabsf(sp - 1.0f) < 0.02f) sp = 1.0f;
    else if (fabsf(sp + 1.0f) < 0.02f) sp = -1.0f;
    m->speed_target = sp;
}

static void dis_sync_engine_active(dis_model_t* m) {
    if (!m->eng_active) {
        m->eng_active = true;
        engine_set_active(true);
    }
}

/*
 * Keep live input monitoring and WET FX-chain pass-through strictly OFF unless
 * actively capturing (ST_REC), preventing mic bleed or feedback out of speakers/headphones.
 * Note: os_audio_set_monitor(true) is declined by OS on Mic without headphones (and
 * remembered for later hotplug), so track mon_req to avoid calling it every tick.
 */
static void dis_sync_monitor(dis_model_t* m) {
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

/*
 * When built-in Mic is selected, gently cap hot hardware input gain to 0.35 (~-9 dB)
 * via param_update_fast() so both codec monitoring and capture sit at a comfortable level.
 */
static void dis_tame_hot_mic_input(void) {
    if (os_audio_get_input() != 1u) return;
    mixer_t* mx = mixer_get();
    params_t* in_vol = mx ? mixer_get_input_vol(mx) : NULL;
    if (in_vol && in_vol->val > 0.35f) {
        in_vol->target = 0.35f;
        param_update_fast(in_vol, NULL);
    }
}

/* ----------------------------------------------------------------------------
 * Tape free-space placement for Bounce-to-Tape (mirrors examples/recorder.c)
 * ------------------------------------------------------------------------- */
static bool bounce_free_region(tape_t* t, uint32_t* start_fr, uint32_t* limit_fr) {
    if (!t) return false;
    const uint32_t cap = tape_size(t) * FRAMES_PER_POS;
    if (cap < MIN_BOUNCE_FR) return false;

    const uint32_t n = tape_marks_count(t);
    uint32_t end = 0u;
    for (uint32_t i = 0; i < n; ++i) {
        uint32_t s = 0u, e = 0u;
        if (tape_marks_get(t, i, &s, &e) && e > end) end = e;
    }
    const uint32_t start = end ? (end + BOUNCE_GUARD_FR) : 0u;
    if (start >= cap || (cap - start) < MIN_BOUNCE_FR) return false;

    *start_fr = start;
    *limit_fr = cap;
    return true;
}

static void dis_bounce_start(dis_model_t* m) {
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
    dis_sync_engine_active(m);
    dis_hints(m);
    ui_notify_toast(NotifyInfo, "Bouncing to tape", 1800u);
}

static void dis_bounce_stop(dis_model_t* m, tape_t* t) {
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
        ui_notify_toast(NotifySuccess, "Bounce saved", 1800u);
    }

    dis_sync_engine_active(m);
    dis_hints(m);
}

/* ----------------------------------------------------------------------------
 * Audio Engine (real-time, zero blocking, f32 only)
 * ------------------------------------------------------------------------- */
static inline void bounce_win_request(dis_model_t* m, uint32_t win) {
    tape_buffer_update(m->wb, win * TAPE_WINDOW_POS, true);
}

static void dis_process(void* restrict ctx, mixer_t* restrict mix) {
    dis_model_t* m = (dis_model_t*)ctx;
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
        m->gate_cur  = 0.0f;
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
    const float gate_target = (st == ST_PLAY && m->erode_on && m->loop_len >= 8u) ? 1.0f : 0.0f;
    const float bounce_target = (bouncing && !m->bounce_stop_req) ? 1.0f : 0.0f;
    const float play_step = 1.0f / 720.0f;       /* ~15 ms anti-click */
    const float gate_step = 1.0f / 1440.0f;      /* ~30 ms SoftSwitch */
    const float bounce_step = 1.0f / (float)BOUNCE_FADE_FR;

    float* bbuf = bouncing ? tape_buffer_get_ptr(m->wb) : NULL;
    float pl = 0.0f;
    float pr = 0.0f;

    dis_frame_t* lbuf = m->loop_buf;
    const uint32_t len = m->loop_len;

    for (uint32_t i = 0; i < fs; i += 2) {
        float ol = 0.0f;
        float orr = 0.0f;

        if (st == ST_REC && lbuf && in) {
            const float itrim = (m->in_src == 1u) ? m->mic_trim.val : 1.0f;
            float il = in[i] * itrim;
            float ir = in[i + 1] * itrim;

            /* 85 Hz chassis-thump & DC high-pass on built-in Mic (warms up during holdoff) */
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
                    lbuf[rf] = dis_frame_pack(dis_soft_clip_guard(il * fg),
                                              dis_soft_clip_guard(ir * fg));
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
            /* Smooth tape motor speed & play/gate envelopes */
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

            float gg = m->gate_cur;
            if (gg < gate_target) {
                gg += gate_step;
                if (gg > gate_target) gg = gate_target;
            } else if (gg > gate_target) {
                gg -= gate_step;
                if (gg < gate_target) gg = gate_target;
            }
            m->gate_cur = gg;
            dis_set_gate(&m->dsp, gg);

            if (pg > 0.0001f && lbuf && len >= 8u) {
                const int32_t size = (int32_t)len;
                int32_t i0 = m->head_i;
                if (i0 < 0 || i0 >= size) i0 = 0;
                const int32_t i1 = (i0 + 1 < size) ? (i0 + 1) : 0;
                const float frac = m->head_f;

                /* Linearly interpolated stereo playback from 32-bit packed frames */
                float l0 = 0.0f, r0 = 0.0f, l1 = 0.0f, r1 = 0.0f;
                dis_frame_unpack(lbuf[i0], &l0, &r0);
                dis_frame_unpack(lbuf[i1], &l1, &r1);
                float sl = dis_soft_clip_guard(l0 + frac * (l1 - l0));
                float sr = dis_soft_clip_guard(r0 + frac * (r1 - r0));

                ol = sl * pg;
                orr = sr * pg;

                /* Advance integer + fractional playhead */
                float nf = frac + m->speed_cur;
                int32_t step = 0;
                while (nf >= 1.0f) { nf -= 1.0f; step++; }
                while (nf < 0.0f)  { nf += 1.0f; step--; }

                int32_t curr_i = i0 + step;
                if (curr_i >= size) {
                    curr_i -= size;
                    m->passes++;
                } else if (curr_i < 0) {
                    curr_i += size;
                    m->passes++;
                }
                m->head_i = curr_i;
                m->head_f = nf;

                /* Destructively erode frames the playhead just stepped past */
                if (step != 0 && gg > 0.0001f) {
                    dis_step_erode(&m->dsp, lbuf, len, i0, step, 1.0f);
                }
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
        out[i] = out_l;
        out[i + 1] = out_r;
        if (send) {
            send[i]     += out_l * vf;
            send[i + 1] += out_r * vf;
        }
    }

    if (fabsf(vout_tgt - vo) < 1.0e-5f) vo = vout_tgt;
    if (fabsf(vfx_tgt  - vf) < 1.0e-5f) vf = vfx_tgt;
    m->vout_cur = vo;
    m->vfx_cur  = vf;

    m->peak_l = (pl > m->peak_l) ? pl : (m->peak_l * 0.85f);
    m->peak_r = (pr > m->peak_r) ? pr : (m->peak_r * 0.85f);
}

static void dis_eng_active(void* ctx, bool state) {
    ((dis_model_t*)ctx)->eng_active = state;
}

static void dis_tape_changed(void* ctx) {
    ((dis_model_t*)ctx)->tape_swapped = true;
}

static const engine_callbacks_t dis_engine_cb = {
    .process = dis_process,
    .tape_changed = dis_tape_changed,
    .active = dis_eng_active,
};

/* ----------------------------------------------------------------------------
 * RGB LED Pattern (runs on OS LED task)
 * Mirrors Spotykach's Disintegrate cyan-teal (#1CE8D0) oxide shimmer
 * ------------------------------------------------------------------------- */
static void dis_led_step(const os_led_t* led, const void* ctx) {
    const uint8_t step = os_led_get_step(led);
    const dis_model_t* m = (const dis_model_t*)ctx;
    if (!m) return;

    if (m->state == ST_REC) {
        /* Soft pulsing crimson while capturing loop */
        const int tri = (step & 32u) ? (63 - (int)(step & 63u)) : (int)(step & 63u);
        const uint8_t r = (uint8_t)(16 + tri * 2);
        for (uint8_t i = 0; i < 4u; ++i) hal_led_set_rgb(i, r, 2u, 2u);
        return;
    }

    if (m->state == ST_PLAY && m->loop_len >= 8u) {
        const float health = (float)m->integrity_pct * 0.01f;
        const float act = clampf(m->dsp.activity * 2.5f, 0.0f, 1.0f);
        const uint8_t head_led = (uint8_t)(((uint32_t)(uint32_t)m->head_i * 4u) / m->loop_len) & 3u;

        for (uint8_t i = 0; i < 4u; ++i) {
            if (m->bouncing && i == 0u) {
                /* LED 0 pulses amber/red while bouncing to tape */
                const uint8_t pulse = ((step & 16u) == 0u) ? 55u : 18u;
                hal_led_set_rgb(0u, pulse, (uint8_t)(pulse >> 2), 0u);
                continue;
            }

            float br = (i == head_led) ? 1.0f : 0.30f;
            if (m->erode_on) {
                /* Cyan/teal dims with oxide flaking & fades as tape dissolves */
                br *= (1.0f - 0.38f * act) * (0.18f + 0.82f * health);
                const uint8_t r = (uint8_t)(7.0f * br);
                const uint8_t g = (uint8_t)(58.0f * br);
                const uint8_t b = (uint8_t)(52.0f * br);
                hal_led_set_rgb(i, r, g, b);
            } else {
                /* Clean playback: soft neutral warm-white chase */
                const uint8_t v = (uint8_t)(38.0f * br);
                hal_led_set_rgb(i, v, v, (uint8_t)(v + 6u));
            }
        }
        return;
    }

    /* Idle / stopped */
    for (uint8_t i = 0; i < 4u; ++i) {
        if (m->bouncing && i == 0u) {
            hal_led_set_rgb(0u, 36u, 8u, 0u);
        } else {
            hal_led_set_rgb(i, 0u, 3u, 3u);
        }
    }
}

static led_pattern_t dis_led_pattern = {
    .callback = dis_led_step,
    .steps = 255u,
    .rate = 35u,
    .loop = true,
};

/* ----------------------------------------------------------------------------
 * Loop capture finalization, restore, clear & take loading
 * ------------------------------------------------------------------------- */
static void dis_finish_capture(dis_model_t* m) {
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
        dis_sync_monitor(m);
        dis_sync_engine_active(m);
        dis_hints(m);
        ui_notify_toast(NotifyWarning, "Loop too short", 1800u);
        return;
    }

    apply_loop_splice(m->loop_buf, captured);
    m->loop_len = captured;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->passes = 0u;
    m->play_gain = 0.0f;

    dis_reset_tape(&m->dsp);
    build_waves_and_rms(m);

    /* Spill pristine capture to app-private SD storage (zero RAM backup overhead) */
    const size_t bytes = (size_t)captured * sizeof(dis_frame_t);
    if (storage_write_file(DIS_BACKUP_PATH, m->loop_buf, bytes) > 0u) {
        m->backup_mode = BACKUP_STORAGE_FILE;
        m->backup_len = captured;
    } else {
        m->backup_mode = BACKUP_NONE;
        m->backup_len = 0u;
    }

    m->state = ST_PLAY;
    m->buf_locked = false;
    dis_sync_monitor(m);
    dis_sync_engine_active(m);
    dis_hints(m);
}

static void dis_reset_restored_state(dis_model_t* m, uint32_t len) {
    m->loop_len = len;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->passes = 0u;
    m->play_gain = 0.0f;
    m->dsp.drift_phase = 0.0f;
    m->dsp.detail_phase = 17.3f;
    m->dsp.dust_lp_l = 0.0f;
    m->dsp.dust_lp_r = 0.0f;
    m->dsp.activity = 0.0f;

    memcpy(m->wave_curr, m->wave_orig, sizeof(m->wave_curr));
    m->curr_rms = m->orig_rms;
    m->integrity_pct = 100u;
    m->wave_cursor = 0u;
    m->sweep_rms_acc = 0.0f;
}

static void dis_do_restore(dis_model_t* m) {
    if (m->backup_mode == BACKUP_NONE || !m->loop_buf || m->backup_len < 8u ||
        m->backup_len > m->max_frames) {
        ui_notify_toast(NotifyWarning, "No backup to restore", 1800u);
        return;
    }

    const uint32_t len = m->backup_len;

    if (m->backup_mode == BACKUP_TAPE_SPAN) {
        tape_t* t = tape_get();
        if (!t || !m->read_stage) {
            ui_notify_toast(NotifyWarning, "Tape unavailable", 1800u);
            return;
        }
        m->buf_locked = true;
        m->restore_prev_state = (m->state == ST_PLAY) ? ST_PLAY : ST_STOP;
        m->load_is_restore = true;
        m->load_src_frame = m->backup_src_frame;
        m->load_total = len;
        m->load_done = 0u;
        m->state = ST_LOADING;
        dis_sync_engine_active(m);
        dis_hints(m);
        return;
    }

    m->buf_locked = true;
    const size_t bytes = (size_t)len * sizeof(dis_frame_t);
    if (storage_read_file(DIS_BACKUP_PATH, m->loop_buf, bytes) == 0u) {
        m->buf_locked = false;
        ui_notify_toast(NotifyError, "Backup read failed", 1800u);
        return;
    }

    dis_reset_restored_state(m, len);
    m->buf_locked = false;
    ui_notify_toast(NotifySuccess, "Fresh loop restored", 1800u);
}

static void dis_do_clear(dis_model_t* m) {
    m->buf_locked = true;
    m->state = ST_STOP;
    m->loop_len = 0u;
    m->backup_len = 0u;
    m->backup_mode = BACKUP_NONE;
    m->load_is_restore = false;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->passes = 0u;
    m->orig_rms = 0.0f;
    m->curr_rms = 0.0f;
    m->integrity_pct = 0u;
    memset(m->wave_orig, 0, sizeof(m->wave_orig));
    memset(m->wave_curr, 0, sizeof(m->wave_curr));
    memset(m->flake_vis, 0, sizeof(m->flake_vis));
    m->buf_locked = false;
    dis_sync_engine_active(m);
    dis_hints(m);
    ui_notify_toast(NotifyInfo, "Loop cleared", 1800u);
}

static void dis_start_take_load(dis_model_t* m, uint32_t take_idx) {
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
    m->load_is_restore = false;
    m->load_src_frame = s;
    m->load_total = len;
    m->load_done = 0u;
    m->state = ST_LOADING;
    dis_sync_engine_active(m);
    dis_hints(m);
}

static void dis_step_take_load(dis_model_t* m) {
    tape_t* t = tape_get();
    if (!t || !m->read_stage || !m->loop_buf) {
        m->state = ST_STOP;
        m->load_is_restore = false;
        m->buf_locked = false;
        return;
    }

    uint32_t rem = m->load_total - m->load_done;
    uint32_t chunk = (rem > LOAD_CHUNK_FR) ? LOAD_CHUNK_FR : rem;
    const uint32_t got = tape_read(t, m->load_src_frame + m->load_done, m->read_stage, chunk);
    if (got == 0u) {
        m->state = ST_STOP;
        m->load_is_restore = false;
        m->buf_locked = false;
        ui_notify_toast(NotifyError, "Tape read failed", 1800u);
        return;
    }

    for (uint32_t i = 0; i < got; ++i) {
        m->loop_buf[m->load_done + i] = dis_frame_pack(
            dis_soft_clip_guard(m->read_stage[i * 2u]),
            dis_soft_clip_guard(m->read_stage[i * 2u + 1u]));
    }
    m->load_done += got;

    if (m->load_done >= m->load_total) {
        const uint32_t len = m->load_done;
        apply_loop_splice(m->loop_buf, len);

        if (m->load_is_restore) {
            m->load_is_restore = false;
            dis_reset_restored_state(m, len);
            m->state = m->restore_prev_state;
            m->buf_locked = false;
            dis_sync_engine_active(m);
            dis_hints(m);
            ui_notify_toast(NotifySuccess, "Fresh loop restored", 1800u);
        } else {
            m->loop_len = len;
            m->head_i = 0;
            m->head_f = 0.0f;
            m->passes = 0u;
            m->play_gain = 0.0f;

            m->backup_mode = BACKUP_TAPE_SPAN;
            m->backup_src_frame = m->load_src_frame;
            m->backup_len = len;

            dis_reset_tape(&m->dsp);
            build_waves_and_rms(m);

            m->state = ST_PLAY;
            m->buf_locked = false;
            dis_sync_engine_active(m);
            dis_hints(m);
            ui_notify_toast(NotifySuccess, "Take loaded", 1800u);
        }
    }
}

/* ----------------------------------------------------------------------------
 * Menu System (BTN3 Hold)
 * ------------------------------------------------------------------------- */
static void menu_act_load_take(ui_menu_t* menu, ui_menu_node_t* entry) {
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    const uint16_t idx = UI_PICK_IDX(entry);
    ui_menu_hide(menu);
    dis_start_take_load(m, (uint32_t)idx);
}

static void menu_takes_builder(ui_menu_t* menu) {
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    tape_t* t = tape_get();
    const uint32_t n = t ? tape_marks_count(t) : 0u;

    if (n == 0u) {
        ui_menu_add(menu, UI_ACTION("(no tape takes)", NULL));
        return;
    }

    const uint32_t count = (n > MAX_MENU_TAKES) ? MAX_MENU_TAKES : n;
    for (uint32_t i = 0; i < count; ++i) {
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

static void menu_act_restore(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    m->restore_req = true;
    ui_menu_hide(menu);
}

static void menu_act_flip_dir(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    float sp = m->prm[ENC_SPEED].val;
    sp = (fabsf(sp) < 0.05f) ? -1.0f : -sp;
    m->prm[ENC_SPEED].val = m->prm[ENC_SPEED].target = clampf(sp, -2.0f, 2.0f);
    sync_params_to_dsp(m);
    ui_menu_hide(menu);
}

static void menu_act_speed_1x(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    const float sign = (m->prm[ENC_SPEED].val < 0.0f) ? -1.0f : 1.0f;
    m->prm[ENC_SPEED].val = m->prm[ENC_SPEED].target = sign * 1.0f;
    sync_params_to_dsp(m);
    ui_menu_hide(menu);
}

static void menu_act_speed_half(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    const float sign = (m->prm[ENC_SPEED].val < 0.0f) ? -1.0f : 1.0f;
    m->prm[ENC_SPEED].val = m->prm[ENC_SPEED].target = sign * 0.5f;
    sync_params_to_dsp(m);
    ui_menu_hide(menu);
}

static void menu_act_new_seed(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    m->new_seed_req = true;
    ui_menu_hide(menu);
}

static void menu_act_input(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_switch_input();
    m->mon_req = false;
    dis_tame_hot_mic_input();
    dis_sync_monitor(m);
    ui_menu_rebuild(menu);
}

static void menu_act_clear(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    m->clear_req = true;
    ui_menu_hide(menu);
}

static void menu_root_builder(ui_menu_t* menu) {
    dis_model_t* m = (dis_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_get_input();
    snprintf(m->input_label, sizeof(m->input_label), "Input: %s",
             m->in_src == 1u ? "Mic" : "Line");
    snprintf(m->dir_label, sizeof(m->dir_label), "Direction: %s",
             m->prm[ENC_SPEED].val < 0.0f ? "REV -> FWD" : "FWD -> REV");

    ui_menu_add(menu, UI_ENTRY("Load Tape Take", menu_takes_builder));
    ui_menu_add(menu, UI_ACTION("Restore Fresh Loop", menu_act_restore));
    ui_menu_add(menu, UI_ACTION(m->dir_label, menu_act_flip_dir));
    ui_menu_add(menu, UI_ACTION("Speed: 1.00x", menu_act_speed_1x));
    ui_menu_add(menu, UI_ACTION("Speed: 0.50x (Half)", menu_act_speed_half));
    ui_menu_add(menu, UI_ACTION("New Oxide Seed", menu_act_new_seed));
    ui_menu_add(menu, UI_ACTION(m->input_label, menu_act_input));
    ui_menu_add(menu, UI_PARAM("Mic Trim", MenuNodeParam, &m->mic_trim));
    ui_menu_add(menu, UI_ACTION("Clear Loop", menu_act_clear));
}

/* ----------------------------------------------------------------------------
 * Hint Band & Volume Overlay
 * ------------------------------------------------------------------------- */
static void dis_hints(const dis_model_t* m) {
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
        [HintPress3] = {"erode", gfx_icon(GfxIconFx), NULL,
                        m->erode_on ? HINT_ACTIVE : 0u},
        [HintHold3]  = {"menu", gfx_icon(GfxIconMenu)},
        [HintPress4] = {NULL, gfx_icon(GfxIconPlayRev)},
        [HintPress5] = {NULL, gfx_icon(GfxIconPlay)},
    };
    ui_hints_set(hints);
}

static void dis_vol_show(dis_model_t* m) {
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

static void dis_vol_hide(dis_model_t* m) {
    m->vol_ui = false;
    dis_hints(m);
}

/* ----------------------------------------------------------------------------
 * Drawing (400x240 1-bit monochrome OLED, content y = 48..240)
 * ------------------------------------------------------------------------- */
#define WAVE_X       8
#define WAVE_Y       74
#define WAVE_W       384      /* 192 bins * 2px */
#define WAVE_H       110
#define WAVE_MID_Y   (WAVE_Y + 51)
#define WAVE_HALF_H  45
#define FLAKE_BAR_Y  (WAVE_Y + WAVE_H - 7)

#define CARD_Y       190
#define CARD_H       42
#define CARD_W       93
#define CARD_GAP     4

static void draw_header(gfx_t* gfx, const dis_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);

    /* State pill badge on left */
    const char* badge = "READY";
    bool inv_badge = false;
    if (m->state == ST_REC) {
        badge = "CAPTURE";
        inv_badge = (((os_tick_get() / 300u) & 1u) == 0u);
    } else if (m->state == ST_LOADING) {
        badge = "LOADING";
        inv_badge = true;
    } else if (m->state == ST_PLAY && m->erode_on) {
        badge = "ERODING";
        inv_badge = true;
    } else if (m->state == ST_PLAY) {
        badge = "LOOPING";
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

    /* Bounce indicator pill if actively writing to tape */
    if (m->bouncing) {
        const uint32_t bsec = (m->bounce_frame - m->bounce_start) / TAPP_SAMPLE_RATE;
        char btxt[16];
        snprintf(btxt, sizeof(btxt), "REC %02u:%02u", (unsigned)(bsec / 60u), (unsigned)(bsec % 60u));
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

    /* Loop length & pass counter */
    if (m->state == ST_REC) {
        const uint32_t ds = m->rec_frames / 4800u;
        const uint32_t max_ds = m->max_frames / 4800u;
        gfx_draw_strf(gfx, tx, 64, "%u.%us / %u.%us",
                      (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                      (unsigned)(max_ds / 10u), (unsigned)(max_ds % 10u));
    } else if (m->loop_len >= 8u) {
        const uint32_t ds = m->loop_len / 4800u;
        gfx_draw_strf(gfx, tx, 64, "%u.%us  PASS %u",
                      (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                      (unsigned)m->passes);
    } else {
        const uint32_t max_s = m->max_frames / TAPP_SAMPLE_RATE;
        gfx_draw_strf(gfx, tx, 64, "MAX %us RAM", (unsigned)max_s);
    }

    /* Right-aligned tape integrity readout */
    if (m->loop_len >= 8u && m->state != ST_REC && m->state != ST_LOADING) {
        char itxt[20];
        snprintf(itxt, sizeof(itxt), "OXIDE %3u%%", (unsigned)m->integrity_pct);
        const int iw = (int)gfx_get_str_width(gfx, itxt);
        gfx_draw_str(gfx, 392 - iw, 64, itxt);
    }
}

static void draw_waveform_stage(gfx_t* gfx, const dis_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_rect_r(gfx, WAVE_X, WAVE_Y, WAVE_W, WAVE_H, 3);

    /* Loading / restoring progress view */
    if (m->state == ST_LOADING) {
        gfx_set_font(gfx, gfx_nunito_semibold_14);
        const char* msg = m->load_is_restore ? "RESTORING FRESH LOOP..."
                                             : "LOADING TAPE TAKE...";
        const int mw = (int)gfx_get_str_width(gfx, msg);
        gfx_draw_str(gfx, SCREEN_CENTER_X - mw / 2, WAVE_Y + 46, msg);

        const uint32_t pct = (m->load_total > 0u) ? ((m->load_done * 100u) / m->load_total) : 0u;
        gfx_draw_rect_r(gfx, 60, WAVE_Y + 58, 280, 14, 3);
        const int fw = (int)((276u * pct) / 100u);
        if (fw > 0) gfx_draw_rect_fill_r(gfx, 62, WAVE_Y + 60, fw, 10, 2);
        return;
    }

    /* Live capture view */
    if (m->state == ST_REC) {
        const uint32_t pct = (m->max_frames > 0u) ? ((m->rec_frames * 1000u) / m->max_frames) : 0u;
        const int fill_w = (int)(((WAVE_W - 8u) * pct) / 1000u);
        if (fill_w > 0) {
            gfx_fill_rect_dithered(gfx, WAVE_X + 4, WAVE_Y + 4, fill_w, WAVE_H - 8, 2);
        }

        /* Live input level bar in center */
        const float pk = (m->peak_l > m->peak_r) ? m->peak_l : m->peak_r;
        const int mh = (int)(clampf(pk, 0.0f, 1.0f) * (float)WAVE_HALF_H);
        gfx_draw_hline_dithered(gfx, WAVE_X + 4, WAVE_MID_Y, WAVE_W - 8, 2);
        if (mh > 0 && fill_w > 2) {
            gfx_draw_rect_fill(gfx, WAVE_X + 4 + fill_w - 3, WAVE_MID_Y - mh, 3, mh * 2 + 1);
        }

        gfx_set_font(gfx, DepartureMono_Regular_10);
        const char* cap = "CAPTURING LIVE INPUT  [PRESS LOOP TO CLOSE]";
        const int cw = (int)gfx_get_str_width(gfx, cap);
        gfx_draw_str(gfx, SCREEN_CENTER_X - cw / 2, WAVE_Y + WAVE_H - 10, cap);
        return;
    }

    /* Empty state */
    if (m->loop_len < 8u) {
        /* Subtle twin tape-reel motif centered vertically in WAVE_H (82px) */
        const int cy = WAVE_Y + WAVE_H / 2;
        gfx_draw_circle(gfx, 152, cy, 15, GFX_DRAW_ALL);
        gfx_draw_circle(gfx, 152, cy, 5, GFX_DRAW_ALL);
        gfx_draw_circle(gfx, 248, cy, 15, GFX_DRAW_ALL);
        gfx_draw_circle(gfx, 248, cy, 5, GFX_DRAW_ALL);
        gfx_draw_hline_dithered(gfx, 152, cy - 15, 96, 3);
        gfx_draw_hline_dithered(gfx, 152, cy + 15, 96, 3);
        return;
    }

    /* Center zero-line */
    gfx_draw_hline_dithered(gfx, WAVE_X + 2, WAVE_MID_Y, WAVE_W - 4, 1);

    /* Ghost original waveform (halftone) + solid surviving waveform */
    for (uint32_t b = 0; b < WAVE_BINS; ++b) {
        const int x = WAVE_X + (int)(b * 2u);
        const int oh = ((int)m->wave_orig[b] * WAVE_HALF_H) / 255;
        const int ch = ((int)m->wave_curr[b] * WAVE_HALF_H) / 255;

        if (oh > ch && oh > 0) {
            gfx_draw_vline_dithered(gfx, x, WAVE_MID_Y - oh, oh * 2 + 1, 1);
        }
        if (ch > 0) {
            gfx_draw_vline(gfx, x, WAVE_MID_Y - ch, ch * 2 + 1);
            gfx_draw_vline(gfx, x + 1, WAVE_MID_Y - ch, ch * 2 + 1);
        }
    }

    /* Spatial oxide flaking strip along bottom of tape window */
    gfx_draw_hline_dithered(gfx, WAVE_X + 2, FLAKE_BAR_Y - 2, WAVE_W - 4, 2);
    for (uint32_t k = 0; k < FLAKE_BINS; ++k) {
        const uint8_t fv = m->flake_vis[k];
        if (fv < 18u) continue;
        const int fx = WAVE_X + 2 + (int)((k * (WAVE_W - 4u)) / FLAKE_BINS);
        const int fw = 5;
        const uint8_t shade = (uint8_t)(1u + ((uint32_t)fv * 6u) / 255u);
        gfx_fill_rect_dithered(gfx, fx, FLAKE_BAR_Y, fw, 5, shade);
    }

    /* Falling oxide dust pixels when actively eroding */
    if (m->erode_on && m->state == ST_PLAY) {
        for (uint32_t p = 0; p < DUST_COUNT; ++p) {
            if (m->dust_life[p] > 0u) {
                const int dx = m->dust_x[p];
                const int dy = m->dust_y[p];
                if (dx > WAVE_X + 1 && dx < WAVE_X + WAVE_W - 2 &&
                    dy > WAVE_Y + 2 && dy < FLAKE_BAR_Y - 1) {
                    gfx_draw_pixel(gfx, dx, dy);
                    if (m->dust_life[p] > 10u) gfx_draw_pixel(gfx, dx + 1, dy);
                }
            }
        }
    }

    /* Playhead cursor (XOR) + direction chevron at top */
    const int px = WAVE_X + 1 + (int)(((uint32_t)(uint32_t)m->head_i * (WAVE_W - 3u)) / m->loop_len);
    gfx_set_color(gfx, GFX_COLOR_XOR);
    gfx_draw_vline(gfx, px, WAVE_Y + 1, WAVE_H - 2);
    gfx_draw_vline(gfx, px + 1, WAVE_Y + 1, WAVE_H - 2);
    gfx_set_color(gfx, GFX_COLOR_SET);

    /* Tiny directional head cap */
    if (m->speed_cur < -0.02f) {
        gfx_draw_line(gfx, px + 3, WAVE_Y + 2, px - 2, WAVE_Y + 5);
        gfx_draw_line(gfx, px + 3, WAVE_Y + 8, px - 2, WAVE_Y + 5);
    } else if (m->speed_cur > 0.02f) {
        gfx_draw_line(gfx, px - 2, WAVE_Y + 2, px + 3, WAVE_Y + 5);
        gfx_draw_line(gfx, px - 2, WAVE_Y + 8, px + 3, WAVE_Y + 5);
    }
}

static void draw_param_cards(gfx_t* gfx, const dis_model_t* m) {
    for (uint8_t idx = 0; idx < ENC_TOTAL; ++idx) {
        const int x = 8 + (int)idx * (CARD_W + CARD_GAP);
        const bool sel = (m->enc_sel == idx);
        const params_t* p = &m->prm[idx];

        gfx_set_color(gfx, GFX_COLOR_SET);
        ui_draw_frame(gfx, x, CARD_Y, CARD_W, CARD_H, 3, sel);

        /* Parameter name & numeric readout */
        gfx_set_font(gfx, DepartureMono_Regular_10);
        gfx_draw_str(gfx, x + 6, CARD_Y + 14, p->name);

        char val_s[16];
        if (idx == ENC_SPEED) {
            const float sp = p->val;
            const char sign = (sp < -0.01f) ? '-' : '+';
            const int sp100 = (int)(fabsf(sp) * 100.0f + 0.5f);
            snprintf(val_s, sizeof(val_s), "%c%d.%02d", sign, sp100 / 100, sp100 % 100);
        } else {
            const int pc = (int)(p->val * 100.0f + 0.5f);
            snprintf(val_s, sizeof(val_s), "%d%%", pc);
        }
        const int vw = (int)gfx_get_str_width(gfx, val_s);
        gfx_draw_str(gfx, x + CARD_W - 6 - vw, CARD_Y + 14, val_s);

        /* Bar meter */
        const int bx = x + 6;
        const int by = CARD_Y + 23;
        const int bw = CARD_W - 12;
        const int bh = 12;
        gfx_draw_rect_r(gfx, bx, by, bw, bh, 2);

        if (idx == ENC_SPEED) {
            /* Bipolar bar centered at 0.00x */
            const int mid = bx + bw / 2;
            gfx_draw_vline_dithered(gfx, mid, by + 1, bh - 2, 4);
            const float norm = clampf(p->val * 0.5f, -1.0f, 1.0f);
            const int half = (bw - 4) / 2;
            const int span = (int)(fabsf(norm) * (float)half + 0.5f);
            if (span > 0) {
                if (norm >= 0.0f) {
                    gfx_draw_rect_fill(gfx, mid, by + 2, span, bh - 4);
                } else {
                    gfx_draw_rect_fill(gfx, mid - span, by + 2, span, bh - 4);
                }
            }
        } else {
            const int fill = (int)(clampf(p->val, 0.0f, 1.0f) * (float)(bw - 4) + 0.5f);
            if (fill > 0) {
                gfx_draw_rect_fill(gfx, bx + 2, by + 2, fill, bh - 4);
            }
        }
    }
}

static void dis_redraw(gfx_t* gfx, const os_app_t* app) {
    const dis_model_t* m = (const dis_model_t*)os_app_get_model(app);
    if (!m) return;

    draw_header(gfx, m);
    draw_waveform_stage(gfx, m);
    draw_param_cards(gfx, m);
}

/* ----------------------------------------------------------------------------
 * Tick (main UI thread — safe for tape_read, bookkeeping, waveform refresh)
 * ------------------------------------------------------------------------- */
static bool dis_tick(os_app_t* app) {
    dis_model_t* m = (dis_model_t*)os_app_get_model(app);
    tape_t* t = tape_get();

    /* Handle tape swap from main menu */
    if (m->tape_swapped) {
        m->tape_swapped = false;
        if (m->bouncing) {
            m->bouncing = false;
            m->bounce_tok = 0u;
        }
        if (m->backup_mode == BACKUP_TAPE_SPAN) {
            m->backup_mode = BACKUP_NONE;
            m->backup_len = 0u;
        }
        if (m->wb) tape_buffer_clear_all_buffers(m->wb);
        m->bounce_cur_win = WIN_NONE;
        dis_hints(m);
    }

    /* Handle deferred menu requests */
    if (m->restore_req) {
        m->restore_req = false;
        dis_do_restore(m);
    }
    if (m->clear_req) {
        m->clear_req = false;
        dis_do_clear(m);
    }
    if (m->new_seed_req) {
        m->new_seed_req = false;
        dis_reset_tape(&m->dsp);
        m->flake_cursor = 0u;
        ui_notify_toast(NotifyInfo, "New oxide seed", 1800u);
    }

    /* Handle chunked tape take loading */
    if (m->state == ST_LOADING) {
        dis_step_take_load(m);
        return true;
    }

    /* Handle finished live capture */
    if (m->state == ST_REC && m->rec_done_req) {
        dis_finish_capture(m);
    }

    /* Handle bounce-to-tape stop / end-of-tape */
    if (m->bouncing && t) {
        if (!m->bounce_stop_req && m->bounce_frame + BOUNCE_FADE_FR >= m->bounce_limit) {
            m->bounce_stop_req = true;
        }
        if ((m->bounce_stop_req && m->bounce_gain <= 0.001f) ||
            m->bounce_frame >= m->bounce_limit) {
            dis_bounce_stop(m, t);
        }
    }

    /* Keep input source, monitor gate (ON only while ST_REC), and engine active synced */
    m->in_src = os_audio_get_input();
    dis_sync_monitor(m);
    dis_sync_engine_active(m);

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

    /* Incremental waveform & integrity refresh (16 bins per tick) */
    if (m->loop_len >= 8u && !m->buf_locked && m->loop_buf &&
        m->state != ST_REC && m->state != ST_LOADING) {
        const uint32_t len = m->loop_len;
        for (uint32_t step = 0; step < 16u; ++step) {
            const uint32_t b = m->wave_cursor;
            const uint32_t f0 = (uint32_t)(((uint32_t)b * len) / WAVE_BINS);
            uint32_t f1 = (uint32_t)(((uint32_t)(b + 1u) * len) / WAVE_BINS);
            if (f1 <= f0) f1 = f0 + 1u;
            if (f1 > len) f1 = len;

            uint32_t stride = (f1 - f0) / 16u;
            if (stride < 1u) stride = 1u;

            float pk = 0.0f;
            float sq = 0.0f;
            uint32_t cnt = 0u;
            for (uint32_t i = f0; i < f1; i += stride) {
                float sl = 0.0f, sr = 0.0f;
                dis_frame_unpack(m->loop_buf[i], &sl, &sr);
                const float al = fabsf(sl);
                const float ar = fabsf(sr);
                const float a = (al > ar) ? al : ar;
                if (a > pk) pk = a;
                sq += 0.5f * (al * al + ar * ar);
                cnt++;
            }
            m->wave_curr[b] = (uint8_t)(clampf(pk, 0.0f, 1.0f) * 255.0f);
            if (cnt > 0u) m->sweep_rms_acc += sq / (float)cnt;

            m->wave_cursor++;
            if (m->wave_cursor >= WAVE_BINS) {
                m->curr_rms = sqrtf(m->sweep_rms_acc / (float)WAVE_BINS);
                if (m->orig_rms > 1.0e-4f) {
                    const float ratio = clampf(m->curr_rms / m->orig_rms, 0.0f, 1.0f);
                    m->integrity_pct = (uint8_t)(ratio * 100.0f + 0.5f);
                } else {
                    m->integrity_pct = 0u;
                }
                m->wave_cursor = 0u;
                m->sweep_rms_acc = 0.0f;
            }
        }

        /* Incremental spatial oxide flake strip update (8 bins per tick) */
        for (uint32_t step = 0; step < 8u; ++step) {
            const uint32_t k = m->flake_cursor;
            const uint32_t idx = (uint32_t)(((uint32_t)k * len) / FLAKE_BINS);
            const float fl = dis_sample_flake_vis(&m->dsp, len, idx);
            m->flake_vis[k] = (uint8_t)(clampf(fl, 0.0f, 1.0f) * 255.0f);
            m->flake_cursor = (uint8_t)((k + 1u) % FLAKE_BINS);
        }

        /* Update falling oxide dust particles */
        const int px = WAVE_X + 1 + (int)(((uint32_t)(uint32_t)m->head_i * (WAVE_W - 3u)) / len);
        for (uint32_t p = 0; p < DUST_COUNT; ++p) {
            if (m->dust_life[p] > 0u) {
                m->dust_y[p] += 2;
                m->dust_x[p] += (int16_t)((vis_rand(m) % 3u) - 1u);
                m->dust_life[p]--;
            } else if (m->erode_on && m->state == ST_PLAY && m->dsp.activity > 0.04f &&
                       (vis_rand(m) & 7u) == 0u) {
                m->dust_x[p] = (int16_t)(px + (int)(vis_rand(m) % 5u) - 2);
                m->dust_y[p] = (int16_t)(WAVE_MID_Y + (int)(vis_rand(m) % 24u) - 12);
                m->dust_life[p] = (uint8_t)(8u + (vis_rand(m) & 15u));
            }
        }
    }

    /* Refresh hint bar if state drifted */
    if (!m->vol_ui && (m->hint_state != m->state ||
                       m->hint_erode != m->erode_on ||
                       m->hint_bounce != m->bouncing)) {
        m->hint_state = m->state;
        m->hint_erode = m->erode_on;
        m->hint_bounce = m->bouncing;
        dis_hints(m);
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
static void dis_input(os_app_t* app, uint8_t btn, KeyStateEnum st) {
    dis_model_t* m = (dis_model_t*)os_app_get_model(app);
    if (btn == BTN_ENCODER) return;

    if (m->vol_ui) {
        if (st == KEY_STATE_RELEASED && btn == BTN1) dis_vol_hide(m);
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
                dis_sync_monitor(m);
                dis_sync_engine_active(m);
            } else if (m->state == ST_STOP && m->loop_len >= 8u) {
                m->state = ST_PLAY;
                dis_sync_monitor(m);
                dis_sync_engine_active(m);
            }
            break;

        case BTN2:
            if (m->state == ST_REC) {
                m->rec_btn_down = false;
                m->rec_holdoff_fr = 0u;
                m->rec_manual_stop = true;
                m->rec_done_req = true;
                dis_sync_monitor(m);
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
                dis_sync_monitor(m);
                dis_sync_engine_active(m);
            }
            break;

        case BTN3:
            m->erode_on = !m->erode_on;
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
        m->hint_erode = m->erode_on;
        m->hint_bounce = m->bouncing;
        dis_hints(m);
    } else if (st == KEY_STATE_HOLD) {
        if (btn == BTN1 && !m->vol_ui) {
            m->vol_ui = true;
            dis_vol_show(m);
        } else if (btn == BTN2 && m->state != ST_REC) {
            if (m->bouncing) {
                m->bounce_stop_req = true;
            } else {
                dis_bounce_start(m);
            }
        } else if (btn == BTN3) {
            if (m->menu) ui_menu_show(m->menu);
        }
    }
}

/* ----------------------------------------------------------------------------
 * Lifecycle (init / deinit)
 * ------------------------------------------------------------------------- */
static bool dis_app_init(os_app_t* app) {
    dis_model_t* m = (dis_model_t*)os_app_get_model(app);

    /*
     * 1. Initialize DSP & all `params_t` fields FIRST so `ui_menu_create()`
     *    sees valid parameter descriptors when `menu_root_builder` runs.
     */
    dis_init(&m->dsp, (float)TAPP_SAMPLE_RATE);
    m->vis_rng = os_tick_get() ^ 0x5A17C3E1u;

    /* Live encoder parameters */
    m->prm[ENC_RATE]  = (params_t)PARAM_RANGE("RATE",  0.25f,  0.00f, 1.00f, 0.02f);
    m->prm[ENC_FLAKE] = (params_t)PARAM_RANGE("FLAKE", 0.45f,  0.00f, 1.00f, 0.02f);
    m->prm[ENC_WEAR]  = (params_t)PARAM_RANGE("WEAR",  0.30f,  0.00f, 1.00f, 0.02f);
    m->prm[ENC_SPEED] = (params_t)PARAM_RANGE("SPEED", 1.00f, -2.00f, 2.00f, 0.05f);
    m->enc_sel = ENC_RATE;
    m->speed_cur = 1.0f;
    m->speed_target = 1.0f;
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
     *    large audio loop buffers so menu creation NEVER runs out of heap.
     */
    m->read_stage = (float*)os_malloc((size_t)LOAD_CHUNK_FR * 2u * sizeof(float));

    tape_t* t = tape_get();
    if (t) {
        m->wb = tape_buffer_init(t);
    }
    m->bounce_cur_win = WIN_NONE;

    m->menu = ui_menu_create(m, menu_root_builder);
    if (m->menu) {
        ui_menu_set_name(m->menu, "DISINTEGRATE");
        ui_menu_set_header(m->menu, true);
    }

    /*
     * 3. Allocate 4-byte/frame (192 KB/s at 48 kHz) stereo loop buffer with
     *    1-second granularity and zero RAM backup overhead:
     *    - On *tape!* hardware (~1.5–2.0 MB free PSRAM): 8s–10s (1.54–1.92 MB) fits!
     *    - On web emulator (~4.19 MB bump arena): 16s (3.07 MB) fits on first call!
     */
    static const uint32_t kCandSecs[] = {
        16u, 14u, 12u, 11u, 10u, 9u, 8u, 7u, 6u, 5u, 4u, 3u, 2u, 1u
    };
    const uint32_t n_cands = (uint32_t)(sizeof(kCandSecs) / sizeof(kCandSecs[0]));
    for (uint32_t c = 0u; c < n_cands; ++c) {
        const uint32_t frames = kCandSecs[c] * TAPP_SAMPLE_RATE;
        dis_frame_t* lb = (dis_frame_t*)os_malloc((size_t)frames * sizeof(dis_frame_t));
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
    dis_tame_hot_mic_input();
    (void)os_audio_set_monitor(false);
    (void)os_audio_set_wet(false);

    ui_statusbar_show(false);
    ui_hints_show(true);
    m->hint_state = m->state;
    m->hint_erode = m->erode_on;
    m->hint_bounce = m->bouncing;
    dis_hints(m);

    os_led_set_pattern(&dis_led_pattern, m);
    m->eng_active = true;
    engine_set_active(true);
    return m->loop_buf != NULL;
}

static void dis_app_deinit(os_app_t* app) {
    dis_model_t* m = (dis_model_t*)os_app_get_model(app);
    if (!m) return;

    /* Restore user's WET and live-input monitor preferences on exit */
    (void)os_audio_set_wet(m->wet_saved);
    (void)os_audio_set_monitor(m->mon_saved);

    if (m->bouncing) {
        dis_bounce_stop(m, tape_get());
    }
    if (m->menu) {
        ui_menu_destroy(m->menu);
        m->menu = NULL;
    }

    tape_timeline_show(false);

    if (m->backup_mode == BACKUP_STORAGE_FILE) {
        (void)storage_delete_file(DIS_BACKUP_PATH);
        (void)storage_delete_file(DIS_BACKUP_PATH ".bak");
        m->backup_mode = BACKUP_NONE;
    }

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

TAPP_APP("Disintegrate", dis_model_t, dis_app_init, dis_app_deinit,
         .redraw = dis_redraw,
         .tick = dis_tick,
         .on_input = dis_input,
         .engine_cb = &dis_engine_cb);
