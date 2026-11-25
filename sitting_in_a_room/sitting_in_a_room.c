/*
 * sitting_in_a_room.c — Alvin Lucier "I Am Sitting in a Room" (1969) Standalone TAPP for tape!
 *
 * Iteratively re-records a captured or loaded loop in-place through an acoustic room
 * modal resonator bank + early wall reflections + speaker/microphone spectral tilt EQ +
 * matched-envelope AGC & soft tape limiting. Over successive generations (GEN 1 -> GEN N),
 * consonant transients and speech formants dissolve into pure, singing standing-wave chords
 * determined by the physical geometry of the room.
 *
 * Controls:
 *   BTN1 press : Play / Stop loop
 *   BTN1 hold  : Volume overlay (out / fx send / in)
 *   BTN2 press : Capture live input into RAM loop / close loop & start GEN 1
 *   BTN2 hold  : Bounce evolving room generations to free SD tape space (start / stop & commit)
 *   BTN3 press : Re-Record Pass ON / OFF (smooth 30ms gate; OFF freezes current generation)
 *   BTN3 hold  : Menu (Load Tape Take, Restore Gen 1, Space Geometry, Direction, Speed, Input, Clear)
 *   BTN4 / 5   : Previous / Next encoder parameter (PASS, ROOM, RESO, TONE)
 *   ENCODER    : Adjust selected parameter
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tapp_api.h"
#include "lucier_dsp.h"

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

#define WAVE_BINS        192u                       /* 2px per bin across 384px */
#define MAX_MENU_TAKES   32u

enum {
    ST_STOP = 0,
    ST_PLAY,
    ST_REC,
    ST_LOADING,
};

enum {
    ENC_PASS = 0,
    ENC_ROOM,
    ENC_RESO,
    ENC_TONE,
    ENC_TOTAL,
};

/* ----------------------------------------------------------------------------
 * Model (kept small ~3.5 KB in internal RAM; audio & room buffers in os_malloc)
 * ------------------------------------------------------------------------- */
typedef struct {
    /* Transport & sync flags */
    volatile uint8_t  state;
    volatile bool     buf_locked;     /* audio thread mutes while UI mutates loop_buf */
    volatile bool     rerec_on;       /* iterative room re-recording active toggle */
    volatile bool     gen1_statement; /* play Gen 1 clean once while priming room AGC */
    volatile bool     rec_done_req;
    volatile bool     tape_swapped;
    volatile bool     restore_req;
    volatile bool     clear_req;

    /* Audio engine active flag */
    volatile bool     eng_active;

    /* RAM loop buffer + Q15 Generation-1 backup + room delay space (PSRAM via os_malloc) */
    lucier_frame_t*   loop_buf;
    int16_t*          backup_buf;     /* interleaved Q15 L/R Generation-1 snapshot */
    float*            read_stage;     /* LOAD_CHUNK_FR * 2 floats for tape_read */
    lucier_space_t*   room_space;     /* early reflections & Schroeder allpass workspace */
    uint32_t          max_frames;
    volatile uint32_t loop_len;
    uint32_t          backup_len;
    bool              has_backup;

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

    /* Playhead & generation tracking */
    volatile int32_t  head_i;
    float             head_f;
    volatile uint32_t generation;
    float             speed_target;
    float             speed_cur;
    float             play_gain;
    float             gate_cur;

    /* Core Lucier Room-Resonance + EQ DSP engine */
    lucier_engine_t   dsp;

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

    /* Metering, waveform & modal spectrum caches */
    volatile float    peak_l;
    volatile float    peak_r;
    float             orig_rms;
    float             curr_rms;
    float             curr_peak;
    float             sweep_rms_acc;
    float             sweep_peak;
    uint8_t           modal_pct;
    uint8_t           mode_vis[LUCIER_NUM_MODES];

    uint8_t           wave_orig[WAVE_BINS];
    uint8_t           wave_curr[WAVE_BINS];
    uint16_t          wave_cursor;

    /* Live performance parameters */
    params_t          prm[ENC_TOTAL];
    uint8_t           enc_sel;

    /* Volume overlay, mic trim & timeline params */
    params_t          vol_out;
    params_t          vol_fx;
    params_t          mic_trim;       /* app-local input trim (0.40x default when Mic) */
    volatile bool     vol_ui;
    bool              mon_saved;      /* user's monitor state saved at launch, restored at exit */
    bool              wet_saved;      /* user's WET state saved at launch, restored at exit */
    bool              mon_req;        /* true when monitor ON has been requested for ST_REC */
    params_t          tl_pos;
    params_t          tl_start;
    params_t          tl_end;

    /* Menu & cached UI state */
    ui_menu_t*        menu;
    uint8_t           in_src;
    uint8_t           hint_state;
    bool              hint_rerec;
    bool              hint_bounce;
    char              take_labels[MAX_MENU_TAKES][24];
    char              space_label[28];
    char              input_label[20];
    char              dir_label[24];
} sit_model_t;

/* Forward declarations */
static void sit_hints(const sit_model_t* m);
static void sit_finish_capture(sit_model_t* m);
static void sit_bounce_stop(sit_model_t* m, tape_t* t);
static void sit_sync_engine_active(sit_model_t* m);
static void sit_sync_monitor(sit_model_t* m);

/* ----------------------------------------------------------------------------
 * Small helpers (f32 & 32-bit int only)
 * ------------------------------------------------------------------------- */
static inline int16_t f32_to_q15(float x) {
    float c = clampf(x, -1.0f, 1.0f);
    float s = c * 32767.0f;
    return (int16_t)(s >= 0.0f ? (s + 0.5f) : (s - 0.5f));
}

static void apply_loop_splice(lucier_frame_t* buf, uint32_t len) {
    if (!buf || len < 16u) return;
    uint32_t fade = SPLICE_FADE_FR;
    if (fade > (len >> 3)) fade = len >> 3;
    if (fade < 4u) return;

    const float inv = 1.0f / (float)fade;
    for (uint32_t i = 0; i < fade; ++i) {
        const float t = (float)i * inv;
        const float w = t * t * (3.0f - 2.0f * t);
        buf[i].l *= w;
        buf[i].r *= w;
        const uint32_t tail = len - 1u - i;
        buf[tail].l *= w;
        buf[tail].r *= w;
    }
}

static void save_gen1_backup_and_waves(sit_model_t* m, bool write_q15) {
    const uint32_t len = m->loop_len;
    if (!m->loop_buf || len < 8u) return;

    if (write_q15 && m->backup_buf) {
        for (uint32_t i = 0; i < len; ++i) {
            m->backup_buf[i * 2u]      = f32_to_q15(m->loop_buf[i].l);
            m->backup_buf[i * 2u + 1u] = f32_to_q15(m->loop_buf[i].r);
        }
        m->backup_len = len;
        m->has_backup = true;
    }

    float energy = 0.0f;
    float global_pk = 0.0f;
    uint32_t energy_cnt = 0u;
    for (uint32_t b = 0; b < WAVE_BINS; ++b) {
        const uint32_t f0 = (b * len) / WAVE_BINS;
        uint32_t f1 = ((b + 1u) * len) / WAVE_BINS;
        if (f1 <= f0) f1 = f0 + 1u;
        if (f1 > len) f1 = len;

        uint32_t step = (f1 - f0) / 32u;
        if (step < 1u) step = 1u;

        float pk = 0.0f;
        for (uint32_t i = f0; i < f1; i += step) {
            const float al = fabsf(m->loop_buf[i].l);
            const float ar = fabsf(m->loop_buf[i].r);
            const float a = (al > ar) ? al : ar;
            if (a > pk) pk = a;
            energy += 0.5f * (al * al + ar * ar);
            energy_cnt++;
        }
        if (pk > global_pk) global_pk = pk;
        const uint32_t h = (uint32_t)(clampf(pk, 0.0f, 1.0f) * 255.0f);
        if (write_q15) m->wave_orig[b] = (uint8_t)h;
        m->wave_curr[b] = (uint8_t)h;
    }

    m->orig_rms = (energy_cnt > 0u) ? sqrtf(energy / (float)energy_cnt) : 0.16f;
    m->curr_rms = m->orig_rms;
    m->curr_peak = global_pk;
    m->wave_cursor = 0u;
    m->sweep_rms_acc = 0.0f;
    m->sweep_peak = 0.0f;
    m->modal_pct = 0u;
    memset(m->mode_vis, 0, sizeof(m->mode_vis));

    lucier_reset_state(&m->dsp, m->orig_rms);
}

static void sync_params_to_dsp(sit_model_t* m) {
    lucier_set_pass(&m->dsp, m->prm[ENC_PASS].val);
    lucier_set_room(&m->dsp, m->prm[ENC_ROOM].val);
    lucier_set_reso(&m->dsp, m->prm[ENC_RESO].val);

    float tn = m->prm[ENC_TONE].val;
    if (fabsf(tn) < 0.02f) tn = 0.0f;
    lucier_set_tone(&m->dsp, tn);
}

static void sit_sync_engine_active(sit_model_t* m) {
    const bool need = (m->state == ST_PLAY || m->state == ST_REC ||
                       m->bouncing || m->play_gain > 0.001f);
    if (need != m->eng_active) {
        engine_set_active(need);
    }
}

/*
 * Keep live input monitoring and WET FX-chain pass-through strictly OFF unless
 * actively capturing (ST_REC), preventing mic bleed or feedback out of speakers/headphones.
 */
static void sit_sync_monitor(sit_model_t* m) {
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

static void sit_tame_hot_mic_input(void) {
    if (os_audio_get_input() != 1u) return;
    mixer_t* mx = mixer_get();
    params_t* in_vol = mx ? mixer_get_input_vol(mx) : NULL;
    if (in_vol && in_vol->val > 0.35f) {
        in_vol->target = 0.35f;
        param_update_fast(in_vol, NULL);
    }
}

/* ----------------------------------------------------------------------------
 * Tape free-space placement for Bounce-to-Tape (DMA windows)
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

static void sit_bounce_start(sit_model_t* m) {
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
    sit_sync_engine_active(m);
    sit_hints(m);
    ui_notify_toast(NotifyInfo, "Bouncing room to tape", 1800u);
}

static void sit_bounce_stop(sit_model_t* m, tape_t* t) {
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
        ui_notify_toast(NotifySuccess, "Room bounce saved", 1800u);
    }

    sit_sync_engine_active(m);
    sit_hints(m);
}

/* ----------------------------------------------------------------------------
 * Audio Engine (real-time 48 kHz, zero blocking, f32 only)
 * ------------------------------------------------------------------------- */
static inline void bounce_win_request(sit_model_t* m, uint32_t win) {
    tape_buffer_update(m->wb, win * TAPE_WINDOW_POS, true);
}

static void sit_process(void* restrict ctx, mixer_t* restrict mix) {
    sit_model_t* m = (sit_model_t*)ctx;
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
        for (uint32_t i = 0; i < fs; i += 2) {
            out[i] = 0.0f;
            out[i + 1] = 0.0f;
        }
        return;
    }

    const float vout = m->vol_out.val;
    const float vfx = m->vol_fx.val;
    const float play_target = (st == ST_PLAY && m->loop_len >= 8u) ? 1.0f : 0.0f;
    const float bounce_target = (bouncing && !m->bounce_stop_req) ? 1.0f : 0.0f;
    const float play_step = 1.0f / 720.0f;       /* ~15 ms anti-click */
    const float gate_step = 1.0f / 1440.0f;      /* ~30 ms SoftSwitch */
    const float bounce_step = 1.0f / (float)BOUNCE_FADE_FR;

    float* bbuf = bouncing ? tape_buffer_get_ptr(m->wb) : NULL;
    float pl = 0.0f;
    float pr = 0.0f;

    lucier_frame_t* lbuf = m->loop_buf;
    const uint32_t len = m->loop_len;

    for (uint32_t i = 0; i < fs; i += 2) {
        float ol = 0.0f;
        float orr = 0.0f;

        if (st == ST_REC && lbuf && in) {
            const float itrim = (m->in_src == 1u) ? m->mic_trim.val : 1.0f;
            float il = in[i] * itrim;
            float ir = in[i + 1] * itrim;

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
                    lbuf[rf].l = lucier_clip_guard(il * fg);
                    lbuf[rf].r = lucier_clip_guard(ir * fg);
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

            const bool active_rerec = (st == ST_PLAY && m->rerec_on &&
                                       !m->gen1_statement && len >= 8u);
            const float gate_target = active_rerec ? 1.0f : 0.0f;
            float gg = m->gate_cur;
            if (gg < gate_target) {
                gg += gate_step;
                if (gg > gate_target) gg = gate_target;
            } else if (gg > gate_target) {
                gg -= gate_step;
                if (gg < gate_target) gg = gate_target;
            }
            m->gate_cur = gg;
            lucier_set_gate(&m->dsp, gg);

            if (pg > 0.0001f && lbuf && len >= 8u) {
                const int32_t size = (int32_t)len;
                int32_t i0 = m->head_i;
                if (i0 < 0 || i0 >= size) i0 = 0;
                const int32_t i1 = (i0 + 1 < size) ? (i0 + 1) : 0;
                const float frac = m->head_f;

                /* Linearly interpolated stereo read from current generation buffer */
                lucier_frame_t dry = {
                    lucier_clip_guard(lbuf[i0].l + frac * (lbuf[i1].l - lbuf[i0].l)),
                    lucier_clip_guard(lbuf[i0].r + frac * (lbuf[i1].r - lbuf[i0].r)),
                };

                /* Run room + EQ chain continuously (primes AGC during GEN 1; transforms when gg > 0) */
                const float wr_en = (gg > 0.0001f) ? 1.0f : 0.0f;
                const lucier_frame_t next_gen = lucier_process_frame(&m->dsp, dry, wr_en);

                ol  = next_gen.l * pg;
                orr = next_gen.r * pg;

                /* Advance integer + fractional playhead */
                float nf = frac + m->speed_cur;
                int32_t step = 0;
                while (nf >= 1.0f) { nf -= 1.0f; step++; }
                while (nf < 0.0f)  { nf += 1.0f; step--; }

                int32_t curr_i = i0 + step;
                bool wrapped = false;
                if (curr_i >= size) {
                    curr_i -= size;
                    wrapped = true;
                } else if (curr_i < 0) {
                    curr_i += size;
                    wrapped = true;
                }
                m->head_i = curr_i;
                m->head_f = nf;

                /* Overwrite frames just stepped past with newly re-recorded generation N+1 */
                if (step != 0 && gg > 0.0001f) {
                    lucier_commit_step(&m->dsp, lbuf, len, i0, step, next_gen, gg);
                }

                /* Advance generation counter on loop wrap when re-recording */
                if (wrapped) {
                    if (m->gen1_statement) {
                        m->gen1_statement = false;
                        if (m->rerec_on) {
                            m->generation = 2u;
                        }
                    } else if (m->rerec_on && m->dsp.pass_mix > 0.0001f) {
                        m->generation++;
                    }
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

        const float out_l = ol * vout;
        const float out_r = orr * vout;
        out[i] = out_l;
        out[i + 1] = out_r;
        if (send) {
            send[i]     += out_l * vfx;
            send[i + 1] += out_r * vfx;
        }
    }

    m->peak_l = (pl > m->peak_l) ? pl : (m->peak_l * 0.85f);
    m->peak_r = (pr > m->peak_r) ? pr : (m->peak_r * 0.85f);
}

static void sit_eng_active(void* ctx, bool state) {
    ((sit_model_t*)ctx)->eng_active = state;
}

static uint_fast8_t sit_eng_is_active(void* ctx) {
    const sit_model_t* m = (const sit_model_t*)ctx;
    return (m->state == ST_PLAY || m->state == ST_REC ||
            m->bouncing || m->play_gain > 0.001f) ? 1u : 0u;
}

static void sit_tape_changed(void* ctx) {
    ((sit_model_t*)ctx)->tape_swapped = true;
}

static const engine_callbacks_t sit_engine_cb = {
    .process = sit_process,
    .tape_changed = sit_tape_changed,
    .active = sit_eng_active,
    .is_active = sit_eng_is_active,
};

/* ----------------------------------------------------------------------------
 * RGB LED Pattern (runs on OS LED task)
 * Warm tungsten speech glow morphing into luminous indigo-gold standing waves
 * ------------------------------------------------------------------------- */
static void sit_led_step(const os_led_t* led, const void* ctx) {
    const uint8_t step = os_led_get_step(led);
    const sit_model_t* m = (const sit_model_t*)ctx;
    if (!m) return;

    if (m->state == ST_REC) {
        const int tri = (step & 32u) ? (63 - (int)(step & 63u)) : (int)(step & 63u);
        const uint8_t r = (uint8_t)(16 + tri * 2);
        for (uint8_t i = 0; i < 4u; ++i) hal_led_set_rgb(i, r, 2u, 2u);
        return;
    }

    if (m->state == ST_PLAY && m->loop_len >= 8u) {
        const float purity = (float)m->modal_pct * 0.01f;
        const uint8_t head_led = (uint8_t)(((uint32_t)m->head_i * 4u) / m->loop_len) & 3u;
        const int breath_tri = (step & 64u) ? (127 - (int)(step & 127u)) : (int)(step & 127u);
        const float breath = 0.65f + 0.35f * ((float)breath_tri * (1.0f / 64.0f));

        for (uint8_t i = 0; i < 4u; ++i) {
            if (m->bouncing && i == 0u) {
                const uint8_t pulse = ((step & 16u) == 0u) ? 56u : 18u;
                hal_led_set_rgb(0u, pulse, (uint8_t)(pulse >> 2), 0u);
                continue;
            }

            /* Pair adjacent room modes onto each of the 4 physical LEDs */
            const uint32_t mv = (uint32_t)m->mode_vis[i * 2u] + (uint32_t)m->mode_vis[i * 2u + 1u];
            const float mode_glow = clampf((float)mv * (1.0f / 320.0f), 0.15f, 1.0f);
            const float chase = (i == head_led) ? 1.0f : 0.35f;
            /* As modal purity increases, discrete playhead chase melts into continuous room breath */
            const float br = ((1.0f - purity) * chase + purity * (breath * mode_glow));

            if (m->rerec_on) {
                /* Morph from warm tungsten-amber (Gen 1) -> golden-violet resonant chord (Gen 20+) */
                const float r_f = (58.0f * (1.0f - 0.38f * purity) + 34.0f * purity) * br;
                const float g_f = (34.0f * (1.0f - 0.45f * purity) + 22.0f * purity) * br;
                const float b_f = (8.0f  * (1.0f - purity)         + 62.0f * purity) * br;
                hal_led_set_rgb(i, (uint8_t)r_f, (uint8_t)g_f, (uint8_t)b_f);
            } else {
                /* Frozen drone: serene architectural ice-gold & indigo */
                const uint8_t r = (uint8_t)(18.0f * br);
                const uint8_t g = (uint8_t)(30.0f * br);
                const uint8_t b = (uint8_t)(52.0f * br);
                hal_led_set_rgb(i, r, g, b);
            }
        }
        return;
    }

    for (uint8_t i = 0; i < 4u; ++i) {
        if (m->bouncing && i == 0u) {
            hal_led_set_rgb(0u, 36u, 8u, 0u);
        } else {
            hal_led_set_rgb(i, 3u, 2u, 4u);
        }
    }
}

static led_pattern_t sit_led_pattern = {
    .callback = sit_led_step,
    .steps = 255u,
    .rate = 35u,
    .loop = true,
};

/* ----------------------------------------------------------------------------
 * Loop capture finalization, restore, clear & take loading
 * ------------------------------------------------------------------------- */
static void sit_finish_capture(sit_model_t* m) {
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
        sit_sync_monitor(m);
        sit_sync_engine_active(m);
        sit_hints(m);
        ui_notify_toast(NotifyWarning, "Loop too short", 1800u);
        return;
    }

    apply_loop_splice(m->loop_buf, captured);
    m->loop_len = captured;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->generation = 1u;
    m->gen1_statement = true;
    m->play_gain = 0.0f;
    m->gate_cur = 0.0f;

    save_gen1_backup_and_waves(m, true);

    m->state = ST_PLAY;
    m->buf_locked = false;
    sit_sync_monitor(m);
    sit_sync_engine_active(m);
    sit_hints(m);
}

static void sit_do_restore(sit_model_t* m) {
    if (!m->has_backup || !m->backup_buf || !m->loop_buf || m->backup_len < 8u) {
        ui_notify_toast(NotifyWarning, "No Gen 1 backup", 1800u);
        return;
    }

    m->buf_locked = true;
    const uint32_t len = m->backup_len;
    const float inv = 1.0f / 32767.0f;
    for (uint32_t i = 0; i < len; ++i) {
        m->loop_buf[i].l = (float)m->backup_buf[i * 2u] * inv;
        m->loop_buf[i].r = (float)m->backup_buf[i * 2u + 1u] * inv;
    }

    m->loop_len = len;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->generation = 1u;
    m->gen1_statement = true;
    m->play_gain = 0.0f;
    m->gate_cur = 0.0f;

    memcpy(m->wave_curr, m->wave_orig, sizeof(m->wave_curr));
    save_gen1_backup_and_waves(m, false);

    if (m->state == ST_STOP) {
        m->state = ST_PLAY;
    }
    m->buf_locked = false;
    sit_sync_engine_active(m);
    sit_hints(m);
    ui_notify_toast(NotifySuccess, "Restored Gen 1", 1800u);
}

static void sit_do_clear(sit_model_t* m) {
    m->buf_locked = true;
    m->state = ST_STOP;
    m->loop_len = 0u;
    m->backup_len = 0u;
    m->has_backup = false;
    m->head_i = 0;
    m->head_f = 0.0f;
    m->generation = 0u;
    m->gen1_statement = false;
    m->orig_rms = 0.0f;
    m->curr_rms = 0.0f;
    m->modal_pct = 0u;
    memset(m->wave_orig, 0, sizeof(m->wave_orig));
    memset(m->wave_curr, 0, sizeof(m->wave_curr));
    memset(m->mode_vis, 0, sizeof(m->mode_vis));
    lucier_reset_state(&m->dsp, 0.16f);
    m->buf_locked = false;
    sit_sync_engine_active(m);
    sit_hints(m);
    ui_notify_toast(NotifyInfo, "Loop cleared", 1800u);
}

static void sit_start_take_load(sit_model_t* m, uint32_t take_idx) {
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
    sit_sync_engine_active(m);
    sit_hints(m);
}

static void sit_step_take_load(sit_model_t* m) {
    tape_t* t = tape_get();
    if (!t || !m->read_stage || !m->loop_buf) {
        m->state = ST_STOP;
        m->buf_locked = false;
        return;
    }

    uint32_t rem = m->load_total - m->load_done;
    uint32_t chunk = (rem > LOAD_CHUNK_FR) ? LOAD_CHUNK_FR : rem;
    const uint32_t got = tape_read(t, m->load_src_frame + m->load_done, m->read_stage, chunk);
    if (got == 0u) {
        m->state = ST_STOP;
        m->buf_locked = false;
        ui_notify_toast(NotifyError, "Tape read failed", 1800u);
        return;
    }

    for (uint32_t i = 0; i < got; ++i) {
        m->loop_buf[m->load_done + i].l = m->read_stage[i * 2u];
        m->loop_buf[m->load_done + i].r = m->read_stage[i * 2u + 1u];
    }
    m->load_done += got;

    if (m->load_done >= m->load_total) {
        const uint32_t len = m->load_done;
        apply_loop_splice(m->loop_buf, len);
        m->loop_len = len;
        m->head_i = 0;
        m->head_f = 0.0f;
        m->generation = 1u;
        m->gen1_statement = true;
        m->play_gain = 0.0f;
        m->gate_cur = 0.0f;

        save_gen1_backup_and_waves(m, true);

        m->state = ST_PLAY;
        m->buf_locked = false;
        sit_sync_engine_active(m);
        sit_hints(m);
        ui_notify_toast(NotifySuccess, "Loaded as Gen 1", 1800u);
    }
}

/* ----------------------------------------------------------------------------
 * Menu System (BTN3 Hold)
 * ------------------------------------------------------------------------- */
static void menu_act_load_take(ui_menu_t* menu, ui_menu_node_t* entry) {
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    const uint16_t idx = UI_PICK_IDX(entry);
    ui_menu_hide(menu);
    sit_start_take_load(m, (uint32_t)idx);
}

static void menu_takes_builder(ui_menu_t* menu) {
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
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
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    m->restore_req = true;
    ui_menu_hide(menu);
}

static void menu_act_cycle_space(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    const uint8_t next = (uint8_t)((m->dsp.geom_idx + 1u) % LUCIER_GEOM_COUNT);
    lucier_set_geometry(&m->dsp, next);
    ui_menu_rebuild(menu);
}

static void menu_act_flip_dir(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    m->speed_target = -m->speed_target;
    if (fabsf(m->speed_target) < 0.1f) m->speed_target = -1.0f;
    ui_menu_hide(menu);
}

static void menu_act_speed_1x(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 1.0f;
    ui_menu_hide(menu);
}

static void menu_act_speed_half(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 0.5f;
    ui_menu_hide(menu);
}

static void menu_act_speed_double(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    const float sign = (m->speed_target < 0.0f) ? -1.0f : 1.0f;
    m->speed_target = sign * 2.0f;
    ui_menu_hide(menu);
}

static void menu_act_input(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_switch_input();
    m->mon_req = false;
    sit_tame_hot_mic_input();
    sit_sync_monitor(m);
    ui_menu_rebuild(menu);
}

static void menu_act_clear(ui_menu_t* menu, ui_menu_node_t* entry) {
    (void)entry;
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    m->clear_req = true;
    ui_menu_hide(menu);
}

static void menu_root_builder(ui_menu_t* menu) {
    sit_model_t* m = (sit_model_t*)ui_menu_ctx(menu);
    m->in_src = os_audio_get_input();
    snprintf(m->space_label, sizeof(m->space_label), "Space: %s",
             lucier_geom_name(m->dsp.geom_idx));
    snprintf(m->input_label, sizeof(m->input_label), "Input: %s",
             m->in_src == 1u ? "Mic" : "Line");
    snprintf(m->dir_label, sizeof(m->dir_label), "Direction: %s",
             m->speed_target < 0.0f ? "REV -> FWD" : "FWD -> REV");

    ui_menu_add(menu, UI_ENTRY("Load Tape Take", menu_takes_builder));
    ui_menu_add(menu, UI_ACTION("Restore Gen 1", menu_act_restore));
    ui_menu_add(menu, UI_ACTION(m->space_label, menu_act_cycle_space));
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
static void sit_hints(const sit_model_t* m) {
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
        [HintPress3] = {"rerec", gfx_icon(GfxIconFx), NULL,
                        m->rerec_on ? HINT_ACTIVE : 0u},
        [HintHold3]  = {"menu", gfx_icon(GfxIconMenu)},
        [HintPress4] = {NULL, gfx_icon(GfxIconPlayRev)},
        [HintPress5] = {NULL, gfx_icon(GfxIconPlay)},
    };
    ui_hints_set(hints);
}

static void sit_vol_show(sit_model_t* m) {
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

static void sit_vol_hide(sit_model_t* m) {
    m->vol_ui = false;
    sit_hints(m);
}

/* ----------------------------------------------------------------------------
 * Drawing (400x240 1-bit monochrome OLED, content y = 48..240)
 * ------------------------------------------------------------------------- */
#define WAVE_X       8
#define WAVE_Y       74
#define WAVE_W       384      /* 192 bins * 2px */
#define WAVE_H       82
#define WAVE_MID_Y   (WAVE_Y + 32)
#define WAVE_HALF_H  27
#define MODE_BAR_Y   (WAVE_Y + WAVE_H - 14)

#define CARD_Y       162
#define CARD_H       40
#define CARD_W       93
#define CARD_GAP     4

/* Map frequency in [50..2200 Hz] onto horizontal log-frequency pixel coordinate */
static int freq_to_stage_x(float freq_hz) {
    const float f = clampf(freq_hz, 50.0f, 2200.0f);
    /* log10(2200 / 50) = log10(44.0) = 1.6434527f */
    const float norm = clampf(log10f(f * (1.0f / 50.0f)) * (1.0f / 1.6434527f), 0.0f, 1.0f);
    return (WAVE_X + 10) + (int)(norm * (float)(WAVE_W - 20) + 0.5f);
}

static void draw_header(gfx_t* gfx, const sit_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);

    const char* badge = "READY";
    bool inv_badge = false;
    if (m->state == ST_REC) {
        badge = "CAPTURE";
        inv_badge = (((os_tick_get() / 300u) & 1u) == 0u);
    } else if (m->state == ST_LOADING) {
        badge = "LOADING";
        inv_badge = true;
    } else if (m->state == ST_PLAY && m->gen1_statement) {
        badge = "GEN 1";
    } else if (m->state == ST_PLAY && m->rerec_on && m->dsp.pass_mix > 0.0001f) {
        badge = "REREC";
        inv_badge = true;
    } else if (m->state == ST_PLAY) {
        badge = "FROZEN";
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

    /* Loop duration + iconic Alvin Lucier GEN N counter */
    if (m->state == ST_REC) {
        const uint32_t ds = m->rec_frames / 4800u;
        const uint32_t max_ds = m->max_frames / 4800u;
        gfx_draw_strf(gfx, tx, 64, "%u.%us / %u.%us",
                      (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                      (unsigned)(max_ds / 10u), (unsigned)(max_ds % 10u));
    } else if (m->loop_len >= 8u) {
        const uint32_t ds = m->loop_len / 4800u;
        const uint32_t gen = (m->generation > 0u) ? m->generation : 1u;
        if (fabsf(m->speed_target - 1.0f) > 0.05f) {
            const char sgn = (m->speed_target < 0.0f) ? '-' : '+';
            const int sp10 = (int)(fabsf(m->speed_target) * 10.0f + 0.5f);
            gfx_draw_strf(gfx, tx, 64, "%u.%us GEN %u (%c%d.%dx)",
                          (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                          (unsigned)gen, sgn, sp10 / 10, sp10 % 10);
        } else {
            gfx_draw_strf(gfx, tx, 64, "%u.%us  GEN %u",
                          (unsigned)(ds / 10u), (unsigned)(ds % 10u),
                          (unsigned)gen);
        }
    } else {
        const uint32_t max_s = m->max_frames / TAPP_SAMPLE_RATE;
        gfx_draw_strf(gfx, tx, 64, "MAX %us RAM", (unsigned)max_s);
    }

    /* Right-aligned architectural geometry + modal resonance readout */
    if (m->state != ST_REC && m->state != ST_LOADING) {
        char itxt[24];
        if (m->loop_len >= 8u) {
            snprintf(itxt, sizeof(itxt), "%s %3u%%",
                     lucier_geom_short_name(m->dsp.geom_idx),
                     (unsigned)m->modal_pct);
        } else {
            const int f0_hz = (int)(m->dsp.f_base + 0.5f);
            snprintf(itxt, sizeof(itxt), "%s %dHz",
                     lucier_geom_short_name(m->dsp.geom_idx), f0_hz);
        }
        const int iw = (int)gfx_get_str_width(gfx, itxt);
        gfx_draw_str(gfx, 392 - iw, 64, itxt);
    }
}

static void draw_room_modes_strip(gfx_t* gfx, const sit_model_t* m) {
    const int strip_top = MODE_BAR_Y;
    const int base_y = WAVE_Y + WAVE_H - 3;

    /* Subtle separator & EQ tilt horizon */
    gfx_draw_hline_dithered(gfx, WAVE_X + 2, strip_top - 2, WAVE_W - 4, 2);

    const int tilt_px = (int)(m->prm[ENC_TONE].val * 3.0f);
    gfx_draw_line(gfx, WAVE_X + 6, base_y - 3 + tilt_px,
                  WAVE_X + WAVE_W - 7, base_y - 3 - tilt_px);
    gfx_set_color(gfx, GFX_COLOR_CLEAR);
    for (int x = WAVE_X + 6; x < WAVE_X + WAVE_W - 6; x += 3) {
        gfx_draw_vline(gfx, x, strip_top - 1, 13);
    }
    gfx_set_color(gfx, GFX_COLOR_SET);

    /* Render 8 architectural room eigenmodes along log-frequency axis */
    for (uint32_t k = 0; k < LUCIER_NUM_MODES; ++k) {
        const int mx = freq_to_stage_x(m->dsp.mode_freq[k]);
        const uint8_t mv = m->mode_vis[k];

        /* Mode frequency anchor tick on floor */
        gfx_draw_vline(gfx, mx, base_y - 1, 2);

        /* Standing-wave column height: idle tuning preview + live modal energy */
        int bar_h = 2 + (int)(clampf(m->dsp.mode_weight[k] * 0.45f, 0.0f, 1.0f) * 3.0f);
        const int live_h = ((int)mv * 11) / 255;
        if (live_h > bar_h) bar_h = live_h;
        if (bar_h > 11) bar_h = 11;

        if (mv > 45u) {
            gfx_draw_rect_fill(gfx, mx - 2, base_y - bar_h, 5, bar_h);
        } else {
            gfx_fill_rect_dithered(gfx, mx - 2, base_y - bar_h, 5, bar_h, 3);
        }
    }
}

static void draw_waveform_stage(gfx_t* gfx, const sit_model_t* m) {
    gfx_set_color(gfx, GFX_COLOR_SET);
    gfx_draw_rect_r(gfx, WAVE_X, WAVE_Y, WAVE_W, WAVE_H, 3);

    /* Loading progress view */
    if (m->state == ST_LOADING) {
        gfx_set_font(gfx, gfx_nunito_semibold_14);
        const char* msg = "LOADING TAKE AS GEN 1...";
        const int mw = (int)gfx_get_str_width(gfx, msg);
        gfx_draw_str(gfx, SCREEN_CENTER_X - mw / 2, WAVE_Y + 34, msg);

        const uint32_t pct = (m->load_total > 0u) ? ((m->load_done * 100u) / m->load_total) : 0u;
        gfx_draw_rect_r(gfx, 60, WAVE_Y + 46, 280, 14, 3);
        const int fw = (int)((276u * pct) / 100u);
        if (fw > 0) gfx_draw_rect_fill_r(gfx, 62, WAVE_Y + 48, fw, 10, 2);
        return;
    }

    /* Live capture view */
    if (m->state == ST_REC) {
        const uint32_t pct = (m->max_frames > 0u) ? ((m->rec_frames * 1000u) / m->max_frames) : 0u;
        const int fill_w = (int)(((WAVE_W - 8u) * pct) / 1000u);
        if (fill_w > 0) {
            gfx_fill_rect_dithered(gfx, WAVE_X + 4, WAVE_Y + 4, fill_w, WAVE_H - 8, 2);
        }

        const float pk = (m->peak_l > m->peak_r) ? m->peak_l : m->peak_r;
        const int mh = (int)(clampf(pk, 0.0f, 1.0f) * (float)WAVE_HALF_H);
        gfx_draw_hline_dithered(gfx, WAVE_X + 4, WAVE_MID_Y, WAVE_W - 8, 2);
        if (mh > 0 && fill_w > 2) {
            gfx_draw_rect_fill(gfx, WAVE_X + 4 + fill_w - 3, WAVE_MID_Y - mh, 3, mh * 2 + 1);
        }

        gfx_set_font(gfx, DepartureMono_Regular_10);
        const char* cap = "SPEAK OR PLAY PHRASE  [PRESS LOOP TO CLOSE]";
        const int cw = (int)gfx_get_str_width(gfx, cap);
        gfx_draw_str(gfx, SCREEN_CENTER_X - cw / 2, WAVE_Y + WAVE_H - 10, cap);
        return;
    }

    /* Empty state: architectural room speaker -> standing wave -> mic diagram */
    if (m->loop_len < 8u) {
        /* Speaker icon on left */
        gfx_draw_rect_fill(gfx, 118, WAVE_Y + 23, 5, 12);
        gfx_draw_line(gfx, 123, WAVE_Y + 23, 130, WAVE_Y + 17);
        gfx_draw_line(gfx, 123, WAVE_Y + 34, 130, WAVE_Y + 40);
        gfx_draw_vline(gfx, 130, WAVE_Y + 17, 24);

        /* Standing-wave antinodes across center */
        for (int i = 0; i < 128; ++i) {
            const uint16_t ph = (uint16_t)((i * 98304) / 128); /* 1.5 cycles = 3 antinodes */
            const int s = (int)lutsin_q15(ph);
            const int amp = ((s < 0 ? -s : s) * 12) >> 15;
            gfx_draw_pixel(gfx, 136 + i, WAVE_Y + 29 - amp);
            gfx_draw_pixel(gfx, 136 + i, WAVE_Y + 29 + amp);
        }

        /* Microphone icon on right */
        gfx_draw_disc(gfx, 274, WAVE_Y + 25, 4, GFX_DRAW_ALL);
        gfx_draw_vline(gfx, 274, WAVE_Y + 29, 11);
        gfx_draw_hline(gfx, 270, WAVE_Y + 40, 9);

        draw_room_modes_strip(gfx, m);

        gfx_set_font(gfx, DepartureMono_Regular_10);
        const char* hint = "REC VOICE/PHRASE  OR  HOLD REREC FOR TAKES";
        const int hw = (int)gfx_get_str_width(gfx, hint);
        gfx_draw_str(gfx, SCREEN_CENTER_X - hw / 2, MODE_BAR_Y - 5, hint);
        return;
    }

    /* Center zero-line */
    gfx_draw_hline_dithered(gfx, WAVE_X + 2, WAVE_MID_Y, WAVE_W - 4, 1);

    /* Ghost Generation-1 speech waveform (halftone) + current generation standing wave (solid) */
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

    /* Bottom 8-mode architectural standing-wave spectrum strip */
    draw_room_modes_strip(gfx, m);

    /* Playhead cursor (XOR) + direction chevron */
    const int px = WAVE_X + 1 + (int)(((uint32_t)m->head_i * (WAVE_W - 3u)) / m->loop_len);
    gfx_set_color(gfx, GFX_COLOR_XOR);
    gfx_draw_vline(gfx, px, WAVE_Y + 1, MODE_BAR_Y - WAVE_Y - 3);
    gfx_draw_vline(gfx, px + 1, WAVE_Y + 1, MODE_BAR_Y - WAVE_Y - 3);
    gfx_set_color(gfx, GFX_COLOR_SET);

    if (m->speed_cur < -0.02f) {
        gfx_draw_line(gfx, px + 3, WAVE_Y + 2, px - 2, WAVE_Y + 5);
        gfx_draw_line(gfx, px + 3, WAVE_Y + 8, px - 2, WAVE_Y + 5);
    } else if (m->speed_cur > 0.02f) {
        gfx_draw_line(gfx, px - 2, WAVE_Y + 2, px + 3, WAVE_Y + 5);
        gfx_draw_line(gfx, px - 2, WAVE_Y + 8, px + 3, WAVE_Y + 5);
    }
}

static void draw_param_cards(gfx_t* gfx, const sit_model_t* m) {
    for (uint8_t idx = 0; idx < ENC_TOTAL; ++idx) {
        const int x = 8 + (int)idx * (CARD_W + CARD_GAP);
        const bool sel = (m->enc_sel == idx);
        const params_t* p = &m->prm[idx];

        gfx_set_color(gfx, GFX_COLOR_SET);
        ui_draw_frame(gfx, x, CARD_Y, CARD_W, CARD_H, 3, sel);

        gfx_set_font(gfx, DepartureMono_Regular_10);
        gfx_draw_str(gfx, x + 6, CARD_Y + 13, p->name);

        char val_s[16];
        if (idx == ENC_TONE) {
            const float tv = p->val;
            const int pct = (int)(fabsf(tv) * 100.0f + 0.5f);
            if (pct == 0) {
                snprintf(val_s, sizeof(val_s), "0%%");
            } else {
                snprintf(val_s, sizeof(val_s), "%c%d%%", (tv < 0.0f) ? '-' : '+', pct);
            }
        } else {
            const int pc = (int)(p->val * 100.0f + 0.5f);
            snprintf(val_s, sizeof(val_s), "%d%%", pc);
        }
        const int vw = (int)gfx_get_str_width(gfx, val_s);
        gfx_draw_str(gfx, x + CARD_W - 6 - vw, CARD_Y + 13, val_s);

        const int bx = x + 6;
        const int by = CARD_Y + 22;
        const int bw = CARD_W - 12;
        const int bh = 11;
        gfx_draw_rect_r(gfx, bx, by, bw, bh, 2);

        if (idx == ENC_TONE) {
            /* Bipolar EQ tilt bar centered at flat 0% */
            const int mid = bx + bw / 2;
            gfx_draw_vline_dithered(gfx, mid, by + 1, bh - 2, 4);
            const float norm = clampf(p->val, -1.0f, 1.0f);
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

static void sit_redraw(gfx_t* gfx, const os_app_t* app) {
    const sit_model_t* m = (const sit_model_t*)os_app_get_model(app);
    if (!m) return;

    draw_header(gfx, m);
    draw_waveform_stage(gfx, m);
    draw_param_cards(gfx, m);
}

/* ----------------------------------------------------------------------------
 * Tick (main UI thread — safe for tape_read, bookkeeping, waveform refresh)
 * ------------------------------------------------------------------------- */
static bool sit_tick(os_app_t* app) {
    sit_model_t* m = (sit_model_t*)os_app_get_model(app);
    tape_t* t = tape_get();

    if (m->tape_swapped) {
        m->tape_swapped = false;
        if (m->bouncing) {
            m->bouncing = false;
            m->bounce_tok = 0u;
        }
        if (m->wb) tape_buffer_clear_all_buffers(m->wb);
        m->bounce_cur_win = WIN_NONE;
        tape_timeline_set_loop(false);
        sit_hints(m);
    }

    if (m->restore_req) {
        m->restore_req = false;
        sit_do_restore(m);
    }
    if (m->clear_req) {
        m->clear_req = false;
        sit_do_clear(m);
    }

    if (m->state == ST_LOADING) {
        sit_step_take_load(m);
        return true;
    }

    if (m->state == ST_REC && m->rec_done_req) {
        sit_finish_capture(m);
    }

    if (m->bouncing && t) {
        if (!m->bounce_stop_req && m->bounce_frame + BOUNCE_FADE_FR >= m->bounce_limit) {
            m->bounce_stop_req = true;
        }
        if ((m->bounce_stop_req && m->bounce_gain <= 0.001f) ||
            m->bounce_frame >= m->bounce_limit) {
            sit_bounce_stop(m, t);
        }
    }

    m->in_src = os_audio_get_input();
    sit_sync_monitor(m);
    sit_sync_engine_active(m);

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

    /* Incremental waveform, crest smoothness & modal spectrum refresh */
    if (m->loop_len >= 8u && !m->buf_locked && m->loop_buf &&
        m->state != ST_REC && m->state != ST_LOADING) {
        const uint32_t len = m->loop_len;
        for (uint32_t step = 0; step < 16u; ++step) {
            const uint32_t b = m->wave_cursor;
            const uint32_t f0 = (b * len) / WAVE_BINS;
            uint32_t f1 = ((b + 1u) * len) / WAVE_BINS;
            if (f1 <= f0) f1 = f0 + 1u;
            if (f1 > len) f1 = len;

            uint32_t stride = (f1 - f0) / 16u;
            if (stride < 1u) stride = 1u;

            float pk = 0.0f;
            float sq = 0.0f;
            uint32_t cnt = 0u;
            for (uint32_t i = f0; i < f1; i += stride) {
                const float al = fabsf(m->loop_buf[i].l);
                const float ar = fabsf(m->loop_buf[i].r);
                const float a = (al > ar) ? al : ar;
                if (a > pk) pk = a;
                sq += 0.5f * (al * al + ar * ar);
                cnt++;
            }
            m->wave_curr[b] = (uint8_t)(clampf(pk, 0.0f, 1.0f) * 255.0f);
            if (pk > m->sweep_peak) m->sweep_peak = pk;
            if (cnt > 0u) m->sweep_rms_acc += sq / (float)cnt;

            m->wave_cursor++;
            if (m->wave_cursor >= WAVE_BINS) {
                m->curr_rms = sqrtf(m->sweep_rms_acc / (float)WAVE_BINS);
                m->curr_peak = m->sweep_peak;

                /* Combine DSP modal focus + crest-factor smoothing into 0..100% resonance metric */
                const float crest = (m->curr_rms > 1.0e-4f) ? (m->curr_peak / m->curr_rms) : 6.0f;
                const float crest_smooth = clampf((5.8f - crest) * (1.0f / 3.6f), 0.0f, 1.0f);
                float focus = 0.55f * m->dsp.modal_focus + 0.45f * crest_smooth;
                if (m->generation <= 1u && m->gen1_statement) focus = 0.0f;
                m->modal_pct = (uint8_t)(clampf(focus, 0.0f, 1.0f) * 100.0f + 0.5f);

                m->wave_cursor = 0u;
                m->sweep_rms_acc = 0.0f;
                m->sweep_peak = 0.0f;
            }
        }

        /* Normalize 8-mode envelopes for standing-wave strip & LEDs */
        float max_env = 0.02f;
        for (uint32_t k = 0; k < LUCIER_NUM_MODES; ++k) {
            if (m->dsp.mode_env[k] > max_env) max_env = m->dsp.mode_env[k];
        }
        const float inv_max = 1.0f / max_env;
        for (uint32_t k = 0; k < LUCIER_NUM_MODES; ++k) {
            const float norm = clampf(m->dsp.mode_env[k] * inv_max, 0.0f, 1.0f);
            m->mode_vis[k] = (uint8_t)(norm * 255.0f + 0.5f);
        }
    }

    if (m->bouncing) {
        const float pos = (float)(m->bounce_frame / FRAMES_PER_POS);
        m->tl_pos.prev = m->tl_pos.val;
        m->tl_pos.val = m->tl_pos.target = pos;
        m->tl_end.val = m->tl_end.target = pos;
    }

    if (!m->vol_ui && (m->hint_state != m->state ||
                       m->hint_rerec != m->rerec_on ||
                       m->hint_bounce != m->bouncing)) {
        m->hint_state = m->state;
        m->hint_rerec = m->rerec_on;
        m->hint_bounce = m->bouncing;
        sit_hints(m);
    }

    return true;
}

/* ----------------------------------------------------------------------------
 * Input Handler
 * ------------------------------------------------------------------------- */
static void sit_input(os_app_t* app, uint8_t btn, KeyStateEnum st) {
    sit_model_t* m = (sit_model_t*)os_app_get_model(app);
    if (btn == BTN_ENCODER) return;

    if (m->vol_ui) {
        if (st == KEY_STATE_RELEASED && btn == BTN1) sit_vol_hide(m);
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
                sit_sync_monitor(m);
                sit_sync_engine_active(m);
            } else if (m->state == ST_STOP && m->loop_len >= 8u) {
                m->state = ST_PLAY;
                sit_sync_monitor(m);
                sit_sync_engine_active(m);
            }
            break;

        case BTN2:
            if (m->state == ST_REC) {
                m->rec_btn_down = false;
                m->rec_holdoff_fr = 0u;
                m->rec_manual_stop = true;
                m->rec_done_req = true;
                sit_sync_monitor(m);
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
                sit_sync_monitor(m);
                sit_sync_engine_active(m);
            }
            break;

        case BTN3:
            m->rerec_on = !m->rerec_on;
            if (m->rerec_on && m->gen1_statement) {
                /* Explicit toggle ON during Gen 1 engages re-recording right away */
                m->gen1_statement = false;
                if (m->generation < 2u) m->generation = 2u;
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
        m->hint_rerec = m->rerec_on;
        m->hint_bounce = m->bouncing;
        sit_hints(m);
    } else if (st == KEY_STATE_HOLD) {
        if (btn == BTN1 && !m->vol_ui) {
            m->vol_ui = true;
            sit_vol_show(m);
        } else if (btn == BTN2 && m->state != ST_REC) {
            if (m->bouncing) {
                m->bounce_stop_req = true;
            } else {
                sit_bounce_start(m);
            }
        } else if (btn == BTN3) {
            if (m->menu) ui_menu_show(m->menu);
        }
    }
}

/* ----------------------------------------------------------------------------
 * Lifecycle (init / deinit)
 * ------------------------------------------------------------------------- */
static bool sit_app_init(os_app_t* app) {
    sit_model_t* m = (sit_model_t*)os_app_get_model(app);

    /* Allocate stereo float RAM loop buffer + Q15 Gen-1 backup with graceful fallback */
    static const uint32_t kCandSecs[] = { 16u, 12u, 8u, 6u, 4u, 2u };
    for (uint32_t c = 0; c < (sizeof(kCandSecs) / sizeof(kCandSecs[0])); ++c) {
        const uint32_t frames = kCandSecs[c] * TAPP_SAMPLE_RATE;
        lucier_frame_t* lb = (lucier_frame_t*)os_malloc((size_t)frames * sizeof(lucier_frame_t));
        if (!lb) continue;
        int16_t* bb = (int16_t*)os_malloc((size_t)frames * 2u * sizeof(int16_t));
        if (bb) {
            m->loop_buf = lb;
            m->backup_buf = bb;
            m->max_frames = frames;
            break;
        }
        os_free(lb);
    }
    if (!m->loop_buf) {
        const uint32_t frames = 2u * TAPP_SAMPLE_RATE;
        m->loop_buf = (lucier_frame_t*)os_malloc((size_t)frames * sizeof(lucier_frame_t));
        if (!m->loop_buf) return false;
        m->max_frames = frames;
    }

    m->room_space = (lucier_space_t*)os_malloc(sizeof(lucier_space_t));
    m->read_stage = (float*)os_malloc((size_t)LOAD_CHUNK_FR * 2u * sizeof(float));

    tape_t* t = tape_get();
    if (t) {
        m->wb = tape_buffer_init(t);
    }
    m->bounce_cur_win = WIN_NONE;

    /* Initialize Lucier Room-Resonance DSP engine & attach reflection workspace */
    lucier_init(&m->dsp, (float)TAPP_SAMPLE_RATE);
    lucier_attach_space(&m->dsp, m->room_space);

    /* Default to re-recording ON so captured loops begin evolving after Gen 1 statement */
    m->rerec_on = true;
    m->speed_cur = 1.0f;
    m->speed_target = 1.0f;

    /* 4 live encoder cards: PASS, ROOM, RESO, TONE */
    m->prm[ENC_PASS] = (params_t)PARAM_RANGE("PASS", 0.42f,  0.00f, 1.00f, 0.02f);
    m->prm[ENC_ROOM] = (params_t)PARAM_RANGE("ROOM", 0.48f,  0.00f, 1.00f, 0.02f);
    m->prm[ENC_RESO] = (params_t)PARAM_RANGE("RESO", 0.62f,  0.00f, 1.00f, 0.02f);
    m->prm[ENC_TONE] = (params_t)PARAM_RANGE("TONE", 0.08f, -1.00f, 1.00f, 0.04f);
    m->enc_sel = ENC_PASS;
    sync_params_to_dsp(m);

    /* Volume overlay (default 25% = 0.12f / -18.4 dB for headphones) & mic trim */
    m->vol_out = (params_t)PARAM_RANGE("out", 0.12f, 0.0f, 2.0f, 0.02f);
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
        ui_menu_set_name(m->menu, "SITTING ROOM");
        ui_menu_set_header(m->menu, true);
    }

    /* Save user's monitor/WET state, silence idle speaker bleed, and tame hot mic gain */
    m->in_src = os_audio_get_input();
    m->mon_saved = os_audio_get_monitor();
    m->wet_saved = os_audio_get_wet();
    m->mon_req = false;
    sit_tame_hot_mic_input();
    (void)os_audio_set_monitor(false);
    (void)os_audio_set_wet(false);

    ui_statusbar_show(false);
    ui_hints_show(true);
    m->hint_state = m->state;
    m->hint_rerec = m->rerec_on;
    m->hint_bounce = m->bouncing;
    sit_hints(m);

    os_led_set_pattern(&sit_led_pattern, m);
    engine_set_active(false);
    return true;
}

static void sit_app_deinit(os_app_t* app) {
    sit_model_t* m = (sit_model_t*)os_app_get_model(app);
    if (!m) return;

    /* Restore user's WET and live-input monitor preferences on exit */
    (void)os_audio_set_wet(m->wet_saved);
    (void)os_audio_set_monitor(m->mon_saved);

    if (m->bouncing) {
        sit_bounce_stop(m, tape_get());
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
    if (m->room_space) {
        os_free(m->room_space);
        m->room_space = NULL;
    }
    if (m->backup_buf) {
        os_free(m->backup_buf);
        m->backup_buf = NULL;
    }
    if (m->loop_buf) {
        os_free(m->loop_buf);
        m->loop_buf = NULL;
    }
}

TAPP_APP("Sitting Room", sit_model_t, sit_app_init, sit_app_deinit,
         .redraw = sit_redraw,
         .tick = sit_tick,
         .on_input = sit_input,
         .engine_cb = &sit_engine_cb);
