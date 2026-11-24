#include "disintegrate_dsp.h"

#ifdef DIS_HOST_TEST
#include <math.h>
static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
#else
#include "tapp_api.h"
#endif

/* Constants matching phcuadra/Spotykach src/core/fx/fx.disintegrate.cpp */
#define DIS_MIN_ACTIVE_INTENSITY 0.005f
#define DIS_SILENCE_FLOOR        1.0e-5f
#define DIS_CRACKLE_GATE_FLOOR   0.0025f

/* Patch width spans ~16 ms (750 samples) at character=0 to ~1.33 s (64000 samples) at character=1 */
#define DIS_MAX_SPATIAL_FREQ     (1.0f / 750.0f)
#define DIS_MIN_SPATIAL_FREQ     (1.0f / 64000.0f)
/* ln(DIS_MIN_SPATIAL_FREQ / DIS_MAX_SPATIAL_FREQ) = ln(750 / 64000) */
#define DIS_LOG_SPATIAL_RATIO    (-4.44656603f)

#define DIS_MAX_STEP_FRAMES      8

static inline float dis_fast_clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline float dis_smoothstep01(float t) {
    const float c = dis_fast_clamp(t, 0.0f, 1.0f);
    return c * c * (3.0f - 2.0f * c);
}

static inline float dis_hash01(int32_t n, uint32_t seed) {
    uint32_t h = ((uint32_t)n * 0x9E3779B1u) ^ seed;
    h ^= h >> 16;
    h *= 0x85EBCA6Bu;
    h ^= h >> 13;
    h *= 0xC2B2AE35u;
    h ^= h >> 16;
    return (float)(h >> 8) * (1.0f / 16777216.0f);
}

/* C1-continuous 1D value noise in [0, 1] */
static inline float dis_smooth_noise_1d(float x, uint32_t seed) {
    const float fl = floorf(x);
    const int32_t i = (int32_t)fl;
    const float f = x - fl;
    const float u = f * f * (3.0f - 2.0f * f);
    const float a = dis_hash01(i, seed);
    const float b = dis_hash01(i + 1, seed);
    return a + u * (b - a);
}

/* Evaluates localized oxide flaking strength [0, 1] at a spatial coordinate */
static inline float dis_eval_oxide_flake(float pos, float detail_phase, uint32_t seed) {
    const float coarse = dis_smooth_noise_1d(pos, seed);
    const float fine   = dis_smooth_noise_1d(pos * 2.73f + detail_phase, seed ^ 0xA53C9E47u);
    const float field  = 0.72f * coarse + 0.28f * fine;

    /* ~52% of the tape starts as an intact island (flake == 0), while defect peaks reach 0.8..1.0 */
    const float raw = dis_fast_clamp((field - 0.51f) * 3.2f, 0.0f, 1.0f);
    return dis_smoothstep01(raw);
}

float dis_soft_clip_guard(float x) {
    if (x > 1.2f) return 1.2f;
    if (x < -1.2f) return -1.2f;
    return x;
}

static inline uint32_t dis_next_u32(dis_engine_t* d) {
    uint32_t x = d->rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    d->rng_state = x ? x : 0x6D2B79F5u;
    return d->rng_state;
}

static inline float dis_next_bipolar(dis_engine_t* d) {
    return (float)((int32_t)(dis_next_u32(d) >> 8)) * (1.0f / 8388608.0f) - 1.0f;
}

static void dis_apply(dis_engine_t* d) {
    if (d->intensity < DIS_MIN_ACTIVE_INTENSITY) {
        d->rate = 0.0f;
    } else {
        /* Musical curve: gentle multi-minute decay at low settings, dramatic shedding near 1.0 */
        d->rate = 0.14f * d->intensity + 0.86f * (d->intensity * d->intensity);
    }

    /* Exponential interpolation between DIS_MAX_SPATIAL_FREQ and DIS_MIN_SPATIAL_FREQ */
    d->spatial_freq = DIS_MAX_SPATIAL_FREQ * expf(d->character * DIS_LOG_SPATIAL_RATIO);

    /* Slow boundary creep (~0.008 to ~0.045 patch-widths/sec) keeps weak spots physically
     * anchored on the loop over many passes while gradually nibbling into surviving islands */
    d->drift_inc = (0.008f + 0.037f * d->rate) / d->sample_rate;

    /* Fine pitting allows subtle L/R independence; large sheets peel both tracks together */
    d->stereo_link = 0.35f + 0.65f * d->character;
}

void dis_init(dis_engine_t* d, float sample_rate) {
    d->sample_rate  = (sample_rate > 1000.0f) ? sample_rate : 48000.0f;
    d->intensity    = 0.25f;
    d->character    = 0.45f;
    d->mix          = 0.30f;
    d->gate         = 0.0f;
    d->rng_state    = 0x6D2B79F5u;
    d->tape_seed_l  = 0x1F83D9ABu;
    d->tape_seed_r  = 0x5BE0CD19u;
    d->drift_phase  = 0.0f;
    d->detail_phase = 17.3f;
    d->dust_lp_l    = 0.0f;
    d->dust_lp_r    = 0.0f;
    d->activity     = 0.0f;
    dis_apply(d);
}

void dis_reset_tape(dis_engine_t* d) {
    d->tape_seed_l  = dis_next_u32(d) | 1u;
    d->tape_seed_r  = dis_next_u32(d) | 1u;
    d->drift_phase  = 0.0f;
    d->detail_phase = 17.3f;
    d->dust_lp_l    = 0.0f;
    d->dust_lp_r    = 0.0f;
    d->activity     = 0.0f;
}

void dis_set_intensity(dis_engine_t* d, float norm) {
    d->intensity = dis_fast_clamp(norm, 0.0f, 1.0f);
    dis_apply(d);
}

void dis_set_character(dis_engine_t* d, float norm) {
    d->character = dis_fast_clamp(norm, 0.0f, 1.0f);
    dis_apply(d);
}

void dis_set_mix(dis_engine_t* d, float norm) {
    d->mix = dis_fast_clamp(norm, 0.0f, 1.0f);
    dis_apply(d);
}

void dis_set_gate(dis_engine_t* d, float gate) {
    d->gate = dis_fast_clamp(gate, 0.0f, 1.0f);
}

bool dis_is_active(const dis_engine_t* d) {
    return d->gate > 0.0001f && d->intensity >= DIS_MIN_ACTIVE_INTENSITY;
}

void dis_erode_frame(dis_engine_t* d, dis_frame_t* buf, uint32_t rec_size,
                     int32_t idx, float weight) {
    if (!buf || rec_size < 8u || d->rate <= 0.0f) return;

    const float eff = dis_fast_clamp(weight * d->gate, 0.0f, 1.0f);
    if (eff <= 0.0001f) return;

    const int32_t size = (int32_t)rec_size;
    int32_t i0 = idx % size;
    if (i0 < 0) i0 += size;

    /* 1. Spatially persistent, periodic C1-smooth oxide flaking field */
    const float pos = (float)i0 * d->spatial_freq + d->drift_phase;
    float flake_l = dis_eval_oxide_flake(pos, d->detail_phase, d->tape_seed_l);
    float flake_r = dis_eval_oxide_flake(pos, d->detail_phase + 5.17f, d->tape_seed_r);

    /* Blend near loop wraparound so the oxide wear profile is C1-periodic across the splice seam */
    const int32_t quarter = size >> 2;
    const int32_t seam_len = (quarter < 2048) ? quarter : 2048;
    const int32_t seam_start = size - seam_len;
    if (seam_len > 16 && i0 >= seam_start) {
        const float seam_t = dis_smoothstep01((float)(i0 - seam_start) / (float)seam_len);
        const float wrapped_pos = (float)(i0 - size) * d->spatial_freq + d->drift_phase;
        const float wrap_l = dis_eval_oxide_flake(wrapped_pos, d->detail_phase, d->tape_seed_l);
        const float wrap_r = dis_eval_oxide_flake(wrapped_pos, d->detail_phase + 5.17f, d->tape_seed_r);
        flake_l += seam_t * (wrap_l - flake_l);
        flake_r += seam_t * (wrap_r - flake_r);
    }

    flake_r = flake_r + d->stereo_link * (flake_l - flake_r);

    /* Advance slow defect drift (wrapped to preserve float precision indefinitely) */
    d->drift_phase += d->drift_inc * eff;
    if (d->drift_phase >= 4096.0f) d->drift_phase -= 4096.0f;
    d->detail_phase += d->drift_inc * 1.618f * eff;
    if (d->detail_phase >= 4096.0f) d->detail_phase -= 4096.0f;

    d->activity += 0.002f * (0.5f * (flake_l + flake_r) - d->activity);

    float xl = buf[i0].l;
    float xr = buf[i0].r;

    /* 2. Progressive High-Frequency Dulling (zero-phase convex stride-1 + stride-3 waveform diffusion) */
    const int32_t im1 = (i0 == 0) ? (size - 1) : (i0 - 1);
    const int32_t ip1 = (i0 + 1 == size) ? 0 : (i0 + 1);
    int32_t im3 = i0 - 3;
    if (im3 < 0) im3 += size;
    int32_t ip3 = i0 + 3;
    if (ip3 >= size) ip3 -= size;

    const float wear_base = eff * (0.06f + 0.94f * d->rate) * (0.08f + 0.92f * d->mix);
    const float kappa_l = dis_fast_clamp(wear_base * (0.75f + 1.45f * flake_l), 0.0f, 0.48f);
    const float kappa_r = dis_fast_clamp(wear_base * (0.75f + 1.45f * flake_r), 0.0f, 0.48f);

    const float lap1_l = 0.5f * (buf[im1].l + buf[ip1].l) - xl;
    const float lap3_l = 0.5f * (buf[im3].l + buf[ip3].l) - xl;
    xl += kappa_l * (0.65f * lap1_l + 0.35f * lap3_l * d->mix);

    const float lap1_r = 0.5f * (buf[im1].r + buf[ip1].r) - xr;
    const float lap3_r = 0.5f * (buf[im3].r + buf[ip3].r) - xr;
    xr += kappa_r * (0.65f * lap1_r + 0.35f * lap3_r * d->mix);

    /* 3. Oxide Dropouts / Flaking (damaged/thinned regions shed oxide slightly faster) */
    const float env_l = fabsf(xl);
    const float env_r = fabsf(xr);
    const float thin_l = 1.0f + 0.65f * (1.0f - dis_fast_clamp(env_l * 6.67f, 0.0f, 1.0f));
    const float thin_r = 1.0f + 0.65f * (1.0f - dis_fast_clamp(env_r * 6.67f, 0.0f, 1.0f));

    const float loss_l = dis_fast_clamp(eff * d->rate * (0.016f + 0.58f * flake_l * thin_l), 0.0f, 1.0f);
    const float loss_r = dis_fast_clamp(eff * d->rate * (0.016f + 0.58f * flake_r * thin_r), 0.0f, 1.0f);
    xl *= (1.0f - loss_l);
    xr *= (1.0f - loss_r);

    /* 4. Subtle Odd-Symmetric Tape Compression / Saturation Warmth (zero DC offset, strictly contractive) */
    const float sat = eff * d->rate * d->mix * 0.24f;
    if (sat > 0.0f) {
        xl = xl / (1.0f + sat * xl * xl);
        xr = xr / (1.0f + sat * xr * xr);
    }

    /* 5. Oxide Asperity Micro-Crackle (band-limited warm dust modulated by signal envelope) */
    if (d->mix > 0.01f && (env_l > DIS_CRACKLE_GATE_FLOOR || env_r > DIS_CRACKLE_GATE_FLOOR)) {
        float imp_l = 0.0f;
        float imp_r = 0.0f;
        if ((dis_next_u32(d) & 0xFFu) == 0u) {
            imp_l = dis_next_bipolar(d);
            imp_r = d->stereo_link * imp_l + (1.0f - d->stereo_link) * dis_next_bipolar(d);
        }
        d->dust_lp_l += 0.32f * (imp_l - d->dust_lp_l);
        d->dust_lp_r += 0.32f * (imp_r - d->dust_lp_r);

        const float crackle_scale = eff * d->mix * (0.30f + 0.70f * d->intensity);
        if (env_l > DIS_CRACKLE_GATE_FLOOR) {
            xl += d->dust_lp_l * crackle_scale * (0.012f + 0.045f * flake_l) * env_l;
        }
        if (env_r > DIS_CRACKLE_GATE_FLOOR) {
            xr += d->dust_lp_r * crackle_scale * (0.012f + 0.045f * flake_r) * env_r;
        }
    }

    /* Flush sub-audible tails cleanly to pure silence (prevents Cortex-M7 denormals & residual hiss) */
    buf[i0].l = (fabsf(xl) < DIS_SILENCE_FLOOR) ? 0.0f : dis_soft_clip_guard(xl);
    buf[i0].r = (fabsf(xr) < DIS_SILENCE_FLOOR) ? 0.0f : dis_soft_clip_guard(xr);
}

void dis_step_erode(dis_engine_t* d, dis_frame_t* buf, uint32_t rec_size,
                    int32_t prev_i, int32_t step, float weight) {
    if (!dis_is_active(d) || !buf || rec_size < 8u || weight <= 0.0001f) return;

    if (step > 0 && step <= DIS_MAX_STEP_FRAMES) {
        for (int32_t k = 0; k < step; ++k) {
            dis_erode_frame(d, buf, rec_size, prev_i + k, weight);
        }
    } else if (step < 0 && -step <= DIS_MAX_STEP_FRAMES) {
        const int32_t n = -step;
        for (int32_t k = 0; k < n; ++k) {
            dis_erode_frame(d, buf, rec_size, prev_i + 1 - k, weight);
        }
    }
}

float dis_sample_flake_vis(const dis_engine_t* d, uint32_t rec_size, uint32_t idx) {
    if (rec_size < 8u) return 0.0f;
    const int32_t size = (int32_t)rec_size;
    const int32_t i0 = (int32_t)(idx % rec_size);
    const float pos = (float)i0 * d->spatial_freq + d->drift_phase;
    float flake_l = dis_eval_oxide_flake(pos, d->detail_phase, d->tape_seed_l);
    float flake_r = dis_eval_oxide_flake(pos, d->detail_phase + 5.17f, d->tape_seed_r);

    const int32_t quarter = size >> 2;
    const int32_t seam_len = (quarter < 2048) ? quarter : 2048;
    const int32_t seam_start = size - seam_len;
    if (seam_len > 16 && i0 >= seam_start) {
        const float seam_t = dis_smoothstep01((float)(i0 - seam_start) / (float)seam_len);
        const float wrapped_pos = (float)(i0 - size) * d->spatial_freq + d->drift_phase;
        const float wrap_l = dis_eval_oxide_flake(wrapped_pos, d->detail_phase, d->tape_seed_l);
        const float wrap_r = dis_eval_oxide_flake(wrapped_pos, d->detail_phase + 5.17f, d->tape_seed_r);
        flake_l += seam_t * (wrap_l - flake_l);
        flake_r += seam_t * (wrap_r - flake_r);
    }

    flake_r = flake_r + d->stereo_link * (flake_l - flake_r);
    return 0.5f * (flake_l + flake_r);
}
