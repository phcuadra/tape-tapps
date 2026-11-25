# Gonna Rain — Steve Reich *It's Gonna Rain* (1965) Tape-Phasing Instrument for *tape!*

A standalone *tape!* application (`AppTypeStandalone`) modeled after Steve Reich's seminal 1965 tape composition *It's Gonna Rain* (Parts I & II). Record a rhythmic vocal phrase or percussive fragment, or load a recorded take from tape: two (Part I) or four (Part II) virtual Wollensak reel-to-reel tape heads start in exact `0.0°` unison and gradually drift out of phase — sweeping continuously from resonant comb-filtering (`0.5–15 ms`), through slapback echo doubling (`15–75 ms`), into interlocking polyrhythmic canons and antiphonal stereo hockets (`90°`, `120°`, `180°`, `270°`), before converging seamlessly back into unison (`360.0°`).

## Architecture & DSP (`reich_dsp.c` / `inc/reich_dsp.h`)

- **Non-Destructive Shared PSRAM Loop Memory**: Because Reich's tape-phasing reads multiple virtual playback heads at slightly offset capstan speeds over an intact magnetic loop, only a single stereo `float` loop buffer is required in PSRAM via `os_malloc()` — supporting up to **24 seconds** of 48 kHz stereo audio (`24s → 16s → 12s → 8s → 6s → 4s → 2s` graceful fallback) while keeping `rain_model_t` compact (~2.4 KB in fast internal SRAM).
- **Sub-Sample 4-Point Hermite Cubic Tape Heads**: All 2 or 4 virtual tape heads use wrapped 4-point, 3rd-order Catmull-Rom / Hermite spline interpolation (`> 108 dB` half-wavelength comb-nulling accuracy even at `0.5`-sample fractional offsets) so early-stage phasing (`0.1°–3°`) sweeps through silky-smooth acoustic comb-filtering without zipper noise.
- **Drift-Free `(int32_t + float)` Sample-Domain Phase Accumulator**: Relative phase lead between Deck B and Deck A is tracked in sample-accurate `(phase_i, phase_f)` coordinates rather than normalized `[0, 1)` float turns, preventing IEEE-754 `float32` ULP underflow at glacial multi-minute drift rates on 24-second loops and holding exact `0.0000°` phase lock indefinitely when `LOCKED`.
- **Part I (2-Deck) & Part II (4-Deck Harmonic Canon) Modes**:
  - **2 Decks (Part I)**: Deck A (`0x` reference) + Deck B (`1x` phase lead), panned across stereo by `SPREAD`.
  - **4 Decks (Part II)**: Heads A, B, C, D at harmonic multiples `(0, 1φ, 2φ, 3φ)` panned outer/inner L/R — forming 4-voice unison at `0°`, an equal 4-voice quarter-loop canon at `90°`, a 3-against-1 canon at `120°`, reinforced double-unison antiphonal hockets at `180°`, and simultaneous 4-head convergence back to `0°` at `360°`.
- **Continuous Drift vs. Canon Dwell (*Piano Phase* Mode) + Reel-Flange Scrub**:
  - **Continuous**: Uninterrupted exponential capstan slip from multi-minute glacial flanging (`5–20%`) to expressive sweeps (`75–100%`).
  - **Canon Dwell**: Automatically locks into each `45°` rational division (`0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°`) for 2 full loop revolutions before gliding to the next groove.
  - **Thumb-on-Reel-Flange Servo**: Turning `PHASE` (or invoking `Re-Align Unison`, `Snap Nearest Canon`, or `Jump +90 deg`) feeds a smooth capstan servo slew accumulator so Deck B accelerates or brakes organically into the new phase angle with zero clicks.
- **Wollensak 1965 Tape Character (`TAPE`)**: Zero-mean independent capstan micro-wow/flutter (`±0.29 ms` max, preserving exact `phase_i/phase_f` calibration), `~125 Hz` playback head bump warmth, gentle oxide high-frequency smoothing, and an odd-symmetric C1 Padé rational `tanh` tape limiter (`±0.98` ceiling).
- **Seamless Non-Destructive Sub-Loop Crop**: Crop loaded takes (`Full`, `1/2`, `1/4`, or authentic `1.8s Tight`) with a C1-continuous wraparound crossfade splice that leaves the underlying loop untouched.
- **Bounce-to-Tape**: Streams live phasing performances directly onto free SD tape space via `tape_buffer_t` DMA windows, committing a mark and loop cue on stop.

## Controls

| Control | Action |
|---|---|
| **BTN1 Press** | Play / Stop reels |
| **BTN1 Hold** | Volume overlay (`out` @ `25%` default, `fx send`, `in`, `mic trim`) |
| **BTN2 Press** | Capture live input into RAM loop (live input is monitored **only** while recording) / Close loop & start Lap 1 in `0.0°` Unison |
| **BTN2 Hold** | Bounce live phasing output to free SD tape space (Start / Stop & commit) |
| **BTN3 Press** | Toggle Phase Drift (`phase` / `PHASING`) ↔ Capstan Thumb-Lock (`lock` / `LOCKED` at `0.000%` slip) |
| **BTN3 Hold** | Open `"GONNA RAIN"` menu (`Load Tape Take`, `Re-Align Unison (0 deg)`, `Snap Nearest Canon`, `Jump Phase: +90 deg`, `Decks: 2/4`, `Motion: Continuous/Canon Dwell`, `Crop`, `Direction`, `Speed 1.00x / 0.50x / 2.00x`, `Input Line/Mic`, `Mic Trim`, `Clear Loop`) |
| **BTN4 / BTN5 Press** | Previous / Next encoder parameter (`DRIFT`, `PHASE`, `SPREAD`, `TAPE`) |
| **BTN4 / BTN5 Hold** | System main menu (standalone TAPP) |
| **Encoder** | Adjust selected parameter (on `PHASE`: smooth velocity-sensitive reel-flange phase nudge/scrub) |

### Encoder Parameters

| Parameter | Range | Default | Description |
|---|---|---|---|
| `DRIFT` | `0–100%` | `35%` | Relative capstan speed differential (`5–20%` multi-minute Reich flanging, `35%` ~33s concert cycle on a 2s loop, `100%` fast reel sweep) |
| `PHASE` | `0.0°–360.0°` | `0.0°` | Live Deck B phase lead readout & manual "thumb-on-flange" phase scrub |
| `SPREAD` | `0–100%` | `70%` | Stereo separation (`0%` center mono sum for deep acoustic comb-filtering → `100%` hard L/R stereo hocket) |
| `TAPE` | `0–100%` | `28%` | Analog Wollensak capstan micro-wow/flutter, 125 Hz head bump, oxide HF roll-off & saturation warmth |

## Building & Deploying

### Build & Verify Locally
```bash
make verify TAPE_SDK=/path/to/tape_sdk
```
Or directly via the SDK:
```bash
/path/to/tape_sdk/tapp-build /path/to/gonna_rain gonna_rain.tapp
/path/to/tape_sdk/tools/verify-tapp.sh gonna_rain.tapp
```

### Deploy to *tape!*
1. Enter USB disk mode on *tape!* and copy `gonna_rain.tapp` into the `apps/` folder on the SD card.
2. On *tape!*, hold **BTN1** on the **`tape`** entry of the main menu (**apps** switcher) and select **Gonna Rain**.
