# Sitting Room — Alvin Lucier *I Am Sitting in a Room* (1969) Instrument for *tape!*

A standalone *tape!* application (`AppTypeStandalone`) modeled after Alvin Lucier's 1969 composition *I Am Sitting in a Room*. Speak a phrase, capture a musical fragment, or load a recorded take from tape: on each pass (`GEN 1` → `GEN N`), the loop is iteratively re-recorded in-place through an architectural room modal transfer function, loudspeaker/microphone spectral tilt EQ, early wall reflections, and energy-governed tape saturation until sharp consonants and transients dissolve into pure, singing standing-wave chords of the room.

## Architecture & DSP (`lucier_dsp.c` / `inc/lucier_dsp.h`)

- **RAM Loop Buffer + Clean Generation-1 Q15 Backup + Room Space**: Allocates up to 16 seconds of stereo 48 kHz `float` loop memory, a matching `int16_t` Q15 Generation-1 backup snapshot (with graceful fallback down to 2 s), and a ~28 KB stereo reflection/diffusion workspace via `os_malloc()` in PSRAM while keeping the `sit_model_t` app struct small (~2.7 KB in internal SRAM).
- **Generation-1 Clean Statement**: Newly captured, loaded, or restored loops play through `GEN 1` cleanly once while priming the room's RMS level trackers, then seamlessly engage iterative room re-recording on `GEN 2, 3, 4...` (or immediately if toggled manually).
- **Per-Generation Acoustic Re-Recording Chain**:
  1. **Early Wall Reflections & Stereo Schroeder Allpass Diffusion**: 3-tap stereo room reflection lines scaled by `ROOM` with 18% L↔R cross-wall coupling feeding 2 cascaded Schroeder allpass diffusers per channel (`g_ap = 0.38 + 0.24 * RESO`). Bridges inter-word gaps and progressively smears consonant plosives across passes while preserving zero-delay direct arrival phase.
  2. **Per-Pass Speaker / Microphone Tilt EQ (`TONE`)**: 1-pole sub-rumble high-pass (`52 ± 26 Hz`), dynamic loudspeaker/air high-frequency roll-off (`~2.7–7.9 kHz`), and a phase-coherent 1st-order Linkwitz spectral tilt shelf around `380 Hz` (`±4.8 dB` shelf + per-mode octave tilt weighting).
  3. **8-Mode Zero-Delay-Feedback (ZDF) Room Resonator Bank**: 8 parallel unity-peak trapezoidal SVF bandpass resonators (`Q = 7 .. 72` controlled by `RESO`) tuned to 4 selectable architectural room geometries (`1969 Room`, `Tiled Chamber`, `Stone Chapel`, `Cistern`) scaled exponentially from small chambers (`240 Hz` fundamental) to vast halls (`58 Hz` fundamental), with subtle stereo mic detuning (`±2–5.5 cents`) and gentle per-mode envelope compression (`1 / (1 + 2.4 * env_m)`) so 3–5 excited room modes bloom together into rich harmonic chords.
  4. **Matched-Envelope AGC + Zero-DC Blocker + C1 Tape Limiter**: Dual 450 ms RMS trackers match filter insertion loss / modal gain (`sqrt(E_dry / E_wet)`) anchored by a gentle loudness governor toward `target_rms`, followed by an `18 Hz` 1-pole DC blocker and odd-symmetric Padé rational `tanh` soft limiter (`±0.95` ceiling, zero DC offset, denormal-flushed).
- **Bounce-to-Tape**: Streams the evolving generations directly onto free SD tape space via `tape_buffer_t` DMA windows, committing a mark and loop cue on stop.

## Controls

| Control | Action |
|---|---|
| **BTN1 Press** | Play / Stop loop |
| **BTN1 Hold** | Volume overlay (`out` @ `15%` default, `fx send`, `in`, `mic trim`) |
| **BTN2 Press** | Capture live input into RAM loop (live input is monitored **only** while recording) / Close loop & start `GEN 1` |
| **BTN2 Hold** | Bounce evolving room loop to free SD tape space (Start / Stop & commit) |
| **BTN3 Press** | Toggle Re-Record Pass (`rerec`) ON / OFF (30 ms smooth gate; OFF freezes current generation as an infinite drone) |
| **BTN3 Hold** | Open menu (`Load Tape Take`, `Restore Gen 1`, `Space` Geometry cycle, `Direction`, `1.00x / 0.50x / 2.00x Speed`, `Input Line/Mic`, `Mic Trim`, `Clear Loop`) |
| **BTN4 / BTN5 Press** | Previous / Next encoder parameter (`PASS`, `ROOM`, `RESO`, `TONE`) |
| **BTN4 / BTN5 Hold** | System main menu (standalone TAPP) |
| **Encoder** | Adjust selected parameter |

### Encoder Parameters

| Parameter | Range | Default | Description |
|---|---|---|---|
| `PASS` | `0–100%` | `42%` | Per-generation room + EQ re-recording depth (~25–40 passes for subtle speech dissolution at 35–45%; rapid 6–10 pass modal bloom near 100%) |
| `ROOM` | `0–100%` | `48%` | Physical room dimensions (`0%` small 240 Hz chamber → `100%` vast 58 Hz architectural hall & longer reflections) |
| `RESO` | `0–100%` | `62%` | Modal resonator `Q` (`7 → 72`) & wall-reflection diffusion tail |
| `TONE` | `-100% .. +100%` | `+8%` | Per-pass speaker/mic spectral tilt (`-100%` warm fundamental hum ↔ `+100%` crystalline upper-harmonic bells) |

## Building & Deploying

### Build & Verify Locally
```bash
make verify TAPE_SDK=/path/to/tape_sdk
```
Or directly via the SDK:
```bash
/path/to/tape_sdk/tapp-build /path/to/sitting_in_a_room sitting_in_a_room.tapp
/path/to/tape_sdk/tools/verify-tapp.sh sitting_in_a_room.tapp
```

### Deploy to *tape!*
1. Enter USB disk mode on *tape!* and copy `sitting_in_a_room.tapp` into the `apps/` folder on the SD card.
2. On *tape!*, hold **BTN1** on the **`tape`** entry of the main menu (**apps** switcher) and select **Sitting Room**.
