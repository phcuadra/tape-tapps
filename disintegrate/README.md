# Disintegrate — Basinski-Style Disintegration Looper for *tape!*

A standalone *tape!* application (`AppTypeStandalone`) inspired by William Basinski's *The Disintegration Loops*. Ports the destructive magnetic-tape oxide erosion algorithm from `phcuadra/Spotykach` (`feat/disintegrate-mode`) to *tape!*.

## Architecture & DSP

- **RAM Loop Buffer + Clean Q15 Backup Snapshot**: Allocates up to 16 seconds of stereo 48 kHz float audio (plus a matching Q15 stereo backup snapshot, with automatic fallback down to 2 s if heap is tighter) via `os_malloc()` in PSRAM while keeping the app model small (~3 KB in internal RAM).
- **Destructive Per-Sample Oxide Erosion** (`disintegrate_dsp.c` / `inc/disintegrate_dsp.h`):
  1. **Spatially persistent C1-periodic oxide flaking field**: Dual-octave smoothstep 1D value noise (`coarse` + `2.73x` fine detail) with C1 seam blending across loop wraparound, slow boundary creep (`drift_phase` / `detail_phase`), and character-controlled stereo track linking.
  2. **Progressive high-frequency dulling**: Zero-phase convex stride-1 + stride-3 waveform diffusion weighted by local flake intensity and `WEAR`.
  3. **Oxide dropouts / flaking**: Thinned tape regions shed oxide slightly faster (`thin = 1 + 0.65 * (1 - clamp(env * 6.67))`), dissolving into silence over passes.
  4. **Odd-symmetric tape warmth**: Strictly contractive soft compression `x / (1 + sat * x^2)` with zero DC offset.
  5. **Oxide asperity micro-crackle**: Envelope-gated, 1-pole low-pass filtered sparse impulses modulated by flaking strength and `WEAR`.
  6. **Silence floor & safety guard**: Flushes sub-audible tails (`< 1e-5`) to `0.0` (preventing Cortex-M7 denormals) and clamps at `±1.2`.
- **Drift-Free Varispeed & Reverse**: Integer + fractional phase accumulator (`-2.00x .. +2.00x`) with smooth tape-motor inertia. Erosion steps only over frames the playhead just left behind (forward or reverse) so linear interpolation never reads mid-stepped boundaries.
- **Bounce-to-Tape**: Streams the live disintegrating output onto free space on the mounted SD tape using lock-free `tape_buffer_t` DMA windows, committing a mark and cue on stop.

## Controls

| Control | Action |
|---|---|
| **BTN1 Press** | Play / Stop loop |
| **BTN1 Hold** | Volume overlay (`out`, `fx send`, `in`, `mic trim`) |
| **BTN2 Press** | Capture live input into RAM loop (live input is monitored **only** while recording) / Close loop & start playing |
| **BTN2 Hold** | Bounce disintegrating output to free tape space (Start / Stop & commit) |
| **BTN3 Press** | Toggle Disintegrate oxide erosion ON / OFF (30 ms smooth gate) |
| **BTN3 Hold** | Open menu (Load Tape Take, Restore Fresh Loop, Flip Direction, 1.00x / 0.50x Speed, New Oxide Seed, Input Line/Mic, Mic Trim, Clear Loop) |
| **BTN4 / BTN5 Press** | Previous / Next encoder parameter (`RATE`, `FLAKE`, `WEAR`, `SPEED`) |
| **BTN4 / BTN5 Hold** | System main menu (standalone TAPP) |
| **Encoder** | Adjust selected parameter |

### Parameters (Spotykach Mapping)

| Parameter | Spotykach Equivalent | Range | Default | Description |
|---|---|---|---|---|
| `RATE` | `intensity` | `0–100%` | `25%` | Erosion speed (gentle multi-minute decay at low values, rapid flaking near 100%) |
| `FLAKE` | `character` | `0–100%` | `45%` | Oxide defect patch size (~16 ms fine pitting → ~1.33 s peeling sheets & stereo link) |
| `WEAR` | `mix` | `0–100%` | `30%` | Progressive HF dulling, tape saturation warmth & micro-crackle |
| `SPEED` | — | `-2.00x .. +2.00x` | `+1.00x` | Varispeed & reverse playback with tape-motor inertia |

## Building & Deploying

### Build Locally
```bash
make TAPE_SDK=/path/to/tape_sdk
```
Or directly via the SDK:
```bash
/path/to/tape_sdk/tapp-build /path/to/disintegrate disintegrate.tapp
```

### Build & Preview in Browser
Drag and drop the entire `disintegrate/` folder onto **[tapp.b.edti.me](https://tapp.b.edti.me)** to compile and run it immediately in the WebAssembly ARM emulator.

### Deploy to *tape!*
1. Enter USB disk mode on *tape!* and copy `disintegrate.tapp` into the `apps/` folder on the SD card.
2. On *tape!*, hold **BTN1** on the **`tape`** entry of the main menu (**apps** switcher) and select **Disintegrate**.
