# Swinging Piano — Aphex Twin Barbican Hall *Remote Orchestra* (2012) & *aisatsana [102]* Instrument for *tape!*

A standalone *tape!* application (`AppTypeStandalone`) inspired by Richard D. James's October 2012 Barbican Hall *Remote Orchestra* suspended swinging grand piano installation and the intimate room-captured felt piano aesthetic of *aisatsana [102]*. Capture live audio from Line/Mic, load any recorded take from SD tape, or conjure a seamless 48 kHz felt-piano loop on the spot via **`Seed Felt Piano`** — then suspend the soundboard from a towering fly-tower cable over a stationary stereo hall microphone pair.

## Architecture & Acoustic Physics (`pendulum_dsp.c` / `inc/pendulum_dsp.h`)

- **Nonlinear 2D Gravitational Pendulum (`Velocity-Verlet` ODE)**:
  - Integrates $\ddot{\theta}_x = -\frac{g}{L}\sin\theta_x - \gamma\dot{\theta}_x + u_{\text{drive}}$ alongside a quadrature-coupled front-to-back depth pendulum $\theta_y$ across 4 selectable hall rigs (`Barbican 10m`, `Elliptical 2D`, `Chapel 5m`, `Close 2.4m`).
  - **Sustained Swing vs. Natural Free-Pendulum Coast (`BTN3`)**: When `SWINGING` is active, a phase-coherent energy-regulating escapement maintains exact peak excursion `ARC` (`0°` to `±55°`) without distorting the conservative nonlinear velocity profile. Disengaging drive (`COASTING`) lets the heavy suspended piano coast down organically under real air/pivot damping, while **`Push Piano (Impulse)`** injects fresh angular momentum for decaying pendulum sweeps.
- **Stationary Stereo Hall Mics & 4-Point Hermite Fractional Delay Propagation**:
  - Computes exact 3D Euclidean distances $r_L(t), r_R(t)$ from `(x_p, y_p, z_p)` to a spaced ORTF/A-B stereo microphone pair beneath the arc, smoothed via a cascaded 2-pole filter for 100% zipper-free `C1` delay modulation.
  - Reads dual `8192`-sample circular propagation lines via **4-point, 3rd-order Hermite cubic interpolation**, naturally producing authentic physical **Doppler pitch bending** (swooping sharp on approach toward center mics, gliding wistfully flat on recession toward the turning points), true **Interaural Time Difference (ITD)**, **inverse-distance proximity swell** + outward cardioid directivity, and dynamic off-axis **soundboard/air-absorption low-pass filtering**.
- **Opposite-Wall Early Reflections + Barbican Concert Hall FDN (`HALL`)**:
  - Models two lateral side-wall image-source reflections (`r_wall_L`, `r_wall_R`) whose delays counter-modulate in pitch as the piano approaches one wall while receding from the other, feeding dual Schroeder input diffusers and a modulated 4×4 **Householder Feedback Delay Network (FDN)** with warm felt-hall HF damping, an 18 Hz DC blocker, and an odd-symmetric `C1` Padé rational `tanh` soft limiter (`±0.98` ceiling).
- **Built-In `"Seed Felt Piano"` Generator**:
  - Synthesizes an intimate `9.60 s` stereo 48 kHz felt-piano progression (*Cmaj9 → Am11 → Fmaj9(#11) → G6sus4*) with stiff-string inharmonicity, detuned unison bichords, soft wool-hammer transients, and modulo-wrapped sustain resonance so the loop repeats with zero seam.
- **Compact SRAM Footprint + Record-Only Input/WET Gating**:
  - Keeps `swing_model_t` compact (~2.1 KB in internal SRAM) while allocating the `96.0 KB` `swing_space_t` acoustic workspace and up to `24 s` stereo loop RAM via `os_malloc()`.
  - Enforces strict record-only input monitoring and `WET` FX gating (`swing_sync_monitor`), plus `mic_trim` (`ParamValDBType`, `0.40x` default) and automatic hot-mic taming (`swing_tame_hot_mic_input`).

## Controls

| Control | Action |
|---|---|
| **BTN1 Press** | Play / Stop loop (`play` / `stop`) |
| **BTN1 Hold** | Volume overlay (`out` @ `25%` default, `fx send`, `in`, `mic trim`) |
| **BTN2 Press** | Capture live input into PSRAM loop (monitored **only** during `ST_REC`) / Close loop & start swinging |
| **BTN2 Hold** | Bounce live swinging output to free SD tape space via DMA windows (Start / Stop & commit) |
| **BTN3 Press** | Toggle Sustained Pendulum Drive (`swing` / `SWINGING`) ↔ Free-Pendulum Coast (`coast` / `COASTING`) |
| **BTN3 Hold** | Open `"SWINGING PIANO"` menu (`Load Tape Take`, `Seed Felt Piano`, `Push Piano (Impulse)`, `Center & Still`, `Rig`, `Direction`, `Speed 1.00x / 0.50x / 2.00x`, `Input Line/Mic`, `Mic Trim`, `Clear Loop`) |
| **BTN4 / BTN5 Press** | Previous / Next encoder parameter (`ROPE`, `ARC`, `DOPPLER`, `HALL`) |
| **BTN4 / BTN5 Hold** | System main menu (standalone TAPP) |
| **Encoder** | Adjust selected parameter |

### Encoder Parameters

| Parameter | Range | Default | Description |
|---|---|---|---|
| `ROPE` | `0–100%` (`2.2m–14.0m`) | `45%` (`7.5m`) | Suspension cable length & gravitational swing period (`~3.0s` studio arc → `~7.5s` Barbican fly-tower sweep) |
| `ARC` | `0–100%` (`0°–55°`) | `55%` (`30°`) | Peak pendulum excursion angle (`0°` stationary center → `±55°` wide concert-hall sweep) |
| `DOPPLER` | `0–100%` | `50%` | Acoustic propagation depth (`0%` pure spatial panning/proximity with zero pitch shift → `50%` exact `343 m/s` hall physics → `100%` dramatic close-mic swoop) |
| `HALL` | `0–100%` | `42%` | Opposite-wall counter-Doppler early reflections & Barbican Concert Hall FDN reverberation tail |

## Building & Verification

```bash
make verify TAPE_SDK=/home/antigravity/tape_sdk
```
