# Pendulum Music — Standalone *tape!* TAPP

Modeled after **Steve Reich's August 1968 score *Pendulum Music*** (*"For Microphones, Amplifiers, Speakers and Performers"*).

Simulates **4 suspended microphones (`M1..M4`)** swinging on overhead cables above **4 upward-facing floor speakers** in a shared gallery space at 48 kHz (`float32`), featuring a full **4×4 3D acoustic propagation delay matrix**, **inter-mic constructive & destructive phase interference**, and **two live performance modes**.

## Two Performance Modes (`BTN3` Press — Instant 20 ms Crossfade)

1. **`1:FEEDBACK` (`fbk`)** — The sound output **is** the stereo gallery mix of the 4 coupled microphone-speaker feedback loops:
   - Nonlinear Velocity-Verlet gravitational kinematics (`sinf(theta_i)`) capture true amplitude-dependent period dispersion and staggered cable-length phasing (`SPREAD`).
   - 4-point Hermite cubic fractional delay lines read all 16 speaker-to-microphone paths (`d_ij`), yielding authentic Doppler/Barkhausen pitch bending (`whoo-eee-ooo`), constructive feedback surges & shared-harmonic entrainment when mics swing **in phase**, and destructive acoustic cancellation / partial shifting when mics swing **out of phase** (`COUPLE`).
   - As `DECAY > 0%` settles the 4 pendulums over their speaker cones, rhythmic chirps widen and merge organically into the classic **continuous 4-voice resting feedback chord**.

2. **`2:SOUND VCA` (`vca`)** — The exact same 4-pendulum coupled feedback physics runs continuously, using each mic's post-interference feedback envelope `env[0..3]` to shape the level and live swinging stereo position of **4 distinct stereo sounds (`M1..M4`)**:
   - **4 Built-In Multi-Timbral Sound Banks** (`Organ Quartet`, `Marimba & Bells`, `Tape Choir`, `Bowed Glass` — `Organ Quartet` auto-seeded at launch).
   - **Live Per-Pendulum Recording (`BTN2` Press)**: Record up to 4 distinct Mic/Line sounds directly onto `M1`, `M2`, `M3`, and `M4` (auto-advancing target slot, monitored strictly during recording only).
   - **SD Tape Take Loading**: Load any tape take onto the active mic (`Load Take -> Mic X`) or load Takes 1–4 across all 4 pendulums at once (`Load Takes 1-4 -> M1-M4`).

## Controls

| Control | Action |
| --- | --- |
| **BTN1 Press** | `play` / `stop` (auto-pulls & releases the 4 mics if nearly still) |
| **BTN1 Hold** | Volume overlay (`out` @ `25%` default, `fx send`, `in`, `mic trim`) |
| **BTN2 Press** | `rec` / `done` — capture live Mic/Line audio into target pendulum slot `M1..M4` |
| **BTN2 Hold** | `bounce` — stream stereo gallery output to free SD tape space via DMA windows |
| **BTN3 Press** | `fbk` / `vca` — toggle **Mode 1 (`1:FEEDBACK`)** ↔ **Mode 2 (`2:SOUND VCA`)** |
| **BTN3 Hold** | Open **PENDULUM MUSIC** menu (`Pull & Release Mics`, `Mode`, `Bank`, `Rec Target`, `Load Take -> Mic X`, `Load Takes 1-4 -> M1-M4`, `Release: Unison/Staggered`, `Tuning`, `Input: Line/Mic`, `Mic Trim`, `Re-Seed`) |
| **BTN4 / BTN5** | Select live parameter card (`GAIN`, `COUPLE`, `SPREAD`, `DECAY`) |
| **ENCODER** | Adjust selected parameter (`DECAY = 0% / INF` swings perpetually) |

## Build & Verify

```bash
make
make verify
```
