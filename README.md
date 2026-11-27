# tape-tapps — Experimental Tape-Music Firmwares for Bedtime *tape!*

A collection of five standalone applications (`AppTypeStandalone`) for **[Bedtime *tape!*](https://github.com/bedtime-llc/tape_sdk)** inspired by landmark works of experimental tape, acoustic-feedback, and physical-process music.

Pre-built, verified `.tapp` binaries are included directly in [`apps/`](./apps) for drag-and-drop SD card installation.

---

## Included Instruments

| App | Binary | Inspiration | Overview |
|---|---|---|---|
| **[Disintegrate](./disintegrate)** | [`apps/disintegrate.tapp`](./apps/disintegrate.tapp) | William Basinski — *The Disintegration Loops* (2002) | Destructive magnetic-tape oxide erosion looper (ported from [`phcuadra/Spotykach`](https://github.com/phcuadra/Spotykach)): C1-periodic spatial oxide flaking, progressive HF dulling, tape saturation warmth, asperity crackle, varispeed/reverse (`-2.00x .. +2.00x`), 32-bit packed stereo Q2.14 (`8–10s` on hardware, `16s` on emulator), and zero-RAM SD/tape loop restore. |
| **[Sitting Room](./sitting_in_a_room)** | [`apps/sitting_in_a_room.tapp`](./apps/sitting_in_a_room.tapp) | Alvin Lucier — *I Am Sitting in a Room* (1969) | Iterative generational room re-recorder (`GEN 1 → GEN N`): plays Generation 1 clean, then re-records the loop in-place on each pass through early wall reflections, Schroeder diffusers, speaker/mic tilt EQ, and an 8-mode ZDF architectural resonator bank (`1969 Room`, `Tiled Chamber`, `Stone Chapel`, `Cistern`), with 32-bit packed stereo Q2.14 (`8–10s` on hardware, `16s` on emulator) and zero-RAM Gen-1 restore. |
| **[Gonna Rain](./gonna_rain)** | [`apps/gonna_rain.tapp`](./apps/gonna_rain.tapp) | Steve Reich — *It's Gonna Rain* (1965) | Dual-deck (Part I) and quad-deck harmonic canon (Part II) Wollensak reel-to-reel phaser: 4-point Hermite cubic tape heads, sample-accurate `(int32 + float)` drift-free phase accumulator, Continuous vs. Canon Dwell (`45°` rational grooves), manual reel-flange scrub, and non-destructive sub-loop cropping (`Full`, `1/2`, `1/4`, `1.8s Tight`). |
| **[Swinging Piano](./swinging_piano)** | [`apps/swinging_piano.tapp`](./apps/swinging_piano.tapp) | Aphex Twin — Barbican Hall *Remote Orchestra* (2012) & *aisatsana [102]* | 2D nonlinear Velocity-Verlet suspended grand-piano pendulum (`2.2m–14.0m` fly-tower cable) swinging over a stationary ORTF stereo mic pair: true $343\text{ m/s}$ Hermite propagation delay Doppler pitch-gliding, ITD, proximity swell, counter-Doppler lateral wall reflections, 4×4 Householder hall FDN, plus a built-in *aisatsana*-inspired felt-piano loop generator. |
| **[Pendulum Music](./pendulum_music)** | [`apps/pendulum_music.tapp`](./apps/pendulum_music.tapp) | Steve Reich — *Pendulum Music* (1968) | 4 suspended microphones swinging over 4 upward-facing floor speakers with coupled acoustic propagation & inter-mic phase interference, settling into a 4-voice resting chord. Instant toggle (`BTN3`) between **Mode 1 (`1:FEEDBACK`)** (coupled mic feedback output) and **Mode 2 (`2:SOUND VCA`)** (each mic's feedback envelope gates its own distinct built-in, recorded, or SD-loaded sound). |

---

## Installing onto *tape!*

1. Connect *tape!* over USB-C and enable **USB Disk Mode** (or insert the microSD card into your computer).
2. Copy the `.tapp` files from [`apps/`](./apps) into the `apps/` directory at the root of your *tape!* SD card.
3. Eject safely. On *tape!*, open the main menu (`BTN4`/`BTN5` hold), hold **`BTN1`** on the **`tape`** cell to open the **apps** switcher, and launch any of the five instruments.

---

## Shared Workflow & Recording Features

All five standalone apps share a unified hardware interface and handheld-mic recording pipeline:

- **Transport & Overlays**:
  - **`BTN1` Press**: Play / Stop
  - **`BTN1` Hold**: Volume overlay (`out`, `fx send`, `in`, `mic trim`)
  - **`BTN2` Press**: Record live input into PSRAM loop / Close loop
  - **`BTN2` Hold**: **Bounce-to-Tape** — streams stereo output onto free space on the mounted SD cassette via lock-free `tape_buffer_t` DMA windows and commits a take mark + loop cue on stop
  - **`BTN3` Press**: Primary process toggle (`erode` / `rerec` / `phase` / `swing` / `fbk↔vca`)
  - **`BTN3` Hold**: Instrument menu (`Load Tape Take`, presets/modes, `Input: Line/Mic`, `Mic Trim`, etc.)
  - **`BTN4` / `BTN5` Press + Encoder**: Select & adjust the 4 live performance parameter cards
  - **`BTN4` / `BTN5` Hold**: System main menu / quick hints
- **Record-Only Input & WET Monitoring Gating**:
  - Live input monitoring (`os_audio_set_monitor`) and firmware WET FX pass-through (`os_audio_set_wet`) are kept strictly **OFF** while stopped or playing so the built-in microphone never bleeds through the speakers, engaging only during active recording (`ST_REC`) and restoring your system settings on exit.
- **Tamed Mic Gain + `Mic Trim`**:
  - Automatically caps hot hardware mic gain on launch/input-switch while respecting manual `in` adjustments, and provides an app-local dB **`Mic Trim`** control (`0.40x` / ~`-8 dB` default) in both the Volume overlay and App Menu.
- **Headphone-Safe `15%` Default Output Volume**:
  - Output trim (`out`) defaults to **`15%`** (`0.045x` / `-26.9 dB` on *tape!*'s dB fader taper instead of `75%` / `0 dB` full scale) with per-sample exponential anti-zipper smoothing for comfortable headphone listening, while **Bounce-to-Tape** continues to capture the pre-trim unity signal at full `0 dB` resolution.
- **4-Layer Mechanical Button-Pop Suppression (Built-In Mic)**:
  - Blanks capture and monitoring until **`85 ms` after `BTN2` is released** on record start (followed by a `25 ms` smoothstep fade-in), trims the final **`110 ms`** of pre-debounce switch-click audio on manual record stop, runs an **`85 Hz`** chassis-thump/DC high-pass filter on mic captures, and applies a **`20 ms`** smoothstep loop splice.

---

## Building from Source

Clone [`bedtime-llc/tape_sdk`](https://github.com/bedtime-llc/tape_sdk) alongside this repository (requires `clang` targeting `armv7em-none-eabihf`, `ld.lld`, `llvm-objdump`/`llvm-readelf`/`llvm-nm`, `python3`, and ImageMagick/`PIL` for icon conversion):

```bash
git clone https://github.com/bedtime-llc/tape_sdk.git
git clone https://github.com/phcuadra/tape-tapps.git
cd tape-tapps

# Build, verify (zero .f64 ops, 100% symbol check), and update apps/*.tapp:
make TAPE_SDK=../tape_sdk
```

You can also drag-and-drop any individual app directory (`disintegrate/`, `sitting_in_a_room/`, `gonna_rain/`, `swinging_piano/`, or `pendulum_music/`) onto **[tapp.b.edti.me](https://tapp.b.edti.me)** to build and test in the browser emulator.

---

## License

Released under the [MIT License](./LICENSE).

