# Betaflight OSD Megademo

<img width="1920" height="1080" alt="vlcsnap-2026-10-15-14h30m40s165" src="https://github.com/user-attachments/assets/7f371b06-e6b9-4272-ba06-07277e0d98e2" />

A real-time demoscene production that runs on an FPV drone flight controller. Everything on
screen is drawn by the board's analog OSD chip (MAX7456 / AT7456E), the one that normally
overlays battery voltage on the camera picture: 30 × 16 characters, white, black or clear. The
music is a software AY-3-8910 playing a Pro Tracker 3 module on the same board.

Released at Zoo Party 2026 (Wild Demo). Full quality render [available on Youtube](https://youtu.be/_8Bu0jS38WI).
This repository is a fork of
[Betaflight](https://github.com/betaflight/betaflight) with the demo added; the original
README is [README.betaflight.md](README.betaflight.md).

## How it works

- **Racing the beam.** The OSD chip reads a line's character codes just before it draws that
  line. Codes changed between two lines of the same text row become new pixels, so one
  18-line text row turns into 3, 6, 9 or 18 independent bands.
- **Logical pixels.** 6 × 6, 3 × 3, 2 × 2 and 2 px × 1 line (180 × 288) pictures from a small
  set of column-mask glyphs written into the font at start.
- **Slot engine.** The chip's HSYNC goes to a timer input (one wire). TIM2 restarts on every
  line, TIM3 paces slots inside it, and DMA writes prepared 16-bit SPI words at exact points of
  every line. The CPU only fills line buffers.
- **Per-line registers.** Row brightness and screen position (VOS / HOS) change per line for
  copper bars, bounces and flashes.
- **Timing from the video.** Every beam position is video time measured from HSYNC, so the
  demo does not care whether the MCU runs on its crystal or its internal RC oscillator.
- **Music.** A PT3 player and an AY-3-8910 synth render 32 kHz audio into a PWM timer by DMA.
  The scenes follow the tracker's rows, kicks and snares.

## Requirements

- An **STM32F4 or STM32F7** flight controller with a **MAX7456 or AT7456E** OSD on **SPI2 or
  SPI3**. Tested on HAKRC F411 (STM32F411, AT7456E). F405, F446, F722 and F745 boards build and
  should work, but are not tested yet. H7 is not supported.
- An analog **PAL** camera, or none: without a camera the OSD chip makes its own PAL signal.
  NTSC is not supported.
- A video capture device or an analog VTX and goggles.
- One wire for HSYNC (below). The audio wiring is optional.
- The demo is for the bench: it disables the motors (it borrows the DShot timer and DMA).
  Reboot before flying, and better flash normal Betaflight back.

## Hardware

### HSYNC wire (required)

<img width="4032" height="3024" alt="IMG_3060" src="https://github.com/user-attachments/assets/3fa1f13f-d302-48ed-8d07-0aae2e5ac783" />

<img width="2048" height="1536" alt="5A9B9E04-F067-4DD4-82D4-3DC7078328DD_1_102_o" src="https://github.com/user-attachments/assets/b66b95b7-33ca-4b1a-be2d-53a130c85b78" />

Solder **pin 18 (HSYNC) of the OSD chip** to a free pad of the flight controller that
connects to **PA0, PA1, PA5, PA15 or PB3** (TIM2 channel 1 or 2). On many boards these are a
UART pad (UART4 TX = PA0, RX = PA1 on F405 / F722), a soft serial pad or an unused LED / PINIO
pad (PB3 is the LED pad on some F7 boards, such as HAKRCF722V2 and MAMBAF722);
check the board's pinout or `resource` in the CLI. The pad must not be in use (no serial port
or other function configured on it).

HSYNC is an open-drain output. The MCU's internal pull-up works, but a **2.2–4.7 kΩ resistor
from the wire to 3.3 V** gives clean edges.

You do not need to configure the pin: at start the demo listens on every free candidate and
uses the one with HSYNC. If it finds none, it prints the free pads to the CLI, for example:

```
osd_demo: failed: no HSYNC. Solder OSD chip pin 18 to one of: TX4 (PA0), RX4 (PA1), PB3
```

### Audio from the LED pad (optional)

The music comes out of the **LED strip pad** as a PWM signal (32 kHz samples, a 128 kHz
carrier on TIM1 / TIM8 pins). Disconnect the LED strip while you use the demo build (the
demo build has no LED strip support); reconnect it when you go back to normal firmware.

The pad needs a timer channel with DMA that the demo does not use itself (TIM2, TIM3 and TIM5
are its own). When the LED pad is in use (for example by the HSYNC wire) or sits on one of
those timers (PB3 = TIM2 on some F7 boards), the sound goes to an I2C pad with a suitable timer
(SCL = PB8 on most F7 boards; the demo defaults turn the baro off), else to the first free UART
pad that has one. `osd_demo play` prints where it is, for example:

```
osd_demo: sound on TX1 (PB6)
```

To choose the pin yourself, build with `EXTRA_FLAGS=-DAY_PWM_PIN=PB6` (example).

Three ways to listen, from that pad to ground:

<img width="2048" height="1536" alt="1920ED22-3027-4BAE-9701-C82D0DAA21C8_1_102_o" src="https://github.com/user-attachments/assets/77b97a7a-0a32-4167-901e-4836629a778a" />


1. **A small speaker directly** (8–32 Ω): LED pad → **270 Ω** → speaker → GND. The resistor
   keeps the pin current near 12 mA. A piezo buzzer sounds bad; use a real speaker.
2. **Line level** (capture card audio input, VTX audio input):
   LED pad → **10 kΩ** → output node; output node → **4.7 kΩ** → GND; **4.7 nF** from the
   output node to GND (low-pass ~10 kHz); output node → **1–10 µF** in series → line input.
   About 1 V peak to peak.
3. **Microphone level** (a camera or recorder microphone input):
   LED pad → **47 kΩ** → output node; output node → **220 Ω** → GND; **68 nF** from the output
   node to GND; output node → **1 µF** in series → mic input. About 15 mV peak to peak. The
   series capacitor also blocks the microphone bias voltage.

Without any audio wiring the demo still runs: its scenes then follow a 120 BPM clock.

## Build

Set up the Betaflight build environment as described in the
[Betaflight build docs](https://betaflight.com/docs/development), then:

```
git clone --recursive https://github.com/350d/betaflight-osd-megademo.git
cd betaflight-osd-megademo
make arm_sdk_install
./build_demo.sh <TARGET>
```

`<TARGET>` is your board's config name, the one Betaflight shows in the CLI `version` / `status`
output (for example `HAKRCF411D`, `MATEKF405SE`). The firmware is in
`obj/betaflight_*_<TARGET>.hex`; flash it with the Betaflight Configurator ("Load firmware
[Local]").

`build_demo.sh` adds the demo (`OSD_DEMO_BUILD`) to a lite build (`CLOUD_BUILD`: no GPS,
blackbox, LED strip, VTX control, telemetry, extra RX protocols and so on). The demo itself
takes about 100 KB of flash and 42 KB of RAM.

### If the firmware does not fit

On 512 KB boards (F411) only the lite build fits. If the build still reports a flash or RAM
overflow:

- Check that you build with `./build_demo.sh`, not plain `make`.
- Drop more features in `src/main/target/common_post.h`, in the `OSD_DEMO_BUILD` block, for
  example:
  ```
  #undef USE_CMS
  #undef USE_GPS
  #undef USE_TELEMETRY
  #undef USE_VTX_COMMON
  #undef USE_SERIALRX_CRSF   // keep the one your receiver uses
  ```
- Remove scenes you do not need: the demo order is `demoOrder[]` in `src/main/io/osd_demo.c`.

## Settings after flashing

Run `defaults` in the CLI once after flashing (or flash with "Full chip erase"). The demo build's
defaults turn on what the demo needs: PAL, the MAX7456 OSD, and a light flight loop. Without
them the screen is not drawn to the bottom (NTSC rows) and the effects stutter (the PID loop
and filters take the CPU). To keep your configuration instead, set these by hand:

```
feature OSD
set vcd_video_system = PAL
set osd_displayport_device = MAX7456
set osd_framerate_hz = 60
set pid_process_denom = 8
set gyro_lpf1_static_hz = 0
set gyro_lpf2_static_hz = 0
set gyro_lpf1_dyn_min_hz = 0
set gyro_lpf1_dyn_max_hz = 0
set simplified_gyro_filter = OFF
set rc_smoothing = OFF
profile 0
set dterm_lpf1_dyn_min_hz = 0
set dterm_lpf1_dyn_max_hz = 0
set dterm_lpf1_static_hz = 0
set dterm_lpf2_static_hz = 0
set simplified_dterm_filter = OFF
save
```

These settings are for the bench, not for flying.

## Use

In the CLI:

```
osd_demo              play the demo (once, then it stops)
osd_demo stop         stop it
osd_demo scene<0..14> run one scene and hold it
osd_demo vsync [auto | internal | external]  sync source from the next start
osd_demo font         show the state of the font backup
osd_demo font restore restore Betaflight's font now
```

Or put the **OSD DEMO** mode on a switch in the Configurator's Modes tab: switch on plays the
demo, switch off stops it.

- **Video sync.** With a camera the demo runs on its timing, so video grabbers keep their sync.
  After the intro (Betaflight's own OSD eaten by a matrix rain), cover the camera lens for a
  black background. Without a camera (checked at each start) the chip's own sync is used.
  `osd_demo vsync internal` or `external` forces one (`auto` is the default; not saved).
- **Font.** At start the demo writes its glyphs into the OSD chip's font memory, and `stop`
  puts Betaflight's font back. On the AT7456E the backup lives in the chip's second glyph bank,
  which is RAM: it survives an FC reset or reflash, but **not a power cut**. If you unplug the
  battery during the demo, upload the font again in the Configurator's Font Manager.

## Render the demo without hardware

`demo/render` builds the firmware's demo code and music on the host, models the AT7456E and
writes video and audio; `make_video.sh` turns them into a 1080p50 MP4 (needs ffmpeg).

```
cd demo/render
./make_video.sh ../osd_megademo.mp4
```

## Files

- `src/main/io/osd_demo.c`: the scenes and the raster engine.
- `src/main/drivers/max7456.c`: the OSD driver with the demo's SPI session, NVM and upper bank access.
- `src/main/io/chiptune.c`, `src/main/io/pt3.c`, `src/main/drivers/ay_synth.c`,
  `src/main/drivers/ay_pwm.c`: the music.
- `demo/`: the music module, the megacolor picture sources, their generators and the host renderer.

## Credits

- Code, design, hardware: **350D** ([x.com/350d](https://x.com/350d)).
- Music: **"Summer Steps" by E.J.Kot** (Andrey Pitolin, Kotsoft),
  [zxart.ee/tune/192405](https://zxart.ee/tune/192405), used with the author's permission.
- PT3 note and volume tables after Vince Weaver's 6502 PT3 library (0BSD).
- Built on [Betaflight](https://github.com/betaflight/betaflight).

## License

GPL-3.0, like Betaflight (see [LICENSE](LICENSE)), with the attribution term in
[NOTICE](NOTICE). The music module `demo/music/01_Summer_Steps.pt3` and its copy in
`src/main/io/chiptune_tracks.c` are not covered by the GPL: all rights stay with the author;
ask him before using it elsewhere.
