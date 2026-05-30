# nRF52840 hardware-in-the-loop controller firmware

This firmware runs the sailboat heading + sail-trim controller on an nRF52840
and talks to the host simulator over USB. It is the embedded half of the
hardware-in-the-loop (HIL) rig:

```
 host: sailboat_sim --scenario hil        device: this firmware
 ┌───────────────────────────┐  USB CDC-ACM  ┌────────────────────────────┐
 │ simulate() physics loop    │ ── sensors ─▶ │ apparent_wind → sail_angle │
 │ HilAutopilot (serial)      │ ◀─ commands ─ │ → HeadingController<f32>   │
 └───────────────────────────┘               └────────────────────────────┘
```

The control logic itself lives in the [`boat-control`](../boat-control) crate
(`#![no_std]`, generic over the float type). The simulator instantiates it at
`f64`; this firmware instantiates it at `f32` to use the Cortex-M4F FPU. There
is only one copy of the control law — the host's bit-exact Python-trace tests
exercise the same code the device runs.

## Protocol

postcard-encoded, COBS-framed messages (`boat_control::proto`):

- Host → device, once at startup: `ConfigPacket` (controller `factor`, gains,
  sample time, sail stretching).
- Host → device, every control tick: `SensorPacket` (raw simulated sensors).
- Device → host, in reply: `CommandPacket` (rudder + sail angle).

## Build

```bash
# from this directory (its .cargo/config.toml pins the target)
cargo build --release
```

This is a standalone crate (its own `[workspace]`), excluded from the host
`sailboat_sim` workspace because it only builds for `thumbv7em-none-eabihf`.
Add the target once with `rustup target add thumbv7em-none-eabihf`.

## Flash & run

```bash
# with a debug probe attached (uses probe-rs, configured as the cargo runner)
cargo run --release
```

Then, on the host, drive the simulated boat through the device:

```bash
cd ../..                       # back to rust_src/
cargo run --features hil -- --scenario hil --hil-port /dev/ttyACM0 --hil-twa-deg 90
```

The host prints the steady-state speed and a short trace; compare it against the
in-process `--scenario polar` run at the same true-wind angle to confirm the
device produces the same control behaviour.

## Optional status screen

Build with `--features display` to drive a 128x64 SSD1306 I2C OLED that shows
the live controller output and sensors each tick:

```
HDG  90  SPD 1.42
AWA  -78 AWS  4.8
RUDDER  -3.2 deg     [====|       ]   (centre-zero rudder bar)
SAIL    41.0 deg     [======      ]   (0..90° sail bar)
```

- `HDG`/`SPD`: heading (deg) and through-water speed (m/s) from the sensor packet.
- `AWA`/`AWS`: apparent wind angle (deg) and speed (m/s), computed on-device.
- `RUDDER`/`SAIL`: the controller's commanded actuator angles, with bar gauges.

Wiring (defaults, change in `main.rs`): **P0.26 = SDA, P0.27 = SCL** on `TWISPI0`
(I2C @ 400 kHz). Without the feature the screen code compiles out to a no-op, so
the base firmware stays ~34 KB (vs ~58 KB with the display). A dead/absent panel
never blocks the controller — init and draw errors are ignored.

```bash
cargo build --release --features display
cargo run   --release --features display    # flash + run with a probe attached
```
