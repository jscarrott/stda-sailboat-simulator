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
