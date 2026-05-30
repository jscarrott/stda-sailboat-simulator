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

## Optional LoRa supervisory link

Build with `--features lora` to add a long-range, low-rate channel for **tracking
the boat's position** and **sending it new waypoints** — separate from the fast
USB control loop. The controller stays onboard; LoRa only carries *where it is*
and *where to go*, which is well within LoRa's bandwidth and duty-cycle limits.

- **Boat → shore:** a `PositionReport` (x, y, heading, speed) every ~10 s.
- **Shore → boat:** a `WaypointCmd` (x, y); on receipt the controller steers to
  the bearing of that waypoint instead of the host's commanded heading.

Point-to-point (no LoRaWAN): a matching SX1262 radio at the shore station. Each
LoRa packet carries one bare postcard message (no COBS — a LoRa frame is already
delimited and CRC-checked). Defaults: SF10/125 kHz, **868.1 MHz (EU)**, 14 dBm —
change the frequency in `radio.rs` to a 915 MHz channel for US/Canada.

### Recommended radio + wiring (nRF52840-DK)

- **Module:** a Semtech **SX1262** breakout, e.g. **Waveshare Core1262-HF**
  (868/915 MHz) or **EBYTE E22-900M22S**. Pick the 868 MHz (EU) or 915 MHz (US)
  variant for your region. (Driver: `lora-phy`, which supports the SX1262.)
- **Bus:** SPI3 (kept off the I2C instance the OLED uses). Default DK Arduino-header pins:

  | Signal | nRF pin | Arduino |
  |--------|---------|---------|
  | SCK    | P1.15   | D13     |
  | MISO   | P1.14   | D12     |
  | MOSI   | P1.13   | D11     |
  | NSS    | P1.12   | D10     |
  | RESET  | P1.11   | D9      |
  | BUSY   | P1.10   | D8      |
  | DIO1   | P1.08   | D7      |

  Plus 3V3 + GND, and an antenna for your band. Adjust pins in `radio.rs`.

```bash
cargo build --release --features lora
cargo build --release --features "display lora"   # both options together
```

### Shore station: bridge + host tool

To actually see positions and send waypoints you need a second radio at the
shore. Flash a **second nRF52840-DK** with the **`bridge`** role — same firmware,
it relays USB serial ⇄ LoRa instead of running the controller — and drive it
with the host `shore` tool:

```text
 boat DK (--features lora) ⇄ LoRa ⇄ shore DK (--features bridge) ⇄ USB ⇄ `shore` tool
```

```bash
# shore-station DK (relay; no controller):
cargo build --release --features bridge
cargo run   --release --features bridge        # flash + run

# host tool (in rust_src/, needs the `hil` feature for serial):
cargo run --features hil --bin shore -- --port /dev/ttyACM0 --waypoint 800,950
#   prints PositionReports as they arrive; type more "X,Y" lines to send waypoints.
cargo run --features hil --bin shore -- --loopback   # offline self-test, no hardware
```

When the boat receives a waypoint it steers to that bearing instead of the
host's commanded heading (visible in the `--scenario hil` trace).

> Note: the LoRa path builds for the target but has not been exercised on real
> RF hardware here. The `radio.rs` SX1262 setup follows the `lora-phy` API; if
> your `lora-phy` minor version differs, the radio calls may need small tweaks.

## Recommended screen

A **0.96" 128×64 SSD1306 I2C OLED** (e.g. Adafruit #326, or a generic
HiLetgo/AZ-Delivery SSD1306 module). Make sure it's a genuine **SSD1306** (some
1.3" modules are SH1106, which needs a different driver). Wire SDA→P0.26,
SCL→P0.27, VCC→3V3, GND→GND on the DK; build with `--features display`.
