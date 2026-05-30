//! Optional status screen.
//!
//! [`Screen`] is the seam: the control loop calls `show(sensor, command)` once
//! per tick. The default implementation (`()`) is a no-op, so the base firmware
//! carries no display code. Building with `--features display` swaps in
//! [`OledScreen`], a 128x64 SSD1306 I2C OLED rendered with `embedded-graphics`:
//! it shows the controller's rudder/sail output plus heading, boat speed and
//! apparent wind derived from the incoming sensor packet.

use boat_control::proto::{CommandPacket, SensorPacket};

/// Per-tick status sink. Implemented as a no-op for `()` and as a real OLED
/// under the `display` feature.
pub trait Screen {
    fn show(&mut self, sensor: &SensorPacket, command: &CommandPacket);
}

/// No display: do nothing.
impl Screen for () {
    fn show(&mut self, _sensor: &SensorPacket, _command: &CommandPacket) {}
}

#[cfg(feature = "display")]
pub use oled::OledScreen;

#[cfg(feature = "display")]
mod oled {
    use super::Screen;
    use boat_control::{apparent_wind, proto::{CommandPacket, SensorPacket}};
    use core::fmt::Write;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyle, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        primitives::{PrimitiveStyle, Rectangle},
        text::{Baseline, Text},
    };
    use num_traits::Float;
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, Ssd1306};

    const RAD2DEG: f32 = 57.295_78;

    /// SSD1306 over any `display-interface` transport (here an I2C `Twim`).
    pub struct OledScreen<DI> {
        display: Ssd1306<DI, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>,
        text: MonoTextStyle<'static, BinaryColor>,
    }

    impl<DI: WriteOnlyDataCommand> OledScreen<DI> {
        /// Initialise the panel. Returns the wrapper even if init fails (a dead
        /// screen must never take the controller down).
        pub fn new(
            mut display: Ssd1306<DI, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>,
        ) -> Self {
            let _ = display.init();
            let text = MonoTextStyleBuilder::new()
                .font(&FONT_6X10)
                .text_color(BinaryColor::On)
                .build();
            Self { display, text }
        }

        fn line(&mut self, s: &str, y: i32) {
            let _ = Text::with_baseline(s, Point::new(0, y), self.text, Baseline::Top)
                .draw(&mut self.display);
        }

        /// A centre-zero horizontal bar for a signed value in `[-range, range]`.
        fn bar(&mut self, value: f32, range: f32, y: i32) {
            let w = 128i32;
            let h = 6i32;
            let frame = PrimitiveStyle::with_stroke(BinaryColor::On, 1);
            let _ = Rectangle::new(Point::new(0, y), Size::new(w as u32, h as u32))
                .into_styled(frame)
                .draw(&mut self.display);
            let centre = w / 2;
            let frac = (value / range).clamp(-1.0, 1.0);
            let span = (frac * (centre as f32 - 1.0)) as i32;
            let (x, bw) = if span >= 0 { (centre, span) } else { (centre + span, -span) };
            if bw > 0 {
                let fill = PrimitiveStyle::with_fill(BinaryColor::On);
                let _ = Rectangle::new(Point::new(x, y), Size::new(bw as u32, h as u32))
                    .into_styled(fill)
                    .draw(&mut self.display);
            }
        }
    }

    impl<DI: WriteOnlyDataCommand> Screen for OledScreen<DI> {
        fn show(&mut self, s: &SensorPacket, cmd: &CommandPacket) {
            let _ = self.display.clear(BinaryColor::Off);

            let speed = (s.vel_x_body * s.vel_x_body + s.vel_y_body * s.vel_y_body).sqrt();
            let (awa, aws) = apparent_wind(
                s.heading,
                s.vel_x_body,
                s.vel_y_body,
                s.true_wind_dir,
                s.true_wind_speed,
            );
            let mut heading_deg = (s.heading * RAD2DEG) % 360.0;
            if heading_deg < 0.0 {
                heading_deg += 360.0;
            }

            let mut buf: heapless::String<24> = heapless::String::new();
            let _ = write!(buf, "HDG {:>3.0}  SPD {:>4.2}", heading_deg, speed);
            self.line(&buf, 0);

            buf.clear();
            let _ = write!(buf, "AWA {:>4.0} AWS {:>4.1}", awa * RAD2DEG, aws);
            self.line(&buf, 11);

            buf.clear();
            let _ = write!(buf, "RUDDER {:>+5.1} deg", cmd.rudder_angle * RAD2DEG);
            self.line(&buf, 24);
            self.bar(cmd.rudder_angle, 20.0_f32.to_radians(), 34);

            buf.clear();
            let _ = write!(buf, "SAIL   {:>5.1} deg", cmd.sail_angle * RAD2DEG);
            self.line(&buf, 44);
            // Sail magnitude 0..90°, drawn as a left-anchored bar (offset so the
            // centre-zero `bar` fills from the left edge).
            self.bar(cmd.sail_angle - 45.0_f32.to_radians(), 45.0_f32.to_radians(), 54);

            let _ = self.display.flush();
        }
    }
}
