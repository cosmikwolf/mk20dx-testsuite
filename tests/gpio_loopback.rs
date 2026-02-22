//! GPIO loopback tests — validates output driving input via external wire.
//!
//! REQUIRES: Wire connecting PTD5 (Teensy pin 20) to PTD6 (Teensy pin 21).
//!
//! Priority: MEDIUM
//! Wiring: PTD5 → PTD6

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::digital::{InputPin, OutputPin};
use mk20dx_hal as hal;
use hal::gpio::{Pin, Output, PushPull, Input, PullDown};
use hal::pac;
use hal::prelude::*;

struct State {
    out_pin: Pin<'D', 5, Output<PushPull>>,
    in_pin: Pin<'D', 6, Input<PullDown>>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let pins_d = dp.portd.split(dp.ptd, &dp.sim);

        let out_pin = pins_d.pd5.into_push_pull_output();
        let in_pin = pins_d.pd6.into_pull_down_input();

        super::State { out_pin, in_pin }
    }

    /// PTD5 output → PTD6 input. Set high/low and verify PTD6 reads match.
    #[test]
    fn test_output_drives_input(state: &mut super::State) {
        // Set high
        state.out_pin.set_high().unwrap();
        cortex_m::asm::delay(1000); // Short settling time
        defmt::assert!(
            state.in_pin.is_high().unwrap(),
            "PTD6 should read HIGH when PTD5 drives HIGH"
        );

        // Set low
        state.out_pin.set_low().unwrap();
        cortex_m::asm::delay(1000);
        defmt::assert!(
            state.in_pin.is_low().unwrap(),
            "PTD6 should read LOW when PTD5 drives LOW"
        );
    }

    /// Toggle PTD5 10 times, verify PTD6 follows each time.
    #[test]
    fn test_toggle_reflected(state: &mut super::State) {
        for i in 0..10u32 {
            if i % 2 == 0 {
                state.out_pin.set_high().unwrap();
                cortex_m::asm::delay(1000);
                defmt::assert!(
                    state.in_pin.is_high().unwrap(),
                    "Iteration {}: PTD6 should read HIGH",
                    i
                );
            } else {
                state.out_pin.set_low().unwrap();
                cortex_m::asm::delay(1000);
                defmt::assert!(
                    state.in_pin.is_low().unwrap(),
                    "Iteration {}: PTD6 should read LOW",
                    i
                );
            }
        }
        defmt::info!("All 10 toggle iterations matched");
    }
}
