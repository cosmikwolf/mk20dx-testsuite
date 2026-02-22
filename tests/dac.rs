//! DAC self-tests — validates 12-bit DAC register configuration.
//!
//! DAC0 is present only on MK20DX256 (mk20d7 / Teensy 3.1/3.2).
//!
//! Priority: HIGH
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::dac::{Dac, DacExt, VrefSource};
use hal::pac;
use hal::prelude::*;

struct State {
    dac: Dac,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let dac = dp.dac0.dac(&dp.sim);
        super::State { dac }
    }

    /// After init, SIM SCGC2 DAC0 clock gate should be enabled.
    #[test]
    fn test_clock_gate_enabled(_state: &mut super::State) {
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.scgc2().read().dac0().is_1(),
            "DAC0 clock gate should be enabled"
        );
    }

    /// Initial DAC output value should be 0.
    #[test]
    fn test_initial_value_zero(state: &mut super::State) {
        let val = state.dac.get_value();
        defmt::info!("Initial DAC value: {}", val);
        defmt::assert_eq!(val, 0, "Initial DAC value should be 0");
    }

    /// set_value(2048) → get_value() = 2048 (mid-scale).
    #[test]
    fn test_set_get_midscale(state: &mut super::State) {
        state.dac.set_value(2048);
        let val = state.dac.get_value();
        defmt::info!("DAC mid-scale: {}", val);
        defmt::assert_eq!(val, 2048, "DAC should read 2048 after setting mid-scale");
        state.dac.set_value(0); // restore
    }

    /// set_value(4095) → get_value() = 4095 (full-scale).
    #[test]
    fn test_set_get_max(state: &mut super::State) {
        state.dac.set_value(4095);
        let val = state.dac.get_value();
        defmt::info!("DAC full-scale: {}", val);
        defmt::assert_eq!(val, 4095, "DAC should read 4095 after setting full-scale");
        state.dac.set_value(0); // restore
    }

    /// set_value(0xFFFF) should be masked to 12 bits → get_value() = 4095.
    #[test]
    fn test_value_masking(state: &mut super::State) {
        state.dac.set_value(0xFFFF);
        let val = state.dac.get_value();
        defmt::info!("DAC masked value: {}", val);
        defmt::assert_eq!(val, 4095, "Value above 4095 should be masked to 12 bits");
        state.dac.set_value(0); // restore
    }

    /// VREF selection: Vref2 → DACRFS=1, Vref1 → DACRFS=0.
    #[test]
    fn test_vref_selection(state: &mut super::State) {
        let dac0 = unsafe { &*pac::Dac0::PTR };

        state.dac.set_vref(VrefSource::Vref2);
        defmt::assert!(
            dac0.c0().read().dacrfs().is_1(),
            "DACRFS should be 1 for Vref2"
        );

        state.dac.set_vref(VrefSource::Vref1);
        defmt::assert!(
            dac0.c0().read().dacrfs().is_0(),
            "DACRFS should be 0 for Vref1"
        );
    }
}
