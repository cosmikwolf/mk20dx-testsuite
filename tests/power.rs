//! Power mode control (SMC) self-tests — validates initial state queries
//! and mode protection configuration.
//!
//! These tests only query and configure registers. No low-power modes
//! are actually entered (would halt the test runner).
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::power::{PowerControl, PowerMode, SmcExt};
use hal::prelude::*;

struct State {
    power: PowerControl,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let power = dp.smc.power_control(&dp.sim);
        super::State { power }
    }

    /// After reset, the chip should be in normal Run mode.
    #[test]
    fn test_initial_mode_is_run(state: &mut super::State) {
        let mode = state.power.current_mode();
        defmt::info!("Current power mode: {:?}", mode);
        defmt::assert!(
            mode == PowerMode::Run,
            "Initial power mode should be Run"
        );
    }

    /// No stop mode was entered, so stop_aborted() should be false.
    #[test]
    fn test_stop_not_aborted(state: &mut super::State) {
        let aborted = state.power.stop_aborted();
        defmt::info!("Stop aborted: {}", aborted);
        defmt::assert!(!aborted, "No stop was attempted, so stop_aborted should be false");
    }

    /// allow_vlp() should set the AVLP bit in PMPROT.
    /// Note: PMPROT can only be written once after reset.
    #[test]
    fn test_allow_vlp(state: &mut super::State) {
        state.power.allow_vlp();
        let smc = unsafe { &*pac::Smc::PTR };
        defmt::assert!(
            smc.pmprot().read().avlp().is_1(),
            "PMPROT AVLP should be set after allow_vlp()"
        );
    }
}
