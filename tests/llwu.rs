//! Low-Leakage Wakeup Unit (LLWU) self-tests — validates pin and module
//! wakeup source configuration via register checks.
//!
//! No low-power modes are entered. These tests only verify that the LLWU
//! registers are configured correctly by the HAL driver.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::llwu::{Llwu, LlwuExt, LlwuModule, LlwuPin, WakeEdge};
use hal::pac;
use hal::prelude::*;

struct State {
    llwu: Llwu,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let llwu = dp.llwu.llwu(&dp.sim);
        super::State { llwu }
    }

    /// Initially, no pin wakeup flags should be set.
    #[test]
    fn test_pin_flags_zero(state: &mut super::State) {
        let flags = state.llwu.pin_flags();
        defmt::info!("Pin flags: 0x{:04X}", flags);
        defmt::assert_eq!(flags, 0, "Pin flags should be 0 initially");
    }

    /// Initially, no module wakeup flags should be set.
    #[test]
    fn test_module_flags_zero(state: &mut super::State) {
        let flags = state.llwu.module_flags();
        defmt::info!("Module flags: 0x{:02X}", flags);
        defmt::assert_eq!(flags, 0, "Module flags should be 0 initially");
    }

    /// Enable pin P5 for falling edge wakeup → PE2 register bits set.
    /// Disable pin P5 → bits cleared.
    #[test]
    fn test_enable_disable_pin(state: &mut super::State) {
        let llwu = unsafe { &*pac::Llwu::PTR };

        // P5 is pin 5 → reg PE2 (pins 4-7), field shift = (5%4)*2 = 2
        state.llwu.enable_pin(LlwuPin::P5, WakeEdge::Falling);
        let pe2 = llwu.pe2().read().bits();
        let field = (pe2 >> 2) & 0x3;
        defmt::info!("PE2 after enable P5 Falling: 0x{:02X}, field=0b{:02b}", pe2, field);
        defmt::assert_eq!(field, 0b10, "P5 should be configured for falling edge (0b10)");

        state.llwu.disable_pin(LlwuPin::P5);
        let pe2 = llwu.pe2().read().bits();
        let field = (pe2 >> 2) & 0x3;
        defmt::info!("PE2 after disable P5: 0x{:02X}, field=0b{:02b}", pe2, field);
        defmt::assert_eq!(field, 0b00, "P5 should be disabled (0b00)");
    }

    /// Enable LPTMR module wakeup → ME bit 0 set.
    /// Disable → bit 0 cleared.
    #[test]
    fn test_enable_disable_module(state: &mut super::State) {
        let llwu = unsafe { &*pac::Llwu::PTR };

        state.llwu.enable_module(LlwuModule::Lptmr);
        let me = llwu.me().read().bits();
        defmt::info!("ME after enable LPTMR: 0x{:02X}", me);
        defmt::assert!(me & 0x01 != 0, "ME bit 0 (LPTMR) should be set");

        state.llwu.disable_module(LlwuModule::Lptmr);
        let me = llwu.me().read().bits();
        defmt::info!("ME after disable LPTMR: 0x{:02X}", me);
        defmt::assert!(me & 0x01 == 0, "ME bit 0 (LPTMR) should be cleared");
    }

    /// Clearing all pin flags on zero flags should not crash and flags should remain 0.
    #[test]
    fn test_clear_all_pin_flags(state: &mut super::State) {
        state.llwu.clear_all_pin_flags();
        let flags = state.llwu.pin_flags();
        defmt::info!("Pin flags after clear: 0x{:04X}", flags);
        defmt::assert_eq!(flags, 0, "Pin flags should still be 0 after clear");
    }
}
