//! Analog comparator (CMP) self-tests — validates register configuration
//! and internal 6-bit DAC.
//!
//! Uses CMP0 with internal DAC as plus input and IN0 as minus input.
//! Tests are register-level — no external signal is required.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::cmp::{Cmp, Cmp0, CmpDacVref, CmpExt, Hysteresis, Input};
use hal::pac;
use hal::prelude::*;

struct State {
    cmp: Cmp<Cmp0>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let cmp = dp.cmp0.cmp(Input::INTERNAL_DAC, Input::IN0, &dp.sim);
        super::State { cmp }
    }

    /// After init, the CMP clock gate (SIM SCGC4 CMP) should be enabled.
    #[test]
    fn test_clock_gate_enabled(_state: &mut super::State) {
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.scgc4().read().cmp().is_enabled(),
            "CMP clock gate should be enabled"
        );
    }

    /// After init, the comparator should be enabled (CR1 EN=1).
    #[test]
    fn test_comparator_enabled(_state: &mut super::State) {
        let cmp0 = unsafe { &*pac::Cmp0::PTR };
        defmt::assert!(
            cmp0.cr1().read().en().is_1(),
            "CMP0 should be enabled after init"
        );
    }

    /// set_internal_dac(32, Vin1) → DACCR: DACEN=1, VOSEL=32, VRSEL=0.
    #[test]
    fn test_internal_dac_config(state: &mut super::State) {
        state.cmp.set_internal_dac(32, CmpDacVref::Vin1);
        let cmp0 = unsafe { &*pac::Cmp0::PTR };
        let daccr = cmp0.daccr().read();

        defmt::assert!(daccr.dacen().is_1(), "DACEN should be 1");
        defmt::assert_eq!(daccr.vosel().bits(), 32, "VOSEL should be 32");
        defmt::assert!(daccr.vrsel().is_0(), "VRSEL should be 0 for Vin1");
    }

    /// set_hysteresis(Level3) → CR0 HYSTCTR=0b11.
    /// set_hysteresis(Level0) → CR0 HYSTCTR=0b00.
    #[test]
    fn test_hysteresis(state: &mut super::State) {
        let cmp0 = unsafe { &*pac::Cmp0::PTR };

        state.cmp.set_hysteresis(Hysteresis::Level3);
        defmt::assert_eq!(
            cmp0.cr0().read().hystctr().bits(),
            0b11,
            "HYSTCTR should be 0b11 for Level3"
        );

        state.cmp.set_hysteresis(Hysteresis::Level0);
        defmt::assert_eq!(
            cmp0.cr0().read().hystctr().bits(),
            0b00,
            "HYSTCTR should be 0b00 for Level0"
        );
    }

    /// set_inverted(true) → CR1 INV=1.
    /// set_inverted(false) → CR1 INV=0.
    #[test]
    fn test_inversion(state: &mut super::State) {
        let cmp0 = unsafe { &*pac::Cmp0::PTR };

        state.cmp.set_inverted(true);
        defmt::assert!(cmp0.cr1().read().inv().is_1(), "INV should be 1");

        state.cmp.set_inverted(false);
        defmt::assert!(cmp0.cr1().read().inv().is_0(), "INV should be 0");
    }

    /// Enable rising/falling interrupts → SCR IER/IEF bits set.
    /// Disable interrupts → both cleared.
    #[test]
    fn test_interrupt_enable_disable(state: &mut super::State) {
        let cmp0 = unsafe { &*pac::Cmp0::PTR };

        state.cmp.enable_rising_interrupt();
        defmt::assert!(
            cmp0.scr().read().ier().is_1(),
            "IER should be set after enable_rising_interrupt"
        );

        state.cmp.enable_falling_interrupt();
        defmt::assert!(
            cmp0.scr().read().ief().is_1(),
            "IEF should be set after enable_falling_interrupt"
        );

        state.cmp.disable_interrupts();
        let scr = cmp0.scr().read();
        defmt::assert!(scr.ier().is_0(), "IER should be cleared after disable");
        defmt::assert!(scr.ief().is_0(), "IEF should be cleared after disable");
    }

    /// clear_flags() should clear both CFR and CFF.
    #[test]
    fn test_clear_flags(state: &mut super::State) {
        state.cmp.clear_flags();
        let cmp0 = unsafe { &*pac::Cmp0::PTR };
        let scr = cmp0.scr().read();
        defmt::assert!(scr.cfr().is_0(), "CFR should be cleared after clear_flags");
        defmt::assert!(scr.cff().is_0(), "CFF should be cleared after clear_flags");
    }
}
