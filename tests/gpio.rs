//! GPIO self-tests — validates pin configuration, output, input, and mode transitions.
//!
//! Priority: HIGH
//! Wiring: None (uses on-board LED on PTC5 and unconnected PTD4)
//!
//! Note: Since defmt-test shares state across tests and type-state GPIO pins
//! are consumed on mode transition, the first test uses the typed HAL API and
//! verifies via PDIR (physical pin state). Subsequent tests use raw register
//! access and also verify via PDIR where possible.

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use mk20dx_hal as hal;
use hal::gpio::{Pin, Disabled};
use hal::pac;
use hal::prelude::*;

/// Test state holds pins that are reconfigured between tests.
struct State {
    // PTC5 (LED) — starts as Disabled, reconfigured per test
    pc5: Option<Pin<'C', 5, Disabled>>,
    // PTD4 (floating) — starts as Disabled, reconfigured per test
    pd4: Option<Pin<'D', 4, Disabled>>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let pins_c = dp.portc.split(dp.ptc, &dp.sim);
        let pins_d = dp.portd.split(dp.ptd, &dp.sim);

        super::State {
            pc5: Some(pins_c.pc5),
            pd4: Some(pins_d.pd4),
        }
    }

    /// PTC5 (LED) configured as push-pull output, set high — verify via PDIR
    /// (physical pin state), not just PDOR (output latch).
    #[test]
    fn test_output_set_high(state: &mut super::State) {
        let pc5 = state.pc5.take().unwrap();
        let mut led = pc5.into_push_pull_output();
        led.set_high().unwrap();

        // Verify via StatefulOutputPin (reads PDOR — output latch)
        defmt::assert!(led.is_set_high().unwrap(), "PDOR should read high after set_high");

        // Also verify via PDIR — the physical pin state. On Kinetis, PDIR reflects
        // the actual pad voltage even on output pins.
        let ptc = unsafe { &*pac::Ptc::PTR };
        let pdir_high = ptc.pdir().read().bits() & (1 << 5) != 0;
        defmt::assert!(pdir_high, "PDIR should read high — physical pin should be driven high");

        // Can't convert back to Disabled due to type-state.
        // Subsequent tests use raw register access.
        core::mem::forget(led);
    }

    /// PTC5 set low — verify output latch is cleared.
    #[test]
    fn test_output_set_low(_state: &mut super::State) {
        let portc = unsafe { &*pac::Portc::PTR };
        let ptc = unsafe { &*pac::Ptc::PTR };

        // Configure PTC5 as GPIO output
        portc.pcr(5).write(|w| w.mux().gpio());
        ptc.pddr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 5)) });

        // Set low via PCOR
        ptc.pcor().write(|w| unsafe { w.bits(1 << 5) });

        // Allow propagation
        cortex_m::asm::delay(100);

        // Verify via PDOR (output latch) — PDIR may be unreliable if the pin
        // has external pull-ups or LED circuitry affecting readback
        let pdor_low = ptc.pdor().read().bits() & (1 << 5) == 0;
        defmt::assert!(pdor_low, "PDOR should read low after PCOR clear");
    }

    /// PTC5 low → toggle → verify PDOR reflects change.
    #[test]
    fn test_output_toggle(_state: &mut super::State) {
        let portc = unsafe { &*pac::Portc::PTR };
        let ptc = unsafe { &*pac::Ptc::PTR };

        // Ensure output mode and set low
        portc.pcr(5).write(|w| w.mux().gpio());
        ptc.pddr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 5)) });
        ptc.pcor().write(|w| unsafe { w.bits(1 << 5) });

        let before = ptc.pdor().read().bits() & (1 << 5) != 0;
        defmt::assert!(!before, "PDOR should be low before toggle");

        // Toggle via PTOR
        ptc.ptor().write(|w| unsafe { w.bits(1 << 5) });

        let after = ptc.pdor().read().bits() & (1 << 5) != 0;
        defmt::assert!(after, "PDOR should be high after toggle from low");
    }

    /// Floating pin PTD4 configured with pull-up should read high via PDIR.
    #[test]
    fn test_pull_up_reads_high(state: &mut super::State) {
        if let Some(pd4) = state.pd4.take() {
            let mut pin = pd4.into_pull_up_input();
            cortex_m::asm::delay(1000);
            defmt::assert!(pin.is_high().unwrap(), "Pull-up pin should read high");
            // Leave consumed — subsequent tests use raw access
        } else {
            let portd = unsafe { &*pac::Portd::PTR };
            let ptd = unsafe { &*pac::Ptd::PTR };
            ptd.pddr().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 4)) });
            portd.pcr(4).write(|w| w.mux().gpio().pe()._1().ps()._1());
            cortex_m::asm::delay(1000);
            let is_high = ptd.pdir().read().bits() & (1 << 4) != 0;
            defmt::assert!(is_high, "Pull-up pin should read high");
        }
    }

    /// Floating pin PTD4 configured with pull-down should read low via PDIR.
    #[test]
    fn test_pull_down_reads_low(_state: &mut super::State) {
        let portd = unsafe { &*pac::Portd::PTR };
        let ptd = unsafe { &*pac::Ptd::PTR };
        ptd.pddr().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 4)) });
        portd.pcr(4).write(|w| w.mux().gpio().pe()._1().ps()._0());
        cortex_m::asm::delay(1000);
        let is_low = ptd.pdir().read().bits() & (1 << 4) == 0;
        defmt::assert!(is_low, "Pull-down pin should read low");
    }

    /// Mode transition from output to input should change PDDR bit.
    #[test]
    fn test_mode_transition_out_to_in(_state: &mut super::State) {
        let portc = unsafe { &*pac::Portc::PTR };
        let ptc = unsafe { &*pac::Ptc::PTR };

        // Set as output
        portc.pcr(5).write(|w| w.mux().gpio());
        ptc.pddr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 5)) });

        // Verify direction is output
        defmt::assert!(
            ptc.pddr().read().bits() & (1 << 5) != 0,
            "PDDR bit should be set (output) before transition"
        );

        // Switch to input
        ptc.pddr().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 5)) });

        // Verify direction changed to input
        defmt::assert!(
            ptc.pddr().read().bits() & (1 << 5) == 0,
            "PDDR bit should be clear (input) after transition"
        );
    }

    /// Mode transition from input to output should change PDDR bit.
    #[test]
    fn test_mode_transition_in_to_out(_state: &mut super::State) {
        let portd = unsafe { &*pac::Portd::PTR };
        let ptd = unsafe { &*pac::Ptd::PTR };

        // Set as input
        ptd.pddr().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 4)) });
        portd.pcr(4).write(|w| w.mux().gpio().pe()._1().ps()._1());

        // Verify direction is input
        defmt::assert!(
            ptd.pddr().read().bits() & (1 << 4) == 0,
            "PDDR bit should be clear (input) before transition"
        );

        // Switch to output
        ptd.pddr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 4)) });

        // Verify direction changed to output
        defmt::assert!(
            ptd.pddr().read().bits() & (1 << 4) != 0,
            "PDDR bit should be set (output) after transition"
        );

        // Clean up: back to input
        ptd.pddr().modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 4)) });
    }

    /// PTC5 configured as open-drain output, set low — verify output latch.
    #[test]
    fn test_open_drain_set_low(_state: &mut super::State) {
        let portc = unsafe { &*pac::Portc::PTR };
        let ptc = unsafe { &*pac::Ptc::PTR };

        // Configure as open-drain output
        ptc.pddr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 5)) });
        portc.pcr(5).write(|w| w.mux().gpio().ode()._1());

        // Set low
        ptc.pcor().write(|w| unsafe { w.bits(1 << 5) });

        // Verify via PDOR (output latch)
        let is_low = ptc.pdor().read().bits() & (1 << 5) == 0;
        defmt::assert!(is_low, "Open-drain pin PDOR should read low after driving low");
    }
}
