//! Low-Power Timer (LPTMR) self-tests — validates timer operation using
//! the LPO 1 kHz internal clock source.
//!
//! LPO accuracy is ~1 kHz +/-10%. Timing tolerances are generous to
//! accommodate this variation.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::delay::DelayNs;
use mk20dx_hal as hal;
use hal::delay::Delay;
use hal::lptmr::{Lptmr, LptmrClock, LptmrExt, Prescaler};
use hal::pac;
use hal::prelude::*;

struct State {
    lptmr: Lptmr,
    delay: Delay,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let lptmr = dp.lptmr0.lptmr(&dp.sim);
        let delay = Delay::new(cp.SYST, &clocks);
        super::State { lptmr, delay }
    }

    /// After init, the LPTMR clock gate (SIM SCGC5 LPTIMER) should be enabled.
    #[test]
    fn test_clock_gate_enabled(_state: &mut super::State) {
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.scgc5().read().lptimer().is_enabled(),
            "LPTMR clock gate should be enabled"
        );
    }

    /// Start a 500ms timer, immediate wait() should return WouldBlock.
    #[test]
    fn test_start_immediate_would_block(state: &mut super::State) {
        state.lptmr.start(500, LptmrClock::Lpo1kHz);
        let result = state.lptmr.wait();
        defmt::assert!(result.is_err(), "Immediate wait should return WouldBlock");
        state.lptmr.cancel();
    }

    /// Start a 100ms timer, delay 150ms, wait() should return Ok.
    #[test]
    fn test_start_fires(state: &mut super::State) {
        state.lptmr.start(100, LptmrClock::Lpo1kHz);
        state.delay.delay_ms(150);
        let result = state.lptmr.wait();
        defmt::assert!(result.is_ok(), "Timer should have fired after 150ms delay for 100ms period");
    }

    /// Start a 100ms timer, cancel, wait() should return WouldBlock.
    #[test]
    fn test_cancel_stops(state: &mut super::State) {
        state.lptmr.start(100, LptmrClock::Lpo1kHz);
        state.lptmr.cancel();
        let result = state.lptmr.wait();
        defmt::assert!(result.is_err(), "Cancelled timer should return WouldBlock");
    }

    /// Start a 1000ms timer, delay 200ms, counter should be > 0 and < 1000.
    #[test]
    fn test_count_increments(state: &mut super::State) {
        state.lptmr.start(1000, LptmrClock::Lpo1kHz);
        state.delay.delay_ms(200);
        let count = state.lptmr.count();
        state.lptmr.cancel();

        defmt::info!("LPTMR count after 200ms: {}", count);
        defmt::assert!(count > 0, "Counter should be > 0 after 200ms");
        defmt::assert!(count < 1000, "Counter should be < 1000 (not yet expired)");
    }

    /// Enable/disable interrupt: TIE bit should toggle.
    #[test]
    fn test_interrupt_enable_disable(state: &mut super::State) {
        state.lptmr.start(1000, LptmrClock::Lpo1kHz);
        let lptmr = unsafe { &*pac::Lptmr0::PTR };

        state.lptmr.enable_interrupt();
        defmt::assert!(
            lptmr.csr().read().tie().is_1(),
            "TIE should be set after enable_interrupt"
        );

        state.lptmr.disable_interrupt();
        defmt::assert!(
            lptmr.csr().read().tie().is_0(),
            "TIE should be clear after disable_interrupt"
        );

        state.lptmr.cancel();
    }

    /// Start 50ms timer, wait 100ms, TCF should be set.
    /// clear_flag() → TCF cleared.
    #[test]
    fn test_clear_flag(state: &mut super::State) {
        state.lptmr.start(50, LptmrClock::Lpo1kHz);
        state.delay.delay_ms(100);

        let lptmr = unsafe { &*pac::Lptmr0::PTR };
        defmt::assert!(
            lptmr.csr().read().tcf().is_1(),
            "TCF should be set after timer expired"
        );

        state.lptmr.clear_flag();
        defmt::assert!(
            lptmr.csr().read().tcf().is_0(),
            "TCF should be cleared after clear_flag"
        );

        state.lptmr.cancel();
    }

    /// start_raw with LPO, bypass prescaler, compare=200.
    /// CMR should read 200. Timer should fire within ~300ms.
    #[test]
    fn test_start_raw_precise(state: &mut super::State) {
        state.lptmr.start_raw(LptmrClock::Lpo1kHz, Prescaler::Bypass, 200);

        let lptmr = unsafe { &*pac::Lptmr0::PTR };
        let cmr = lptmr.cmr().read().compare().bits();
        defmt::info!("CMR after start_raw(200): {}", cmr);
        defmt::assert_eq!(cmr, 200, "CMR should be 200");

        state.delay.delay_ms(300);
        let result = state.lptmr.wait();
        defmt::assert!(result.is_ok(), "Timer with compare=200 should fire within 300ms");
    }
}
