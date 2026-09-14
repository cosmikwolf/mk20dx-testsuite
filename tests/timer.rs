//! PIT timer self-tests — validates periodic interrupt timer channels.
//!
//! Priority: HIGH
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::delay::DelayNs;
use mk20dx_hal as hal;
use hal::delay::Delay;
use hal::pac;
use hal::prelude::*;
use hal::timer::{PitChannels, PitExt};

struct State {
    pit: PitChannels,
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
        let pit = dp.pit.split(&dp.sim, &clocks);
        let delay = Delay::new(cp.SYST, &clocks);
        super::State { pit, delay }
    }

    /// Start ch0 with 1ms period, block until it fires.
    /// Verify LDVAL register was computed correctly from the duration.
    /// Bus clock = 36 MHz → 1ms = 36,000 ticks → LDVAL = 35,999.
    #[test]
    fn test_start_and_wait(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_millis(1));

        // Verify LDVAL was set correctly: 1ms at 36 MHz = 35,999
        let pit = unsafe { &*pac::Pit::PTR };
        let ldval = pit.ldval(0).read().tsv().bits();
        defmt::info!("1ms LDVAL: {} (expected 35999)", ldval);
        defmt::assert_eq!(ldval, 35_999, "LDVAL should be 35999 for 1ms at 36MHz bus");

        // Verify the timer actually fires
        nb::block!(state.pit.ch0.wait()).unwrap();
        defmt::info!("PIT ch0 1ms wait completed");
    }

    /// Start ch0 with 1s period, immediate poll should return WouldBlock.
    #[test]
    fn test_wait_would_block(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_secs(1));
        let result = state.pit.ch0.wait();
        defmt::assert!(result.is_err(), "Immediate poll should return WouldBlock");
        state.pit.ch0.cancel();
    }

    /// Start ch0 with 1s period, cancel, poll should return WouldBlock.
    #[test]
    fn test_cancel(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_secs(1));
        state.pit.ch0.cancel();
        let result = state.pit.ch0.wait();
        defmt::assert!(result.is_err(), "Cancelled timer should return WouldBlock");
    }

    /// PIT counts DOWN — two reads with a delay should show decreasing values.
    #[test]
    fn test_current_counts_down(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_secs(1));
        let first = state.pit.ch0.current();
        cortex_m::asm::delay(10_000);
        let second = state.pit.ch0.current();
        state.pit.ch0.cancel();

        defmt::info!("PIT current: first={} second={}", first, second);
        defmt::assert!(second < first, "PIT should count down");
    }

    /// Before expiry, has_expired() should be false. After expiry, true.
    #[test]
    fn test_has_expired(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_millis(1));
        let before = state.pit.ch0.has_expired();
        state.delay.delay_ms(5);
        let after = state.pit.ch0.has_expired();

        defmt::assert!(!before, "Should not be expired immediately after start");
        defmt::assert!(after, "Should be expired after 5ms for 1ms timer");
    }

    /// After expiry, clear_interrupt() should reset has_expired() to false.
    #[test]
    fn test_clear_interrupt_flag(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_millis(1));
        state.delay.delay_ms(5);
        defmt::assert!(state.pit.ch0.has_expired(), "Should be expired");

        state.pit.ch0.clear_interrupt();
        defmt::assert!(
            !state.pit.ch0.has_expired(),
            "Should not be expired after clear"
        );
    }

    /// Enable interrupt: TIE bit should be set. Disable: cleared.
    #[test]
    fn test_enable_disable_interrupt(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_secs(1));

        state.pit.ch0.enable_interrupt();
        let pit = unsafe { &*pac::Pit::PTR };
        defmt::assert!(
            pit.tctrl(0).read().tie().is_1(),
            "TIE should be set after enable_interrupt"
        );

        state.pit.ch0.disable_interrupt();
        defmt::assert!(
            pit.tctrl(0).read().tie().is_0(),
            "TIE should be clear after disable_interrupt"
        );

        state.pit.ch0.cancel();
    }

    /// Two channels should operate independently.
    #[test]
    fn test_channels_independent(state: &mut super::State) {
        state.pit.ch0.start(fugit::MicrosDurationU32::from_millis(10));
        state.pit.ch1.start(fugit::MicrosDurationU32::from_millis(50));

        // Wait for ch0 (10ms)
        nb::block!(state.pit.ch0.wait()).unwrap();

        // ch1 (50ms) should NOT have expired yet
        defmt::assert!(
            !state.pit.ch1.has_expired(),
            "ch1 (50ms) should not expire when ch0 (10ms) fires"
        );

        // Now wait for ch1
        nb::block!(state.pit.ch1.wait()).unwrap();
        defmt::info!("Both channels completed independently");
    }

    /// After expiry, restarting and waiting again should succeed.
    /// Verify LDVAL is set correctly for both iterations.
    #[test]
    fn test_reload_after_expiry(state: &mut super::State) {
        let pit = unsafe { &*pac::Pit::PTR };

        // First iteration
        state.pit.ch0.start(fugit::MicrosDurationU32::from_millis(1));
        let ldval1 = pit.ldval(0).read().tsv().bits();
        nb::block!(state.pit.ch0.wait()).unwrap();

        // Second iteration
        state.pit.ch0.start(fugit::MicrosDurationU32::from_millis(1));
        let ldval2 = pit.ldval(0).read().tsv().bits();
        nb::block!(state.pit.ch0.wait()).unwrap();

        defmt::info!("Reload: ldval1={} ldval2={}", ldval1, ldval2);
        defmt::assert_eq!(ldval1, 35_999, "First LDVAL should be 35999");
        defmt::assert_eq!(ldval2, 35_999, "Second LDVAL should be 35999");
    }

    /// start_ticks(36_000) should set LDVAL to 36,000 and fire after ~1ms.
    /// Verify LDVAL register directly.
    #[test]
    fn test_start_ticks_raw(state: &mut super::State) {
        state.pit.ch0.start_ticks(36_000);

        // Verify LDVAL was set to the exact tick count
        let pit = unsafe { &*pac::Pit::PTR };
        let ldval = pit.ldval(0).read().tsv().bits();
        defmt::assert_eq!(ldval, 36_000, "LDVAL should be 36000 for start_ticks(36000)");

        // Verify the timer actually fires
        nb::block!(state.pit.ch0.wait()).unwrap();
        defmt::info!("start_ticks(36000) completed");
    }
}
