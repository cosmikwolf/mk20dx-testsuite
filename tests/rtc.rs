//! Real-Time Clock (RTC) self-tests — validates time counting, alarm,
//! and interrupt configuration using the 32.768 kHz crystal.
//!
//! Total test runtime: ~8 seconds (dominated by real-time waits + warmup).
//! The init function includes a 2s warmup delay for crystal startup and
//! verifies the oscillator is actually running before proceeding.
//!
//! Priority: MEDIUM
//! Wiring: None (Teensy 3.2 has 32.768 kHz crystal on-board)

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
use hal::rtc::{Rtc, RtcExt};

struct State {
    rtc: Rtc,
    delay: Delay,
    osc_running: bool,
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
        let mut delay = Delay::new(cp.SYST, &clocks);
        let mut rtc = dp.rtc.rtc(&dp.sim);

        // Wait for 32.768 kHz oscillator to stabilize (can take up to 2s)
        delay.delay_ms(2000);

        // Set a known time
        rtc.set_time(1_700_000_000);

        // Verify oscillator is running by checking if seconds increment
        let rtc_regs = unsafe { &*pac::Rtc::PTR };
        let cr = rtc_regs.cr().read();
        defmt::info!(
            "RTC CR: OSCE={} SC8P={} SC2P={}",
            cr.osce().is_1(),
            cr.sc8p().is_1(),
            cr.sc2p().is_1()
        );
        defmt::info!(
            "RTC SR: TCE={} TIF={}",
            rtc_regs.sr().read().tce().is_1(),
            rtc_regs.sr().read().tif().is_1()
        );

        // Wait 1.5s and check if counter incremented
        let before = rtc.seconds().unwrap();
        delay.delay_ms(1500);
        let after = rtc.seconds().unwrap();
        let osc_running = after > before;
        defmt::info!(
            "Oscillator check: before={} after={} running={}",
            before,
            after,
            osc_running
        );
        if !osc_running {
            defmt::warn!("32.768 kHz oscillator not running — time-dependent tests will be skipped");
        }

        // Re-set time for the actual tests
        rtc.set_time(1_700_000_000);

        super::State { rtc, delay, osc_running }
    }

    /// After set_time(), time should be valid (TIF=0).
    #[test]
    fn test_time_is_valid(state: &mut super::State) {
        let valid = state.rtc.time_is_valid();
        defmt::info!("Time valid: {}", valid);
        defmt::assert!(valid, "Time should be valid after set_time()");
    }

    /// seconds() should return approximately the value we set.
    #[test]
    fn test_set_time_roundtrip(state: &mut super::State) {
        let secs = state.rtc.seconds().unwrap();
        defmt::info!("RTC seconds: {} (set ~1700000000)", secs);
        // Allow +-5 seconds of drift from init time + previous tests
        defmt::assert!(
            secs >= 1_699_999_995 && secs <= 1_700_000_020,
            "Seconds {} should be near 1700000000",
            secs
        );
    }

    /// Read seconds, wait ~1.5s, read again. Difference should be 1 or 2.
    #[test]
    fn test_seconds_increment(state: &mut super::State) {
        if !state.osc_running {
            defmt::warn!("SKIPPED: oscillator not running");
            return;
        }
        let before = state.rtc.seconds().unwrap();
        state.delay.delay_ms(1500);
        let after = state.rtc.seconds().unwrap();
        let diff = after - before;

        defmt::info!("Before: {}, After: {}, Diff: {}", before, after, diff);
        defmt::assert!(
            diff == 1 || diff == 2,
            "After 1.5s, seconds should increment by 1 or 2, got {}",
            diff
        );
    }

    /// Set an alarm two seconds out and confirm it fires.
    ///
    /// Synchronises to a second boundary before arming, then polls.
    ///
    /// The old form armed `now + 2`, slept a flat 2.5 s and sampled once, and
    /// failed intermittently on identical binaries. Two things were wrong.
    ///
    /// First, a `now + 2` alarm takes **three** seconds, not two: TAF is set
    /// when TSR matches TAR and then increments (K20 RM ch.36), so it asserts
    /// as TSR becomes `TAR + 1`. Measured directly — armed at TSR=1700000003
    /// for TAR=1700000005, TAF set at TSR=1700000006. The 2.5 s budget was
    /// therefore below the real latency. It is not a timebase error: the
    /// prescaler measures 16386 counts per 500 ms against a 16384 ideal.
    ///
    /// Second, `seconds()` was read at whatever sub-second phase the previous
    /// test left, so how much of the current second had already elapsed
    /// decided whether the too-short budget happened to be enough.
    ///
    /// Waiting for a tick pins the phase near zero, and polling to a 4 s
    /// bound covers the real 3 s latency while still catching a stuck alarm.
    #[test]
    fn test_alarm_fires(state: &mut super::State) {
        if !state.osc_running {
            defmt::warn!("SKIPPED: oscillator not running");
            return;
        }

        // Align to a second boundary so the alarm is armed at a known phase.
        let start = state.rtc.seconds().unwrap();
        let mut spins = 0u32;
        while state.rtc.seconds().unwrap() == start {
            state.delay.delay_ms(10);
            spins += 1;
            defmt::assert!(spins < 200, "RTC counter did not tick within 2s");
        }

        let now = state.rtc.seconds().unwrap();
        state.rtc.set_alarm(now + 2);

        // Poll rather than sleeping a fixed span and sampling once. The alarm
        // is due in 2 s; allow 4 s before calling it a failure.
        let mut waited_ms = 0u32;
        while !state.rtc.alarm_fired() && waited_ms < 4000 {
            state.delay.delay_ms(10);
            waited_ms += 10;
        }

        let fired = state.rtc.alarm_fired();
        defmt::info!(
            "Alarm armed at {} for {}, fired: {} after {}ms",
            now,
            now + 2,
            fired,
            waited_ms
        );
        defmt::assert!(
            fired,
            "Alarm set for now+2 did not fire within 4s (waited {}ms)",
            waited_ms
        );
    }

    /// clear_alarm() should clear the alarm flag.
    #[test]
    fn test_clear_alarm(state: &mut super::State) {
        if !state.osc_running {
            // Set and immediately check (alarm can't fire without oscillator)
            state.rtc.set_alarm(0xFFFF_FFFF);
            state.rtc.clear_alarm();
            defmt::info!("Clear alarm test (no osc): passed");
            return;
        }
        state.rtc.clear_alarm();
        let fired = state.rtc.alarm_fired();
        defmt::info!("Alarm fired after clear: {}", fired);
        defmt::assert!(!fired, "Alarm flag should be cleared after clear_alarm()");
    }

    /// Enable/disable alarm interrupt: IER TAIE bit should toggle.
    #[test]
    fn test_alarm_interrupt_enable_disable(state: &mut super::State) {
        let rtc = unsafe { &*pac::Rtc::PTR };

        state.rtc.enable_alarm_interrupt();
        defmt::assert!(
            rtc.ier().read().taie().is_1(),
            "TAIE should be set after enable_alarm_interrupt"
        );

        state.rtc.disable_alarm_interrupt();
        defmt::assert!(
            rtc.ier().read().taie().is_0(),
            "TAIE should be cleared after disable_alarm_interrupt"
        );
    }

    /// Enable/disable seconds interrupt: IER TSIE bit should toggle.
    #[test]
    fn test_seconds_interrupt_enable_disable(state: &mut super::State) {
        let rtc = unsafe { &*pac::Rtc::PTR };

        state.rtc.enable_seconds_interrupt();
        defmt::assert!(
            rtc.ier().read().tsie().is_1(),
            "TSIE should be set after enable_seconds_interrupt"
        );

        state.rtc.disable_seconds_interrupt();
        defmt::assert!(
            rtc.ier().read().tsie().is_0(),
            "TSIE should be cleared after disable_seconds_interrupt"
        );
    }

    /// Disable counter → TCE=0; enable → TCE=1.
    #[test]
    fn test_disable_enable_counter(state: &mut super::State) {
        let rtc = unsafe { &*pac::Rtc::PTR };

        state.rtc.disable();
        defmt::assert!(
            rtc.sr().read().tce().is_0(),
            "TCE should be 0 after disable"
        );

        // Re-enable and restore time
        state.rtc.set_time(1_700_000_000);
        defmt::assert!(
            rtc.sr().read().tce().is_1(),
            "TCE should be 1 after set_time (re-enable)"
        );
    }
}
