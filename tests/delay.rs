//! Delay self-tests — validates SysTick-based DelayNs, cross-checked via PIT.
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
use hal::timer::{PitChannel, PitExt};

struct State {
    delay: Delay,
    pit_ch0: PitChannel<0>,
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
        let delay = Delay::new(cp.SYST, &clocks);
        let pit = dp.pit.split(&dp.sim, &clocks);
        super::State {
            delay,
            pit_ch0: pit.ch0,
        }
    }

    /// 1ms delay should complete and take approximately 1ms (PIT crosscheck).
    /// Bus clock = 36 MHz → 1ms = 36,000 ticks. Allow ±20% for short delays.
    #[test]
    fn test_delay_1ms_completes(state: &mut super::State) {
        state.pit_ch0.start_ticks(u32::MAX);
        let before = state.pit_ch0.current();
        state.delay.delay_ms(1);
        let after = state.pit_ch0.current();
        state.pit_ch0.cancel();

        let elapsed = before.wrapping_sub(after);
        let expected: u32 = 36_000;
        let tolerance = expected / 5; // ±20% for 1ms (short delay, more jitter)
        defmt::info!("1ms delay: elapsed={} expected={}", elapsed, expected);
        defmt::assert!(
            elapsed > expected - tolerance && elapsed < expected + tolerance,
            "1ms delay should be within ±20% of 36k PIT ticks, got {}",
            elapsed
        );
    }

    /// 100ms delay should complete and be accurate (PIT crosscheck, ±2%).
    #[test]
    fn test_delay_100ms_completes(state: &mut super::State) {
        state.pit_ch0.start_ticks(u32::MAX);
        let before = state.pit_ch0.current();
        state.delay.delay_ms(100);
        let after = state.pit_ch0.current();
        state.pit_ch0.cancel();

        let elapsed = before.wrapping_sub(after);
        let expected: u32 = 3_600_000;
        let tolerance = expected / 50; // ±2%
        defmt::info!("100ms delay: elapsed={} expected={}", elapsed, expected);
        defmt::assert!(
            elapsed > expected - tolerance && elapsed < expected + tolerance,
            "100ms delay should be within ±2% of 3.6M PIT ticks, got {}",
            elapsed
        );
    }

    /// 1s delay exercises the SysTick loop path (24-bit reload max ~233ms at 72MHz).
    /// PIT crosscheck with ±2% tolerance.
    #[test]
    fn test_delay_1s_completes(state: &mut super::State) {
        state.pit_ch0.start_ticks(u32::MAX);
        let before = state.pit_ch0.current();
        state.delay.delay_ms(1000);
        let after = state.pit_ch0.current();
        state.pit_ch0.cancel();

        let elapsed = before.wrapping_sub(after);
        let expected: u32 = 36_000_000;
        let tolerance = expected / 50; // ±2%
        defmt::info!("1s delay: elapsed={} expected={}", elapsed, expected);
        defmt::assert!(
            elapsed > expected - tolerance && elapsed < expected + tolerance,
            "1s delay should be within ±2% of 36M PIT ticks, got {}",
            elapsed
        );
    }

    /// 1us delay via delay_ns(1000). PIT crosscheck with generous tolerance
    /// since sub-microsecond timing has high relative jitter.
    #[test]
    fn test_delay_ns_1us(state: &mut super::State) {
        state.pit_ch0.start_ticks(u32::MAX);
        let before = state.pit_ch0.current();
        state.delay.delay_ns(1000);
        let after = state.pit_ch0.current();
        state.pit_ch0.cancel();

        let elapsed = before.wrapping_sub(after);
        // 1us = 36 ticks at 36 MHz. Allow large tolerance (0-200 ticks)
        // because overhead of function calls dominates at this scale.
        defmt::info!("1us delay: elapsed={} ticks (expected ~36)", elapsed);
        defmt::assert!(
            elapsed < 200,
            "1us delay should complete in under ~5us of PIT ticks, got {}",
            elapsed
        );
    }

    /// Zero-length delay should return nearly immediately.
    #[test]
    fn test_delay_zero(state: &mut super::State) {
        state.pit_ch0.start_ticks(u32::MAX);
        let before = state.pit_ch0.current();
        state.delay.delay_ns(0);
        let after = state.pit_ch0.current();
        state.pit_ch0.cancel();

        let elapsed = before.wrapping_sub(after);
        // Zero delay should complete in under 100 PIT ticks (~2.7us of overhead)
        defmt::info!("Zero delay: elapsed={} ticks", elapsed);
        defmt::assert!(
            elapsed < 100,
            "Zero delay should complete nearly instantly, got {} ticks",
            elapsed
        );
    }

    /// Cross-check 10ms delay against PIT free-running counter.
    /// Bus clock = 36 MHz -> 10ms = 360,000 ticks. Allow +/-5%.
    #[test]
    fn test_delay_10ms_pit_crosscheck(state: &mut super::State) {
        // Start PIT with max value (free-running countdown)
        state.pit_ch0.start_ticks(u32::MAX);

        let before = state.pit_ch0.current();
        state.delay.delay_ms(10);
        let after = state.pit_ch0.current();

        state.pit_ch0.cancel();

        // PIT counts DOWN, so elapsed = before - after
        let elapsed = before.wrapping_sub(after);
        let expected: u32 = 360_000; // 10ms * 36MHz
        let tolerance = expected / 20; // +/-5%

        defmt::info!(
            "PIT crosscheck 10ms: elapsed={} expected={} tolerance={}",
            elapsed,
            expected,
            tolerance
        );
        defmt::assert!(
            elapsed > expected - tolerance && elapsed < expected + tolerance,
            "10ms delay should be within +/-5% of 360k PIT ticks"
        );
    }

}
