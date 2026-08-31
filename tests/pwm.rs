//! PWM self-tests — validates FTM register configuration (no oscilloscope needed).
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

extern crate alloc;

use defmt_rtt as _;
use panic_probe as _;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use embedded_hal::pwm::SetDutyCycle;
use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::pwm::{Ftm1Channels, FtmExt, calc_prescaler};
use hal::time::U32Ext;

struct State {
    ftm1: Ftm1Channels,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use proptest::test_runner::{Config, TestRunner};

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        // Initialize heap allocator for proptest
        unsafe { super::HEAP.init((&raw mut HEAP_MEM) as usize, HEAP_SIZE) }

        let mut ftm1 = dp.ftm1.pwm(1000u32.Hz(), &clocks, &dp.sim);
        // Enable ch0 in PWM mode so CnV writes take effect
        ftm1.ch0.set_pwm();
        super::State { ftm1 }
    }

    /// max_duty_cycle() should be nonzero after PWM initialization.
    #[test]
    fn test_max_duty_nonzero(state: &mut super::State) {
        let max = state.ftm1.ch0.max_duty_cycle();
        defmt::info!("max_duty_cycle = {}", max);
        defmt::assert!(max > 0, "max_duty_cycle should be > 0");
    }

    /// At 1 kHz with 36 MHz bus clock, MOD should be ~35999 (36MHz/1kHz - 1).
    #[test]
    fn test_max_duty_reasonable(state: &mut super::State) {
        let max = state.ftm1.ch0.max_duty_cycle();
        defmt::info!("max_duty_cycle = {} (expected ~35999)", max);
        defmt::assert!(
            max > 35000 && max < 36500,
            "MOD should be ~35999 for 1kHz at 36MHz bus, got {}",
            max
        );
    }

    /// set_duty_cycle(0) should succeed and CnV should read 0 after counter reload.
    #[test]
    fn test_set_duty_zero(state: &mut super::State) {
        state.ftm1.ch0.set_duty_cycle(0).unwrap();

        // CnV is double-buffered — wait for counter overflow to latch
        cortex_m::asm::delay(72_000 * 2); // ~2ms at 72 MHz (PWM period is 1ms)

        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        let cnv = ftm1.cv(0).read().val().bits();
        defmt::info!("set_duty_cycle(0): CnV={}", cnv);
        defmt::assert_eq!(cnv, 0, "CnV should be 0 after set_duty_cycle(0)");
    }

    /// set_duty_cycle(max) should succeed and CnV should read max after counter reload.
    #[test]
    fn test_set_duty_max(state: &mut super::State) {
        let max = state.ftm1.ch0.max_duty_cycle();
        state.ftm1.ch0.set_duty_cycle(max).unwrap();

        // CnV is double-buffered — wait for counter overflow to latch
        cortex_m::asm::delay(72_000 * 2);

        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        let cnv = ftm1.cv(0).read().val().bits();
        defmt::info!("set_duty_cycle({}): CnV={}", max, cnv);
        defmt::assert_eq!(cnv, max, "CnV should match max duty cycle");
    }

    /// set_duty_cycle(max/2) should write a CnV value of approximately max/2.
    #[test]
    fn test_set_duty_half(state: &mut super::State) {
        let max = state.ftm1.ch0.max_duty_cycle();
        let half = max / 2;
        state.ftm1.ch0.set_duty_cycle(half).unwrap();

        // CnV is double-buffered — wait for counter overflow to latch
        cortex_m::asm::delay(72_000 * 2);

        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        let cnv = ftm1.cv(0).read().val().bits();
        defmt::info!("Set duty half={}, CnV read back={}", half, cnv);
        defmt::assert_eq!(cnv, half, "CnV should match half duty");
    }

    /// CnSC should have MSB and ELSB set (edge-aligned, high-true) from init enable().
    #[test]
    fn test_enable_channel(_state: &mut super::State) {
        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        let csc = ftm1.csc(0).read();
        defmt::assert!(csc.msb().bit_is_set(), "MSB should be set after enable");
        defmt::assert!(csc.elsb().bit_is_set(), "ELSB should be set after enable");
    }

    /// After init, FTM SC should have CLKS=System and the correct prescaler.
    #[test]
    fn test_ftm_sc_register(_state: &mut super::State) {
        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        let sc = ftm1.sc().read();

        defmt::assert!(
            sc.clks().is_system(),
            "CLKS should be System clock"
        );
        // At 1kHz with 36MHz, prescaler should be div1 (ps=0)
        // since 36MHz/1 / 36000 = 1kHz
        defmt::assert!(
            sc.ps().is_div1(),
            "PS should be div1 for 1kHz at 36MHz bus"
        );
    }

    // ----- Property-based tests -----

    /// calc_prescaler always returns ps_idx in 0..=7.
    #[test]
    fn test_prescaler_output_range(_state: &mut super::State) {
        let config = Config::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1u32..=100_000_000, 1u32..=10_000_000);

        let result = runner.run(&strategy, |(bus_clk, target_freq)| {
            let (ps_idx, _mod_val) = calc_prescaler(bus_clk, target_freq);

            proptest::prop_assert!(
                ps_idx <= 7,
                "ps_idx {} out of range for bus_clk={}, target_freq={}",
                ps_idx,
                bus_clk,
                target_freq
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("prescaler output range: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("prescaler output range FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// The selected prescaler/mod combo should produce a reasonable frequency.
    #[test]
    fn test_prescaler_frequency_bounded(_state: &mut super::State) {
        let config = Config::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1_000_000u32..=72_000_000, 100u32..=1_000_000);

        let result = runner.run(&strategy, |(bus_clk, target_freq)| {
            let (ps_idx, mod_val) = calc_prescaler(bus_clk, target_freq);

            let divider: u32 = 1 << ps_idx;
            let counter_clk = bus_clk / divider;
            let actual_freq = counter_clk / (mod_val as u32 + 1);

            proptest::prop_assert!(
                actual_freq <= target_freq * 2,
                "actual_freq {} > 2 * target {} for bus_clk={}, ps_idx={}, mod_val={}",
                actual_freq,
                target_freq,
                bus_clk,
                ps_idx,
                mod_val
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("prescaler frequency bounded: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("prescaler frequency bounded FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// Fallback case: impossible frequencies should return (7, 0xFFFF).
    #[test]
    fn test_prescaler_fallback(_state: &mut super::State) {
        let (ps_idx, mod_val) = calc_prescaler(1, 1_000_000);
        defmt::info!("fallback: ps_idx={}, mod_val={}", ps_idx, mod_val);
        defmt::assert_eq!(ps_idx, 7, "Should fall back to max prescaler");
        defmt::assert_eq!(mod_val, 0xFFFF, "Should fall back to max mod");
    }
}
