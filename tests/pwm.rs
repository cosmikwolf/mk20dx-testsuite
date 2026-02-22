//! PWM self-tests — validates FTM register configuration (no oscilloscope needed).
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::pwm::SetDutyCycle;
use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::pwm::{Ftm1Channels, FtmExt};
use hal::time::U32Ext;

struct State {
    ftm1: Ftm1Channels,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let mut ftm1 = dp.ftm1.pwm(1000u32.Hz(), &clocks, &dp.sim);
        // Enable ch0 in PWM mode so CnV writes take effect
        ftm1.ch0.enable();
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
}
