//! Clock configuration self-tests — validates 120 MHz overclock preset.
//!
//! Priority: HIGH
//! Wiring: None
//!
//! Uses `freeze_at(ClockSpeed::Mhz120)` to configure:
//!   120 MHz core, 60 MHz bus, 24 MHz flash
//! PLL: 16 MHz / 4 = 4 MHz ref × 30 = 120 MHz

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::clocks::{ClockSpeed, Clocks};
use hal::pac;
use hal::prelude::*;

struct State {
    clocks: Clocks,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze_at(ClockSpeed::Mhz120, dp.osc, &dp.sim);
        super::State { clocks }
    }

    /// Core clock should be 120 MHz.
    /// Cross-validates the Clocks struct against MCG PLL register settings.
    /// PLL output = (16 MHz / (3+1)) * (6+24) = 4 * 30 = 120 MHz.
    #[test]
    fn test_core_clk_120mhz(state: &mut super::State) {
        defmt::assert_eq!(
            state.clocks.core_clk().to_raw(),
            120_000_000,
            "Core clock should be 120 MHz"
        );

        // Cross-validate against MCG hardware registers
        let mcg = unsafe { &*pac::Mcg::PTR };
        let prdiv = mcg.c5().read().prdiv0().bits() as u32; // should be 3
        let vdiv = mcg.c6().read().vdiv0().bits() as u32;   // should be 6
        let pll_output = (16_000_000 / (prdiv + 1)) * (vdiv + 24);
        defmt::info!("PLL math: 16MHz / {} * {} = {} Hz", prdiv + 1, vdiv + 24, pll_output);
        defmt::assert_eq!(pll_output, 120_000_000, "PLL register math should yield 120 MHz");

        // Verify OUTDIV1 = 0 (divide by 1)
        let sim = unsafe { &*pac::Sim::PTR };
        let outdiv1 = sim.clkdiv1().read().outdiv1().bits();
        defmt::assert_eq!(outdiv1, 0, "OUTDIV1 should be 0 (divide by 1)");
    }

    /// Bus clock should be 60 MHz (core / 2).
    #[test]
    fn test_bus_clk_60mhz(state: &mut super::State) {
        defmt::assert_eq!(
            state.clocks.bus_clk().to_raw(),
            60_000_000,
            "Bus clock should be 60 MHz"
        );

        let sim = unsafe { &*pac::Sim::PTR };
        let outdiv2 = sim.clkdiv1().read().outdiv2().bits();
        defmt::assert_eq!(outdiv2, 1, "OUTDIV2 should be 1 (divide by 2)");
    }

    /// Flash clock should be 24 MHz (core / 5).
    #[test]
    fn test_flash_clk_24mhz(state: &mut super::State) {
        defmt::assert_eq!(
            state.clocks.flash_clk().to_raw(),
            24_000_000,
            "Flash clock should be 24 MHz"
        );

        let sim = unsafe { &*pac::Sim::PTR };
        let outdiv4 = sim.clkdiv1().read().outdiv4().bits();
        defmt::assert_eq!(outdiv4, 4, "OUTDIV4 should be 4 (divide by 5)");
    }

    /// After freeze_at(), the MCG should be in PEE mode with PLL locked.
    #[test]
    fn test_pll_locked(_state: &mut super::State) {
        let mcg = unsafe { &*pac::Mcg::PTR };
        let s = mcg.s().read();

        defmt::assert!(s.lock0().is_locked(), "PLL should be locked (LOCK0=1)");
        defmt::assert!(s.pllst().is_pll(), "PLL should be selected (PLLST=1)");
        defmt::assert!(
            s.clkst().is_pll(),
            "Clock source should be PLL (CLKST=0b11)"
        );
    }

    /// The external oscillator should be initialized after freeze_at().
    #[test]
    fn test_osc_initialized(_state: &mut super::State) {
        let mcg = unsafe { &*pac::Mcg::PTR };
        defmt::assert!(
            mcg.s().read().oscinit0().bit(),
            "Oscillator should be initialized (OSCINIT0=1)"
        );
    }
}
