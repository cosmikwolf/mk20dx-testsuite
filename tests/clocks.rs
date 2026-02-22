//! Clock configuration self-tests — validates MCG + SIM clock tree.
//!
//! Priority: CRITICAL
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::clocks::Clocks;
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
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        super::State { clocks }
    }

    /// Core clock should be 72 MHz on Teensy 3.1/3.2.
    /// Cross-validates the Clocks struct against MCG PLL register settings.
    /// PLL output = (XTAL / (PRDIV+1)) * (VDIV+24) = (16 / 8) * 36 = 72 MHz.
    #[test]
    fn test_core_clk_72mhz(state: &mut super::State) {
        // Verify Clocks struct reports correct frequency
        defmt::assert_eq!(
            state.clocks.core_clk().raw(),
            72_000_000,
            "Core clock should be 72 MHz"
        );

        // Cross-validate against MCG hardware registers
        let mcg = unsafe { &*pac::Mcg::PTR };
        let prdiv = mcg.c5().read().prdiv0().bits() as u32; // should be 7
        let vdiv = mcg.c6().read().vdiv0().bits() as u32;   // should be 12
        let pll_output = (16_000_000 / (prdiv + 1)) * (vdiv + 24);
        defmt::info!("PLL math: 16MHz / {} * {} = {} Hz", prdiv + 1, vdiv + 24, pll_output);
        defmt::assert_eq!(pll_output, 72_000_000, "PLL register math should yield 72 MHz");

        // Verify OUTDIV1 = 0 (divide by 1)
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.clkdiv1().read().outdiv1().is_0000(),
            "OUTDIV1 should be 0 (divide by 1)"
        );
    }

    /// Bus clock should be 36 MHz (core / 2).
    /// Verifies against SIM CLKDIV1 OUTDIV2 register.
    #[test]
    fn test_bus_clk_36mhz(state: &mut super::State) {
        defmt::assert_eq!(
            state.clocks.bus_clk().raw(),
            36_000_000,
            "Bus clock should be 36 MHz"
        );

        // Verify OUTDIV2 = 1 means divide-by-2 → 72/2 = 36
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.clkdiv1().read().outdiv2().is_0001(),
            "OUTDIV2 should be 1 (divide by 2)"
        );
    }

    /// Flash clock should be 24 MHz (core / 3).
    /// Verifies against SIM CLKDIV1 OUTDIV4 register.
    #[test]
    fn test_flash_clk_24mhz(state: &mut super::State) {
        defmt::assert_eq!(
            state.clocks.flash_clk().raw(),
            24_000_000,
            "Flash clock should be 24 MHz"
        );

        // Verify OUTDIV4 = 2 means divide-by-3 → 72/3 = 24
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.clkdiv1().read().outdiv4().is_0010(),
            "OUTDIV4 should be 2 (divide by 3)"
        );
    }

    /// After freeze(), the MCG should be in PEE mode with PLL locked.
    #[test]
    fn test_pll_locked(_state: &mut super::State) {
        let mcg = unsafe { &*pac::Mcg::PTR };
        let s = mcg.s().read();

        defmt::assert!(s.lock0().is_1(), "PLL should be locked (LOCK0=1)");
        defmt::assert!(s.pllst().is_1(), "PLL should be selected (PLLST=1)");
        defmt::assert!(
            s.clkst().is_11(),
            "Clock source should be PLL (CLKST=0b11)"
        );
    }

    /// The external oscillator should be initialized after freeze().
    #[test]
    fn test_osc_initialized(_state: &mut super::State) {
        let mcg = unsafe { &*pac::Mcg::PTR };
        defmt::assert!(
            mcg.s().read().oscinit0().bit(),
            "Oscillator should be initialized (OSCINIT0=1)"
        );
    }
}
