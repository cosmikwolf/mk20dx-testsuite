//! SPI per-device SCK rate: `Baud` and `Spi::set_baud`.
//!
//! Checks the CTAR0 fields that `set_baud` writes, that the module runs
//! again after the write, and that the same rate again is a no-op. No frame
//! is sent, so nothing on the bus sees a clock.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::clocks::{ClockSpeed, Clocks};
use hal::pac;
use hal::prelude::*;
use hal::spi::{Baud, Config, Spi, Spi0, SpiExt};
use hal::time::U32Ext;

struct State {
    spi: Spi<Spi0>,
    clocks: Clocks,
}

fn mcr_halted() -> bool {
    unsafe { &*pac::Spi0::PTR }.mcr().read().halt().bit_is_set()
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze_at(ClockSpeed::Mhz120, dp.osc, &dp.sim);
        let pins_c = dp.portc.split(dp.ptc, &dp.sim);
        let sck = pins_c.pc5.into_alternate::<2>();
        let mosi = pins_c.pc6.into_alternate::<2>();
        let miso = pins_c.pc7.into_alternate::<2>();
        let spi = dp.spi0.spi(sck, mosi, miso, Config::new(15_000_000u32.Hz()), &clocks, &dp.sim);
        super::State { spi, clocks }
    }

    /// The init rate reads back as `Baud::new` of the same target.
    #[test]
    fn init_rate_reads_back(state: &mut super::State) {
        let bus = state.clocks.bus_clk().to_Hz();
        defmt::assert_eq!(bus, 60_000_000);
        let want = Baud::new(&state.clocks, 15_000_000u32.Hz());
        defmt::assert_eq!(state.spi.baud(), want);
        defmt::assert_eq!(want.rate(bus), 15_000_000);
    }

    /// `set_baud` writes the fields, leaves the module running, and puts the
    /// old rate back.
    #[test]
    fn set_baud_switches_and_restores(state: &mut super::State) {
        let bus = state.clocks.bus_clk().to_Hz();
        let fast = Baud::new(&state.clocks, 15_000_000u32.Hz());
        let slow = Baud::new(&state.clocks, 10_000_000u32.Hz());
        defmt::assert_eq!(slow.rate(bus), 10_000_000);
        state.spi.set_baud(slow);
        defmt::assert_eq!(state.spi.baud(), slow);
        defmt::assert!(!mcr_halted(), "running after set_baud");
        state.spi.set_baud(fast);
        defmt::assert_eq!(state.spi.baud(), fast);
        defmt::assert!(!mcr_halted(), "running after the restore");
    }

    /// The same rate again changes nothing.
    #[test]
    fn same_rate_is_a_no_op(state: &mut super::State) {
        let fast = Baud::new(&state.clocks, 15_000_000u32.Hz());
        state.spi.set_baud(fast);
        let ctar = unsafe { &*pac::Spi0::PTR }.spi0_ctar(0).read().bits();
        state.spi.set_baud(fast);
        defmt::assert_eq!(unsafe { &*pac::Spi0::PTR }.spi0_ctar(0).read().bits(), ctar);
    }

    /// One 8-bit frame at 15 MHz (PBR /2, BR /2) with the init's minimum
    /// delays (each 1 x 2^1) is 8 x 4 + 3 x 2 = 38 bus cycles; at 30 MHz
    /// (DBR) 8 x 2 + 6 = 22; at 10 MHz (PBR /2, BR /6, DBR) 8 x 6 + 6 = 54.
    #[test]
    fn frame_bus_cycles_at_board_rates(_state: &mut super::State) {
        let min = [(0u8, 0u8); 3];
        defmt::assert_eq!(hal::spi::frame_bus_cycles(8, 0, 0, false, min), 38);
        defmt::assert_eq!(hal::spi::frame_bus_cycles(8, 0, 0, true, min), 22);
        defmt::assert_eq!(hal::spi::frame_bus_cycles(8, 2, 0, true, min), 54);
        defmt::assert_eq!(hal::spi::frame_bus_cycles(8, 0, 3, false, [(3, 2), (0, 0), (0, 0)]), 8 * 14 + 7 * 8 + 4);
    }

    /// `rate` agrees with `calc_baud` for the board's three rates.
    #[test]
    fn rates_at_60_mhz(_state: &mut super::State) {
        for (target, want) in [(30_000_000u32, 30_000_000u32), (20_000_000, 20_000_000), (10_000_000, 10_000_000)] {
            defmt::assert_eq!(Baud::from_bus_clk(60_000_000, target).rate(60_000_000), want);
        }
    }
}
