//! I2C self-tests — validates NACK detection on empty bus.
//!
//! REQUIRES: 4.7k pull-up resistors on PTB0 (SCL) and PTB1 (SDA) to 3.3V.
//! Without pull-ups, the bus floats and NACK detection may hang.
//!
//! Priority: MEDIUM
//! Wiring: Pull-up resistors on SCL/SDA to 3.3V

#![no_std]
#![no_main]

extern crate alloc;

use defmt_rtt as _;
use panic_probe as _;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use embedded_hal::i2c::I2c as I2cTrait;
use mk20dx_hal as hal;
use hal::i2c::{Config, I2c, I2c0, I2cExt, calc_frequency, ICR_DIVIDERS, MULT_VALUES};
use hal::pac;
use hal::prelude::*;
use hal::time::U32Ext;

struct State {
    i2c: I2c<I2c0>,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use proptest::test_runner::{Config as ProptestConfig, TestRunner};

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        // Initialize heap allocator for proptest
        unsafe { super::HEAP.init((&raw mut HEAP_MEM) as usize, HEAP_SIZE) }

        let pins_b = dp.portb.split(dp.ptb, &dp.sim);

        // PTB0 = SCL (ALT2), PTB1 = SDA (ALT2)
        let scl = pins_b.pb0.into_alternate::<2>();
        let sda = pins_b.pb1.into_alternate::<2>();

        let config = Config::new(100_000u32.Hz()); // 100 kHz standard mode
        let i2c = dp.i2c0.i2c(scl, sda, config, &clocks, &dp.sim);

        super::State { i2c }
    }

    /// Write to address 0x50 with no device — should get AddressNack.
    #[test]
    fn test_write_nack_on_empty_bus(state: &mut super::State) {
        let result = state.i2c.write(0x50, &[0x00]);
        defmt::info!("Write to 0x50 result: {:?}", defmt::Debug2Format(&result));
        match result {
            Err(hal::i2c::Error::AddressNack) => {
                defmt::info!("Got expected AddressNack on write");
            }
            _ => {
                defmt::panic!("Expected AddressNack, got {:?}", defmt::Debug2Format(&result));
            }
        }
    }

    /// Read from address 0x50 with no device — should get AddressNack.
    #[test]
    fn test_read_nack_on_empty_bus(state: &mut super::State) {
        let mut buf = [0u8; 1];
        let result = state.i2c.read(0x50, &mut buf);
        match result {
            Err(hal::i2c::Error::AddressNack) => {
                defmt::info!("Got expected AddressNack on read");
            }
            _ => {
                defmt::panic!("Expected AddressNack, got {:?}", defmt::Debug2Format(&result));
            }
        }
    }

    /// After a NACK error, the bus should recover and a second attempt should
    /// also return AddressNack (not hang).
    #[test]
    fn test_bus_recovers_after_nack(state: &mut super::State) {
        // First attempt
        let r1 = state.i2c.write(0x50, &[0x00]);
        defmt::assert!(r1.is_err(), "First attempt should NACK");

        // Second attempt — should not hang
        let r2 = state.i2c.write(0x50, &[0x00]);
        defmt::assert!(r2.is_err(), "Second attempt should also NACK");
        defmt::info!("Bus recovered after NACK");
    }

    /// Scan all valid 7-bit addresses (0x08-0x77). All should NACK on empty bus.
    #[test]
    fn test_scan_empty_bus(state: &mut super::State) {
        let mut nack_count = 0u32;
        for addr in 0x08..=0x77 {
            let result = state.i2c.write(addr, &[]);
            if result.is_err() {
                nack_count += 1;
            } else {
                defmt::warn!("Unexpected ACK at address 0x{:02X}", addr);
            }
        }
        defmt::info!("Bus scan: {}/112 addresses NACKed", nack_count);
        defmt::assert_eq!(
            nack_count, 112,
            "All 112 addresses should NACK on empty bus"
        );
    }

    // ----- Property-based tests -----

    /// calc_frequency must return icr < 64 and mult < 3.
    #[test]
    fn test_calc_frequency_index_ranges(_state: &mut super::State) {
        let config = ProptestConfig::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1u32..=100_000_000, 1u32..=1_000_000);

        let result = runner.run(&strategy, |(bus_clk, target)| {
            let (icr, mult) = calc_frequency(bus_clk, target);

            proptest::prop_assert!(
                icr < 64,
                "icr {} out of range for bus_clk={}, target={}",
                icr,
                bus_clk,
                target
            );
            proptest::prop_assert!(
                mult < 3,
                "mult {} out of range for bus_clk={}, target={}",
                mult,
                bus_clk,
                target
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("calc_frequency index ranges: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("calc_frequency index ranges FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// Reconstructed SCL frequency must not exceed the target.
    #[test]
    fn test_calc_frequency_not_exceeding(_state: &mut super::State) {
        let config = ProptestConfig::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1_000_000u32..=72_000_000, 10_000u32..=1_000_000);

        let result = runner.run(&strategy, |(bus_clk, target)| {
            let (icr, mult) = calc_frequency(bus_clk, target);

            let divider = MULT_VALUES[mult as usize] as u32 * ICR_DIVIDERS[icr as usize] as u32;
            let freq = bus_clk / divider;

            proptest::prop_assert!(
                freq <= target,
                "freq {} > target {} for bus_clk={}, icr={}, mult={}",
                freq,
                target,
                bus_clk,
                icr,
                mult
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("calc_frequency not exceeding: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("calc_frequency not exceeding FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// No other valid combo produces a higher frequency still <= target.
    #[test]
    fn test_calc_frequency_optimality(_state: &mut super::State) {
        let config = ProptestConfig::with_cases(16);
        let mut runner = TestRunner::new(config);

        // Realistic bus clocks and targets
        let strategy = (24_000_000u32..=36_000_000, 100_000u32..=400_000);

        let result = runner.run(&strategy, |(bus_clk, target)| {
            let (icr, mult) = calc_frequency(bus_clk, target);

            let chosen_div = MULT_VALUES[mult as usize] as u32 * ICR_DIVIDERS[icr as usize] as u32;
            let chosen_freq = bus_clk / chosen_div;

            // Exhaustively check all combos
            for (mi, &m) in MULT_VALUES.iter().enumerate() {
                for (ii, &d) in ICR_DIVIDERS.iter().enumerate() {
                    let freq = bus_clk / (m as u32 * d as u32);
                    if freq <= target {
                        proptest::prop_assert!(
                            freq <= chosen_freq,
                            "Found better combo: mult_idx={}, icr_idx={} gives {} > chosen {} (target={}, bus_clk={})",
                            mi, ii, freq, chosen_freq, target, bus_clk
                        );
                    }
                }
            }

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("calc_frequency optimality: PASSED (16 cases)"),
            Err(e) => {
                defmt::error!("calc_frequency optimality FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }
}
