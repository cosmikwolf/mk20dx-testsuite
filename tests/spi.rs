//! SPI baud rate calculation property-based tests.
//!
//! Tests spi::calc_baud with random inputs to verify index ranges,
//! rate-not-exceeding-target, and optimality. No hardware wiring needed.
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

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::spi::{calc_baud, BR_SCALERS, PBR_PRESCALERS};

struct State {}

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

        // Initialize heap allocator for proptest
        unsafe { super::HEAP.init((&raw mut HEAP_MEM) as usize, HEAP_SIZE) }

        super::State {}
    }

    /// br_idx must be < 16 and pbr_idx must be < 4.
    #[test]
    fn test_index_ranges(_state: &mut super::State) {
        let config = Config::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1u32..=100_000_000, 1u32..=50_000_000);

        let result = runner.run(&strategy, |(bus_clk, target)| {
            let (br_idx, pbr_idx, _dbr) = calc_baud(bus_clk, target);

            proptest::prop_assert!(
                br_idx < 16,
                "br_idx {} out of range for bus_clk={}, target={}",
                br_idx,
                bus_clk,
                target
            );
            proptest::prop_assert!(
                pbr_idx < 4,
                "pbr_idx {} out of range for bus_clk={}, target={}",
                pbr_idx,
                bus_clk,
                target
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("index ranges: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("index ranges FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// Reconstructed baud rate must not exceed the target.
    #[test]
    fn test_rate_not_exceeding_target(_state: &mut super::State) {
        let config = Config::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1_000_000u32..=72_000_000, 100u32..=36_000_000);

        let result = runner.run(&strategy, |(bus_clk, target)| {
            let (br_idx, pbr_idx, dbr) = calc_baud(bus_clk, target);

            let br = BR_SCALERS[br_idx as usize] as u64;
            let pbr = PBR_PRESCALERS[pbr_idx as usize] as u64;
            let mult: u64 = if dbr { 2 } else { 1 };
            let actual = (bus_clk as u64 * mult / (pbr * br)) as u32;

            proptest::prop_assert!(
                actual <= target,
                "actual {} > target {} for bus_clk={}, br_idx={}, pbr_idx={}, dbr={}",
                actual,
                target,
                bus_clk,
                br_idx,
                pbr_idx,
                dbr
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("rate not exceeding target: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("rate not exceeding target FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// No other valid combo produces a higher baud rate still <= target.
    #[test]
    fn test_optimality(_state: &mut super::State) {
        let config = Config::with_cases(16);
        let mut runner = TestRunner::new(config);

        // Realistic bus clocks and targets
        let strategy = (24_000_000u32..=36_000_000, 100_000u32..=10_000_000);

        let result = runner.run(&strategy, |(bus_clk, target)| {
            let (br_idx, pbr_idx, dbr) = calc_baud(bus_clk, target);

            let br = BR_SCALERS[br_idx as usize] as u64;
            let pbr = PBR_PRESCALERS[pbr_idx as usize] as u64;
            let mult: u64 = if dbr { 2 } else { 1 };
            let chosen_baud = (bus_clk as u64 * mult / (pbr * br)) as u32;

            // Exhaustively check all combos
            for (pi, &p) in PBR_PRESCALERS.iter().enumerate() {
                for (bi, &b) in BR_SCALERS.iter().enumerate() {
                    for d in [false, true] {
                        let m: u64 = if d { 2 } else { 1 };
                        let baud = (bus_clk as u64 * m / (p as u64 * b as u64)) as u32;
                        if baud <= target {
                            proptest::prop_assert!(
                                baud <= chosen_baud,
                                "Found better combo: pbr_idx={}, br_idx={}, dbr={} gives {} > chosen {} (target={}, bus_clk={})",
                                pi, bi, d, baud, chosen_baud, target, bus_clk
                            );
                        }
                    }
                }
            }

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("optimality: PASSED (16 cases)"),
            Err(e) => {
                defmt::error!("optimality FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }
}
