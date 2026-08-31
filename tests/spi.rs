//! SPI baud rate calculation property-based tests.
//!
//! Tests spi::calc_baud with random inputs to verify index ranges,
//! rate-not-exceeding-target, and optimality, and spi::pack_pushr field
//! placement. No hardware wiring needed.
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
use hal::spi::{calc_baud, pack_pushr, BR_SCALERS, PBR_PRESCALERS};

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

    /// pack_pushr places every field where the DSPI PUSHR layout says it goes.
    ///
    /// PUSHR: [31] CONT, [30:28] CTAS, [27] EOQ, [26] CTCNT, [21:16] PCS, [15:0] TXDATA.
    /// `read_dma` builds its dummy word with this, so a misplaced field would
    /// clock the wrong data for a whole transfer.
    #[test]
    fn test_pack_pushr_field_placement(_state: &mut super::State) {
        // One field at a time, everything else zero.
        defmt::assert_eq!(pack_pushr(0xA5, 0, false, false), 0x0000_00A5);
        defmt::assert_eq!(pack_pushr(0xBEEF, 0, false, false), 0x0000_BEEF);
        defmt::assert_eq!(pack_pushr(0, 0x01, false, false), 0x0001_0000);
        defmt::assert_eq!(pack_pushr(0, 0x3F, false, false), 0x003F_0000);
        defmt::assert_eq!(pack_pushr(0, 0, true, false), 0x8000_0000);
        defmt::assert_eq!(pack_pushr(0, 0, false, true), 0x0800_0000);

        // CTAS and CTCNT are hardcoded to 0, so those bits stay clear.
        defmt::assert_eq!(pack_pushr(0xFFFF, 0x3F, true, true) & 0x7400_0000, 0);

        // The AD5676 framing FLXS1 uses: PCS0 held low across two bytes.
        defmt::assert_eq!(pack_pushr(0x12, 1, true, false), 0x8001_0012);
        defmt::assert_eq!(pack_pushr(0x12, 1, false, false), 0x0001_0012);

        defmt::info!("pack_pushr field placement: PASSED");
    }

    /// PCS is a 6-bit field; pack_pushr must mask, not overflow into CTCNT.
    #[test]
    fn test_pack_pushr_masks_pcs(_state: &mut super::State) {
        let config = Config::with_cases(64);
        let mut runner = TestRunner::new(config);

        let strategy = (0u16..=0xFFFF, 0u8..=0xFF);

        let result = runner.run(&strategy, |(data, pcs)| {
            let word = pack_pushr(data, pcs, false, false);

            proptest::prop_assert_eq!(
                word & 0x0000_FFFF, data as u32,
                "txdata corrupted for data={}, pcs={}", data, pcs
            );
            proptest::prop_assert_eq!(
                (word >> 16) & 0x3F, (pcs & 0x3F) as u32,
                "pcs mismatch for data={}, pcs={}", data, pcs
            );
            proptest::prop_assert_eq!(
                word & 0xFFC0_0000, 0,
                "pcs {} overflowed past bit 21 (word {})", pcs, word
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("pack_pushr pcs masking: PASSED (64 cases)"),
            Err(e) => {
                defmt::error!("pack_pushr pcs masking FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }
}
