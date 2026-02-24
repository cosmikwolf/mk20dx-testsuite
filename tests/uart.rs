//! UART baud rate calculation property-based tests.
//!
//! Tests uart::calc_baud with random inputs to verify output ranges
//! and accuracy. No hardware wiring needed — pure computation tests.
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
use hal::uart::calc_baud;

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

    /// SBR must be in range 1..=0x1FFF for any valid inputs.
    #[test]
    fn test_sbr_range(_state: &mut super::State) {
        let config = Config::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1u32..=120_000_000, 1u32..=10_000_000);

        let result = runner.run(&strategy, |(module_clk, baudrate)| {
            let (sbr, _brfa) = calc_baud(module_clk, baudrate);

            proptest::prop_assert!(
                sbr >= 1 && sbr <= 0x1FFF,
                "SBR {} out of range for module_clk={}, baudrate={}",
                sbr,
                module_clk,
                baudrate
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("SBR range: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("SBR range FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// BRFA must be in range 0..=31 (5-bit field).
    #[test]
    fn test_brfa_range(_state: &mut super::State) {
        let config = Config::with_cases(32);
        let mut runner = TestRunner::new(config);

        let strategy = (1u32..=120_000_000, 1u32..=10_000_000);

        let result = runner.run(&strategy, |(module_clk, baudrate)| {
            let (_sbr, brfa) = calc_baud(module_clk, baudrate);

            proptest::prop_assert!(
                brfa <= 31,
                "BRFA {} out of range for module_clk={}, baudrate={}",
                brfa,
                module_clk,
                baudrate
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("BRFA range: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("BRFA range FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// For realistic clocks and standard baud rates, actual rate should be
    /// within 5% of target.
    #[test]
    fn test_accuracy_realistic(_state: &mut super::State) {
        let config = Config::with_cases(32);
        let mut runner = TestRunner::new(config);

        // Realistic module clocks: 48 or 72 MHz; standard baud rates
        let strategy = (0u32..=1, 0u32..=5);

        let result = runner.run(&strategy, |(clk_idx, baud_idx)| {
            let module_clk = if clk_idx == 0 { 48_000_000 } else { 72_000_000 };
            let baudrate = match baud_idx % 6 {
                0 => 9600,
                1 => 19200,
                2 => 38400,
                3 => 57600,
                4 => 115200,
                _ => 230400,
            };

            let (sbr, brfa) = calc_baud(module_clk, baudrate);

            // Reconstruct: baud = module_clk / (16 * (SBR + BRFA/32))
            // = module_clk * 32 / (16 * (32*SBR + BRFA))
            // = module_clk * 2 / (32*SBR + BRFA)
            let divisor = 32u64 * sbr as u64 + brfa as u64;
            if divisor == 0 {
                return Ok(());
            }
            let actual = (module_clk as u64 * 2 / divisor) as u32;

            let error_pct = if actual > baudrate {
                ((actual - baudrate) as u64 * 100) / baudrate as u64
            } else {
                ((baudrate - actual) as u64 * 100) / baudrate as u64
            };

            proptest::prop_assert!(
                error_pct <= 5,
                "Baud error {}% too high: target={}, actual={}, clk={}, sbr={}, brfa={}",
                error_pct,
                baudrate,
                actual,
                module_clk,
                sbr,
                brfa
            );

            Ok(())
        });

        match result {
            Ok(()) => defmt::info!("accuracy realistic: PASSED (32 cases)"),
            Err(e) => {
                defmt::error!("accuracy realistic FAILED: {}", e);
                defmt::assert!(false, "Property test failed");
            }
        }
    }

    /// calc_baud should not panic on extreme inputs.
    #[test]
    fn test_no_panic_extremes(_state: &mut super::State) {
        // Near u32::MAX module_clk
        let (sbr, brfa) = calc_baud(u32::MAX, 9600);
        defmt::info!("u32::MAX clk: sbr={}, brfa={}", sbr, brfa);
        defmt::assert!(sbr >= 1 && sbr <= 0x1FFF);
        defmt::assert!(brfa <= 31);

        // Very low baudrate
        let (sbr, brfa) = calc_baud(72_000_000, 1);
        defmt::info!("baudrate=1: sbr={}, brfa={}", sbr, brfa);
        defmt::assert!(sbr >= 1 && sbr <= 0x1FFF);
        defmt::assert!(brfa <= 31);

        // Both minimal
        let (sbr, brfa) = calc_baud(1, 1);
        defmt::info!("clk=1, baud=1: sbr={}, brfa={}", sbr, brfa);
        defmt::assert!(sbr >= 1 && sbr <= 0x1FFF);
        defmt::assert!(brfa <= 31);

        // High baudrate
        let (sbr, brfa) = calc_baud(72_000_000, 10_000_000);
        defmt::info!("high baud: sbr={}, brfa={}", sbr, brfa);
        defmt::assert!(sbr >= 1 && sbr <= 0x1FFF);
        defmt::assert!(brfa <= 31);
    }
}
