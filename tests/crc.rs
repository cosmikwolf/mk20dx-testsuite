//! CRC hardware accelerator self-tests — validates CRC-16 and CRC-32
//! against known test vectors.
//!
//! Priority: HIGH (deterministic, no timing dependencies)
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::crc_module::{Crc, CrcConfig, CrcExt};
use hal::pac;
use hal::prelude::*;

struct State {
    crc: Crc,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let crc = dp.crc.crc_engine(&dp.sim);
        super::State { crc }
    }

    /// CRC-16-CCITT of "123456789" should produce 0x29B1.
    #[test]
    fn test_crc16_known_vector(state: &mut super::State) {
        state.crc.configure(CrcConfig::crc16_ccitt());
        state.crc.feed(b"123456789");
        let result = state.crc.result_u16();
        defmt::info!("CRC-16 of '123456789': 0x{:04X} (expected 0x29B1)", result);
        defmt::assert_eq!(result, 0x29B1, "CRC-16-CCITT mismatch");
    }

    /// CRC-32 of "123456789" should produce 0xCBF43926.
    #[test]
    fn test_crc32_known_vector(state: &mut super::State) {
        state.crc.configure(CrcConfig::crc32());
        state.crc.feed(b"123456789");
        let result = state.crc.result();
        defmt::info!("CRC-32 of '123456789': 0x{:08X} (expected 0xCBF43926)", result);
        defmt::assert_eq!(result, 0xCBF4_3926, "CRC-32 mismatch");
    }

    /// No data fed → result_u16() should return seed value (0xFFFF for CRC-16-CCITT).
    #[test]
    fn test_crc16_empty_is_seed(state: &mut super::State) {
        state.crc.configure(CrcConfig::crc16_ccitt());
        let result = state.crc.result_u16();
        defmt::info!("CRC-16 empty: 0x{:04X} (expected 0xFFFF)", result);
        defmt::assert_eq!(result, 0xFFFF, "Empty CRC-16 should equal seed");
    }

    /// After reset with same seed, re-feeding same data should produce same result.
    #[test]
    fn test_reset_same_result(state: &mut super::State) {
        state.crc.configure(CrcConfig::crc16_ccitt());
        state.crc.feed(b"hello");
        let first = state.crc.result_u16();

        state.crc.reset(0xFFFF);
        state.crc.feed(b"hello");
        let second = state.crc.result_u16();

        defmt::info!("First: 0x{:04X}, Second: 0x{:04X}", first, second);
        defmt::assert_eq!(first, second, "Reset + re-feed should match original");
    }

    /// Incremental feeding should equal bulk feeding.
    #[test]
    fn test_incremental_equals_bulk(state: &mut super::State) {
        // Bulk
        state.crc.configure(CrcConfig::crc16_ccitt());
        state.crc.feed(&[1, 2, 3, 4]);
        let bulk = state.crc.result_u16();

        // Incremental
        state.crc.configure(CrcConfig::crc16_ccitt());
        state.crc.feed(&[1, 2]);
        state.crc.feed(&[3, 4]);
        let incremental = state.crc.result_u16();

        defmt::info!("Bulk: 0x{:04X}, Incremental: 0x{:04X}", bulk, incremental);
        defmt::assert_eq!(bulk, incremental, "Incremental should equal bulk");
    }

    /// Different data should produce different CRC values.
    #[test]
    fn test_different_data_different_crc(state: &mut super::State) {
        state.crc.configure(CrcConfig::crc16_ccitt());
        state.crc.feed(&[0xAA, 0x55]);
        let crc1 = state.crc.result_u16();

        state.crc.configure(CrcConfig::crc16_ccitt());
        state.crc.feed(&[0x55, 0xAA]);
        let crc2 = state.crc.result_u16();

        defmt::info!("CRC [AA,55]: 0x{:04X}, CRC [55,AA]: 0x{:04X}", crc1, crc2);
        defmt::assert!(crc1 != crc2, "Different data should produce different CRC");
    }

    /// CRC-32 of a short byte sequence should produce a non-zero result.
    #[test]
    fn test_crc32_multi_byte(state: &mut super::State) {
        state.crc.configure(CrcConfig::crc32());
        state.crc.feed(&[0x00, 0x01, 0x02, 0x03]);
        let result = state.crc.result();
        defmt::info!("CRC-32 of [00,01,02,03]: 0x{:08X}", result);
        defmt::assert!(result != 0, "CRC-32 should be non-zero for non-empty data");
    }
}
