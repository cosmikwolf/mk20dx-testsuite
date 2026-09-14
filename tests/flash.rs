//! Flash memory (FTFL) self-tests — validates read operations and
//! safety floor protections. All tests are read-only or verify error
//! paths; no erase/write operations are performed on flash.
//!
//! Priority: HIGH (critical safety verification)
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::eeprom::Eeprom;
use hal::flash::{Flash, FlashError, FlashExt};
use hal::pac;
use hal::prelude::*;

use embedded_storage::nor_flash::ReadNorFlash;

struct State {
    flash: Flash,
    _eeprom: Eeprom,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let (flash, eeprom) = dp.ftfl.flash();
        super::State { flash, _eeprom: eeprom }
    }

    /// Flash capacity should be 262144 bytes (256 KB for MK20DX256).
    #[test]
    fn test_capacity(state: &mut super::State) {
        let cap = state.flash.capacity();
        defmt::info!("Flash capacity: {} bytes", cap);
        defmt::assert_eq!(cap, 262144, "Flash capacity should be 256 KB");
    }

    /// FSEC register SEC field should indicate unsecured (0b10).
    #[test]
    fn test_security_unsecured(state: &mut super::State) {
        let fsec = state.flash.security_status();
        let sec_field = fsec & 0x03;
        defmt::info!("FSEC: 0x{:02X}, SEC field: 0b{:02b}", fsec, sec_field);
        defmt::assert_eq!(sec_field, 0x02, "SEC field should be 0b10 (unsecured)");
    }

    /// Read the first 8 bytes of flash (vector table).
    /// - Bytes 0-3: Initial SP (should be in SRAM range 0x1FFF_xxxx or 0x2000_xxxx)
    /// - Bytes 4-7: Reset vector (should be in flash range 0x0000_xxxx)
    #[test]
    fn test_read_vector_table(state: &mut super::State) {
        let mut buf = [0u8; 8];
        state.flash.read(0, &mut buf).unwrap();

        let sp = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let reset = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);

        defmt::info!("Vector table: SP=0x{:08X} Reset=0x{:08X}", sp, reset);

        // SP should point to end of SRAM (0x1FFF_xxxx or 0x2000_xxxx range)
        defmt::assert!(
            sp >= 0x1FFF_0000 && sp <= 0x2001_0000,
            "SP 0x{:08X} should be in SRAM range",
            sp
        );

        // Reset vector should be in flash range (0x0000_0000 to 0x0004_0000)
        defmt::assert!(
            reset < 0x0004_0000,
            "Reset vector 0x{:08X} should be in flash range",
            reset
        );
    }

    /// Read the 16-byte flash configuration field at 0x400.
    /// FSEC byte (offset 0xC from 0x400 = address 0x40C) must be 0xFE (unsecured).
    #[test]
    fn test_read_flash_config(state: &mut super::State) {
        let mut buf = [0u8; 16];
        state.flash.read(0x400, &mut buf).unwrap();

        defmt::info!(
            "Flash config: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
            buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15]
        );

        // FSEC is at offset 0xC within the flash config field
        defmt::assert_eq!(
            buf[0xC], 0xFE,
            "FSEC byte at 0x40C should be 0xFE (unsecured), got 0x{:02X}",
            buf[0xC]
        );
    }

    /// Attempting to erase sector 0 (below safety floor) should return Protected.
    #[test]
    fn test_safety_floor_blocks_erase(state: &mut super::State) {
        let result = state.flash.erase_sector(0);
        defmt::info!("Erase sector 0 result: {:?}", result);
        defmt::assert_eq!(
            result,
            Err(FlashError::Protected),
            "Erasing below safety floor should return Protected"
        );
    }

    /// Attempting to write at address 0 (below safety floor) should return Protected.
    #[test]
    fn test_safety_floor_blocks_write(state: &mut super::State) {
        let result = state.flash.program_longword(0, &[0xFF; 4]);
        defmt::info!("Write at 0 result: {:?}", result);
        defmt::assert_eq!(
            result,
            Err(FlashError::Protected),
            "Writing below safety floor should return Protected"
        );
    }
}
