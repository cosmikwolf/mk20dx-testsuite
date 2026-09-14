//! EEPROM / FlexMemory self-tests — validates FlexRAM interface.
//!
//! The EEPROM partition state varies between Teensy boards. Tests handle
//! both partitioned (EEE enabled) and unpartitioned (RAM mode) states
//! gracefully using conditional assertions.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::eeprom::{Eeprom, EepromError};
use hal::flash::{Flash, FlashExt};
use hal::pac;
use hal::prelude::*;

struct State {
    _flash: Flash,
    eeprom: Eeprom,
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
        super::State { _flash: flash, eeprom }
    }

    /// FlexRAM capacity should be 2048 bytes.
    #[test]
    fn test_capacity(state: &mut super::State) {
        let cap = state.eeprom.capacity();
        defmt::info!("EEPROM capacity: {} bytes", cap);
        defmt::assert_eq!(cap, 2048, "EEPROM capacity should be 2048 bytes");
    }

    /// At least one of is_eee_enabled() or is_ram_mode() should be true.
    /// FlexRAM is always in one mode or the other.
    #[test]
    fn test_eee_or_ram_mode(state: &mut super::State) {
        let eee = state.eeprom.is_eee_enabled();
        let ram = state.eeprom.is_ram_mode();
        defmt::info!("EEE enabled: {}, RAM mode: {}", eee, ram);
        defmt::assert!(
            eee || ram,
            "FlexRAM should be in either EEE or RAM mode"
        );
    }

    /// If EEE is enabled, reading offset 0 should not panic.
    /// If not, skip the read test.
    #[test]
    fn test_read_no_crash(state: &mut super::State) {
        if state.eeprom.is_eee_enabled() {
            let val = state.eeprom.read(0);
            defmt::info!("EEPROM[0] = 0x{:02X}", val);
        } else {
            defmt::info!("EEE not enabled, skipping read test");
        }
    }

    /// Reading beyond capacity should return OutOfBounds.
    #[test]
    fn test_read_slice_bounds(state: &mut super::State) {
        let mut buf = [0u8; 1];
        let result = state.eeprom.read_slice(2048, &mut buf);
        defmt::info!("Read at 2048 result: {:?}", result);
        defmt::assert_eq!(
            result,
            Err(EepromError::OutOfBounds),
            "Reading beyond capacity should return OutOfBounds"
        );
    }

    /// If EEE is enabled, write/read roundtrip at a high offset.
    /// If not partitioned, write should return NotPartitioned.
    #[test]
    fn test_write_or_not_partitioned(state: &mut super::State) {
        if state.eeprom.is_eee_enabled() {
            // Read original value, write test pattern, verify, restore
            let offset = 2000u16;
            let original = state.eeprom.read(offset);
            let test_val = original.wrapping_add(1);

            state.eeprom.write(offset, test_val).unwrap();
            let readback = state.eeprom.read(offset);
            defmt::info!(
                "Write/read roundtrip: wrote 0x{:02X}, read 0x{:02X}",
                test_val, readback
            );
            defmt::assert_eq!(readback, test_val, "Write/read roundtrip failed");

            // Restore original value
            state.eeprom.write(offset, original).unwrap();
        } else {
            let result = state.eeprom.write(0, 0x42);
            defmt::info!("Write without partition: {:?}", result);
            defmt::assert_eq!(
                result,
                Err(EepromError::NotPartitioned),
                "Write without EEE should return NotPartitioned"
            );
        }
    }
}
