//! SPI loopback tests — validates DSPI TX/RX via external wire.
//!
//! Uses SPI0: PTC6 (Teensy pin 11, MOSI) → PTC7 (Teensy pin 12, MISO).
//! SCK on PTD1 (Teensy pin 13, ALT2 for SPI0_SCK).
//!
//! REQUIRES: Wire connecting PTC6 (MOSI) → PTC7 (MISO).
//!
//! Priority: MEDIUM
//! Wiring: PTC6 (MOSI) → PTC7 (MISO)

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::spi::SpiBus;
use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::spi::{Config, Spi, Spi0, SpiExt, MODE_0};
use hal::time::U32Ext;

struct State {
    spi: Spi<Spi0>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let pins_c = dp.portc.split(dp.ptc, &dp.sim);
        let pins_d = dp.portd.split(dp.ptd, &dp.sim);

        // SPI0 pin assignment (ALT2):
        // SCK  = PTD1 (Teensy pin 13) - ALT2 for SPI0_SCK
        // MOSI = PTC6 (Teensy pin 11) - ALT2 for SPI0_MOSI
        // MISO = PTC7 (Teensy pin 12) - ALT2 for SPI0_MISO
        let sck = pins_d.pd1.into_alternate::<2>();
        let mosi = pins_c.pc6.into_alternate::<2>();
        let miso = pins_c.pc7.into_alternate::<2>();

        let config = Config::new(1_000_000u32.Hz()).mode(MODE_0);
        let spi = dp.spi0.spi(sck, mosi, miso, config, &clocks, &dp.sim);

        super::State { spi }
    }

    /// Transfer 1 byte (0xA5) in-place — loopback should return the same byte.
    #[test]
    fn test_transfer_in_place(state: &mut super::State) {
        let mut buf = [0xA5u8];
        state.spi.transfer_in_place(&mut buf).unwrap();
        defmt::assert_eq!(buf[0], 0xA5, "In-place transfer should return 0xA5");
    }

    /// Transfer 4 bytes in-place — all should match.
    #[test]
    fn test_transfer_multiple(state: &mut super::State) {
        let mut buf = [0x12u8, 0x34, 0x56, 0x78];
        let expected = buf;
        state.spi.transfer_in_place(&mut buf).unwrap();
        defmt::assert_eq!(buf, expected, "4-byte in-place transfer should match");
    }

    /// Transfer with separate read/write buffers.
    #[test]
    fn test_transfer_separate_bufs(state: &mut super::State) {
        let write_buf = [0xAA, 0xBB, 0xCC, 0xDD];
        let mut read_buf = [0u8; 4];
        state.spi.transfer(&mut read_buf, &write_buf).unwrap();
        defmt::assert_eq!(
            read_buf, write_buf,
            "Separate-buffer transfer should match write data"
        );
    }

    /// Transfer all 256 byte values via loopback.
    #[test]
    fn test_all_byte_values(state: &mut super::State) {
        for value in 0u16..=255 {
            let byte = value as u8;
            let mut buf = [byte];
            state.spi.transfer_in_place(&mut buf).unwrap();
            defmt::assert_eq!(
                buf[0], byte,
                "Byte 0x{:02X} mismatch in loopback",
                byte
            );
        }
        defmt::info!("All 256 byte values passed");
    }

    /// read() should send zeros and receive whatever comes back (zeros in loopback).
    #[test]
    fn test_read_sends_zeros(state: &mut super::State) {
        let mut buf = [0xFFu8; 4];
        state.spi.read(&mut buf).unwrap();
        defmt::assert_eq!(
            buf,
            [0x00; 4],
            "read() with loopback should receive zeros"
        );
    }

    /// write() should complete and leave the bus in a functional state.
    /// Verified by performing a transfer_in_place afterward.
    #[test]
    fn test_write_completes(state: &mut super::State) {
        let buf = [0x11, 0x22, 0x33, 0x44];
        state.spi.write(&buf).unwrap();

        // Verify bus is still functional by doing a loopback transfer
        let mut verify = [0xAB];
        state.spi.transfer_in_place(&mut verify).unwrap();
        defmt::assert_eq!(
            verify[0], 0xAB,
            "SPI bus should be functional after write()"
        );
    }

    /// flush() after write should complete and leave the bus functional.
    #[test]
    fn test_flush(state: &mut super::State) {
        let buf = [0xAA, 0xBB];
        state.spi.write(&buf).unwrap();
        state.spi.flush().unwrap();

        // Verify bus is still functional by doing a loopback transfer
        let mut verify = [0xCD];
        state.spi.transfer_in_place(&mut verify).unwrap();
        defmt::assert_eq!(
            verify[0], 0xCD,
            "SPI bus should be functional after flush()"
        );
    }
}
