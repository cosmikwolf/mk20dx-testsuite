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

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::spi::SpiBus;
use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::dma::{DmaChannels, DmaExt};
use hal::spi::{pack_pushr, Config, Spi, Spi0, SpiExt, MODE_0};
use hal::time::U32Ext;

struct State {
    spi: Spi<Spi0>,
    dma: DmaChannels,
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
        let dma = dp.dma.split(dp.dmamux, &dp.sim);

        super::State { spi, dma }
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

    /// read_dma over the loopback wire: every captured byte must equal the
    /// dummy byte that TX replayed.
    ///
    /// A non-zero dummy is what gives this test teeth. With 0x00 it would pass
    /// even if the RX DMA read the wrong byte lane off POPR, since POPR reads
    /// as 0 when nothing arrived.
    #[test]
    fn test_read_dma_loopback(state: &mut super::State) {
        let dummy = pack_pushr(0xA5, 0, false, false);
        let mut buf = [0u8; 64];

        let xfer = state
            .spi
            .read_dma(&mut buf, &mut state.dma.ch0, &mut state.dma.ch1, &dummy);
        xfer.wait().unwrap();

        state.spi.disable_dma_requests();
        state.spi.flush_fifos();
        state.spi.clear_status();

        for (i, &b) in buf.iter().enumerate() {
            defmt::assert_eq!(b, 0xA5, "byte {} was {}, expected 0xA5", i, b);
        }
        defmt::info!("read_dma loopback: PASSED (64 bytes)");
    }

    /// A longer read crosses many minor loops and a full TX/RX FIFO cycle.
    #[test]
    fn test_read_dma_long(state: &mut super::State) {
        let dummy = pack_pushr(0x5A, 0, false, false);
        let mut buf = [0u8; 256];

        let xfer = state
            .spi
            .read_dma(&mut buf, &mut state.dma.ch0, &mut state.dma.ch1, &dummy);
        xfer.wait().unwrap();

        state.spi.disable_dma_requests();
        state.spi.flush_fifos();
        state.spi.clear_status();

        defmt::assert!(buf.iter().all(|&b| b == 0x5A), "long read mismatched");
        defmt::info!("read_dma long: PASSED (256 bytes)");
    }

    /// read_dma must fill exactly `buf.len()` bytes and not one more.
    #[test]
    fn test_read_dma_no_overrun(state: &mut super::State) {
        let dummy = pack_pushr(0x3C, 0, false, false);
        let mut backing = [0u8; 17];
        backing[16] = 0xEE; // sentinel past the end of the read

        {
            let (head, _tail) = backing.split_at_mut(16);
            let xfer = state
                .spi
                .read_dma(head, &mut state.dma.ch0, &mut state.dma.ch1, &dummy);
            xfer.wait().unwrap();
        }

        state.spi.disable_dma_requests();
        state.spi.flush_fifos();
        state.spi.clear_status();

        defmt::assert!(
            backing[..16].iter().all(|&b| b == 0x3C),
            "first 16 bytes should all be 0x3C"
        );
        defmt::assert_eq!(backing[16], 0xEE, "read_dma wrote past the buffer");
        defmt::info!("read_dma no overrun: PASSED");
    }

    /// The bus must still work for byte transfers after a DMA read.
    #[test]
    fn test_transfer_after_read_dma(state: &mut super::State) {
        let mut verify = [0x7Eu8];
        state.spi.transfer_in_place(&mut verify).unwrap();
        defmt::assert_eq!(verify[0], 0x7E, "SPI bus should work after read_dma");
    }
}
