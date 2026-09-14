//! UART loopback tests — validates serial TX/RX via external wire.
//!
//! Uses UART2: PTD3 (Teensy pin 8, TX) → PTD2 (Teensy pin 7, RX).
//!
//! REQUIRES: Wire connecting PTD3 → PTD2.
//!
//! Priority: MEDIUM
//! Wiring: PTD3 (TX) → PTD2 (RX)

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal_nb::serial::{Read as NbRead, Write as NbWrite};
use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::time::U32Ext;
use hal::uart::{Config, Serial, Uart2, UartExt};

struct State {
    serial: Serial<Uart2>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let pins_d = dp.portd.split(dp.ptd, &dp.sim);

        // UART2: PTD3 (TX, ALT3), PTD2 (RX, ALT3)
        let tx = pins_d.pd3.into_alternate::<3>();
        let rx = pins_d.pd2.into_alternate::<3>();

        let config = Config::new(115_200u32.Hz());
        let serial = dp.uart2.serial(tx, rx, config, &clocks, &dp.sim);

        super::State { serial }
    }

    /// Send 0xA5, receive it back via loopback wire.
    #[test]
    fn test_single_byte(state: &mut super::State) {
        nb::block!(state.serial.write(0xA5)).unwrap();
        nb::block!(state.serial.flush()).unwrap();

        let received = nb::block!(state.serial.read()).unwrap();
        defmt::assert_eq!(received, 0xA5, "Loopback byte should match");
    }

    /// Send [0x01, 0x02, 0x03, 0x04], receive in order.
    /// UART2 has a 1-entry RX buffer, so we must read each byte before
    /// sending the next to avoid RX overrun.
    #[test]
    fn test_multiple_bytes(state: &mut super::State) {
        let data = [0x01u8, 0x02, 0x03, 0x04];
        for &byte in &data {
            nb::block!(state.serial.write(byte)).unwrap();
            nb::block!(state.serial.flush()).unwrap();
            let received = nb::block!(state.serial.read()).unwrap();
            defmt::assert_eq!(received, byte, "Byte mismatch in sequence");
        }
    }

    /// Send all 256 byte values (0x00..=0xFF) and verify each.
    #[test]
    fn test_all_byte_values(state: &mut super::State) {
        for value in 0u16..=255 {
            let byte = value as u8;
            nb::block!(state.serial.write(byte)).unwrap();
            nb::block!(state.serial.flush()).unwrap();

            let received = nb::block!(state.serial.read()).unwrap();
            defmt::assert_eq!(
                received, byte,
                "Byte value 0x{:02X} mismatch",
                byte
            );
        }
        defmt::info!("All 256 byte values passed");
    }

    /// Verify raw UART register loopback works independently of the HAL
    /// serial driver. Writes and reads a byte via direct register access.
    /// (Cannot use split() as it would consume the shared Serial state.)
    #[test]
    fn test_raw_register_loopback(_state: &mut super::State) {
        let uart2 = unsafe { &*pac::Uart2::PTR };

        // Write a byte via data register
        while uart2.s1().read().tdre().bit_is_clear() {}
        uart2.d().write(|w| unsafe { w.bits(0x42) });

        // Wait for transmission complete
        while uart2.s1().read().tc().bit_is_clear() {}

        // Read back via loopback
        while uart2.s1().read().rdrf().bit_is_clear() {}
        let received = uart2.d().read().bits();
        defmt::assert_eq!(received, 0x42, "Raw register loopback should match");
    }

    /// Reading RX with nothing sent should return WouldBlock.
    /// First drains any leftover bytes from previous tests.
    #[test]
    fn test_rx_empty_returns_wouldblock(state: &mut super::State) {
        // Drain any leftover bytes from previous tests
        loop {
            match state.serial.read() {
                Err(nb::Error::WouldBlock) => break,
                _ => continue,
            }
        }

        // Now the RX FIFO should be empty
        let result = state.serial.read();
        match result {
            Err(nb::Error::WouldBlock) => {
                defmt::info!("Got expected WouldBlock on empty RX");
            }
            _ => {
                defmt::panic!("Expected WouldBlock, got {:?}", defmt::Debug2Format(&result));
            }
        }
    }
}
