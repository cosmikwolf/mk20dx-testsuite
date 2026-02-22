//! I2C self-tests — validates NACK detection on empty bus.
//!
//! REQUIRES: 4.7k pull-up resistors on PTB0 (SCL) and PTB1 (SDA) to 3.3V.
//! Without pull-ups, the bus floats and NACK detection may hang.
//!
//! Priority: MEDIUM
//! Wiring: Pull-up resistors on SCL/SDA to 3.3V

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::i2c::I2c as I2cTrait;
use mk20dx_hal as hal;
use hal::i2c::{Config, I2c, I2c0, I2cExt};
use hal::pac;
use hal::prelude::*;
use hal::time::U32Ext;

struct State {
    i2c: I2c<I2c0>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
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
}
