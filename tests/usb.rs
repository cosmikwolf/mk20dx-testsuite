//! USB self-tests — validates clock setup, endpoint allocation, and initialization.
//!
//! Init/allocation tests only — no USB host connection needed.
//!
//! Note: Since defmt-test shares state across tests (init runs once), and
//! USB requires calling usb_bus() which consumes the PAC peripheral, we
//! initialize the UsbBus once in init and verify clock registers + allocation
//! through the shared state.
//!
//! Priority: LOW
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::usb::UsbBusExt;
use usb_device::bus::UsbBus as UsbBusTrait;
use usb_device::endpoint::EndpointType;
use usb_device::UsbDirection;

struct State {
    usb_bus: hal::usb::UsbBus,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let usb_bus = dp.usb0.usb_bus(&dp.sim);
        super::State { usb_bus }
    }

    /// After usb_bus(), the USB clock gate in SIM SCGC4 should be enabled.
    #[test]
    fn test_clock_gate_enabled(_state: &mut super::State) {
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.scgc4().read().usbotg().is_1(),
            "USBOTG clock gate should be enabled"
        );
    }

    /// SIM SOPT2 should select PLL as USB clock source.
    #[test]
    fn test_clock_48mhz_source(_state: &mut super::State) {
        let sim = unsafe { &*pac::Sim::PTR };
        let sopt2 = sim.sopt2().read();
        defmt::assert!(
            sopt2.pllfllsel().is_pll(),
            "PLLFLLSEL should select PLL"
        );
        defmt::assert!(
            sopt2.usbsrc().is_1(),
            "USBSRC should be set (use PLLFLLSEL clock)"
        );
    }

    /// SIM CLKDIV2 should be configured for 48 MHz USB clock.
    /// For mk20d7 (72 MHz PLL): USBFRAC=1, USBDIV=2 → 72 * 2/3 = 48 MHz.
    #[test]
    fn test_clock_divider(_state: &mut super::State) {
        let sim = unsafe { &*pac::Sim::PTR };
        let clkdiv2 = sim.clkdiv2().read();
        defmt::assert!(
            clkdiv2.usbfrac().bit_is_set(),
            "USBFRAC should be 1"
        );
        defmt::assert_eq!(
            clkdiv2.usbdiv().bits(),
            2,
            "USBDIV should be 2"
        );
    }

    /// Allocating EP0 as a control endpoint should succeed.
    #[test]
    fn test_alloc_ep0_control(state: &mut super::State) {
        let result = state.usb_bus.alloc_ep(
            UsbDirection::In,
            None,
            EndpointType::Control,
            64,
            0,
        );
        defmt::assert!(result.is_ok(), "EP0 control allocation should succeed");
        defmt::info!("EP0 allocated: {:?}", defmt::Debug2Format(&result));
    }

    /// Allocating additional bulk endpoints should succeed.
    #[test]
    fn test_alloc_bulk_endpoints(state: &mut super::State) {
        let ep_in = state.usb_bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0);
        defmt::assert!(ep_in.is_ok(), "Bulk IN allocation should succeed");

        let ep_out = state.usb_bus.alloc_ep(UsbDirection::Out, None, EndpointType::Bulk, 64, 0);
        defmt::assert!(ep_out.is_ok(), "Bulk OUT allocation should succeed");
    }

    /// Calling enable() should configure the USB peripheral registers:
    /// - USB CTL: USBENSOFEN should be set
    /// - ENDPT0: EPHSHK, EPTXEN, EPRXEN should be set (control EP0)
    #[test]
    fn test_enable_configures_registers(state: &mut super::State) {
        state.usb_bus.enable();

        let usb = unsafe { &*pac::Usb0::PTR };

        // Verify USB module is enabled
        defmt::assert!(
            usb.ctl().read().usbensofen().bit_is_set(),
            "USB CTL USBENSOFEN should be set after enable()"
        );

        // Verify EP0 is configured for control transfers
        let endpt0 = usb.endpt(0).read();
        defmt::assert!(
            endpt0.ephshk().bit_is_set(),
            "EP0 EPHSHK (handshake) should be set"
        );
        defmt::assert!(
            endpt0.eptxen().bit_is_set(),
            "EP0 EPTXEN (TX enable) should be set"
        );
        defmt::assert!(
            endpt0.eprxen().bit_is_set(),
            "EP0 EPRXEN (RX enable) should be set"
        );

        defmt::info!("USB enable() configured registers correctly");
    }
}
