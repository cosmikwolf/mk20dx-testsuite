//! USB packet buffers: two per allocated endpoint direction, from the BDT
//! tail and then from the application (`UsbBus::with_buffers`).
//!
//! No USB host needed: allocation, `enable()`, `reset()` and one `write()`.
//! Run it twice, once per BDT layout:
//!
//! ```text
//! cargo test --test usb_bufs                            # 16 endpoints, no tail
//! cargo test --test usb_bufs --features usb-endpoints-4 # 6 buffers in the tail
//! ```
//!
//! The buffers given are exactly enough for EP0 (4), one bulk OUT (2) and one
//! bulk IN (2), so a third bulk endpoint must fail.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::usb::{UsbBuffer, UsbBus};
use usb_device::bus::UsbBus as UsbBusTrait;
use usb_device::endpoint::{EndpointAddress, EndpointType};
use usb_device::{UsbDirection, UsbError};

/// EP0 (4) + one OUT (2) + one IN (2), less what the BDT tail holds.
const GIVEN: usize = 8 - UsbBus::TAIL_BUFFERS;

struct State {
    usb_bus: UsbBus,
    /// Address range of the buffers given to `with_buffers`.
    given: (u32, u32),
}

/// The BDT base from BDTPAGE1-3.
fn bdt_base() -> u32 {
    let usb = unsafe { &*pac::Usb0::PTR };
    let p1 = usb.bdtpage1().read().bits() as u32;
    let p2 = usb.bdtpage2().read().bits() as u32;
    let p3 = usb.bdtpage3().read().bits() as u32;
    (p3 << 24) | (p2 << 16) | ((p1 & 0xFE) << 8)
}

/// The buffer address in the BDT entry of (ep, tx, odd).
fn bd_addr(ep: usize, tx: bool, odd: bool) -> u32 {
    let idx = ep * 4 + if tx { 2 } else { 0 } + if odd { 1 } else { 0 };
    unsafe { core::ptr::read_volatile((bdt_base() + idx as u32 * 8 + 4) as *const u32) }
}

/// Inside the BDT tail (past the table, inside the 512-byte block) or inside
/// the buffers given.
fn in_pool(addr: u32, given: (u32, u32)) -> bool {
    let base = bdt_base();
    let table = 512 - UsbBus::TAIL_BUFFERS as u32 * 64;
    let tail = addr >= base + table && addr + 64 <= base + 512;
    let app = addr >= given.0 && addr + 64 <= given.1;
    addr % 4 == 0 && (tail || app)
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let bufs: &'static mut [UsbBuffer; GIVEN] =
            cortex_m::singleton!(: [UsbBuffer; GIVEN] = [UsbBuffer::new(); GIVEN]).unwrap();
        let start = bufs.as_ptr() as u32;
        let given = (start, start + (GIVEN * 64) as u32);
        defmt::info!("TAIL_BUFFERS {}, given {} at {:#x}", UsbBus::TAIL_BUFFERS, GIVEN, start);
        let usb_bus = UsbBus::with_buffers(dp.usb0, &dp.sim, &clocks, bufs);
        super::State { usb_bus, given }
    }

    /// EP0 both ways, one bulk OUT (EP1) and one bulk IN (EP2) fit.
    #[test]
    fn test_alloc_fits(state: &mut super::State) {
        let bus = &mut state.usb_bus;
        let ep0_out = EndpointAddress::from_parts(0, UsbDirection::Out);
        let ep0_in = EndpointAddress::from_parts(0, UsbDirection::In);
        defmt::assert!(bus.alloc_ep(UsbDirection::Out, Some(ep0_out), EndpointType::Control, 64, 0).is_ok());
        defmt::assert!(bus.alloc_ep(UsbDirection::In, Some(ep0_in), EndpointType::Control, 64, 0).is_ok());
        let out = bus.alloc_ep(UsbDirection::Out, None, EndpointType::Bulk, 64, 0);
        let inp = bus.alloc_ep(UsbDirection::In, None, EndpointType::Bulk, 64, 0);
        defmt::assert!(matches!(out.map(|a| a.index()), Ok(1)), "bulk OUT is EP1");
        defmt::assert!(matches!(inp.map(|a| a.index()), Ok(2)), "bulk IN is EP2");
    }

    /// A direction that already has buffers takes none again.
    #[test]
    fn test_realloc_takes_no_buffer(state: &mut super::State) {
        let ep0_out = EndpointAddress::from_parts(0, UsbDirection::Out);
        let r = state.usb_bus.alloc_ep(UsbDirection::Out, Some(ep0_out), EndpointType::Control, 64, 0);
        defmt::assert!(r.is_ok());
    }

    /// The buffers are gone: a third bulk endpoint fails, and cleanly.
    #[test]
    fn test_alloc_overflow(state: &mut super::State) {
        let r = state.usb_bus.alloc_ep(UsbDirection::Out, None, EndpointType::Bulk, 64, 0);
        defmt::assert!(matches!(r, Err(UsbError::EndpointMemoryOverflow)));
    }

    /// After enable() and a bus reset, every armed RX descriptor points into
    /// the pool, each at its own buffer, and ENDPTn enables only the
    /// allocated directions.
    #[test]
    fn test_rx_descriptors_and_endpt(state: &mut super::State) {
        state.usb_bus.enable();
        state.usb_bus.reset();
        let given = state.given;

        let addrs = [
            bd_addr(0, false, false),
            bd_addr(0, false, true),
            bd_addr(1, false, false),
            bd_addr(1, false, true),
        ];
        for (i, &a) in addrs.iter().enumerate() {
            defmt::assert!(in_pool(a, given), "RX BD {} addr {:#x} outside the pool", i, a);
            for &b in &addrs[i + 1..] {
                defmt::assert_ne!(a, b, "two RX BDs share a buffer");
            }
        }

        let usb = unsafe { &*pac::Usb0::PTR };
        let e1 = usb.endpt(1).read();
        let e2 = usb.endpt(2).read();
        let e3 = usb.endpt(3).read();
        defmt::assert!(e1.eprxen().bit_is_set() && e1.eptxen().bit_is_clear(), "EP1 is OUT only");
        defmt::assert!(e2.eptxen().bit_is_set() && e2.eprxen().bit_is_clear(), "EP2 is IN only");
        defmt::assert_eq!(e3.bits(), 0, "EP3 was never allocated");
    }

    /// write() on the IN endpoint copies into its own buffer and points the
    /// TX descriptor at it.
    #[test]
    fn test_write_uses_tx_buffer(state: &mut super::State) {
        let ep2_in = EndpointAddress::from_parts(2, UsbDirection::In);
        let data = [0xA5u8, 0x5A, 0x01, 0x02, 0x03];
        defmt::assert!(matches!(state.usb_bus.write(ep2_in, &data), Ok(5)), "write took 5 bytes");

        let a = bd_addr(2, true, false);
        defmt::assert!(in_pool(a, state.given), "TX BD addr {:#x} outside the pool", a);
        for ep_rx in [bd_addr(0, false, false), bd_addr(0, false, true), bd_addr(1, false, false), bd_addr(1, false, true)] {
            defmt::assert_ne!(a, ep_rx, "the TX buffer is an RX buffer");
        }
        let copied = unsafe { core::slice::from_raw_parts(a as *const u8, data.len()) };
        defmt::assert_eq!(copied, &data[..]);

        // No buffers for EP1 IN: it was never allocated.
        let ep1_in = EndpointAddress::from_parts(1, UsbDirection::In);
        defmt::assert!(matches!(state.usb_bus.write(ep1_in, &data), Err(UsbError::InvalidEndpoint)));
    }
}
