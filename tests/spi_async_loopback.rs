//! Async SPI loopback tests — validates the interrupt-driven transfer path.
//!
//! Uses SPI0: PTC6 (Teensy pin 11, MOSI) → PTC7 (Teensy pin 12, MISO).
//! SCK on PTD1 (Teensy pin 13, ALT2 for SPI0_SCK).
//!
//! REQUIRES: Wire connecting PTC6 (MOSI) → PTC7 (MISO).
//!
//! What these cover: the data correctness of `transfer_byte_async`, which
//! depends on the SPI0 ISR clearing TCF and TCF_RE, on no stale TCF being left
//! by an earlier transfer, and on POPR being read before RFDF is cleared.
//!
//! What these do NOT cover: the sleep-until-woken property. `block_on` below
//! re-polls on a timer rather than parking on `wfi`, so a genuinely lost wakeup
//! would be papered over by the next poll. That is a deliberate trade — parking
//! on `wfi` turns a lost wakeup into a hung test run rather than a failure.
//!
//! Priority: MEDIUM
//! Wiring: PTC6 (MOSI) → PTC7 (MISO)

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

use mk20dx_hal as hal;
use hal::interrupt;
use hal::pac;
use hal::prelude::*;
use hal::spi::{Config, Spi, Spi0, SpiExt, MODE_0};
use hal::time::U32Ext;

#[cortex_m_rt::interrupt]
fn SPI0() {
    hal::spi::on_spi0_interrupt();
}

// ----- Minimal executor -----

static VTABLE: RawWakerVTable = RawWakerVTable::new(vt_clone, vt_noop, vt_noop, vt_noop);

fn raw_waker() -> RawWaker {
    RawWaker::new(core::ptr::null(), &VTABLE)
}
unsafe fn vt_clone(_: *const ()) -> RawWaker {
    raw_waker()
}
unsafe fn vt_noop(_: *const ()) {}

/// Poll a future to completion, bounded so a stall fails the test instead of
/// hanging the run.
fn block_on<F: Future>(future: F) -> F::Output {
    const MAX_POLLS: u32 = 100_000;

    let waker = unsafe { Waker::from_raw(raw_waker()) };
    let mut cx = Context::from_waker(&waker);
    let mut future = future;
    // SAFETY: `future` is owned by this frame and never moved after this point.
    let mut future = unsafe { Pin::new_unchecked(&mut future) };

    for _ in 0..MAX_POLLS {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
        cortex_m::asm::delay(100);
    }
    defmt::panic!("future did not complete within {} polls", MAX_POLLS);
}

/// Blocking transfer. Polls RFDF and never touches TCF.
fn blocking_xfer(spi: &mut Spi<Spi0>, buf: &mut [u8]) {
    embedded_hal::spi::SpiBus::transfer_in_place(spi, buf).unwrap();
}

/// Async transfer, driven to completion by `block_on`. Completes on TCF.
fn async_xfer(spi: &mut Spi<Spi0>, buf: &mut [u8]) {
    block_on(embedded_hal_async::spi::SpiBus::transfer_in_place(spi, buf)).unwrap();
}

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

        let sck = pins_d.pd1.into_alternate::<2>();
        let mosi = pins_c.pc6.into_alternate::<2>();
        let miso = pins_c.pc7.into_alternate::<2>();

        let config = Config::new(1_000_000u32.Hz()).mode(MODE_0);
        let spi = dp.spi0.spi(sck, mosi, miso, config, &clocks, &dp.sim);

        // The async path completes on the TCF interrupt, so SPI0 must be
        // unmasked for any of these tests to finish.
        unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::SPI0) };

        super::State { spi }
    }

    /// A single async transfer must echo its byte back over the wire.
    #[test]
    fn test_async_transfer_echoes(state: &mut super::State) {
        let mut buf = [0xA5u8];
        async_xfer(&mut state.spi, &mut buf);
        defmt::assert_eq!(buf[0], 0xA5, "async transfer should echo 0xA5");
    }

    /// An async transfer straight after a blocking one must still return the
    /// byte from its own frame.
    ///
    /// The blocking path polls RFDF and never clears TCF, so it leaves TCF set.
    /// If the async path then enables TCF_RE without clearing TCF first, the ISR
    /// fires immediately for the *previous* frame, the future completes early,
    /// and POPR is read before the new byte has arrived.
    #[test]
    fn test_async_after_blocking(state: &mut super::State) {
        let mut sync_buf = [0x11u8];
        blocking_xfer(&mut state.spi, &mut sync_buf);
        defmt::assert_eq!(sync_buf[0], 0x11, "blocking transfer should echo 0x11");

        let mut async_buf = [0xC3u8];
        async_xfer(&mut state.spi, &mut async_buf);
        defmt::assert_eq!(
            async_buf[0], 0xC3,
            "async transfer after a blocking one returned a stale byte"
        );
    }

    /// Alternating blocking and async transfers must not desync.
    #[test]
    fn test_async_blocking_alternating(state: &mut super::State) {
        for i in 0..8u8 {
            let sync_val = 0x40 | i;
            let mut sync_buf = [sync_val];
            blocking_xfer(&mut state.spi, &mut sync_buf);
            defmt::assert_eq!(sync_buf[0], sync_val, "blocking transfer {} desynced", i);

            let async_val = 0x80 | i;
            let mut async_buf = [async_val];
            async_xfer(&mut state.spi, &mut async_buf);
            defmt::assert_eq!(async_buf[0], async_val, "async transfer {} desynced", i);
        }
        defmt::info!("alternating blocking/async: PASSED (8 pairs)");
    }

    /// Consecutive async transfers must each return their own byte.
    ///
    /// Clearing RFDF before reading POPR leaves the flag re-asserted against a
    /// non-empty FIFO, so the next transfer reads the previous byte.
    #[test]
    fn test_async_consecutive_distinct(state: &mut super::State) {
        let values = [0x01u8, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFF, 0x00];
        for &value in values.iter() {
            let mut buf = [value];
            async_xfer(&mut state.spi, &mut buf);
            defmt::assert_eq!(buf[0], value, "async transfer returned a stale byte");
        }
        defmt::info!("consecutive async: PASSED ({} bytes)", values.len());
    }

    /// Every byte value survives an async round trip.
    #[test]
    fn test_async_all_byte_values(state: &mut super::State) {
        for value in 0..=255u8 {
            let mut buf = [value];
            async_xfer(&mut state.spi, &mut buf);
            defmt::assert_eq!(buf[0], value, "async transfer corrupted a byte");
        }
        defmt::info!("all 256 byte values passed async");
    }

    /// A multi-byte async transfer must echo the whole buffer.
    #[test]
    fn test_async_multibyte(state: &mut super::State) {
        let mut buf = [0xDEu8, 0xAD, 0xBE, 0xEF];
        async_xfer(&mut state.spi, &mut buf);
        defmt::assert_eq!(buf, [0xDE, 0xAD, 0xBE, 0xEF], "multibyte async mismatch");
    }

    /// The bus must still work for blocking transfers after async use.
    #[test]
    fn test_blocking_after_async(state: &mut super::State) {
        let mut buf = [0x7Eu8];
        blocking_xfer(&mut state.spi, &mut buf);
        defmt::assert_eq!(buf[0], 0x7E, "blocking transfer broken after async use");
    }
}
