//! USB with `poll()` in the USB0 interrupt handler, SOF interrupt on.
//!
//! Enumerates as a USB-MIDI device, checks the SOF tick rate and the
//! interrupt rate (no storm), then waits for one USB-MIDI packet from the
//! host (and then three more). Send them with `host/usb_midi_send.py`, which waits for the port:
//!
//! ```text
//! uv run host/usb_midi_send.py &
//! cargo test --features usb-host --test usb_irq
//! ```
//!
//! Priority: MEDIUM
//! Wiring: Teensy USB port to the host. Gated on the `usb-host` feature.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use cortex_m::peripheral::NVIC;
use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use embedded_hal::delay::DelayNs;
use hal::delay::Delay;
use hal::interrupt;
use hal::pac;
use hal::prelude::*;
use hal::usb::{UsbBus, UsbBusExt};
use mk20dx_hal as hal;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_midi::UsbMidiClass;

/// Written by `init` before USB0 is unmasked, then used only by `USB0`.
static mut USB_DEV: Option<UsbDevice<'static, UsbBus>> = None;
static mut MIDI: Option<UsbMidiClass<'static, UsbBus>> = None;

/// Handler runs (every entry, for whatever flag).
static IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
/// Handler runs where `take_sof()` returned true.
static SOF_COUNT: AtomicU32 = AtomicU32::new(0);
/// `UsbDeviceState` as a u8 (Default=0, Addressed=1, Configured=2, Suspend=3).
static STATE: AtomicU8 = AtomicU8::new(0);
/// First USB-MIDI Note On packet received, as a little-endian u32; 0 = none yet.
static MIDI_PACKET: AtomicU32 = AtomicU32::new(0);
/// Note On packets received, and the note number of the last one.
static NOTE_COUNT: AtomicU32 = AtomicU32::new(0);
static LAST_NOTE: AtomicU8 = AtomicU8::new(0);
/// Diagnostic: every successful class read, and the first 8 packets raw.
static READS: AtomicU32 = AtomicU32::new(0);
static RAW: [AtomicU32; 8] = [const { AtomicU32::new(0) }; 8];
/// The frame number at the most recent SOF.
static FRAME: AtomicU32 = AtomicU32::new(0);
/// The first state changes, as (handler run << 8) | new state. For a
/// failed enumeration, the log shows where it stopped.
static TRACE: [AtomicU32; 32] = [const { AtomicU32::new(0) }; 32];
static TRACE_LEN: AtomicU32 = AtomicU32::new(0);

#[cortex_m_rt::interrupt]
fn USB0() {
    IRQ_COUNT.fetch_add(1, Ordering::Relaxed);

    // SAFETY: init wrote both statics before it unmasked USB0, and this
    // handler is the only code that uses them afterwards.
    let (usb_dev, midi) = unsafe {
        (
            (*(&raw mut USB_DEV)).as_mut().unwrap(),
            (*(&raw mut MIDI)).as_mut().unwrap(),
        )
    };

    usb_dev.poll(&mut [&mut *midi]);

    let mut buf = [0u8; 64];
    if let Ok(n) = midi.read(&mut buf) {
        let r = READS.fetch_add(1, Ordering::Relaxed) as usize;
        if r < RAW.len() && n >= 4 {
            RAW[r].store(u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]), Ordering::Relaxed);
        }
        // macOS sends an empty SysEx (06 f0 f7 00) when it opens the port.
        // Keep the first Note On packet (code index 0x9) only.
        for pkt in buf[..n].chunks_exact(4) {
            if pkt[0] & 0x0F == 0x09 {
                if MIDI_PACKET.load(Ordering::Relaxed) == 0 {
                    let p = u32::from_le_bytes([pkt[0], pkt[1], pkt[2], pkt[3]]);
                    MIDI_PACKET.store(p, Ordering::Relaxed);
                }
                NOTE_COUNT.fetch_add(1, Ordering::Relaxed);
                LAST_NOTE.store(pkt[2], Ordering::Relaxed);
            }
        }
    }

    if usb_dev.bus().take_sof() {
        SOF_COUNT.fetch_add(1, Ordering::Relaxed);
        FRAME.store(u32::from(usb_dev.bus().frame_number()), Ordering::Relaxed);
    }

    let st = usb_dev.state() as u8;
    if STATE.swap(st, Ordering::Relaxed) != st {
        let i = TRACE_LEN.fetch_add(1, Ordering::Relaxed) as usize;
        if i < TRACE.len() {
            let run = IRQ_COUNT.load(Ordering::Relaxed);
            TRACE[i].store((run << 8) | u32::from(st), Ordering::Relaxed);
        }
    }
}

const CONFIGURED: u8 = UsbDeviceState::Configured as u8;

struct State {
    delay: Delay,
}

/// Wait up to `timeout_ms` for `cond`. Return the time it took, or None.
fn wait_for(delay: &mut Delay, timeout_ms: u32, mut cond: impl FnMut() -> bool) -> Option<u32> {
    for ms in 0..timeout_ms {
        if cond() {
            return Some(ms);
        }
        if option_env!("USB_IRQ_PEND").is_some() {
            // Diagnostic: run the handler about every 20 us, as a polling
            // loop would, to compare with interrupt-only operation.
            for _ in 0..50 {
                NVIC::pend(pac::Interrupt::USB0);
                delay.delay_us(20);
            }
        } else {
            delay.delay_ms(1);
        }
    }
    None
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        static mut USB_ALLOC: Option<UsbBusAllocator<UsbBus>> = None;

        let dp = pac::Peripherals::take().unwrap();
        let cp = cortex_m::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let delay = Delay::new(cp.SYST, &clocks);

        let usb_bus = dp.usb0.usb_bus(&dp.sim, &clocks);
        usb_bus.enable_sof_interrupt(true);

        // SAFETY: init runs once, before USB0 is unmasked.
        let alloc: &'static UsbBusAllocator<UsbBus> =
            unsafe { (*(&raw mut USB_ALLOC)).insert(UsbBusAllocator::new(usb_bus)) };

        let midi = UsbMidiClass::new(alloc, 1, 1).unwrap();
        let usb_dev = UsbDeviceBuilder::new(alloc, UsbVidPid(0x16c0, 0x0485))
            .strings(&[StringDescriptors::default()
                .manufacturer("mk20dx")
                .product("mk20dx-usb-irq")
                .serial_number("mk20dx-usb-irq")])
            .unwrap()
            // 64, as flxs1 uses. With the usb-device default of 8 the host
            // stops after SET_ADDRESS (open issue, mk20dx-hal STATUS.md).
            .max_packet_size_0(64)
            .unwrap()
            .build();

        // SAFETY: USB0 is still masked, so the handler cannot run yet.
        unsafe {
            *(&raw mut MIDI) = Some(midi);
            *(&raw mut USB_DEV) = Some(usb_dev);
            NVIC::unmask(pac::Interrupt::USB0);
        }

        super::State { delay }
    }

    /// The host enumerates the device with poll() only in the handler.
    #[test]
    fn test_enumerates(state: &mut super::State) {
        let t = wait_for(&mut state.delay, 10_000, || {
            STATE.load(Ordering::Relaxed) == CONFIGURED
        });
        defmt::info!(
            "state={} irqs={} after {:?} ms",
            STATE.load(Ordering::Relaxed),
            IRQ_COUNT.load(Ordering::Relaxed),
            t
        );
        let n = (TRACE_LEN.load(Ordering::Relaxed) as usize).min(TRACE.len());
        for e in TRACE[..n].iter() {
            let e = e.load(Ordering::Relaxed);
            defmt::info!("  run {}: state {}", e >> 8, e & 0xFF);
        }
        defmt::assert!(t.is_some(), "device did not reach Configured in 10 s");
    }

    /// With SOFTOKEN on, the handler sees one SOF each 1 ms, and the frame
    /// number moves with it.
    #[test]
    fn test_sof_tick(state: &mut super::State) {
        let sof0 = SOF_COUNT.load(Ordering::Relaxed);
        let frame0 = FRAME.load(Ordering::Relaxed);
        state.delay.delay_ms(500);
        let sofs = SOF_COUNT.load(Ordering::Relaxed) - sof0;
        let frames = (FRAME.load(Ordering::Relaxed).wrapping_sub(frame0)) & 0x7FF;
        defmt::info!("500 ms: {} SOF runs, frame number +{}", sofs, frames);
        defmt::assert!((450..=550).contains(&sofs), "SOF runs not about 1 per ms");
        defmt::assert!((450..=550).contains(&frames), "frame number not about 1 per ms");
    }

    /// No interrupt storm: with the bus idle except for SOF, the handler runs
    /// about once per frame. A flag that poll() did not clear (or RESUME on
    /// bus activity) would give a far higher count.
    #[test]
    fn test_no_irq_storm(state: &mut super::State) {
        let irq0 = IRQ_COUNT.load(Ordering::Relaxed);
        let sof0 = SOF_COUNT.load(Ordering::Relaxed);
        state.delay.delay_ms(500);
        let irqs = IRQ_COUNT.load(Ordering::Relaxed) - irq0;
        let sofs = SOF_COUNT.load(Ordering::Relaxed) - sof0;
        let inten = unsafe { (*pac::Usb0::PTR).inten().read().bits() };
        defmt::info!("500 ms: {} handler runs, {} SOF, INTEN={=u8:#04x}", irqs, sofs, inten);
        defmt::assert!(irqs < 2 * sofs + 50, "handler runs far more than once per frame");
        defmt::assert!(inten & (1 << 5) == 0, "RESUMEEN on outside suspend");
    }

    /// Read one USB-MIDI packet from the host, received in the handler.
    #[test]
    fn test_reads_midi_packet(state: &mut super::State) {
        defmt::info!("waiting 30 s for a MIDI packet: run host/usb_midi_send.py");
        let t = wait_for(&mut state.delay, 30_000, || MIDI_PACKET.load(Ordering::Relaxed) != 0);
        let p = MIDI_PACKET.load(Ordering::Relaxed).to_le_bytes();
        defmt::info!("packet {=[u8]:#04x} after {:?} ms", p, t);
        let reads = READS.load(Ordering::Relaxed);
        defmt::info!("{} class reads", reads);
        for r in RAW[..(reads as usize).min(RAW.len())].iter() {
            defmt::info!("  raw {=[u8]:#04x}", r.load(Ordering::Relaxed).to_le_bytes());
        }
        // The RX BDs of EP1..EP2 (index 4..12): desc words.
        let usb = unsafe { &*pac::Usb0::PTR };
        let bdt = (u32::from(usb.bdtpage1().read().bits() & 0xFE) << 8)
            | (u32::from(usb.bdtpage2().read().bits()) << 16)
            | (u32::from(usb.bdtpage3().read().bits()) << 24);
        for i in 4..12u32 {
            let d = unsafe { core::ptr::read_volatile((bdt + i * 8) as *const u32) };
            defmt::info!("  bd[{}] = {=u32:#010x}", i, d);
        }
        for e in 0..4 {
            defmt::info!("  ENDPT{} = {=u8:#04x}", e, usb.endpt(e).read().bits());
        }
        defmt::assert!(t.is_some(), "no MIDI packet in 30 s");
        // The script sends Note On, channel 1, velocity 100, cable 0, for
        // notes 60..=63, one USB packet each.
        defmt::assert_eq!(p, [0x09, 0x90, 60, 100]);

        // All four, not every second one: the RX banks expect DATA0 on EVEN
        // and DATA1 on ODD. A wrong PID in a bank drops the packet.
        wait_for(&mut state.delay, 2_000, || NOTE_COUNT.load(Ordering::Relaxed) >= 4);
        let count = NOTE_COUNT.load(Ordering::Relaxed);
        let last = LAST_NOTE.load(Ordering::Relaxed);
        defmt::info!("{} Note On packets, last note {}", count, last);
        defmt::assert_eq!(count, 4);
        defmt::assert_eq!(last, 63);
    }
}
