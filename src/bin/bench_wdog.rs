#![no_std]
#![no_main]

//! Like `bench`, but never touches the WDOG - it stays enabled and the main loop
//! refreshes it.
//!
//! This was written to test whether probe-rs halting the core causes a WDOG
//! reset loop. It cannot test that, and its original claim that it is "stable
//! standalone" was wrong: **standalone, this binary reset-loops at ~20 kHz**
//! (measured 2026-09-10, 78% of the time in reset, `RCM_SRS0 = 0x20`).
//!
//! The reason is K20 RM §23.3.2: the WDOG must be *unlocked* within 256 bus
//! clock cycles (WCT) of any system reset, "failing which the WDOG issues a
//! reset to the system". Refreshing later is irrelevant; the unlock never
//! happens here. Under a debugger the rule is suspended ("entry into Debug mode
//! within WCT after reset ... there is no need to unlock"), which is why it looked
//! fine over RTT. See `bench_wct.rs` for the controlled reproduction and the
//! flxs1 `pre_init` for the fix.
//!
//! Kept as the "never unlocks" control. The `.uninit` DIAG record below is how
//! the loop was measured.

use core::mem::MaybeUninit;

use mk20dx_hal::pac;

/// RCM latches why the last reset happened, and keeps it across resets.
const RCM_SRS0: *const u8 = 0x4007_F000 as *const u8;
const RCM_SRS1: *const u8 = 0x4007_F001 as *const u8;

const DIAG_MAGIC: u32 = 0x5744_4744; // "WDGD"

/// Post-mortem record kept in .uninit, which startup does not zero and a reset
/// does not clear. A chip that resets several times a second cannot be read over
/// SWD - the AP is gone most of the time - so the target records for itself why
/// each boot happened, and the host collects the tally afterwards.
#[repr(C)]
struct Diag {
    magic: u32,
    boots: u32,
    /// SRS0 of the most recent reset (K20 RM ch.13): bit 7 = POR, bit 6 =
    /// external pin (nRST), bit 5 = WDOG, bit 3 = LOL, bit 2 = LOC, bit 1 = LVD,
    /// bit 0 = wakeup.
    srs0: u32,
    /// SRS1. Bit 1 = LOCKUP, bit 2 = SW (SYSRESETREQ), bit 3 = MDM-AP.
    srs1: u32,
    /// Bitwise OR of every SRS0 seen, so a single read shows every source hit.
    srs0_seen: u32,
    srs1_seen: u32,
    /// WDOG STCTRLH as found at boot: bit0 WDOGEN, bit1 CLKSRC, bit3 WINEN,
    /// bit4 ALLOWUPDATE, bit5 DBGEN. A set WINEN would mean an early refresh is
    /// itself a reset.
    stctrlh: u32,
    /// TOVALH:TOVALL and WINH:WINL, to show the real timeout and window.
    toval: u32,
    win: u32,
}

#[link_section = ".uninit.DIAG"]
#[no_mangle]
static mut DIAG: MaybeUninit<Diag> = MaybeUninit::uninit();

/// Bumped every loop iteration, at the SRAM_L base. Read it over the MEM-AP to
/// confirm the firmware is alive.
#[no_mangle]
pub static mut HEARTBEAT: u32 = 0;

/// Spin rather than reset, so a panic cannot be mistaken for a WDOG reset.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // Record why this boot happened, before anything else can disturb it.
    unsafe {
        let srs0 = RCM_SRS0.read_volatile() as u32;
        let srs1 = RCM_SRS1.read_volatile() as u32;
        let d = (&raw mut DIAG) as *mut Diag;
        if (&raw const (*d).magic).read_volatile() != DIAG_MAGIC {
            // First boot after power-on: the record is garbage, start it clean.
            (&raw mut (*d).magic).write_volatile(DIAG_MAGIC);
            (&raw mut (*d).boots).write_volatile(0);
            (&raw mut (*d).srs0_seen).write_volatile(0);
            (&raw mut (*d).srs1_seen).write_volatile(0);
        }
        let boots = (&raw const (*d).boots).read_volatile().wrapping_add(1);
        (&raw mut (*d).boots).write_volatile(boots);
        (&raw mut (*d).srs0).write_volatile(srs0);
        (&raw mut (*d).srs1).write_volatile(srs1);
        let seen0 = (&raw const (*d).srs0_seen).read_volatile() | srs0;
        let seen1 = (&raw const (*d).srs1_seen).read_volatile() | srs1;
        (&raw mut (*d).srs0_seen).write_volatile(seen0);
        (&raw mut (*d).srs1_seen).write_volatile(seen1);

        // Capture the WDOG configuration as the hardware actually left it.
        (&raw mut (*d).stctrlh)
            .write_volatile((0x4005_2000 as *const u16).read_volatile() as u32);
        let tovalh = (0x4005_2004 as *const u16).read_volatile() as u32;
        let tovall = (0x4005_2006 as *const u16).read_volatile() as u32;
        (&raw mut (*d).toval).write_volatile((tovalh << 16) | tovall);
        let winh = (0x4005_2008 as *const u16).read_volatile() as u32;
        let winl = (0x4005_200a as *const u16).read_volatile() as u32;
        (&raw mut (*d).win).write_volatile((winh << 16) | winl);
    }

    let dp = pac::Peripherals::take().unwrap();

    // Deliberately NOT calling dp.wdog.disable(). The WDOG stays at its reset
    // default (~1.25 s timeout) and this loop keeps it fed.
    loop {
        unsafe {
            let p = &raw mut HEARTBEAT;
            p.write_volatile(p.read_volatile().wrapping_add(1));

            // WDOG refresh sequence: 0xA602 then 0xB480, back to back.
            cortex_m::interrupt::free(|_| {
                dp.wdog.refresh().write(|w| w.bits(0xA602));
                dp.wdog.refresh().write(|w| w.bits(0xB480));
            });
        }
        cortex_m::asm::delay(1000);
    }
}
