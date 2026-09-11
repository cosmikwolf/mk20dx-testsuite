#![no_std]
#![no_main]

//! `bench` with one change: a short busy delay before `wdog.disable()`.
//!
//! K20 RM §23.3.2: the WDOG must be unlocked within WCT (256 bus clock cycles)
//! of a system reset, "failing which the WDOG issues a reset to the system".
//! `bench` slips under that limit by accident - it has no `.data` and one word
//! of `.bss`, so `disable()` runs about 60 instructions after reset. Any real
//! firmware that copies `.data` and zeroes `.bss` first does not.
//!
//! So this binary is the minimal reproduction of the flxs1 reset loop: if the
//! WCT rule is real, this loops at ~20 kHz standalone and is fine under a
//! debugger (RM: "entry into Debug mode within WCT after reset ... there is no
//! need to unlock and configure it within WCT"). `bench` is the control.
//!
//! Reset-source record layout matches flxs1's `RESET_DIAG` so `hold_collect`
//! decodes it: magic, resets, srs0, srs1, srs0_seen, srs1_seen.

use core::mem::MaybeUninit;

use mk20dx_hal::pac;
use mk20dx_hal::prelude::*;

const RCM_SRS0: *const u8 = 0x4007_F000 as *const u8;
const RCM_SRS1: *const u8 = 0x4007_F001 as *const u8;
const RESET_MAGIC: u32 = 0x5253_5453; // "RSTS"

#[repr(C)]
struct ResetDiag {
    magic: u32,
    resets: u32,
    srs0: u32,
    srs1: u32,
    srs0_seen: u32,
    srs1_seen: u32,
}

#[used]
#[no_mangle]
#[link_section = ".uninit.RESET_DIAG"]
static mut RESET_DIAG: MaybeUninit<ResetDiag> = MaybeUninit::uninit();

/// Bumped every loop iteration; proves the firmware got past init.
#[no_mangle]
pub static mut HEARTBEAT: u32 = 0;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[cortex_m_rt::pre_init]
unsafe fn record_reset_source() {
    let srs0 = RCM_SRS0.read_volatile() as u32;
    let srs1 = RCM_SRS1.read_volatile() as u32;
    let d = core::ptr::addr_of_mut!(RESET_DIAG).cast::<ResetDiag>();
    if core::ptr::read_volatile(core::ptr::addr_of!((*d).magic)) != RESET_MAGIC {
        core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).magic), RESET_MAGIC);
        core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).resets), 0);
        core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).srs0_seen), 0);
        core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).srs1_seen), 0);
    }
    let n = core::ptr::read_volatile(core::ptr::addr_of!((*d).resets)).wrapping_add(1);
    core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).resets), n);
    core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).srs0), srs0);
    core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).srs1), srs1);
    let s0 = core::ptr::read_volatile(core::ptr::addr_of!((*d).srs0_seen)) | srs0;
    let s1 = core::ptr::read_volatile(core::ptr::addr_of!((*d).srs1_seen)) | srs1;
    core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).srs0_seen), s0);
    core::ptr::write_volatile(core::ptr::addr_of_mut!((*d).srs1_seen), s1);
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // The one difference from `bench`: ~1000 cycles of nothing, standing in
    // for the .data copy / .bss zeroing a real firmware does here.
    cortex_m::asm::delay(1000);

    dp.wdog.disable();

    loop {
        unsafe {
            let p = &raw mut HEARTBEAT;
            p.write_volatile(p.read_volatile().wrapping_add(1));
        }
        cortex_m::asm::delay(1000);
    }
}
