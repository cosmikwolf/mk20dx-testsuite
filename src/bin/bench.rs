#![no_std]
#![no_main]

//! Minimal, deterministic target for exercising probe-rs debug sequences.
//!
//! Deliberately has no RTT, no interrupts, and no self-reset path. The core
//! disables the WDOG and then spins, bumping a counter at a fixed address. That
//! gives two things the host can check over SWD:
//!
//!   * HEARTBEAT advances  -> the firmware is running
//!   * the chip never resets itself -> any reset seen over SWD is the debugger
//!
//! With this flashed, nothing in the target can produce a reset loop, so a loop
//! observed on the MDM-AP can only come from the host side.

use mk20dx_hal::pac;
use mk20dx_hal::prelude::*;

/// Bumped every loop iteration. Read it over the MEM-AP to confirm the firmware
/// is alive. Find its address with `nm` - it lands in .bss.
#[no_mangle]
pub static mut HEARTBEAT: u32 = 0;

/// Spin rather than reset. A panic here must never reboot the chip, or it would
/// reintroduce the very behaviour this firmware exists to rule out.
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Must happen quickly after reset - the unlock has a 20 bus cycle window.
    dp.wdog.disable();

    loop {
        // Plain spin, not WFI: keeps the core fully awake so a sleep state
        // cannot confound MDM-AP readings.
        unsafe {
            let p = &raw mut HEARTBEAT;
            p.write_volatile(p.read_volatile().wrapping_add(1));
        }
        cortex_m::asm::delay(1000);
    }
}
