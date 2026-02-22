#![no_std]
#![no_main]

use mk20dx_hal::pac;
use mk20dx_hal::prelude::*;

use defmt_rtt as _;
use panic_probe as _;

// defmt requires a timestamp function
defmt::timestamp!("{=u32}", 0);

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Disable watchdog first — must happen quickly after reset
    dp.wdog.disable();

    // Enable PORTC clock gate
    dp.sim.scgc5().modify(|_, w| w.portc()._1());

    // Set PTC5 (Teensy pin 13) to GPIO mode
    dp.portc.pcr(5).write(|w| w.mux().gpio());

    // Set PTC5 as output
    dp.ptc.pddr().modify(|r, w| unsafe { w.bits(r.bits() | (1 << 5)) });

    defmt::info!("Hello from Teensy 3.2!");

    let mut count: u32 = 0;
    loop {
        defmt::info!("Blink #{}", count);

        // Set high
        dp.ptc.psor().write(|w| unsafe { w.bits(1 << 5) });
        cortex_m::asm::delay(2_000_000);

        // Set low
        dp.ptc.pcor().write(|w| unsafe { w.bits(1 << 5) });
        cortex_m::asm::delay(2_000_000);

        count = count.wrapping_add(1);
    }
}
