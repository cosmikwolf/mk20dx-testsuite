//! Watchdog self-tests — validates WDOG disable sequence.
//!
//! Priority: CRITICAL
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;

struct State {
    // No fields needed — tests read registers directly
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        super::State {}
    }

    /// After disable(), the WDOGEN bit in STCTRLH should be 0.
    #[test]
    fn test_watchdog_disable(_state: &mut super::State) {
        let wdog = unsafe { &*pac::Wdog::PTR };
        let stctrlh = wdog.stctrlh().read();
        defmt::assert!(
            stctrlh.wdogen().is_disabled(),
            "WDOGEN should be 0 after disable"
        );
    }

    /// If the watchdog is properly disabled, a 500ms busy-wait should complete
    /// without the device resetting. We verify the watchdog timer counter is
    /// not incrementing (frozen at its disabled value).
    #[test]
    fn test_watchdog_survives_500ms(_state: &mut super::State) {
        let wdog = unsafe { &*pac::Wdog::PTR };

        // Read watchdog timer counter before delay
        let tmr_before_h = wdog.tmrouth().read().bits();
        let tmr_before_l = wdog.tmroutl().read().bits();

        // Busy-wait ~500ms at 72 MHz (~36M cycles)
        cortex_m::asm::delay(36_000_000);

        // Read watchdog timer counter after delay
        let tmr_after_h = wdog.tmrouth().read().bits();
        let tmr_after_l = wdog.tmroutl().read().bits();

        defmt::info!(
            "WDOG TMR: before=0x{:04X}{:04X} after=0x{:04X}{:04X}",
            tmr_before_h, tmr_before_l,
            tmr_after_h, tmr_after_l
        );

        // With WDOGEN=0, the timer should not be counting
        defmt::assert!(
            tmr_before_h == tmr_after_h && tmr_before_l == tmr_after_l,
            "WDOG timer should not increment when disabled"
        );

        defmt::info!("Survived 500ms without watchdog reset");
    }

    /// After disabling the watchdog, we should be able to access peripheral
    /// registers and configure GPIO. Verify by toggling a GPIO pin direction
    /// register and confirming the write took effect.
    #[test]
    fn test_system_functional_after_disable(_state: &mut super::State) {
        // Enable PORTD clock gate
        let sim = unsafe { &*pac::Sim::PTR };
        sim.scgc5().modify(|_, w| w.portd().enabled());

        // Set PTD0 as output
        let ptd = unsafe { &*pac::Ptd::PTR };
        ptd.pddr().modify(|r, w| unsafe { w.bits(r.bits() | 1) });

        // Verify direction bit is set
        let pddr = ptd.pddr().read().bits();
        defmt::assert!(
            pddr & 1 != 0,
            "PDDR bit 0 should be set after configuring as output"
        );

        // Clean up: set back to input
        ptd.pddr().modify(|r, w| unsafe { w.bits(r.bits() & !1) });

        defmt::info!("System functional after watchdog disable — GPIO register write verified");
    }
}
