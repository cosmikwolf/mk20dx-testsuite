//! PDB (Programmable Delay Block) self-tests.
//!
//! The PDB driver had no tests at all, even though the ADC's PDB-triggered
//! scan modes depend on it. adc.rs never touches it.
//!
//! Most of these watch the counter actually run rather than checking register
//! contents: a PDB that accepts configuration but never counts would pass a
//! register-level test and fail every one of these.
//!
//! Priority: HIGH
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::pdb::{Multiplier, Pdb, PdbExt, Prescaler, TriggerSource};
use hal::prelude::*;

/// Large enough that the counter is easy to catch mid-run at this prescaler.
const MODULUS: u16 = 0xFFFF;

struct State {
    pdb: Pdb,
}

/// Configure for a software-triggered run and arm it.
fn arm(pdb: &mut Pdb, continuous: bool) {
    pdb.disable();
    pdb.configure(
        TriggerSource::Software,
        Prescaler::Div128,
        Multiplier::Mult40,
        MODULUS,
    );
    pdb.set_continuous(continuous);
    // enable() before load_ok(): the buffered load needs the PDB clocked.
    pdb.enable();
    pdb.load_ok();
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        let pdb = dp.pdb0.constrain(&dp.sim);
        super::State { pdb }
    }

    /// The clock gate opens when the driver is constructed.
    #[test]
    fn test_clock_gate_enabled(_state: &mut super::State) {
        let sim = unsafe { &*pac::Sim::PTR };
        defmt::assert!(
            sim.scgc6().read().pdb().is_enabled(),
            "SIM_SCGC6.PDB should be enabled"
        );
    }

    /// configure() lands in SC and MOD.
    #[test]
    fn test_configure_writes_registers(state: &mut super::State) {
        state.pdb.disable();
        state.pdb.configure(
            TriggerSource::Software,
            Prescaler::Div128,
            Multiplier::Mult40,
            0x1234,
        );
        state.pdb.enable();
        state.pdb.load_ok();
        cortex_m::asm::delay(1_000);

        let pdb = unsafe { &*pac::Pdb0::PTR };
        let sc = pdb.sc().read();
        defmt::assert_eq!(sc.trgsel().bits(), 15, "TRGSEL should select software");
        defmt::assert_eq!(sc.prescaler().bits(), 7, "PRESCALER should be Div128");
        defmt::assert_eq!(sc.mult().bits(), 3, "MULT should be Mult40");
        defmt::assert_eq!(pdb.mod_().read().mod_().bits(), 0x1234, "MOD should be set");
    }

    /// MOD stays in its buffer until the PDB is enabled.
    ///
    /// LDOK only moves the buffered value across while the PDB is clocked, so
    /// configure() then load_ok() then enable() silently keeps the old modulus.
    /// MOD resets to 0xFFFF, which makes the mistake easy to miss: the counter
    /// still runs, just with the wrong period.
    #[test]
    fn test_mod_needs_enable_before_load_ok(state: &mut super::State) {
        let pdb = unsafe { &*pac::Pdb0::PTR };

        state.pdb.disable();
        state.pdb.configure(
            TriggerSource::Software,
            Prescaler::Div128,
            Multiplier::Mult40,
            0x0ABC,
        );
        state.pdb.load_ok(); // too early — the PDB is not clocked yet
        cortex_m::asm::delay(1_000);
        let while_disabled = pdb.mod_().read().mod_().bits();

        state.pdb.enable();
        state.pdb.load_ok();
        cortex_m::asm::delay(1_000);
        let after_enable = pdb.mod_().read().mod_().bits();

        defmt::info!(
            "MOD with LDOK while disabled: {}, after enable + LDOK: {}",
            while_disabled, after_enable
        );
        defmt::assert!(
            while_disabled != 0x0ABC,
            "LDOK appeared to work while disabled, so this ordering trap is gone"
        );
        defmt::assert_eq!(after_enable, 0x0ABC, "MOD should load once enabled");
    }

    /// Without a trigger the counter stays put.
    #[test]
    fn test_counter_idle_before_trigger(state: &mut super::State) {
        arm(&mut state.pdb, false);
        let first = state.pdb.counter();
        cortex_m::asm::delay(200_000);
        let second = state.pdb.counter();

        defmt::info!("idle counter: {} then {}", first, second);
        defmt::assert_eq!(first, second, "counter moved with no trigger issued");
    }

    /// A software trigger starts it, and it counts.
    #[test]
    fn test_software_trigger_starts_the_counter(state: &mut super::State) {
        arm(&mut state.pdb, true);
        let before = state.pdb.counter();

        state.pdb.software_trigger();
        cortex_m::asm::delay(200_000);
        let after = state.pdb.counter();

        defmt::info!("counter {} -> {} after trigger", before, after);
        defmt::assert!(after != before, "counter did not advance after a software trigger");
    }

    /// It keeps advancing, so it is running rather than having stepped once.
    #[test]
    fn test_counter_advances_monotonically(state: &mut super::State) {
        arm(&mut state.pdb, true);
        state.pdb.software_trigger();

        let mut samples = [0u16; 4];
        for slot in samples.iter_mut() {
            cortex_m::asm::delay(100_000);
            *slot = state.pdb.counter();
        }
        defmt::info!("samples: {}", samples);

        let mut moves = 0;
        for pair in samples.windows(2) {
            if pair[1] != pair[0] {
                moves += 1;
            }
        }
        defmt::assert!(moves >= 2, "counter should keep moving, saw {} changes", moves);
    }

    /// Continuous mode wraps rather than stopping at the modulus.
    #[test]
    fn test_continuous_mode_wraps(state: &mut super::State) {
        state.pdb.disable();
        state.pdb.configure(
            TriggerSource::Software,
            Prescaler::Div1,
            Multiplier::Mult1,
            0x00FF, // short period, so it wraps quickly
        );
        state.pdb.set_continuous(true);
        state.pdb.enable();
        state.pdb.load_ok();
        state.pdb.software_trigger();

        // Look for the counter to be below a previous reading, which can only
        // happen if it wrapped.
        let mut wrapped = false;
        let mut previous = state.pdb.counter();
        for _ in 0..200 {
            cortex_m::asm::delay(200);
            let now = state.pdb.counter();
            if now < previous {
                wrapped = true;
                break;
            }
            previous = now;
        }
        defmt::assert!(wrapped, "continuous mode never wrapped past the modulus");
        defmt::info!("continuous mode wrapped");
    }

    /// Disabling stops the counter.
    #[test]
    fn test_disable_stops_the_counter(state: &mut super::State) {
        arm(&mut state.pdb, true);
        state.pdb.software_trigger();
        cortex_m::asm::delay(100_000);

        state.pdb.disable();
        let first = state.pdb.counter();
        cortex_m::asm::delay(200_000);
        let second = state.pdb.counter();

        defmt::info!("after disable: {} then {}", first, second);
        defmt::assert_eq!(first, second, "counter kept running after disable()");
    }

    /// Pre-trigger delay and enable reach CH0.
    #[test]
    fn test_pretrigger_config(state: &mut super::State) {
        state.pdb.disable();
        state.pdb.configure(
            TriggerSource::Software,
            Prescaler::Div128,
            Multiplier::Mult40,
            MODULUS,
        );
        state.pdb.set_pretrigger_delay(0, 0, 0x0080);
        state.pdb.enable_pretrigger(0, 0);
        // CHnDLYm is buffered like MOD, so the PDB must be clocked for LDOK.
        state.pdb.enable();
        state.pdb.load_ok();
        cortex_m::asm::delay(1_000);

        let pdb = unsafe { &*pac::Pdb0::PTR };
        defmt::assert_eq!(
            pdb.chdly0(0).read().dly().bits(), 0x0080,
            "CH0 pre-trigger 0 delay should be set"
        );
        defmt::assert!(
            pdb.chc1(0).read().en().bits() & 0x01 != 0,
            "CH0 pre-trigger 0 should be enabled"
        );

        state.pdb.disable_pretrigger(0, 0);
        state.pdb.load_ok();
        cortex_m::asm::delay(1_000);
        defmt::assert!(
            pdb.chc1(0).read().en().bits() & 0x01 == 0,
            "CH0 pre-trigger 0 should be disabled again"
        );
    }

    /// No sequence errors arise from a plain run.
    #[test]
    fn test_no_sequence_errors_on_clean_run(state: &mut super::State) {
        state.pdb.clear_errors(0);
        arm(&mut state.pdb, true);
        state.pdb.software_trigger();
        cortex_m::asm::delay(400_000);

        let errors = state.pdb.clear_errors(0);
        defmt::info!("sequence error flags: {}", errors);
        defmt::assert_eq!(errors, 0, "a run with no ADC attached should not error");
    }
}
