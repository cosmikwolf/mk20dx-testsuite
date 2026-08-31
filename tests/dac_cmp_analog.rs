//! DAC → CMP analog loopback tests.
//!
//! dac.rs writes a value and reads the register back; cmp.rs checks DACCR and
//! VOSEL. Neither observes an analog signal. These do, and they need no wiring:
//! the chip routes the 12-bit DAC output to CMP1 input 3 internally (ch03, chip
//! configuration, "12-bit DAC0_OUT/CMP1_IN3").
//!
//! CMP1 compares that against its own 6-bit DAC on input 7, so sweeping the
//! 12-bit DAC past the 6-bit reference must flip COUT. That exercises the DAC's
//! analog output and the comparator's decision in one measurement.
//!
//! Both references are set to VDDA so the two ladders share a scale: the 6-bit
//! reference sits at Vin x (level + 1) / 64, and the 12-bit DAC at value / 4095.
//!
//! Priority: HIGH
//! Wiring: None — the DAC → CMP1_IN3 path is internal

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::cmp::{Cmp, Cmp1, CmpDacVref, CmpExt, Input};
use hal::dac::{Dac, DacExt, VrefSource};
use hal::pac;
use hal::prelude::*;

/// 6-bit reference level. Vout = Vin x (31 + 1) / 64, so half of VDDA.
const REF_LEVEL: u8 = 31;
/// The 12-bit code that should sit at the same voltage.
const REF_CODE: u16 = 2048;

/// Analog settling after a DAC step, generous enough for the comparator to
/// follow without the test depending on how fast it does.
fn settle() {
    cortex_m::asm::delay(20_000);
}

struct State {
    dac: Dac,
    cmp: Cmp<Cmp1>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        let mut dac = dp.dac0.dac(&dp.sim);
        dac.set_vref(VrefSource::Vref1); // VDDA
        dac.set_value(0);
        dac.enable();

        // Plus = IN3, the 12-bit DAC output. Minus = IN7, the 6-bit reference.
        // COUT is then "the 12-bit DAC is above the reference".
        let mut cmp = dp.cmp1.cmp(Input::IN3, Input::INTERNAL_DAC, &dp.sim);
        cmp.set_internal_dac(REF_LEVEL, CmpDacVref::Vin1);
        cmp.enable();
        settle();

        super::State { dac, cmp }
    }

    /// At zero the DAC output sits below the reference.
    #[test]
    fn test_dac_low_reads_below_reference(state: &mut super::State) {
        state.dac.set_value(0);
        settle();
        defmt::assert!(
            !state.cmp.output(),
            "COUT should be low with the DAC at 0 and the reference at mid-scale"
        );
    }

    /// At full scale it sits above.
    #[test]
    fn test_dac_high_reads_above_reference(state: &mut super::State) {
        state.dac.set_value(4095);
        settle();
        defmt::assert!(
            state.cmp.output(),
            "COUT should be high with the DAC at full scale"
        );
    }

    /// COUT follows the DAC back and forth, so it tracks rather than latching.
    #[test]
    fn test_cout_follows_the_dac(state: &mut super::State) {
        for _ in 0..4 {
            state.dac.set_value(0);
            settle();
            defmt::assert!(!state.cmp.output(), "COUT stuck high after a low step");

            state.dac.set_value(4095);
            settle();
            defmt::assert!(state.cmp.output(), "COUT stuck low after a high step");
        }
        defmt::info!("COUT tracked 4 full swings");
    }

    /// The DAC output really is monotonic across its range: sweeping upward
    /// crosses the reference exactly once, near the code that matches it.
    ///
    /// This is the test that proves the analog output moves with the code,
    /// rather than merely that the data register accepted it.
    #[test]
    fn test_sweep_crosses_once_near_midscale(state: &mut super::State) {
        let mut transitions = 0u32;
        let mut threshold = 0u16;
        let mut previous = false;

        state.dac.set_value(0);
        settle();
        previous = state.cmp.output();

        let mut code = 0u16;
        while code <= 4095 {
            state.dac.set_value(code);
            settle();
            let now = state.cmp.output();
            if now != previous {
                transitions += 1;
                threshold = code;
                previous = now;
            }
            code += 64;
        }

        defmt::info!(
            "sweep crossed {} time(s), threshold near code {} (reference code {})",
            transitions, threshold, REF_CODE
        );
        defmt::assert_eq!(transitions, 1, "a monotonic sweep should cross once");

        // A quarter of full scale either side is loose enough for reference and
        // ladder tolerances, tight enough to catch an output stuck at a rail.
        let low = REF_CODE - 1024;
        let high = REF_CODE + 1024;
        defmt::assert!(
            threshold > low && threshold < high,
            "threshold {} should sit near mid-scale, between {} and {}",
            threshold, low, high
        );
    }

    /// Moving the 6-bit reference moves the crossing point with it.
    ///
    /// Confirms the comparator is really comparing against its own DAC, not
    /// against a fixed rail that happens to sit mid-scale.
    #[test]
    fn test_reference_level_moves_the_threshold(state: &mut super::State) {
        // A low reference: the DAC should already be above it at a quarter scale.
        state.cmp.set_internal_dac(7, CmpDacVref::Vin1); // 8/64 of VDDA
        state.dac.set_value(1024); // about a quarter of full scale
        settle();
        let above_low_ref = state.cmp.output();

        // A high reference: the same DAC code should now be below it.
        state.cmp.set_internal_dac(55, CmpDacVref::Vin1); // 56/64 of VDDA
        settle();
        let above_high_ref = state.cmp.output();

        // Put it back for any test that runs after this one.
        state.cmp.set_internal_dac(REF_LEVEL, CmpDacVref::Vin1);
        settle();

        defmt::info!(
            "at code 1024: above low ref {}, above high ref {}",
            above_low_ref, above_high_ref
        );
        defmt::assert!(above_low_ref, "code 1024 should exceed a 8/64 reference");
        defmt::assert!(!above_high_ref, "code 1024 should fall below a 56/64 reference");
    }
}
