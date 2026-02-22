//! FTM advanced feature self-tests — validates Input Capture, Output Compare,
//! and Quadrature Decoder modes via register checks.
//!
//! FTM0 is used for OC (ch0) and IC (ch1). FTM2 is used for QuadratureDecoder.
//! MOD is set to 0xFFFF manually so the counter actually runs through a
//! full 16-bit range.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::pwm::{
    CaptureEdge, CompareAction, Ftm0, Ftm2, InputCapture, OutputCompare,
    QuadMode, QuadratureDecoder,
};

struct State {
    oc: OutputCompare<Ftm0, 0>,
    ic: InputCapture<Ftm0, 1>,
    quad: QuadratureDecoder<Ftm2>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        // Create OC and IC on FTM0 — these enable the clock and start the counter
        let oc = OutputCompare::<Ftm0, 0>::new(
            CompareAction::Toggle,
            5000,
            &clocks,
            &dp.sim,
        );
        let ic = InputCapture::<Ftm0, 1>::new(CaptureEdge::Rising, &clocks, &dp.sim);

        // Set FTM0 MOD=0xFFFF so counter runs through full range
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0xFFFF) });

        // Create QuadratureDecoder on FTM2 (sets MOD=0xFFFF internally)
        let quad = QuadratureDecoder::<Ftm2>::new(QuadMode::PhaseAB, &dp.sim);

        super::State { oc, ic, quad }
    }

    // --- Output Compare tests ---

    /// OC ch0 CnSC should have MSA=1 (output compare), ELSA=1 (toggle mode).
    #[test]
    fn test_oc_csc_bits(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let csc = ftm0.csc(0).read();
        defmt::assert!(csc.msa().bit(), "MSA should be 1 for output compare");
        defmt::assert!(csc.elsa().bit(), "ELSA should be 1 for toggle mode");
        defmt::assert!(!csc.msb().bit(), "MSB should be 0 for output compare");
    }

    /// set_compare(5000) → CnV should read 5000.
    #[test]
    fn test_oc_set_compare(state: &mut super::State) {
        state.oc.set_compare(5000);
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let cnv = ftm0.cv(0).read().val().bits();
        defmt::info!("OC CnV: {} (expected 5000)", cnv);
        defmt::assert_eq!(cnv, 5000, "CnV should be 5000");
    }

    /// Wait for the counter to wrap past the compare value,
    /// then has_matched() should return true.
    #[test]
    fn test_oc_match_flag(state: &mut super::State) {
        state.oc.clear_flag();
        state.oc.set_compare(100); // Low value so counter reaches it quickly

        // Spin until match (at 36 MHz bus clock, counter reaches 100 very fast)
        let mut timeout = 100_000u32;
        while !state.oc.has_matched() && timeout > 0 {
            timeout -= 1;
        }

        defmt::assert!(state.oc.has_matched(), "OC should have matched");
        state.oc.clear_flag();
        state.oc.set_compare(5000); // Restore
    }

    /// CHIE bit should toggle with enable/disable_interrupt.
    #[test]
    fn test_oc_interrupt_control(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        state.oc.enable_interrupt();
        defmt::assert!(
            ftm0.csc(0).read().chie().bit(),
            "CHIE should be set after enable_interrupt"
        );

        state.oc.disable_interrupt();
        defmt::assert!(
            !ftm0.csc(0).read().chie().bit(),
            "CHIE should be cleared after disable_interrupt"
        );
    }

    // --- Input Capture tests ---

    /// IC ch1 CnSC should have MSA=0, MSB=0, ELSA=1 (rising edge capture).
    #[test]
    fn test_ic_csc_bits(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let csc = ftm0.csc(1).read();
        defmt::assert!(!csc.msa().bit(), "MSA should be 0 for input capture");
        defmt::assert!(!csc.msb().bit(), "MSB should be 0 for input capture");
        defmt::assert!(csc.elsa().bit(), "ELSA should be 1 for rising edge capture");
    }

    /// With no signal on the unconnected pin, capture() should return None.
    #[test]
    fn test_ic_no_capture(state: &mut super::State) {
        state.ic.clear_flag();
        let result = state.ic.capture();
        defmt::info!("IC capture on unconnected pin: {:?}", result);
        defmt::assert!(result.is_none(), "No capture expected on unconnected pin");
    }

    /// CHIE bit should toggle with enable/disable_interrupt.
    #[test]
    fn test_ic_interrupt_control(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        state.ic.enable_interrupt();
        defmt::assert!(
            ftm0.csc(1).read().chie().bit(),
            "CHIE should be set after enable_interrupt"
        );

        state.ic.disable_interrupt();
        defmt::assert!(
            !ftm0.csc(1).read().chie().bit(),
            "CHIE should be cleared after disable_interrupt"
        );
    }

    // --- Quadrature Decoder tests ---

    /// FTM2 QDCTRL: QUADEN=1, QUADMODE=0 (PhaseAB mode).
    #[test]
    fn test_quad_enabled(_state: &mut super::State) {
        let ftm2 = unsafe { &*pac::Ftm2::PTR };
        let qdctrl = ftm2.qdctrl().read();
        defmt::assert!(
            qdctrl.quaden().is_1(),
            "QUADEN should be 1 for quadrature mode"
        );
        defmt::assert!(
            qdctrl.quadmode().is_0(),
            "QUADMODE should be 0 for PhaseAB"
        );
    }

    /// With no external signal, counter should be 0 initially.
    #[test]
    fn test_quad_count_zero(state: &mut super::State) {
        let count = state.quad.count();
        defmt::info!("Quad counter: {}", count);
        defmt::assert_eq!(count, 0, "Quadrature counter should be 0 with no input");
    }
}
