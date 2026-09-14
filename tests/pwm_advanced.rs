//! FTM advanced feature self-tests — validates Input Capture, Output Compare,
//! and Quadrature Decoder modes via register checks.
//!
//! FTM0 uses the split API: timer handle controls MOD/counter, ch0 is OC,
//! ch1 is IC. FTM2 uses standalone QuadratureDecoder.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::clocks::Clocks;
use hal::pwm::{
    CaptureEdge, CompareAction, Ftm0Parts, Ftm1Parts, Ftm2,
    FtmExt, PwmAlignment, PwmPolarity,
    QuadMode, QuadratureDecoder,
};
use hal::time::U32Ext;

struct State {
    ftm0: Ftm0Parts,
    ftm1: Ftm1Parts,
    clocks: Clocks,
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

        // Split FTM0 and configure channels
        let mut ftm0 = dp.ftm0.split(&clocks, &dp.sim);

        // Set MOD=0xFFFF so counter runs through full range
        ftm0.timer.set_modulo(0xFFFF);

        // Configure ch0 for output compare (toggle at 5000)
        ftm0.ch0.set_output_compare(CompareAction::Toggle, 5000);

        // Configure ch1 for input capture (rising edge)
        ftm0.ch1.set_input_capture(CaptureEdge::Rising);

        // Start the counter
        ftm0.timer.start();

        // Split FTM1 for center-aligned / polarity tests
        let ftm1 = dp.ftm1.split(&clocks, &dp.sim);

        // Create QuadratureDecoder on FTM2 (sets MOD=0xFFFF internally)
        let quad = QuadratureDecoder::<Ftm2>::new(QuadMode::PhaseAB, &dp.sim);

        super::State { ftm0, ftm1, clocks, quad }
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

    /// set_value(5000) → CnV should read 5000.
    #[test]
    fn test_oc_set_compare(state: &mut super::State) {
        state.ftm0.ch0.set_value(5000);
        let cnv = state.ftm0.ch0.value();
        defmt::info!("OC CnV: {} (expected 5000)", cnv);
        defmt::assert_eq!(cnv, 5000, "CnV should be 5000");
    }

    /// Wait for the counter to wrap past the compare value,
    /// then has_flag() should return true.
    #[test]
    fn test_oc_match_flag(state: &mut super::State) {
        state.ftm0.ch0.clear_flag();
        state.ftm0.ch0.set_value(100); // Low value so counter reaches it quickly

        // Spin until match (at 36 MHz bus clock, counter reaches 100 very fast)
        let mut timeout = 100_000u32;
        while !state.ftm0.ch0.has_flag() && timeout > 0 {
            timeout -= 1;
        }

        defmt::assert!(state.ftm0.ch0.has_flag(), "OC should have matched");
        state.ftm0.ch0.clear_flag();
        state.ftm0.ch0.set_value(5000); // Restore
    }

    /// CHIE bit should toggle with enable/disable_interrupt.
    #[test]
    fn test_oc_interrupt_control(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        state.ftm0.ch0.enable_interrupt();
        defmt::assert!(
            ftm0.csc(0).read().chie().bit(),
            "CHIE should be set after enable_interrupt"
        );

        state.ftm0.ch0.disable_interrupt();
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
        state.ftm0.ch1.clear_flag();
        let result = state.ftm0.ch1.capture();
        defmt::info!("IC capture on unconnected pin: {:?}", result);
        defmt::assert!(result.is_none(), "No capture expected on unconnected pin");
    }

    /// CHIE bit should toggle with enable/disable_interrupt.
    #[test]
    fn test_ic_interrupt_control(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        state.ftm0.ch1.enable_interrupt();
        defmt::assert!(
            ftm0.csc(1).read().chie().bit(),
            "CHIE should be set after enable_interrupt"
        );

        state.ftm0.ch1.disable_interrupt();
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

    // --- Center-aligned PWM tests ---

    /// set_alignment(CenterAligned) should set SC.CPWMS=1.
    #[test]
    fn test_center_aligned_cpwms_bit(state: &mut super::State) {
        let ftm1 = unsafe { &*pac::Ftm1::PTR };

        state.ftm1.timer.set_alignment(PwmAlignment::CenterAligned);
        defmt::assert!(
            ftm1.sc().read().cpwms().bit(),
            "CPWMS should be 1 after set_alignment(CenterAligned)"
        );

        // Verify read-back
        defmt::assert_eq!(
            state.ftm1.timer.alignment(),
            PwmAlignment::CenterAligned,
            "alignment() should return CenterAligned"
        );

        // Restore edge-aligned
        state.ftm1.timer.set_alignment(PwmAlignment::EdgeAligned);
        defmt::assert!(
            !ftm1.sc().read().cpwms().bit(),
            "CPWMS should be 0 after set_alignment(EdgeAligned)"
        );
    }

    /// set_frequency() should preserve CPWMS when center-aligned is active.
    #[test]
    fn test_center_aligned_preserves_on_set_frequency(state: &mut super::State) {
        state.ftm1.timer.set_alignment(PwmAlignment::CenterAligned);
        state.ftm1.timer.set_frequency(1000u32.Hz(), &state.clocks);

        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        defmt::assert!(
            ftm1.sc().read().cpwms().bit(),
            "CPWMS should be preserved after set_frequency()"
        );
        defmt::assert_eq!(
            state.ftm1.timer.alignment(),
            PwmAlignment::CenterAligned,
            "alignment() should still be CenterAligned after set_frequency"
        );

        // Restore edge-aligned
        state.ftm1.timer.set_alignment(PwmAlignment::EdgeAligned);
    }

    // --- PWM polarity tests ---

    /// set_pwm_polarity(HighTrue) should set MSB=1, ELSB=1, ELSA=0.
    #[test]
    fn test_pwm_polarity_high_true(state: &mut super::State) {
        state.ftm1.ch0.set_pwm_polarity(PwmPolarity::HighTrue);

        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        let csc = ftm1.csc(0).read();
        defmt::assert!(csc.msb().bit(), "MSB should be 1 for PWM");
        defmt::assert!(csc.elsb().bit(), "ELSB should be 1 for high-true");
        defmt::assert!(!csc.elsa().bit(), "ELSA should be 0 for high-true");
    }

    /// set_pwm_polarity(LowTrue) should set MSB=1, ELSB=0, ELSA=1.
    #[test]
    fn test_pwm_polarity_low_true(state: &mut super::State) {
        state.ftm1.ch0.set_pwm_polarity(PwmPolarity::LowTrue);

        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        let csc = ftm1.csc(0).read();
        defmt::assert!(csc.msb().bit(), "MSB should be 1 for PWM");
        defmt::assert!(!csc.elsb().bit(), "ELSB should be 0 for low-true");
        defmt::assert!(csc.elsa().bit(), "ELSA should be 1 for low-true");
    }
}
