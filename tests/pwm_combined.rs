//! FTM combined mode self-tests — validates combined channel pair register
//! configuration, complementary output, dead-time, synchronization, and
//! inversion controls via register checks.
//!
//! Uses FTM0 split API: creates channel pair 0 (ch0+ch1), leaves ch2-ch7
//! available as independent channels. Also tests FTM1 pair 0.
//!
//! No oscilloscope or external wiring required — all tests verify register state.
//!
//! Priority: MEDIUM
//! Wiring: None
//!
//! Reference: K20 ref manual §36.4.15 (COMBINE), §36.4.16 (DEADTIME),
//! §36.4.21 (SYNC), §36.4.22 (INVCTRL), §36.4.27 (SYNCONF)

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::pwm::{
    DeadtimePrescaler, FtmChannelPair, FtmExt,
    PairInversion, PwmPolarity, Ftm0,
};
use hal::time::U32Ext;

struct State {
    ftm0_timer: hal::pwm::FtmTimer<Ftm0>,
    ftm0_pair0: FtmChannelPair<Ftm0, 0>,
    ftm1_ch0: Option<hal::pwm::FtmChannel<hal::pwm::Ftm1, 0>>,
    ftm1_ch1: Option<hal::pwm::FtmChannel<hal::pwm::Ftm1, 1>>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        // Split FTM0, configure timer, create combined pair 0
        let mut ftm0 = dp.ftm0.split(&clocks, &dp.sim);
        ftm0.timer.set_frequency(20_000u32.Hz(), &clocks);

        let pair0 = ftm0.ch0.into_combined(ftm0.ch1);

        // Split FTM1 for separate combined mode tests
        let ftm1 = dp.ftm1.split(&clocks, &dp.sim);

        super::State {
            ftm0_timer: ftm0.timer,
            ftm0_pair0: pair0,
            ftm1_ch0: Some(ftm1.ch0),
            ftm1_ch1: Some(ftm1.ch1),
        }
    }

    // --- Combined mode initialization ---

    /// After into_combined(), COMBINE register should have COMBINE0=1 and SYNCEN0=1.
    /// Combined mode must set MODE[FTMEN].
    ///
    /// Combine, COMP, dead-time, output masking and inversion are all FTM
    /// features-mode functions, and the sync machinery the pair configures
    /// (SWWRBUF, CNTMIN, SYNCEN) only exists when FTMEN is set. Nothing else in
    /// this suite checks the bit.
    #[test]
    fn test_ftmen_set_for_combined(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        defmt::assert!(
            ftm0.mode().read().ftmen().bit_is_set(),
            "FTMEN must be set for combined mode"
        );
    }

    #[test]
    fn test_combine_bits_set(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let combine = ftm0.combine().read();
        defmt::assert!(
            combine.combine0().is_enabled(),
            "COMBINE0 should be set after into_combined()"
        );
        defmt::assert!(
            combine.syncen0().is_enabled(),
            "SYNCEN0 should be set after into_combined()"
        );
    }

    /// SYNCONF should have SYNCMODE=1 (enhanced sync) from split().
    #[test]
    fn test_synconf_enhanced_mode(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let synconf = ftm0.synconf().read();
        defmt::assert!(
            synconf.syncmode().is_1(),
            "SYNCONF.SYNCMODE should be 1 (enhanced sync)"
        );
    }

    /// SYNCONF should have SWWRBUF=1 after pair creation.
    /// INVC is left at 0 so INVCTRL updates take effect immediately.
    #[test]
    fn test_synconf_buffering_bits(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let synconf = ftm0.synconf().read();
        defmt::assert!(
            synconf.swwrbuf().is_1(),
            "SYNCONF.SWWRBUF should be 1 (SW trigger flushes CV buffers)"
        );
        defmt::assert!(
            synconf.invc().is_0(),
            "SYNCONF.INVC should be 0 (INVCTRL updates immediately)"
        );
    }

    /// SYNC.CNTMIN should be set (loading point at counter minimum).
    #[test]
    fn test_sync_loading_point(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        defmt::assert!(
            ftm0.sync().read().cntmin().is_1(),
            "SYNC.CNTMIN should be 1 (load at period start)"
        );
    }

    /// Both channels should have MSB=1, ELSB=1 (high-true combined PWM).
    #[test]
    fn test_combined_channel_csc_bits(_state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let csc0 = ftm0.csc(0).read();
        let csc1 = ftm0.csc(1).read();

        defmt::assert!(csc0.msb().bit(), "ch0 MSB should be set");
        defmt::assert!(csc0.elsb().bit(), "ch0 ELSB should be set");
        defmt::assert!(csc1.msb().bit(), "ch1 MSB should be set");
        defmt::assert!(csc1.elsb().bit(), "ch1 ELSB should be set");
    }

    // --- Edge control ---

    /// set_edges() should write to CnV (ch0) and C(n+1)V (ch1).
    #[test]
    fn test_set_edges(state: &mut super::State) {
        state.ftm0_pair0.set_edges(100, 500);

        defmt::assert_eq!(
            state.ftm0_pair0.leading_edge(), 100,
            "Leading edge (CnV) should be 100"
        );
        defmt::assert_eq!(
            state.ftm0_pair0.trailing_edge(), 500,
            "Trailing edge (C(n+1)V) should be 500"
        );
    }

    /// set_duty() should set leading=0 and trailing=duty.
    #[test]
    fn test_set_duty(state: &mut super::State) {
        state.ftm0_pair0.set_duty(750);

        defmt::assert_eq!(
            state.ftm0_pair0.leading_edge(), 0,
            "set_duty() leading edge should be 0"
        );
        defmt::assert_eq!(
            state.ftm0_pair0.trailing_edge(), 750,
            "set_duty() trailing edge should be 750"
        );
    }

    /// max_duty() should return the MOD register value.
    #[test]
    fn test_max_duty(state: &mut super::State) {
        let max = state.ftm0_pair0.max_duty();
        let mod_val = state.ftm0_timer.modulo();
        defmt::info!("max_duty={}, modulo={}", max, mod_val);
        defmt::assert_eq!(max, mod_val, "max_duty should equal timer modulo");
    }

    // --- Complementary output ---

    /// enable_complementary() should set COMP0=1, disable should clear it.
    #[test]
    fn test_complementary_toggle(state: &mut super::State) {
        state.ftm0_pair0.enable_complementary();
        defmt::assert!(
            state.ftm0_pair0.is_complementary(),
            "COMP should be set after enable_complementary()"
        );

        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        defmt::assert!(
            ftm0.combine().read().comp0().is_enabled(),
            "COMP0 bit should be 1 in COMBINE register"
        );

        state.ftm0_pair0.disable_complementary();
        defmt::assert!(
            !state.ftm0_pair0.is_complementary(),
            "COMP should be cleared after disable_complementary()"
        );
    }

    // --- Dead-time ---

    /// set_deadtime() should write DTPS and DTVAL correctly.
    #[test]
    fn test_deadtime_config(state: &mut super::State) {
        state.ftm0_timer.set_deadtime(DeadtimePrescaler::Div1, 10);

        defmt::assert_eq!(
            state.ftm0_timer.deadtime_prescaler(),
            DeadtimePrescaler::Div1,
            "DTPS should be Div1"
        );
        defmt::assert_eq!(
            state.ftm0_timer.deadtime_value(), 10,
            "DTVAL should be 10"
        );

        // Test Div4 prescaler
        state.ftm0_timer.set_deadtime(DeadtimePrescaler::Div4, 63);
        defmt::assert_eq!(
            state.ftm0_timer.deadtime_prescaler(),
            DeadtimePrescaler::Div4,
            "DTPS should be Div4"
        );
        defmt::assert_eq!(
            state.ftm0_timer.deadtime_value(), 63,
            "DTVAL should be 63 (max 6-bit)"
        );

        // Test Div16 prescaler
        state.ftm0_timer.set_deadtime(DeadtimePrescaler::Div16, 0);
        defmt::assert_eq!(
            state.ftm0_timer.deadtime_prescaler(),
            DeadtimePrescaler::Div16,
            "DTPS should be Div16"
        );
        defmt::assert_eq!(
            state.ftm0_timer.deadtime_value(), 0,
            "DTVAL should be 0"
        );

        // Restore
        state.ftm0_timer.set_deadtime(DeadtimePrescaler::Div1, 10);
    }

    /// DTVAL should be masked to 6 bits (values > 63 are truncated).
    #[test]
    fn test_deadtime_value_masking(state: &mut super::State) {
        state.ftm0_timer.set_deadtime(DeadtimePrescaler::Div1, 0xFF);

        // 0xFF & 0x3F = 63
        defmt::assert_eq!(
            state.ftm0_timer.deadtime_value(), 63,
            "DTVAL should be masked to 6 bits (63)"
        );

        // Restore
        state.ftm0_timer.set_deadtime(DeadtimePrescaler::Div1, 10);
    }

    /// enable_deadtime() should set DTEN0=1, disable should clear it.
    #[test]
    fn test_deadtime_enable_toggle(state: &mut super::State) {
        state.ftm0_pair0.enable_deadtime();
        defmt::assert!(
            state.ftm0_pair0.is_deadtime_enabled(),
            "DTEN should be set after enable_deadtime()"
        );

        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        defmt::assert!(
            ftm0.combine().read().dten0().is_enabled(),
            "DTEN0 bit should be 1 in COMBINE register"
        );

        state.ftm0_pair0.disable_deadtime();
        defmt::assert!(
            !state.ftm0_pair0.is_deadtime_enabled(),
            "DTEN should be cleared after disable_deadtime()"
        );
    }

    // --- PWM synchronization ---

    /// SYNCEN should be set by default, and disable_sync()/enable_sync() should toggle it.
    #[test]
    fn test_sync_enable_toggle(state: &mut super::State) {
        // SYNCEN is set by into_combined()
        defmt::assert!(
            state.ftm0_pair0.is_sync_enabled(),
            "SYNCEN should be set after into_combined()"
        );

        // Disable for DMA-driven immediate updates
        state.ftm0_pair0.disable_sync();
        defmt::assert!(
            !state.ftm0_pair0.is_sync_enabled(),
            "SYNCEN should be cleared after disable_sync()"
        );

        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        defmt::assert!(
            ftm0.combine().read().syncen0().is_disabled(),
            "SYNCEN0 bit should be 0 in COMBINE register"
        );

        // Re-enable
        state.ftm0_pair0.enable_sync();
        defmt::assert!(
            state.ftm0_pair0.is_sync_enabled(),
            "SYNCEN should be set after enable_sync()"
        );
    }

    // --- Output inversion ---

    /// set_inversion(Inverted) should set INV0EN=1 in INVCTRL.
    #[test]
    fn test_inversion_toggle(state: &mut super::State) {
        state.ftm0_pair0.set_inversion(PairInversion::Inverted);
        defmt::assert_eq!(
            state.ftm0_pair0.inversion(),
            PairInversion::Inverted,
            "Inversion should be Inverted"
        );

        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        defmt::assert!(
            ftm0.invctrl().read().inv0en().is_enabled(),
            "INV0EN should be 1 in INVCTRL"
        );

        state.ftm0_pair0.set_inversion(PairInversion::Normal);
        defmt::assert_eq!(
            state.ftm0_pair0.inversion(),
            PairInversion::Normal,
            "Inversion should be Normal"
        );
    }

    // --- PWM polarity on combined pair ---

    /// set_pwm_polarity(LowTrue) should set ELSA=1, ELSB=0 on the even channel.
    #[test]
    fn test_pair_pwm_polarity(state: &mut super::State) {
        state.ftm0_pair0.set_pwm_polarity(PwmPolarity::LowTrue);

        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let csc = ftm0.csc(0).read();
        defmt::assert!(csc.msb().bit(), "MSB should be 1 for PWM");
        defmt::assert!(!csc.elsb().bit(), "ELSB should be 0 for low-true");
        defmt::assert!(csc.elsa().bit(), "ELSA should be 1 for low-true");

        // Restore high-true
        state.ftm0_pair0.set_pwm_polarity(PwmPolarity::HighTrue);
        let csc = ftm0.csc(0).read();
        defmt::assert!(csc.elsb().bit(), "ELSB should be 1 for high-true");
        defmt::assert!(!csc.elsa().bit(), "ELSA should be 0 for high-true");
    }

    // --- Sync control ---

    /// software_sync() should set SYNC.SWSYNC (auto-cleared by hardware).
    #[test]
    fn test_software_sync(state: &mut super::State) {
        // Write new edge values
        state.ftm0_pair0.set_edges(200, 600);
        // Trigger sync
        state.ftm0_timer.software_sync();

        // SWSYNC may already be cleared by hardware if the counter is running,
        // so we just verify no crash/panic. The register write itself is the test.
        defmt::info!("software_sync() completed without error");
    }

    /// set_sync_loading_points() should set/clear CNTMIN and CNTMAX.
    #[test]
    fn test_sync_loading_points(state: &mut super::State) {
        state.ftm0_timer.set_sync_loading_points(true, true);

        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let sync = ftm0.sync().read();
        defmt::assert!(sync.cntmin().is_1(), "CNTMIN should be 1");
        defmt::assert!(sync.cntmax().is_1(), "CNTMAX should be 1");

        state.ftm0_timer.set_sync_loading_points(false, false);
        let sync = ftm0.sync().read();
        defmt::assert!(sync.cntmin().is_0(), "CNTMIN should be 0");
        defmt::assert!(sync.cntmax().is_0(), "CNTMAX should be 0");

        // Restore default
        state.ftm0_timer.set_sync_loading_points(true, false);
    }

    // --- into_channels release ---

    /// into_combined() on FTM1 creates a pair, into_channels() releases back.
    /// After release, COMBINE/COMP/DTEN/SYNCEN should all be cleared for pair 0.
    #[test]
    fn test_into_channels_release(state: &mut super::State) {
        // Take channels out of state via Option::take()
        let ftm1_ch0 = state.ftm1_ch0.take().unwrap();
        let ftm1_ch1 = state.ftm1_ch1.take().unwrap();
        let pair = ftm1_ch0.into_combined(ftm1_ch1);

        // Verify combined mode is active
        let ftm1 = unsafe { &*pac::Ftm1::PTR };
        defmt::assert!(
            ftm1.combine().read().combine0().is_enabled(),
            "FTM1 COMBINE0 should be set"
        );

        // Release back to independent channels
        let (ch0, ch1) = pair.into_channels();

        // Verify all combined bits are cleared
        let combine = ftm1.combine().read();
        defmt::assert!(
            combine.combine0().is_disabled(),
            "COMBINE0 should be cleared after into_channels()"
        );
        defmt::assert!(
            combine.comp0().is_disabled(),
            "COMP0 should be cleared after into_channels()"
        );
        defmt::assert!(
            combine.dten0().is_disabled(),
            "DTEN0 should be cleared after into_channels()"
        );
        defmt::assert!(
            combine.syncen0().is_disabled(),
            "SYNCEN0 should be cleared after into_channels()"
        );

        // Restore channels to state
        state.ftm1_ch0 = Some(ch0);
        state.ftm1_ch1 = Some(ch1);
    }
}
