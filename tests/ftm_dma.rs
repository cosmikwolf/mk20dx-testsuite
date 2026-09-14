//! FTM+DMA integration self-tests — validates that FTM channel events
//! trigger DMA transfers.
//!
//! Uses FTM0 (Output Compare ch0, PWM ch7) and FTM1 (PWM ch0) with
//! DMA channels 0-7. Each test uses a different DMA channel to avoid
//! reconfiguration.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use core::sync::atomic::{compiler_fence, Ordering};
use mk20dx_hal as hal;
use hal::dma::{DmaChannels, DmaExt, DmaSource, TransferConfig, TransferSize};
use hal::pac;
use hal::prelude::*;
use hal::pwm::{CompareAction, Ftm0Parts, FtmChannel, FtmTimer};
use embedded_hal::pwm::SetDutyCycle;

struct State {
    ftm0: Ftm0Parts,
    dma: DmaChannels,
    _ftm1_timer: FtmTimer<hal::pwm::Ftm1>,
    ftm1_ch0: FtmChannel<hal::pwm::Ftm1, 0>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        // Split FTM0: get timer handle + all channel handles
        let mut ftm0 = dp.ftm0.split(&clocks, &dp.sim);

        // Configure FTM0 for free-running with OC on ch0
        ftm0.timer.set_modulo(0xFFFF);
        ftm0.ch0.set_output_compare(CompareAction::Toggle, 100);
        ftm0.timer.start();

        // Split FTM1 and configure for 10 kHz PWM
        let mut ftm1 = dp.ftm1.split(&clocks, &dp.sim);
        ftm1.timer.set_frequency(10_000u32.Hz(), &clocks);
        ftm1.timer.start();
        ftm1.ch0.set_pwm();
        ftm1.ch0.set_duty_cycle(ftm1.ch0.max_duty_cycle() / 2).unwrap();

        // Split DMA channels
        let dma = dp.dma.split(dp.dmamux, &dp.sim);

        super::State {
            ftm0,
            dma,
            _ftm1_timer: ftm1.timer,
            ftm1_ch0: ftm1.ch0,
        }
    }

    /// Register-level sanity check: enable_dma sets DMA=1, CHIE=1;
    /// disable_dma clears them; MSA=1 is preserved throughout.
    #[test]
    fn test_enable_disable_dma_bits(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        // Verify MSA=1 from OC init
        let csc = ftm0.csc(0).read();
        defmt::assert!(csc.msa().bit(), "MSA should be 1 for output compare");

        // Enable DMA
        state.ftm0.ch0.enable_dma();
        let csc = ftm0.csc(0).read();
        defmt::assert!(csc.dma().bit(), "DMA bit should be set after enable_dma");
        defmt::assert!(csc.chie().bit(), "CHIE bit should be set after enable_dma");
        defmt::assert!(csc.msa().bit(), "MSA should be preserved after enable_dma");

        // Disable DMA
        state.ftm0.ch0.disable_dma();
        let csc = ftm0.csc(0).read();
        defmt::assert!(!csc.dma().bit(), "DMA bit should be cleared after disable_dma");
        defmt::assert!(!csc.chie().bit(), "CHIE bit should be cleared after disable_dma");
        defmt::assert!(csc.msa().bit(), "MSA should be preserved after disable_dma");
    }

    /// OC match triggers DMA to copy a sentinel value from source to dest.
    /// Uses DMA ch0 with FTM0_CH0 source.
    #[test]
    fn test_oc_dma_single_transfer(state: &mut super::State) {
        // Clean state
        state.ftm0.ch0.disable_dma();
        state.ftm0.ch0.clear_flag();

        let sentinel: u32 = 0xDEAD_BEEF;
        let mut dest: u32 = 0;

        // Configure DMA ch0: single 32-bit transfer from sentinel to dest
        unsafe {
            state.dma.ch0.configure(&TransferConfig {
                source_addr: &sentinel as *const u32 as u32,
                dest_addr: &mut dest as *mut u32 as u32,
                source_size: TransferSize::Bits32,
                dest_size: TransferSize::Bits32,
                source_offset: 0,
                dest_offset: 0,
                minor_loop_bytes: 4,
                major_loop_count: 1,
                source_last_adjust: 0,
                dest_last_adjust: 0,
                dest_modulo: 0,
                auto_disable: true,
            });
        }

        // Route DMAMUX to FTM0_CH0 and enable hardware requests
        state.dma.ch0.set_source(DmaSource::FTM0_CH0);
        state.dma.ch0.enable_request();

        // Reset FTM0 counter and set compare=100 for fast match
        state.ftm0.timer.reset_counter();
        state.ftm0.ch0.set_value(100);

        // Enable DMA on the OC channel (arms the FTM→DMA path)
        state.ftm0.ch0.enable_dma();

        // Poll for DMA completion with timeout
        let mut timeout = 1_000_000u32;
        while !state.dma.ch0.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Clean up
        state.ftm0.ch0.disable_dma();
        state.dma.ch0.disable_request();
        compiler_fence(Ordering::SeqCst);

        defmt::assert!(timeout > 0, "DMA transfer timed out");
        defmt::assert!(!state.dma.ch0.has_error(), "DMA should have no errors");
        defmt::assert_eq!(dest, 0xDEAD_BEEF, "DMA should have copied sentinel");

        state.dma.ch0.clear_done();
    }

    /// DMA writes to FTM0 C0V register on each match — canonical waveform
    /// generation pattern. Source buffer of 4 compare values; after completion
    /// C0V should hold the last written value.
    /// Uses DMA ch1 with FTM0_CH0 source.
    #[test]
    fn test_oc_dma_writes_compare_value(state: &mut super::State) {
        // Clean state
        state.ftm0.ch0.disable_dma();
        state.ftm0.ch0.clear_flag();

        // Buffer of compare values to write to C0V on each match
        let compare_values: [u32; 4] = [2000, 3000, 4000, 5000];

        // FTM0 C0V address: base 0x4003_8000 + 0x10 (CV(0))
        const FTM0_C0V: u32 = 0x4003_8010;

        // Configure DMA ch1: write buffer → FTM0_C0V (fixed dest)
        unsafe {
            state.dma.ch1.configure_peripheral_write(
                compare_values.as_ptr() as *const u8,
                FTM0_C0V,
                TransferSize::Bits32,
                4,
            );
        }

        // Route and enable
        state.dma.ch1.set_source(DmaSource::FTM0_CH0);
        state.dma.ch1.enable_request();

        // Use short MOD for fast repeated matches
        state.ftm0.timer.stop();
        state.ftm0.timer.set_modulo(0x00FF);
        state.ftm0.timer.reset_counter();
        state.ftm0.ch0.set_value(0x0080);
        state.ftm0.timer.start();

        // Arm FTM→DMA
        state.ftm0.ch0.enable_dma();

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch1.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Clean up
        state.ftm0.ch0.disable_dma();
        state.dma.ch1.disable_request();
        compiler_fence(Ordering::SeqCst);

        defmt::assert!(timeout > 0, "DMA transfer timed out");
        defmt::assert!(!state.dma.ch1.has_error(), "DMA should have no errors");

        // Read C0V — should be last written value (5000)
        let c0v = state.ftm0.ch0.value();
        defmt::info!("FTM0 C0V after DMA writes: {} (expected 5000)", c0v);
        defmt::assert_eq!(c0v, 5000, "C0V should be the last DMA-written value");

        // Restore MOD for subsequent tests
        state.ftm0.timer.stop();
        state.ftm0.timer.set_modulo(0xFFFF);
        state.ftm0.timer.start();
        state.dma.ch1.clear_done();
    }

    /// DMA reads FTM0 CNT register on each match — peripheral-to-memory
    /// capture pattern. After 4 captures, all values should be non-zero
    /// (counter was running).
    /// Uses DMA ch2 with FTM0_CH0 source.
    #[test]
    fn test_oc_dma_captures_counter(state: &mut super::State) {
        // Clean state
        state.ftm0.ch0.disable_dma();
        state.ftm0.ch0.clear_flag();

        let mut captures: [u32; 4] = [0; 4];

        // FTM0 CNT address: base 0x4003_8000 + 0x04
        const FTM0_CNT: u32 = 0x4003_8004;

        // Configure DMA ch2: FTM0_CNT (fixed source) → buffer
        unsafe {
            state.dma.ch2.configure_peripheral_read(
                FTM0_CNT,
                captures.as_mut_ptr() as *mut u8,
                TransferSize::Bits32,
                4,
            );
        }

        // Route and enable
        state.dma.ch2.set_source(DmaSource::FTM0_CH0);
        state.dma.ch2.enable_request();

        // Short MOD + compare for fast repeated matches
        state.ftm0.timer.stop();
        state.ftm0.timer.set_modulo(0x00FF);
        state.ftm0.timer.reset_counter();
        state.ftm0.ch0.set_value(0x0080);
        state.ftm0.timer.start();

        // Arm FTM→DMA
        state.ftm0.ch0.enable_dma();

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch2.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Clean up
        state.ftm0.ch0.disable_dma();
        state.dma.ch2.disable_request();
        compiler_fence(Ordering::SeqCst);

        defmt::assert!(timeout > 0, "DMA transfer timed out");
        defmt::assert!(!state.dma.ch2.has_error(), "DMA should have no errors");

        // All captured counter values should be non-zero (counter was running)
        defmt::info!(
            "Captured counter values: [{}, {}, {}, {}]",
            captures[0], captures[1], captures[2], captures[3]
        );
        for (i, &val) in captures.iter().enumerate() {
            defmt::assert!(val != 0, "Capture {} should be non-zero", i);
        }

        // Restore MOD
        state.ftm0.timer.stop();
        state.ftm0.timer.set_modulo(0xFFFF);
        state.ftm0.timer.start();
        state.dma.ch2.clear_done();
    }

    /// PWM channel match triggers DMA — proves FTM1_CH0 events route
    /// through DMAMUX to activate DMA transfers. DMA writes a sentinel
    /// sequence to a memory buffer (not the peripheral register) so we
    /// can verify the exact values without FTM double-buffering concerns.
    /// Uses DMA ch3 with FTM1_CH0 source.
    #[test]
    fn test_pwm_dma_duty_update(state: &mut super::State) {
        let ftm1_regs = unsafe { &*pac::Ftm1::PTR };
        let dma_regs = unsafe { &*pac::Dma::PTR };

        // Clean state: disable DMA on the channel, clear stale CHF
        state.ftm1_ch0.disable_dma();
        state.ftm1_ch0.clear_flag();

        // Source buffer (simulated duty values) and destination buffer
        let src: [u32; 4] = [100, 200, 300, 400];
        let mut dst: [u32; 4] = [0; 4];

        // Configure DMA ch3: src buffer → dst buffer, one u32 per activation,
        // 4 major loop iterations (one per PWM match event)
        unsafe {
            state.dma.ch3.configure(&TransferConfig {
                source_addr: src.as_ptr() as u32,
                dest_addr: dst.as_mut_ptr() as u32,
                source_size: TransferSize::Bits32,
                dest_size: TransferSize::Bits32,
                source_offset: 4,
                dest_offset: 4,
                minor_loop_bytes: 4,
                major_loop_count: 4,
                source_last_adjust: -16,
                dest_last_adjust: -16,
                dest_modulo: 0,
                auto_disable: true,
            });
        }

        // Route to FTM1_CH0 and enable hardware requests
        state.dma.ch3.set_source(DmaSource::FTM1_CH0);
        state.dma.ch3.enable_request();

        // Clear CHF then enable DMA on the PWM channel
        state.ftm1_ch0.clear_flag();
        state.ftm1_ch0.enable_dma();

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch3.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Capture state for diagnostics
        let citer = dma_regs.tcd(3).citer_elinkno().read().citer().bits();
        let has_err = state.dma.ch3.has_error();

        // Clean up
        state.ftm1_ch0.disable_dma();
        state.dma.ch3.disable_request();
        compiler_fence(Ordering::SeqCst);

        if timeout == 0 {
            defmt::info!(
                "TIMEOUT: CITER={} ERR={} C0SC=0x{:02x}",
                citer, has_err, ftm1_regs.csc(0).read().bits()
            );
        }

        defmt::assert!(timeout > 0, "DMA transfer timed out");
        defmt::assert!(!has_err, "DMA should have no errors");

        // Verify all 4 values were transferred by PWM match events
        defmt::info!(
            "PWM→DMA transferred: [{}, {}, {}, {}]",
            dst[0], dst[1], dst[2], dst[3]
        );
        defmt::assert_eq!(dst, [100, 200, 300, 400], "DMA should have copied all 4 values");

        state.dma.ch3.clear_done();
    }

    /// After DMA major loop completes with DREQ=1, verify:
    /// - CITER is reloaded from BITER (not stuck at 0)
    /// - ERQ is auto-cleared
    /// - DONE is set
    /// This catches the diagnostic pitfall where CITER==BITER is
    /// misinterpreted as "no transfers occurred."
    /// Uses DMA ch4 with FTM0_CH0 source.
    #[test]
    fn test_citer_reloads_after_completion(state: &mut super::State) {
        let dma_regs = unsafe { &*pac::Dma::PTR };

        // Clean state
        state.ftm0.ch0.disable_dma();
        state.ftm0.ch0.clear_flag();

        let src: [u32; 4] = [0xAA, 0xBB, 0xCC, 0xDD];
        let mut dst: [u32; 4] = [0; 4];

        unsafe {
            state.dma.ch4.configure(&TransferConfig {
                source_addr: src.as_ptr() as u32,
                dest_addr: dst.as_mut_ptr() as u32,
                source_size: TransferSize::Bits32,
                dest_size: TransferSize::Bits32,
                source_offset: 4,
                dest_offset: 4,
                minor_loop_bytes: 4,
                major_loop_count: 4,
                source_last_adjust: -16,
                dest_last_adjust: -16,
                dest_modulo: 0,
                auto_disable: true,
            });
        }

        state.dma.ch4.set_source(DmaSource::FTM0_CH0);
        state.dma.ch4.enable_request();

        // Short MOD for fast matches
        state.ftm0.timer.stop();
        state.ftm0.timer.set_modulo(0x00FF);
        state.ftm0.timer.reset_counter();
        state.ftm0.ch0.set_value(0x0080);
        state.ftm0.timer.start();
        state.ftm0.ch0.enable_dma();

        // Wait for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch4.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        state.ftm0.ch0.disable_dma();
        compiler_fence(Ordering::SeqCst);

        defmt::assert!(timeout > 0, "DMA transfer timed out");

        // --- Key assertions: post-completion register state ---

        // DONE should be set (major loop completed)
        let csr = dma_regs.tcd(4).csr().read();
        defmt::assert!(csr.done().bit_is_set(), "DONE should be set after completion");

        // CITER should have reloaded from BITER (both should be 4)
        let citer = dma_regs.tcd(4).citer_elinkno().read().citer().bits();
        let biter = dma_regs.tcd(4).biter_elinkno().read().biter().bits();
        defmt::info!("Post-completion: CITER={} BITER={}", citer, biter);
        defmt::assert_eq!(citer, biter, "CITER should reload from BITER after completion");
        defmt::assert_eq!(citer, 4, "CITER should be 4 (the original major_loop_count)");

        // ERQ should be auto-cleared (DREQ=1)
        let erq = dma_regs.erq().read().bits();
        defmt::assert_eq!(erq & (1 << 4), 0, "ERQ bit 4 should be cleared by DREQ");

        // Data should have actually transferred
        defmt::assert_eq!(dst, [0xAA, 0xBB, 0xCC, 0xDD], "Data should match source");

        // Restore
        state.ftm0.timer.stop();
        state.ftm0.timer.set_modulo(0xFFFF);
        state.ftm0.timer.start();
        state.dma.ch4.clear_done();
    }

    /// FTM0 CH7 in PWM mode triggers DMA — mirrors the SK6812 LED driver
    /// setup. Uses the split API to configure CH7 for PWM with a short
    /// period for fast DMA triggers.
    /// Uses DMA ch5 with FTM0_CH7 source.
    #[test]
    fn test_ftm0_ch7_pwm_dma(state: &mut super::State) {
        let ftm0_regs = unsafe { &*pac::Ftm0::PTR };
        let dma_regs = unsafe { &*pac::Dma::PTR };

        // ---- Re-initialize FTM0 for CH7 PWM via split API ----
        // Stop counter so MOD/CnV writes take effect immediately
        state.ftm0.timer.stop();

        // Set MOD for a short period (fast DMA triggers)
        state.ftm0.timer.set_modulo(0x00FF);

        // Configure CH7 for edge-aligned PWM
        state.ftm0.ch7.set_pwm();

        // Set CnV=1 (non-zero so CHF fires; CnV=0=CNTIN suppresses match)
        state.ftm0.ch7.set_value(1);

        // Reset counter
        state.ftm0.timer.reset_counter();

        // Verify writes took effect (should be immediate with counter stopped)
        let mod_val = state.ftm0.timer.modulo();
        let cv_val = state.ftm0.ch7.value();
        defmt::info!(
            "After init (counter stopped): MOD={} CnV={} (want 255, 1)",
            mod_val, cv_val
        );
        defmt::assert_eq!(mod_val, 0x00FF, "MOD should be 0xFF with counter stopped");
        defmt::assert_eq!(cv_val, 1, "CnV should be 1 with counter stopped");

        // Restart counter
        state.ftm0.timer.start();

        // Verify CHF fires before involving DMA
        state.ftm0.ch7.clear_flag();
        cortex_m::asm::delay(720); // ~10 µs = many periods at MOD=0xFF
        defmt::assert!(state.ftm0.ch7.has_flag(), "CH7 match flag should fire with CnV=1, MOD=0xFF");

        // ---- DMA transfer ----
        let src: [u16; 8] = [10, 21, 10, 21, 10, 21, 10, 1];
        let mut dst: [u16; 8] = [0; 8];

        // DMA ch5: src buffer → dst buffer, one u16 per FTM match, 8 iterations
        unsafe {
            state.dma.ch5.configure(&TransferConfig {
                source_addr: src.as_ptr() as u32,
                dest_addr: dst.as_mut_ptr() as u32,
                source_size: TransferSize::Bits16,
                dest_size: TransferSize::Bits16,
                source_offset: 2,
                dest_offset: 2,
                minor_loop_bytes: 2,
                major_loop_count: 8,
                source_last_adjust: -16,
                dest_last_adjust: -16,
                dest_modulo: 0,
                auto_disable: true,
            });
        }

        state.dma.ch5.set_source(DmaSource::FTM0_CH7);
        state.dma.ch5.enable_request();

        // Clear CHF then enable DMA on CH7
        state.ftm0.ch7.clear_flag();
        state.ftm0.ch7.enable_dma();

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch5.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Capture state for diagnostics
        let citer = dma_regs.tcd(5).citer_elinkno().read().citer().bits();
        let has_err = state.dma.ch5.has_error();

        // Clean up: disable DMA on CH7, restore FTM0 for OC tests
        state.ftm0.ch7.disable_dma();
        state.dma.ch5.disable_request();
        compiler_fence(Ordering::SeqCst);

        if timeout == 0 {
            defmt::info!(
                "TIMEOUT: CITER={} ERR={} C7SC=0x{:02x}",
                citer, has_err, ftm0_regs.csc(7).read().bits()
            );
        }

        defmt::assert!(timeout > 0, "FTM0_CH7 DMA transfer timed out");
        defmt::assert!(!has_err, "DMA should have no errors");

        defmt::info!(
            "FTM0_CH7 DMA transferred: [{}, {}, {}, {}, {}, {}, {}, {}]",
            dst[0], dst[1], dst[2], dst[3], dst[4], dst[5], dst[6], dst[7]
        );
        defmt::assert_eq!(
            dst,
            [10, 21, 10, 21, 10, 21, 10, 1],
            "CH7 DMA should have copied all 8 values"
        );

        // Restore MOD for any subsequent use
        state.ftm0.timer.stop();
        state.ftm0.timer.set_modulo(0xFFFF);
        state.ftm0.timer.start();
        state.dma.ch5.clear_done();
    }
}
