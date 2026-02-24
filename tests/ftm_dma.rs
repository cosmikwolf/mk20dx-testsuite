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

use defmt_rtt as _;
use panic_probe as _;

use core::sync::atomic::{compiler_fence, Ordering};
use mk20dx_hal as hal;
use hal::dma::{DmaChannels, DmaExt, DmaSource, TransferConfig, TransferSize};
use hal::pac;
use hal::prelude::*;
use hal::pwm::{CompareAction, Ftm0, Ftm1, OutputCompare, PwmChannel};
use embedded_hal::pwm::SetDutyCycle;

struct State {
    oc: OutputCompare<Ftm0, 0>,
    dma: DmaChannels,
    ftm1_ch0: PwmChannel<Ftm1, 0>,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        // Create OC on FTM0 CH0: Toggle action, compare=100
        let oc = OutputCompare::<Ftm0, 0>::new(
            CompareAction::Toggle,
            100,
            &clocks,
            &dp.sim,
        );

        // Set FTM0 MOD=0xFFFF (free-running) and reset CNT
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0xFFFF) });
        ftm0.cnt().write(|w| unsafe { w.count().bits(0) });

        // Create FTM1 PWM at 10 kHz
        let mut ftm1 = dp.ftm1.pwm(10_000u32.Hz(), &clocks, &dp.sim);
        ftm1.ch0.enable();
        ftm1.ch0.set_duty_cycle(ftm1.ch0.max_duty_cycle() / 2).unwrap();

        // Split DMA channels
        let dma = dp.dma.split(dp.dmamux, &dp.sim);

        super::State {
            oc,
            dma,
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
        state.oc.enable_dma();
        let csc = ftm0.csc(0).read();
        defmt::assert!(csc.dma().bit(), "DMA bit should be set after enable_dma");
        defmt::assert!(csc.chie().bit(), "CHIE bit should be set after enable_dma");
        defmt::assert!(csc.msa().bit(), "MSA should be preserved after enable_dma");

        // Disable DMA
        state.oc.disable_dma();
        let csc = ftm0.csc(0).read();
        defmt::assert!(!csc.dma().bit(), "DMA bit should be cleared after disable_dma");
        defmt::assert!(!csc.chie().bit(), "CHIE bit should be cleared after disable_dma");
        defmt::assert!(csc.msa().bit(), "MSA should be preserved after disable_dma");
    }

    /// OC match triggers DMA to copy a sentinel value from source to dest.
    /// Uses DMA ch0 with FTM0_CH0 source.
    #[test]
    fn test_oc_dma_single_transfer(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        // Clear any pending channel flag and disable DMA from prior test
        state.oc.disable_dma();
        state.oc.clear_flag();

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
            });
        }

        // Route DMAMUX to FTM0_CH0 and enable hardware requests
        state.dma.ch0.set_source(DmaSource::FTM0_CH0);
        state.dma.ch0.enable_request();

        // Reset FTM0 counter and set compare=100 for fast match
        ftm0.cnt().write(|w| unsafe { w.count().bits(0) });
        state.oc.set_compare(100);

        // Enable DMA on the OC channel (arms the FTM→DMA path)
        state.oc.enable_dma();

        // Poll for DMA completion with timeout
        let mut timeout = 1_000_000u32;
        while !state.dma.ch0.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Clean up
        state.oc.disable_dma();
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
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        // Clean state
        state.oc.disable_dma();
        state.oc.clear_flag();

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
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0x00FF) });
        ftm0.cnt().write(|w| unsafe { w.count().bits(0) });
        state.oc.set_compare(0x0080);

        // Arm FTM→DMA
        state.oc.enable_dma();

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch1.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Clean up
        state.oc.disable_dma();
        state.dma.ch1.disable_request();
        compiler_fence(Ordering::SeqCst);

        defmt::assert!(timeout > 0, "DMA transfer timed out");
        defmt::assert!(!state.dma.ch1.has_error(), "DMA should have no errors");

        // Read C0V — should be last written value (5000)
        let c0v = ftm0.cv(0).read().val().bits();
        defmt::info!("FTM0 C0V after DMA writes: {} (expected 5000)", c0v);
        defmt::assert_eq!(c0v, 5000, "C0V should be the last DMA-written value");

        // Restore MOD for subsequent tests
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0xFFFF) });
        state.dma.ch1.clear_done();
    }

    /// DMA reads FTM0 CNT register on each match — peripheral-to-memory
    /// capture pattern. After 4 captures, all values should be non-zero
    /// (counter was running).
    /// Uses DMA ch2 with FTM0_CH0 source.
    #[test]
    fn test_oc_dma_captures_counter(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };

        // Clean state
        state.oc.disable_dma();
        state.oc.clear_flag();

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
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0x00FF) });
        ftm0.cnt().write(|w| unsafe { w.count().bits(0) });
        state.oc.set_compare(0x0080);

        // Arm FTM→DMA
        state.oc.enable_dma();

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch2.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Clean up
        state.oc.disable_dma();
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
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0xFFFF) });
        state.dma.ch2.clear_done();
    }

    /// PWM channel match triggers DMA — proves FTM1_CH0 events route
    /// through DMAMUX to activate DMA transfers. DMA writes a sentinel
    /// sequence to a memory buffer (not the peripheral register) so we
    /// can verify the exact values without FTM double-buffering concerns.
    /// Uses DMA ch3 with FTM1_CH0 source.
    #[test]
    fn test_pwm_dma_duty_update(state: &mut super::State) {
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
            });
        }

        // Route to FTM1_CH0 and enable hardware requests
        state.dma.ch3.set_source(DmaSource::FTM1_CH0);
        state.dma.ch3.enable_request();

        // Enable DMA on the PWM channel (arms the FTM→DMA path)
        state.ftm1_ch0.enable_dma();

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch3.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Clean up
        state.ftm1_ch0.disable_dma();
        state.dma.ch3.disable_request();
        compiler_fence(Ordering::SeqCst);

        defmt::assert!(timeout > 0, "DMA transfer timed out");
        defmt::assert!(!state.dma.ch3.has_error(), "DMA should have no errors");

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
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let dma_regs = unsafe { &*pac::Dma::PTR };

        // Clean state
        state.oc.disable_dma();
        state.oc.clear_flag();

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
            });
        }

        state.dma.ch4.set_source(DmaSource::FTM0_CH0);
        state.dma.ch4.enable_request();

        // Short MOD for fast matches
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0x00FF) });
        ftm0.cnt().write(|w| unsafe { w.count().bits(0) });
        state.oc.set_compare(0x0080);
        state.oc.enable_dma();

        // Wait for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch4.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        state.oc.disable_dma();
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
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0xFFFF) });
        state.dma.ch4.clear_done();
    }

    /// FTM0 CH7 in PWM mode triggers DMA — mirrors the SK6812 LED driver
    /// setup. Properly re-initializes FTM0 for CH7 PWM by stopping the
    /// counter first (CLKS=00), which ensures MOD/CnV writes take effect
    /// immediately instead of going to a write buffer.
    /// Uses DMA ch5 with FTM0_CH7 source.
    #[test]
    fn test_ftm0_ch7_pwm_dma(state: &mut super::State) {
        let ftm0 = unsafe { &*pac::Ftm0::PTR };
        let dma_regs = unsafe { &*pac::Dma::PTR };

        // ---- Re-initialize FTM0 for CH7 PWM ----
        // Key: stop counter FIRST so MOD/CnV writes take effect immediately.
        // In legacy mode (FTMEN=0) with counter running, MOD/CnV writes go
        // to a write buffer and only transfer on counter rollover. With the
        // counter stopped (CLKS=00), writes go directly to the active register.

        // 1. Stop counter
        ftm0.sc().modify(|_, w| w.clks().none());

        // 2. Disable write protection
        ftm0.mode().modify(|_, w| w.wpdis()._1());

        // 3. Set MOD for a short period (fast DMA triggers)
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0x00FF) });

        // 4. Configure CH7 for edge-aligned PWM: MSB=1, ELSB=1
        ftm0.csc(7).write(|w| w.msb().set_bit().elsb().set_bit());

        // 5. Set CnV=1 (non-zero so CHF fires; CnV=0=CNTIN suppresses match)
        ftm0.cv(7).write(|w| unsafe { w.val().bits(1) });

        // 6. Reset counter
        ftm0.cnt().write(|w| unsafe { w.count().bits(0) });

        // Verify writes took effect (should be immediate with CLKS=00)
        let mod_val = ftm0.mod_().read().mod_().bits();
        let cv_val = ftm0.cv(7).read().val().bits();
        defmt::info!(
            "After init (counter stopped): MOD={} CnV={} (want 255, 1)",
            mod_val, cv_val
        );
        defmt::assert_eq!(mod_val, 0x00FF, "MOD should be 0xFF with counter stopped");
        defmt::assert_eq!(cv_val, 1, "CnV should be 1 with counter stopped");

        // 7. Restart counter
        ftm0.sc().modify(|_, w| w.clks().system());

        // Verify CHF fires before involving DMA
        // Clear CHF by read-then-write-0
        ftm0.csc(7).modify(|_, w| w);
        cortex_m::asm::delay(720); // ~10 µs = many periods at MOD=0xFF
        let chf = ftm0.csc(7).read().chf().is_1();
        defmt::info!("CHF after restart: {} (should be true)", chf);
        defmt::assert!(chf, "CH7 match flag should fire with CnV=1, MOD=0xFF");

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
            });
        }

        state.dma.ch5.set_source(DmaSource::FTM0_CH7);
        state.dma.ch5.enable_request();

        // Clear CHF then enable DMA on CH7 (DMA=1 + CHIE=1, preserve PWM mode)
        ftm0.csc(7).modify(|_, w| w);
        ftm0.csc(7).modify(|_, w| w.dma()._1().chie()._1());

        // Poll for completion
        let mut timeout = 1_000_000u32;
        while !state.dma.ch5.is_complete() && timeout > 0 {
            timeout -= 1;
        }

        // Capture state for diagnostics
        let citer = dma_regs.tcd(5).citer_elinkno().read().citer().bits();
        let has_err = state.dma.ch5.has_error();

        // Clean up: disable DMA on CH7, restore FTM0 for OC tests
        ftm0.csc(7).modify(|_, w| w.dma()._0().chie()._0());
        state.dma.ch5.disable_request();
        compiler_fence(Ordering::SeqCst);

        if timeout == 0 {
            defmt::info!(
                "TIMEOUT: CITER={} ERR={} C7SC=0x{:02x}",
                citer, has_err, ftm0.csc(7).read().bits()
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
        ftm0.sc().modify(|_, w| w.clks().none());
        ftm0.mod_().write(|w| unsafe { w.mod_().bits(0xFFFF) });
        ftm0.sc().modify(|_, w| w.clks().system());
        state.dma.ch5.clear_done();
    }
}
