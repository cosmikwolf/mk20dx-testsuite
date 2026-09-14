//! DMA self-tests — validates eDMA engine with memory-to-memory transfers.
//!
//! All tests use memory-to-memory transfers — no external peripherals needed.
//!
//! Priority: MEDIUM
//! Wiring: None

#![no_std]
#![no_main]

extern crate alloc;

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();

use core::sync::atomic::{compiler_fence, Ordering};
use mk20dx_hal as hal;
use hal::dma::{DmaChannels, DmaExt, DmaSource, ScatterGatherTcd, TransferConfig, TransferSize, dchpri_index};
use hal::pac;
use hal::prelude::*;

struct State {
    dma: DmaChannels,
}

#[defmt_test::tests]
mod tests {
    use super::*;

    const HEAP_SIZE: usize = 8192;
    static mut HEAP_MEM: [u8; HEAP_SIZE] = [0; HEAP_SIZE];

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let _clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);

        // Initialize heap allocator for proptest
        unsafe { super::HEAP.init((&raw mut HEAP_MEM) as usize, HEAP_SIZE) }

        let dma = dp.dma.split(dp.dmamux, &dp.sim);
        super::State { dma }
    }

    /// Copy 4 bytes via DMA — verify destination matches source.
    #[test]
    fn test_memcpy_4_bytes(state: &mut super::State) {
        let src: [u8; 4] = [1, 2, 3, 4];
        let mut dst: [u8; 4] = [0; 4];

        unsafe {
            state.dma.ch0.configure_memcpy(src.as_ptr(), dst.as_mut_ptr(), 4);
        }
        state.dma.ch0.start();
        while !state.dma.ch0.is_complete() {}
        compiler_fence(Ordering::SeqCst);
        state.dma.ch0.clear_done();

        defmt::assert_eq!(dst, [1, 2, 3, 4], "DMA 4-byte copy should match");
    }

    /// Copy 256 bytes of patterned data via DMA.
    #[test]
    fn test_memcpy_256_bytes(state: &mut super::State) {
        let mut src = [0u8; 256];
        for (i, v) in src.iter_mut().enumerate() {
            *v = i as u8;
        }
        let mut dst = [0u8; 256];

        unsafe {
            state.dma.ch0.configure_memcpy(src.as_ptr(), dst.as_mut_ptr(), 256);
        }
        state.dma.ch0.start();
        while !state.dma.ch0.is_complete() {}
        compiler_fence(Ordering::SeqCst);
        state.dma.ch0.clear_done();

        for i in 0..256 {
            defmt::assert_eq!(dst[i], src[i], "Byte {} mismatch", i);
        }
    }

    /// Copy 1024 bytes via DMA.
    #[test]
    fn test_memcpy_1024_bytes(state: &mut super::State) {
        let mut src = [0u8; 1024];
        for (i, v) in src.iter_mut().enumerate() {
            *v = (i & 0xFF) as u8;
        }
        let mut dst = [0u8; 1024];

        unsafe {
            state.dma.ch0.configure_memcpy(src.as_ptr(), dst.as_mut_ptr(), 1024);
        }
        state.dma.ch0.start();
        while !state.dma.ch0.is_complete() {}
        compiler_fence(Ordering::SeqCst);
        state.dma.ch0.clear_done();

        for i in 0..1024 {
            defmt::assert_eq!(dst[i], src[i], "Byte {} mismatch", i);
        }
    }

    /// 4-byte aligned src/dst should use 32-bit transfers (SSIZE/DSIZE = Bits32).
    #[test]
    fn test_aligned_uses_32bit(state: &mut super::State) {
        #[repr(align(4))]
        struct Aligned([u8; 16]);

        let src = Aligned([0xAA; 16]);
        let mut dst = Aligned([0; 16]);

        unsafe {
            state.dma.ch0.configure_memcpy(src.0.as_ptr(), dst.0.as_mut_ptr(), 16);
        }

        // Read TCD ATTR to verify transfer size
        let dma = unsafe { &*pac::Dma::PTR };
        let attr = dma.tcd(0).attr().read();
        // Bits32 = 0b010 in SSIZE/DSIZE fields
        defmt::assert_eq!(attr.ssize().bits(), 0b010, "SSIZE should be Bits32 (0b010)");
        defmt::assert_eq!(attr.dsize().bits(), 0b010, "DSIZE should be Bits32 (0b010)");
    }

    /// Odd-aligned address should fall back to 8-bit transfers.
    #[test]
    fn test_unaligned_uses_8bit(state: &mut super::State) {
        let src = [0u8; 8];
        let mut dst = [0u8; 8];

        // Offset by 1 to make address unaligned
        unsafe {
            state.dma.ch0.configure_memcpy(
                src.as_ptr().add(1),
                dst.as_mut_ptr().add(1),
                3,
            );
        }

        let dma = unsafe { &*pac::Dma::PTR };
        let attr = dma.tcd(0).attr().read();
        // Bits8 = 0b000 in SSIZE/DSIZE fields
        defmt::assert_eq!(attr.ssize().bits(), 0b000, "SSIZE should be Bits8 (0b000)");
        defmt::assert_eq!(attr.dsize().bits(), 0b000, "DSIZE should be Bits8 (0b000)");
    }

    /// Before starting a transfer, is_complete() should be false and TCD
    /// registers should be properly configured.
    #[test]
    fn test_not_complete_before_start(state: &mut super::State) {
        let src = [1u8; 4];
        let mut dst = [0u8; 4];

        unsafe {
            state.dma.ch0.configure_memcpy(src.as_ptr(), dst.as_mut_ptr(), 4);
        }
        // Don't start — verify state
        defmt::assert!(
            !state.dma.ch0.is_complete(),
            "Should not be complete before start"
        );

        // Verify TCD was actually configured: NBYTES should be 4,
        // CITER should be 1 (one major loop iteration)
        let dma = unsafe { &*pac::Dma::PTR };
        let tcd = dma.tcd(0);
        let nbytes = tcd.nbytes_mlno().read().nbytes().bits();
        defmt::assert_eq!(nbytes, 4, "NBYTES should be 4 for 4-byte memcpy");

        let citer = tcd.citer_elinkno().read().citer().bits();
        defmt::assert_eq!(citer, 1, "CITER should be 1 for single major loop");

        // Verify source and destination addresses are set
        let saddr = tcd.saddr().read().saddr().bits();
        let daddr = tcd.daddr().read().daddr().bits();
        defmt::assert_eq!(saddr, src.as_ptr() as u32, "SADDR should point to source");
        defmt::assert_eq!(daddr, dst.as_mut_ptr() as u32, "DADDR should point to dest");
    }

    /// After configure + start + wait, is_complete() should be true.
    #[test]
    fn test_complete_after_transfer(state: &mut super::State) {
        let src = [1u8; 4];
        let mut dst = [0u8; 4];

        unsafe {
            state.dma.ch0.configure_memcpy(src.as_ptr(), dst.as_mut_ptr(), 4);
        }
        state.dma.ch0.start();
        while !state.dma.ch0.is_complete() {}

        defmt::assert!(
            state.dma.ch0.is_complete(),
            "Should be complete after transfer"
        );
        state.dma.ch0.clear_done();
    }

    /// clear_done() should reset the DONE flag.
    #[test]
    fn test_clear_done_flag(state: &mut super::State) {
        let src = [1u8; 4];
        let mut dst = [0u8; 4];

        unsafe {
            state.dma.ch0.configure_memcpy(src.as_ptr(), dst.as_mut_ptr(), 4);
        }
        state.dma.ch0.start();
        while !state.dma.ch0.is_complete() {}

        state.dma.ch0.clear_done();
        defmt::assert!(
            !state.dma.ch0.is_complete(),
            "Should not be complete after clear_done"
        );
    }

    /// Two channels performing independent memcpy should both succeed.
    #[test]
    fn test_multiple_channels(state: &mut super::State) {
        let src0 = [0xAAu8; 4];
        let mut dst0 = [0u8; 4];
        let src1 = [0x55u8; 4];
        let mut dst1 = [0u8; 4];

        unsafe {
            state.dma.ch0.configure_memcpy(src0.as_ptr(), dst0.as_mut_ptr(), 4);
            state.dma.ch1.configure_memcpy(src1.as_ptr(), dst1.as_mut_ptr(), 4);
        }
        state.dma.ch0.start();
        state.dma.ch1.start();

        while !state.dma.ch0.is_complete() {}
        while !state.dma.ch1.is_complete() {}
        compiler_fence(Ordering::SeqCst);

        state.dma.ch0.clear_done();
        state.dma.ch1.clear_done();

        defmt::assert_eq!(dst0, [0xAA; 4], "ch0 copy should match");
        defmt::assert_eq!(dst1, [0x55; 4], "ch1 copy should match");
    }

    /// Set DMAMUX source to ALWAYS_ON0, verify the CHCFG register.
    #[test]
    fn test_source_routing(state: &mut super::State) {
        state.dma.ch0.set_source(DmaSource::ALWAYS_ON0);

        let dmamux = unsafe { &*pac::Dmamux::PTR };
        let chcfg = dmamux.chcfg(0).read();
        defmt::assert_eq!(
            chcfg.source().bits(),
            54, // ALWAYS_ON0 = slot 54
            "CHCFG source should be ALWAYS_ON0 (54)"
        );
        defmt::assert!(chcfg.enbl().is_enabled(), "CHCFG ENBL should be set");
    }

    /// After set_source + disable_source, ENBL should be 0.
    #[test]
    fn test_disable_source(state: &mut super::State) {
        state.dma.ch0.set_source(DmaSource::ALWAYS_ON0);
        state.dma.ch0.disable_source();

        let dmamux = unsafe { &*pac::Dmamux::PTR };
        let chcfg = dmamux.chcfg(0).read();
        defmt::assert!(chcfg.enbl().is_disabled(), "CHCFG ENBL should be cleared after disable");
    }

    /// 32-bit transfer with misaligned source address should set the error flag.
    /// The eDMA engine validates alignment at transfer time — SSIZE=Bits32
    /// requires 4-byte aligned SADDR.
    #[test]
    fn test_error_on_bad_alignment(state: &mut super::State) {
        let src = [0u8; 8];
        let mut dst = [0u8; 8];

        // Deliberately configure 32-bit transfer with misaligned source (offset by 1)
        unsafe {
            state.dma.ch0.configure(&TransferConfig {
                source_addr: src.as_ptr().add(1) as u32,
                dest_addr: dst.as_mut_ptr() as u32,
                source_size: TransferSize::Bits32,
                dest_size: TransferSize::Bits32,
                source_offset: 4,
                dest_offset: 4,
                minor_loop_bytes: 4,
                major_loop_count: 1,
                source_last_adjust: -4,
                dest_last_adjust: -4,
                dest_modulo: 0,
                auto_disable: true,
            });
        }
        state.dma.ch0.start();

        // Brief spin to let the error register, then check
        cortex_m::asm::delay(100);
        defmt::assert!(
            state.dma.ch0.has_error(),
            "Misaligned 32-bit transfer should set error flag"
        );

        // Clear the error so it doesn't affect subsequent tests
        state.dma.ch0.clear_error();
    }

    // ----- Property-based tests -----

    /// attr_from_sizes should only set bits in SSIZE[10:8] and DSIZE[2:0] fields.
    /// Exhaustive test over all 16 combos of TransferSize.
    #[test]
    fn test_attr_from_sizes_bit_isolation(_state: &mut super::State) {
        let sizes = [
            TransferSize::Bits8,
            TransferSize::Bits16,
            TransferSize::Bits32,
            TransferSize::Burst16,
        ];

        for &src in &sizes {
            for &dst in &sizes {
                let attr = ScatterGatherTcd::attr_from_sizes(src, dst);
                // SSIZE is bits [10:8], DSIZE is bits [2:0]
                // Valid mask: 0x0707
                let invalid_bits = attr & !0x0707;
                defmt::assert_eq!(
                    invalid_bits, 0,
                    "attr_from_sizes set bits outside SSIZE/DSIZE: attr=0x{:04X}",
                    attr
                );
            }
        }
        defmt::info!("attr_from_sizes bit isolation: PASSED (16 combos)");
    }

    /// dchpri_index should produce a bijection within each 4-byte group.
    /// Channels 0..16 should map to unique indices.
    #[test]
    fn test_dchpri_index_bijection(_state: &mut super::State) {
        // Check all 16 channels produce unique indices
        let mut seen = [false; 16];
        for ch in 0u8..16 {
            let idx = dchpri_index(ch);
            defmt::assert!(
                idx < 16,
                "dchpri_index({}) = {} out of range",
                ch,
                idx
            );
            defmt::assert!(
                !seen[idx],
                "dchpri_index({}) = {} already used",
                ch,
                idx
            );
            seen[idx] = true;
        }

        // Within each 4-channel group, verify byte-swap pattern:
        // ch0→idx3, ch1→idx2, ch2→idx1, ch3→idx0
        for group in 0..4u8 {
            let base = group * 4;
            for offset in 0..4u8 {
                let ch = base + offset;
                let idx = dchpri_index(ch);
                let expected = (base + 3 - offset) as usize;
                defmt::assert_eq!(
                    idx,
                    expected,
                    "dchpri_index({}) = {}, expected {}",
                    ch,
                    idx,
                    expected
                );
            }
        }

        defmt::info!("dchpri_index bijection: PASSED (16 channels)");
    }
}
