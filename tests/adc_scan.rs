//! ADC multi-channel scan (`start_multi_channel_scan`) self-tests.
//!
//! The scan uses PDB + DMA-A (ADC result to buffer) + DMA-B (next channel to
//! SC1A, link-triggered by DMA-A). Each test puts VREFSH (ch29, ~1023 at
//! 10-bit) in one slot and VREFSL (ch30, ~0) in all other slots, then reads
//! the results buffer for about 100 ms while the scan runs. A channel-to-slot
//! map that moves shows up as VREFSH in the wrong slot.
//!
//! Regression: in 0.2.1 DMA-A linked DMA-B on the minor loop only. The eDMA
//! suppresses that link on the minor loop that ends the major loop, so DMA-B
//! ran N-1 times per N results and the map moved one step every scan.
//!
//! Regression: in 0.2.1 the scan ignored the PDB modulus (see
//! `test_rate_follows_pdb_modulus`).
//!
//! Setup as on the FLXS1: ADC1, PDB0, DMA-A = ch4, DMA-B = ch5.
//!
//! Priority: HIGH
//! Wiring: None

#![no_std]
#![no_main]

use cortex_m_rt as _;
use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::adc::{Adc, Adc1, AdcExt, ScanConfig};
use hal::dma::{DmaChannel, DmaExt};
use hal::pac;
use hal::pdb::{Multiplier, Pdb, PdbExt, Prescaler};
use hal::prelude::*;

const VREFSH: u8 = 29;
const VREFSL: u8 = 30;

/// VREFSH must read above this at 10-bit, VREFSL below `LOW`.
const HIGH: u16 = 900;
const LOW: u16 = 100;

/// Reads of the whole buffer per test, 20 µs apart: about 100 ms, so about
/// 250 scans at 4 channels.
const READS: u32 = 5_000;

/// 36 MHz bus / 3600 = 10 kHz conversion rate.
const MODULUS: u16 = 3_600;

struct State {
    adc: Adc<Adc1>,
    pdb: Pdb,
    dma_a: DmaChannel<4>,
    dma_b: DmaChannel<5>,
}

/// DMA-A TCD CITER (16-bit at TCD + 0x16). It counts down once per result.
fn dma_a_citer() -> u16 {
    // SAFETY: a read-only volatile access to an eDMA TCD register.
    unsafe { core::ptr::read_volatile((0x4000_9000 + 4 * 0x20 + 0x16) as *const u16) }
}

/// Start the DWT cycle counter and return its value.
fn cycles() -> u32 {
    // SAFETY: DEMCR.TRCENA, DWT_CTRL.CYCCNTENA and DWT_CYCCNT; no other user.
    unsafe {
        let demcr = 0xE000_EDFC as *mut u32;
        demcr.write_volatile(demcr.read_volatile() | 1 << 24);
        let ctrl = 0xE000_1000 as *mut u32;
        ctrl.write_volatile(ctrl.read_volatile() | 1);
        (0xE000_1004 as *const u32).read_volatile()
    }
}

/// Run the scan over `channels` and check that each slot keeps its channel.
fn check_slots(state: &mut State, channels: &'static [u8]) {
    let n = channels.len();
    let config = ScanConfig {
        channels,
        modulus: MODULUS,
        prescaler: Prescaler::Div1,
        multiplier: Multiplier::Mult1,
    };
    // 0xBEEF is out of range at 10-bit, so a slot that DMA never writes shows.
    let mut results = [0xBEEFu16; 8];
    let mut mux_buf = [0u32; 8];

    let scan = state.adc.start_multi_channel_scan(
        &config,
        &mut results[..n],
        &mut mux_buf[..n],
        &mut state.dma_a,
        &mut state.dma_b,
        &mut state.pdb,
    );

    cortex_m::asm::delay(72_000 * 2); // 2 ms: several scans
    let mut wrong = [0u32; 8];
    let mut citer_moves = 0u32;
    let mut last_citer = dma_a_citer();
    let mut first_wrong: Option<(u32, usize, u16)> = None;
    let t0 = cycles();
    for r in 0..READS {
        for (slot, &ch) in channels.iter().enumerate() {
            let v = scan.read_latest(slot);
            let ok = if ch == VREFSH { v > HIGH && v <= 1023 } else { v < LOW };
            if !ok {
                wrong[slot] += 1;
                if first_wrong.is_none() {
                    first_wrong = Some((r, slot, v));
                }
            }
        }
        let c = dma_a_citer();
        if c != last_citer {
            citer_moves += 1;
            last_citer = c;
        }
        cortex_m::asm::delay(72 * 20); // 20 µs
    }
    let ms = cycles().wrapping_sub(t0) / 72_000;
    let last: [u16; 8] = core::array::from_fn(|i| if i < n { scan.read_latest(i) } else { 0 });
    drop(scan);

    defmt::info!(
        "channels {}: wrong reads per slot {}, last {}, CITER moves {} in {} ms",
        channels, &wrong[..n], &last[..n], citer_moves, ms
    );
    if let Some((r, slot, v)) = first_wrong {
        defmt::info!("first wrong: read {} slot {} value {}", r, slot, v);
    }
    // A scan that stops keeps the right values but no longer moves CITER.
    // Each sample sees at most one move, so this counts samples, not results.
    defmt::assert!(citer_moves > 50, "scan stopped: CITER moved {} times", citer_moves);
    defmt::assert!(
        wrong[..n].iter().all(|&w| w == 0),
        "channel-to-slot map moved: wrong reads per slot {}",
        &wrong[..n]
    );
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let mut adc = dp.adc1.adc(&clocks, &dp.sim);
        adc.calibrate().unwrap();
        let pdb = dp.pdb0.constrain(&dp.sim);
        let dma = dp.dma.split(dp.dmamux, &dp.sim);
        super::State { adc, pdb, dma_a: dma.ch4, dma_b: dma.ch5 }
    }

    /// The scan runs at the PDB rate: 10 kHz, so 1000 results in 100 ms.
    /// In 0.2.1 `load_ok()` came before `enable()`, so MOD stayed at its reset
    /// value 0xFFFF and every scan ran at 36 MHz / 65536 = 549 Hz.
    #[test]
    fn test_rate_follows_pdb_modulus(state: &mut super::State) {
        static CH: [u8; 4] = [VREFSH, VREFSL, VREFSL, VREFSL];
        let config = ScanConfig {
            channels: &CH,
            modulus: MODULUS,
            prescaler: Prescaler::Div1,
            multiplier: Multiplier::Mult1,
        };
        let mut results = [0u16; 4];
        let mut mux_buf = [0u32; 4];
        let scan = state.adc.start_multi_channel_scan(
            &config, &mut results, &mut mux_buf,
            &mut state.dma_a, &mut state.dma_b, &mut state.pdb,
        );
        let pdb = unsafe { &*pac::Pdb0::PTR };
        let t0 = cycles();
        let mut last = dma_a_citer();
        let mut count = 0u32;
        let mut cnt_min = u16::MAX;
        let mut cnt_max = 0u16;
        while cycles().wrapping_sub(t0) < 72_000 * 100 {
            let c = dma_a_citer();
            count += ((last + 4 - c) % 4) as u32;
            last = c;
            let k = pdb.cnt().read().bits() as u16;
            cnt_min = cnt_min.min(k);
            cnt_max = cnt_max.max(k);
        }
        let modulus = pdb.mod_().read().bits();
        drop(scan);
        defmt::info!(
            "results in 100 ms: {}; PDB MOD={} CNT {}..{}",
            count, modulus, cnt_min, cnt_max
        );
        defmt::assert_eq!(modulus, MODULUS as u32, "PDB MOD did not load");
        defmt::assert!(
            (900..=1100).contains(&count),
            "expected about 1000 results in 100 ms, got {}",
            count
        );
    }

    /// VREFSH in the first slot of 4.
    #[test]
    fn test_4ch_first_slot(state: &mut super::State) {
        check_slots(state, &[VREFSH, VREFSL, VREFSL, VREFSL]);
    }

    /// VREFSH in the last slot of 4: the result whose minor-loop link the
    /// eDMA suppresses.
    #[test]
    fn test_4ch_last_slot(state: &mut super::State) {
        check_slots(state, &[VREFSL, VREFSL, VREFSL, VREFSH]);
    }

    /// The smallest scan this path accepts.
    #[test]
    fn test_3ch_middle_slot(state: &mut super::State) {
        check_slots(state, &[VREFSL, VREFSH, VREFSL]);
    }

    /// An odd count above 4.
    #[test]
    fn test_5ch_middle_slot(state: &mut super::State) {
        check_slots(state, &[VREFSL, VREFSL, VREFSH, VREFSL, VREFSL]);
    }
}
