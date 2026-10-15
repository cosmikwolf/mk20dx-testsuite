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
//! Regression: in 0.2.1 the scan wrote SC3 = 0 and dropped the averaging
//! from `set_averaging()` (see `test_averaging_*`).
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
use hal::adc::{Adc, Adc1, AdcExt, Averaging, ScanConfig};
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

/// Run a 4-channel scan for 100 ms. Returns the results counted from DMA-A
/// CITER (sampled in a tight loop), PDB MOD, and ADC1 SC3 during the scan.
fn measure_rate(state: &mut State) -> (u32, u32, u32) {
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
    // SAFETY: read-only register access while the scan owns the peripherals.
    let pdb = unsafe { &*pac::Pdb0::PTR };
    let adc = unsafe { &*pac::Adc1::PTR };
    let sc3 = adc.sc3().read().bits();
    let t0 = cycles();
    let mut last = dma_a_citer();
    let mut count = 0u32;
    while cycles().wrapping_sub(t0) < 72_000 * 100 {
        let c = dma_a_citer();
        count += ((last + 4 - c) % 4) as u32;
        last = c;
    }
    let modulus = pdb.mod_().read().bits();
    drop(scan);
    defmt::info!("results in 100 ms: {}; PDB MOD={}, SC3=0x{:02x}", count, modulus, sc3);
    (count, modulus, sc3)
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
        let (count, modulus, sc3) = measure_rate(state);
        defmt::assert_eq!(modulus, MODULUS as u32, "PDB MOD did not load");
        defmt::assert_eq!(sc3 & 0b1111, 0, "no averaging, no ADCO expected");
        defmt::assert!(
            (900..=1100).contains(&count),
            "expected about 1000 results in 100 ms, got {}",
            count
        );
    }

    /// The scan keeps the averaging from `set_averaging()` (0.2.1 wrote
    /// SC3 = 0). 4 samples (about 50 µs) fit in the 100 µs PDB slot, so the
    /// rate and the slot map do not change.
    #[test]
    fn test_averaging_4_kept(state: &mut super::State) {
        state.adc.set_averaging(Averaging::Avg4);
        let (count, _, sc3) = measure_rate(state);
        defmt::assert_eq!(sc3 & 0b1111, 0b0100, "SC3 AVGE=1 AVGS=00 (4 samples), ADCO=0");
        defmt::assert!((900..=1100).contains(&count), "Avg4: got {} results in 100 ms", count);
        check_slots(state, &[VREFSL, VREFSL, VREFSL, VREFSH]);
        check_slots(state, &[VREFSL, VREFSH, VREFSL, VREFSL, VREFSL]);
        state.adc.set_averaging(Averaging::Disabled);
    }

    /// Proof that the hardware really averages during a scan: 32 samples
    /// (about 400 µs) do not fit in a 100 µs slot. The next trigger comes
    /// while the conversion runs, the PDB sets a sequence error, and the
    /// scan stops after the first result. A new scan must clear the error
    /// and run again (in 0.2.1 a sequence error blocked every later scan).
    #[test]
    fn test_averaging_32_overrun_then_restart(state: &mut super::State) {
        state.adc.set_averaging(Averaging::Avg32);
        let (count, _, sc3) = measure_rate(state);
        state.adc.set_averaging(Averaging::Disabled);
        defmt::assert_eq!(sc3 & 0b1111, 0b0111, "SC3 AVGE=1 AVGS=11 (32 samples)");
        defmt::assert!(count < 500, "Avg32 should not keep up with 10 kHz, got {}", count);
        let (count, _, _) = measure_rate(state);
        defmt::assert!(
            (900..=1100).contains(&count),
            "restart after a sequence error: got {} results in 100 ms",
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
