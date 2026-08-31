//! FTM combined-mode loopback tests — measures complementary output and
//! dead-time at the pins.
//!
//! pwm_combined.rs checks that COMBINE, COMP and DEADTIME hold the right bits.
//! Nothing checked that they do anything. These tests capture both outputs of
//! combined pair 0 and measure the result in counter ticks.
//!
//! FTM0 pair 0 (CH0, CH1) drives the outputs, and pair 1 (CH2, CH3) captures
//! them. All four share one counter, so every edge lands in the same timebase.
//!
//! Dead-time is `DTPS x DTVAL` system-clock cycles (ref manual §36.4.16). With
//! the FTM prescaler and the dead-time prescaler both at Div16 the two cancel,
//! so the delay is exactly DTVAL counter ticks.
//!
//! REQUIRES: two wires.
//!   PTC1 (Teensy pin 22) → PTC3 (Teensy pin 9)
//!   PTC2 (Teensy pin 23) → PTC4 (Teensy pin 10)
//!
//! Priority: HIGH
//! Wiring: PTC1 → PTC3 and PTC2 → PTC4

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::pwm::{
    CaptureEdge, DeadtimePrescaler, Ftm0, Ftm0Parts, FtmChannel, FtmChannelPair, FtmExt,
    FtmTimer, Prescaler,
};

const MODULO: u16 = 3999;
const PERIOD: u32 = MODULO as u32 + 1;
const LEADING: u16 = 1000;
const TRAILING: u16 = 3000;

/// 63 counter ticks of dead-time, since the two Div16 prescalers cancel.
const DTVAL: u8 = 63;
const DEADTIME_TICKS: u32 = DTVAL as u32;

/// Captures in ftm_loopback came back exact, so this only needs to absorb the
/// dead-time counter's own synchronisation.
const TOLERANCE: u32 = 8;

struct State {
    timer: FtmTimer<Ftm0>,
    pair: FtmChannelPair<Ftm0, 0>,
    /// Captures the CH0 output, arriving on PTC3.
    cap_a: FtmChannel<Ftm0, 2>,
    /// Captures the CH1 output, arriving on PTC4.
    cap_b: FtmChannel<Ftm0, 3>,
}

fn capture<const CH: u8>(ch: &mut FtmChannel<Ftm0, CH>, edge: CaptureEdge) -> u16 {
    ch.set_input_capture(edge);
    ch.clear_flag();

    let mut timeout = 2_000_000u32;
    while !ch.has_flag() && timeout > 0 {
        timeout -= 1;
    }
    defmt::assert!(timeout > 0, "no edge captured — check both wires");
    ch.value()
}

fn forward_delta(from: u16, to: u16) -> u32 {
    let (from, to) = (from as u32, to as u32);
    if to >= from { to - from } else { to + PERIOD - from }
}

fn close_to(actual: u32, expected: u32) -> bool {
    let diff = if actual > expected { actual - expected } else { expected - actual };
    diff <= TOLERANCE || PERIOD - diff <= TOLERANCE
}

#[defmt_test::tests]
mod tests {
    use super::*;

    #[init]
    fn init() -> super::State {
        let dp = pac::Peripherals::take().unwrap();
        dp.wdog.disable();
        let clocks = dp.mcg.constrain().freeze(dp.osc, &dp.sim);
        let pins_c = dp.portc.split(dp.ptc, &dp.sim);

        // All four are ALT4: CH0, CH1, CH2, CH3 of FTM0.
        let _out_a = pins_c.pc1.into_alternate::<4>();
        let _out_b = pins_c.pc2.into_alternate::<4>();
        let _cap_a = pins_c.pc3.into_alternate::<4>();
        let _cap_b = pins_c.pc4.into_alternate::<4>();

        let Ftm0Parts { mut timer, ch0, ch1, ch2, ch3, .. } =
            dp.ftm0.split(&clocks, &dp.sim);
        // ch2 captures the CH0 output, ch3 captures the CH1 output.
        let (mut cap_a, mut cap_b) = (ch2, ch3);

        timer.set_prescaler(Prescaler::Div16);
        timer.set_modulo(MODULO);
        timer.set_deadtime(DeadtimePrescaler::Div16, DTVAL);

        let mut pair = ch0.into_combined(ch1);
        pair.set_edges(LEADING, TRAILING);
        pair.enable_complementary();
        pair.disable_deadtime();

        cap_a.set_input_capture(CaptureEdge::Rising);
        cap_b.set_input_capture(CaptureEdge::Rising);
        timer.start();

        super::State { timer, pair, cap_a, cap_b }
    }

    /// The combined pair's edges appear on the pin where set_edges put them.
    #[test]
    fn test_combined_edges_at_the_pin(state: &mut super::State) {
        state.pair.disable_deadtime();
        state.timer.software_sync();

        let rise = capture(&mut state.cap_a, CaptureEdge::Rising);
        let fall = capture(&mut state.cap_a, CaptureEdge::Falling);
        let high = forward_delta(rise, fall);

        defmt::info!("combined output: rise {}, fall {}, high {}", rise, fall, high);
        defmt::assert!(
            close_to(rise as u32, LEADING as u32),
            "rising edge {} should sit at the leading edge {}",
            rise, LEADING
        );
        defmt::assert!(
            close_to(high, (TRAILING - LEADING) as u32),
            "high time {} should be trailing - leading = {}",
            high, TRAILING - LEADING
        );
    }

    /// With COMP set and no dead-time, CH1 is the inverse of CH0.
    #[test]
    fn test_complementary_output_is_inverse(state: &mut super::State) {
        state.pair.disable_deadtime();
        state.timer.software_sync();

        let a_rise = capture(&mut state.cap_a, CaptureEdge::Rising);
        let b_fall = capture(&mut state.cap_b, CaptureEdge::Falling);
        let a_fall = capture(&mut state.cap_a, CaptureEdge::Falling);
        let b_rise = capture(&mut state.cap_b, CaptureEdge::Rising);

        defmt::info!(
            "A rise {} / B fall {}   |   A fall {} / B rise {}",
            a_rise, b_fall, a_fall, b_rise
        );
        defmt::assert!(
            close_to(a_rise as u32, b_fall as u32),
            "B should fall as A rises: {} vs {}", b_fall, a_rise
        );
        defmt::assert!(
            close_to(a_fall as u32, b_rise as u32),
            "B should rise as A falls: {} vs {}", b_rise, a_fall
        );
    }

    /// Clearing COMP stops B inverting A.
    ///
    /// Without this, test_complementary_output_is_inverse could pass on a chip
    /// that simply drove both pins from the same signal.
    #[test]
    fn test_complement_disabled_stops_inverting(state: &mut super::State) {
        state.pair.disable_deadtime();
        state.pair.disable_complementary();
        state.timer.software_sync();

        let a_rise = capture(&mut state.cap_a, CaptureEdge::Rising);
        let b_rise = capture(&mut state.cap_b, CaptureEdge::Rising);
        let a_fall = capture(&mut state.cap_a, CaptureEdge::Falling);
        defmt::info!("COMP off: A rise {}, B rise {}, A fall {}", a_rise, b_rise, a_fall);

        // Restore, since the other tests expect a complementary pair.
        state.pair.enable_complementary();
        state.timer.software_sync();

        defmt::assert!(
            !close_to(b_rise as u32, a_fall as u32),
            "B still inverts A with COMP cleared: B rose at {}, A fell at {}",
            b_rise, a_fall
        );
    }

    /// Dead-time delays the turn-on edge by DTVAL ticks.
    #[test]
    fn test_deadtime_delays_turn_on(state: &mut super::State) {
        state.pair.disable_deadtime();
        state.timer.software_sync();
        let without = capture(&mut state.cap_a, CaptureEdge::Rising);

        state.pair.enable_deadtime();
        state.timer.software_sync();
        let with = capture(&mut state.cap_a, CaptureEdge::Rising);

        let shift = forward_delta(without, with);
        defmt::info!(
            "rising edge {} -> {}, shifted {} ticks (expected {})",
            without, with, shift, DEADTIME_TICKS
        );
        defmt::assert!(
            close_to(shift, DEADTIME_TICKS),
            "dead-time shifted the turn-on by {} ticks, expected {}",
            shift, DEADTIME_TICKS
        );
    }

    /// Dead-time opens a gap in which neither output is driven high.
    ///
    /// This is the property dead-time exists for, and the one no register read
    /// can reach: B turns off, then A turns on DTVAL ticks later.
    #[test]
    fn test_deadtime_creates_non_overlap(state: &mut super::State) {
        state.pair.enable_deadtime();
        state.timer.software_sync();

        let b_fall = capture(&mut state.cap_b, CaptureEdge::Falling);
        let a_rise = capture(&mut state.cap_a, CaptureEdge::Rising);
        let gap = forward_delta(b_fall, a_rise);

        defmt::info!("B off at {}, A on at {}, gap {} ticks", b_fall, a_rise, gap);
        defmt::assert!(
            close_to(gap, DEADTIME_TICKS),
            "non-overlap gap was {} ticks, expected {}",
            gap, DEADTIME_TICKS
        );
    }

    /// Disabling dead-time closes the gap again.
    #[test]
    fn test_deadtime_disable_closes_the_gap(state: &mut super::State) {
        state.pair.enable_deadtime();
        state.timer.software_sync();
        let b_fall_on = capture(&mut state.cap_b, CaptureEdge::Falling);
        let a_rise_on = capture(&mut state.cap_a, CaptureEdge::Rising);
        let gap_enabled = forward_delta(b_fall_on, a_rise_on);

        state.pair.disable_deadtime();
        state.timer.software_sync();
        let b_fall_off = capture(&mut state.cap_b, CaptureEdge::Falling);
        let a_rise_off = capture(&mut state.cap_a, CaptureEdge::Rising);
        let gap_disabled = forward_delta(b_fall_off, a_rise_off);

        defmt::info!("gap enabled {} ticks, disabled {} ticks", gap_enabled, gap_disabled);
        defmt::assert!(
            close_to(gap_disabled, 0),
            "gap should close to 0 with dead-time off, measured {}",
            gap_disabled
        );
        defmt::assert!(
            gap_enabled > gap_disabled + TOLERANCE,
            "enabling dead-time did not widen the gap: {} vs {}",
            gap_enabled, gap_disabled
        );
    }
}
