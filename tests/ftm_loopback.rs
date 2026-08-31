//! FTM output loopback tests — measures the PWM waveform on a real pin.
//!
//! Every other FTM test infers the waveform from MOD and CnV. These observe it.
//! FTM0_CH0 drives the signal, FTM0_CH2 captures it, and because both channels
//! share one counter, a capture is directly comparable to the compare value
//! that produced it — in exact counter ticks, with no scope.
//!
//! In high-true EPWM (ELSB:ELSA = 1:0) the output is set at the counter reload
//! and cleared on the CnV match, so:
//!
//!   rising edge  -> captured near CNTIN (0)
//!   falling edge -> captured near CnV
//!
//! REQUIRES: Wire connecting PTC1 (Teensy pin 22) → PTC3 (Teensy pin 9).
//!
//! Priority: HIGH
//! Wiring: PTC1 (FTM0_CH0 out) → PTC3 (FTM0_CH2 capture)

#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

use mk20dx_hal as hal;
use hal::pac;
use hal::prelude::*;
use hal::pwm::{CaptureEdge, Ftm0, Ftm0Parts, FtmChannel, FtmExt, Prescaler};

/// Counter period in ticks. The counter runs at bus/16, so at a 36 MHz bus one
/// tick is about 444 ns and a period is roughly 1.8 ms.
const MODULO: u16 = 3999;
const PERIOD: u32 = MODULO as u32 + 1;

/// Capture latency plus the edge-to-sample delay. Measured deltas sit within a
/// few ticks; this leaves room without letting a real error through, since the
/// duty steps under test are hundreds of ticks apart.
const TOLERANCE: u32 = 12;

struct State {
    ftm0: Ftm0Parts,
}

/// Arm the capture channel for `edge`, wait for it, and return the counter
/// value the capture recorded.
fn capture(ch: &mut FtmChannel<Ftm0, 2>, edge: CaptureEdge) -> u16 {
    ch.set_input_capture(edge);
    ch.clear_flag();

    let mut timeout = 2_000_000u32;
    while !ch.has_flag() && timeout > 0 {
        timeout -= 1;
    }
    defmt::assert!(timeout > 0, "no edge captured — is PTC1 wired to PTC3?");
    ch.value()
}

/// Distance from `from` to `to` going forwards around the counter.
fn forward_delta(from: u16, to: u16) -> u32 {
    let from = from as u32;
    let to = to as u32;
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

        // ALT4 on both: PTC1 is FTM0_CH0, PTC3 is FTM0_CH2.
        let _out = pins_c.pc1.into_alternate::<4>();
        let _cap = pins_c.pc3.into_alternate::<4>();

        let mut ftm0 = dp.ftm0.split(&clocks, &dp.sim);
        ftm0.timer.set_prescaler(Prescaler::Div16);
        ftm0.timer.set_modulo(MODULO);
        ftm0.ch0.set_pwm();
        ftm0.ch0.set_value(PERIOD as u16 / 4);
        ftm0.ch2.set_input_capture(CaptureEdge::Rising);
        ftm0.timer.start();

        super::State { ftm0 }
    }

    /// There is a waveform on the pin at all.
    #[test]
    fn test_edges_are_present(state: &mut super::State) {
        let rise = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
        let fall = capture(&mut state.ftm0.ch2, CaptureEdge::Falling);
        defmt::info!("rising at {}, falling at {}", rise, fall);
        defmt::assert!(
            forward_delta(rise, fall) > 0,
            "rising and falling edges landed on the same tick"
        );
    }

    /// The output rises at the counter reload, so the capture sits near CNTIN.
    #[test]
    fn test_rising_edge_at_counter_reload(state: &mut super::State) {
        let rise = capture(&mut state.ftm0.ch2, CaptureEdge::Rising) as u32;
        defmt::info!("rising edge captured at tick {}", rise);
        defmt::assert!(
            close_to(rise, 0),
            "rising edge should land near CNTIN (0), captured at {}",
            rise
        );
    }

    /// The falling edge lands on the CnV match, for several duty values.
    ///
    /// This is the measurement no register read can make: it proves the compare
    /// value reached the output, not merely that it reached a register.
    #[test]
    fn test_falling_edge_tracks_duty(state: &mut super::State) {
        for &duty in [PERIOD / 4, PERIOD / 2, (PERIOD * 3) / 4].iter() {
            state.ftm0.ch0.set_value(duty as u16);
            // Let the new value reach the active register at the next reload.
            let _ = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
            let _ = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);

            let rise = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
            let fall = capture(&mut state.ftm0.ch2, CaptureEdge::Falling);
            let high = forward_delta(rise, fall);

            defmt::info!("duty {}: high for {} ticks", duty, high);
            defmt::assert!(
                close_to(high, duty),
                "high time {} does not match CnV {}",
                high,
                duty
            );
        }
    }

    /// Changing the duty moves the falling edge on the pin.
    ///
    /// The regression this pins: with FTMEN set on a single channel, CnV writes
    /// never reach the active register and this edge would never move.
    #[test]
    fn test_duty_change_moves_the_edge(state: &mut super::State) {
        state.ftm0.ch0.set_value((PERIOD / 4) as u16);
        let _ = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
        let _ = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
        let rise_a = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
        let fall_a = capture(&mut state.ftm0.ch2, CaptureEdge::Falling);
        let high_a = forward_delta(rise_a, fall_a);

        state.ftm0.ch0.set_value(((PERIOD * 3) / 4) as u16);
        let _ = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
        let _ = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
        let rise_b = capture(&mut state.ftm0.ch2, CaptureEdge::Rising);
        let fall_b = capture(&mut state.ftm0.ch2, CaptureEdge::Falling);
        let high_b = forward_delta(rise_b, fall_b);

        defmt::info!("high time went from {} to {} ticks", high_a, high_b);
        defmt::assert!(
            high_b > high_a + (PERIOD / 4),
            "duty change did not move the falling edge: {} then {}",
            high_a,
            high_b
        );
    }

    /// A 0% duty channel produces no rising edge.
    #[test]
    fn test_zero_duty_has_no_pulse(state: &mut super::State) {
        state.ftm0.ch0.set_value(0);
        // Cannot wait on an edge here — the point is that there are none. Give
        // the counter several periods to reload the new value and go quiet.
        cortex_m::asm::delay(400_000);

        state.ftm0.ch2.set_input_capture(CaptureEdge::Rising);
        state.ftm0.ch2.clear_flag();
        // More than one full period.
        cortex_m::asm::delay(400_000);
        let saw_edge = state.ftm0.ch2.has_flag();

        // Restore a duty the later tests can use.
        state.ftm0.ch0.set_value((PERIOD / 4) as u16);

        defmt::assert!(!saw_edge, "0% duty should produce no rising edge");
    }

    /// The counter is free-running, so successive captures advance.
    #[test]
    fn test_counter_is_running(state: &mut super::State) {
        let first = state.ftm0.timer.counter();
        cortex_m::asm::delay(10_000);
        let second = state.ftm0.timer.counter();
        defmt::assert!(first != second, "FTM counter is not advancing");
    }
}
