# /// script
# requires-python = ">=3.10"
# dependencies = ["mido", "python-rtmidi"]
# ///
"""Host side of tests/usb_irq.rs: wait for the mk20dx-usb-irq MIDI port, then
send four Note On messages (channel 1, notes 60..63, velocity 100).

    uv run host/usb_midi_send.py [--timeout SECONDS]
"""

import argparse
import sys
import time

import mido

PORT_NAME = "mk20dx-usb-irq"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=90.0)
    args = ap.parse_args()

    deadline = time.monotonic() + args.timeout

    def ports():
        return [n for n in mido.get_output_names() if PORT_NAME in n]

    # A halted core from the last run leaves its USB device enumerated, so a
    # port can be stale. Wait for it to go away (flash and reset) first.
    while ports() and time.monotonic() < deadline:
        time.sleep(0.2)

    while time.monotonic() < deadline:
        names = ports()
        if names:
            # The device reaches Configured before CoreMIDI opens the port;
            # give it a moment, then send.
            time.sleep(1.0)
            with mido.open_output(names[0]) as out:
                # Four packets: a driver that drops every second OUT packet
                # shows up as 2 of 4. The gap keeps them in separate packets.
                for note in range(60, 64):
                    out.send(mido.Message("note_on", channel=0, note=note, velocity=100))
                    time.sleep(0.05)
            print(f"sent 4 note_on to {names[0]!r}", flush=True)
            return 0
        time.sleep(0.2)
    print(f"no MIDI output port with {PORT_NAME!r} in {args.timeout} s", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
