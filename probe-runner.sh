#!/bin/bash
# Cargo runner: probe-rs, with a retry when the SWD link fails to come up.
#
# The Kinetis debug handshake is unreliable even with --connect-under-reset:
# roughly one binary in ten needs a second attempt. A failure to connect is not
# a test result, so retry it. A test that actually ran and failed is a result,
# so report it and stop.
#
# Override the attempt count with PROBE_RETRIES, and pick a specific probe with
# PROBE_SELECTOR (VID:PID:SERIAL) when more than one is plugged in.
#
# A secured chip is recovered automatically, once per invocation. FSEC at 0x40C
# left in its erased state (0xFF) secures the part — losing power or the cable
# partway through a flash is enough to do it. The only way back is a mass erase,
# which probe-rs gates behind --allow-erase-all, so plain retries can never
# clear it. Set PROBE_NO_UNSECURE=1 to turn the recovery off and fail instead.

set -o pipefail

ELF="$1"
ATTEMPTS="${PROBE_RETRIES:-5}"
CHIP="MK20DX256xxx7"
CHIP_DESC="../mk20dx-hal/resources/K20_Series.yaml"

PROBE_ARG=()
if [ -n "${PROBE_SELECTOR:-}" ]; then
    PROBE_ARG=(--probe "$PROBE_SELECTOR")
elif [ "$(probe-rs list 2>/dev/null | grep -c '^\[')" -gt 1 ]; then
    # probe-rs falls back to an interactive prompt here, which just fails under
    # cargo. Say what to do instead.
    echo "probe-runner: more than one probe attached and PROBE_SELECTOR is unset." >&2
    echo "              Set it to VID:PID:SERIAL from \`probe-rs list\`, in" >&2
    echo "              .cargo/config.toml [env] or the environment." >&2
    exit 1
fi

LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT

unsecured_once=0

# Mass erase, to clear a secured FSEC. Destructive by definition: it is the
# only documented way to unsecure a Kinetis, and it takes the flash with it.
unsecure() {
    echo "probe-runner: chip reports SECURED (FSEC at 0x40C is probably 0xFF)." >&2
    echo "              Mass erasing to unlock — this erases the flash." >&2
    echo "              Set PROBE_NO_UNSECURE=1 to fail instead of erasing." >&2
    # --connect-under-reset matters: a secured part is often reset-looping too,
    # and the erase times out unless nRST is held.
    probe-rs erase --chip "$CHIP" --chip-description-path "$CHIP_DESC" \
        "${PROBE_ARG[@]}" --allow-erase-all --connect-under-reset >&2
}

for attempt in $(seq 1 "$ATTEMPTS"); do
    probe-rs run --chip "$CHIP" --chip-description-path "$CHIP_DESC" \
        "${PROBE_ARG[@]}" --connect-under-reset "$ELF" 2>&1 | tee "$LOG"
    status=$?

    [ "$status" -eq 0 ] && exit 0

    # The firmware started, so this is a real test failure, not a bad connect.
    if grep -q 'running `' "$LOG"; then
        exit "$status"
    fi

    # Another process holds the probe. Retrying cannot help.
    if grep -q 'exclusive access' "$LOG"; then
        echo "probe-runner: the probe is held by another process." >&2
        echo "              Check for a leftover probe-rs from an interrupted run." >&2
        exit "$status"
    fi

    # A secured chip refuses every connect the same way, so retrying is futile
    # until the part is unlocked. Recover once, then let the loop carry on.
    if [ "$unsecured_once" -eq 0 ] && grep -qE 'is SECURED|erase_all' "$LOG"; then
        if [ -n "${PROBE_NO_UNSECURE:-}" ]; then
            echo "probe-runner: chip is secured and PROBE_NO_UNSECURE is set." >&2
            exit "$status"
        fi
        unsecured_once=1
        if unsecure; then
            echo "probe-runner: mass erase done, retrying" >&2
            continue
        fi
        echo "probe-runner: mass erase failed; recover by hand with" >&2
        echo "              probe-rs erase --chip $CHIP --allow-erase-all --connect-under-reset" >&2
        exit "$status"
    fi

    echo "probe-runner: no connection on attempt ${attempt}/${ATTEMPTS}, retrying" >&2
done

echo "probe-runner: gave up after ${ATTEMPTS} attempts" >&2
exit 1
