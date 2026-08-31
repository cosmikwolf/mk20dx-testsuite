#!/bin/bash
# Cargo runner: probe-rs, with a retry when the SWD link fails to come up.
#
# The Kinetis debug handshake is unreliable even with --connect-under-reset:
# roughly one binary in ten needs a second attempt. A failure to connect is not
# a test result, so retry it. A test that actually ran and failed is a result,
# so report it and stop.
#
# Override the attempt count with PROBE_RETRIES.

set -o pipefail

ELF="$1"
ATTEMPTS="${PROBE_RETRIES:-5}"
CHIP="MK20DX256xxx7"
CHIP_DESC="../mk20dx-hal/resources/K20_Series.yaml"

LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT

for attempt in $(seq 1 "$ATTEMPTS"); do
    probe-rs run --chip "$CHIP" --chip-description-path "$CHIP_DESC" \
        --connect-under-reset "$ELF" 2>&1 | tee "$LOG"
    status=$?

    [ "$status" -eq 0 ] && exit 0

    # The firmware started, so this is a real test failure, not a bad connect.
    if grep -q 'running `' "$LOG"; then
        exit "$status"
    fi

    echo "probe-runner: no connection on attempt ${attempt}/${ATTEMPTS}, retrying" >&2
done

echo "probe-runner: gave up after ${ATTEMPTS} attempts" >&2
exit 1
