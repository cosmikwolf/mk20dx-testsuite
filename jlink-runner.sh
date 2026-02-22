#!/bin/sh
# Cargo runner script for J-Link on Kinetis MK20DX256 (Teensy 3.2)
# Usage: jlink-runner.sh <elf-path>

ELF="$1"
HEX="/tmp/mk20dx-flash.hex"

# Convert ELF to Intel HEX
arm-none-eabi-objcopy -O ihex "$ELF" "$HEX"

# Create J-Link command file
CMD="/tmp/mk20dx-jlink.cmd"
cat > "$CMD" << EOF
loadfile $HEX
r
g
exit
EOF

# Flash, reset, and run
JLinkExe -device MK20DX256xxx7 -if SWD -speed 1000 -AutoConnect 1 -CommandFile "$CMD"
