#!/bin/bash

# Compile and flash the Arducam MEGA capture sketch to the LilyGo LoRa32.
# The Arducam_Mega library lives outside the sketch dir, so it must be passed
# with --library or the compile fails with "Arducam_Mega.h: No such file".

set -e

usage() {
    cat <<EOF
Usage: $(basename "$0") [SKETCH_ARG] [PORT]

Compile and flash the Arducam MEGA capture sketch to the LilyGo LoRa32.

Arguments:
  SKETCH_ARG   Extra argument passed to 'arduino-cli upload' (optional).
  PORT         Serial port to upload to (default: /dev/ttyACM0).

Options:
  -h, --help   Show this help message and exit.
EOF
}

case "$1" in
    -h|--help)
        usage
        exit 0
        ;;
esac

FQBN="esp32:esp32:esp32"


# This board enumerates as /dev/ttyACM* (CH9102 bridge), not /dev/ttyUSB*.
# Override by passing the port as the first argument.
PORT="${2:-/dev/ttyACM0}"

LIBRARY="$HOME/Arducam_Mega"

if [ ! -d "$LIBRARY" ]; then
    echo "Error: Arducam_Mega library not found at $LIBRARY" >&2
    echo "Clone it there, or set LIBRARY to its location." >&2
    exit 1
fi


echo "Compiling..."
arduino-cli compile --fqbn "$FQBN" --library "$LIBRARY" .

echo "Uploading to $PORT..."
arduino-cli upload -p "$PORT" --fqbn "$FQBN" $1

echo ""
echo "Upload complete!"

