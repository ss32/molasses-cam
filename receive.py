#!/usr/bin/env python3
"""LoRa image receiver that auto-selects its input.

Images can arrive over one of two links, depending on what's plugged in:

  * a LilyGo LoRa32 board (running receiver.ino) as a USB-serial device -- it
    prints every LoRa packet as a hex line; we bracket each image between the
    header and fingerprint packets and reassemble the JPEG (v2 with Reed-Solomon
    FEC, v1 blind-concatenation fallback); or
  * an RTL-SDR dongle -- we drive rtl_sdr, detect each transmission by RF power,
    and decode it to a timestamped PNG.

This script detects which device is present and runs the matching path with no
user input required:

    ./receive.py                       # auto-detect and run
    ./receive.py --port /dev/ttyACM1   # probe a different serial port
    ./receive.py --force sdr --gain 40 # force the SDR path
    ./receive.py --file rec.raw --rate 1.8e6   # replay a recording (SDR path)

The two paths live in lib/: lib.lora_serial (serial board + config link) and
lib.sdr (RTL-SDR demod), sharing lib.helpers for the wire protocol and reassembly.
Each path's heavy dependencies (opencv/pyserial for serial, scipy/PIL for SDR) are
imported only when that path actually runs, so a machine set up for just one link
doesn't need the other's libraries -- importing either path module for detection
stays lightweight.
"""
import argparse
import os
import sys

from lib import lora_serial, sdr


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    # Dispatch + serial-path options
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="serial port to probe for the LoRa board (default /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=lora_serial.BAUD,
                    help=f"serial baud rate for the LoRa board (default {lora_serial.BAUD})")
    ap.add_argument("--force", choices=("serial", "sdr"),
                    help="skip auto-detection and use this path")

    # SDR-path options (used only when the SDR path runs)
    ap.add_argument("--file", help="read IQ from this file instead of driving the SDR (replay/test)")
    ap.add_argument("--format", choices=list(sdr.FORMATS), default="fc32",
                    help="IQ format for --file / piped stdin (default fc32; SDR mode is always cu8)")
    ap.add_argument("--rate", type=float, default=1.8e6, help="sample rate (Hz)")
    ap.add_argument("--bw", type=float, default=None,
                    help="LoRa bandwidth Hz (default: decoder's BW); must match the transmitter")
    ap.add_argument("--drive-sdr", action="store_true",
                    help="on the SDR path, drive rtl_sdr even when stdin isn't a TTY (headless/service)")
    ap.add_argument("--freq", type=float, default=915e6, help="SDR centre frequency (Hz)")
    ap.add_argument("--gain", default="auto",
                    help="tuner gain in dB, or 'auto' for AGC (default auto)")
    ap.add_argument("--device", "-d", default=0, help="rtl_sdr device index (default 0)")
    ap.add_argument("--ppm", type=int, default=0, help="SDR frequency correction (ppm)")
    ap.add_argument("--outdir", default=".", help="base output dir (YYYYMMDD dirs go here)")
    ap.add_argument("--threshold-db", type=float, default=9.0,
                    help="power above noise floor (dB) that starts a burst")
    ap.add_argument("--gap", type=float, default=0.5,
                    help="silence (s) that ends a burst (must exceed inter-packet gaps)")
    ap.add_argument("--min-burst", type=float, default=0.4,
                    help="ignore active bursts shorter than this (s)")
    ap.add_argument("--max-burst", type=float, default=600,
                    help="force-close a burst after this long (s)")
    ap.add_argument("--realtime", action="store_true",
                    help="with --file, replay at real time so the progress bar is meaningful")
    ap.add_argument("--no-progress", action="store_true", help="disable the live progress bar")
    ap.add_argument("--workers", type=int, default=min(8, os.cpu_count() or 1),
                    help="parallel decode processes (1 = decode in-thread)")
    args = ap.parse_args()

    # 1) Explicit override.
    if args.force == "serial":
        print(f"forced serial path on {args.port}")
        return lora_serial.serial_main(args)
    if args.force == "sdr":
        print("forced SDR path")
        return sdr.sdr_main(args)

    # 2) SDR replay from a file needs no hardware.
    if args.file:
        print("SDR replay (--file) -> SDR path")
        return sdr.sdr_main(args)

    # 3) Auto-detect.
    have_serial, desc = lora_serial.serial_present(args.port)
    have_sdr = sdr.rtl_sdr_present()
    board = args.port + (f" ({desc})" if desc else "")

    if have_serial and have_sdr:
        print(f"Both devices detected: serial board at {board} AND an RTL-SDR.",
              file=sys.stderr)
        print("Refusing to guess -- choose one with: "
              "--force serial   or   --force sdr", file=sys.stderr)
        return 2
    if have_serial:
        print(f"serial board detected at {board} -> serial path")
        return lora_serial.serial_main(args)
    if have_sdr:
        print("RTL-SDR detected -> SDR path")
        return sdr.sdr_main(args)

    print(f"No receiver found: no serial board at {args.port} and no RTL-SDR.",
          file=sys.stderr)
    print("Attach a LoRa board (or pass --port), plug in an RTL-SDR, or use "
          "--file <recording> / --force {serial,sdr}.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
