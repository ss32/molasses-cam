#!/usr/bin/env python3
"""Self-contained LoRa image receiver that auto-selects its input.

Images can arrive over one of two links, depending on what's plugged in:

  * a LilyGo LoRa32 board (running receiver.ino) as a USB-serial device -- it
    prints every LoRa packet as a hex line; we bracket each image between the
    header and fingerprint packets and reassemble the JPEG (v2 with Reed-Solomon
    FEC, v1 blind-concatenation fallback); or
  * an RTL-SDR dongle -- we drive rtl_sdr, detect each transmission by RF power,
    and decode it to a timestamped PNG.

This one script contains both code paths. It detects which device is present
and runs the matching path with no user input required:

    ./receive.py                       # auto-detect and run
    ./receive.py --port /dev/ttyACM1   # probe a different serial port
    ./receive.py --force sdr --gain 40 # force the SDR path
    ./receive.py --file rec.raw --rate 1.8e6   # replay a recording (SDR path)

Needs: numpy; the serial path also needs pyserial, opencv-python and rs_gf256;
the SDR path also needs scipy and rs_gf256 (and rtl_sdr on PATH). Each path's
heavy dependencies are imported only when that path actually runs, so a machine
set up for just one link doesn't need the other's libraries.
"""
import argparse
import os
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from concurrent.futures import ProcessPoolExecutor

import numpy as np

# ---------------------------------------------------------------------------
# Shared wire-protocol constants
# ---------------------------------------------------------------------------
HEADER = bytes([0x61, 0x79, 0x79, 0x79, 0x79])       # precedes every image
FINGERPRINT = bytes([0x6c, 0x6d, 0x61, 0x6f])        # closes a serial-bridged image

# USB IDs (vendor:product) of RTL2832U-based dongles commonly sold as RTL-SDRs.
# Matched against lsusb output; detection is non-invasive (never opens the
# device, so it can't collide with rtl_sdr grabbing it).
RTL_SDR_USB_IDS = (
    "0bda:2832",   # Realtek RTL2832U (generic DVB-T)
    "0bda:2838",   # RTL2832U + R820T/R820T2 (RTL-SDR Blog v3, NooElec, etc.)
    "1d50:60a1",   # OpenMoko-assigned RTL-SDR
    "0ccd:00b4",   # TerraTec T Stick
)


# ===========================================================================
# Serial path -- read hex packet lines from the LoRa board and rebuild JPEGs
# (ported from monitor_serial.py; protocol v2 with FEC, v1 fallback)
# ===========================================================================
BAUD = 115200                                # must match receiver.ino's Serial.begin()

LORA_PAYLOAD = 250
V2_PAYLOAD = 246                              # 250 - 2 seq - 2 crc
PARAMS_MAGIC = bytes([0x70, 0x32])
PARAMS_LEN = 15

# --- Two-way config link (serial path only; see sender/sender.ino for the mirror) ---
# 32-bit request word, LSB->MSB: MODE bit0 (0=continuous, 1=specific count),
# RESOLUTION bits1-4, DELAY bits5-19 (s), COUNT bits20-31. Request/ACK are 8-byte
# packets [MAGIC:2][value:4 LE][crc16:2]; the receiver node relays them verbatim.
REQ_MAGIC = bytes([0x72, 0x71])
ACK_MAGIC = bytes([0x72, 0x63])
# Resolution index -> CAM_IMAGE_MODE name; order must match sender.ino's RES_TABLE.
RES_NAMES = ["96X96", "128X128", "QQVGA", "QVGA", "320X320", "VGA", "SVGA",
             "1024X768", "HD", "1280X1024", "UXGA", "FHD", "QXGA", "WQXGA2"]
# Pixel dimensions for the named modes (the ones whose name isn't already the size),
# shown in the menu as "QVGA (320x240)". Modes like 96X96 are their own dimensions.
RES_DIMS = {"QQVGA": "160x120", "QVGA": "320x240", "VGA": "640x480", "SVGA": "800x600",
            "HD": "1280x720", "UXGA": "1600x1200", "FHD": "1920x1080",
            "QXGA": "2048x1536", "WQXGA2": "2592x1944"}
RES_DEFAULT = 8                               # HD


def pack_config(mode, res, delay, count):
    return ((mode & 0x1) | ((res & 0xF) << 1) | ((delay & 0x7FFF) << 5)
            | ((count & 0xFFF) << 20)) & 0xFFFFFFFF


def expected_ack(cfg):
    """ack = cfg - DELAY*COUNT + RESOLUTION (32-bit wrap), matching sender.ino's sendAck."""
    res = (cfg >> 1) & 0xF
    delay = (cfg >> 5) & 0x7FFF
    count = (cfg >> 20) & 0xFFF
    return (cfg - delay * count + res) & 0xFFFFFFFF


def build_request(cfg):
    body = REQ_MAGIC + cfg.to_bytes(4, "little")
    return body + crc16_ccitt(body).to_bytes(2, "little")


def parse_ack(pkt):
    """Return the ACK value if pkt is a valid ACK packet, else None."""
    if len(pkt) == 8 and pkt[:2] == ACK_MAGIC \
       and crc16_ccitt(pkt[:6]) == (pkt[6] | (pkt[7] << 8)):
        return int.from_bytes(pkt[2:6], "little")
    return None


def crc16_ccitt(data):
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Matches the ESP crc16()."""
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
    return c


def _find_params(packets):
    for d in packets:
        if len(d) >= PARAMS_LEN and d[:2] == PARAMS_MAGIC \
           and crc16_ccitt(d[:13]) == (d[13] | (d[14] << 8)):
            return dict(image_len=int.from_bytes(d[3:7], "little"),
                        Ndata=d[7] | (d[8] << 8),
                        K=d[9] | (d[10] << 8),
                        M=d[11] | (d[12] << 8))
    return None


def reassemble_v2(packets, params):
    """Reassemble a v2 image from the raw packets collected for one burst.
    CRC-checks each 250-byte packet, places data by seq, RS-recovers up to M
    missing/corrupt packets per block. Returns (jpeg_bytes, stats)."""
    Ndata, K, M, image_len = params["Ndata"], params["K"], params["M"], params["image_len"]
    data_rows, parity_rows, crc_fail = {}, {}, 0
    for d in packets:
        if len(d) != LORA_PAYLOAD:
            continue
        body, crc = d[:248], d[248] | (d[249] << 8)
        if crc16_ccitt(body) != crc:
            crc_fail += 1
            continue
        seq = body[0] | (body[1] << 8)
        payload = body[2:2 + V2_PAYLOAD]
        (data_rows if seq < Ndata else parity_rows)[seq] = payload

    out = bytearray(Ndata * V2_PAYLOAD)
    recovered = unrecoverable = present = 0
    nblocks = (Ndata + K - 1) // K if K else 0
    for b in range(nblocks):
        lo = b * K
        k = min(K, Ndata - lo)
        have_data = {j: data_rows[lo + j] for j in range(k) if (lo + j) in data_rows}
        have_par = {i: parity_rows[Ndata + b * M + i]
                    for i in range(M) if (Ndata + b * M + i) in parity_rows}
        present += len(have_data)
        missing = [j for j in range(k) if j not in have_data]
        if not missing:
            for j in range(k):
                out[(lo + j) * V2_PAYLOAD:(lo + j + 1) * V2_PAYLOAD] = have_data[j]
            continue
        if len(have_data) + len(have_par) >= k and len(missing) <= M:
            recv = np.zeros((k + M, V2_PAYLOAD), dtype=np.uint8)
            erased = []
            for j in range(k):
                if j in have_data:
                    recv[j] = np.frombuffer(have_data[j], dtype=np.uint8)
                else:
                    erased.append(j)
            for i in range(M):
                if i in have_par:
                    recv[k + i] = np.frombuffer(have_par[i], dtype=np.uint8)
                else:
                    erased.append(k + i)
            try:
                dec = rs.decode(recv, k, M, erased=erased)
                for j in range(k):
                    out[(lo + j) * V2_PAYLOAD:(lo + j + 1) * V2_PAYLOAD] = dec[j].tobytes()
                recovered += len(missing)
                continue
            except Exception:
                pass
        unrecoverable += len(missing)
        for j, row in have_data.items():
            out[(lo + j) * V2_PAYLOAD:(lo + j + 1) * V2_PAYLOAD] = row

    stats = dict(v2=True, Ndata=Ndata, present=present, recovered=recovered,
                 unrecoverable=unrecoverable, crc_fail=crc_fail)
    return bytes(out[:image_len]), stats


def reassemble(packets):
    """Reassemble one burst's packets -> (jpeg_bytes_or_None, stats). Auto-detects v2
    (params packet present) vs legacy v1 (blind concatenation)."""
    params = _find_params(packets)
    if params is not None:
        return reassemble_v2(packets, params)
    # v1 fallback: concatenate raw payloads and slice SOI..EOI
    blob = b"".join(packets)
    soi, eoi = blob.find(b"\xff\xd8"), blob.rfind(b"\xff\xd9")
    jpg = blob[soi:eoi + 2] if (soi != -1 and eoi > soi) else None
    return jpg, dict(v2=False, packets=len(packets))


def show_and_save(jpg, stats, outdir="."):
    if not jpg:
        print(f"  no image ({stats}) -- skipped")
        return
    arr = np.frombuffer(jpg, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        print(f"  JPEG undecodable ({stats}) -- skipped")
        return
    # Same layout as the SDR path: <outdir>/<YYYYMMDD UTC>/<unix>.png, new dir per UTC day.
    now = int(time.time())
    day = os.path.join(outdir, time.strftime("%Y%m%d", time.gmtime(now)))
    os.makedirs(day, exist_ok=True)
    path = os.path.join(day, f"{now}.png")
    cv2.imwrite(path, img)
    if stats.get("v2"):
        print(f"  saved {img.shape[1]}x{img.shape[0]} -> {path}  "
              f"[v2 Ndata={stats['Ndata']} present={stats['present']} "
              f"recovered={stats['recovered']} unrecoverable={stats['unrecoverable']} "
              f"crc_fail={stats['crc_fail']}]")
    else:
        print(f"  saved {img.shape[1]}x{img.shape[0]} -> {path}  [v1 {stats['packets']} pkts]")


def _ask_int(label, lo, hi, default):
    while True:
        s = input(f"{label} [{lo}-{hi}, default {default}]: ").strip()
        if not s:
            return default
        try:
            v = int(s)
        except ValueError:
            print("  not a number"); continue
        if lo <= v <= hi:
            return v
        print(f"  out of range {lo}-{hi}")


def prompt_config():
    """Interactively build a 32-bit config word, or return None to just listen passively."""
    if input("\nSend a new configuration? [y/N] ").strip().lower() not in ("y", "yes"):
        return None
    print("Mode:  0) continuous   1) specific count")
    mode = _ask_int("Mode", 0, 1, 0)
    print("Resolution:")
    for i, name in enumerate(RES_NAMES):
        dims = RES_DIMS.get(name)
        print(f"  {i:2d}) {name}" + (f" ({dims})" if dims else ""))
    res = _ask_int("Resolution index", 0, len(RES_NAMES) - 1, RES_DEFAULT)
    delay = _ask_int("Delay between frames (s)", 0, 32767, 5)
    count = _ask_int("Count (images)", 0, 4095, 1) if mode == 1 else 0
    cfg = pack_config(mode, res, delay, count)
    print(f"  -> config 0x{cfg:08X}  (mode={'continuous' if mode == 0 else 'count'}, "
          f"res={RES_NAMES[res]}, delay={delay}s, count={count})")
    return cfg


def wait_for_ack(ser, expected, timeout):
    """Scan forwarded hex lines up to `timeout` s for a valid ACK; True if it matches."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().strip()
        if not line:
            continue
        try:
            pkt = bytes.fromhex(line.decode("ascii", "ignore"))
        except ValueError:
            continue
        ack = parse_ack(pkt)
        if ack is None:
            continue
        if ack == expected:
            return True
        print(f"  ACK mismatch: got 0x{ack:08X}, expected 0x{expected:08X}")
    return False


def send_request(ser, cfg, budget=120.0):
    """Push the request out through the receiver node and wait for the sender's ACK.
    Resends periodically for up to `budget` s -- the sender only hears us during its
    inter-frame listen window, so a request sent mid-image must be retried."""
    expected = expected_ack(cfg)
    cmd = b"TX" + build_request(cfg).hex().upper().encode() + b"\n"
    deadline = time.time() + budget
    attempt = 0
    while time.time() < deadline:
        attempt += 1
        ser.reset_input_buffer()          # drop any in-flight image bytes before the ACK
        ser.write(cmd)
        print(f"  request sent (attempt {attempt}); waiting for ACK...")
        if wait_for_ack(ser, expected, timeout=6.0):
            print("  ACK OK")
            return True
    print("  no ACK -- sender offline or never reached a listen window; back to menu")
    return False


def _params_total(pkt):
    """If pkt is the v2 params packet, return the total data+parity packet count the
    burst will send (same arithmetic as receiver.ino); else None."""
    if len(pkt) >= PARAMS_LEN and pkt[:2] == PARAMS_MAGIC \
       and crc16_ccitt(pkt[:13]) == (pkt[13] | (pkt[14] << 8)):
        Ndata = pkt[7] | (pkt[8] << 8)
        K = pkt[9] | (pkt[10] << 8)
        M = pkt[11] | (pkt[12] << 8)
        nblocks = (Ndata + K - 1) // K if K else 0
        return Ndata + nblocks * M
    return None


def render_serial_progress(recv, total):
    """In-place progress bar on stderr for the image currently arriving over serial.
    Unlike the SDR bar this is exact -- we count real packets against the params total."""
    w = 24
    if total:
        frac = min(1.0, recv / total)
        fill = int(frac * w)
        bar = "#" * fill + "." * (w - fill)
        sys.stderr.write(f"\r  receiving [{bar}] {frac * 100:3.0f}%  {recv}/{total} pkts   ")
    else:
        sys.stderr.write(f"\r  receiving [{'.' * w}]  {recv} pkts (reading params)   ")
    sys.stderr.flush()


def receive_images(ser, n=None, idle_finalize=3.0, outdir=".", progress=True):
    """Collect header..fingerprint bursts and save each. Stop after n images (n=None:
    forever). Same reassembly as before, refactored so a config run can bound it to n.
    If the burst's single (unprotected) fingerprint packet is lost, finalize the image
    after `idle_finalize` s of silence -- intra-image gaps are ~10 ms, so this can't fire
    mid-image, and it prevents a count run from hanging on a dropped trailing fingerprint.
    When `progress`, a live packet bar is drawn on stderr for each incoming image."""
    collecting, packets, got, last_rx = False, [], 0, time.time()
    recv = 0            # data/parity (250-byte) packets seen this image
    total = None        # expected data+parity count, learned from the params packet

    def finalize(msg):
        nonlocal got
        if progress:
            render_serial_progress(recv, total)
            sys.stderr.write("\n")          # leave the completed bar on its own line
            sys.stderr.flush()
        if msg:
            print(msg)
        show_and_save(*reassemble(packets), outdir=outdir)
        got += 1

    while n is None or got < n:
        line = ser.readline().strip()
        if not line:
            if collecting and packets and time.time() - last_rx > idle_finalize:
                finalize("  (silence after last packet -- finalizing image)")
                collecting, packets = False, []
            continue
        try:
            pkt = bytes.fromhex(line.decode("ascii", "ignore"))       # case-insensitive
        except ValueError:
            continue                                                  # boot/log line, skip
        last_rx = time.time()
        if pkt[:5] == HEADER:
            # New image starting. If the previous one never got its (single, unprotected)
            # fingerprint packet, finalize it now -- with v2 the params+data+parity suffice.
            if collecting and packets:
                finalize("  (no fingerprint seen -- finalizing on next header)")
                if n is not None and got >= n:
                    break
            print(f"[{time.strftime('%H:%M:%S')}] header -- receiving image")
            collecting, packets, recv, total = True, [], 0, None
            continue
        if not collecting:
            continue
        if pkt[:4] == FINGERPRINT:
            collecting = False
            finalize(None)
            continue
        packets.append(pkt)
        if total is None:                       # first CRC-valid params packet sets the total
            total = _params_total(pkt)
        if len(pkt) == LORA_PAYLOAD:            # a data or parity packet arrived
            recv += 1
            if progress:
                render_serial_progress(recv, total)


def serial_main(args):
    """Interactive LoRa config link: optionally push a config to the sender, then save the
    reassembled image(s). Declining a config falls back to today's passive save loop."""
    global cv2, rs
    import cv2                                            # noqa: F401 (bound to global)
    import rs_gf256 as rs                                 # noqa: F401 (bound to global)
    import serial

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print("Connected to", args.port, "-- LoRa config link (Ctrl-C to quit)")
    try:
        while True:
            cfg = prompt_config()
            if cfg is None:
                print("Listening for images (Ctrl-C to return to menu)...")
                try:
                    receive_images(ser, None, outdir=args.outdir, progress=not args.no_progress)
                except KeyboardInterrupt:
                    print()
                continue
            if not send_request(ser, cfg):
                continue
            if cfg & 0x1:                                 # specific count
                n = max(1, (cfg >> 20) & 0xFFF)
                print(f"Waiting for {n} image(s)...")
                try:
                    receive_images(ser, n, outdir=args.outdir, progress=not args.no_progress)
                    print(f"Received {n} image(s).")
                except KeyboardInterrupt:
                    print()
            else:                                         # continuous
                print("Continuous mode -- receiving (Ctrl-C to return to menu)...")
                try:
                    receive_images(ser, None, outdir=args.outdir, progress=not args.no_progress)
                except KeyboardInterrupt:
                    print()
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


# ===========================================================================
# SDR path -- drive rtl_sdr, detect bursts by power, decode each to a PNG
# (ported from monitor_lora_image.py)
# ===========================================================================
PKT_S = 0.475                              # ~seconds per 250-byte packet (SF7/BW125)

# bytes-per-complex-sample and decoder for each stdin/file IQ format
FORMATS = {
    "fc32": (8, lambda b: np.frombuffer(b, np.float32).view(np.complex64)),
    "cu8":  (2, lambda b: (np.frombuffer(b, np.uint8).astype(np.float32) - 127.5)
                          .view(np.complex64) / 127.5),
    "cs16": (4, lambda b: (np.frombuffer(b, np.int16).astype(np.float32) / 32768.0)
                          .view(np.complex64)),
}


def _worker_init():
    """Decode-pool workers ignore SIGINT; the main process handles Ctrl-C and shuts the
    pool down cleanly, instead of every worker raising KeyboardInterrupt at once."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)


def iq_blocks(stream, fmt, block_samps, pace=0.0):
    """Yield complex64 blocks of ~block_samps from a binary stream. If pace>0, sleep
    that long after each block (used to replay a file at real time for testing)."""
    bps, conv = FORMATS[fmt]
    nbytes = block_samps * bps
    buf = b""
    while True:
        chunk = stream.read(nbytes - len(buf))
        if not chunk:
            if buf and len(buf) >= bps:
                yield conv(buf[:len(buf) // bps * bps])
            return
        buf += chunk
        if len(buf) >= nbytes:
            yield conv(buf[:nbytes])
            buf = b""
            if pace:
                time.sleep(pace)


def block_power(x):
    m = x.mean()
    return float(np.mean(np.abs(x - m) ** 2))


# ===========================================================================
# SDR PHY -- LoRa demodulator + v2/v1 reassembly.
# Inlined so receive.py is standalone (was a separate decode_lora_image.py).
# The PHY chain (gray / diagonal deinterleave / dewhiten / Hamming) is ported
# from LoRaPHY (jkadbear). scipy / shared_memory / rs_gf256 are imported lazily
# in sdr_main so the serial-only path never needs them.
# ===========================================================================

def _attach_shm(name):
    """Attach to an existing shared-memory segment created by the parent process. On
    Python 3.13+ pass track=False so this worker's resource_tracker does NOT also adopt
    the segment -- only the creator should unlink it. Without this, when the workers exit
    each one's tracker tries to clean up the (already-freed) segment and floods stderr
    with 'leaked shared_memory' / 'No such file' warnings at shutdown. Older Pythons lack
    the parameter, where those warnings are cosmetic."""
    try:
        return shared_memory.SharedMemory(name=name, track=False)   # Python 3.13+
    except TypeError:
        return shared_memory.SharedMemory(name=name)

# ---- LoRa parameters (must match the transmitter) ----
SF = 7
BW = 500e3            # matches the transmitter's LORA_BW (Lever A); override with --bw
CR = 1                # 4/5  -> rdd = 4+CR = 5 (only used if header parse is skipped)
N = 1 << SF           # 128 chips / symbol
OSF = 2               # oversample factor for demod
SPS = OSF * N         # 256 samples / symbol
ZP = 10               # FFT zero-padding ratio (fine bin resolution)
FFT_LEN = SPS * ZP    # 2560
BIN_NUM = N * ZP      # 1280
PREAMBLE_LEN = 8
SYNC_WORD = 0x12
RF_FREQ = 915e6       # carrier, for SFO-drift compensation
HDR = HEADER          # header magic; receive.py's serial half calls it HEADER




# LoRa data-whitening sequence (LFSR x^8+x^6+x^5+x^4+1), from LoRaPHY.
WHITENING = np.array([
0xff,0xfe,0xfc,0xf8,0xf0,0xe1,0xc2,0x85,0x0b,0x17,0x2f,0x5e,0xbc,0x78,0xf1,0xe3,
0xc6,0x8d,0x1a,0x34,0x68,0xd0,0xa0,0x40,0x80,0x01,0x02,0x04,0x08,0x11,0x23,0x47,
0x8e,0x1c,0x38,0x71,0xe2,0xc4,0x89,0x12,0x25,0x4b,0x97,0x2e,0x5c,0xb8,0x70,0xe0,
0xc0,0x81,0x03,0x06,0x0c,0x19,0x32,0x64,0xc9,0x92,0x24,0x49,0x93,0x26,0x4d,0x9b,
0x37,0x6e,0xdc,0xb9,0x72,0xe4,0xc8,0x90,0x20,0x41,0x82,0x05,0x0a,0x15,0x2b,0x56,
0xad,0x5b,0xb6,0x6d,0xda,0xb5,0x6b,0xd6,0xac,0x59,0xb2,0x65,0xcb,0x96,0x2c,0x58,
0xb0,0x61,0xc3,0x87,0x0f,0x1f,0x3e,0x7d,0xfb,0xf6,0xed,0xdb,0xb7,0x6f,0xde,0xbd,
0x7a,0xf5,0xeb,0xd7,0xae,0x5d,0xba,0x74,0xe8,0xd1,0xa2,0x44,0x88,0x10,0x21,0x43,
0x86,0x0d,0x1b,0x36,0x6c,0xd8,0xb1,0x63,0xc7,0x8f,0x1e,0x3c,0x79,0xf3,0xe7,0xce,
0x9c,0x39,0x73,0xe6,0xcc,0x98,0x31,0x62,0xc5,0x8b,0x16,0x2d,0x5a,0xb4,0x69,0xd2,
0xa4,0x48,0x91,0x22,0x45,0x8a,0x14,0x29,0x52,0xa5,0x4a,0x95,0x2a,0x54,0xa9,0x53,
0xa7,0x4e,0x9d,0x3b,0x77,0xee,0xdd,0xbb,0x76,0xec,0xd9,0xb3,0x67,0xcf,0x9e,0x3d,
0x7b,0xf7,0xef,0xdf,0xbf,0x7e,0xfd,0xfa,0xf4,0xe9,0xd3,0xa6,0x4c,0x99,0x33,0x66,
0xcd,0x9a,0x35,0x6a,0xd4,0xa8,0x51,0xa3,0x46,0x8c,0x18,0x30,0x60,0xc1,0x83,0x07,
0x0e,0x1d,0x3a,0x75,0xea,0xd5,0xaa,0x55,0xab,0x57,0xaf,0x5f,0xbe,0x7c,0xf9,0xf2,
0xe5,0xca,0x94,0x28,0x50,0xa1,0x42,0x84,0x09,0x13,0x27,0x4f,0x9f,0x3f,0x7f],
dtype=np.uint8)


def make_chirps():
    k = np.arange(SPS) / OSF                       # chip index 0..N
    phase = 2 * np.pi * (k * k / (2 * N) - k / 2)
    up = np.exp(1j * phase).astype(np.complex64)
    return up, np.conj(up)

UPCHIRP, DOWNCHIRP = make_chirps()


def dechirp(sig, x, is_up=True):
    """Return (peak_height, peak_bin) for the symbol window at sample x.
    is_up=True detects an up-chirp (multiply by downchirp)."""
    seg = sig[x:x + SPS]
    if len(seg) < SPS:
        return (0.0, 0)
    c = DOWNCHIRP if is_up else UPCHIRP
    ft = np.fft.fft(seg * c, FFT_LEN)
    folded = np.abs(ft[:BIN_NUM]) + np.abs(ft[FFT_LEN - BIN_NUM:])
    b = int(np.argmax(folded))
    return (float(folded[b]), b)


# ---------- decode chain (ported from LoRaPHY) ----------
def gray_coding(din):
    din = din.astype(np.int64).copy()
    din[:8] //= 4                              # header: reduced rate (SF-2)
    din[8:] = (din[8:] - 1) % N                # payload: -1
    s = din.astype(np.uint16)
    return (s ^ (s >> 1)).astype(np.int64)


def diag_deinterleave(symbols_g, ppm):
    """symbols_g: array of len M; ppm bits/symbol -> ppm codewords of M bits.
    Vectorized: MSB-first bit matrix, roll row i left by i, reassemble, reverse."""
    s = np.asarray(symbols_g, dtype=np.int64)
    M = len(s)
    b = (s[:, None] >> np.arange(ppm - 1, -1, -1)) & 1          # (M, ppm) MSB-first
    j = (np.arange(ppm)[None, :] + np.arange(M)[:, None]) % ppm  # roll row i by -i
    b = b[np.arange(M)[:, None], j]
    w = (1 << np.arange(M, dtype=np.int64))                      # codeword j = sum_m b[m,j]<<m
    return (b * w[:, None]).sum(0)[::-1].astype(np.int64)


def _deinterleave_batch(blocks, ppm):
    """Batched diag_deinterleave: blocks (nblk, M) -> codewords (nblk, ppm)."""
    nblk, M = blocks.shape
    b = (blocks[:, :, None] >> np.arange(ppm - 1, -1, -1)) & 1   # (nblk, M, ppm)
    j = (np.arange(ppm)[None, :] + np.arange(M)[:, None]) % ppm
    b = b[:, np.arange(M)[:, None], j]
    w = (1 << np.arange(M, dtype=np.int64))
    return (b * w[None, :, None]).sum(1)[:, ::-1]


def _bit_reduce(cw, positions):
    r = np.zeros_like(cw)
    for p in positions:                        # positions are 1-indexed (bit 1 = LSB)
        r ^= (cw >> (p - 1)) & 1
    return r


def hamming_decode(codewords, rdd):
    cw = codewords.astype(np.int64)
    if rdd in (5, 6):
        return (cw & 0xF).astype(np.int64)
    p2 = _bit_reduce(cw, [7, 4, 2, 1])
    p3 = _bit_reduce(cw, [5, 3, 2, 1])
    p5 = _bit_reduce(cw, [6, 4, 3, 2])
    parity = p2 * 4 + p3 * 2 + p5
    fix = {3: 4, 5: 8, 6: 1, 7: 2}
    pf = np.array([fix.get(int(p), 0) for p in parity], dtype=np.int64)
    cw = cw ^ pf
    return (cw & 0xF).astype(np.int64)


def dewhiten(byts):
    b = np.frombuffer(bytes(byts), dtype=np.uint8)
    return (b ^ WHITENING[:len(b)]).astype(np.uint8)


def decode_symbols(symbols):
    """symbols: raw demodulated values (0..127). Returns payload bytes (undewhitened
    header handled separately). Returns (payload_len, cr, crc, data_bytes)."""
    sg = gray_coding(np.asarray(symbols, dtype=np.int64))
    # header: first 8 symbols, ppm = SF-2, rdd=8
    cw = diag_deinterleave(sg[:8], SF - 2)
    hn = hamming_decode(cw, 8)
    payload_len = int(hn[0] * 16 + hn[1])
    crc = int(hn[2] & 1)
    cr = int(hn[2] >> 1)
    nibbles = list(hn[5:])
    rdd = cr + 4
    if rdd < 5:
        rdd = 5
    ii = 8
    while ii + rdd <= len(sg):
        cw = diag_deinterleave(sg[ii:ii + rdd], SF)
        nibbles.extend(list(hamming_decode(cw, rdd)))
        ii += rdd
    nb = len(nibbles) // 2
    byts = bytes((int(nibbles[2 * i]) | (int(nibbles[2 * i + 1]) << 4)) for i in range(nb))
    data = dewhiten(byts[:payload_len]) if payload_len else dewhiten(byts)
    return payload_len, cr, crc, bytes(data)


# ---------- packet detection / sync / demod ----------
def detect_and_demod(sig, start):
    """From sample `start`, find the next preamble, sync, and demodulate the packet.
    Returns (next_start, symbols) or (None, None) if no more packets."""
    ii = start
    bins = []
    # preamble detection: PREAMBLE_LEN-1 consecutive stable up-chirp bins
    while ii < len(sig) - SPS * (PREAMBLE_LEN + 6):
        h, b = dechirp(sig, ii)
        if bins:
            d = (bins[-1] - b) % BIN_NUM
            d = min(d, BIN_NUM - d)
            if d <= ZP * 2:
                bins.append(b)
            else:
                bins = [b]
        else:
            bins = [b]
        if len(bins) == PREAMBLE_LEN - 1:
            x = ii - round((bins[-1]) / ZP * OSF)   # coarse timing align
            break
        ii += SPS
    else:
        return None, None, None

    # find SFD: slide until down-chirp peak beats up-chirp peak
    x = max(x, 0)
    guard = 0
    while x < len(sig) - SPS and guard < PREAMBLE_LEN + 8:
        hu, _ = dechirp(sig, x, True)
        hd, _ = dechirp(sig, x, False)
        if hd > hu:
            break
        x += SPS
        guard += 1
    # fine align on the down-chirp bin
    _, bd = dechirp(sig, x, False)
    to = round((bd - BIN_NUM) / ZP) if bd > BIN_NUM / 2 else round(bd / ZP)
    x += to * OSF
    # preamble reference bin, 4 symbols before the SFD
    _, preamble_bin = dechirp(sig, x - 4 * SPS, True)
    # carrier freq offset from the preamble bin (fraction of BW)
    if preamble_bin > BIN_NUM / 2:
        cfo = (preamble_bin - BIN_NUM) * BW / BIN_NUM
    else:
        cfo = preamble_bin * BW / BIN_NUM
    # start of first data symbol: 2.25 symbols after SFD
    x_sync = x + round(2.25 * SPS)

    # demodulate symbols until the signal power drops (end of packet). Correct the
    # per-symbol SFO drift (proportional to CFO) that otherwise walks long packets
    # off by a bin.
    ref = preamble_bin
    win_pow = np.abs(sig[x - 4 * SPS:x]).mean() ** 2
    # Number of symbols before the power drops -- same rule as the old per-symbol
    # loop (stop at the first window below 0.15*win_pow; cap 701; stop at buffer end).
    max_k = min(701, (len(sig) - x_sync) // SPS)
    empty = dict(preamble_bin=preamble_bin, cfo=cfo,
                 raw_bins=np.array([], dtype=np.int64), quality=np.array([]))
    if max_k <= 0:
        return x_sync, np.array([], dtype=np.int64), empty
    wins = sig[x_sync:x_sync + max_k * SPS].reshape(max_k, SPS)
    pw = np.abs(wins).mean(axis=1) ** 2
    below = np.nonzero(pw < 0.15 * win_pow)[0]
    nsym = int(below[0]) if below.size else max_k
    if nsym == 0:
        return x_sync, np.array([], dtype=np.int64), empty
    # Batched dechirp: one FFT over all symbol windows instead of one per symbol.
    ft = np.fft.fft(wins[:nsym] * DOWNCHIRP, FFT_LEN, axis=1)
    folded = np.abs(ft[:, :BIN_NUM]) + np.abs(ft[:, FFT_LEN - BIN_NUM:])
    raw_bins = np.argmax(folded, axis=1)
    quality = folded[np.arange(nsym), raw_bins] / (folded.mean(axis=1) + 1e-9)
    next_start = x_sync + nsym * SPS
    meta = dict(preamble_bin=preamble_bin, cfo=cfo, raw_bins=raw_bins, quality=quality,
                pkt_start=x - (PREAMBLE_LEN + 2) * SPS)   # ~preamble start, for sharding
    symbols = bins_to_symbols(raw_bins, ref, cfo)
    return next_start, symbols, meta


def bins_to_symbols(raw_bins, ref, cfo, drift_coef=1.0, off=0.0):
    """Convert fine FFT bins to symbol values (drift_coef*SFO drift + constant
    sub-bin offset removed). Vectorized; np.round matches Python round (banker's)."""
    b = np.asarray(raw_bins, dtype=np.float64)
    k = np.arange(len(b))
    drift = drift_coef * (k + 1) * N * cfo / RF_FREQ
    return np.round((b - ref) / ZP - drift - off).astype(np.int64) % N


def _parity_violations(syms):
    """Count CR4/5 payload codewords that fail even parity (error detection).
    Vectorized over all 5-symbol payload blocks."""
    sg = gray_coding(syms)
    pay = sg[8:]
    nblk = len(pay) // 5
    if nblk == 0:
        return 0
    cw = _deinterleave_batch(pay[:nblk * 5].reshape(nblk, 5), SF)  # (nblk, 7), 5-bit
    bits = (cw[:, :, None] >> np.arange(5)) & 1
    ok = bits[..., 4] == (bits[..., 0] ^ bits[..., 1] ^ bits[..., 2] ^ bits[..., 3])
    return int((~ok).sum())


def best_demod(raw_bins, ref, cfo):
    """Pick the demod (drift, sub-bin offset) minimizing parity violations. This
    removes a ~0.4-bin rounding bias that otherwise flips ~3% of symbols on the
    long 368-symbol data packets. Returns (symbols, residual_violations)."""
    best = None
    for dc in (0.0, 0.5, 1.0):
        for off in np.arange(-0.48, 0.49, 0.03):
            syms = bins_to_symbols(raw_bins, ref, cfo, dc, off)
            v = _parity_violations(syms)
            if best is None or v < best[1]:
                best = (syms, v)
            if v == 0:
                return best
    return best


def resample_to_lora(iq, fs, sub_mean=None):
    """DC-remove and resample complex IQ from fs to OSF*BW (250 kHz). `sub_mean`
    lets shard workers remove the *whole-signal* DC so a chunk's interior samples
    match the single-shot resample exactly."""
    x = np.ascontiguousarray(iq).astype(np.complex64)
    x = x - (x.mean() if sub_mean is None else sub_mean)
    fs2 = BW * OSF
    g = np.gcd(int(fs), int(fs2))
    return sp_signal.resample_poly(x, int(fs2) // g, int(fs) // g).astype(np.complex64)


def _decode_packets_located(sig, verbose=False):
    """Packet loop; returns [(pkt_start_sample_in_sig, payload, payload_len, resid), ...].
    `payload_len` is the header-declared length and `resid` the residual CR4/5 parity
    violations from best_demod -- both used downstream to accept/reject a packet."""
    out, pos = [], 0
    while pos < len(sig):
        nxt, _syms, meta = detect_and_demod(sig, pos)
        if nxt is None:
            break
        if meta and len(meta["raw_bins"]) >= 8:
            syms, resid = best_demod(meta["raw_bins"], meta["preamble_bin"], meta["cfo"])
            pl, cr, crc, data = decode_symbols(syms)
            out.append((meta["pkt_start"], data, pl, resid))
            if verbose:
                flag = "" if resid == 0 else f"  !! {resid} parity errs"
                print(f"  packet @{pos/(BW*OSF):6.3f}s  len={pl:3d} cr={cr} "
                      f"nsym={len(syms):3d} bytes[0:8]={data[:8].hex(' ')}{flag}")
        pos = nxt if nxt > pos else pos + SPS
    return out


def decode_packets(sig, verbose=False):
    """Run the packet loop over a resampled signal; return list of packet payloads."""
    return [d for _p, d, _pl, _r in _decode_packets_located(sig, verbose=verbose)]


# max packet span (resampled samples) and resample edge transient, for shard interiors
_PKT_SPAN = (PREAMBLE_LEN + 4 + 420) * SPS   # 420 symbols covers a 250-byte payload
_TRANS = 4000


def _decode_raw_shard(task):
    """Worker: resample one raw-IQ chunk and decode the packets whose start lies in
    its non-transient interior. Returns [(global_raw_start, payload), ...]."""
    shm_name, shape, dtype_str, o_lo, o_hi, pad, fs, gmean = task
    shm = _attach_shm(shm_name)                   # attach without spurious tracker cleanup
    try:
        iq = np.ndarray(shape, dtype=np.dtype(dtype_str), buffer=shm.buf)
        lo = max(0, o_lo - pad)
        hi = min(shape[0], o_hi + pad)
        sig = resample_to_lora(iq[lo:hi], fs, sub_mean=gmean)
    finally:
        shm.close()
    scale = fs / (BW * OSF)                   # raw samples per resampled sample
    hi_lim = len(sig) - _PKT_SPAN - _TRANS
    return [(lo + pstart * scale, payload, pl, resid)
            for pstart, payload, pl, resid in _decode_packets_located(sig)
            if _TRANS <= pstart <= hi_lim]


def extract_jpeg(packets):
    """Reassemble packet payloads and return the JPEG bytes (SOI..EOI) or None."""
    blob = b"".join(packets)
    soi, eoi = blob.find(b"\xff\xd8"), blob.rfind(b"\xff\xd9")
    if soi != -1 and eoi != -1 and eoi > soi:
        return blob[soi:eoi + 2]
    return None


def _reassemble_indexed(located, fs, verbose=False):
    """Place each ACCEPTED 250-byte data packet at its inferred stream index and
    zero-fill gaps, so a lost or rejected packet becomes a localized hole instead of
    shifting the whole JPEG byte stream. A packet is accepted only if its declared
    length is exactly LORA_PAYLOAD and it has no residual CR4/5 parity violations;
    corrupt / short / wrong-length packets are dropped (they'd otherwise desync the
    stream). Index is inferred from the packet's raw-sample position and the (regular)
    inter-packet spacing, since the current wire format carries no sequence number.

    `located` : [(raw_pos, data, payload_len, resid), ...] for every decoded packet
                (control + data), positions in raw-sample units.
    Returns (blob_bytes, stats).
    """
    located = sorted(located, key=lambda t: t[0])
    # Match the tiny control packets tolerantly: they carry few LoRa symbols and
    # (with CRC off / CR4/5) occasionally decode with an undetected bit error, so an
    # exact magic compare misses them. Allow up to 1 differing byte.
    hdr_like = lambda d: sum(a != b for a, b in zip(bytes(d[:5]), HDR)) <= 1 and len(d) >= 5
    fp_like = lambda d: sum(a != b for a, b in zip(bytes(d[:4]), FINGERPRINT)) <= 1 and len(d) >= 4
    is_ctrl = lambda d: hdr_like(d) or fp_like(d)
    # transmitted packet count, read from the header -> count sequence (if present).
    # The count packet is exactly 2 bytes; require that so we don't misread the first
    # data packet's bytes (e.g. the JPEG SOI) as a bogus count.
    N = None
    for i in range(len(located) - 1):
        if hdr_like(located[i][1]):
            _cp, cdata, cpl, _cr = located[i + 1]
            if cpl == 2 and len(cdata) >= 2:
                N = int(cdata[0] | (cdata[1] << 8))
            break
    data_pkts = [(pos, bytes(data)) for (pos, data, pl, resid) in located
                 if pl == LORA_PAYLOAD and resid == 0]
    rej_parity = sum(1 for (_p, _d, pl, r) in located if pl == LORA_PAYLOAD and r != 0)
    rej_len = sum(1 for (_p, d, pl, _r) in located
                  if pl != LORA_PAYLOAD and not is_ctrl(d))
    stats = dict(count=N, accepted=len(data_pkts), rej_parity=rej_parity,
                 rej_len=rej_len, missing=0, span=0, missing_idx=[])
    if not data_pkts:
        return b"", stats

    positions = np.array([p for p, _ in data_pkts], dtype=np.float64)
    nominal = 0.475 * fs                       # ~one data packet, in raw samples
    gaps = np.diff(positions)
    if len(gaps):
        spacing = float(np.median(gaps))       # robust to real (multi-packet) gaps
        if not (0.5 * nominal < spacing < 2.0 * nominal):
            spacing = nominal                  # fall back if the estimate is nonsense
    else:
        spacing = nominal
    # Assign indices from LOCAL consecutive gaps: a ~1x gap is the next packet (+1),
    # a ~2x gap means one packet was lost in between (+2), etc. Anchoring off a single
    # global spacing instead would let a small spacing/true-rate mismatch accumulate
    # into phantom gaps -- inferring per-gap avoids that drift.
    idx = np.empty(len(positions), dtype=int)
    idx[0] = 0
    for i, g in enumerate(gaps):
        idx[i + 1] = idx[i] + max(1, int(round(g / spacing)))
    span = int(idx[-1]) + 1
    # Trust the transmitted count only if it's consistent with the observed span
    # (a plausible number of trailing packets may be lost); otherwise it's a
    # mis-decoded count -- fall back to the span so we don't allocate a bogus buffer.
    if N is None or not (span <= N <= span + 64):
        N = span
    buf = bytearray(N * LORA_PAYLOAD)
    seen = set()
    for (pos, data), k in zip(data_pkts, idx):
        if 0 <= k < N:
            buf[k * LORA_PAYLOAD:(k + 1) * LORA_PAYLOAD] = \
                data[:LORA_PAYLOAD].ljust(LORA_PAYLOAD, b"\x00")
            seen.add(int(k))
    stats.update(count=N, missing=N - len(seen), span=span,
                 missing_idx=sorted(set(range(N)) - seen))
    if verbose:
        print(f"  reassemble: N={N} placed={len(seen)} missing={stats['missing']} "
              f"rej(len={rej_len},parity={rej_parity}) spacing={spacing/fs*1e3:.0f}ms")
    return bytes(buf), stats


def _find_params_located(located):
    """Return the decoded v2 params dict (from the first CRC-valid params packet), or
    None if this burst isn't protocol v2. Params is a 15-byte packet:
    [0x70 0x32][ver][image_len:4 LE][Ndata:2][K:2][M:2][crc16:2]."""
    for _p, data, pl, _r in located:
        d = bytes(data)
        # Don't insist on payload_len==15: short control packets sometimes decode with
        # a wrong length nibble. The magic + CRC over 13 bytes is decisive on its own.
        if len(d) >= PARAMS_LEN and d[:2] == PARAMS_MAGIC:
            if crc16_ccitt(d[:13]) != (d[13] | (d[14] << 8)):
                continue
            return dict(version=d[2],
                        image_len=int.from_bytes(d[3:7], "little"),
                        Ndata=d[7] | (d[8] << 8),
                        K=d[9] | (d[10] << 8),
                        M=d[11] | (d[12] << 8))
    return None


def _reassemble_v2(located, params, verbose=False):
    """Protocol-v2 reassembly. CRC-check every 250-byte packet, split into data
    (seq < Ndata) and parity (seq >= Ndata) by block, run Reed-Solomon erasure decode
    per block to reconstruct missing/corrupt data packets, then return the JPEG bytes
    (first image_len bytes) plus stats. `located` = [(pos,data,pl,resid), ...]."""
    Ndata, K, M, image_len = params["Ndata"], params["K"], params["M"], params["image_len"]
    nblocks = (Ndata + K - 1) // K if K else 0
    # dedup by seq; CRC-good packets win. rows: seq -> 246-byte payload
    data_rows, parity_rows, crc_fail = {}, {}, 0
    for _p, data, pl, _r in located:
        d = bytes(data)
        if pl != LORA_PAYLOAD or len(d) < LORA_PAYLOAD:
            continue
        body, crc = d[:248], d[248] | (d[249] << 8)
        if crc16_ccitt(body) != crc:
            crc_fail += 1
            continue
        seq = body[0] | (body[1] << 8)
        payload = body[2:2 + V2_PAYLOAD]
        if seq < Ndata:
            data_rows[seq] = payload
        else:
            parity_rows[seq] = payload

    out = bytearray(Ndata * V2_PAYLOAD)
    recovered = unrecoverable = present = 0
    for b in range(nblocks):
        lo = b * K
        k = min(K, Ndata - lo)
        have_data = {j: data_rows[lo + j] for j in range(k) if (lo + j) in data_rows}
        have_par = {i: parity_rows[Ndata + b * M + i]
                    for i in range(M) if (Ndata + b * M + i) in parity_rows}
        present += len(have_data)
        missing_data = [j for j in range(k) if j not in have_data]
        if not missing_data:
            for j in range(k):
                out[(lo + j) * V2_PAYLOAD:(lo + j + 1) * V2_PAYLOAD] = have_data[j]
            continue
        # need k rows total (data + parity) to invert
        if len(have_data) + len(have_par) >= k and len(missing_data) <= M:
            recv = np.zeros((k + M, V2_PAYLOAD), dtype=np.uint8)
            erased = []
            for j in range(k):
                if j in have_data:
                    recv[j] = np.frombuffer(have_data[j], dtype=np.uint8)
                else:
                    erased.append(j)
            for i in range(M):
                if i in have_par:
                    recv[k + i] = np.frombuffer(have_par[i], dtype=np.uint8)
                else:
                    erased.append(k + i)
            try:
                dec = rs.decode(recv, k, M, erased=erased)
                for j in range(k):
                    out[(lo + j) * V2_PAYLOAD:(lo + j + 1) * V2_PAYLOAD] = dec[j].tobytes()
                recovered += len(missing_data)
                continue
            except Exception as e:
                if verbose:
                    print(f"  block {b}: RS decode failed ({e})")
        # unrecoverable: place what we have, zero-fill the rest
        unrecoverable += len(missing_data)
        for j, row in have_data.items():
            out[(lo + j) * V2_PAYLOAD:(lo + j + 1) * V2_PAYLOAD] = row

    jpg = bytes(out[:image_len])
    stats = dict(v2=True, count=Ndata, accepted=present, missing=Ndata - present,
                 recovered=recovered, unrecoverable=unrecoverable, crc_fail=crc_fail,
                 rej_len=0, rej_parity=0, missing_idx=[])
    if verbose:
        print(f"  v2: Ndata={Ndata} K={K} M={M} present={present} "
              f"recovered={recovered} unrecoverable={unrecoverable} crc_fail={crc_fail}")
    return jpg, stats


def decode_iq(iq, fs, pool=None, workers=None, verbose=False):
    """Decode a burst of complex IQ -> (jpeg_bytes_or_None, num_data_packets, stats).

    With workers>1 (and/or a ProcessPoolExecutor `pool`), the raw IQ is split into
    overlapping shards, each resampled + decoded in a separate process (parallelizes
    the dominant resample too); results are merged by packet position and deduped.
    Reassembly rejects corrupt/wrong-length packets and places the good ones by
    inferred stream index (zero-filling gaps) -- see _reassemble_indexed. `stats`
    reports transmitted count / accepted / missing / rejected for diagnostics."""
    iq = np.ascontiguousarray(iq)
    if workers is None:
        workers = getattr(pool, "_max_workers", 1) if pool is not None else 1
    workers = int(workers)
    scale = fs / (BW * OSF)
    pad = int((_PKT_SPAN + _TRANS) * scale) + 10000     # raw-sample overlap per shard
    nshards = max(1, min(workers, len(iq) // (3 * pad))) if workers > 1 else 1

    if nshards <= 1:                                     # single-process (small burst)
        loc = _decode_packets_located(resample_to_lora(iq, fs), verbose=verbose)
        located = [(p * scale, d, pl, r) for (p, d, pl, r) in loc]  # -> raw samples
    else:
        gmean = complex(iq.astype(np.complex64).mean())  # shared DC for every shard
        shm = shared_memory.SharedMemory(create=True, size=iq.nbytes)
        try:
            np.ndarray(iq.shape, dtype=iq.dtype, buffer=shm.buf)[:] = iq
            edges = np.linspace(0, len(iq), nshards + 1).astype(int)
            tasks = [(shm.name, iq.shape, iq.dtype.str, int(edges[i]), int(edges[i + 1]),
                      pad, fs, gmean) for i in range(nshards)]
            ex = pool or ProcessPoolExecutor(max_workers=workers)
            try:
                results = list(ex.map(_decode_raw_shard, tasks))
            finally:
                if pool is None:
                    ex.shutdown()
        finally:
            shm.close(); shm.unlink()
        merged = sorted((pp for r in results for pp in r), key=lambda t: t[0])
        located, last, tol = [], None, 0.15 * fs       # dedup overlap duplicates
        for tup in merged:
            if last is None or tup[0] - last > tol:
                located.append(tup); last = tup[0]

    params = _find_params_located(located)      # protocol v2 if present
    if params is not None:
        jpg, stats = _reassemble_v2(located, params, verbose=verbose)
        jpg = extract_jpeg([jpg]) if jpg else None
        # keep the legacy stat keys the callers print
        stats.setdefault("count", params["Ndata"])
        return jpg, stats["accepted"], stats

    blob, stats = _reassemble_indexed(located, fs, verbose=verbose)
    jpg = extract_jpeg([blob]) if blob else None
    return jpg, stats["accepted"], stats


def stats_summary(stats):
    """One-line human summary of a decode's packet accounting, for logs."""
    if stats.get("v2"):
        return (f"v2 Ndata={stats['count']} present={stats['accepted']} "
                f"recovered={stats['recovered']} unrecoverable={stats['unrecoverable']} "
                f"crc_fail={stats['crc_fail']}")
    return (f"count={stats['count']} placed={stats['accepted']} missing={stats['missing']} "
            f"rej(len={stats['rej_len']},par={stats['rej_parity']})")


def sniff_count(blocks, rate, shared):
    """Decode just the start of a burst to read the transmitted packet count. Runs in
    a side thread so it can't stall the SDR reader. Sets shared['N'] once known.
    Handles both v2 (params packet: Ndata + K/M parity) and legacy v1 (2-byte count)."""
    try:
        packets = decode_packets(resample_to_lora(np.concatenate(blocks), rate))
        # v2: params packet gives Ndata, K, M -> total = Ndata + ceil(Ndata/K)*M
        for p in packets:
            if len(p) >= PARAMS_LEN and bytes(p[:2]) == PARAMS_MAGIC \
               and crc16_ccitt(bytes(p[:13])) == (p[13] | (p[14] << 8)):
                Ndata = p[7] | (p[8] << 8)
                K = p[9] | (p[10] << 8)
                M = p[11] | (p[12] << 8)
                nblocks = (Ndata + K - 1) // K if K else 0
                shared["N"] = Ndata + nblocks * M
                return
        # v1: 2-byte little-endian count right after the header
        for i in range(len(packets) - 1):
            if bytes(packets[i][:5]) == HEADER:
                c = packets[i + 1]
                shared["N"] = (c[0] | (c[1] << 8)) if len(c) >= 2 else (c[0] if c else 0)
                return
    except Exception:
        pass


def render_progress(elapsed, N, done=False):
    """Draw an in-place progress bar on stderr for the burst currently arriving."""
    if N:
        k = N if done else min(N, max(0.0, elapsed / PKT_S - 2))  # -2: header+count
        frac = k / N
        w = 24
        bar = "#" * int(frac * w) + "." * (w - int(frac * w))
        sys.stderr.write(f"\r  receiving [{bar}] {frac*100:3.0f}%  ~{int(round(k))}/{N} pkts  {elapsed:4.1f}s   ")
    else:
        sys.stderr.write(f"\r  receiving image... {elapsed:4.1f}s (reading count)   ")
    sys.stderr.flush()


def decoder_worker(q, args, pool):
    """Decode completed bursts and write timestamped PNGs. Decode runs in `pool`
    (separate processes) so it never contends with the SDR-reader thread's GIL."""
    from PIL import Image, ImageFile
    from io import BytesIO
    while True:
        item = q.get()
        if item is None:
            return
        start_unix, iq = item
        t0 = time.time()
        try:
            jpg, npkt, stats = decode_iq(iq, args.rate, pool=pool)
        except Exception as e:
            log(f"decode error: {e}")
            q.task_done(); continue
        if not jpg:
            log(f"burst @{start_unix}: no image [{stats_summary(stats)}] "
                f"({len(iq)/args.rate:.1f}s) -- skipped")
            q.task_done(); continue
        # render (strict first; fall back to truncated so partial frames still land)
        strict = True
        try:
            im = Image.open(BytesIO(jpg)); im.load()
        except Exception:
            strict = False
            ImageFile.LOAD_TRUNCATED_IMAGES = True
            try:
                im = Image.open(BytesIO(jpg)); im.load()
            except Exception as e:
                log(f"burst @{start_unix}: JPEG undecodable ({e}) -- skipped")
                q.task_done(); continue
        day = time.strftime("%Y%m%d", time.gmtime(start_unix))
        outdir = os.path.join(args.outdir, day)
        os.makedirs(outdir, exist_ok=True)
        path = os.path.join(outdir, f"{start_unix}.png")
        im.convert("RGB").save(path)
        log(f"burst @{start_unix}: saved {im.size} {'OK' if strict else 'PARTIAL'} "
            f"[{stats_summary(stats)}] decode {time.time()-t0:.1f}s -> {path}")
        q.task_done()


def log(msg):
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())}Z] {msg}", flush=True)


class _PipeReader:
    """File-like reader that drains a subprocess's stdout in a background thread into a
    bounded byte buffer, then serves it via .read(n). This keeps rtl_sdr's pipe emptied
    even when the main loop briefly stalls (e.g. np.concatenate of a finished burst), so
    the SDR is far less likely to drop samples than behind the OS's small pipe buffer."""

    def __init__(self, proc, bufbytes=16 << 20, chunk=64 << 10):
        self.proc, self.chunk = proc, chunk
        self.q = queue.Queue(maxsize=max(2, bufbytes // chunk))
        self._buf, self._eof = b"", False
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        try:
            while True:
                data = self.proc.stdout.read(self.chunk)
                if not data:
                    break
                self.q.put(data)
        except Exception:
            pass
        finally:
            self.q.put(None)                              # EOF sentinel

    def read(self, n):
        while len(self._buf) < n and not self._eof:
            item = self.q.get()
            if item is None:
                self._eof = True
                break
            self._buf += item
        out, self._buf = self._buf[:n], self._buf[n:]
        return out


def _relay_stderr(proc):
    """Forward rtl_sdr's stderr (device info, 'lost samples' overflow warnings, errors)
    into our own log so it isn't silently swallowed."""
    for raw in iter(proc.stderr.readline, b""):
        line = raw.decode("utf-8", "replace").rstrip()
        if line:
            log(f"rtl_sdr: {line}")


def spawn_rtl_sdr(args):
    """Launch rtl_sdr streaming cu8 IQ to its stdout; return (proc, _PipeReader)."""
    cmd = ["rtl_sdr", "-f", str(int(args.freq)), "-s", str(int(args.rate)),
           "-d", str(args.device)]
    if args.ppm:
        cmd += ["-p", str(int(args.ppm))]
    if str(args.gain).lower() != "auto":                  # omit -g for AGC (auto)
        cmd += ["-g", str(args.gain)]
    cmd += ["-"]
    log("spawning: " + " ".join(cmd))
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                bufsize=0)
    except FileNotFoundError:
        log("ERROR: rtl_sdr not found on PATH. Install rtl-sdr, or pipe IQ into stdin, "
            "e.g.  rtl_sdr -f 915000000 -s 1800000 - | ./receive.py --force sdr "
            "--format cu8 --rate 1.8e6")
        sys.exit(1)
    threading.Thread(target=_relay_stderr, args=(proc,), daemon=True).start()
    return proc, _PipeReader(proc)


def sdr_main(args):
    """Monitor an IQ source (rtl_sdr / piped stdin / --file) and decode each burst."""
    global BW, sp_signal, shared_memory, rs
    from scipy import signal as sp_signal          # noqa: F401 (SDR-only heavy import;
    from multiprocessing import shared_memory      #   aliased so it can't shadow the
    import rs_gf256 as rs                           #   stdlib `signal` used for SIGINT)

    if getattr(args, "bw", None):        # set before the decode pool forks (workers inherit it)
        BW = args.bw
        globals()["PKT_S"] = PKT_S * (125e3 / args.bw)   # keep the progress bar honest

    block_samps = 1 << 16
    block_dur = block_samps / args.rate
    pre_blocks = max(1, int(0.15 / block_dur))          # pre-trigger buffer
    gap_blocks = max(1, int(args.gap / block_dur))
    max_blocks = int(args.max_burst / block_dur)

    # Persistent decode pool. Created (and warmed) while still single-threaded so
    # forking workers is safe; then decode never blocks the capture thread's GIL.
    # Workers ignore SIGINT so a Ctrl-C (the normal way to stop SDR mode) is handled
    # only by the main process -- otherwise every worker dumps its own KeyboardInterrupt
    # traceback and the shutdown stalls.
    pool = None
    if args.workers > 1:
        pool = ProcessPoolExecutor(max_workers=args.workers, initializer=_worker_init)
        list(pool.map(time.sleep, [0.02] * args.workers))   # pre-fork all workers

    # Pick the IQ source: an explicit file, IQ piped on stdin, or (default) our own
    # rtl_sdr subprocess so the user only has to run this script.
    proc = None
    if args.file:
        stream, fmt = open(args.file, "rb"), args.format
    elif args.drive_sdr or sys.stdin.isatty():           # forced, or run interactively
        proc, stream = spawn_rtl_sdr(args)               # drive the SDR ourselves
        fmt = "cu8"                                       # rtl_sdr's native format
    else:
        stream, fmt = sys.stdin.buffer, args.format      # someone piped IQ in

    q = queue.Queue(maxsize=8)
    worker = threading.Thread(target=decoder_worker, args=(q, args, pool), daemon=True)
    worker.start()

    src = "rtl_sdr" if proc else (args.file or "stdin")
    log(f"monitoring: src={src} fmt={fmt} rate={args.rate/1e6}Msps thr={args.threshold_db}dB "
        f"gap={args.gap}s workers={args.workers} outdir={os.path.abspath(args.outdir)}")

    noise = None
    thr_lin = 10 ** (args.threshold_db / 10)
    prebuf = deque(maxlen=pre_blocks)
    recording = False
    buf, start_unix, quiet, nblk = [], 0, 0, 0
    pace = block_dur if (args.realtime and args.file) else 0.0
    show = not args.no_progress
    t0, shared, sniffed, last_draw = 0.0, {}, False, 0.0

    try:
        for blk in iq_blocks(stream, fmt, block_samps, pace=pace):
            p = block_power(blk)
            if noise is None:
                noise = p
            thresh = noise * thr_lin

            if not recording:
                prebuf.append(blk)
                if p > thresh:                              # burst starts
                    recording = True
                    start_unix = int(time.time())
                    buf = list(prebuf)                      # include pre-trigger context
                    quiet, nblk = 0, len(buf)
                    t0, shared, sniffed, last_draw = time.time(), {}, False, 0.0
                else:
                    noise = 0.98 * noise + 0.02 * p         # track noise floor while idle
            else:
                buf.append(blk); nblk += 1
                quiet = quiet + 1 if p < thresh else 0
                # once ~2 s in, read the packet count from the stream (side thread)
                if show and not sniffed and nblk * block_dur >= 2.0:
                    threading.Thread(target=sniff_count, args=(list(buf), args.rate, shared),
                                     daemon=True).start()
                    sniffed = True
                if show and time.time() - last_draw > 0.2:  # throttle the redraw
                    render_progress(time.time() - t0, shared.get("N"))
                    last_draw = time.time()
                if quiet >= gap_blocks or nblk >= max_blocks:
                    active = (nblk - quiet) * block_dur
                    recording = False
                    prebuf.clear()
                    if show:
                        render_progress(time.time() - t0, shared.get("N"), done=True)
                        sys.stderr.write("\n"); sys.stderr.flush()
                    if active >= args.min_burst:
                        iq = np.concatenate(buf)
                        try:
                            q.put((start_unix, iq), timeout=1.0)
                        except queue.Full:
                            log("decoder busy -- dropped a burst")
                    buf = []
    except KeyboardInterrupt:
        log("stopping (Ctrl-C)")
    finally:
        if proc is not None:                              # stop our rtl_sdr subprocess
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()
        q.put(None)
        worker.join(timeout=60)
        if pool is not None:
            pool.shutdown()


# ===========================================================================
# Device detection + dispatch
# ===========================================================================
def serial_present(port):
    """Return (present, description) for a likely LoRa board on `port`.

    A live /dev node is the primary signal. If pyserial's list_ports is
    available we also confirm the port is a currently-enumerated device (so a
    stale /dev entry doesn't read as present) and grab a human description."""
    try:
        from serial.tools import list_ports
    except Exception:
        return os.path.exists(port), None
    for p in list_ports.comports():
        if p.device == port:
            return True, (p.description or None)
    # list_ports didn't enumerate it; fall back to raw existence.
    return os.path.exists(port), None


def rtl_sdr_present():
    """True if an RTL-SDR dongle appears in lsusb. Falls back to checking that
    rtl_test is installed when lsusb isn't available."""
    lsusb = shutil.which("lsusb")
    if lsusb:
        try:
            out = subprocess.run([lsusb], capture_output=True, text=True,
                                 timeout=5).stdout.lower()
            return any(uid in out for uid in RTL_SDR_USB_IDS)
        except Exception:
            pass
    # No lsusb (or it failed): best we can do without opening the device.
    return shutil.which("rtl_test") is not None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    # Dispatch + serial-path options
    ap.add_argument("--port", default="/dev/ttyACM0",
                    help="serial port to probe for the LoRa board (default /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=BAUD,
                    help=f"serial baud rate for the LoRa board (default {BAUD})")
    ap.add_argument("--force", choices=("serial", "sdr"),
                    help="skip auto-detection and use this path")

    # SDR-path options (used only when the SDR path runs)
    ap.add_argument("--file", help="read IQ from this file instead of driving the SDR (replay/test)")
    ap.add_argument("--format", choices=list(FORMATS), default="fc32",
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
        return serial_main(args)
    if args.force == "sdr":
        print("forced SDR path")
        return sdr_main(args)

    # 2) SDR replay from a file needs no hardware.
    if args.file:
        print("SDR replay (--file) -> SDR path")
        return sdr_main(args)

    # 3) Auto-detect.
    have_serial, desc = serial_present(args.port)
    have_sdr = rtl_sdr_present()
    board = args.port + (f" ({desc})" if desc else "")

    if have_serial and have_sdr:
        print(f"Both devices detected: serial board at {board} AND an RTL-SDR.",
              file=sys.stderr)
        print("Refusing to guess -- choose one with: "
              "--force serial   or   --force sdr", file=sys.stderr)
        return 2
    if have_serial:
        print(f"serial board detected at {board} -> serial path")
        return serial_main(args)
    if have_sdr:
        print("RTL-SDR detected -> SDR path")
        return sdr_main(args)

    print(f"No receiver found: no serial board at {args.port} and no RTL-SDR.",
          file=sys.stderr)
    print("Attach a LoRa board (or pass --port), plug in an RTL-SDR, or use "
          "--file <recording> / --force {serial,sdr}.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
