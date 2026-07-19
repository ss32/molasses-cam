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
the SDR path also needs decode_lora_image (and rtl_sdr on PATH). Each path's
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
BAUD = 38400

LORA_PAYLOAD = 250
V2_PAYLOAD = 246                              # 250 - 2 seq - 2 crc
PARAMS_MAGIC = bytes([0x70, 0x32])
PARAMS_LEN = 15


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


def show_and_save(jpg, stats):
    if not jpg:
        print(f"  no image ({stats}) -- skipped")
        return
    arr = np.frombuffer(jpg, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        print(f"  JPEG undecodable ({stats}) -- skipped")
        return
    path = f"received_{int(time.time())}.png"
    cv2.imwrite(path, img)
    if stats.get("v2"):
        print(f"  saved {img.shape[1]}x{img.shape[0]} -> {path}  "
              f"[v2 Ndata={stats['Ndata']} present={stats['present']} "
              f"recovered={stats['recovered']} unrecoverable={stats['unrecoverable']} "
              f"crc_fail={stats['crc_fail']}]")
    else:
        print(f"  saved {img.shape[1]}x{img.shape[0]} -> {path}  [v1 {stats['packets']} pkts]")


def serial_main(args):
    """Read the LoRa board's hex packet stream and save each reassembled image."""
    global cv2, rs
    import cv2                                            # noqa: F401 (bound to global)
    import rs_gf256 as rs                                 # noqa: F401 (bound to global)
    import serial

    ser = serial.Serial(args.port, args.baud)
    print("Connected to", args.port, "-- waiting for images (Ctrl-C to stop)")
    collecting, packets = False, []
    try:
        while True:
            line = ser.readline().strip()
            if not line:
                continue
            try:
                pkt = bytes.fromhex(line.decode("ascii", "ignore"))   # case-insensitive
            except ValueError:
                continue                                              # boot/log line, skip
            if pkt[:5] == HEADER:
                # A new image is starting. If the previous one never got its (single,
                # unprotected) fingerprint packet, finalize it now from what we
                # collected -- with v2 the params + data + parity are enough anyway.
                if collecting and packets:
                    print("  (no fingerprint seen -- finalizing on next header)")
                    show_and_save(*reassemble(packets))
                print(f"[{time.strftime('%H:%M:%S')}] header -- receiving image")
                collecting, packets = True, []
                continue
            if not collecting:
                continue
            if pkt[:4] == FINGERPRINT:
                collecting = False
                show_and_save(*reassemble(packets))
                continue
            packets.append(pkt)
    except KeyboardInterrupt:
        print("\nstopped")
        cv2.destroyAllWindows()


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


def sniff_count(blocks, rate, shared):
    """Decode just the start of a burst to read the transmitted packet count. Runs in
    a side thread so it can't stall the SDR reader. Sets shared['N'] once known.
    Handles both v2 (params packet: Ndata + K/M parity) and legacy v1 (2-byte count)."""
    try:
        packets = D.decode_packets(D.resample_to_lora(np.concatenate(blocks), rate))
        # v2: params packet gives Ndata, K, M -> total = Ndata + ceil(Ndata/K)*M
        for p in packets:
            if len(p) >= D.PARAMS_LEN and bytes(p[:2]) == D.PARAMS_MAGIC \
               and D.crc16_ccitt(bytes(p[:13])) == (p[13] | (p[14] << 8)):
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
            jpg, npkt, stats = D.decode_iq(iq, args.rate, pool=pool)
        except Exception as e:
            log(f"decode error: {e}")
            q.task_done(); continue
        if not jpg:
            log(f"burst @{start_unix}: no image [{D.stats_summary(stats)}] "
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
            f"[{D.stats_summary(stats)}] decode {time.time()-t0:.1f}s -> {path}")
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
    global D
    import decode_lora_image as D                         # noqa: F401 (bound to global)

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
