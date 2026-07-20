"""Serial (LoRa32-board) receive path.

The board running receiver.ino appears as a USB-serial device and prints every
LoRa packet as a hex line. This module brackets each image between the header and
fingerprint packets, reassembles the JPEG (v2 with Reed-Solomon FEC, v1 blind-
concatenation fallback), and -- unlike the listen-only SDR path -- can push config
requests back to the sender and wait for its ACK. `serial_main` is the driver.

The shared wire/CRC/v2-reassembly/JPEG/UI code lives in `lib.helpers`; only the
serial-specific pieces (the two-way config link, the interactive prompt, the
board-relayed request/ACK, the per-packet progress) are here. `cv2` and `serial`
are imported lazily inside the functions that need them, so a machine set up only
for the SDR path never needs opencv/pyserial.
"""
import os
import time

import numpy as np

from lib import helpers

BAUD = 115200                                # must match receiver.ino's Serial.begin()

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
    return body + helpers.crc16_ccitt(body).to_bytes(2, "little")


def parse_ack(pkt):
    """Return the ACK value if pkt is a valid ACK packet, else None."""
    if len(pkt) == 8 and pkt[:2] == ACK_MAGIC \
       and helpers.crc16_ccitt(pkt[:6]) == (pkt[6] | (pkt[7] << 8)):
        return int.from_bytes(pkt[2:6], "little")
    return None


def reassemble(packets):
    """Reassemble one burst's packets -> (jpeg_bytes_or_None, stats). Auto-detects v2
    (params packet present) vs legacy v1 (blind concatenation)."""
    params = helpers.find_params(packets)
    if params is not None:
        return helpers.reassemble_v2(packets, params)
    # v1 fallback: concatenate raw payloads and slice SOI..EOI
    jpg = helpers.extract_jpeg(b"".join(packets))
    return jpg, dict(v2=False, packets=len(packets))


def show_and_save(jpg, stats, ui, outdir="."):
    if not jpg:
        ui.log(f"  no image ({stats}) -- skipped")
        return
    arr = np.frombuffer(jpg, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        ui.log(f"  JPEG undecodable ({stats}) -- skipped")
        return
    # Same layout as the SDR path: <outdir>/<YYYYMMDD UTC>/<unix>.png, new dir per UTC day.
    path = helpers.dated_png_path(outdir, int(time.time()))
    cv2.imwrite(path, img)
    if stats.get("v2"):
        ui.log(f"  saved {img.shape[1]}x{img.shape[0]} -> {path}  "
               f"[v2 Ndata={stats['Ndata']} present={stats['present']} "
               f"recovered={stats['recovered']} unrecoverable={stats['unrecoverable']} "
               f"crc_fail={stats['crc_fail']}]")
    else:
        ui.log(f"  saved {img.shape[1]}x{img.shape[0]} -> {path}  [v1 {stats['packets']} pkts]")


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


def _prompt_config_text():
    """Plain line-by-line fallback used when a curses TUI can't run (non-tty host or
    curses unavailable). Returns (mode, res, delay, count) selections."""
    print("Mode:  0) continuous   1) specific count")
    mode = _ask_int("Mode", 0, 1, 0)
    print("Resolution:")
    for i, name in enumerate(RES_NAMES):
        dims = RES_DIMS.get(name)
        print(f"  {i:2d}) {name}" + (f" ({dims})" if dims else ""))
    res = _ask_int("Resolution index", 0, len(RES_NAMES) - 1, RES_DEFAULT)
    delay = _ask_int("Delay between frames (s)", 0, 32767, 5)
    count = _ask_int("Count (images)", 0, 4095, 1) if mode == 1 else 0
    return mode, res, delay, count


def _console_config():
    """Non-TUI config prompt (the [y/N] gate + plain text fields), used by ConsoleUI /
    non-tty hosts. Returns a selections dict {mode,res,delay,count} or None to listen."""
    if input("\nSend a new configuration? [y/N] ").strip().lower() not in ("y", "yes"):
        return None
    mode, res, delay, count = _prompt_config_text()
    return dict(mode=mode, res=res, delay=delay, count=count)


def cfg_from_selection(sel):
    """Pack a selections dict into the 32-bit config word (count ignored unless the
    mode is 'specific count'), returning (cfg, summary_line)."""
    mode, res, delay = sel["mode"], sel["res"], sel["delay"]
    count = sel["count"] if mode == 1 else 0
    cfg = pack_config(mode, res, delay, count)
    summary = (f"  -> config 0x{cfg:08X}  (mode={'continuous' if mode == 0 else 'count'}, "
               f"res={RES_NAMES[res]}, delay={delay}s, count={count})")
    return cfg, summary


def wait_for_ack(ser, expected, timeout, ui):
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
        ui.log(f"  ACK mismatch: got 0x{ack:08X}, expected 0x{expected:08X}")
    return False


def send_request(ser, cfg, ui, budget=120.0):
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
        ui.log(f"  request sent (attempt {attempt}); waiting for ACK...")
        if wait_for_ack(ser, expected, timeout=6.0, ui=ui):
            ui.log("  ACK OK")
            return True
    ui.log("  no ACK -- sender offline or never reached a listen window; back to menu")
    return False


def receive_images(ser, ui, n=None, idle_finalize=3.0, outdir=".", progress=True):
    """Collect header..fingerprint bursts and save each. Stop after n images (n=None:
    forever), or early if `ui.check_quit()` reports the user asked to return to the menu.
    Returns the number of images saved. Same reassembly as before, refactored so a config
    run can bound it to n. If the burst's single (unprotected) fingerprint packet is lost,
    finalize the image after `idle_finalize` s of silence -- intra-image gaps are ~10 ms,
    so this can't fire mid-image, and it prevents a count run from hanging on a dropped
    trailing fingerprint. When `progress`, a live packet bar is shown for each image."""
    collecting, packets, got, last_rx = False, [], 0, time.time()
    recv = 0            # data/parity (250-byte) packets seen this image
    total = None        # expected data+parity count, learned from the params packet

    def show_progress():
        if not progress:
            return
        if total:
            ui.progress(min(1.0, recv / total), "receiving", f"{recv}/{total} pkts")
        else:
            ui.progress(None, "receiving", f"{recv} pkts (reading params)")

    def finalize(msg):
        nonlocal got
        if progress:
            show_progress()
            ui.end_progress()
        if msg:
            ui.log(msg)
        show_and_save(*reassemble(packets), ui=ui, outdir=outdir)
        got += 1

    while n is None or got < n:
        if ui.check_quit():                  # user pressed q -- back to the menu
            break
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
        if pkt[:5] == helpers.HEADER:
            # New image starting. If the previous one never got its (single, unprotected)
            # fingerprint packet, finalize it now -- with v2 the params+data+parity suffice.
            if collecting and packets:
                finalize("  (no fingerprint seen -- finalizing on next header)")
                if n is not None and got >= n:
                    break
            ui.log(f"[{time.strftime('%H:%M:%S')}] header -- receiving image")
            collecting, packets, recv, total = True, [], 0, None
            continue
        if not collecting:
            continue
        if pkt[:4] == helpers.FINGERPRINT:
            collecting = False
            finalize(None)
            continue
        packets.append(pkt)
        if total is None:                       # first CRC-valid params packet sets the total
            params = helpers.parse_params(pkt)
            total = helpers.params_total(params) if params else None
        if len(pkt) == helpers.LORA_PAYLOAD:    # a data or parity packet arrived
            recv += 1
            show_progress()
    return got


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


def serial_main(args):
    """Interactive LoRa config link: optionally push a config to the sender, then save the
    reassembled image(s). Declining a config falls back to a passive save loop.

    On a real terminal the whole session runs inside a curses Dashboard (lib.ui): the
    config form opens modally, and headers / progress / FEC results stream into the
    dashboard's log. 'q' during a receive returns to the config form; Ctrl-C quits. On a
    non-tty host a ConsoleUI reproduces the original plain prompts and stdout/stderr."""
    global cv2
    import cv2                                            # noqa: F401 (bound to global, used by show_and_save)
    import serial
    from lib.ui import make_ui

    ser = serial.Serial(args.port, args.baud, timeout=1)
    ui = make_ui("serial", args.port)
    progress = not args.no_progress

    def receive(n, status):
        """Run a receive loop, letting the Dashboard's Ctrl-C quit the whole session
        while ConsoleUI's Ctrl-C just returns to the menu (the original behaviour).
        Returns images saved."""
        ui.status(status)
        try:
            return receive_images(ser, ui, n, outdir=args.outdir, progress=progress)
        except KeyboardInterrupt:
            if ui.interactive_form:
                raise                                    # dashboard: Ctrl-C -> quit
            ui.end_progress()                            # console: Ctrl-C -> back to menu
            return 0

    from lib import tui
    try:
        ui.log(f"Connected to {args.port} -- LoRa config link")
        while True:
            sel = (ui.config_form(RES_NAMES, RES_DIMS, RES_DEFAULT)
                   if ui.interactive_form else _console_config())
            if sel == tui.QUIT:                          # Exit button -> quit program
                break
            if sel is None:                              # declined -> passive listen
                receive(None, "Listening for images (q returns to menu)...")
                continue
            cfg, summary = cfg_from_selection(sel)
            ui.log(summary)
            if not send_request(ser, cfg, ui):
                continue
            if cfg & 0x1:                                 # specific count
                n = max(1, (cfg >> 20) & 0xFFF)
                if receive(n, f"Waiting for {n} image(s)...") >= n:
                    ui.log(f"Received {n} image(s).")
            else:                                         # continuous
                receive(None, "Continuous mode -- receiving (q returns to menu)...")
    except KeyboardInterrupt:
        ui.log("stopped")
    finally:
        ui.close()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
