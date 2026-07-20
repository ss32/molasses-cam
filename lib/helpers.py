"""Code shared by both receive paths (serial LoRa board and RTL-SDR).

Everything in here is used by *both* `lib.lora_serial` and `lib.sdr`: the wire
constants, the CCITT CRC, protocol-v2 params parsing, the Reed-Solomon v2
reassembly, JPEG slicing, and the small UI/IO helpers (progress-bar text, dated
PNG paths, timestamped logging). Keeping the single copy here is the point of the
refactor -- the two paths used to carry near-identical versions of each.

Only lightweight dependencies (numpy + rs_gf256, which itself needs only numpy)
are imported here, so importing this module never pulls a path's heavy deps
(scipy / opencv / PIL); those stay lazily imported inside the path modules.
"""
import os
import time

import numpy as np

import rs_gf256 as rs

# ---------------------------------------------------------------------------
# Shared wire-protocol constants
# ---------------------------------------------------------------------------
HEADER = bytes([0x61, 0x79, 0x79, 0x79, 0x79])       # precedes every image
FINGERPRINT = bytes([0x6c, 0x6d, 0x61, 0x6f])        # closes a serial-bridged image

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


# ---------------------------------------------------------------------------
# Protocol-v2 params packet: [0x70 0x32][ver][image_len:4 LE][Ndata:2][K:2][M:2][crc16:2]
# ---------------------------------------------------------------------------
def parse_params(pkt):
    """Decode a v2 params packet, or return None if `pkt` isn't one. The magic plus
    the CRC over the first 13 bytes is decisive, so we don't insist on an exact
    length (short control packets sometimes decode with a wrong length nibble)."""
    d = bytes(pkt)
    if len(d) >= PARAMS_LEN and d[:2] == PARAMS_MAGIC \
       and crc16_ccitt(d[:13]) == (d[13] | (d[14] << 8)):
        return dict(version=d[2],
                    image_len=int.from_bytes(d[3:7], "little"),
                    Ndata=d[7] | (d[8] << 8),
                    K=d[9] | (d[10] << 8),
                    M=d[11] | (d[12] << 8))
    return None


def find_params(packets):
    """Return the params dict from the first CRC-valid params packet in `packets`
    (an iterable of byte-like packets), or None if the burst isn't protocol v2."""
    for pkt in packets:
        params = parse_params(pkt)
        if params is not None:
            return params
    return None


def params_total(params):
    """Total data+parity packet count a v2 burst sends: Ndata + ceil(Ndata/K)*M
    (same arithmetic as receiver.ino / sender.ino)."""
    Ndata, K, M = params["Ndata"], params["K"], params["M"]
    nblocks = (Ndata + K - 1) // K if K else 0
    return Ndata + nblocks * M


def reassemble_v2(packets, params, verbose=False):
    """Protocol-v2 reassembly shared by both receive paths.

    `packets` is an iterable of raw 250-byte LoRa payloads (the caller filters to
    the exact length / its own accept rules first). Each is CRC-checked and split
    into data (seq < Ndata) and parity (seq >= Ndata) rows; every block is then
    Reed-Solomon erasure-decoded to reconstruct up to M missing/corrupt data
    packets, zero-filling anything unrecoverable. Returns (jpeg_bytes, stats) where
    jpeg_bytes is the first image_len bytes of the reassembled stream (callers may
    still SOI..EOI-trim via extract_jpeg). `stats` carries the superset of keys the
    two paths print."""
    Ndata, K, M, image_len = params["Ndata"], params["K"], params["M"], params["image_len"]
    nblocks = (Ndata + K - 1) // K if K else 0
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
            except Exception as e:
                if verbose:
                    print(f"  block {b}: RS decode failed ({e})")
        unrecoverable += len(missing)
        for j, row in have_data.items():
            out[(lo + j) * V2_PAYLOAD:(lo + j + 1) * V2_PAYLOAD] = row

    jpg = bytes(out[:image_len])
    stats = dict(v2=True, Ndata=Ndata, count=Ndata, present=present, accepted=present,
                 missing=Ndata - present, recovered=recovered,
                 unrecoverable=unrecoverable, crc_fail=crc_fail,
                 rej_len=0, rej_parity=0, missing_idx=[])
    if verbose:
        print(f"  v2: Ndata={Ndata} K={K} M={M} present={present} "
              f"recovered={recovered} unrecoverable={unrecoverable} crc_fail={crc_fail}")
    return jpg, stats


def extract_jpeg(blob):
    """Return the JPEG bytes (SOI..EOI) inside `blob`, or None if not found."""
    soi, eoi = blob.find(b"\xff\xd8"), blob.rfind(b"\xff\xd9")
    if soi != -1 and eoi != -1 and eoi > soi:
        return blob[soi:eoi + 2]
    return None


# ---------------------------------------------------------------------------
# Small UI / IO helpers
# ---------------------------------------------------------------------------
def progress_bar(frac, width=24):
    """Render the `[####....]` fill for a fraction in [0,1] (both paths' bars)."""
    fill = int(frac * width)
    return "#" * fill + "." * (width - fill)


def dated_png_path(outdir, unix):
    """Return <outdir>/<YYYYMMDD UTC>/<unix>.png, creating the day dir. Shared image
    layout for both paths -- a new directory per UTC day, files named by Unix seconds."""
    day = os.path.join(outdir, time.strftime("%Y%m%d", time.gmtime(unix)))
    os.makedirs(day, exist_ok=True)
    return os.path.join(day, f"{unix}.png")


def log(msg):
    """Timestamped (UTC) line to stdout, flushed -- used by the SDR path's logging."""
    print(f"[{time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())}Z] {msg}", flush=True)
