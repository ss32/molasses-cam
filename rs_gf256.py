#!/usr/bin/env python3
"""Systematic Reed-Solomon erasure code over GF(256), used to recover whole LoRa
packets that arrive missing or CRC-failed.

The image's data packets are grouped into blocks of K packets; for each block the
transmitter adds M parity packets computed as `parity = A . data` over GF(256), where
A is an M x K Cauchy matrix. Because a Cauchy matrix makes the systematic generator
[I_K ; A] an MDS code, ANY K of the K+M packets in a block recover the original K data
packets -- i.e. up to M lost/corrupt packets per block are reconstructed exactly.

The coding is applied independently to each byte-column of the packets, so a whole
lost packet is a single erased symbol shared by every column. This module is the
Python (decode) side; espArducam_lora32.ino carries a byte-identical encoder (same
primitive polynomial 0x11d, same Cauchy construction) -- keep them in lock-step and
covered by test_rs_gf256.py.
"""
import numpy as np

PRIM = 0x11d  # primitive polynomial x^8+x^4+x^3+x^2+1 (must match the ESP encoder)

# ---- GF(256) exp/log tables (generator alpha = 2) ----
GF_EXP = np.zeros(512, dtype=np.uint8)
GF_LOG = np.zeros(256, dtype=np.uint16)
_x = 1
for _i in range(255):
    GF_EXP[_i] = _x
    GF_LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= PRIM
for _i in range(255, 512):
    GF_EXP[_i] = GF_EXP[_i - 255]


def gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return int(GF_EXP[GF_LOG[a] + GF_LOG[b]])


def gf_inv(a):
    if a == 0:
        raise ZeroDivisionError("GF inverse of 0")
    return int(GF_EXP[255 - GF_LOG[a]])


# Full 256x256 multiply table for vectorized GF matrix ops (64 KB).
_MULT = np.zeros((256, 256), dtype=np.uint8)
for _a in range(256):
    for _b in range(256):
        _MULT[_a, _b] = gf_mul(_a, _b)


def cauchy_matrix(m, k):
    """M x K Cauchy matrix over GF(256): A[i][j] = 1 / (x_i ^ y_j), with the x_i
    (parity ids) and y_j (data ids) drawn from disjoint element sets. Requires
    m + k <= 256. Deterministic, so encoder and decoder agree."""
    if m + k > 256:
        raise ValueError(f"m+k={m+k} exceeds GF(256) capacity of 256")
    x = np.arange(k, k + m, dtype=np.int64)   # parity-row ids
    y = np.arange(0, k, dtype=np.int64)       # data-col ids
    A = np.zeros((m, k), dtype=np.uint8)
    for i in range(m):
        for j in range(k):
            A[i, j] = gf_inv(int(x[i]) ^ int(y[j]))
    return A


def _gf_matmul(mat, vecs):
    """(rows x k) GF matrix times (k x L) byte block -> (rows x L). Vectorized via
    the multiply table and XOR-reduce."""
    rows, k = mat.shape
    L = vecs.shape[1]
    out = np.zeros((rows, L), dtype=np.uint8)
    for i in range(rows):
        acc = np.zeros(L, dtype=np.uint8)
        for j in range(k):
            c = mat[i, j]
            if c:
                acc ^= _MULT[c, vecs[j]]
        out[i] = acc
    return out


def encode(data_block, m):
    """data_block: (k, L) uint8 (k data packets, L bytes each). Returns (m, L) parity
    packets. This mirrors the ESP encoder exactly."""
    data_block = np.ascontiguousarray(data_block, dtype=np.uint8)
    k = data_block.shape[0]
    return _gf_matmul(cauchy_matrix(m, k), data_block)


def _gf_inv_matrix(mat):
    """Invert a KxK GF(256) matrix via Gauss-Jordan. Raises if singular."""
    k = mat.shape[0]
    a = mat.astype(np.uint8).copy()
    inv = np.eye(k, dtype=np.uint8)
    for col in range(k):
        p = col
        while p < k and a[p, col] == 0:
            p += 1
        if p == k:
            raise np.linalg.LinAlgError("singular GF matrix")
        if p != col:
            a[[col, p]] = a[[p, col]]
            inv[[col, p]] = inv[[p, col]]
        ipiv = gf_inv(int(a[col, col]))
        a[col] = _MULT[ipiv, a[col]]
        inv[col] = _MULT[ipiv, inv[col]]
        for r in range(k):
            if r != col and a[r, col]:
                f = a[r, col]
                a[r] ^= _MULT[f, a[col]]
                inv[r] ^= _MULT[f, inv[col]]
    return inv


def decode(received, k, m, erased):
    """Recover the k data packets from a block with erasures.

    received : (k+m, L) uint8; rows 0..k-1 are data packets, k..k+m-1 are parity.
               Erased rows may hold anything (they're ignored).
    erased   : iterable of erased row indices (0..k+m-1). Must have <= m entries.
    Returns  : (k, L) uint8 reconstructed data packets.
    """
    received = np.ascontiguousarray(received, dtype=np.uint8)
    erased = set(int(e) for e in erased)
    if len(erased) > m:
        raise ValueError(f"{len(erased)} erasures exceed m={m}; unrecoverable")
    # systematic generator G = [I_k ; A]
    A = cauchy_matrix(m, k)
    G = np.vstack([np.eye(k, dtype=np.uint8), A])
    # pick the first k non-erased rows
    present = [r for r in range(k + m) if r not in erased][:k]
    sub = G[present]                                   # (k, k)
    inv = _gf_inv_matrix(sub)
    data = _gf_matmul(inv, received[present])          # recovered k data packets
    return data
