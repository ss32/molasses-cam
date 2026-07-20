# molasses-cam: On-Demand imagery over LoRa, slow as molasses

Capture JPEGs from an **Arducam MEGA** SPI camera on a **LilyGo/TTGO LoRa32 T3 v1.6.1**
(ESP32 + SX127x + SSD1306) and ship them over LoRa with forward error
correction. 

Two nodes:

- **Sender** (`sender/sender.ino`) LoRa32 with the Arducam MEGA. Captures and
  transmits images; listens for config requests between frames and ACKs them.
- **Receiver**, one of:
  - a second LoRa32 **receiver node** (`receiver/receiver.ino`, no camera) on USB serial,
    driven by `receive.py`. 
      - **This is the only path that can send config requests and get imagery on-demand.**
  - an **RTL-SDR** dongle, decoded passively by `receive.py`. Listen-only, it can
    receive images but cannot request them.

---

## Pinout (both boards are the same LoRa32 T3 v1.6.1)

The **sender** needs the camera wired up. The **receiver node** needs no wiring at
all, just the on-board radio + USB.

```
  Arducam MEGA                 LoRa32 T3 v1.6.1  (SENDER only)
  ┌─────────┐
  │ VCC ────┼──── 3V3
  │ GND ────┼──── GND
  │ SCK ────┼──── GPIO14  ┐
  │ MISO ───┼──── GPIO36  │  camera SPI  (VSPI = the SD-card bus)
  │ MOSI ───┼──── GPIO15  │  GPIO36 = SVP, input-only, non-strapping
  │ CS  ────┼──── GPIO13  ┘
  └─────────┘
                             on-board, no wiring needed (both boards):
                             SX127x LoRa (HSPI): SCK5  MISO19 MOSI27 CS18 RST23 DIO0-26
                             SSD1306 OLED (I2C):  SDA21 SCL22
                             MicroSD (shares VSPI): MISO=GPIO2, CS=GPIO13
```

**Radio PHY** (must match on every node): 915 MHz, SF7, **BW 500 kHz**, CR 4/5, sync
`0x12`. 


## Quickstart


1. `cd $HOME && git clone https://github.com/ArduCAM/Arducam_Mega.git`
2. Plug in the board that will be the sender
```bash
cd sender && bash ../upload.sh "" /dev/ttyACM0
```
3. Unplug the board from the computer
4. Wire up the camera
5. Plug in the board that will be the receiver
```bash
cd receiver && bash ../upload.sh "" /dev/ttyACM0
```
6. `python3 receive.py`

---

# Deeper Dive

## 1. Firmware

Prereqs: [`arduino-cli`](https://arduino.github.io/arduino-cli/) with the **esp32** core
(`arduino-cli core install esp32:esp32`) and the **Arducam_Mega** library cloned to
`~/Arducam_Mega`.

Use `upload.sh` to compile the sketch in the current directory and flash it. Flash each board from its own sketch dirrectory.

```bash
cd sender   && bash ../upload.sh "" /dev/ttyACM0    # camera node
cd receiver && bash ../upload.sh "" /dev/ttyACM1    # receive node (skip if using an SDR)
```

### Knobs you can turn, transmit side

| Constant | Default | Meaning |
|---|---|---|
| `BOOT_CONTINUOUS` | `true` | `true`: boot into a continuous timer, still listening for requests in each gap. `false`: boot idle and send nothing until asked. |
| `REQUEST_TO_CAPTURE_DELAY_S` | `5` | **N**,  firmware delay from sending the ACK to the first capture. |
| `LORA_BW` | `500E3` | Signal bandwidth; must match the receiver and `--bw` |
| `SAVE_TO_SD` / `USE_OLED` | `true` | MicroSD copy of each frame / OLED status text. |


**Serial link is 115200 baud.** `receiver.ino` and `receive.py` are both set to 115200; keep them in sync if you change it, but don't change it.

---

## 2. Python receiver

```
usage: receive.py [-h] [--port PORT] [--baud BAUD] [--force {serial,sdr}] [--file FILE] [--format {fc32,cu8,cs16}] [--rate RATE] [--bw BW] [--drive-sdr] [--freq FREQ] [--gain GAIN]
                  [--device DEVICE] [--ppm PPM] [--outdir OUTDIR] [--threshold-db THRESHOLD_DB] [--gap GAP] [--min-burst MIN_BURST] [--max-burst MAX_BURST] [--realtime] [--no-progress]
                  [--workers WORKERS]

Self-contained LoRa image receiver that auto-selects its input.

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

options:
  -h, --help            show this help message and exit
  --port PORT           serial port to probe for the LoRa board (default /dev/ttyACM0)
  --baud BAUD           serial baud rate for the LoRa board (default 115200)
  --force {serial,sdr}  skip auto-detection and use this path
  --file FILE           read IQ from this file instead of driving the SDR (replay/test)
  --format {fc32,cu8,cs16}
                        IQ format for --file / piped stdin (default fc32; SDR mode is always cu8)
  --rate RATE           sample rate (Hz)
  --bw BW               LoRa bandwidth Hz (default: decoder's BW); must match the transmitter
  --drive-sdr           on the SDR path, drive rtl_sdr even when stdin isn't a TTY (headless/service)
  --freq FREQ           SDR centre frequency (Hz)
  --gain GAIN           tuner gain in dB, or 'auto' for AGC (default auto)
  --device, -d DEVICE   rtl_sdr device index (default 0)
  --ppm PPM             SDR frequency correction (ppm)
  --outdir OUTDIR       base output dir (YYYYMMDD dirs go here)
  --threshold-db THRESHOLD_DB
                        power above noise floor (dB) that starts a burst
  --gap GAP             silence (s) that ends a burst (must exceed inter-packet gaps)
  --min-burst MIN_BURST
                        ignore active bursts shorter than this (s)
  --max-burst MAX_BURST
                        force-close a burst after this long (s)
  --realtime            with --file, replay at real time so the progress bar is meaningful
  --no-progress         disable the live progress bar
  --workers WORKERS     parallel decode processes (1 = decode in-thread)
```

Prerequisites:

```bash
python3 -m pip install numpy pyserial opencv-python scipy      
```

`receive.py` auto-detects whether a serial LoRa node or an RTL-SDR is attached and reacts accordingly.

### Interactive LoRa Serial Option

Make sure to point it at the **receive node** if both boards are on the same machine

```bash
python3 ./receive.py --port /dev/ttyACM1
```

It prompts each round:

```
Send a new configuration? [y/N]
```

- **No** --> passively save every image the sender streams (Ctrl-C returns to the prompt).
- **Yes** --> pick **Mode** (continuous / specific count), **Resolution** (0–13, table
  below), **Delay** between frames (s), and **Count**. It sends the request, waits for the
  sender's ACK (resending until the sender reaches a listen window), then:
  - *specific count* --> saves exactly `COUNT` images, then re-prompts;
  - *continuous* --> saves images until Ctrl-C.

Images are written as `<YYYYMMDD>/<unix>.png` timestamped with Unix seconds. 

### SDR: passive, listen-only

```bash
./receive.py --force sdr --gain 40           # drive an RTL-SDR (spawns rtl_sdr, only necessary if SDR is present with a LoRa board)
./receive.py --file rec.raw --rate 1.8e6     # replay a gqrx/GNU Radio fc32 recording
```

The SDR can't transmit, so it never sends configs, it
just decodes whatever the sender puts on the air. 



---

## Resolution index table

`RESOLUTION` is a 4-bit index into this list (identical order in `sender.ino`'s
`RES_TABLE` and `receive.py`'s `RES_NAMES`):

| # | Mode | # | Mode | # | Mode |
|---|---|---|---|---|---|
| 0 | 96×96 | 5 | VGA (640×480) | 10 | UXGA (1600×1200) |
| 1 | 128×128 | 6 | SVGA (800×600) | 11 | FHD (1920×1080) |
| 2 | QQVGA (160×120) | 7 | 1024×768 | 12 | QXGA (2048×1536) |
| 3 | QVGA (320×240) | 8 | **HD (1280×720)** ← default | 13 | WQXGA2 (2592×1944) |
| 4 | 320×320 | 9 | 1280×1024 | | |

Higher resolutions mean much longer airtime (~2 packets/s); the camera also over-reports
some large modes. 
QVGA/VGA are good for quick on-demand shots.

---

## Wire protocol

**Config request** (receive node --> sender) and **ACK** (sender --> node) are 8-byte packets:
`[MAGIC:2][value:4 LE][crc16:2]` (CRC-16/CCITT-FALSE), `REQ_MAGIC = 72 71`,
`ACK_MAGIC = 72 63`. The receive node forwards them verbatim; the format lives in
`sender.ino` and `receive.py`.

The 32-bit config value, LSB --> MSB

| Field | Bits | Range | Meaning |
|---|---|---|---|
| `MODE` | 0 | 0/1 | 0 = continuous, 1 = specific count |
| `RESOLUTION` | 1–4 | 0–13 | index into the table above |
| `DELAY` | 5–19 | 0–32767 | seconds between frames (timer starts at the last packet) |
| `COUNT` | 20–31 | 0–4095 | frames to send (0 or 1 --> exactly one); ignored in continuous mode |

**ACK value** = `config - (DELAY x COUNT) + RESOLUTION` (32-bit wrap)

**Image stream** (protocol v2, per frame): header `61 79 79 79 79` --> a params packet
(`70 32`, sent 3×: version, image length, `Ndata`, `K=32`, `M=4`, CRC16) --> `Ndata` data
packets `[seq:2 LE][246 JPEG bytes][crc16:2]` --> `M` Reed–Solomon parity packets per block
of `K` --> fingerprint `6c 6d 61 6f`. Any `M` lost/corrupt packets per block are recovered.
The GF(256) math is byte-identical between `sender.ino` and `rs_gf256.py`.

---

## Flow at a glance

```
  receive.py ──"TX<hex>"──▶ receiver.ino ──LoRa request──▶ sender.ino
     (menu)     (serial)       (relay)                     (listens in DELAY gap)
     ▲                                                          │
     │  images (hex over serial)          ACK, wait N s, then   │
     └──────────── receiver.ino ◀───LoRa image + ACK──────── capture per config
```

## Gotchas

- Both LoRa32 boards show up as `/dev/ttyACM*`. Flash the sender first (ACM0), the relay
  node second (ACM1), and give `receive.py` `--port /dev/ttyACM1`.
- If a request seems ignored, the sender is mid-image (half-duplex), `receive.py` keeps
  resending until the sender's next listen window; a long transfer may take a bit.
- All nodes must share the same `LORA_BW`; the SDR path also needs `--bw` to match.
- `upload.sh` can fail if a MicroSD card is seated in the target board. Remove it to flash.
