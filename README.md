# slow-loras: On-Demand imagery over LoRa, slowly

![slow_loris](slow_loris.svg)



## Hardware

* [LilyGo Lora32](https://lilygo.cc/products/lora3)
  * x2 if you want on-demand imagery
* Any SDR (optional, receive-only from the camera)
  * Tested with [RTL-SDR v3](https://www.rtl-sdr.com/buy-rtl-sdr-dvb-t-dongles/)
* [Arducam Mega SPI Camera](https://www.amazon.com/Arducam-Mega-Camera-Module-Microcontroller/dp/B0BW4L21KS?th=1)


Two nodes:

- **Sender** LoRa32 with the Arducam MEGA. Captures and
  transmits images; listens for config requests between frames and ACKs them.
- **Receiver**, one of:
  - a second unmodified LoRa32 **receiver node** on USB serial. 
  - an **RTL-SDR** dongle. Listen-only, it can
    receive images but cannot request them.

---

## Pinout (sender node only)

* The **sender** needs the camera wired up. 

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




## Quickstart


1. `cd $HOME && git clone https://github.com/ArduCAM/Arducam_Mega.git`
2. `python3 -m pip install numpy pyserial opencv-python scipy`
3. Wire up the camera
4. Plug in the board that will be the sender
```bash
cd sender && bash ../upload.sh "" /dev/ttyACM0
```
5. Wire up the camera
6. Plug in the device that will be the receiver
```bash
cd receiver && bash ../upload.sh "" /dev/ttyACM1 # Only if using a LoRa board, not required for SDR
```
7. `python3 receive.py`

---

# Deeper Dive

## 1. Firmware

Prereqs: [`arduino-cli`](https://arduino.github.io/arduino-cli/) with the **esp32** core
(`arduino-cli core install esp32:esp32`) and the **Arducam_Mega** library cloned to
`~/Arducam_Mega`.

Use `upload.sh` to compile the sketch in the current directory and flash it. Flash each board from its own sketch directory.

```bash
cd sender   && bash ../upload.sh "" /dev/ttyACM0    # camera node
cd receiver && bash ../upload.sh "" /dev/ttyACM1    # receive node (skip if using an SDR)
```

### Knobs you can turn, transmit side ESP code

| Constant | Default | Meaning |
|---|---|---|
| `BOOT_CONTINUOUS` | `true` | `true`: boot into a continuous timer, still listening for requests in each gap. `false`: boot idle and send nothing until asked. |
| `REQUEST_TO_CAPTURE_DELAY_S` | `5` | Seconds, wait this long before capturing an image after a request has been acknowledged |
| `LORA_BW` | `500E3` | Signal bandwidth; must match the receiver and `--bw` |
| `SAVE_TO_SD` / `USE_OLED` | `true` | MicroSD copy of each frame / OLED status text. |


**Serial link is 115200 baud.** `receiver.ino` and `receive.py` are both set to 115200; keep them in sync if you change it, but don't change it.

---

## 2. Python receiver

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

On a real terminal it opens a full-screen **dashboard** (curses, no extra deps). Each
round a config form appears: highlight **Mode** (continuous / specific count) and
**Resolution** with the arrow keys, type in **Delay** (s) and **Count**, then navigate to
**Send**. A live line at the bottom previews the packed config word as you edit.

- **Send** --> the request goes out, the dashboard waits for the sender's ACK (resending
  until the sender reaches a listen window), then streams status into its log: per-image
  headers, a live progress bar, and the post-image Reed–Solomon/FEC summary. *Specific
  count* saves exactly `COUNT` images then reopens the form; *continuous* saves until you
  press **q**.
- **Cancel** (or `q`/Esc) --> passively save every image the sender streams. Press **q**
  to return to the form; **Ctrl-C** quits.

When stdout isn't a terminal (piped / redirected), it falls back to the original plain
`Send a new configuration? [y/N]` text prompts and line-by-line output, so scripted and
headless use is unchanged.

Images are written as `<YYYYMMDD>/<unix>.png` timestamped with Unix seconds.

### SDR: passive, listen-only

```bash
./receive.py --force sdr --gain 40           # drive an RTL-SDR (spawns rtl_sdr, only necessary if SDR is present with a LoRa board)
./receive.py --file rec.raw --rate 1.8e6     # replay a gqrx/GNU Radio fc32 recording
```

The SDR can't transmit, so it never sends configs, it
just decodes whatever the sender puts on the air. On a terminal it shows the same
dashboard as the serial path -- minus the config form -- streaming each inbound burst's
progress bar and decode/FEC result into the log (**q** or Ctrl-C to quit); piped or with
`--file` into a non-terminal it prints the original timestamped log lines instead.



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
| `RESOLUTION` | 1–4 | 0–13 | index into available resolutions |
| `DELAY` | 5–19 | 0–32767 | seconds between frames (timer starts at the last packet) |
| `COUNT` | 20–31 | 0–4095 | frames to send (0 or 1 --> exactly one); ignored in continuous mode |

**ACK value** = `config - (DELAY x COUNT) + RESOLUTION` (32-bit wrap)

**Image stream** (protocol v2, per frame): header `61 79 79 79 79` --> a params packet
(`70 32`, sent 3×: version, image length, `Ndata`, `K=32`, `M=4`, CRC16) --> `Ndata` data
packets `[seq:2 LE][246 JPEG bytes][crc16:2]` --> `M` Reed–Solomon parity packets per block
of `K` --> fingerprint `6c 6d 61 6f`. Any `M` lost/corrupt packets per block are recovered.
The GF(256) math is byte-identical between `sender.ino` and `rs_gf256.py`.


## Gotchas

- Both LoRa32 boards show up as `/dev/ttyACM*`. Flash the sender first (ACM0), the relay
  node second (ACM1), and give `receive.py` `--port /dev/ttyACM1`.
- If a request seems ignored, the sender is mid-image (half-duplex), `receive.py` keeps
  resending until the sender's next listen window; a long transfer may take a bit.
- All nodes must share the same `LORA_BW`; the SDR path also needs `--bw` to match.
- `upload.sh` can fail if a MicroSD card is seated in the target board. Remove it to flash.
