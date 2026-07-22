// Capture a JPEG from an Arducam MEGA and transmit it over LoRa once a minute,
// on a LilyGo/TTGO LoRa32 T3 v1.6.1 (ESP32 + SX127x LoRa + SSD1306 OLED).
//
// Two independent SPI buses are used so the camera and radio never fight:
//   * Camera  -> VSPI  : SCK=14, MISO=36 (SVP), MOSI=15, CS=13  (board SD-card bus)
//   * LoRa    -> HSPI  : SCK=5,  MISO=19, MOSI=27, SS=18, DIO0=26, RST=23
// GPIO36 (input-only, non-strapping) is used for camera MISO instead of the SD
// slot's GPIO2 (a boot strap pin).
//
// Two SPI init subtleties, both handled the same way -- pre-begin() the bus on the
// pins we want so the library's own bare begin() early-returns (ESP32 SPIClass::begin
// no-ops if the bus is already initialized):
//   1. The Arducam constructor runs SPI.begin() (default VSPI pins) at static-init,
//      before setup(). We SPI.end() + SPI.begin(14,36,15) to move VSPI to the camera.
//   2. LoRa.begin() calls _spi->begin() with no args; we pre-begin loraSPI on the
//      LoRa pins first so that call keeps them.
// LoRa RST is on GPIO23 (the T3 v1.6.1's real reset), NOT GPIO14 as in the stock
// LoRa_Sender example -- GPIO14 is the camera's SCK here, and LoRa.begin() drives
// the reset pin, which would collide with the camera bus.
//
// Wire protocol v2 (with Reed-Solomon forward error correction; see sendBufferV2):
// a header packet, a params packet (magic 0x70 0x32; sent 3x) carrying image length +
// FEC geometry, then Ndata data packets [seq:2 LE][246 bytes][crc16:2], then RS_M
// parity packets per RS_K-packet block (same framing), then a fingerprint packet. Each
// packet's seq + CRC16 lets the receiver place packets exactly and the RS parity
// reconstruct up to RS_M lost/corrupt packets per block. The GF(256)/CRC math is kept
// byte-identical to rs_gf256.py / receive.py. The practical size limit is
// airtime, not the count (~2 packets/sec over LoRa). If the image can't be buffered in
// RAM the code falls back to the legacy v1 stream (2-byte count + raw 250-byte chunks,
// no FEC), which the decoder still auto-detects.

#include <SPI.h>
#include <Wire.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Arducam_Mega.h"
#include "FS.h"
#include "SD.h"


// Save each captured JPEG to the on-board MicroSD card.
// Non-blocking if there's a card failure
const bool SAVE_TO_SD = true;

// Drive the on-board SSD1306 OLED with status text. 
// Set true to show capture/send progress.
const bool USE_OLED = true;

// Available CAPTURE_MODE values (Arducam MEGA CAM_IMAGE_MODE_*):
//   CAM_IMAGE_MODE_96X96       96x96
//   CAM_IMAGE_MODE_128X128     128x128
//   CAM_IMAGE_MODE_QQVGA       160x120
//   CAM_IMAGE_MODE_QVGA        320x240
//   CAM_IMAGE_MODE_320X320     320x320 // Fast transfer
//   CAM_IMAGE_MODE_VGA         640x480 // Good balance of speed and quality
//   CAM_IMAGE_MODE_SVGA        800x600
//////////////////// HERE BE DRAGONS ///////////////////////////////////
//// The camera lies about some of these sizes and returns VGA images
//// Or maybe the library is broken.
////////////////////////////////////////////////////////////////////////
//   CAM_IMAGE_MODE_1024X768    1024x768
//   CAM_IMAGE_MODE_HD          1280x720
//   CAM_IMAGE_MODE_1280X1024   1280x1024
//   CAM_IMAGE_MODE_UXGA        1600x1200
//   CAM_IMAGE_MODE_FHD         1920x1080
//   CAM_IMAGE_MODE_QXGA        2048x1536   (max for this 3MP sensor) --> lies
//   CAM_IMAGE_MODE_WQXGA2      2592x1944   (5MP sensor only)

// Resolution index table -- the receiver's 4-bit RESOLUTION field indexes this, so the
// order must stay byte-identical to receive.py's RES_NAMES. Index 8 (HD) is the default.
// TODO: Figure out a better way to keep this in sync with the Python code
static const CAM_IMAGE_MODE RES_TABLE[] = {
  CAM_IMAGE_MODE_96X96,     CAM_IMAGE_MODE_128X128,  CAM_IMAGE_MODE_QQVGA,
  CAM_IMAGE_MODE_QVGA,      CAM_IMAGE_MODE_320X320,  CAM_IMAGE_MODE_VGA,
  CAM_IMAGE_MODE_SVGA,      CAM_IMAGE_MODE_1024X768, CAM_IMAGE_MODE_HD,
  CAM_IMAGE_MODE_1280X1024, CAM_IMAGE_MODE_UXGA,     CAM_IMAGE_MODE_FHD,
  CAM_IMAGE_MODE_QXGA,      CAM_IMAGE_MODE_WQXGA2,
};
const uint8_t RES_COUNT   = sizeof(RES_TABLE) / sizeof(RES_TABLE[0]);
const uint8_t RES_DEFAULT = 8;   

// Enums LOW_QUALITY, DEFAULT_QUALITY, HIGH_QUALITY
const IMAGE_QUALITY CAPTURE_QUALITY = HIGH_QUALITY;

// --- On-demand config (two-way messaging with the receiver) ---
// BOOT_CONTINUOUS true  -> boot straight into a continuous timer at the defaults below
//   (legacy behavior), still listening for a config request in each DELAY gap.
// BOOT_CONTINUOUS false -> boot idle and send nothing until a request arrives.
const bool BOOT_CONTINUOUS = true;
// Firmware delay (N) between sending the ACK and the first capture of a new config.
const uint32_t REQUEST_TO_CAPTURE_DELAY_S = 5;
// Request/ACK framing magics (distinct from header/params/fingerprint below).
const uint8_t REQ_MAGIC[] = {0x72, 0x71};
const uint8_t ACK_MAGIC[] = {0x72, 0x63};

// Current running config, unpacked from the 32-bit request word (bit layout in receive.py).
// Defaults to 1280x720 @ 60 s continuous shooting
bool     cfgContinuous = true;
uint8_t  cfgResIdx     = RES_DEFAULT;
uint16_t cfgDelayS     = 60;
uint16_t cfgCount      = 0;
bool     activeCfg     = BOOT_CONTINUOUS;   // false -> idle until first request
uint32_t pendingCfg    = 0;                 // set by listenForConfig()

// --- Camera: VSPI on the free "SD card" bus ---
const int PIN_SCK  = 14;
const int PIN_MISO = 36;  // SVP; input-only, non-strapping (was GPIO2)
const int PIN_MOSI = 15;
const int CS       = 13;

// --- MicroSD slot: shares the camera's VSPI bus (SCK14/MOSI15/CS13); only MISO
// differs (SD = GPIO2, camera = GPIO36). Camera and SD are used at different
// times, so the shared bus is re-pointed between them. ---
const int SD_MISO = 2;
const int SD_CS   = 13;   // same select line as the camera CS
uint32_t sdIndex  = 0;    // next "/img_NNNNN.jpg" index

// --- LoRa SX127x: its own HSPI bus (see header note on RST=23) ---
const int LORA_SCK = 5, LORA_MISO = 19, LORA_MOSI = 27;
const int LORA_SS = 18, LORA_RST = 23, LORA_DIO0 = 26;
#define LORA_BAND 915E6
// LoRa signal bandwidth. The SX127x default is 125 kHz; raising it ~halves airtime
// per doubling (250E3 ~2x, 500E3 ~4x) at the cost of ~3 dB sensitivity each step --
// a good trade on a short-range link. The receiver must match: receiver.ino's
// setSignalBandwidth and receive.py's BW (or its --bw flag).
#define LORA_BW 500E3
SPIClass loraSPI(HSPI);

// --- OLED (I2C, optional/non-fatal) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool haveOLED = false;

// --- LoRa transfer protocol (must match the slow-loras receiver) ---
#define LORA_TRANSFER_BUFFER 250
// Gap after every packet. arduino-LoRa's endPacket() already blocks until TxDone, so
// this is pure dead air on top of airtime; trimmed 75 -> 10 ms as part of the PHY
// speedup (the receiver's burst detector only ends a burst after 0.5 s of silence).
const int delayMillis = 10;
const uint8_t header[]      = {0x61, 0x79, 0x79, 0x79, 0x79};
const uint8_t fingerprint[] = {0x6c, 0x6d, 0x61, 0x6f};
uint8_t lora_buffer[LORA_TRANSFER_BUFFER];

// --- Protocol v2: per-packet sequence + CRC16, plus Reed-Solomon erasure parity ---
// Each 250-byte data/parity packet is [seq:2 LE][payload:246][crc16:2]. The receiver
// CRC-checks each packet, places data by seq, and RS-reconstructs up to RS_M missing
// packets per RS_K-packet block -- so a lost/corrupt packet no longer desyncs the JPEG.
// A 15-byte params packet [0x70 0x32][ver][image_len:4][Ndata:2][K:2][M:2][crc16:2]
// (sent 3x) tells the receiver the geometry. Must stay byte-identical to rs_gf256.py /
// rs_gf256.py (GF(256) poly 0x11d, CRC-16/CCITT-FALSE, Cauchy matrix).
#define V2_DATA 246                 // image bytes per packet (250 - 2 seq - 2 crc)
#define RS_K    32                  // data packets per FEC block
#define RS_M    4                   // parity packets per block (recovers <=M losses/block)
const uint8_t PARAMS_MAGIC[] = {0x70, 0x32};

// GF(256) tables (primitive polynomial 0x11d, generator alpha=2).
static uint8_t GF_EXP[512];
static uint8_t GF_LOG[256];
void gf_init() {
  int x = 1;
  for (int i = 0; i < 255; i++) { GF_EXP[i] = (uint8_t)x; GF_LOG[x] = (uint8_t)i;
                                  x <<= 1; if (x & 0x100) x ^= 0x11d; }
  for (int i = 255; i < 512; i++) GF_EXP[i] = GF_EXP[i - 255];
}
static inline uint8_t gf_mul(uint8_t a, uint8_t b) {
  if (!a || !b) return 0;
  return GF_EXP[(int)GF_LOG[a] + (int)GF_LOG[b]];
}
static inline uint8_t gf_inv(uint8_t a) { return GF_EXP[255 - GF_LOG[a]]; }

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection).
uint16_t crc16(const uint8_t* p, uint32_t n) {
  uint16_t c = 0xFFFF;
  for (uint32_t i = 0; i < n; i++) {
    c ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++) c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

// Scratch for one packet body and one block's parity (static: keep off the stack).
static uint8_t v2row[V2_DATA];
static uint8_t v2parity[RS_M][V2_DATA];

// Frame + transmit one v2 packet: [seq:2 LE][body:246][crc16:2].
void writeV2Packet(uint16_t seq, const uint8_t* body) {
  lora_buffer[0] = (uint8_t)(seq & 0xFF);
  lora_buffer[1] = (uint8_t)(seq >> 8);
  memcpy(lora_buffer + 2, body, V2_DATA);
  uint16_t c = crc16(lora_buffer, 2 + V2_DATA);
  lora_buffer[2 + V2_DATA]     = (uint8_t)(c & 0xFF);
  lora_buffer[2 + V2_DATA + 1] = (uint8_t)(c >> 8);
  LoRa.beginPacket();
  LoRa.write(lora_buffer, LORA_TRANSFER_BUFFER);
  LoRa.endPacket();
  delay(delayMillis);
}

// The 15-byte params packet, sent 3x for robustness.
void sendParamsV2(uint32_t image_len, uint16_t Ndata) {
  uint8_t p[15];
  p[0] = PARAMS_MAGIC[0]; p[1] = PARAMS_MAGIC[1]; p[2] = 2;      // version 2
  p[3] = (uint8_t)(image_len & 0xFF); p[4] = (uint8_t)(image_len >> 8);
  p[5] = (uint8_t)(image_len >> 16);  p[6] = (uint8_t)(image_len >> 24);
  p[7] = (uint8_t)(Ndata & 0xFF);  p[8]  = (uint8_t)(Ndata >> 8);
  p[9] = (uint8_t)(RS_K & 0xFF);   p[10] = (uint8_t)(RS_K >> 8);
  p[11] = (uint8_t)(RS_M & 0xFF);  p[12] = (uint8_t)(RS_M >> 8);
  uint16_t c = crc16(p, 13);
  p[13] = (uint8_t)(c & 0xFF); p[14] = (uint8_t)(c >> 8);
  for (int r = 0; r < 3; r++) {
    LoRa.beginPacket(); LoRa.write(p, 15); LoRa.endPacket(); delay(delayMillis);
  }
}


const unsigned long SERIAL_BAUD = 115200;

Arducam_Mega myCAM(CS);

void oledMsg(const String& s) {
  if (!haveOLED) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.print(s);
  display.display();
}

// Re-point the shared VSPI bus. The camera reads MISO on GPIO36; the SD card
// slot's MISO is on GPIO2. Only one can be active at a time.
void spiForCamera() { SPI.end(); SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI); }
void spiForSD()     { SPI.end(); SPI.begin(PIN_SCK, SD_MISO,  PIN_MOSI); }

// Write one JPEG to the SD card. Self-contained and non-fatal: it re-mounts the
// card each call (so a re-inserted card recovers) and any failure just logs and
// returns, always handing the bus back to the camera. Never blocks capture/TX.
void saveToSD(const uint8_t* img, uint32_t len) {
  spiForSD();
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("SD: card not detected; save skipped.");
    spiForCamera();
    return;
  }
  char path[24];
  do { snprintf(path, sizeof(path), "/img_%05u.jpg", sdIndex++); } while (SD.exists(path));
  File f = SD.open(path, FILE_WRITE);
  if (f) {
    size_t w = f.write(img, len);
    f.close();
    Serial.printf("SD: saved %s (%u bytes)\n", path, (unsigned)w);
    oledMsg("Saved\n" + String(path));
  } else {
    Serial.println("SD: open for write failed; save skipped.");
  }
  SD.end();
  spiForCamera();  // hand the bus back to the camera
}

// Transmit a buffered JPEG using protocol v2: header, params (x3), data packets
// carrying [seq:2][246 bytes][crc16:2], then RS_M Reed-Solomon parity packets per
// RS_K-packet block, then the fingerprint. The receiver CRC-checks each packet, places
// data by seq, and RS-reconstructs up to RS_M missing/corrupt packets per block -- so a
// dropped or corrupted packet no longer desyncs the whole JPEG. Parity is accumulated
// one block at a time (only RS_M*246 bytes of scratch), independent of image size.
void sendBufferV2(const uint8_t* img, uint32_t len) {
  uint16_t Ndata   = (uint16_t)((len + V2_DATA - 1) / V2_DATA);   // ceil
  uint16_t nblocks = (uint16_t)((Ndata + RS_K - 1) / RS_K);

  // burst-start header (keeps the receiver's power/burst framing happy), then params
  LoRa.beginPacket(); LoRa.write(header, sizeof(header)); LoRa.endPacket();
  delay(delayMillis);
  sendParamsV2(len, Ndata);

  for (uint16_t b = 0; b < nblocks; b++) {
    uint16_t lo = (uint16_t)(b * RS_K);
    uint16_t k  = (uint16_t)((Ndata - lo < RS_K) ? (Ndata - lo) : RS_K);
    memset(v2parity, 0, sizeof(v2parity));
    for (uint16_t j = 0; j < k; j++) {
      uint32_t off = (uint32_t)(lo + j) * V2_DATA;
      uint32_t n = (off < len) ? (len - off) : 0;
      if (n > V2_DATA) n = V2_DATA;
      memset(v2row, 0, V2_DATA);
      if (n) memcpy(v2row, img + off, n);
      writeV2Packet((uint16_t)(lo + j), v2row);              // data packet, seq = index
      // accumulate parity: parity[i] ^= A[i][j] * row, with A[i][j] = 1 / ((k+i) ^ j)
      for (uint16_t i = 0; i < RS_M; i++) {
        uint8_t coef = gf_inv((uint8_t)((k + i) ^ j));
        uint8_t* pr = v2parity[i];
        for (uint16_t c = 0; c < V2_DATA; c++) pr[c] ^= gf_mul(coef, v2row[c]);
      }
    }
    for (uint16_t i = 0; i < RS_M; i++)
      writeV2Packet((uint16_t)(Ndata + b * RS_M + i), v2parity[i]);   // parity packet
    if ((b & 0x03) == 0)
      oledMsg("Sending\n" + String(lo + k) + "/" + String(Ndata));
  }
  delay(delayMillis);
  LoRa.beginPacket(); LoRa.write(fingerprint, sizeof(fingerprint)); LoRa.endPacket();
}

// Same v2 protocol as sendBufferV2, but streams the JPEG straight from the camera
// FIFO one FEC block at a time (only RS_M*246 bytes of parity scratch), so it works
// for images too large to malloc whole -- which is the common case on this no-PSRAM
// board (the ~110-160 KB contiguous-heap limit is exactly why the old buffered path
// silently fell back to the un-protected v1 stream). No SD copy on this path.
void sendBufferV2Stream(uint32_t len) {
  uint16_t Ndata   = (uint16_t)((len + V2_DATA - 1) / V2_DATA);
  uint16_t nblocks = (uint16_t)((Ndata + RS_K - 1) / RS_K);

  LoRa.beginPacket(); LoRa.write(header, sizeof(header)); LoRa.endPacket();
  delay(delayMillis);
  sendParamsV2(len, Ndata);

  uint32_t readTotal = 0;
  for (uint16_t b = 0; b < nblocks; b++) {
    uint16_t lo = (uint16_t)(b * RS_K);
    uint16_t k  = (uint16_t)((Ndata - lo < RS_K) ? (Ndata - lo) : RS_K);
    memset(v2parity, 0, sizeof(v2parity));
    for (uint16_t j = 0; j < k; j++) {
      memset(v2row, 0, V2_DATA);
      uint32_t want = (readTotal < len) ? (len - readTotal) : 0;
      if (want > V2_DATA) want = V2_DATA;
      uint32_t got = 0;
      while (got < want) {                              // FIFO may return short reads
        uint8_t n = myCAM.readBuff(v2row + got, want - got);
        if (n == 0) break;
        got += n;
      }
      readTotal += got;
      writeV2Packet((uint16_t)(lo + j), v2row);         // data packet
      for (uint16_t i = 0; i < RS_M; i++) {
        uint8_t coef = gf_inv((uint8_t)((k + i) ^ j));
        uint8_t* pr = v2parity[i];
        for (uint16_t c = 0; c < V2_DATA; c++) pr[c] ^= gf_mul(coef, v2row[c]);
      }
    }
    for (uint16_t i = 0; i < RS_M; i++)
      writeV2Packet((uint16_t)(Ndata + b * RS_M + i), v2parity[i]);   // parity packet
    if ((b & 0x03) == 0)
      oledMsg("Sending\n" + String(lo + k) + "/" + String(Ndata));
  }
  delay(delayMillis);
  LoRa.beginPacket(); LoRa.write(fingerprint, sizeof(fingerprint)); LoRa.endPacket();
}

void captureAndSend() {
  oledMsg("Capturing");
  myCAM.takePicture(RES_TABLE[cfgResIdx], CAM_IMAGE_PIX_FMT_JPG);

  uint32_t len = myCAM.getTotalLength();
  uint32_t ndata = (len + V2_DATA - 1) / V2_DATA;   // v2 data packets (ceil), 246 B each
  Serial.printf("Captured %u bytes -> %u data packets\n", len, ndata);

  if (ndata > 65535) {  // Ndata is sent as a uint16 in the params packet; can't represent more
    Serial.println("Image too large for 16-bit Ndata; skipping frame.");
    oledMsg("Too big\nskipped");
    return;
  }

  // Transmit with v2 forward error correction. If the whole JPEG fits in RAM, buffer
  // it so it can ALSO be saved to SD (transmit first so an SD hiccup never delays the
  // primary path). Large images that can't be malloc'd stream straight from the camera
  // FIFO with per-block parity instead -- so FEC always runs, regardless of size.
  if (SAVE_TO_SD) {
    uint8_t* img = (uint8_t*)malloc(len);
    if (img) {
      uint32_t got = 0;
      while (got < len) {
        uint32_t want = len - got;
        if (want > LORA_TRANSFER_BUFFER) want = LORA_TRANSFER_BUFFER;
        uint8_t n = myCAM.readBuff(img + got, want);
        if (n == 0) break;
        got += n;
      }
      sendBufferV2(img, got);              // LoRa first (primary functionality)
      saveToSD(img, got);                  // then SD (optional, non-fatal)
      free(img);
      oledMsg("Send\nComplete");
      Serial.println("Send complete (buffered + SD).");
      return;
    }
    Serial.println("Image too large to buffer; streaming with FEC (no SD copy).");
  }

  sendBufferV2Stream(len);                 // streaming FEC path (any image size)
  oledMsg("Send\nComplete");
  Serial.println("Send complete (streamed).");
}

// Unpack a 32-bit request word into the current config. MODE bit0: 0=continuous,
// 1=specific count. RESOLUTION bits1-4, DELAY bits5-19 (s), COUNT bits20-31.
void applyCfg(uint32_t cfg) {
  cfgContinuous = (cfg & 0x1) == 0;
  cfgResIdx     = (uint8_t)((cfg >> 1) & 0xF);
  if (cfgResIdx >= RES_COUNT) cfgResIdx = RES_DEFAULT;
  cfgDelayS     = (uint16_t)((cfg >> 5) & 0x7FFF);
  cfgCount      = (uint16_t)((cfg >> 20) & 0xFFF);
}

// Listen for an inbound config request for up to `secs` seconds. A request is an 8-byte
// packet [REQ_MAGIC:2][cfg:4 LE][crc16:2]; on a valid one, store it in pendingCfg and
// return true. Polling parsePacket() (RX_SINGLE) is fine here -- requests are lone,
// sporadic packets, not the back-to-back burst that forces the image RX node to RX_CONT.
bool listenForConfig(uint32_t secs) {
  uint32_t deadline = millis() + secs * 1000UL;
  while ((int32_t)(millis() - deadline) < 0) {
    int ps = LoRa.parsePacket();
    if (ps <= 0) continue;
    if (ps == 8) {
      uint8_t b[8];
      for (int i = 0; i < 8; i++) b[i] = (uint8_t)LoRa.read();
      if (b[0] == REQ_MAGIC[0] && b[1] == REQ_MAGIC[1] &&
          crc16(b, 6) == (uint16_t)(b[6] | (b[7] << 8))) {
        pendingCfg = (uint32_t)b[2] | ((uint32_t)b[3] << 8) |
                     ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 24);
        return true;
      }
    }
    while (LoRa.available()) LoRa.read();   // drain anything else off the FIFO
  }
  return false;
}

// Reply to a request: ack = cfg - DELAY*COUNT + RESOLUTION (32-bit wrap), framed as
// [ACK_MAGIC:2][ack:4 LE][crc16:2].
void sendAck(uint32_t cfg) {
  uint8_t  res     = (uint8_t)((cfg >> 1) & 0xF);
  uint16_t delay_s = (uint16_t)((cfg >> 5) & 0x7FFF);
  uint16_t count   = (uint16_t)((cfg >> 20) & 0xFFF);
  uint32_t ack     = cfg - (uint32_t)delay_s * count + res;
  uint8_t p[8];
  p[0] = ACK_MAGIC[0]; p[1] = ACK_MAGIC[1];
  p[2] = (uint8_t)(ack);       p[3] = (uint8_t)(ack >> 8);
  p[4] = (uint8_t)(ack >> 16); p[5] = (uint8_t)(ack >> 24);
  uint16_t c = crc16(p, 6);
  p[6] = (uint8_t)(c & 0xFF); p[7] = (uint8_t)(c >> 8);
  LoRa.beginPacket(); LoRa.write(p, 8); LoRa.endPacket();
  Serial.printf("Request 0x%08X -> ACK 0x%08X\n", cfg, ack);
}

// A request arrived (pendingCfg set): ACK it, wait N seconds, then adopt it and mark the
// config active so loop() starts capturing.
void adoptPending() {
  oledMsg("Config\nrx");
  sendAck(pendingCfg);
  delay(REQUEST_TO_CAPTURE_DELAY_S * 1000UL);
  applyCfg(pendingCfg);
  activeCfg = true;
}

void setup() {
  Serial.begin(SERIAL_BAUD);

  gf_init();   // GF(256) tables for Reed-Solomon parity (protocol v2)

  // Camera on VSPI. Release the bus the Arducam constructor grabbed at static-init,
  // then re-init on the camera pins before myCAM.begin().
  SPI.end();
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  myCAM.begin();
  myCAM.setImageQuality(CAPTURE_QUALITY);  

  // Optional SD card: probe once for early feedback (saving re-mounts each time
  // anyway). Non-fatal -- the bus is always handed back to the camera.
  if (SAVE_TO_SD) {
    spiForSD();
    if (SD.begin(SD_CS, SPI)) { Serial.println("SD: card OK."); SD.end(); }
    else Serial.println("SD: no card at boot (will retry each capture).");
    spiForCamera();
  }

  // LoRa on its own HSPI bus. Pre-begin on the LoRa pins so LoRa.begin()'s internal
  // bare _spi->begin() keeps them.
  loraSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setSPI(loraSPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("LoRa init failed! (check antenna/band)");
  } else {
    // Pin the full PHY explicitly instead of relying on SX127x reset defaults, so the
    // radio config is unambiguous and matches the receiver. SF7/CR4-5/sync 0x12 equal
    // the old defaults; LORA_BW is the throughput knob (see its #define above).
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x12);
    Serial.printf("LoRa init ok (SF7, BW=%.0f kHz, CR4/5, delay=%dms).\n",
                  (double)LORA_BW / 1000.0, delayMillis);
  }

  // OLED status display -- optional (haveOLED stays false when off, so oledMsg()
  // no-ops). When disabled we don't power the panel up; instead we explicitly force
  // it off over I2C so a warm reset that left it lit can't keep draining the battery.
  if (USE_OLED) {
    Wire.begin(21, 22);
    haveOLED = display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    if (haveOLED) { display.clearDisplay(); display.display(); }
  } else {
    Wire.begin(21, 22);
    Wire.beginTransmission(SCREEN_ADDRESS);
    Wire.write((uint8_t)0x00);                // control byte: command stream
    Wire.write((uint8_t)SSD1306_CHARGEPUMP);  // 0x8D
    Wire.write((uint8_t)0x10);                //   -> charge pump OFF
    Wire.write((uint8_t)SSD1306_DISPLAYOFF);  // 0xAE -> panel off
    Wire.endTransmission();
  }

  Serial.println("READY");
}

void loop() {
  // Idle (no active config): send nothing, just listen for a request. This is the boot
  // state when BOOT_CONTINUOUS is false, and the resting state after a specific-count run.
  if (!activeCfg) {
    oledMsg("Idle\nlistening");
    if (listenForConfig(60)) adoptPending();
    return;
  }

  // Continuous: capture forever, DELAY between frames. Specific count: COUNT frames
  // (COUNT 0/1 -> exactly 1). In both cases the DELAY gap doubles as the config-listen
  // window, so a new request preempts the current run (ACK -> wait N -> restart).
  uint32_t target = cfgContinuous ? 0xFFFFFFFFUL : (cfgCount <= 1 ? 1UL : cfgCount);
  for (uint32_t i = 0; i < target; i++) {
    captureAndSend();                         // timer starts after the last packet is sent
    if (listenForConfig(cfgDelayS)) { adoptPending(); return; }
  }
  activeCfg = false;   // specific-count run done -> idle-listen for the next request
}
