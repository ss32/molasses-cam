/* For use with LilyGo LORA-32 boards with OLED display
https://www.lilygo.cc/products/lora3
*/

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>

#define SCK 5   // GPIO5  -- SX1278's SCK
#define MISO 19 // GPIO19 -- SX1278's MISnO
#define MOSI 27 // GPIO27 -- SX1278's MOSI
#define SS 18   // GPIO18 -- SX1278's CS
#define RST 23  // GPIO23 -- SX1278's RESET on the LoRa32 T3 v1.6.1 (NOT GPIO14, which
                // the stock LilyGo examples use; on this board 14 isn't the reset line,
                // so LoRa.begin() can't reset the radio and it silently receives nothing).
                // If your board is an older LoRa32 (v1.0/v1.6), change this back to 14.
#define DI0 26  // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND 915E6
// Must match the transmitter's LORA_BW in sender/sender.ino (125E3 / 250E3 / 500E3).
#define LORA_BW 500E3

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS \
  0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const uint8_t header[] = {0x61, 0x79, 0x79, 0x79, 0x79};
const uint8_t fingerprint[] = {0x6c, 0x6d, 0x61, 0x6f};

uint16_t packetsRecieved = 0;
uint16_t payloadSize = 0;
bool nextPacketIsPayloadSize = false;
bool gotHeader = false;

// A full SSD1306 redraw is a ~1KB blocking I2C transfer. Flush it at most a few times
// a second rather than on every packet; the pixels we wrote just wait in the
// framebuffer until the next flush, keeping loop() responsive.
unsigned long lastFlush = 0;
void flushDisplay(bool force)
{
  if (force || millis() - lastFlush > 250)
  {
    display.display();
    lastFlush = millis();
  }
}

// ---- RX capture (interrupt-driven, RX_CONTINUOUS) --------------------------------
// The old code polled LoRa.parsePacket(), which runs the SX1278 in RX_SINGLE mode:
// after every packet the library drops the radio to standby and must re-arm it. The
// sender's next preamble arrives only 75ms after the previous packet ends -- sooner
// than we can finish reading and re-arm -- so RX_SINGLE silently missed every other
// packet (measured: seq numbers stepping by 2). Losing ~50% of every block makes the
// Reed-Solomon parity (M=4 of 32) useless and corrupts every image.
//
// With onReceive()/receive() the radio stays in RX_CONTINUOUS and never goes to
// standby, so it hears every packet. The DIO0 ISR does only the fast, non-blocking
// work -- pull the bytes off the FIFO over SPI -- and hands the packet to loop(),
// which does the slow Serial hex dump and OLED redraw where blocking is safe. The ISR
// only touches rxBuf while rxReady is false and loop() only while it is true, so the
// buffer needs no further locking.
volatile bool     rxReady   = false;
volatile int      rxLen     = 0;
volatile uint16_t rxDropped = 0;   // packets seen while the previous one was unhandled
uint8_t           rxBuf[255];

void IRAM_ATTR onLoraReceive(int packetSize)
{
  if (rxReady) { rxDropped++; return; }             // loop() hasn't consumed the last one
  if (packetSize <= 0 || packetSize > (int)sizeof(rxBuf)) return;
  for (int i = 0; i < packetSize; i++)
    rxBuf[i] = (uint8_t)LoRa.read();
  rxLen = packetSize;
  rxReady = true;
}

bool checkCode(uint8_t *buffer, const uint8_t *code)
{
  for (int i = 0; i < sizeof(code); i++)
  {
    if (buffer[i] != code[i])
    {
      return false;
    }
  }
  return true;
}

// CRC-16/CCITT-FALSE, to validate the v2 params packet before trusting its count.
uint16_t crc16(const uint8_t *p, uint16_t n)
{
  uint16_t c = 0xFFFF;
  for (uint16_t i = 0; i < n; i++)
  {
    c ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

// Interface for the Python serial receiver. Runs in loop() (not the ISR) on a packet
// already pulled off the FIFO, so the blocking Serial/OLED work here can't cost us the
// next over-the-air packet.
void processPacket(uint8_t *packet_buffer, int packetSize)
{
  for (int i = 0; i < packetSize; i++)
    Serial.printf("%02X", packet_buffer[i]);
  Serial.println();
  // v2 params packet [0x70 0x32][ver][len:4][Ndata:2][K:2][M:2][crc16:2] carries the
  // real total packet count (data + Reed-Solomon parity). Prefer it; fall back to the
  // legacy 2-byte count that a v1 sender puts right after the header.
  if (packetSize >= 15 && packet_buffer[0] == 0x70 && packet_buffer[1] == 0x32 &&
      crc16(packet_buffer, 13) == (packet_buffer[13] | ((uint16_t)packet_buffer[14] << 8)))
  {
    uint16_t Ndata = packet_buffer[7] | ((uint16_t)packet_buffer[8] << 8);
    uint16_t K = packet_buffer[9] | ((uint16_t)packet_buffer[10] << 8);
    uint16_t M = packet_buffer[11] | ((uint16_t)packet_buffer[12] << 8);
    uint16_t nblocks = K ? (Ndata + K - 1) / K : 0;
    payloadSize = Ndata + nblocks * M;   // total packets to expect over the air
    nextPacketIsPayloadSize = false;
  }
  else if (nextPacketIsPayloadSize)
  {
    // legacy v1: little-endian 2-byte count (low byte first); tolerate a 1-byte packet
    payloadSize = packet_buffer[0];
    if (packetSize > 1) payloadSize |= (uint16_t)packet_buffer[1] << 8;
    nextPacketIsPayloadSize = false;
  }
  if(gotHeader){
    upperMessage("Rx Node");
    lowerMessage("Packet " + String(packetsRecieved) + "/" + String(payloadSize) + "        ");
    packetsRecieved++;
    flushDisplay(false);   // throttled redraw; RX runs in the ISR so this can't drop packets
  }
  if (checkCode(packet_buffer, header))
  {
    upperMessage("Rx Node");
    nextPacketIsPayloadSize = true;
    gotHeader = true;
  }
  if (checkCode(packet_buffer, fingerprint) && gotHeader)
  {
    display.clearDisplay();
    upperMessage("Rx Node");
    lowerMessage("Download complete");
    flushDisplay(true);    // end of image: force the final redraw
    packetsRecieved = 0;
    gotHeader = false;
  }
}

void setup()
{
  Serial.begin(38400);
  Serial.println("LoRa Receiver Callback");
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  // Match the transmitter's PHY exactly. These are the arduino-LoRa / SX127x defaults,
  // but setting them explicitly means a stale radio state can never cause a silent
  // mismatch (freq is set by LoRa.begin(BAND) above; the TX uses these same values).
  LoRa.setSpreadingFactor(7);      // SF7
  LoRa.setSignalBandwidth(LORA_BW);  // BW (matches sender's LORA_BW)
  LoRa.setCodingRate4(5);          // CR 4/5
  LoRa.setSyncWord(0x12);          // private-network sync word

  LoRa.onReceive(onLoraReceive);   // DIO0 RxDone -> ISR
  LoRa.receive();                  // RX_CONTINUOUS: never standby between packets
  Serial.println("init ok");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.println("Rx Node");
  display.setCursor(0, 20);
  display.setTextSize(1);
  display.println("Waiting for data...");
  display.display();
  delay(1000);
}

// These only write into the framebuffer now -- no display.display()/delay here.
// The actual (throttled) flush happens via flushDisplay() from processPacket(), so the
// RX path is not blocked by a full OLED redraw on every single packet.
void upperMessage(String s)
{
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.print(s);
}

void lowerMessage(String s)
{
  display.setCursor(0, 20);
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);
  display.print(s);
}
void loop()
{
  if (rxReady)
  {
    // Copy out under the rxReady handshake (the ISR won't touch rxBuf while it's set),
    // release the buffer for the next packet, then do the slow work.
    static uint8_t local[255];
    int len = rxLen;
    memcpy(local, rxBuf, len);
    rxReady = false;
    processPacket(local, len);
  }
  delay(1);
}
