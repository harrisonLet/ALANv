#include <Arduino.h>
#include <LoRa.h>

#define RFM95_CS   12
#define RFM95_RST   7
#define RFM95_INT   6
#define LORA_FREQ   915E6

// How long to wait accumulating bytes before sending (ms)
#define FLUSH_TIMEOUT_MS 10
// Max bytes per packet (LoRa max is 255)
#define MAX_PKT_SIZE 128

static uint8_t txBuf[MAX_PKT_SIZE];
static uint8_t txLen = 0;
static uint32_t lastByteTime = 0;

void setup() {
  SerialUSB.begin(115200);
  while (!SerialUSB);  // wait for USB serial to come up

  LoRa.setPins(RFM95_CS, RFM95_RST, RFM95_INT);

  if (!LoRa.begin(LORA_FREQ)) {
    SerialUSB.println("LoRa init failed. Check wiring.");
    while (1);
  }

  // Must match the boat's radio (CM4 LoRa_init): SF8, 250 kHz, CR 4/5, CRC on
  LoRa.setSpreadingFactor(8);      // SF8 (SF7 = fastest, lowest range)
  LoRa.setSignalBandwidth(250E3);  // 250 kHz bandwidth
  LoRa.setCodingRate4(5);          // 4/5 coding rate
  LoRa.enableCrc();                // boat checks the payload CRC, so a corrupted command is dropped, not run
  LoRa.setTxPower(17);             // dBm, 2–20 valid
}

static void flushTx() {
  if (txLen == 0) return;
  LoRa.beginPacket();
  LoRa.write(txBuf, txLen);
  LoRa.endPacket();
  txLen = 0;
}

void loop() {
  // USB → LoRa: buffer bytes, flush on timeout or full buffer
  while (SerialUSB.available()) {
    txBuf[txLen++] = (uint8_t)SerialUSB.read();
    lastByteTime = millis();
    if (txLen >= MAX_PKT_SIZE) {
      flushTx();
    }
  }

  // Flush after idle gap — makes it feel like a stream
  if (txLen > 0 && (millis() - lastByteTime) >= FLUSH_TIMEOUT_MS) {
    flushTx();
  }

  // LoRa → USB
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    while (LoRa.available()) {
      SerialUSB.write((uint8_t)LoRa.read());
    }
  }
}
