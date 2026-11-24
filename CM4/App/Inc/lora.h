#ifndef LORA_H
#define LORA_H

#include <stdint.h>

#define ADDR_SAMD21  0xAA            // ground station radio
#define ADDR_STM32   0xBB            // this boat
#define LORA_HDR_LEN_BYTES (5U)      // dest, src, seq, len, flags/type

// What the radio itself saw, for the telemetry. Parsing the packet is up to the caller.
typedef struct {
    int16_t  last_rssi;      // dBm, last good packet
    int8_t   last_snr;       // quarter dB, last good packet
    uint32_t last_rx_tick;   // HAL tick of the last good packet, 0 = none yet
    int16_t  noise;          // dBm, channel RSSI while listening, sampled by LoRa_GetRxStats
    uint16_t ok;             // packets with a good CRC, wraps
    uint16_t crc_err;        // packets with a bad or missing CRC, dropped, wraps
    uint32_t last_reinit_tick; // HAL tick LoRa_Send last found the radio reset and set it up again, 0 = never
} LoRaRxStats_t;

int     LoRa_init(void);
void    LoRa_Send(uint8_t *data, uint8_t len);
void    LoRa_StartRX(void);
void    LoRa_ProcessDIO0(void);
uint8_t LoRa_GetCmd(uint8_t *buf);
void    LoRa_GetRxStats(LoRaRxStats_t *stats);


#endif


