#ifndef LORA_H
#define LORA_H

#include <stdint.h>

#define ADDR_SAMD21  0xAA            // ground station radio
#define ADDR_STM32   0xBB            // this boat
#define LORA_HDR_LEN_BYTES (5U)      // dest, src, seq, len, flags/type


int     LoRa_init(void);
void    LoRa_Send(uint8_t *data, uint8_t len);
void    LoRa_StartRX(void);
void    LoRa_ProcessDIO0(void);
uint8_t LoRa_GetCmd(uint8_t *buf);


#endif


