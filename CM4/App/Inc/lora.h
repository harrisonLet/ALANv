#ifndef LORA_H
#define LORA_H

#include <stdint.h>



int     LoRa_init(void);
void    LoRa_Send(uint8_t *data, uint8_t len);
void    LoRa_StartRX(void);
void    LoRa_ProcessDIO0(void);
uint8_t LoRa_GetCmd(uint8_t *buf);


#endif


