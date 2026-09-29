#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

typedef struct{//$GPGLL,4717.11634,N,00833.91297,E,124923.00,A,A*6E is an example on page 62 https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
  double latitude;
  double longitude;
  float altitude;
  uint8_t satellites;
  uint8_t fix_valid;
  uint8_t hours;
  uint8_t minutes;
  uint8_t seconds;
} GPS_Data_t;

extern GPS_Data_t myGPS;

void GPS_ProcessChar(uint8_t rx_byte);
void GPS_Parse_GGA(char* nmea_str, GPS_Data_t* gps_struct); // (Global Positioning System Fix Data)
// void GPS_Parse_GLL(char* nmea_str, GPS_Data_t* gps_struct); not sure if we need GLL
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); We will be doing interrupt stuff later

#endif