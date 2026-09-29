#include "main.h"
#include "stm32h7xx_hal_i2c.h"
#include "sensorGPS.h"
#include "gps_parser.h"
#include <stdint.h> // Need this for a struct. If we are low on memory feel free to remove.
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

TaskHandle_t task_sensorGPS;
UART_HandleTypeDef  UART7_Handler = {0};

// static uint8_t g_gps_raw_byte;for interrupt
static char g_nmea_buffer[128];
static int g_index = 0; 
GPS_Data_t myGPS; // The actual data storage for NMEA06


/**
  * Initialize the hardware. CFG_COM1 and CFG_COM0 configuration pins
  */
void sensorGPS_hardwareInit() {
    // Will have to rewrite this part, we wrote code for the incorrect gps chip, we use m6, m6 uses UART
    // M9N is capable of I2c but not m6
    /* PF6 — UART7 RX */
    // here is the sheet: http://content.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* PF7 - UART7 TX */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

     /* Set UART7 kernel clock explicitly */
    // RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    // PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART7;
    // PeriphClkInit.Usart234578ClockSelection  = RCC_UART7CLKSOURCE_D2PCLK1;
    // if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    // {
    //     Error_Handler();
    // }

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection       = RCC_PERIPHCLK_USART234578;
    PeriphClkInit.Usart234578ClockSelection  = RCC_USART234578CLKSOURCE_D2PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }


    /* UART7 peripheral */
    UART7_Handler.Instance        = UART7;
    UART7_Handler.Init.BaudRate   = 9600;
    UART7_Handler.Init.WordLength = UART_WORDLENGTH_8B;
    UART7_Handler.Init.StopBits   = UART_STOPBITS_1;
    UART7_Handler.Init.Parity     = UART_PARITY_NONE;
    UART7_Handler.Init.Mode       = UART_MODE_TX_RX;

    if (HAL_UART_Init(&UART7_Handler) != HAL_OK)
    {
        Error_Handler();
    }

    uint32_t apb1_clk = HAL_RCC_GetPCLK1Freq();
    uint32_t sys_clk  = HAL_RCC_GetSysClockFreq();
    printf("UART7 init complete on PF6(RX) / PF7(TX)\r\n");
    printf("  SYSCLK:  %lu Hz\r\n", sys_clk);
    printf("  APB1CLK: %lu Hz\r\n", apb1_clk);
    printf("  Expected UART7 baud: 9600\r\n"); // page 11 
}


void GPS_Parse_GGA(char* nmea_str, GPS_Data_t* gps_struct)
{

  // gps fields set to null values
  char *gps_fields[15] = {0};

  // parse string by commas
  char *token = strtok(nmea_str, ",");
  
  // move tokens into their respective gps fields
  for (int i = 0; i < 15 && token != NULL; i++) {
    gps_fields[i] = token;
    token = strtok(NULL, ",");  // NULL tells strtok to continue from where it left off
  }

  // Print every field so we can see what strtok produced
  for (int i = 0; i < 10; i++) {
      printf("field[%d] = [%s]\r\n", i, gps_fields[i] ? gps_fields[i] : "NULL");
  }

  if (gps_fields[6] == NULL) { printf("BAILED: field 6 null\r\n"); return; }
  

  // DEBUG - Add this:
  //printf("PAST FIX CHECK: fix=%d\r\n", atoi(gps_fields[6]));

  //convert lat and long from dec to degrees
  // atof converts ASCII string to float
  float lat_raw = atof(gps_fields[2]);
  float long_raw = atof(gps_fields[4]);

  // get degree in int
  int lat_deg = (int) (lat_raw / 100);
  int long_deg = (int) (long_raw / 100);

  // pull out minutes and convert to decimal degrees
  gps_struct -> latitude = lat_deg + (lat_raw - lat_deg * 100) / 60.0f;
  gps_struct -> longitude = long_deg + (long_raw - long_deg * 100) / 60.0f;

  // Get cardinal directions (south and west are considered neg in decimal degrees)
  if (gps_fields[3][0] == 'S')
  {
    gps_struct->latitude  = -gps_struct->latitude;
  }

  if (gps_fields[5][0] == 'W') 
  {
    gps_struct->longitude = -gps_struct->longitude;
  }

  // set up the rest of the gps fields
  gps_struct -> altitude = atof(gps_fields[9]);
  gps_struct -> satellites = atof(gps_fields[7]);
  gps_struct -> fix_valid = atof(gps_fields[6]); // 1 if there is gps fix

  // convert to integer parts instead instead of printing float
  int lat_d  = (int)gps_struct->latitude;
  int lat_f  = (int)((gps_struct->latitude  - lat_d)  * 1000000);
  int lon_d  = (int)gps_struct->longitude;
  int lon_f  = (int)((gps_struct->longitude - lon_d)  * 1000000);
  int alt    = (int)gps_struct->altitude;

  printf("FIX:%d SATS:%d LAT:%d.%06d LON:%d.%06d ALT:%d\r\n",
    gps_struct->fix_valid,
    gps_struct->satellites,
    lat_d, lat_f,
    lon_d, lon_f,
    alt);
}

void GPS_ProcessChar(uint8_t rx_byte)
{
  // reset buffer for new data set
  if (rx_byte == '$') 
  {
    g_index = 0;
  }

  if(rx_byte == '\n')
  {
    if (g_index > 0 && g_nmea_buffer[g_index - 1] == '\r') // page 66, carriage return https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
    {
      g_index--;  // We need to overwrite the \r
    }

    // Have to end with null terminator
    g_nmea_buffer[g_index] = '\0';

    // parse if valid
    if (strstr(g_nmea_buffer, "GGA") != NULL) 
    {
      printf("RAW: [%s]\r\n", g_nmea_buffer);  // print exactly what we're about to parse
      GPS_Parse_GGA(g_nmea_buffer, &myGPS); // (Global Positioning System Fix Data) 
    }

    g_index = 0; // reset for the next data set

  }
  else 
  {
    if (g_index < 127)
    {
      g_nmea_buffer[g_index++] = rx_byte;
    }
  }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // Moving away from interrupts for the moment
// {
//   // 
//   if (huart->Instance == UART7) 
//   {
//     GPS_ProcessChar(g_gps_raw_byte);
//     HAL_UART_Receive_IT(huart, &g_gps_raw_byte, 1); 
//   }
// }

void GPS_Poll(void) // MOVE THIS TO THE MAIN LOOP EVENTUALLY?
{
    uint8_t byte;
    //Blocks until 1 byte arrives (or timeout)
    if (HAL_UART_Receive(&UART7_Handler, &byte, 1, 100) == HAL_OK)
    {
      //printf("%c", byte);
      GPS_ProcessChar(byte);
    }

    // if (HAL_UART_Receive(&UART7_Handler, &byte, 1, 5000) == HAL_OK)
    // {
    //     printf("GOT BYTE: 0x%02X  '%c'\r\n", byte, byte);
    // }
    // else
    // {
    //     printf("5 second timeout — nothing received\r\n");
    // }
}

/**
  * Handler for the task.
  */
void sensorGPS_handler(void *argument) {
  vTaskDelay(pdMS_TO_TICKS(5000));  // give the module 2 seconds to boot  
  for(;;) {
        //vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
        GPS_Poll();
    }
}