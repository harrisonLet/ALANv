  /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "shared_configuration.h"

#include "lora.h"
#include "servoRudder.h"
#include "servoSail.h"
#include <stdint.h>
#include <string.h>

#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct __attribute__((packed)) {
    float    lat;
    float    lon;
    int16_t  heading;
    int16_t  roll;
    int16_t  pitch;
    uint8_t  battery;
    int16_t  wind_speed;
    int16_t  wind_dir;
    int16_t  sail_angle;
} TelemetryPacket_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif

#define ADDR_SAMD21  0xAA
#define ADDR_STM32   0xBB

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
COM_InitTypeDef BspCOMInit;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim6;
volatile uint8_t  tx_flag  = 0;  /* set by TIM6 ISR every 1 s */
volatile uint8_t  dio0_flag = 0; /* set by EXTI ISR when DIO0 (PG9) fires */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void SystemClock_Config(void);
/* USER CODE END PFP */

/* Private user code --------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// initialize gpio pins for di0-3 interrupt and rst 
// initialize led's for debugging

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
  {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
#endif
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
#if defined(CM4_ENABLE) && !defined(CM7_ENABLE)
  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) { Error_Handler(); }
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
  
  
#if defined(CM4_ENABLE)
  MX_SPI1_Init();
  MX_TIM6_Init();

  Debug_LED_Init();

  #if defined(CM4_SERVOS_ENABLE)
  servoRudder_init();
  servoSail_init();
  #endif

  // Debug_LED_Toggle('y');  // toggle yellow LED to indicate SPI initialized

  if (LoRa_init() != 0)
  {
      printf("LoRa init FAILED\r\n");
      Debug_LED_Toggle('o');
  }
  LoRa_StartRX();

  static int16_t rudder_angle = 0;
  static int16_t sail_angle   = 0;
  /* LoRa_init() toggles yellow LED once for version check, so yellow starts ON */
  static uint8_t led_status   = 0x04;  /* bitmask: bit0=green, bit1=red, bit2=yellow */
  static uint8_t tx_seq       = 0;

  TelemetryPacket_t pkt = {
      .lat        = 40.7128f,
      .lon        = -74.0060f,
      .heading    = 180,
      .battery    = 95,
  };

  // char message[] = "Hello from harrisons laptop!\n";

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      __WFI();   /* sleep until any interrupt fires */

      if (tx_flag) {
          tx_flag = 0;
          char payload[40];
          uint8_t plen = snprintf(payload, sizeof(payload), "STATUS,%d,%d,%d", rudder_angle, sail_angle, led_status);
          uint8_t pkt[36];
          pkt[0] = ADDR_SAMD21;   /* dest */
          pkt[1] = ADDR_STM32;    /* src */
          pkt[2] = tx_seq++;
          pkt[3] = plen;
          memcpy(pkt + 4, payload, plen);
          LoRa_Send(pkt, 4 + plen);
          printf("[TX] %s\r\n", payload);
      }

      if (dio0_flag) {
          dio0_flag = 0;
          LoRa_ProcessDIO0();

          uint8_t cmd[64];
          uint8_t len = LoRa_GetCmd(cmd);
          if (len > 0) {
              printf("[RX] cmd len=%d byte0=0x%02X\r\n", len, cmd[0]);

              /* LED toggles */
              if      (cmd[0] == 'g') { led_status ^= 0x01; Debug_LED_Toggle('g'); }
              else if (cmd[0] == 'r') { led_status ^= 0x02; Debug_LED_Toggle('r'); }
              else if (cmd[0] == 'y') { led_status ^= 0x04; Debug_LED_Toggle('y'); }

              #if defined(CM4_SERVOS_ENABLE)
              /* Rudder steps */
              int16_t delta = 0;
              if      (cmd[0] == 'q') delta = +20;
              else if (cmd[0] == 'e') delta = -20;
              else if (cmd[0] == 'a') delta = +10;
              else if (cmd[0] == 'd') delta = -10;
              else if (cmd[0] == 'z') delta = +5;
              else if (cmd[0] == 'c') delta = -5;

              if (delta != 0) {
                rudder_angle += delta;
                if (rudder_angle >  135) rudder_angle =  135;
                if (rudder_angle < -135) rudder_angle = -135;
                servoRudder_setAngle(rudder_angle);
              }

              /* Sail steps */
              delta = 0;
              if      (cmd[0] == 'w') delta = +20;
              else if (cmd[0] == 'r') delta = -20;
              else if (cmd[0] == 's') delta = +10;
              else if (cmd[0] == 'f') delta = -10;
              else if (cmd[0] == 'x') delta = +5;
              else if (cmd[0] == 'v') delta = -5;

              if (delta != 0) {
                sail_angle += delta;
                if (sail_angle >  135) sail_angle =  135;
                if (sail_angle < -135) sail_angle = -135;
                servoSail_setAngle(sail_angle);
              }
              #endif
          }
      }
  }
#endif

  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */


static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;      /* Mode 0: CPOL=0, CPHA=0 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; /* APB2 64MHz → 2MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 63999;  /* 64 MHz / 64000 = 1 kHz  */
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 999;    /* 1 kHz  / 1000  = 1 Hz   */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) { Error_Handler(); }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&htim6);  /* start timer, enable update interrupt */
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                   | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }

  /* SPI1/2/3 kernel clock defaults to PLL1Q after reset, but PLL1 is off.
     Switch to HSI so SPI1 can actually generate clock pulses. */
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI123;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_CLKP; /* CLKP defaults to HSI after reset */
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) { Error_Handler(); }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
