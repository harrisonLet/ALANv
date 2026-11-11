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
#include "ipcCM4.h"

#include "lora.h"
#include "loraTelemetry.h"
#include "telemetryReceive.h"
#include <stdint.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
#define LORA_HDR_LEN_BYTES (5U) // dest, src, seq, len, flags

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
static void MX_StatusLED_Init(void);
static void SystemClock_Config(void);
/* USER CODE END PFP */

/* Private user code --------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
#if defined(CM4_ENABLE) && !defined(CM7_ENABLE) && defined(CM4_DEBUG)
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
  MX_StatusLED_Init();

  bool ipc_attached = false;
  #if defined(IPC_ENABLE)
  // Initialize IPC and wait 5s for Cm7 to setup shared memory.
  // A failed handshake is reported to the ground station through FLAG_NO_IPC
  ipc_attached = (ipc_cm4_attach(5000U) == IPC_STATUS_OK);
  #endif
  telemetryRec_init(ipc_attached);

  if (LoRa_init() != 0)
  {
      printf("LoRa init FAILED\r\n");
  }
  #if defined(CM4_LORA_COMMANDS_ENABLE)
  LoRa_StartRX();
  #endif

  static uint8_t tx_seq = 0;

  #if defined(CM4_LORA_DUMMY_TELEMETRY)
  // Fixed, easy-to-recognise values so the ground station decode can be checked by eye
  LoRaTelemetry_t telemetryPacket = {
      .battery_voltage        = 14800,
      .cell_voltage           = { 3700, 3700, 3700, 3700 },
      .battery_valid          = LORA_TELEMETRY_ALL_VALID,
      .sail_ang_forward       = 15,
      .sail_encoder_magnitude = 2048,
      .sail_encoder_status    = 0x20,
      .encoder_valid          = LORA_TELEMETRY_ALL_VALID,
      .ang_wind               = -45,
      .wind_valid             = LORA_TELEMETRY_ALL_VALID,
      .latitude               = 40.7608,
      .longitude              = -111.8910,
      .altitude               = 1300.0f,
      .satellites             = 8,
      .fix_valid              = 1,
      .utc_hours              = 12,
      .utc_minutes            = 34,
      .utc_seconds            = 56,
      .gps_valid              = LORA_TELEMETRY_ALL_VALID,
      .imu_cal                = 0xFF,
      .imu_temp               = 25,
      .imu_valid              = LORA_TELEMETRY_ALL_VALID,
      .heading                = 180,
      .roll                   = 5,
      .pitch                  = -3,
      .orientation_valid      = LORA_TELEMETRY_ALL_VALID,
      .rudder_servo_angle     = 10,
      .sail_servo_angle       = -20,
      .mode                   = 1,
      .desired_heading        = 190,
      .route_segment          = 2,
      .drift_distance         = 7,
      .last_cmd_seq           = 0,
  };
  #else
  LoRaTelemetry_t telemetryPacket = {0};
  #endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      __WFI();   /* sleep until any interrupt fires */

      // Every 1 s (TIM6): read the latest telemetry from SRAM4 and send it over LoRa.
      // If the CM7 is mid-write, tx_flag stays set and the read retries on the next wake (~1 ms)
      if (tx_flag) {
        uint8_t telem_status_flags = 0;
        #if defined(CM4_LORA_DUMMY_TELEMETRY)
        // Test mode: skip SRAM4, send the fixed packet with a live uptime
        telemetryPacket.uptime_ms = HAL_GetTick();
        bool telem_ready = true;
        #else
        // Only send LoRa for successful read or 5 busy attempts in a row
        bool telem_ready = telemetryRec_read(&telem_status_flags, &telemetryPacket);
        #endif
        if (telem_ready) {
          tx_flag = 0;
          LoRaTelemetry_t *payload = &telemetryPacket;
          uint8_t plen = sizeof(LoRaTelemetry_t);
          uint8_t pkt[LORA_HDR_LEN_BYTES + plen];
          pkt[0] = ADDR_SAMD21;   /* dest */
          pkt[1] = ADDR_STM32;    /* src */
          pkt[2] = tx_seq++;
          pkt[3] = 1 + plen; // payload len (1 is the telem_status flags)
          pkt[4] = telem_status_flags;
          memcpy(pkt + LORA_HDR_LEN_BYTES, payload, plen);
          LoRa_Send(pkt, LORA_HDR_LEN_BYTES + plen);
          HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
          printf("[TX] seq=%u flags=0x%02X len=%u uptime_ms=%lu\r\n",
                 pkt[2], pkt[4], pkt[3], (unsigned long)payload->uptime_ms);
        }

        
        // LoRa_Send("Hello from CM4!\n", 20);
        // tx_flag = 0;
      }

      if (dio0_flag) {
          dio0_flag = 0;
          LoRa_ProcessDIO0();

          #if defined(CM4_LORA_COMMANDS_ENABLE)
          uint8_t cmd[64];
          uint8_t len = LoRa_GetCmd(cmd);
          if (len > 0) {
            if (len == 21 && cmd[0] == ADDR_STM32 && cmd[3] == 17) 
            {
              ipc_cmd_t c = { .type = cmd[4], .seq = cmd[2]};
              memcpy(c.raw, &cmd[5], IPC_CMD_DATA_SIZE);
              // Without a successful attach the CM7 may not be using SRAM4, or may lay it out differently
              if (!ipc_attached)
              {
                printf("[RX] IPC not attached, dropped\r\n");
              }
              else if (ipc_cm4_push_command(&c) != IPC_STATUS_OK) 
              { 
                printf("[RX] ring full, dropped\r\n"); 
              }
            }
          }
          #endif
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

static void MX_StatusLED_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = STATUS_LED_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_LED_PORT, &GPIO_InitStruct);

    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin   = TICK_LED_PIN;
    HAL_GPIO_Init(TICK_LED_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TICK_LED_PORT, TICK_LED_PIN, GPIO_PIN_RESET);
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
