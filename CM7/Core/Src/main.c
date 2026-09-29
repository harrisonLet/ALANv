/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "cmsis_os2.h"
#include "portmacro.h"
#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include <stdint.h>

#include "button.h"

#include "servoSail.h"
#include "servoRudder.h"

#include "sensorWind.h"
#include "sensorMagnetometer.h"
#include "sensorEncoder.h"
#include "sensorGPS.h"

/* Private includes ----------------------------------------------------------*/

//...

/* Private typedef -----------------------------------------------------------*/

//...

/* Private define ------------------------------------------------------------*/

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* Private macro -------------------------------------------------------------*/

//...

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

TaskHandle_t task_live;

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
void hardware_init(void);
void rtos_init(void);

void live_hardwareInit();
void live_handler(void *argument);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  // Dual Boot Sequence --------------------------------------------------------
  // This sequence is based on the hardware semaphore IP, and ensures that both 
  // cores are correctly booted and can run independently.
  //
  // It works as follows:
  // 1) When the CM7 boots, it will wait until the CM4 boots and enters in stop mode (RCC_FLAG_D2CKRDY flag is reset)
  // 2) When the CM4 is in stop mode, the CM7 will release the CM4 by means of a hardware semaphore notification (HSEM_ID_0)
  // 3) When the CM4 receives the notification, it will exit stop mode and the RCC_FLAG_D2CKRDY flag will be set, which the CM7 is waiting for in step 1)
  // 4) When the CM7 detects that the CM4 is out of stop mode, it will continue 
  // 5) Perform the hardware initialization 
  // 6) Start the RTOS scheduler.

  #if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
    uint16_t timeout = 0xFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
    if ( timeout < 0 ) { Error_Handler(); }
  #endif
  
  hardware_init();
  rtos_init();
  vTaskStartScheduler();
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

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

/**
  * @brief  Initialize the hardware components.
  * 1) HAL_Init() is called to initialize the Hardware Abstraction Layer, which will set up the system clock, configure the SysTick timer, and perform any necessary low-level hardware initialization.
  * 2) User defined hardware setup.
  * 3) SystemClock_Config() is called to configure the system clock according to the application's requirements. This function typically sets up the main system clock source, configures the PLL (Phase-Locked Loop) if used, and sets the appropriate clock dividers for the AHB and APB buses.
  * 4) Release the CM4 after hardware initialization is done.
  * 5) Start the COM1 serial port for debugging purposes.
  *
  * @retval None
  */
void hardware_init(void)
{
  HAL_Init();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_UART4_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_UART7_CLK_ENABLE();

  SystemClock_Config();

  #if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
    __HAL_RCC_HSEM_CLK_ENABLE();
    HAL_HSEM_FastTake(HSEM_ID_0);
    HAL_HSEM_Release(HSEM_ID_0,0);
    uint16_t timeout = 0xFFFF;
    while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
    if ( timeout < 0 ) { Error_Handler(); }
  #endif

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  // Should already be in your STM32H7 HAL init but verify:
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  // enable FPU

  /* USER CODE BEGIN SysInit */
  live_hardwareInit();
  button_hardwareInit();

  servoSail_hardwareInit();
  servoRudder_hardwareInit();

  sensorWind_hardwareInit();
  sensorMagnetometer_hardwareInit();
  sensorEncoder_hardwareInit();
  sensorGPS_hardwareInit();
  /* USER CODE END SysInit */

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
}

/**
  * @brief  Initialize the Real-Time Operating System and all of its components.
  * @retval None
  */
void rtos_init()
{
  if ((semphr_button = xSemaphoreCreateBinary()) == NULL) { Error_Handler(); }

  if (xTaskCreate(live_handler,               "liveTask",               64,  NULL, osPriorityNormal,      &task_live)               != pdPASS) { Error_Handler(); }
  if (xTaskCreate(button_handler,             "buttonTask",             64,  NULL, osPriorityNormal,      &task_button)             != pdPASS) { Error_Handler(); }
  if (xTaskCreate(servoSail_handler,          "servoSailTask",          128, NULL, osPriorityNormal,      &task_servoSail)          != pdPASS) { Error_Handler(); }
  if (xTaskCreate(servoRudder_handler,        "servoRudderTask",        128, NULL, osPriorityNormal,      &task_servoRudder)        != pdPASS) { Error_Handler(); }
  if (xTaskCreate(sensorWind_handler,         "sensorWindTask",         512, NULL, osPriorityAboveNormal, &task_sensorWind)         != pdPASS) { Error_Handler(); }
  if (xTaskCreate(sensorMagnetometer_handler, "sensorMagnetometerTask", 128, NULL, osPriorityAboveNormal, &task_sensorMagnetometer) != pdPASS) { Error_Handler(); }
  if (xTaskCreate(sensorEncoder_handler,      "sensorEncoderTask",      256, NULL, osPriorityAboveNormal, &task_sensorEncoder)      != pdPASS) { Error_Handler(); }
  if (xTaskCreate(sensorGPS_handler,          "sensorGPSTask",          512, NULL, osPriorityAboveNormal, &task_sensorGPS)          != pdPASS) { Error_Handler(); }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
// Live Functions
///////////////////////////////////////////////////////////////////////////////////////////////////

void live_hardwareInit()
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void live_handler(void *argument)
{
  for(;;)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    vTaskDelay(1000 * portTICK_PERIOD_MS);
  }
}