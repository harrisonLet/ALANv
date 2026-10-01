/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "shared_configuration.h"

#include "portmacro.h"

#include "stm32h7xx_hal_conf.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"

#include "controller.h"


/* Private includes ----------------------------------------------------------*/

//...

/* Private typedef -----------------------------------------------------------*/

//...

/* Private define ------------------------------------------------------------*/

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif

/* Private macro -------------------------------------------------------------*/

//...

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

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

  uint16_t timeout = 0xFFFF;

  #if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 ) { Error_Handler(); }
  #endif
  
  HAL_Init();
  SystemClock_Config();

  #if defined(CM7_ENABLE)
  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) { Error_Handler(); }
  setvbuf(stdout, NULL, _IONBF, 0);
  #endif

  // Should already be in your STM32H7 HAL init but verify:
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  // enable FPU

  #if defined(CM7_ENABLE)
  hardware_init();
  #endif

  #if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE) && defined(CM4_ENABLE)
  HAL_HSEM_Release(HSEM_ID_0,0);
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if ( timeout < 0 ) { Error_Handler(); }
  #endif

  #if defined(CM7_ENABLE)
  rtos_init();
  vTaskStartScheduler();
  #endif
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
