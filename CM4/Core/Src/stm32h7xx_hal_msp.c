/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32h7xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/*
  USART3 Peripheral is hard wired to the ST-Link com port (the micro usb port on the board) so we can just use that 
  instead of hooking up the usart to serial converter.
*/

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hspi->Instance == SPI1)
    {
      __HAL_RCC_SPI1_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOD_CLK_ENABLE();
  

      /* PA5=SCK, PA6=MISO */
      __HAL_RCC_GPIOA_CLK_ENABLE();
      GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6;  // We cant set mosi here because it uses PB5 ie GPIOB, do it in next HAL_GPIO_Init call
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* PB5=MOSI */
      __HAL_RCC_GPIOB_CLK_ENABLE();
      GPIO_InitStruct.Pin       = GPIO_PIN_5;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
      GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* PD14=CS, software-controlled meaning not by the SPI hardware, we change it in code */
      GPIO_InitStruct.Pin       = GPIO_PIN_14;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
      GPIO_InitStruct.Alternate = 0;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

      /* CS idle high */
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

      /* PD15 = RESET, output */
      GPIO_InitStruct.Pin       = GPIO_PIN_15;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_MEDIUM;
      GPIO_InitStruct.Alternate = 0;
      HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

      /* RESET idle high */
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
    }
}



void Debug_LED_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();  // LD1 is PB0,
    __HAL_RCC_GPIOE_CLK_ENABLE();  // LD2

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_14;  // LD1 green, LD3 red
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // start both off
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14, GPIO_PIN_RESET);

    //yellow
    GPIO_InitStruct.Pin   = GPIO_PIN_1;  // LD2 yellow
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);  // set yellow LED low to indicate we're in main and ready to init lora

}



void Debug_LED_Toggle(char color)
{
    if (color == 'g')
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);   // green LD1
    else if (color == 'r')
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);  // red LD3
    else if (color == 'y')
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1);   // yellow LD2
    }
}

/* USER CODE END 1 */
