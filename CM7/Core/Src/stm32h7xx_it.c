/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "stm32h7xx_hal_gpio.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/

//...

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
  uint32_t type;      /* 0=Hard 1=MemManage 2=Bus 3=Usage */
  uint32_t r0, r1, r2, r3, r12, lr, pc, psr;
  uint32_t excReturn, sp;
  uint32_t cfsr, hfsr, dfsr, afsr, mmfar, bfar;
  char     task[configMAX_TASK_NAME_LEN];
} FaultInfo_t;

/* Read this in the debugger after a freeze. pc = faulting instruction, mmfar/bfar = bad address
 * (valid only if the matching MMARVALID/BFARVALID bit is set in cfsr). */
volatile FaultInfo_t g_fault;

void fault_capture(uint32_t *frame, uint32_t excReturn, uint32_t type) __attribute__((used, noreturn));
void fault_capture(uint32_t *frame, uint32_t excReturn, uint32_t type)
{
  __disable_irq();
  g_fault.type = type;
  g_fault.r0 = frame[0]; g_fault.r1 = frame[1]; g_fault.r2 = frame[2]; g_fault.r3 = frame[3];
  g_fault.r12 = frame[4]; g_fault.lr = frame[5]; g_fault.pc = frame[6]; g_fault.psr = frame[7];
  g_fault.excReturn = excReturn;
  g_fault.sp = (uint32_t)frame;
  g_fault.cfsr = SCB->CFSR; g_fault.hfsr = SCB->HFSR; g_fault.dfsr = SCB->DFSR;
  g_fault.afsr = SCB->AFSR; g_fault.mmfar = SCB->MMFAR; g_fault.bfar = SCB->BFAR;
  TaskHandle_t t = xTaskGetCurrentTaskHandle();
  const char *n = (t != NULL) ? pcTaskGetName(t) : "(no task)";
  for (int i = 0; i < configMAX_TASK_NAME_LEN - 1 && n[i]; i++) { g_fault.task[i] = n[i]; }
  if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) { __BKPT(0); }
  for (;;) { }
}

/* Naked so the stack pointer is untouched when we pick the stacked exception frame. */
#define FAULT_TRAMPOLINE(id) __asm volatile( \
  "tst lr, #4      \n" \
  "ite eq          \n" \
  "mrseq r0, msp   \n" \
  "mrsne r0, psp   \n" \
  "mov r1, lr      \n" \
  "movs r2, %0     \n" \
  "b fault_capture \n" :: "i"(id))
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
//...

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
__attribute__((naked)) void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  FAULT_TRAMPOLINE(0);

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
__attribute__((naked)) void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  FAULT_TRAMPOLINE(1);

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
__attribute__((naked)) void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  FAULT_TRAMPOLINE(2);

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
__attribute__((naked)) void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  FAULT_TRAMPOLINE(3);

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

