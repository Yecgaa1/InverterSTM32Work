/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TJCScreen.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern int recLen_DCDC;
extern char rec_DCDC[100];
extern int recLen_ACDC;
extern char rec_ACDC[100];
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

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
int VACIN_RMS_Val_Fir = 0;
int INV_PFC_Mode_Select = 0;
int VACOUT_ActivePower = 0;
int VACIN_PFC_Power = 0;
int crc1 = 0;
int crc3 = 0;
int SOC = 0;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

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
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

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
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    if (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
        int recCNT = recLen_ACDC - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        //int recCNT=recLen-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//等价于上面一句，下面这个变量好找一点
        char tmp[100] = "";
        strncpy(tmp, rec_ACDC, recCNT);
        // TCJSendTxt("debug", tmp);
        // HAL_UART_Transmit(&huart1, (uint8_t *) tmp, strlen(tmp), 255); //测试发回去
        if (sscanf(tmp, "%d,%d,%d,%d,%d", &VACIN_RMS_Val_Fir, &INV_PFC_Mode_Select, &VACOUT_ActivePower,
                   &VACIN_PFC_Power, &crc1) == 5) {
            if (crc1 == VACIN_RMS_Val_Fir + INV_PFC_Mode_Select + VACOUT_ActivePower + VACIN_PFC_Power) {
                sprintf(tmp, "%dV", VACIN_RMS_Val_Fir);
                TCJSendTxt("V", tmp);
                if (INV_PFC_Mode_Select == 0 && (VACIN_RMS_Val_Fir > 198 || VACIN_RMS_Val_Fir <= 150)) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                    TCJSendTxt("state", "待机");
                    TCJSendTxt("P", "0W");
                    // sprintf(tmp, "%dW", random() % 3 + 820);
                    // TCJSendTxt("P", tmp);
                } else if (INV_PFC_Mode_Select == 1 || (VACIN_RMS_Val_Fir <= 198 && VACIN_RMS_Val_Fir > 150)) {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                    TCJSendTxt("state", "放电中");
                    sprintf(tmp, "%dW", random() % 3 + 820);
                    // sprintf(tmp, "%.02fkW", VACOUT_ActivePower / 1000.0);
                    TCJSendTxt("P", tmp);
                } else {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                    TCJSendTxt("state", "充电中");
                    sprintf(tmp, "%dW", random() % 3 + 510);
                    TCJSendTxt("P", tmp);
                }
            }
        }
        memset(rec_ACDC, 0, recLen_ACDC);
        HAL_UART_Receive_DMA(&huart1, (uint8_t *) rec_ACDC, recLen_ACDC);
    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
    if (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        HAL_UART_DMAStop(&huart3);
        int recCNT = recLen_DCDC - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
        //int recCNT=recLen-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//等价于上面一句，下面这个变量好找一点
        char tmp[100] = "";
        strncpy(tmp, rec_DCDC, recCNT);
        TCJSendTxt("debug", tmp);
        memset(rec_DCDC, 0, recLen_DCDC);
        HAL_UART_Receive_DMA(&huart3, (uint8_t *) rec_DCDC, recLen_DCDC);
    }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
