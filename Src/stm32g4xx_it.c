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
#include <stdbool.h>

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
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
int VACIN_RMS_Val_Fir = 0;
int INV_PFC_Mode_Select = 0;
int VACOUT_ActivePower = 0;
int VACIN_PFC_Power = 0;
int ACDC_ErrorCode = 0;
int DCDC_ErrorCode;
int crc1 = 0;
int crc3 = 0;
int SOC = 0;
bool is_emergency_output = false;
extern char ErrorNote[64];
/**
 * 解析DCDC系统保护标志，获取错误代码和简短描述
 * @param errorCode 错误码，等于System_Protect_Flag_BITS的所有位
 * @param description 输出参数，用于存储错误描述
 * @param descSize description缓冲区大小
 * @return 错误代码(1-16)，如果无错误返回0
 */
int DCDC_DecodeSystemProtectFlag(unsigned short int errorCode, char *description, int descSize) {
    // 检查是否有错误位被设置
    if (errorCode == 0) {
        snprintf(description, descSize, "无错误");
        return 0;
    }

    // 定义错误描述数组
    const char *DCDC_errorDescriptions[] = {
        "过载", // 位 0 (0x0001) OLP
        "vBus过压", // 位 1 (0x0002) vBus_OVP
        "vBus欠压", // 位 2 (0x0004) vBus_LVP
        "输出过压", // 位 3 (0x0008) RES_OVP
        "输出欠压", // 位 4 (0x0010) RES_LVP
        "参考电压错误", // 位 5 (0x0020) vref_ERR
        "辅助电源过压", // 位 6 (0x0040) auxPower_OVP
        "辅助电源欠压", // 位 7 (0x0080) auxPower_LVP
        "过温保护", // 位 8 (0x0100) OTP
        "电池充电过流", // 位 9 (0x0200) iBAT_CHG_OCP
        "电池放电过流", // 位 10 (0x0400) iBAT_DISC_OCP
        "电池过压", // 位 11 (0x0800) vBAT_OVP
        "电池欠压", // 位 12 (0x1000) vBAT_LVP
        "系统初始化失败", // 位 13 (0x2000) sys_Init_Fail
        "ACDC错误", // 位 14 (0x4000) INV_ERR
        "通信错误" // 位 15 (0x8000) communica_ERR
    };

    // 查找第一个被设置的位(最低有效位)
    int firstErrorBit = -1;
    for (int i = 0; i < 16; i++) {
        if (errorCode & (1 << i)) {
            firstErrorBit = i;
            break;
        }
    }

    // 如果没有找到错误位，返回-1
    if (firstErrorBit == -1) {
        snprintf(description, descSize, "未知错误");
        return -1;
    }

    // 计算有多少位被设置
    int errorCount = 0;
    for (int i = 0; i < 16; i++) {
        if (errorCode & (1 << i)) {
            errorCount++;
        }
    }

    // 如果只有一个错误，只提供该错误的描述
    if (errorCount == 1) {
        snprintf(description, descSize, "%s", DCDC_errorDescriptions[firstErrorBit]);
    } else {
        // 多个错误，提供主要错误和额外错误数量
        snprintf(description, descSize, "%s +%d个错误",
                 DCDC_errorDescriptions[firstErrorBit], errorCount - 1);
    }

    // 返回错误代码(从1开始，所以位置+1)
    return firstErrorBit + 1;
}

/**
 * 解析系统保护标志，获取错误代码和简短描述
 * @param errorCode 错误码，等于System_ProtectFlag_Info.all
 * @param description 输出参数，用于存储错误描述
 * @param descSize description缓冲区大小
 * @return 错误代码(1-15)，如果无错误返回0
 */
int ACDC_DecodeSystemProtectFlag(unsigned short int errorCode, char *description, int descSize) {
    // 检查是否有错误位被设置
    if (errorCode == 0) {
        snprintf(description, descSize, "无错误");
        return 0;
    }

    // 定义错误描述数组
    const char *ACDC_errorDescriptions[] = {
        "过载", // 位 0 (0x0001)
        "vBus过压", // 位 1 (0x0002)
        "vBus欠压", // 位 2 (0x0004)
        "AC输出过压", // 位 3 (0x0008)
        "AC输出欠压", // 位 4 (0x0010)
        "AC输出短路", // 位 5 (0x0020)
        "辅助电源过压", // 位 6 (0x0040)
        "辅助电源欠压", // 位 7 (0x0080)
        "过温保护", // 位 8 (0x0100)
        "负载过流", // 位 9 (0x0200)
        "电感过流", // 位 10 (0x0400)
        "系统初始化失败", // 位 11 (0x0800)
        "DCDC错误", // 位 12 (0x1000)
        "通信错误", // 位 13 (0x2000)
        "参考电压错误" // 位 14 (0x4000)
    };

    // 查找第一个被设置的位(最低有效位)
    int firstErrorBit = -1;
    for (int i = 0; i < 15; i++) {
        if (errorCode & (1 << i)) {
            firstErrorBit = i;
            break;
        }
    }

    // 如果没有找到错误位，返回-1
    if (firstErrorBit == -1) {
        snprintf(description, descSize, "未知错误");
        return -1;
    }

    // 计算有多少位被设置
    int errorCount = 0;
    for (int i = 0; i < 15; i++) {
        if (errorCode & (1 << i)) {
            errorCount++;
        }
    }

    // 如果只有一个错误，只提供该错误的描述
    if (errorCount == 1) {
        snprintf(description, descSize, "%s", ACDC_errorDescriptions[firstErrorBit]);
    } else {
        // 多个错误，提供主要错误和额外错误数量
        snprintf(description, descSize, "%s +%d个错误",
                 ACDC_errorDescriptions[firstErrorBit], errorCount - 1);
    }

    // 返回错误代码(从1开始，所以位置+1)
    return firstErrorBit + 1;
}

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
        char tmp[100] = {0};
        strncpy(tmp, rec_ACDC, recCNT);
        // TCJSendTxt("error", tmp);

        //分离数据
        if (sscanf(tmp, "%d,%d,%d,%d,%d,%d", &VACIN_RMS_Val_Fir, &INV_PFC_Mode_Select, &VACOUT_ActivePower,
                   &VACIN_PFC_Power, &ACDC_ErrorCode, &crc1) == 6) {
            //校验
            if (crc1 == VACIN_RMS_Val_Fir + INV_PFC_Mode_Select + VACOUT_ActivePower + VACIN_PFC_Power +
                ACDC_ErrorCode) {
                //先是逆变器采集到的市电电压
                sprintf(tmp, "%dV", VACIN_RMS_Val_Fir);
                TCJSendTxt("V", tmp);

                //根据模式区分显示内容
                if (INV_PFC_Mode_Select == 0) //待机
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
                    TCJSendTxt("state", "待机");
                    TCJSendTxt("P", "0W");
                } else if (INV_PFC_Mode_Select == 1) //放电
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

                    if (is_emergency_output) //应急放电
                    {
                        TCJSendTxt("state", "应急放电中");
                    } else //正常并网放电
                    {
                        TCJSendTxt("state", "放电中");
                    }

                    sprintf(tmp, "%dW", VACOUT_ActivePower);
                    TCJSendTxt("P", tmp);
                } else //充电
                {
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
                    TCJSendTxt("state", "充电中");
                    sprintf(tmp, "%dW", VACIN_PFC_Power);
                    TCJSendTxt("P", tmp);
                }

                //解析错误代码
                char description[64] = {0};
                const int errorCode = ACDC_DecodeSystemProtectFlag(ACDC_ErrorCode, description, 100);
                if (errorCode != 0 ) {
                    if (errorCode != 13) {
                        sprintf(ErrorNote, "%s", description);
                    }
                } else {
                    sprintf(ErrorNote, "正常运行");
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
        if (sscanf(tmp, "%d,%d", &SOC, &DCDC_ErrorCode, &crc3) == 2) {
            if (SOC + DCDC_ErrorCode == crc3) {
                //开始解析
                sprintf(tmp, "%d%%", SOC);
                TCJSendTxt("soc", tmp);

                //解析错误代码
                char description[64] = {0};
                const int errorCode = DCDC_DecodeSystemProtectFlag(DCDC_ErrorCode, description, 100);
                if (errorCode != 0) {
                    if (errorCode != 15) {
                        sprintf(ErrorNote, "%s", description);
                    }
                } else {
                    sprintf(ErrorNote, "正常运行");
                }
            }
        }
        memset(rec_DCDC, 0, recLen_DCDC);
        HAL_UART_Receive_DMA(&huart3, (uint8_t *) rec_DCDC, recLen_DCDC);
    }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
