/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAN_CTRL_Pin GPIO_PIN_0
#define FAN_CTRL_GPIO_Port GPIOC
#define POWERON_SWITCH_Pin GPIO_PIN_1
#define POWERON_SWITCH_GPIO_Port GPIOC
#define OUTPUT_SWITCH_Pin GPIO_PIN_2
#define OUTPUT_SWITCH_GPIO_Port GPIOC
#define AC_FLOW_CTRL_Pin GPIO_PIN_3
#define AC_FLOW_CTRL_GPIO_Port GPIOC
#define AC_OUTPUT_CTRL_Pin GPIO_PIN_1
#define AC_OUTPUT_CTRL_GPIO_Port GPIOA
#define Button1_Pin GPIO_PIN_1
#define Button1_GPIO_Port GPIOB
#define Button1_EXTI_IRQn EXTI1_IRQn
#define Button2_Pin GPIO_PIN_2
#define Button2_GPIO_Port GPIOB
#define Button2_EXTI_IRQn EXTI2_IRQn
#define Button3_Pin GPIO_PIN_3
#define Button3_GPIO_Port GPIOB
#define Button3_EXTI_IRQn EXTI3_IRQn
#define Button4_Pin GPIO_PIN_4
#define Button4_GPIO_Port GPIOB
#define Button4_EXTI_IRQn EXTI4_IRQn
#define Button5_Pin GPIO_PIN_5
#define Button5_GPIO_Port GPIOB
#define Button5_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
