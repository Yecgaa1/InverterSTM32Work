/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TJCScreen.h"
#include "arm_math.h"
#include "Work.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int recLen_DCDC = 100;
char rec_DCDC[100] = "";
int recLen_ACDC = 100;
char rec_ACDC[100] = "";
char ErrorNote[64] = "";
uint8_t isHighLight = 1;
uint8_t WantWorkState = 0;
uint8_t isError = 0;
uint8_t WorkState = 0; //0初始化中 1待机 2治理 3应急 4错误
volatile uint8_t initing_DCDC = 1, initing_ACDC = 1;
uint16_t adc_buffer[1024] = {0};
float GetAC_V();
float now_v = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
    //UART3 is DCDC
    //UART1 is ACDC
    //UART2 is screen
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    TJCScreenInit(&huart2);

    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *) &adc_buffer, 1024);
    HAL_TIM_Base_Start_IT(&htim6);
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        //更新市电电压
        HAL_Delay(50);
        now_v = GetAC_V();

        //状态转换
        if (WantWorkState != WorkState) {
            if (WorkState == 0) {
                if (WantWorkState == 1) {
                    //希望启动
                    TurnON();
                    TJCSendTxt("state", "待机");
                    WorkState = 1;
                } else {
                    //不能进入的模式
                    WantWorkState = WorkState;
                }
            } else if (WorkState == 1) {
                //运行状态下希望的状态转换
                if (WantWorkState == 0) {
                    TurnOFF();
                    TJCSendTxt("state", "未开机");
                    TJCSendTxt("error", "未开机");
                } else if (WantWorkState == 2) {
                    //希望进入治理模式，需要先判断市电是是否插入
                    if (now_v > 180) {
                        //市电条件满足
                        HAL_GPIO_WritePin(AC_FLOW_CTRL_GPIO_Port, AC_FLOW_CTRL_Pin, GPIO_PIN_RESET); //向市电方向
                        HAL_GPIO_WritePin(AC_OUTPUT_CTRL_GPIO_Port, AC_OUTPUT_CTRL_Pin, GPIO_PIN_SET); //打开市电输出
                        HAL_Delay(1000);
                        TurnON_OUTPUT(); //开启放电输出
                        TJCSendTxt("state", "电压治理模式");
                        WorkState = 2;
                    } else {
                        //市电条件不满足
                        WantWorkState = WorkState = 4; //进入错误模式
                        TJCSendTxt("state", "故障");
                        TJCSendTxt("error", "市电未接入时不可启动电压治理");
                    }
                } else if (WantWorkState == 3) {
                    //如果想进入应急放电模式
                    if (now_v < 30) {
                        HAL_GPIO_WritePin(AC_FLOW_CTRL_GPIO_Port, AC_FLOW_CTRL_Pin, GPIO_PIN_SET); //向插头方向
                        HAL_GPIO_WritePin(AC_OUTPUT_CTRL_GPIO_Port, AC_OUTPUT_CTRL_Pin, GPIO_PIN_SET); //打开市电输出
                        HAL_Delay(1000);
                        TurnON_OUTPUT(); //开启放电输出
                        TJCSendTxt("state", "应急放电模式");
                        WorkState = 3;
                    } else {
                        //市电条件不满足
                        WantWorkState = WorkState = 4; //进入错误模式
                        TJCSendTxt("state", "故障");
                        TJCSendTxt("error", "市电插入时不可启动应急供电");
                    }
                } else {
                    //不能进入的模式
                    WantWorkState = WorkState;
                }
            } else if (WorkState == 2) //目前在治理模式
            {
                if (WantWorkState == 1) {
                    //希望进入待机模式
                    Restart(Normal);
                    TJCSendTxt("state", "待机");
                    WorkState = 1;
                } else {
                    //不能进入的模式
                    WantWorkState = WorkState;
                }
            } else if (WorkState == 3) //目前在应急放电模式
            {
                if (WantWorkState == 1) {
                    //希望进入待机模式
                    Restart(Normal);
                    TJCSendTxt("state", "待机");
                    WorkState = 1;
                } else {
                    //不能进入的模式
                    WantWorkState = WorkState;
                }
            } else if (WorkState == 4) //目前在故障模式
            {
                if (WantWorkState == 1) {
                    //希望进入待机模式
                    Restart(ExitError);
                    TJCSendTxt("state", "待机");
                    WorkState = 1;
                } else {
                    //不能进入的模式
                    WantWorkState = WorkState;
                }
            }
        }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_4)
  {
  }
  LL_PWR_EnableRange1BoostMode();
  LL_RCC_HSE_Enable();
   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
  }

  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 85, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_ConfigDomain_ADC(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_3, 85, LL_RCC_PLLP_DIV_20);
  LL_RCC_PLL_EnableDomain_SYS();
  LL_RCC_PLL_EnableDomain_ADC();
  LL_RCC_PLL_Enable();
   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Insure 1us transition state at intermediate medium speed clock*/
  for (__IO uint32_t i = (170 >> 1); i !=0; i--);

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(170000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float GetAC_V() {
    HAL_ADC_Stop_DMA(&hadc2);
    float32_t rms_303 = 0.0f;
    float32_t rms_buffer[1024] = {0};
    for (int i = 0; i < 1024; i++) {
        rms_buffer[i] = (float) adc_buffer[i] / 4096.0f * 3.3f - (3.3f / 4.0f * 2.f);
        //1:(3+1),2 Maginify
        // printf("%f\n", rms_buffer[i]);
    }
    int zeroCrossings[10];
    int zeroCrossingCount = 0;

    for (int i = 0; i < 1023; i++) {
        if (rms_buffer[i] <= 0 && rms_buffer[i + 1] > 0) {
            zeroCrossings[zeroCrossingCount++] = i;
            if (zeroCrossingCount >= 10) {
                break;
            }
        }
    }
    if (zeroCrossingCount >= 3) {
        const int startIdx = zeroCrossings[0];
        const int endIdx = zeroCrossings[3];
        const int length = endIdx - startIdx;

        arm_rms_f32(&rms_buffer[startIdx], length, &rms_303);
        rms_303 = rms_303 / 0.47f * 220.0f / 0.75f / 2.f;
    } else {
        rms_303 = 0;
    }
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *) &adc_buffer, 1024);
    return rms_303;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Button1_Pin) {
        if (HAL_GPIO_ReadPin(Button1_GPIO_Port,Button1_Pin) == GPIO_PIN_RESET) {
            //能量单元启停按钮
            WantWorkState = 0;
        }
        __HAL_GPIO_EXTI_CLEAR_IT(Button1_Pin);
    } else if (GPIO_Pin == Button2_Pin) {
        if (HAL_GPIO_ReadPin(Button2_GPIO_Port,Button2_Pin) == GPIO_PIN_RESET) {
            //待机/复位按钮
            WantWorkState = 1;
        }
        __HAL_GPIO_EXTI_CLEAR_IT(Button2_Pin);
    } else if (GPIO_Pin == Button3_Pin) {
        if (HAL_GPIO_ReadPin(Button3_GPIO_Port,Button3_Pin) == GPIO_PIN_RESET) {
            //电压治理模式按钮
            WantWorkState = 2;
        }
        __HAL_GPIO_EXTI_CLEAR_IT(Button3_Pin);
    } else if (GPIO_Pin == Button4_Pin) {
        if (HAL_GPIO_ReadPin(Button4_GPIO_Port,Button4_Pin) == GPIO_PIN_RESET) {
            WantWorkState = 3;
            //应急发电模式按钮
        }
        __HAL_GPIO_EXTI_CLEAR_IT(Button4_Pin);
    } else if (GPIO_Pin == Button5_Pin) {
        if (HAL_GPIO_ReadPin(Button5_GPIO_Port,Button5_Pin) == GPIO_PIN_RESET) {
            //面板亮度调节按钮
            if (isHighLight == 1) {
                isHighLight = 0;
                TJCSendAny("dim=30");
            } else {
                isHighLight = 1;
                TJCSendAny("dim=100");
            }
        }
        __HAL_GPIO_EXTI_CLEAR_IT(Button5_Pin);
    }
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
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
