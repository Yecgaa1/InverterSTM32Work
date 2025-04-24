//
// Created by xtx on 25-4-24.
//

#include "Work.h"

#include <stdint.h>

#include "stm32g4xx.h"
#include "TJCScreen.h"
#include "usart.h"
uint8_t isAllowOutput = 0;
uint8_t isOn = 0;
extern int recLen_DCDC;
extern char rec_DCDC[100];
extern int recLen_ACDC;
extern char rec_ACDC[100];
extern float now_v;
void TurnON_OUTPUT() {
    if (isAllowOutput == 1) {
        return;
    }
    HAL_GPIO_WritePin(OUTPUT_SWITCH_GPIO_Port,OUTPUT_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(OUTPUT_SWITCH_GPIO_Port,OUTPUT_SWITCH_Pin, GPIO_PIN_RESET);
    isAllowOutput = 1;
}

void TurnOFF_OUTPUT() {
    if (isAllowOutput == 0) {
        return;
    }
    HAL_GPIO_WritePin(OUTPUT_SWITCH_GPIO_Port,OUTPUT_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(OUTPUT_SWITCH_GPIO_Port,OUTPUT_SWITCH_Pin, GPIO_PIN_RESET);
    isAllowOutput = 0;
}

void TurnON() {
    if (isOn) {
        return;
    }
    TJCSendTxt("error", "逆变器开机中");
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, rec_ACDC, recLen_ACDC);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, rec_DCDC, recLen_DCDC);

    //开机
    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(4000);
    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_RESET);

    TJCSendTxt("error", "等待逆变器响应");
    while (initing_ACDC || initing_DCDC) {
    }

    if (now_v < 100) {
        //事实上就是没市电
        TJCSendTxt("error", "正常运行（市电未接入）");
    } else {
        TJCSendTxt("error", "正常运行");
    }

    isOn = 1;
}

void TurnOFF() {
    if (isOn == 0) {
        return;
    }

    TJCSendTxt("error", "正在关机");
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
    TurnOFF_OUTPUT();

    HAL_GPIO_WritePin(AC_OUTPUT_CTRL_GPIO_Port, AC_OUTPUT_CTRL_Pin, GPIO_PIN_RESET); //关闭市电输出

    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(4000);
    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_RESET);

    isOn = 0;
}

void Restart(enum RestartReason reason) {
    if (isOn == 0) {
        return;
    }

    if (reason == Normal) {
        TJCSendTxt("error", "正在退出工作状态");
    } else if (reason == ExitError) {
        TJCSendTxt("error", "正在从故障中复位");
    }
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
    TurnOFF_OUTPUT();

    HAL_GPIO_WritePin(AC_OUTPUT_CTRL_GPIO_Port, AC_OUTPUT_CTRL_Pin, GPIO_PIN_RESET); //关闭市电输出

    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(4000);
    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_RESET);

    isOn = 0;

    //避免显示开机中
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, rec_ACDC, recLen_ACDC);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, rec_DCDC, recLen_DCDC);

    //执行初始化
    //开机
    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_SET);
    HAL_Delay(4000);
    HAL_GPIO_WritePin(POWERON_SWITCH_GPIO_Port,POWERON_SWITCH_Pin, GPIO_PIN_RESET);
    TJCSendTxt("error", "等待逆变器响应");
    while (initing_ACDC || initing_DCDC) {
    }

    if (now_v < 100) {
        //事实上就是没市电
        TJCSendTxt("error", "正常运行（市电未接入）");
    } else {
        TJCSendTxt("error", "正常运行");
    }

    isOn = 1;
}

void RefreshData() {
    if (isOn == 0) {
        TJCSendTxt("soc", "-");
        TJCSendTxt("V", "-");
        TJCSendTxt("P", "-");
    } else {
        char tmp[20] = {0};
        sprintf(tmp, "%d%%", SOC);
        TJCSendTxt("soc", tmp);
        sprintf(tmp, "%dV", V);
        TJCSendTxt("V", tmp);
        sprintf(tmp, "%dW", P);
        TJCSendTxt("P", tmp);
    }
}
