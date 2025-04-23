//
// Created by xtx on 2023/7/14.
//

#include <string.h>
#include <stdio.h>
#include "TJCScreen.h"
#include "main.h"
UART_HandleTypeDef *gHuart;

void TJCScreenInit(UART_HandleTypeDef *huart) {
    gHuart = huart;
    TJCSendEnd();
}

void TJCSendEnd() {
    HAL_UART_Transmit(gHuart, (uint8_t *) "\xff\xff\xff", 3, 0xffff);
}

void TJCSendValue(char *name, int value) {
    char tmp[30]={0};
    sprintf(tmp, "%s.val=%d", name, value);
    HAL_UART_Transmit(gHuart, (uint8_t *) tmp, strlen(tmp), 0xffff);
    TJCSendEnd();
}

void TJCSendTxt(char *name, char *value) {
    char tmp[30]={0};
    sprintf(tmp, "%s.txt=\"%s\"", name, value);
    HAL_UART_Transmit(gHuart, (uint8_t *) tmp, strlen(tmp), 0xffff);
    TJCSendEnd();
}

void TJCSendAnyProperty(char *object_name, char *property, char *value) {
    char tmp[30]={0};
    sprintf(tmp, "%s.%s=%s", object_name, property, value);
    HAL_UART_Transmit(gHuart, (uint8_t *) tmp, strlen(tmp), 0xffff);
    TJCSendEnd();
}

void TJCSendAny(char *any) {
    HAL_UART_Transmit(gHuart, (uint8_t *) any, strlen(any), 0xffff);
    TJCSendEnd();
}