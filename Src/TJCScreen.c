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
    TCJSendEnd();
}

void TCJSendEnd() {
    HAL_UART_Transmit(gHuart, (uint8_t *) "\xff\xff\xff", 3, 0xffff);
}

void TCJSendValue(char *name, int value) {
    char tmp[30];
    sprintf(tmp, "%s.val=%d", name, value);
    HAL_UART_Transmit(gHuart, (uint8_t *) tmp, strlen(tmp), 0xffff);
    TCJSendEnd();
}

void TCJSendTxt(char *name, char *value) {
    char tmp[30];
    sprintf(tmp, "%s.txt=\"%s\"", name, value);
    HAL_UART_Transmit(gHuart, (uint8_t *) tmp, strlen(tmp), 0xffff);
    TCJSendEnd();
}

void TCJSendAnyProperty(char *object_name, char *property, char *value) {
    char tmp[30];
    sprintf(tmp, "%s.%s=%s", object_name, property, value);
    HAL_UART_Transmit(gHuart, (uint8_t *) tmp, strlen(tmp), 0xffff);
    TCJSendEnd();
}

void TCJSendAny(char *any) {
    HAL_UART_Transmit(gHuart, (uint8_t *) any, strlen(any), 0xffff);
    TCJSendEnd();
}