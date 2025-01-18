//
// Created by xtx on 2023/7/14.
//

#ifndef F407ZGT6_LEDTEST_SRC_TJCSCREEN_H_
#define F407ZGT6_LEDTEST_SRC_TJCSCREEN_H_

#include "stm32g4xx_hal.h"
#include "string.h"
#include "stdio.h"
void TJCScreenInit(UART_HandleTypeDef *huart);

void TCJSendEnd();

void TCJSendValue(char *name, int value);

void TCJSendTxt(char *name, char *value);

void TCJSendAnyProperty(char *object_name, char *property, char *value);

void TCJSendAny(char *any);

#endif //F407ZGT6_LEDTEST_SRC_TJCSCREEN_H_
