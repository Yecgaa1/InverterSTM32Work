//
// Created by xtx on 2023/7/14.
//

#ifndef F407ZGT6_LEDTEST_SRC_TJCSCREEN_H_
#define F407ZGT6_LEDTEST_SRC_TJCSCREEN_H_

#include "stm32g4xx_hal.h"
#include "string.h"
#include "stdio.h"
void TJCScreenInit(UART_HandleTypeDef *huart);

void TJCSendEnd();

void TJCSendValue(char *name, int value);

void TJCSendTxt(char *name, char *value);

void TJCSendAnyProperty(char *object_name, char *property, char *value);

void TJCSendAny(char *any);

void TJCSendTxtWithFloat(char *name, float value);

#endif //F407ZGT6_LEDTEST_SRC_TJCSCREEN_H_
