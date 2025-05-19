//
// Created by xtx on 25-4-24.
//

#ifndef WORK_H
#define WORK_H
#include <stdint.h>
enum RestartReason {
    Normal = 1,
    ExitError = 2
};
extern volatile uint8_t initing_DCDC, initing_ACDC;
extern int V, P;
extern float BatteryVoltage;
void TurnON();
void TurnOFF();
void TurnON_OUTPUT();
void TurnOFF_OUTPUT();
void Restart(enum RestartReason reason);
void RefreshData();
#endif //WORK_H
