#ifndef TimerInterrupts_h
#define TimerInterrupts_h

#include "Arduino.h"

const uint16_t t3_initVal = 0;
const uint16_t t3_compareVal = 32000;

const uint16_t t4_initVal = 0;
const uint16_t t4_compareVal = 32000;

const uint16_t t5_initVal = 0;
const uint16_t t5_compareVal = 32000;

void initTimer4(void);
void initTimer3(void);
void initTimer5(void);

#endif