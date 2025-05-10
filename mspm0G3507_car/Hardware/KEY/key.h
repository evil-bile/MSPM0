#ifndef __KEY_H
#define __KEY_H
#include "main.h"
#include "delay.h"


#ifndef KEY_PORT
#define KEY_PORT YOUR name 
#endif

#ifndef KEY1_PIN
#define KEY1_PIN KEY_PIN_0_PIN
#endif

#ifndef KEY2_PIN
#define KEY2_PIN KEY_PIN_1_PIN 
#endif


uint8_t Key1_Monitor(void);

uint8_t Key2_Monitor(void);

#endif
