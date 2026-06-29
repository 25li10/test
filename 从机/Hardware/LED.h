#ifndef __LED_H 
#define __LED_H 

#include "stm32f10x.h"

void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);
void LED_Turn(void);
void LED_Blink(uint8_t times);   /* LED闪烁 times 次 */

#endif 

