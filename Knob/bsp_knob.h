#ifndef __BSP_KNOB_H
#define __BSP_KNOB_H

#include "stm32f10x.h"

/* 供外部调用的函数声明 */
void bsp_InitKnob(void);
void bsp_Knob_Deal(void);
void bsp_KnobScan(void);

#endif

