#ifndef __EXTI_H
#define __EXTI_H
#include "sys.h"
extern u16 Ramount,Gamount,Bamount;//用于存放读取的RGB值
extern u16 amount;//中断计数
void EXTIX_Init(void);
#endif
