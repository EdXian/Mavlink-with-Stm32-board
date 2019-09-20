#ifndef __SYSTEM_H
#define __SYSTEM_H


#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
//#include "estimate.h"
//struct state_3D state3d;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void SystemClock_Config(void);
void system_init(void);
void system_check(void);
void system_running(void);
void Error_Handler(void);
#endif
