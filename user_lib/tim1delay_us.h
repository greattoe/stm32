/*
 * tim1delay_us.h
 *
 * Created on: 2025. 04. 10 by Lee Yongjin
 *
 * STM32 HAL library for delay_us() with TIM1
 */
 
#ifndef TIM1DELAY_US_H
#define TIM1DELAY_US_H

#include "stm32f1xx_hal.h"

extern TIM_HandleTypeDef htim1;

void timer_start(void);
void delay_us(uint16_t us);

#endif /* TIM1DELAY_US_H */
