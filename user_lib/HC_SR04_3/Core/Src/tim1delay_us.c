/*
 * tim1delay_us.c
 *
 * Created on: 2025. 04. 10 by Lee Yongjin
 *
 * STM32 HAL library for delay_us() with TIM1
 */

#include "tim1delay_us.h"

void timer_start(void)
{
	HAL_TIM_Base_Start(&htim1);
}

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);              // initislize counter to start from 0
	while((__HAL_TIM_GET_COUNTER(&htim1))<us);   // wait count until us
}
