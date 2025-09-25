/*
 * dht11.h
 *
 *  Created on: Apr 7, 2025
 *      Author: thumb
 */

#ifndef __DHT11_H__
#define __DHT11_H__

#include "stm32f1xx_hal.h"

#define DHT11_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1


void delay_us(uint16_t us);
void DHT11_trriger(void);
uint8_t DHT11_rx_Data(void);
void DHT11_dumi_read(void);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* __DHT11_H_ */
