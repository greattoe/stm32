/*
 * dht11.c
 *
 *  Created on: Apr 7, 2025
 *      Author: thumb
 */

#include "dht11.h"

extern TIM_HandleTypeDef htim1;

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_trriger(void)
{
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
	HAL_Delay(20);

	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	delay_us(7);
	return;
}

void DHT11_Init(void)
{
	HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
	HAL_Delay(3000);
	return;
}

    uint8_t DHT11_rx_Data(void)
    {
    	uint8_t rx_data = 0;

    	for(int i = 0; i < 8; i++)
    	{
    		//when Input Data == 0
    		while( 0 == HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) );
    #if 1
    		delay_us(40);
    #else  // org
    		delay_us(16);
    #endif
    		rx_data<<=1;

    		//when Input Data == 1
    		if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
    		{
    			rx_data |= 1;
    		}
    		while( 1 == HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) );
    	}
    	return rx_data;
    }


    void DHT11_dumi_read(void)
    {
    	while( 1 == HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) );
    	while( 0 == HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) );
    	while( 1 == HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) );
    	return;
    }
