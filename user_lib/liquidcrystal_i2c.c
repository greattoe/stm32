/*
 * liquidcrystal_i2c.c
 *
 *  Created on: 2025. 03. 12.
 * STM32 HAL library for LCD display based on 16x2 CLCD with PCF8574
 *
 */
#include "liquidcrystal_i2c.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

int delay = 0;
int value = 0;

void us_delay(int us){
value = 3;
delay = us * value;
for(int i=0;i < delay;i++);
}

void LCD_DATA(int data) {
  uint8_t temp=(data & 0xF0)|RS1_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=(data & 0xF0)|RS1_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  us_delay(4);

  temp=((data << 4) & 0xF0)|RS1_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp = ((data << 4) & 0xF0)|RS1_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  us_delay(50);
}

void LCD_CMD(int cmd) {
  uint8_t temp=(cmd & 0xF0)|RS0_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=(cmd & 0xF0)|RS0_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  us_delay(4);

  temp=((cmd << 4) & 0xF0)|RS0_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=((cmd << 4) & 0xF0)|RS0_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  us_delay(50);
}

void LCD_CMD_4bit(int cmd) {
  uint8_t temp=((cmd << 4) & 0xF0)|RS0_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=((cmd << 4) & 0xF0)|RS0_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  us_delay(50);
}

void LCD_INIT(void) {
  HAL_Delay(100);

  LCD_CMD_4bit(0x03); HAL_Delay(5);
  LCD_CMD_4bit(0x03); us_delay(100);
  LCD_CMD_4bit(0x03); us_delay(100);
  LCD_CMD_4bit(0x02); us_delay(100);
  LCD_CMD(0x28);  // 4 bits, 2 line, 5x8 font
  LCD_CMD(0x08);  // display off, cursor off, blink off
  LCD_CMD(0x01);  // clear display
  HAL_Delay(3);
  LCD_CMD(0x06);  // cursor move int direction
  LCD_CMD(0x0C);  // display on, cursor off, blink off
}

void LCD_XY(char x, char y) {
  if      (y == 0) LCD_CMD(0x80 + x);
  else if (y == 1) LCD_CMD(0xC0 + x);
  else if (y == 2) LCD_CMD(0x94 + x);
  else if (y == 3) LCD_CMD(0xD4 + x);
}

void LCD_CLEAR(void) {
  LCD_CMD(0x01);
  HAL_Delay(2);
}

void LCD_PUTS(char *str) {
  while (*str) LCD_DATA(*str++);
}

void LCD_PUT_INT(int num)
{
	int E3, E2, E1, E0;

	if(num >= 1000 && num <= 9999)  // if num is 4 digit No.
	{
		E3 = num / 1000;
		LCD_DATA(E3+'0');
		E2 = num % 1000 / 100;
		LCD_DATA(E2+'0');
		E1 = num % 100 / 10;
		LCD_DATA(E1+'0');
		E0 = num % 100 % 10;
		LCD_DATA(E0+'0');
	}

	else if(num >= 100 && num <=999)  // if num is 3 digit No.
	{
		E2 = num / 100;
		LCD_DATA(E2+'0');
		E1 = num % 100 / 10;
		LCD_DATA(E1+'0');
		E0 = num % 100 % 10;
		LCD_DATA(E0+'0');
	}
	else if(num >= 10 && num <= 99 )  // if num is 2 digit No.
	{
		E1 = num / 10;
		LCD_DATA(E1+'0');
		E0 = num % 100 % 10;
		LCD_DATA(E0+'0');
	}
	else if(num < 10) {  // if num is 1 digit No.
		E0 = num;
		LCD_DATA(E0+'0');
	}
	else;

}
