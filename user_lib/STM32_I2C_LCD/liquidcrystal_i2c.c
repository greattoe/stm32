/*
 * liquidcrystal_i2c.c
 *
 *  Created on: 2025. 03. 12.
 *      Author: Lee Yongjin
 * STM32 HAL library for LCD display based on 16x2 CLCD with PCF8574
 *
 */

#include "liquidcrystal_i2c.h"

int delay = 0;
int value = 0;

//I2C_HandleTypeDef hi2c1;

void delay_us(int us){
value = 3;
delay = us * value;
for(int i=0;i < delay;i++);

}
void LCD_DATA(int data) {
  int temp=(data & 0xF0)|RS1_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=(data & 0xF0)|RS1_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  delay_us(4);

  temp=((data << 4) & 0xF0)|RS1_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp = ((data << 4) & 0xF0)|RS1_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  delay_us(50);
}
void LCD_CMD(int cmd) {
  int temp=(cmd & 0xF0)|RS0_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=(cmd & 0xF0)|RS0_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  delay_us(4);

  temp=((cmd << 4) & 0xF0)|RS0_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=((cmd << 4) & 0xF0)|RS0_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  delay_us(50);
}
void LCD_CMD_4bit(int cmd) {
  int temp=((cmd << 4) & 0xF0)|RS0_EN1|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  temp=((cmd << 4) & 0xF0)|RS0_EN0|BackLight;
  while(HAL_I2C_Master_Transmit(&hi2c1, ADDRESS, &temp, 1, 1000)!=HAL_OK);
  delay_us(50);
}
void LCD_INIT(void) {
  HAL_Delay(100);

  LCD_CMD_4bit(0x03); HAL_Delay(5);
  LCD_CMD_4bit(0x03); delay_us(100);
  LCD_CMD_4bit(0x03); delay_us(100);
  LCD_CMD_4bit(0x02); delay_us(100);
  LCD_CMD(0x28);  // 4 bits, 2 line, 5x8 font
  LCD_CMD(0x08);  // display off, cursor off, blink off
  LCD_CMD(0x01);  // clear display
  HAL_Delay(3);
  LCD_CMD(0x06);  // cursor movint direction
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
