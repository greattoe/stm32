/*
 * liquidcrystal_i2c.h
 *
 * Created on: 2025. 03. 12 by Lee Yongjin
 *
 * STM32 HAL library for LCD display based on 16x2 CLCD with PCF8574
 */
  
#ifndef LIQUIDCRYSTAL_I2C_H
#define LIQUIDCRYSTAL_I2C_H

#define ADDRESS   0x27<<1

#define RS1_EN1   0x05
#define RS1_EN0   0x01
#define RS0_EN1   0x04
#define RS0_EN0   0x00
#define BackLight 0x08

void delay_us(int us);
void LCD_DATA(int data);
void LCD_CMD(int cmd);
void LCD_CMD_4bit(int cmd);
void LCD_INIT(void);
void LCD_XY(char x, char y);
void LCD_CLEAR(void);
void LCD_PUTS(char *str);

#endif /* LIQUIDCRYSTAL_I2C_H_ */
