#ifndef LCD_H_
#define LCD_H_

#include <avr/io.h>

#define LCD_RS_DDR        DDRD
#define LCD_RS_PORT       PORTD
#define LCD_RS            6

#define LCD_RW_DDR        DDRD
#define LCD_RW_PORT       PORTD
#define LCD_RW            5

#define LCD_EN_DDR        DDRD
#define LCD_EN_PORT       PORTD
#define LCD_EN            7

#define LCD_DDR           DDRC
#define LCD_PORT          PORTC
#define LCD_D4    	      4
#define LCD_D5    	      5
#define LCD_D6   	      6
#define LCD_D7    	      7

void LCD_pulseEnable();
void LCD_write4Bits(uint8_t nb);
void LCD_command(uint8_t cmd);
void LCD_data(uint8_t dt);
void LCD_char(char chr);
void LCD_string(char * str);
void LCD_setPos(uint8_t x, uint8_t y);
void LCD_clear();
void LCD_init();

#endif
