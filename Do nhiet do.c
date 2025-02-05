#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.h"
#include "ds18b20.h"
#include <stdio.h>
#include <stdlib.h>


#define BTN_INC_DDR DDRB
#define BTN_INC_PIN PINB
#define BTN_INC (1 << 0) //PB0

#define BTN_DEC_DDR DDRB
#define BTN_DEC_PIN PINB
#define BTN_DEC (1 << 1) // PB1

#define ALARM_LED_DDR DDRD
#define ALARM_LED_PORT PORTD
#define ALARM_LED (1 << 0); //PD0

float temperature; /* Gia tri nhiet do doc */
float temperatureSet = 25; /* Gia tri nhiet do dat */

unsigned long ms = 0; /* Bien dem thoi gian (1ms) */
unsigned long prevMs = 0; /* Bien thoi diem truoc do */

char strBuffer[64]; /* Bo dem du lieu hien thi */

int main(void)
{
	/* Cau hinh vao ra */
	BTN_INC_DDR &= ~BTN_INC; /* PB0 la dau vao */
	BTN_DEC_DDR &= ~BTN_DEC; /* PB1 la dau vao */
	ALARM_LED_DDR |= ALARM_LED; /* PB4 la dau ra */
	ALARM_LED_PORT &= ~ALARM_LED; /* PB4 = 0 de bat den */

	/* Khoi tao lcd */
	LCD_init();

	/* Khoi tao timer 1ms <=> Fupdate = 1000Hz */
	/* Freq / Fupdate = autoreload * prescaler */
	/* 8000000Hz / 1000Hz = (256 - TCNT0) * prescaler */
	/* chon prescaler = 64 => TCNT0 = 131 */
	TCCR0 = (1 << CS00) | (1 << CS01);
	TCNT0 = 131;
	TIMSK = (1 << TOIE0); /* Cho phep ngat TIMER0 */

	/* Cho phep ngat toan cuc */
	sei();

	while (1)
	{
		/*****************************/
		/* Nut DEC *******************/
		/*****************************/

		/* Neu nut giam duoc nhan */
		if (!((BTN_DEC_PIN & BTN_DEC) == BTN_DEC))
		{
			/* Chong doi phim */
			_delay_ms(20);

			/* Neu chac chan duoc nhan */
			if (!((BTN_DEC_PIN & BTN_DEC) == BTN_DEC))
			{
				/* Giam gia tri nhiet do xuong 1 */
				if (temperatureSet > 0)
				{
					temperatureSet=temperatureSet-1;
				}
			}

			/* Cho cho den khi nut nhan duoc nha */
			while (!((BTN_DEC_PIN & BTN_DEC) == BTN_DEC));
		}

		/*****************************/
		/* Nut INC *******************/
		/*****************************/

		/* Neu nut tang duoc nhan */
		if (!((BTN_INC_PIN & BTN_INC) == BTN_INC))
		{
			/* Chong doi phim */
			_delay_ms(20);

			/* Neu chac chan duoc nhan */
			if (!((BTN_INC_PIN & BTN_INC) == BTN_INC))
			{
				/* Giam gia tri nhiet do xuong 1 */
				if (temperatureSet < 99)
				{
					temperatureSet=temperatureSet+1;
				}
			}

			/* Cho cho den khi nut nhan duoc nha */
			while (!((BTN_INC_PIN & BTN_INC) == BTN_INC));
		}

		/*****************************/
		/* Hien thi nhiet do dat *****/
		/*****************************/

		/* Chuyen float sang kieu chuoi */
		dtostrf(temperatureSet, -6, 2, strBuffer);

		/* Hien thi ra LCD */
		LCD_setPos(0, 0);
		LCD_string("WARN = ");
		LCD_setPos(7, 0);
		LCD_string(strBuffer);
		LCD_setPos(13, 0);
		LCD_data(0xDF);
		LCD_char('C');

		/*****************************/
		/* Hien thi nhiet do doc *****/
		/*****************************/

		/* Dinh thoi 500ms doc nhiet do */
		if (ms - prevMs >= 500)
		{
			/* Sao luu thoi diem truoc do */
			prevMs = ms;

			/* Doc gia tri nhiet do */
			temperature = DS18B20_getTemp();

			/* Kiem tra canh bao */
			if (temperature > temperatureSet)
			{
				/* Dao trang thai led */
				ALARM_LED_PORT ^= ALARM_LED;
			}
			else
			{
				/* Tat led */
				ALARM_LED_PORT &= ~ALARM_LED;
			}
		}

		/* Chuyen float sang kieu chuoi */
		dtostrf(temperature, -6, 2, strBuffer);

		/* Hien thi ra LCD */
		LCD_setPos(0, 1);
		LCD_string("WARN = ");
		LCD_setPos(7, 1);
		LCD_string(strBuffer);
		LCD_setPos(13, 1);
		LCD_data(0xDF);
		LCD_char('C');
	}
}

/* Trinh phuc vu ngat timer 0 */
ISR (TIMER0_OVF_vect)
{
	/* Nap lai gia tri */
	TCNT0 = 131;
	
	/* Tang gia tri dem */
	ms++;
}