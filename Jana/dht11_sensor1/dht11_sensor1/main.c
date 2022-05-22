/*
 * dht11_sensor1.c
 *
 * Created: 12/14/2021 5:21:56 AM
 * Author : janit
 */ 

/*
 * humi_sensor1.c
 *
 * Created: 12/3/2021 10:58:35 AM
 * Author : janit
 */ 
#ifndef F_CPU
#define F_CPU 1000000UL // 1 MHz clock speed
#endif

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "LCD16x2_4bit.h"
#define DHT11_PIN 6
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;
#include "LCD16x2_4bit.h"
void lcdcommand(unsigned char cmnd)
{
	LCD_DPRT = (LCD_DPRT & 0x0f)|(cmnd & 0xf0);		/* SEND COMMAND TO DATA PORT */
	LCD_DPRT &= ~ (1<<LCD_RS);						/* RS = 0 FOR COMMAND */
	LCD_DPRT |= (1<<LCD_EN);						/* EN = 1 FOR H TO L PULSE */
	_delay_us(1);									/* WAIT FOR MAKE ENABLE WIDE */
	LCD_DPRT &= ~(1<<LCD_EN);						/* EN = 0 FOR H TO L PULSE */
	_delay_us(100);									/* WAIT FOR MAKE ENABLE WIDE */
	
	LCD_DPRT = (LCD_DPRT & 0x0f)|(cmnd << 4);		/* SEND COMMAND TO DATA PORT */
	LCD_DPRT |= (1<<LCD_EN);						/* EN = 1 FOR H TO L PULSE */
	_delay_us(1);									/* WAIT FOR MAKE ENABLE WIDE */
	LCD_DPRT &= ~(1<<LCD_EN);						/* EN = 0 FOR H TO L PULSE */
	_delay_ms(2);									/* WAIT FOR MAKE ENABLE WIDE */
}

void lcddata(unsigned char data)
{
	LCD_DPRT = (LCD_DPRT & 0x0f)|(data & 0xf0);		/* SEND DATA TO DATA PORT */
	LCD_DPRT |= (1<<LCD_RS);						/* MAKE RS = 1 FOR DATA */
	LCD_DPRT |= (1<<LCD_EN);						/* EN=0 FOR H TO L PULSE */
	_delay_us(1);									/* WAIT FOR MAKE ENABLE WIDE */


    
    
	LCD_DPRT &= ~(1<<LCD_EN);						/* EN = 0 FOR H TO L PULSE */
	_delay_us(100);									/* WAIT FOR MAKE ENABLE WIDE */
	
	LCD_DPRT = (LCD_DPRT & 0x0f)|(data << 4);		/*  */
	LCD_DPRT |= (1<<LCD_EN);						/* EN=0 FOR H TO L PULSE*/
	_delay_us(1);									/* WAIT FOR MAKE ENABLE WIDE*/
	LCD_DPRT &= ~(1<<LCD_EN);						/* EN = 0 FOR H TO L PULSE*/
	_delay_ms(2);									/* WAIT FOR MAKE ENABLE WIDE*/
}

void lcdinit()
{
	LCD_DDDR = 0xFF;
	_delay_ms(200);									/* WAIT FOR SOME TIME */
	lcdcommand(0x33);
	lcdcommand(0x32);								/* SEND $32 FOR INIT OT 0X02 */
	lcdcommand(0x28);								/* INIT. LCD 2 LINE, 5 X 7 MATRIX */
	lcdcommand(0x0C);								/* DISPLAY ON CURSOR ON */
	lcdcommand(0x01);								/* LCD CLEAR */
	_delay_ms(2);
	lcdcommand(0x82);								/* SHIFT CURSOR TO WRITE */
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
	unsigned char firstcharadd[]={0x80, 0xC0};
	lcdcommand(firstcharadd[y] + x);
}

void lcd_print(char *str)
{
	unsigned char i=0;
	while (str[i] |= 0)
	{
		lcddata(str[i]);
		i++;
	}
}

void lcd_clear()
{
	lcdcommand(0x01);
	_delay_ms(2);
}
void Request()						/* Microcontroller send start pulse or request */
{
	DDRD |= (1<<DHT11_PIN);
	PORTD &= ~(1<<DHT11_PIN);		/* set to low pin */
	_delay_ms(20);					/* wait for 20ms */
	PORTD |= (1<<DHT11_PIN);		/* set to high pin */
}

void Response()						/* receive response from DHT11 */
{
	DDRD &= ~(1<<DHT11_PIN);
	while(PIND & (1<<DHT11_PIN));
	while((PIND & (1<<DHT11_PIN))==0);
	while(PIND & (1<<DHT11_PIN));
}

uint8_t Receive_data()							/* receive data */
{
	for (int q=0; q<8; q++)
	{
		while((PIND & (1<<DHT11_PIN)) == 0);	/* check received bit 0 or 1 */
		_delay_us(30);
		if(PIND & (1<<DHT11_PIN))				/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);						/* then its logic HIGH */
		else									/* otherwise its logic LOW */
		c = (c<<1);
		while(PIND & (1<<DHT11_PIN));
	}
	return c;
}

int main(void)
{
	char data[5];
	DDRC |= 1 << PINC0;
	lcdinit();					/* initialize LCD */
	lcd_clear();				/* clear LCD */
	lcd_gotoxy(0,0);			/* enter column and row position */
	lcd_print("H=");
	lcd_gotoxy(8,0);
	lcd_print("T= ");
	
	while(1)
	{
		Request();				/* send start pulse */
		Response();				/* receive response */
		I_RH=Receive_data();	/* store first eight bit in I_RH */
		D_RH=Receive_data();	/* store next eight bit in D_RH */
		I_Temp=Receive_data();	/* store next eight bit in I_Temp */
		D_Temp=Receive_data();	/* store next eight bit in D_Temp */
		CheckSum=Receive_data();/* store next eight bit in CheckSum */
		
		if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
		{
			lcd_gotoxy(0,0);
			lcd_print("Error");
		}
		
		else
		{
			itoa(I_RH,data,10);
			lcd_gotoxy(2,0);
			lcd_print(data);
			lcd_print(".");
			
			itoa(D_RH,data,10);
			lcd_print(data);
			lcd_print("%");

			itoa(I_Temp,data,10);
			lcd_gotoxy(10,0);
			lcd_print(data);
			lcd_print(".");
			
			itoa(D_Temp,data,10);
			lcd_print(data);
			lcddata(0xDF);
			lcd_print("C ");
			
			/*itoa(CheckSum,data,10);
			lcd_print(data);
			lcd_print(" ");*/
		}
		
		_delay_ms(500);
		
		
		
		if ((I_RH + D_RH ) <=75)
		{
			
			lcd_gotoxy(0,1);
			lcd_print("FOGGER ON ");
			PORTC |= 1 << PINC0;
			
		}
		
		else
		{
			lcd_gotoxy(0,1);
			lcd_print("FOGGER OFF");
			PORTC &= ~(1 << PINC0);

			
		}
		_delay_ms(100);
		
	}
}




