#include "i2c_lcd.h"
#include "stm32f4xx_hal.h"


extern I2C_HandleTypeDef hi2c3;

uint8_t backlightState = 1;


void lcd_Goto(uint8_t row, uint8_t col) {
#ifdef LCD_2004
	switch (row){
		case 0:
			lcd_Command(0x80 + col);
			break;
		case 1:
			lcd_Command(0x80 + col + 0x40);
			break;
		case 2:
			lcd_Command(0x80 + col + 0x14);
			break;
		case 3:
		default:
                        lcd_Command(0x80 + col + 0x54);
			break;
	}
#endif
}
void lcd_PrintC(char *str) 
{
 	uint8_t i;
        i = *str++;
 	do
        {
                lcd_Data(i);
                i = *str++;
 	} while(i);
}

void lcd_initialisation(void) {
	lcd_Command(0x33);
	lcd_pause_begin;
	lcd_Command(0x32); // установка режима: 4 линии
	lcd_Command(0x28); // 2 строки и 5*8 точек
	lcd_Command(0x08); // выключить отображение
	lcd_Command(0x01); // очистка экрана
	lcd_pause;
	lcd_Command(0x06); // направление сдвига курсора
	lcd_Command(0x0C); // включить отображение
}

void lcd_Send(uint8_t data) 
{
        HAL_I2C_Master_Transmit(&hi2c3,((0x20+LCD_ADDR) << 1),&data,1,100);
}

void lcd_Command(uint8_t com) {
	uint8_t data = 0;

	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x10) >> 4) << DB4);
	data |= (((com & 0x20) >> 5) << DB5);
	data |= (((com & 0x40) >> 6) << DB6);
	data |= (((com & 0x80) >> 7) << DB7);
	lcd_Send(data);

	data |= (1 << EN);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;

	data = 0;

	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
	data |= (((com & 0x02) >> 1) << DB5);
	data |= (((com & 0x04) >> 2) << DB6);
	data |= (((com & 0x08) >> 3) << DB7);
	lcd_Send(data);

	data |= (1 << EN);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;
}

void lcd_Backlight(uint8_t state) {
	backlightState = (state & 0x01) << BL;
	lcd_Send(backlightState);
}

void lcd_Data(uint8_t com) {
	uint8_t data = 0;

	data |= (1 << EN);
	data |= (1 << RS);
	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x10) >> 4) << DB4);
	data |= (((com & 0x20) >> 5) << DB5);
	data |= (((com & 0x40) >> 6) << DB6);
	data |= (((com & 0x80) >> 7) << DB7);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;

	data = 0;

	data |= (1 << EN);
	data |= (1 << RS);
	data |= (backlightState & 0x01) << BL;

	data |= (((com & 0x01) >> 0) << DB4);
	data |= (((com & 0x02) >> 1) << DB5);
	data |= (((com & 0x04) >> 2) << DB6);
	data |= (((com & 0x08) >> 3) << DB7);
	lcd_Send(data);
	lcd_pause;

	data &= ~(1 << EN);
	lcd_Send(data);
	lcd_pause;
}
