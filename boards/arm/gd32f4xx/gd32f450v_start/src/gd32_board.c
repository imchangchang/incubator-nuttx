/*
 * gd32_board.c
 *
 *  Created on: Oct 14, 2021
 *      Author: cgeng
 */
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>
#include <stdio.h>

#include "gd32_i2c.h"

static FAR struct lcd_dev_s *g_lcddev;

int board_app_initialize(uintptr_t arg) {
	return 0;
}

struct i2c_msg_s msg[2];

void board_late_initialize(void) {
	const int devno = 0;
	FAR struct i2c_master_s *i2c_master = gd32_i2cbus_initialize(0);
	i2c_register(i2c_master, 0);


	uint8_t transfer_mode = 0x40;
	msg[0].frequency = 100000; /* I2C frequency */
	msg[0].addr = 0x03;
	msg[0].flags = I2C_M_NOSTOP; /* Write transaction, beginning with START */
	msg[0].buffer = &transfer_mode; /* Transfer mode send */
	msg[0].length = 1; /* Send the one byte register address */

	/* Followed by the SSD1306 write data (with no RESTART, then STOP) */

	uint8_t data[2] = {0xab, 0xcd};
	msg[1].frequency = 100000; /* I2C frequency */
	msg[1].addr = 0x03; /* 7-bit address */
	msg[1].flags = I2C_M_NOSTART; /* Write transaction with no RESTART */
	msg[1].buffer = data; /* Transfer from this address */
	msg[1].length = 2; /* Send the data, then STOP */
	I2C_TRANSFER(i2c_master, msg, 2);


	g_lcddev = ssd1306_initialize(i2c_master, NULL, devno);
	if (!g_lcddev) {
		return -ENODEV;
	} else {
		g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);
		return OK;
	}
}
int board_lcd_initialize(void)
{
  return OK;
}

FAR struct lcd_dev_s *board_lcd_getdev(int devno)
{
	return g_lcddev;
}
void board_lcd_uninitialize(void)
{
  /* TO-FIX */
}
