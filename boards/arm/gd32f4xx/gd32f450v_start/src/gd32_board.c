/*
 * gd32_board.c
 *
 *  Created on: Oct 14, 2021
 *      Author: cgeng
 */
#include <nuttx/board.h>
#include <stdio.h>
#include "gd32_i2c.h"
#include "gd32_can.h"

void board_early_initialize(void)
{
	struct can_dev_s * can0 = gd32_caninitialize(0);
	can_register("/dev/can0", can0);
}

int board_app_initialize(uintptr_t arg) {

	return 0;
}


void board_late_initialize(void) {
	#if 0
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
	#endif
}