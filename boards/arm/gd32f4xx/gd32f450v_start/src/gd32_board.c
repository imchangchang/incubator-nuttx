/*
 * gd32_board.c
 *
 *  Created on: Oct 14, 2021
 *      Author: cgeng
 */
#include <nuttx/board.h>
#include <stdio.h>
#include <gd32_i2c.h>

int board_app_initialize(uintptr_t arg){
	return 0;
}

struct i2c_msg_s msg;

void board_late_initialize(void)
{
	struct i2c_master_s * i2c_master = gd32_i2cbus_initialize(0);
	printf("init i2c success %x \r\n", i2c_master);

	uint8_t data[10] = {0,1,2,3,4,5,6,7,8,9};

	msg.frequency = 100000;
	msg.addr = 0x01;
	msg.flags = 0;
	msg.buffer = data;
	msg.length = 10;

	i2c_master->ops->transfer(i2c_master, &msg, 1);
}
