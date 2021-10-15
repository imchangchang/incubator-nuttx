/****************************************************************************
 * arch/arm/src/gd32f4xx/gd32_serial.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <nuttx/power/pm.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "gd32f4xx.h"


#define CONFIG_UART0_RXBUFSIZE 512
#define CONFIG_UART0_TXBUFSIZE 512

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

static struct uart_dev_s dev =
{
	.isconsole = true;
	.recv =
	{
		.size = CONFIG_UART0_RXBUFSIZE,
		.buffer = g_uart0rxbuffer,
	},
	.xmit =
	{
		.size = CONFIG_USART1_TXBUFSIZE,
		.buffer = g_usart1txbuffer,
	},
    .ops =
    {
    		.setup 	= up_setup,
    		.shutdown = up_shutdown,
			.attach = up_attach,
			.detach = up_detach,
			.ioctl = up_ioctl,
			.receive = up_receive,
			.rxint = up_rxint,
			.rxavaliable = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
#error "currently not support iflowcontrol"
	        .rxflowcontrol = up_rxflowcontrol,
#endif
			.send = up_send,
			.txint = up_txint,
			.txready = up_txready,
			.tx_empty = up_txready,
    },
};

static int  up_setup(struct uart_dev_s *dev)
{
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART0);

    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_6);
    gpio_af_set(GPIOB, GPIO_AF_7, GPIO_PIN_7);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_7);


    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

	return OK;
}
static void up_shutdown(struct uart_dev_s *dev)
{
}
static int  up_attach(struct uart_dev_s *dev)
{

}
static void up_detach(struct uart_dev_s *dev)
{

}
static int  up_interrupt(int irq, void *context, void *arg)
{

}
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	return OK;
}
static int  up_receive(struct uart_dev_s *dev, unsigned int *status){

}
static void up_rxint(struct uart_dev_s *dev, bool enable){

}
static bool up_rxavailable(struct uart_dev_s *dev)
{

}
static void up_send(struct uart_dev_s *dev, int ch)
{

}
static void up_txint(struct uart_dev_s *dev, bool enable)
{

}
static bool up_txready(struct uart_dev_s *dev)
{
	return true;
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/
void arm_serialinit(void)
{

}

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/
void arm_lowputc(char ch)
{
    usart_data_transmit(USART0, ch);
    while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET) {}
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  arm_lowputc(ch);
  return ch;
}
