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

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "gd32f4xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define GD32_NUSART (8)

 /****************************************************************************
 * Private Types
 ****************************************************************************/

 struct up_dev_s
 {
	 struct uart_dev_s dev;
	 /* Has been initialized and HW is setup*/
	 bool initialized;

	 uint32_t baud;

	 const uint8_t irq;   /* IRQ associated with this USART*/
	 const uint32_t uart_periph;
	 const uint32_t gpio_rx_periph;
	 const uint32_t gpio_rx_pin;
	 const uint32_t gpio_rx_af;
	 const uint32_t gpio_tx_periph;
	 const uint32_t gpio_tx_pin;
	 const uint32_t gpio_tx_af;

 };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops = 
{
	.setup 	= up_setup,
	.shutdown = up_shutdown,
	.attach = up_attach,
	.detach = up_detach,
	.ioctl = up_ioctl,
	.receive = up_receive,
	.rxint = up_rxint,
	.rxavailable = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
#error "currently not support iflowcontrol"
	.rxflowcontrol = up_rxflowcontrol,
#endif
	.send = up_send,
	.txint = up_txint,
	.txready = up_txready,
	.txempty = up_txready,
};

#ifdef CONFIG_UART0_SERIALDRIVER
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
static struct up_dev_s g_uart0priv = 
{
	.dev = 
	{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv = 
		{
			.size = CONFIG_UART0_RXBUFSIZE,
			.buffer = g_uart0rxbuffer,
		},
		.xmit = 
		{
			.size = CONFIG_UART0_TXBUFSIZE,
			.buffer = g_uart0txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart0priv,
	}, 
	.baud = CONFIG_UART0_BAUD,
	.irq =  GD32_IRQ_NUM(USART0_IRQn),
	.uart_periph = USART0,
	.gpio_rx_periph = GPIO_UART0_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART0_RX_PIN,
	.gpio_rx_af  	= GPIO_UART0_RX_AF,
	.gpio_tx_periph = GPIO_UART0_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART0_TX_PIN,
	.gpio_tx_af  	= GPIO_UART0_TX_AF,
};
#endif

#ifdef CONFIG_UART1_SERIALDRIVER
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
static struct up_dev_s g_uart1priv =
{
	.dev =
	{
#ifdef CONFIG_UART1_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv =
		{
			.size = CONFIG_UART1_RXBUFSIZE,
			.buffer = g_uart1rxbuffer,
		},
		.xmit =
		{
			.size = CONFIG_UART1_TXBUFSIZE,
			.buffer = g_uart1txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart1priv,
	},
	.baud = CONFIG_UART1_BAUD,
	.irq =  GD32_IRQ_NUM(USART1_IRQn),
	.uart_periph = USART1,
	.gpio_rx_periph = GPIO_UART1_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART1_RX_PIN,
	.gpio_rx_af  	= GPIO_UART1_RX_AF,
	.gpio_tx_periph = GPIO_UART1_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART1_TX_PIN,
	.gpio_tx_af  	= GPIO_UART1_TX_AF,
};
#endif

#ifdef CONFIG_UART2_SERIALDRIVER
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
static struct up_dev_s g_uart2priv =
{
	.dev =
	{
#ifdef CONFIG_UART2_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv =
		{
			.size = CONFIG_UART2_RXBUFSIZE,
			.buffer = g_uart2rxbuffer,
		},
		.xmit =
		{
			.size = CONFIG_UART2_TXBUFSIZE,
			.buffer = g_uart2txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart2priv,
	},
	.baud = CONFIG_UART2_BAUD,
	.irq =  GD32_IRQ_NUM(USART2_IRQn),
	.uart_periph = USART2,
	.gpio_rx_periph = GPIO_UART2_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART2_RX_PIN,
	.gpio_rx_af  	= GPIO_UART2_RX_AF,
	.gpio_tx_periph = GPIO_UART2_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART2_TX_PIN,
	.gpio_tx_af  	= GPIO_UART2_TX_AF,
};
#endif

#ifdef CONFIG_UART3_SERIALDRIVER
static char g_uart3rxbuffer[CONFIG_UART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_UART3_TXBUFSIZE];
static struct up_dev_s g_uart3priv =
{
	.dev =
	{
#ifdef CONFIG_UART3_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv =
		{
			.size = CONFIG_UART3_RXBUFSIZE,
			.buffer = g_uart3rxbuffer,
		},
		.xmit =
		{
			.size = CONFIG_UART3_TXBUFSIZE,
			.buffer = g_uart3txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart3priv,
	},
	.baud = CONFIG_UART3_BAUD,
	.irq =  GD32_IRQ_NUM(UART3_IRQn),
	.uart_periph = UART3,
	.gpio_rx_periph = GPIO_UART3_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART3_RX_PIN,
	.gpio_rx_af  	= GPIO_UART3_RX_AF,
	.gpio_tx_periph = GPIO_UART3_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART3_TX_PIN,
	.gpio_tx_af  	= GPIO_UART3_TX_AF,
};
#endif

#ifdef CONFIG_UART4_SERIALDRIVER
static char g_uart4rxbuffer[CONFIG_UART4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_UART4_TXBUFSIZE];
static struct up_dev_s g_uart4priv =
{
	.dev =
	{
#ifdef CONFIG_UART4_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv =
		{
			.size = CONFIG_UART4_RXBUFSIZE,
			.buffer = g_uart4rxbuffer,
		},
		.xmit =
		{
			.size = CONFIG_UART4_TXBUFSIZE,
			.buffer = g_uart4txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart4priv,
	},
	.baud = CONFIG_UART4_BAUD,
	.irq =  GD32_IRQ_NUM(UART4_IRQn),
	.uart_periph = UART4,
	.gpio_rx_periph = GPIO_UART4_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART4_RX_PIN,
	.gpio_rx_af  	= GPIO_UART4_RX_AF,
	.gpio_tx_periph = GPIO_UART4_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART4_TX_PIN,
	.gpio_tx_af  	= GPIO_UART4_TX_AF,
};
#endif

#ifdef CONFIG_UART5_SERIALDRIVER
static char g_uart5rxbuffer[CONFIG_UART5_RXBUFSIZE];
static char g_uart5txbuffer[CONFIG_UART5_TXBUFSIZE];
static struct up_dev_s g_uart5priv =
{
	.dev =
	{
#ifdef CONFIG_UART5_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv =
		{
			.size = CONFIG_UART5_RXBUFSIZE,
			.buffer = g_uart5rxbuffer,
		},
		.xmit =
		{
			.size = CONFIG_UART5_TXBUFSIZE,
			.buffer = g_uart5txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart5priv,
	},
	.baud = CONFIG_UART5_BAUD,
	.irq =  GD32_IRQ_NUM(USART5_IRQn),
	.uart_periph = USART5,
	.gpio_rx_periph = GPIO_UART5_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART5_RX_PIN,
	.gpio_rx_af  	= GPIO_UART5_RX_AF,
	.gpio_tx_periph = GPIO_UART5_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART5_TX_PIN,
	.gpio_tx_af  	= GPIO_UART5_TX_AF,
};
#endif

#ifdef CONFIG_UART6_SERIALDRIVER
static char g_uart6rxbuffer[CONFIG_UART6_RXBUFSIZE];
static char g_uart6txbuffer[CONFIG_UART6_TXBUFSIZE];
static struct up_dev_s g_uart6priv =
{
	.dev =
	{
#ifdef CONFIG_UART6_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv =
		{
			.size = CONFIG_UART6_RXBUFSIZE,
			.buffer = g_uart6rxbuffer,
		},
		.xmit =
		{
			.size = CONFIG_UART6_TXBUFSIZE,
			.buffer = g_uart6txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart6priv,
	},
	.baud = CONFIG_UART6_BAUD,
	.irq =  GD32_IRQ_NUM(UART6_IRQn),
	.uart_periph = UART6,
	.gpio_rx_periph = GPIO_UART6_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART6_RX_PIN,
	.gpio_rx_af  	= GPIO_UART6_RX_AF,
	.gpio_tx_periph = GPIO_UART6_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART6_TX_PIN,
	.gpio_tx_af  	= GPIO_UART6_TX_AF,
};
#endif

#ifdef CONFIG_UART7_SERIALDRIVER
static char g_uart7rxbuffer[CONFIG_UART7_RXBUFSIZE];
static char g_uart7txbuffer[CONFIG_UART7_TXBUFSIZE];
static struct up_dev_s g_uart7priv =
{
	.dev =
	{
#ifdef CONFIG_UART7_SERIAL_CONSOLE
		.isconsole = true,
#endif
		.recv =
		{
			.size = CONFIG_UART7_RXBUFSIZE,
			.buffer = g_uart7rxbuffer,
		},
		.xmit =
		{
			.size = CONFIG_UART7_TXBUFSIZE,
			.buffer = g_uart7txbuffer,
		},
		.ops = &g_uart_ops,
		.priv = &g_uart7priv,
	},
	.baud = CONFIG_UART7_BAUD,
	.irq =  GD32_IRQ_NUM(UART7_IRQn),
	.uart_periph = UART7,
	.gpio_rx_periph = GPIO_UART7_RX_PORT,
	.gpio_rx_pin 	= GPIO_UART7_RX_PIN,
	.gpio_rx_af  	= GPIO_UART7_RX_AF,
	.gpio_tx_periph = GPIO_UART7_TX_PORT,
	.gpio_tx_pin 	= GPIO_UART7_TX_PIN,
	.gpio_tx_af  	= GPIO_UART7_TX_AF,
};
#endif

/* The g_low_uart refs to the console uart to let low putc work */
static struct up_dev_s *g_low_uart = NULL;

/* This table lets us iterate over the configured USARTs */
static struct up_dev_s * const g_uart_devs[GD32_NUSART] =
{
#ifdef CONFIG_UART0_SERIALDRIVER
  [0] = &g_uart0priv,
#endif
#ifdef CONFIG_UART1_SERIALDRIVER
  [1] = &g_uart1priv,
#endif
#ifdef CONFIG_UART2_SERIALDRIVER
  [2] = &g_uart2priv,
#endif
#ifdef CONFIG_UART3_SERIALDRIVER
  [3] = &g_uart3priv,
#endif
#ifdef CONFIG_UART4_SERIALDRIVER
  [4] = &g_uart4priv,
#endif
#ifdef CONFIG_UART5_SERIALDRIVER
  [5] = &g_usart5priv,
#endif
#ifdef CONFIG_UART6_SERIALDRIVER
  [6] = &g_uart6priv,
#endif
#ifdef CONFIG_UART7_SERIALDRIVER
  [7] = &g_uart7priv,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static uint32_t _get_uart_rcu_periph(uint32_t uart_periph)
{
	switch (uart_periph)
	{
		case USART0:
			return RCU_USART0;
		case USART1:
			return RCU_USART1;
		case USART2:
			return RCU_USART2;
		case UART3:
			return RCU_UART3;
		case UART4:
			return RCU_UART4;
		case USART5:
			return RCU_USART5;
		case UART6:
			return RCU_UART6;
		case UART7:
			return RCU_UART7;
		default:
			PANIC();
	}
}
static uint32_t _get_gpio_rcu_periph(uint32_t gpio_periph)
{
	switch (gpio_periph)
	{
		case GPIOA:
			return RCU_GPIOA;
		case GPIOB:
			return RCU_GPIOB;
		case GPIOC:
			return RCU_GPIOC;
		case GPIOD:
			return RCU_GPIOD;
		case GPIOE:
			return RCU_GPIOE;
		case GPIOF:
			return RCU_GPIOF;
		case GPIOG:
			return RCU_GPIOG;
		case GPIOH:
			return RCU_GPIOH;
		case GPIOI:
			return RCU_GPIOI;
		default:
			PANIC();
	}
}
static void up_set_clock(struct uart_dev_s *dev, bool on)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	uint32_t rcu_uart = _get_uart_rcu_periph(priv->uart_periph);
	uint32_t rcu_rx_gpio = _get_gpio_rcu_periph(priv->gpio_rx_periph);
	uint32_t rcu_tx_gpio = _get_gpio_rcu_periph(priv->gpio_tx_periph);

	if (on)
	{
		rcu_periph_clock_enable(rcu_uart);
		rcu_periph_clock_enable(rcu_rx_gpio);
		rcu_periph_clock_enable(rcu_tx_gpio);
	}
	else
	{
		rcu_periph_clock_disable(rcu_uart);
		rcu_periph_clock_disable(rcu_rx_gpio);
		rcu_periph_clock_disable(rcu_tx_gpio);
	}
}

static int  up_setup(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	up_set_clock(dev, true);

    gpio_af_set(priv->gpio_rx_periph, priv->gpio_rx_af, priv->gpio_rx_pin);
    gpio_af_set(priv->gpio_tx_periph, priv->gpio_tx_af, priv->gpio_tx_pin);

    gpio_mode_set(priv->gpio_rx_periph, GPIO_MODE_AF, GPIO_PUPD_PULLUP, priv->gpio_rx_pin);
    gpio_output_options_set(priv->gpio_rx_periph, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, priv->gpio_rx_pin);

    gpio_mode_set(priv->gpio_tx_periph, GPIO_MODE_AF, GPIO_PUPD_PULLUP, priv->gpio_tx_pin);
    gpio_output_options_set(priv->gpio_tx_periph, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, priv->gpio_tx_pin);


    usart_deinit(priv->uart_periph);
    usart_baudrate_set(priv->uart_periph, priv->baud);
    usart_receive_config(priv->uart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(priv->uart_periph, USART_TRANSMIT_ENABLE);
    usart_enable(priv->uart_periph);

	priv->initialized = true;

	return OK;
}
static void up_shutdown(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	gpio_af_set(priv->gpio_rx_periph, GPIO_AF_0, priv->gpio_rx_pin);
	gpio_mode_set(priv->gpio_rx_periph, GPIO_MODE_INPUT, GPIO_PUPD_NONE, priv->gpio_rx_pin);
	gpio_af_set(priv->gpio_tx_periph, GPIO_AF_0, priv->gpio_tx_pin);
	gpio_mode_set(priv->gpio_tx_periph, GPIO_MODE_INPUT, GPIO_PUPD_NONE, priv->gpio_tx_pin);

	usart_deinit(priv->uart_periph);

	up_set_clock(dev, false);

	priv->initialized = false;
}
static int  up_attach(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	int ret;

	ret = irq_attach(priv->irq, up_interrupt, priv);
	if (ret == OK)
	{
		up_enable_irq(priv->irq);
	}
	return ret;
}
static void up_detach(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

	up_disable_irq(priv->irq);
	irq_detach(priv->irq);
}
static int  up_interrupt(int irq, void *context, void *arg)
{
    struct up_dev_s *priv = (struct up_dev_s *)arg;

	DEBUGASSERT(priv != NULL);

	if (usart_interrupt_flag_get(priv->uart_periph, USART_INT_FLAG_RBNE))
	{
		uart_recvchars(&priv->dev);
		usart_interrupt_flag_clear(priv->uart_periph, USART_INT_FLAG_RBNE);
	}
	if (usart_interrupt_flag_get(priv->uart_periph, USART_INT_FLAG_TC))
	{
		uart_xmitchars(&priv->dev);
		usart_interrupt_flag_clear(priv->uart_periph, USART_INT_FLAG_TC);
	}
	
	return 0;
}
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	return OK;
}
static int  up_receive(struct uart_dev_s *dev, unsigned int *status)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return usart_data_receive(priv->uart_periph);
}
static void up_rxint(struct uart_dev_s *dev, bool enable)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	irqstate_t flags;
	flags = enter_critical_section();
	if (enable)
	{
		usart_interrupt_enable(priv->uart_periph, USART_INT_RBNE);
	}
	else
	{
		usart_interrupt_disable(priv->uart_periph, USART_INT_RBNE);
	}
	leave_critical_section(flags);
}

static bool up_rxavailable(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return (usart_flag_get(priv->uart_periph, USART_FLAG_RBNE) == SET);
}
static void up_send(struct uart_dev_s *dev, int ch)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

    usart_data_transmit(priv->uart_periph, ch);
}
static void up_txint(struct uart_dev_s *dev, bool enable)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	irqstate_t flags;
	flags = enter_critical_section();
	if (enable)
	{
		usart_interrupt_enable(priv->uart_periph, USART_INT_TC);
	}
	else
	{
		usart_interrupt_disable(priv->uart_periph, USART_INT_TC);
	}
	leave_critical_section(flags);
}
static bool up_txready(struct uart_dev_s *dev)
{
    struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
	return (usart_flag_get(priv->uart_periph, USART_FLAG_TBE) == SET);
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
	char devname[16];
	unsigned i;
	unsigned minor = 0;
	if (g_low_uart)
	{
		uart_register("/dev/console", &(g_low_uart->dev));
	}
	strcpy(devname, "/dev/ttySx");
	for (i = 0; i < GD32_NUSART; i++)
	{
		if (g_uart_devs[i] == 0)
		{
			continue;
		}
		devname[9] = '0' + minor++;
		uart_register(devname, &g_uart_devs[i]->dev);
	}
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
	if (g_low_uart == NULL)
	{
		return;
	}
    usart_data_transmit(g_low_uart->uart_periph, ch);
    while (usart_flag_get(g_low_uart->uart_periph, USART_FLAG_TBE) == RESET) {}
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
	if (g_low_uart)
	{
		/* Check for LF */

		if (ch == '\n')
		{
		  /* Add CR */

		  arm_lowputc('\r');
		}

		arm_lowputc(ch);
	}
	return ch;
}
/****************************************************************************
 * Name: gd32_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/
void gd32_lowsetup(void)
{
#ifdef CONFIG_UART0_SERIAL_CONSOLE
	g_low_uart = &g_uart0priv;
#endif
#ifdef CONFIG_UART1_SERIAL_CONSOLE
	g_low_uart = &g_uart1priv;
#endif
#ifdef CONFIG_UART2_SERIAL_CONSOLE
	g_low_uart = &g_uart2priv;
#endif
#ifdef CONFIG_UART3_SERIAL_CONSOLE
	g_low_uart = &g_uart3priv;
#endif
#ifdef CONFIG_UART4_SERIAL_CONSOLE
	g_low_uart = &g_uart4priv;
#endif
#ifdef CONFIG_UART5_SERIAL_CONSOLE
	g_low_uart = &g_uart5priv;
#endif
#ifdef CONFIG_UART6_SERIAL_CONSOLE
	g_low_uart = &g_uart6priv;
#endif
#ifdef CONFIG_UART7_SERIAL_CONSOLE
	g_low_uart = &g_uart7priv;
#endif
	if (g_low_uart)
	{
		up_setup(&(g_low_uart->dev));
	}
}
