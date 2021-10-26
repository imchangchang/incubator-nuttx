/****************************************************************************
 * arch/arm/src/gd32f4xx/gd32_i2c.c
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
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_arch.h"

#include "gd32f4xx.h"
#include "gd32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
enum gd32_i2c_error{
	I2C_SUCCESS,
	I2C_ERR_7BADDR_WITH_10BHEADER,
	I2C_ERR_MASTER_STPDET,
	I2C_ERR_ISR,
};
enum gd32_i2c_status{
	STATUS_IDLE,
	STATUS_7BADDRESS_SENT,
	STATUS_10BADDRESS1_SENT,
	STATUS_10BADDRESS2_SENT,
};

/* I2C Transfer Data*/
struct gd32_i2c_transfer_s {
	uint8_t *ptr; 			/* Current message buffer */
	uint32_t frequency; 	/* Current I2C frequency */
	uint16_t addr;          /* Slave addr*/
	int dcnt; 				/* Current message length */
	uint16_t flags; 		/* Current message flags */
	uint32_t status;
	uint32_t error;
	bool resent_start; /* 10bit read mode*/
};

/* I2C Device Private Data */
struct gd32_i2c_priv_s {
	/* Standard I2C operations */
	const struct i2c_ops_s *ops;

	uint32_t i2c_periph;

	uint32_t gpio_scl_periph;
	uint32_t gpio_scl_pin;
	uint32_t gpio_scl_af;
	uint32_t gpio_sda_periph;
	uint32_t gpio_sda_pin;
	uint32_t gpio_sda_af;

	uint32_t ev_irq; /* event interrupt */
	uint32_t er_irq; /* error interrupt */

	int refs; 		/* Reference count */
	sem_t sem_excl; /* Mutual exclusion semaphore */
	sem_t sem_isr; 	/* Interrupt wait semaphore */

	struct gd32_i2c_transfer_s transfer;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int gd32_i2c_sem_waitdone(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_waitstop(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_post(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_init(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_destroy(FAR struct gd32_i2c_priv_s *priv);

static void gd32_i2c_setclock(FAR struct gd32_i2c_priv_s *priv,
		uint32_t frequency);
static inline void gd32_i2c_sendstart(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_clrstart(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sendstop(FAR struct gd32_i2c_priv_s *priv);
static inline uint32_t gd32_i2c_getstatus(FAR struct gd32_i2c_priv_s *priv);
static int gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv);
static int gd32_i2c_isr(int irq, void *context, FAR void *arg);
static int gd32_i2c_init(FAR struct gd32_i2c_priv_s *priv);
static int gd32_i2c_deinit(FAR struct gd32_i2c_priv_s *priv);
static int gd32_i2c_transfer(FAR struct i2c_master_s *dev,
		FAR struct i2c_msg_s *msgs, int count);


/****************************************************************************
 * Private Data
 ****************************************************************************/


/* I2C interface */

static const struct i2c_ops_s gd32_i2c_ops = { .transfer = gd32_i2c_transfer };

/* I2C device structures */

#ifdef CONFIG_GD32_I2C0
static struct gd32_i2c_priv_s gd32_i2c0 =
{
	.ops = &gd32_i2c_ops,
	.i2c_periph = I2C0,
	.gpio_scl_periph = GPIO_I2C0_SCL_PORT,
	.gpio_scl_pin = GPIO_I2C0_SCL_PIN,
	.gpio_scl_af = GPIO_I2C0_SCL_AF,
	.gpio_sda_periph = GPIO_I2C0_SDA_PORT,
	.gpio_sda_pin = GPIO_I2C0_SDA_PIN,
	.gpio_sda_af = GPIO_I2C0_SDA_AF,
	.ev_irq = GD32_IRQ_NUM(I2C0_EV_IRQn),
	.er_irq = GD32_IRQ_NUM(I2C0_ER_IRQn),
	.refs = 0,
	.transfer =
	{
		.ptr = NULL,
		.frequency = CONFIG_GD32_I2C0_FREQUENCY,
	},
};
#endif
#ifdef CONFIG_GD32_I2C1
static struct gd32_i2c_priv_s gd32_i2c1 =
{
	.ops = &gd32_i2c_ops,
};
#endif
#ifdef CONFIG_GD32_I2C2
static struct gd32_i2c_priv_s gd32_i2c2 =
{
	.ops = &gd32_i2c_ops,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: gd32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_init(FAR struct gd32_i2c_priv_s *priv) {
	nxsem_init(&priv->sem_excl, 0, 1);
	nxsem_init(&priv->sem_isr, 0, 0);
	nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
}

/****************************************************************************
 * Name: gd32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_destroy(FAR struct gd32_i2c_priv_s *priv) {
	nxsem_destroy(&priv->sem_excl);
	nxsem_destroy(&priv->sem_isr);
}

/****************************************************************************
 * Name: gd32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void gd32_i2c_setclock(FAR struct gd32_i2c_priv_s *priv,
		uint32_t frequency) {

	/* Has the I2C bus frequency changed? */

	if (frequency != priv->transfer.frequency) {
		priv->transfer.frequency = frequency;
		i2c_clock_config(priv->i2c_periph, priv->transfer.frequency, I2C_DTCY_2);
	}
}

static __inline gd32_i2c_end_isr_procss(struct gd32_i2c_priv_s *priv,
		uint32_t error) {
	priv->transfer.error = error;
	i2c_interrupt_disable(priv->i2c_periph, I2C_INT_ERR);
	i2c_interrupt_disable(priv->i2c_periph, I2C_INT_EV);
	i2c_interrupt_disable(priv->i2c_periph, I2C_INT_BUF);

	/* Only post the sem, when it's locked */
	int savl;
	sem_getvalue(&priv->sem_isr, &savl);
	if (savl <= 0)
	{
		nxsem_post(&priv->sem_isr);
	}
}

/****************************************************************************
 * Name: gd32_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv) {

	/* START bit sent */
	if (i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_SBSEND) == SET)
	{
		uint32_t trandirection =
				 (priv->transfer.flags & I2C_M_READ) ? I2C_RECEIVER:I2C_TRANSMITTER;
		if (priv->transfer.resent_start == false)
		{
			/* 10-bit address */
			if (priv->transfer.flags & I2C_M_TEN)
			{
				i2c_master_addressing(priv->i2c_periph,
									  I2C_ADDR10H(priv->transfer.addr),
									  trandirection);
			}
			else
			{
				i2c_master_addressing(priv->i2c_periph,
									  I2C_ADDR8(priv->transfer.addr),
									  trandirection);
			}
		}
		else
		{
			i2c_master_addressing(priv->i2c_periph,
							      I2C_ADDR10H(priv->transfer.addr),
								  I2C_RECEIVER);
		}
	}
	else if (i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_ADDSEND) == SET)
	{
		// 10bits and
		if ((priv->transfer.flags & I2C_M_TEN & I2C_M_READ) && priv->transfer.resent_start == false)
		{
			priv->transfer.resent_start = true;
			i2c_start_on_bus(priv->i2c_periph);
		}
		else if (priv->transfer.flags & I2C_M_READ && priv->transfer.dcnt == 1)
		{
			i2c_ack_config(priv->i2c_periph, I2C_ACK_DISABLE);
		}
		i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_ADDSEND);
	}
	else if (i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_TBE) == SET)
	{
		if (priv->transfer.dcnt > 0)
		{
			priv->transfer.dcnt--;
			i2c_data_transmit(priv->i2c_periph, *priv->transfer.ptr++);
		}
		else
		{
			if (!(priv->transfer.flags & I2C_M_NOSTOP))
				i2c_stop_on_bus(priv->i2c_periph);
			gd32_i2c_end_isr_procss(priv, I2C_SUCCESS);
		}
	}
	else if (i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_ADD10SEND))
	{
		if (priv->transfer.flags & I2C_M_TEN)
		{
			I2C_DATA(priv->i2c_periph) = I2C_ADDR10L(priv->transfer.addr);
		}
		else
		{
			i2c_stop_on_bus(priv->i2c_periph);
			gd32_i2c_end_isr_procss(priv, I2C_ERR_7BADDR_WITH_10BHEADER);
		}
		i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_ADD10SEND);
	}
	else if (i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_STPDET))
	{
		i2c_stop_on_bus(priv->i2c_periph);
		gd32_i2c_end_isr_procss(priv, I2C_ERR_MASTER_STPDET);
		i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_STPDET);
	}
	else if (i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_RBNE))
	{
		if (priv->transfer.dcnt > 0)
		{
			priv->transfer.dcnt--;
			*priv->transfer.ptr++ = i2c_data_receive(priv->i2c_periph);
			if (priv->transfer.dcnt == 1)
			{
				i2c_ack_config(priv->i2c_periph, I2C_ACK_DISABLE);
			}
			else
			{
				if (!(priv->transfer.flags & I2C_M_NOSTOP))
					i2c_stop_on_bus(priv->i2c_periph);
				gd32_i2c_end_isr_procss(priv, I2C_SUCCESS);
			}
		}
		i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_RBNE);
	}
	else if (i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_BTC))
	{
		if (priv->transfer.dcnt == 0)
		{
			if (!(priv->transfer.flags & I2C_M_NOSTOP))
				i2c_stop_on_bus(priv->i2c_periph);
			gd32_i2c_end_isr_procss(priv, I2C_SUCCESS);
		}
		i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_BTC);
	}

	return OK;
}

/****************************************************************************
 * Name: gd32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

static int gd32_i2c_isr(int irq, void *context, FAR void *arg) {
	struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s*) arg;

	DEBUGASSERT(priv != NULL);
	return gd32_i2c_isr_process(priv);
}

static int gd32_i2c_eisr(int irq, void * context, FAR void * arg)
{
	struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s*) arg;
	DEBUGASSERT(priv != NULL);

    /* no acknowledge received */
    if(i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_AERR)){
        i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_AERR);
    }

    /* SMBus alert */
    if(i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_SMBALT)){
        i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_SMBALT);
    }

    /* bus timeout in SMBus mode */
    if(i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_SMBTO)){
        i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_SMBTO);
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_OUERR)){
        i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_OUERR);
    }

    /* arbitration lost */
    if(i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_LOSTARB)){
        i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_LOSTARB);
    }

    /* bus error */
    if(i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_BERR)){
        i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_BERR);
    }

    /* CRC value doesn't match */
    if(i2c_interrupt_flag_get(priv->i2c_periph, I2C_INT_FLAG_PECERR)){
        i2c_interrupt_flag_clear(priv->i2c_periph, I2C_INT_FLAG_PECERR);
    }
	i2c_stop_on_bus(priv->i2c_periph);
	gd32_i2c_end_isr_procss(priv, I2C_ERR_ISR);
}

static uint32_t _get_gpio_rcu_periph(uint32_t gpio_periph) {
	switch (gpio_periph) {
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

static uint32_t _get_i2c_rcu_periph(uint32_t i2c_periph) {
	switch (i2c_periph) {
	case I2C0:
		return RCU_I2C0;
	case I2C1:
		return RCU_I2C1;
	case I2C2:
		return RCU_I2C2;
	default:
		PANIC();
	}
}

/****************************************************************************
 * Name: gd32_i2c_init
 *
 * Description:
 *   Setup the I2C hardware, ready for operation with defaults
 *
 ****************************************************************************/

static int gd32_i2c_init(FAR struct gd32_i2c_priv_s *priv) {

	/* RCU configuration */
	rcu_periph_clock_enable(_get_i2c_rcu_periph(priv->i2c_periph));
	rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_scl_periph));
	rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_sda_periph));


	/* GPIO configuration */
	gpio_af_set(priv->gpio_scl_periph, priv->gpio_scl_af, priv->gpio_scl_pin);
	gpio_mode_set(priv->gpio_scl_periph, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
			priv->gpio_scl_pin);
	gpio_output_options_set(priv->gpio_scl_periph, GPIO_OTYPE_OD,
			GPIO_OSPEED_50MHZ, priv->gpio_scl_pin);

	gpio_af_set(priv->gpio_sda_periph, priv->gpio_sda_af, priv->gpio_sda_pin);
	gpio_mode_set(priv->gpio_sda_periph, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
			priv->gpio_sda_pin);
	gpio_output_options_set(priv->gpio_sda_periph, GPIO_OTYPE_OD,
			GPIO_OSPEED_50MHZ, priv->gpio_sda_pin);

	/* I2C configuration */
	irq_attach(priv->ev_irq, gd32_i2c_isr, priv);
	irq_attach(priv->er_irq, gd32_i2c_eisr, priv);
	up_enable_irq(priv->ev_irq);
	up_enable_irq(priv->er_irq);

	i2c_clock_config(priv->i2c_periph, priv->transfer.frequency, I2C_DTCY_2);
	/* The address is not used in master mode */
	i2c_mode_addr_config(priv->i2c_periph,
			             I2C_I2CMODE_ENABLE,
			             I2C_ADDFORMAT_7BITS, 0x00);
	i2c_enable(priv->i2c_periph);
	i2c_ack_config(priv->i2c_periph, I2C_ACK_ENABLE);

	return OK;
}

/****************************************************************************
 * Name: gd32_i2c_deinit
 *
 * Description:
 *   Shutdown the I2C hardware
 *
 ****************************************************************************/

static int gd32_i2c_deinit(FAR struct gd32_i2c_priv_s *priv) {
	i2c_deinit(priv->i2c_periph);

	up_disable_irq(priv->ev_irq);
	up_disable_irq(priv->er_irq);
	irq_detach(priv->ev_irq);
	irq_detach(priv->er_irq);

	rcu_periph_clock_disable(_get_i2c_rcu_periph(priv->i2c_periph));
	rcu_periph_clock_disable(_get_gpio_rcu_periph(priv->gpio_scl_periph));
	rcu_periph_clock_disable(_get_gpio_rcu_periph(priv->gpio_sda_periph));

	/* GPIO configuration */
	gpio_af_set(priv->gpio_scl_periph,GPIO_AF_0, priv->gpio_scl_pin);
	gpio_mode_set(priv->gpio_scl_periph, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
			priv->gpio_scl_pin);

	gpio_af_set(priv->gpio_sda_periph, GPIO_AF_0, priv->gpio_sda_pin);
	gpio_mode_set(priv->gpio_sda_periph, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
			priv->gpio_sda_pin);

	return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

static int gd32_i2c_transfer_one_msg(FAR struct i2c_master_s *dev,
		FAR struct i2c_msg_s * msg){
	FAR struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s*)dev;
	int ret;


	priv->transfer.status   	= STATUS_IDLE;
	priv->transfer.ptr 			= msg->buffer;
	priv->transfer.dcnt 		= msg->length;
	priv->transfer.flags 		= msg->flags;
	priv->transfer.addr 		= msg->addr;
	priv->transfer.error 		= I2C_SUCCESS;
	priv->transfer.resent_start = false;

	i2c_ack_config(priv->i2c_periph, I2C_ACK_ENABLE);

	i2c_interrupt_enable(priv->i2c_periph, I2C_INT_ERR);
	i2c_interrupt_enable(priv->i2c_periph, I2C_INT_EV);
	i2c_interrupt_enable(priv->i2c_periph, I2C_INT_BUF);

	if (!(priv->transfer.flags & I2C_M_NOSTART))
		i2c_start_on_bus(priv->i2c_periph);


	ret = nxsem_wait(&priv->sem_isr);

	/* Process error */
	if (priv->transfer.error == I2C_ERR_ISR)
	{
		ret = -ENODEV;
	}

	priv->transfer.ptr = NULL;
	priv->transfer.dcnt = 0;
	priv->transfer.flags = 0;
	priv->transfer.addr = 0;

	return ret;
}

/****************************************************************************
 * Name: gd32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int gd32_i2c_transfer(FAR struct i2c_master_s *dev,
		FAR struct i2c_msg_s *msgs, int count) {

	FAR struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s*)dev;
	int ret;

	DEBUGASSERT(count > 0);

	ret = nxsem_wait(&priv->sem_excl);

	if (ret < 0) {
		return ret;
	}

	if (i2c_flag_get(priv->i2c_periph, I2C_FLAG_I2CBSY) == SET)
	{
		ret = -EBUSY;
	}
	else
	{

		for (int i = 0; i < count ; i++)
		{
			gd32_i2c_setclock(priv, msgs[i].frequency);
			ret = gd32_i2c_transfer_one_msg(dev, &msgs[i]);
			if (ret < 0)
			{
				break;
			}
		}
	}

	nxsem_post(&priv->sem_excl);
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2cbus_initialize
 *
 * Description:
 *   Initialize one I2C bus
 *
 ****************************************************************************/

FAR struct i2c_master_s* gd32_i2cbus_initialize(int port) {
	struct gd32_i2c_priv_s *priv = NULL;
	irqstate_t flags;

	/* Get I2C private structure */

	switch (port) {
#ifdef CONFIG_GD32_I2C0
    case 0:
    	priv = &gd32_i2c0;
      break;
#endif
#ifdef CONFIG_GD32_I2C1
    case 1:
    	priv = &gd32_i2c1;
      break;
#endif
#ifdef CONFIG_GD32_I2C2
    case 2:
    	priv = &gd32_i2c2;
      break;
#endif
	default:
		return NULL;
	}

	/* Initialize private data for the first time, increment reference count,
	 * power-up hardware and configure GPIOs.
	 */

	flags = enter_critical_section();

	if ((volatile int) priv->refs++ == 0) {
		gd32_i2c_sem_init(priv);
		gd32_i2c_init(priv);
	}

	leave_critical_section(flags);
	return (struct i2c_master_s*) priv;
}

/****************************************************************************
 * Name: gd32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int gd32_i2cbus_uninitialize(FAR struct i2c_master_s *dev) {
	FAR struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s*) dev;
	irqstate_t flags;

	DEBUGASSERT(dev);

	/* Decrement reference count and check for underflow */

	if (priv->refs == 0) {
		return ERROR;
	}

	flags = enter_critical_section();

	if (--priv->refs) {
		leave_critical_section(flags);
		return OK;
	}

	leave_critical_section(flags);

	/* Disable power and other HW resource (GPIO's) */

	gd32_i2c_deinit(priv);

	/* Release unused resources */

	gd32_i2c_sem_destroy(priv);
	return OK;
}
