/****************************************************************************
 * arch/arm/src/gd32f4xx/gd32_can.c
 *
 *   Copyright (C) 2011, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/can/can.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "gd32f4xx.h"
#ifdef CONFIG_GD32_CAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Delays *******************************************************************/

/* Time out for INAK bit */

#define INAK_TIMEOUT 65535

/* Bit timing ***************************************************************/

#define CAN_BIT_QUANTA (CONFIG_GD32_CAN_TSEG1 + CONFIG_GD32_CAN_TSEG2 + 1)

#ifndef CONFIG_DEBUG_CAN_INFO
#  undef CONFIG_GD32_CAN_REGDEBUG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_can_s {
	uint8_t port; /* CAN port num */
	uint32_t can_periph; /* CAN periph */
	uint8_t canrx[2]; /* CAN RX FIFO 0/1 IRQ number */
	uint8_t cantx; /* CAN TX IRQ number */
	uint8_t filter; /* Filter number */
	uint32_t baud; /* Configured baud */

  uint32_t gpio_rx_periph;
  uint32_t gpio_rx_af;
  uint32_t gpio_rx_pin;

  uint32_t gpio_tx_periph;
  uint32_t gpio_tx_af;
  uint32_t gpio_tx_pin;
  uint8_t sjw;
  uint8_t bs1;
  uint8_t bs2;
  uint16_t psc;
  uint32_t sample_point; /* 0.1%, 1000 for 100%*/
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* CAN Register access */

#ifdef CONFIG_CAN_EXTID
static int  gd32can_addextfilter(FAR struct gd32_can_s *priv,
                                  FAR struct canioc_extfilter_s *arg);
static int  gd32can_delextfilter(FAR struct gd32_can_s *priv,
                                  int arg);
#endif
static int gd32can_addstdfilter(FAR struct gd32_can_s *priv,
		FAR struct canioc_stdfilter_s *arg);
static int gd32can_delstdfilter(FAR struct gd32_can_s *priv, int arg);

/* CAN driver methods */

static void gd32can_reset(FAR struct can_dev_s *dev);
static int gd32can_setup(FAR struct can_dev_s *dev);
static void gd32can_shutdown(FAR struct can_dev_s *dev);
static void gd32can_rxint(FAR struct can_dev_s *dev, bool enable);
static void gd32can_txint(FAR struct can_dev_s *dev, bool enable);
static int gd32can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg);
static int gd32can_remoterequest(FAR struct can_dev_s *dev, uint16_t id);
static int gd32can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg);
static bool gd32can_txready(FAR struct can_dev_s *dev);
static bool gd32can_txempty(FAR struct can_dev_s *dev);

/* CAN interrupt handling */

static int gd32can_rxinterrupt(FAR struct can_dev_s *dev, int rxmb);
static int gd32can_rx0interrupt(int irq, FAR void *context, FAR void *arg);
static int gd32can_rx1interrupt(int irq, FAR void *context, FAR void *arg);
static int gd32can_txinterrupt(int irq, FAR void *context, FAR void *arg);

/* Initialization */

static int gd32can_enterinitmode(FAR struct gd32_can_s *priv);
static int gd32can_exitinitmode(FAR struct gd32_can_s *priv);
static int gd32can_bittiming(FAR struct gd32_can_s *priv);
static int gd32can_cellinit(FAR struct gd32_can_s *priv);
static int gd32can_filterinit(FAR struct gd32_can_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_canops = { 
    .co_reset = gd32can_reset,
	.co_setup = gd32can_setup,
    .co_shutdown = gd32can_shutdown, 
    .co_rxint = gd32can_rxint, 
    .co_txint = gd32can_txint, 
    .co_ioctl = gd32can_ioctl, 
    .co_remoterequest = gd32can_remoterequest,
	.co_send = gd32can_send,
    .co_txready = gd32can_txready, 
    .co_txempty = gd32can_txempty, 
 };

#ifdef CONFIG_GD32_CAN0
static struct gd32_can_s g_can0priv =
{
  .port             = 0,
  .can_periph       = CAN0,
  .canrx            =
  {
					  GD32_IRQ_NUM(CAN0_RX0_IRQn),
					  GD32_IRQ_NUM(CAN0_RX1_IRQn),
  },
  .cantx            = GD32_IRQ_NUM(CAN0_TX_IRQn),
  .filter           = 0,
  .baud             = CONFIG_GD32_CAN0_BAUD,
  .gpio_rx_periph   = GPIO_CAN0_RX_PORT,
  .gpio_rx_af       = GPIO_CAN0_RX_AF,
  .gpio_rx_pin      = GPIO_CAN0_RX_PIN,
  .gpio_tx_periph   = GPIO_CAN0_TX_PORT,
  .gpio_tx_af       = GPIO_CAN0_TX_AF,
  .gpio_tx_pin      = GPIO_CAN0_TX_PIN,
};

static struct can_dev_s g_can0dev =
{
  .cd_ops           = &g_canops,
  .cd_priv          = &g_can0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static uint32_t _get_gpio_rcu_periph(uint32_t gpio_periph) {
#define GPIO_RCU(gpio) case gpio: return RCU_##gpio
	switch (gpio_periph) {
	  GPIO_RCU(GPIOA);
		GPIO_RCU(GPIOB);
		GPIO_RCU(GPIOC);
		GPIO_RCU(GPIOD);
		GPIO_RCU(GPIOE);
		GPIO_RCU(GPIOF);
		GPIO_RCU(GPIOG);
		GPIO_RCU(GPIOH);
		GPIO_RCU(GPIOI);
		default:
		PANIC();
	}
#undef GPIO_RCU
}

static uint32_t _get_can_rcu_periph(uint32_t can_periph) {
#define CAN_RCU(can) case can: return RCU_##can
	switch (can_periph) {
	CAN_RCU(CAN0);		
  CAN_RCU(CAN1);
		default:
		PANIC();
	}
#undef CAN_RCU
}

static uint32_t _can_gpio_rcu_init(FAR struct gd32_can_s *priv){

  rcu_periph_clock_enable(_get_can_rcu_periph(priv->can_periph));
  rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_rx_periph));
  rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_tx_periph));

  gpio_output_options_set(priv->gpio_rx_periph, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, priv->gpio_rx_pin);
  gpio_mode_set(priv->gpio_rx_periph, GPIO_MODE_AF, GPIO_PUPD_NONE, priv->gpio_rx_pin);
  gpio_af_set(priv->gpio_rx_periph, priv->gpio_rx_af, priv->gpio_rx_pin);
  
  gpio_output_options_set(priv->gpio_tx_periph, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, priv->gpio_tx_pin);
  gpio_mode_set(priv->gpio_tx_periph, GPIO_MODE_AF, GPIO_PUPD_NONE, priv->gpio_tx_pin);
  gpio_af_set(priv->gpio_tx_periph, priv->gpio_tx_af, priv->gpio_tx_pin);

  return OK;
}

/****************************************************************************
 * Name: gd32can_reset
 *
 * Description:
 *   Reset the CAN device.  Called early to initialize the hardware. This
 *   function is called, before gd32can_setup() and on error conditions.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

static void gd32can_reset(FAR struct can_dev_s *dev) {
	FAR struct gd32_can_s *priv = dev->cd_priv;
	irqstate_t flags;

	caninfo ("CAN%d\n", priv->port);

	/* Disable interrupts momentarily to stop any ongoing CAN event processing
	 * and to prevent any concurrent access to the AHB1RSTR register.
	 */

	flags = enter_critical_section();

  caninfo("Before reset:0x%08x\n", CAN_CTL(priv->can_periph));
  CAN_CTL(priv->can_periph) |= CAN_CTL_SWRST;
  caninfo("After reset:0x%08x\n", CAN_CTL(priv->can_periph));

	/* Reset the CAN */
	leave_critical_section(flags);
}

/****************************************************************************
 * Name: gd32can_setup
 *
 * Description:
 *   Configure the CAN. This method is called the first time that the CAN
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching CAN interrupts.
 *   All CAN interrupts are disabled upon return.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_setup(FAR struct can_dev_s *dev) {
	FAR struct gd32_can_s *priv = dev->cd_priv;
	int ret;

	caninfo ("CAN%d RX0 irq: %d RX1 irq: %d TX irq: %d\n", priv->port, priv->canrx[0], priv->canrx[1], priv->cantx);

	/* CAN cell initialization */
  _can_gpio_rcu_init(priv);
	ret = gd32can_cellinit(priv);
	if (ret < 0) {
		canerr ("ERROR: CAN%d cell initialization failed: %d\n", priv->port, ret);
		return ret;
	}

	/* CAN filter initialization */
	ret = gd32can_filterinit(priv);
	if (ret < 0) {
		canerr ("ERROR: CAN%d filter initialization failed: %d\n", priv->port, ret);
		return ret;
	}

	/* Attach the CAN RX FIFO 0/1 interrupts and TX interrupts.
	 * The others are not used.
	 */
	ret = irq_attach(priv->canrx[0], gd32can_rx0interrupt, dev);
	if (ret < 0) {
		canerr ("ERROR: Failed to attach CAN%d RX0 IRQ (%d)", priv->port, priv->canrx[0]);
		return ret;
	}

	ret = irq_attach(priv->canrx[1], gd32can_rx1interrupt, dev);
	if (ret < 0) {
		canerr ("ERROR: Failed to attach CAN%d RX1 IRQ (%d)", priv->port, priv->canrx[1]);
		return ret;
	}

	ret = irq_attach(priv->cantx, gd32can_txinterrupt, dev);
	if (ret < 0) {
		canerr ("ERROR: Failed to attach CAN%d TX IRQ (%d)", priv->port, priv->cantx);
		return ret;
	}

	/* Enable the interrupts at the NVIC.  Interrupts are still disabled in
	 * the CAN module.  Since we coming out of reset here, there should be
	 * no pending interrupts.
	 */

	up_enable_irq(priv->canrx[0]);
	up_enable_irq(priv->canrx[1]);
	up_enable_irq(priv->cantx);

	return OK;
}

/****************************************************************************
 * Name: gd32can_shutdown
 *
 * Description:
 *   Disable the CAN.  This method is called when the CAN device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32can_shutdown(FAR struct can_dev_s *dev) {
	FAR struct gd32_can_s *priv = dev->cd_priv;

	caninfo ("CAN%d\n", priv->port);

	/* Disable the RX FIFO 0/1 and TX interrupts */

	up_disable_irq(priv->canrx[0]);
	up_disable_irq(priv->canrx[1]);
	up_disable_irq(priv->cantx);

	/* Detach the RX FIFO 0/1 and TX interrupts */

	irq_detach(priv->canrx[0]);
	irq_detach(priv->canrx[1]);
	irq_detach(priv->cantx);

	/* And reset the hardware */

	gd32can_reset(dev);
}

/****************************************************************************
 * Name: gd32can_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32can_rxint(FAR struct can_dev_s *dev, bool enable) {
	FAR struct gd32_can_s *priv = dev->cd_priv;

	caninfo ("CAN%d enable: %d\n", priv->port, enable);

	/* Enable/disable the FIFO 0/1 message pending interrupt */
  if (enable)
  {
    can_interrupt_enable(priv->can_periph, CAN_INT_RFNE0);
    can_interrupt_enable(priv->can_periph, CAN_INT_RFNE1);
  }
  else{
    can_interrupt_disable(priv->can_periph, CAN_INT_RFNE0);
    can_interrupt_disable(priv->can_periph, CAN_INT_RFNE1);
  }

}

/****************************************************************************
 * Name: gd32can_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gd32can_txint(FAR struct can_dev_s *dev, bool enable) {
	FAR struct gd32_can_s *priv = dev->cd_priv;

	caninfo ("CAN%d enable: %d\n", priv->port, enable);

	/* Support only disabling the transmit mailbox interrupt */
  if (enable)
  {
    can_interrupt_enable(priv->can_periph, CAN_INT_TME);
  }
  else{
    can_interrupt_disable(priv->can_periph, CAN_INT_TME);
  }
}

/****************************************************************************
 * Name: gd32can_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_ioctl(FAR struct can_dev_s *dev, int cmd, unsigned long arg) {
	FAR struct gd32_can_s *priv;
	int ret = -ENOTTY;

	caninfo ("cmd=%04x arg=%lu\n", cmd, arg);

	DEBUGASSERT(dev && dev->cd_priv);
	priv = dev->cd_priv;

	/* Handle the command */

	switch (cmd) {
	/* CANIOC_GET_BITTIMING:
	 *   Description:    Return the current bit timing settings
	 *   Argument:       A pointer to a write-able instance of struct
	 *                   canioc_bittiming_s in which current bit timing
	 *                   values will be returned.
	 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
	 *                   (ERROR) is returned with the errno variable set
	 *                   to indicate the nature of the error.
	 *   Dependencies:   None
	 */

	case CANIOC_GET_BITTIMING: {
		FAR struct canioc_bittiming_s *bt = (FAR struct canioc_bittiming_s*) arg;
		DEBUGASSERT(bt != NULL);
		bt->bt_sjw    = priv->sjw;
		bt->bt_tseg1  = priv->bs1;
		bt->bt_tseg2  = priv->bs2;
		bt->bt_baud   = rcu_clock_freq_get(CK_APB1) / (float)(priv->psc * (uint32_t)(priv->sjw + priv->bs1 + priv->bs2));
    caninfo("clk:%d baud:%d\n", rcu_clock_freq_get(CK_APB1), bt->bt_baud);
		ret = OK;
	}
		break;

		/* CANIOC_SET_BITTIMING:
		 *   Description:    Set new current bit timing values
		 *   Argument:       A pointer to a read-able instance of struct
		 *                   canioc_bittiming_s in which the new bit timing
		 *                   values are provided.
		 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
		 *                   (ERROR)is returned with the errno variable set
		 *                    to indicate thenature of the error.
		 *   Dependencies:   None
		 *
		 * REVISIT: There is probably a limitation here:  If there are
		 * multiple threads trying to send CAN packets, when one of these
		 * threads reconfigures the bitrate, the MCAN hardware will be reset
		 * and the context of operation will be lost.  Hence, this IOCTL can
		 * only safely be executed in quiescent time periods.
		 */

	case CANIOC_SET_BITTIMING: {
		FAR const struct canioc_bittiming_s *bt =
				(FAR const struct canioc_bittiming_s*) arg;

		DEBUGASSERT(bt != NULL);
		DEBUGASSERT(bt->bt_sjw > 0 && bt->bt_sjw <= 4);
		DEBUGASSERT(bt->bt_tseg1 > 0 && bt->bt_tseg1 <= 16);
		DEBUGASSERT(bt->bt_tseg2 > 0 && bt->bt_tseg2 <= 8);

    ret = -ENOSYS;
  }
		break;

		/* CANIOC_GET_CONNMODES:
		 *   Description:    Get the current bus connection modes
		 *   Argument:       A pointer to a write-able instance of struct
		 *                   canioc_connmodes_s in which the new bus modes will
		 *                   be returned.
		 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
		 *                   (ERROR)is returned with the errno variable set
		 *                   to indicate the nature of the error.
		 *   Dependencies:   None
		 */

	case CANIOC_GET_CONNMODES: {
		FAR struct canioc_connmodes_s *bm = (FAR struct canioc_connmodes_s*) arg;
    ret = -ENOSYS;
	}
		break;

		/* CANIOC_SET_CONNMODES:
		 *   Description:    Set new bus connection modes values
		 *   Argument:       A pointer to a read-able instance of struct
		 *                   canioc_connmodes_s in which the new bus modes
		 *                   are provided.
		 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
		 *                   (ERROR) is returned with the errno variable set
		 *                   to indicate the nature of the error.
		 *   Dependencies:   None
		 */

	case CANIOC_SET_CONNMODES: {
		FAR struct canioc_connmodes_s *bm = (FAR struct canioc_connmodes_s*) arg;
    ret = -ENOSYS;
  }
		break;

#ifdef CONFIG_CAN_EXTID
      /* CANIOC_ADD_EXTFILTER:
       *   Description:    Add an address filter for a extended 29 bit
       *                   address.
       *   Argument:       A reference to struct canioc_extfilter_s
       *   Returned Value: A non-negative filter ID is returned on success.
       *                   Otherwise -1 (ERROR) is returned with the errno
       *                   variable set to indicate the nature of the error.
       */

      case CANIOC_ADD_EXTFILTER:
        {
          DEBUGASSERT(arg != 0);
          ret = gd32can_addextfilter(priv,
                                      (FAR struct canioc_extfilter_s *)arg);
        }
        break;

      /* CANIOC_DEL_EXTFILTER:
       *   Description:    Remove an address filter for a standard 29 bit
       *                   address.
       *   Argument:       The filter index previously returned by the
       *                   CANIOC_ADD_EXTFILTER command
       *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
       *                   (ERROR)is returned with the errno variable set
       *                   to indicate the nature of the error.
       */

      case CANIOC_DEL_EXTFILTER:
        {
#if 0 /* Unimplemented */
          DEBUGASSERT(arg <= priv->config->nextfilters);
#endif
          ret = gd32can_delextfilter(priv, (int)arg);
        }
        break;
#endif

		/* CANIOC_ADD_STDFILTER:
		 *   Description:    Add an address filter for a standard 11 bit
		 *                   address.
		 *   Argument:       A reference to struct canioc_stdfilter_s
		 *   Returned Value: A non-negative filter ID is returned on success.
		 *                   Otherwise -1 (ERROR) is returned with the errno
		 *                   variable set to indicate the nature of the error.
		 */

	case CANIOC_ADD_STDFILTER: {
		DEBUGASSERT(arg != 0);
		ret = gd32can_addstdfilter(priv, (FAR struct canioc_stdfilter_s*) arg);
	}
		break;

		/* CANIOC_DEL_STDFILTER:
		 *   Description:    Remove an address filter for a standard 11 bit
		 *                   address.
		 *   Argument:       The filter index previously returned by the
		 *                   CANIOC_ADD_STDFILTER command
		 *   Returned Value: Zero (OK) is returned on success.  Otherwise -1
		 *                   (ERROR) is returned with the errno variable set
		 *                   to indicate the nature of the error.
		 */

	case CANIOC_DEL_STDFILTER: {
		ret = gd32can_delstdfilter(priv, (int) arg);
	}
		break;

		/* Unsupported/unrecognized command */
	case CANIOC_SET_NART: 
	case CANIOC_SET_ABOM: 
    ret = -ENOSYS;
	default:
		canerr ("ERROR: Unrecognized command: %04x\n", cmd);
	}

	return ret;
}

/****************************************************************************
 * Name: gd32can_remoterequest
 *
 * Description:
 *   Send a remote request
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_remoterequest(FAR struct can_dev_s *dev, uint16_t id) {
#warning "Remote request not implemented"
	return -ENOSYS;
}

static bool gd32can_txmb0empty(FAR struct gd32_can_s * priv){
  return CAN_TSTAT_TME0 == (CAN_TSTAT(priv->can_periph) & CAN_TSTAT_TME0);
}
static bool gd32can_txmb1empty(FAR struct gd32_can_s * priv){
  return CAN_TSTAT_TME1 == (CAN_TSTAT(priv->can_periph) & CAN_TSTAT_TME1);
}
static bool gd32can_txmb2empty(FAR struct gd32_can_s * priv){
  return CAN_TSTAT_TME2 == (CAN_TSTAT(priv->can_periph) & CAN_TSTAT_TME2);
}

/****************************************************************************
 * Name: gd32can_send
 *
 * Description:
 *    Send one can message.
 *
 *    One CAN-message consists of a maximum of 10 bytes.  A message is
 *    composed of at least the first 2 bytes (when there are no data bytes).
 *
 *    Byte 0:      Bits 0-7: Bits 3-10 of the 11-bit CAN identifier
 *    Byte 1:      Bits 5-7: Bits 0-2 of the 11-bit CAN identifier
 *                 Bit 4:    Remote Transmission Request (RTR)
 *                 Bits 0-3: Data Length Code (DLC)
 *    Bytes 2-10: CAN data
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_send(FAR struct can_dev_s *dev, FAR struct can_msg_s *msg) {
	FAR struct gd32_can_s *priv = dev->cd_priv;
	int txmb;
  can_trasnmit_message_struct send_msg;


  send_msg.tx_sfid = msg->cm_hdr.ch_id;
  send_msg.tx_ft = CAN_FT_DATA;
  send_msg.tx_ff = CAN_FF_STANDARD;
  send_msg.tx_dlen = msg->cm_hdr.ch_dlc;
  memcpy(send_msg.tx_data, msg->cm_data, send_msg.tx_dlen);

  txmb = can_message_transmit(priv->can_periph, &send_msg);

  if (txmb == CAN_NOMAILBOX)
	return -EBUSY;

	caninfo ("CAN%d ID: %" PRId32 " DLC: %d send mb:%d\n", 
           priv->port, (uint32_t) msg->cm_hdr.ch_id, msg->cm_hdr.ch_dlc, txmb);

	return OK;
}

/****************************************************************************
 * Name: gd32can_txready
 *
 * Description:
 *   Return true if the CAN hardware can accept another TX message.
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if the CAN hardware is ready to accept another TX message.
 *
 ****************************************************************************/

static bool gd32can_txready(FAR struct can_dev_s *dev) {
	FAR struct gd32_can_s *priv = dev->cd_priv;
	uint32_t regval;

	/* Return true if any mailbox is available */
	return gd32can_txmb0empty(priv) || gd32can_txmb1empty(priv)
			|| gd32can_txmb2empty(priv);
}

/****************************************************************************
 * Name: gd32can_txempty
 *
 * Description:
 *   Return true if all message have been sent.  If for example, the CAN
 *   hardware implements FIFOs, then this would mean the transmit FIFO is
 *   empty.  This method is called when the driver needs to make sure that
 *   all characters are "drained" from the TX hardware before calling
 *   co_shutdown().
 *
 * Input Parameters:
 *   dev - An instance of the "upper half" can driver state structure.
 *
 * Returned Value:
 *   True if there are no pending TX transfers in the CAN hardware.
 *
 ****************************************************************************/

static bool gd32can_txempty(FAR struct can_dev_s *dev) {
	FAR struct gd32_can_s *priv = dev->cd_priv;
	return gd32can_txmb0empty(priv) && gd32can_txmb1empty(priv)
			&& gd32can_txmb2empty(priv);
}

/****************************************************************************
 * Name: gd32can_rxinterrupt
 *
 * Description:
 *   CAN RX FIFO 0/1 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *   rxmb - The RX mailbox number.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_rxinterrupt(FAR struct can_dev_s *dev, int fifo_num) {
	FAR struct gd32_can_s *priv;
	struct can_hdr_s hdr = {0};
	uint8_t data[CAN_MAXDATALEN];
	can_receive_message_struct recv_msg;
	int ret;

	DEBUGASSERT(dev != NULL && dev->cd_priv != NULL);
	priv = dev->cd_priv;

	uint8_t msg_num = can_receive_message_length_get(priv->can_periph, fifo_num);

	if (msg_num > 0)
	{
		can_message_receive(priv->can_periph, fifo_num, &recv_msg);
		hdr.ch_id = recv_msg.rx_sfid;
		hdr.ch_unused = 0;
		hdr.ch_dlc = recv_msg.rx_dlen;
		memcpy(data, recv_msg.rx_data,  hdr.ch_dlc);
		ret = can_receive(dev, &hdr, data);
	}

	return ret;
}

/****************************************************************************
 * Name: gd32can_rx0interrupt
 *
 * Description:
 *   CAN RX FIFO 0 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_rx0interrupt(int irq, FAR void *context, FAR void *arg) {
	FAR struct can_dev_s *dev = (FAR struct can_dev_s*) arg;
	return gd32can_rxinterrupt(dev, CAN_FIFO0);
}

/****************************************************************************
 * Name: gd32can_rx1interrupt
 *
 * Description:
 *   CAN RX FIFO 1 interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_rx1interrupt(int irq, FAR void *context, FAR void *arg) {
	FAR struct can_dev_s *dev = (FAR struct can_dev_s*) arg;
	return gd32can_rxinterrupt(dev, CAN_FIFO1);
}

/****************************************************************************
 * Name: gd32can_txinterrupt
 *
 * Description:
 *   CAN TX mailbox complete interrupt handler
 *
 * Input Parameters:
 *   irq - The IRQ number of the interrupt.
 *   context - The register state save array at the time of the interrupt.
 *
 * Returned Value:
 *   Zero on success; a negated errno on failure
 *
 ****************************************************************************/

static int gd32can_txinterrupt(int irq, FAR void *context, FAR void *arg) {
	FAR struct can_dev_s *dev = (FAR struct can_dev_s*) arg;
	FAR struct gd32_can_s *priv;
	uint32_t regval;

	DEBUGASSERT(dev != NULL && dev->cd_priv != NULL);
	priv = dev->cd_priv;

  if (can_interrupt_flag_get(priv->can_periph, CAN_INT_FLAG_MTF0) == SET)
  {
		can_txdone(dev);
    can_interrupt_flag_clear(priv->can_periph, CAN_INT_FLAG_MTF0);
	}
  else if (can_interrupt_flag_get(priv->can_periph, CAN_INT_FLAG_MTF1) == SET)
  {
		can_txdone(dev);
    can_interrupt_flag_clear(priv->can_periph, CAN_INT_FLAG_MTF1);
	}
  else if (can_interrupt_flag_get(priv->can_periph, CAN_INT_FLAG_MTF2) == SET)
  {
		can_txdone(dev);
    can_interrupt_flag_clear(priv->can_periph, CAN_INT_FLAG_MTF2);
	}

	return OK;
}

/****************************************************************************
 * Name: gd32can_cellinit
 *
 * Description:
 *   CAN cell initialization
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32can_cellinit(FAR struct gd32_can_s *priv) {
	int ret;

	caninfo ("CAN%d baud:%d\n", priv->port, priv->baud);

  /* Calculate CAN bit timing*/
  uint32_t can_clk = rcu_clock_freq_get(CK_APB1);
  uint32_t can_baud = priv->baud;
  bool timing_found = false;

  priv->sjw = 1;
  for (priv->psc = 1; priv->psc < 1024 && !timing_found; priv->psc++)
  {
    for (priv->bs1 = 1; priv->bs1 < 16 && !timing_found; priv->bs1++)
    {
      for (priv->bs2 = 1; priv->bs2 < 8 && !timing_found; priv->bs2++)
      {
        uint32_t bit_time_unit = priv->sjw + priv->bs1 + priv->bs2;
        uint32_t calc_baud = can_clk/(priv->psc * bit_time_unit);
        if (can_baud == calc_baud)
        {
          caninfo("can_baud:%d, calc_baud:%d\n", can_baud, calc_baud);
          priv->sample_point = ((priv->sjw + priv->bs1) * 1000) / bit_time_unit;
          if (priv->sample_point >= (CONFIG_GD32_CAN_SAMPLE_POINT - 50) &&
              priv->sample_point <= (CONFIG_GD32_CAN_SAMPLE_POINT + 50))
          {
            timing_found = true;
            goto found_timing;
          }
        }
      }
    }
  }
found_timing:
  if (!timing_found)
  {
    canerr("ERROR: failed to calc bit timing!\n");
    return -EINVAL;
  }

  can_parameter_struct can_parameter;

  can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
  can_deinit(priv->can_periph);

  caninfo("calc bit time,clk:%d, psc:%d, sjw:%d, bs1:%d, bs2:%d, sample_point:%d\n", 
          can_clk, priv->psc, priv->sjw, priv->bs1,priv->bs2,priv->sample_point);

  /* initialize CAN */
  can_parameter.time_triggered = DISABLE;
  can_parameter.auto_bus_off_recovery = DISABLE;
  can_parameter.auto_wake_up = DISABLE;
  can_parameter.no_auto_retrans = DISABLE;
  can_parameter.rec_fifo_overwrite = DISABLE;
  can_parameter.trans_fifo_order = DISABLE;
  can_parameter.working_mode = CAN_NORMAL_MODE;

  can_parameter.resync_jump_width = priv->sjw-1;
  can_parameter.time_segment_1 = priv->bs1-1;
  can_parameter.time_segment_2 = priv->bs2-1;
  can_parameter.prescaler = priv->psc;

  can_init(priv->can_periph, &can_parameter);

  return OK;
}

/****************************************************************************
 * Name: gd32can_filterinit
 *
 * Description:
 *   CAN filter initialization.  CAN filters are not currently used by this
 *   driver.  The CAN filters can be configured in a different way:
 *
 *   1. As a match of specific IDs in a list (IdList mode), or as
 *   2. And ID and a mask (IdMask mode).
 *
 *   Filters can also be configured as:
 *
 *   3. 16- or 32-bit.  The advantage of 16-bit filters is that you get
 *      more filters;  The advantage of 32-bit filters is that you get
 *      finer control of the filtering.
 *
 *   One filter is set up for each CAN.  The filter resources are shared
 *   between the two CAN modules:  CAN1 uses only filter 0 (but reserves
 *   0 through CAN_NFILTERS/2-1); CAN2 uses only filter CAN_NFILTERS/2
 *   (but reserves CAN_NFILTERS/2 through CAN_NFILTERS-1).
 *
 *   32-bit IdMask mode is configured.  However, both the ID and the MASK
 *   are set to zero thus suppressing all filtering because anything masked
 *   with zero matches zero.
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int gd32can_filterinit(FAR struct gd32_can_s *priv) {
  can_filter_parameter_struct     can_filter;
	caninfo ("CAN%d filter: %d\n", priv->port, priv->filter);

  can_filter.filter_number = 0;
  can_filter.filter_mode = CAN_FILTERMODE_MASK;
  can_filter.filter_bits = CAN_FILTERBITS_32BIT;
  can_filter.filter_list_high = 0x0000;
  can_filter.filter_list_low = 0x0000;
  can_filter.filter_mask_high = 0x0000;
  can_filter.filter_mask_low = 0x0000;  
  can_filter.filter_fifo_number = CAN_FIFO0;
  can_filter.filter_enable = ENABLE;
  can_filter_init(&can_filter);

	return OK;
}

/****************************************************************************
 * Name: gd32can_addextfilter
 *
 * Description:
 *   Add a filter for extended CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - A pointer to a structure describing the filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.
 *   Otherwise -1 (ERROR) is returned with the errno
 *   set to indicate the nature of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int gd32can_addextfilter(FAR struct gd32_can_s *priv,
                                 FAR struct canioc_extfilter_s *arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: gd32can_delextfilter
 *
 * Description:
 *   Remove a filter for extended CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - The filter index previously returned by the
 *            CANIOC_ADD_EXTFILTER command
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *   returned with the errno variable set to indicate the
 *   of the error.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_EXTID
static int gd32can_delextfilter(FAR struct gd32_can_s *priv, int arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Name: gd32can_addstdfilter
 *
 * Description:
 *   Add a filter for standard CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - A pointer to a structure describing the filter
 *
 * Returned Value:
 *   A non-negative filter ID is returned on success.
 *   Otherwise -1 (ERROR) is returned with the errno
 *   set to indicate the nature of the error.
 *
 ****************************************************************************/

static int gd32can_addstdfilter(FAR struct gd32_can_s *priv,
		FAR struct canioc_stdfilter_s *arg) {
	return -ENOTTY;
}

/****************************************************************************
 * Name: gd32can_delstdfilter
 *
 * Description:
 *   Remove a filter for standard CAN IDs
 *
 * Input Parameters:
 *   priv - A pointer to the private data structure for this CAN block
 *   arg  - The filter index previously returned by the
 *            CANIOC_ADD_STDFILTER command
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR)
 *   returned with the errno variable set to indicate the
 *   of the error.
 *
 ****************************************************************************/

static int gd32can_delstdfilter(FAR struct gd32_can_s *priv, int arg) {
	return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_caninitialize
 *
 * Description:
 *   Initialize the selected CAN port
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple CAN interfaces)
 *
 * Returned Value:
 *   Valid CAN device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct can_dev_s* gd32_caninitialize(int port) {
	FAR struct can_dev_s *dev = NULL;
	caninfo ("CAN%d\n", port);
	switch (port) {
#ifdef CONFIG_GD32_CAN0
    case 0:
      return &g_can0dev;
#endif
#ifdef CONFIG_GD32_CAN1
    case 1:
      return &g_can1dev;
#endif
	default:
		canerr ("ERROR: Unsupported port %d\n", port);
		return NULL;
	}
}

#endif /* CONFIG_GD32_CAN */ 
