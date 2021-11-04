/****************************************************************************
 * arch/arm/src/gd32f4xx/gd32_spi.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "gd32f4xx.h"

#define CONFIG_GD32_SPI

#ifdef CONFIG_GD32_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gd32_spidev_s {
	struct spi_dev_s spidev; /* Externally visible part of the SPI interface */
	uint32_t spi_periph;
	uint8_t spi_irq; /* SPI IRQ number */
	bool initialized; /* Has SPI interface been initialized */
	sem_t exclsem; /* Held while chip is selected for mutual exclusion */

	uint32_t pclk_clock;
	uint32_t frequency; /* Requested clock frequency */
	uint32_t actual; /* Actual clock frequency */

	uint8_t nbits; /* Width of word in bits (4 through 16) */
	uint8_t mode; /* Mode 0,1,2,3 */
	//
	uint32_t gpio_mosi_periph;
	uint32_t gpio_mosi_pin;
	uint32_t gpio_mosi_af;

	uint32_t gpio_miso_periph;
	uint32_t gpio_miso_pin;
	uint32_t gpio_miso_af;

	uint32_t gpio_sck_periph;
	uint32_t gpio_sck_pin;
	uint32_t gpio_sck_af;
	spi_parameter_struct init_struct;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */

static int spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency);
static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode);
static void spi_setbits(FAR struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int         spi_hwfeatures(FAR struct spi_dev_s *dev,
                                  spi_hwfeatures_t features);
#endif
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
		FAR void *rxbuffer, size_t nwords);
#ifdef CONFIG_SPI_TRIGGER
static int         spi_trigger(FAR struct spi_dev_s *dev);
#endif
#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
		size_t nwords);
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
		size_t nwords);
#endif

/* Initialization */

static void spi_bus_initialize(FAR struct gd32_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_GD32_SPI0
static const struct spi_ops_s g_spi0iops =
{
  .lock              = spi_lock,
  .select            = gd32_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = spi_hwfeatures,
#endif
  .status            = gd32_spi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = gd32_spi0cmddata,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = gd32_spi0register,  /* Provided externally */
#else
  .registercallback  = 0,                   /* Not implemented */
#endif
};

static struct gd32_spidev_s g_spi0dev =
{
  .spidev   =
  {
    &g_spi0ops
  },
  .spi_periph = GD32_SPI0_BASE,
  .spi_irq   = GD32_IRQ_NUM(SPI0_IRQn),

  .gpio_mosi_periph = GPIO_SPI0_MOSI_PORT,
  .gpio_mosi_pin    = GPIO_SPI0_MOSI_PIN,
  .gpio_mosi_af     = GPIO_SPI0_MOSI_AF,

  .gpio_miso_periph = GPIO_SPI0_MISO_PORT,
  .gpio_miso_pin    = GPIO_SPI0_MISO_PIN,
  .gpio_miso_af     = GPIO_SPI0_MISO_AF,

  .gpio_sck_periph = GPIO_SPI0_SCK_PORT,
  .gpio_sck_pin    = GPIO_SPI0_SCK_PIN,
  .gpio_sck_af     = GPIO_SPI0_SCK_AF,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t _get_gpio_rcu_periph(uint32_t gpio_periph) {
#define GPIO_RCU(gpio) case gpio: return RCU_##gpio
	switch (gpio_periph) {
	GPIO_RCU(GPIOA)
;		GPIO_RCU(GPIOB);
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

static uint32_t _get_spi_rcu_periph(uint32_t spi_periph) {
#define SPI_RCU(spi) case spi: return RCU_##spi
	switch (spi_periph) {
		SPI_RCU(SPI0);
		SPI_RCU(SPI1);
		SPI_RCU(SPI2);
		SPI_RCU(SPI3);
		SPI_RCU(SPI4);
		SPI_RCU(SPI5);
		default:
		PANIC();
	}
#undef SPI_RCU
}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock) {
	FAR struct gd32_spidev_s *priv = (FAR struct gd32_spidev_s*) dev;
	int ret;

	if (lock) {
		ret = nxsem_wait_uninterruptible(&priv->exclsem);
	} else {
		ret = nxsem_post(&priv->exclsem);
	}

	return ret;
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t frequency) {
	FAR struct gd32_spidev_s *priv = (FAR struct gd32_spidev_s*) dev;
	uint32_t psc;
	uint32_t actual;

	if (frequency != priv->frequency) {
		/* Choices are limited by PCLK frequency with a set of divisors */

		if (frequency >= priv->pclk_clock >> 1) {
			psc = SPI_PSC_2;
			actual = priv->pclk_clock >> 1;
		} else if (frequency >= priv->pclk_clock >> 2) {
			psc = SPI_PSC_4;
			actual = priv->pclk_clock >> 2;
		} else if (frequency >= priv->pclk_clock >> 3) {
			psc = SPI_PSC_8;
			actual = priv->pclk_clock >> 3;
		} else if (frequency >= priv->pclk_clock >> 4) {
			psc = SPI_PSC_16;
			actual = priv->pclk_clock >> 4;
		} else if (frequency >= priv->pclk_clock >> 5) {
			psc = SPI_PSC_32;
			actual = priv->pclk_clock >> 5;
		} else if (frequency >= priv->pclk_clock >> 6) {
			psc = SPI_PSC_64;
			actual = priv->pclk_clock >> 6;
		} else if (frequency >= priv->pclk_clock >> 7) {
			psc = SPI_PSC_128;
			actual = priv->pclk_clock >> 7;
		} else {
			psc = SPI_PSC_256;
			actual = priv->pclk_clock >> 8;
		}

		uint32_t reg = 0u;
		reg = SPI_CTL0(priv->spi_periph);
		reg |= psc;
		SPI_CTL0(priv->spi_periph) = (uint32_t) reg;

		spiinfo ("Frequency %" PRIu32 "->%" PRIu32 "\n", frequency, actual);

		priv->frequency = frequency;
		priv->actual = actual;
	}

	return priv->actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode) {
	FAR struct gd32_spidev_s *priv = (FAR struct gd32_spidev_s*) dev;

	uint32_t spi_clock_pl_and_ph = SPI_CK_PL_LOW_PH_1EDGE;

	spiinfo ("mode=%d\n", mode);

	if (mode != priv->mode) {
		switch (mode) {
		case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
			spi_clock_pl_and_ph = SPI_CK_PL_LOW_PH_1EDGE;
			break;

		case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
			spi_clock_pl_and_ph = SPI_CK_PL_HIGH_PH_1EDGE;
			break;

		case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
			spi_clock_pl_and_ph = SPI_CK_PL_LOW_PH_2EDGE;
			break;

		case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
			spi_clock_pl_and_ph = SPI_CK_PL_HIGH_PH_2EDGE;
			break;

		case SPIDEV_MODETI:
			spi_clock_pl_and_ph = SPI_CK_PL_LOW_PH_1EDGE;
			break;

		default:
			return;
		}

		uint32_t reg = 0U;
		reg = SPI_CTL0(priv->spi_periph);
		reg |= spi_clock_pl_and_ph;
		SPI_CTL0(priv->spi_periph) = (uint32_t) reg;
		priv->mode = mode;
	}
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits) {
	FAR struct gd32_spidev_s *priv = (FAR struct gd32_spidev_s*) dev;
	uint32_t frame_size;
	spiinfo ("nbits=%d\n", nbits);

	if (nbits != priv->nbits) {
		switch (nbits) {
		case 8:
			frame_size = SPI_FRAMESIZE_8BIT;
			break;

		case 16:
			frame_size = SPI_FRAMESIZE_8BIT;
			break;

		default:
			return;
		}

		uint32_t reg = 0U;
		reg = SPI_CTL0(priv->spi_periph);
		reg |= frame_size;
		SPI_CTL0(priv->spi_periph) = (uint32_t)reg;
		priv->nbits = nbits;
	}
}

/****************************************************************************
 * Name: spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_HWFEATURES
static int spi_hwfeatures(FAR struct spi_dev_s *dev,
                          spi_hwfeatures_t features)
{
#if defined(CONFIG_SPI_BITORDER) || defined(CONFIG_SPI_TRIGGER)
  FAR struct gd32_spidev_s *priv = (FAR struct gd32_spidev_s *)dev;
#endif

#ifdef CONFIG_SPI_BITORDER
  uint32_t bitorder;

  spiinfo("features=%08x\n", features);

  /* Transfer data LSB first? */

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
	  bitorder = SPI_ENDIAN_LSB;
    }
  else
    {
	  bitorder = SPI_ENDIAN_MSB;
    }
  uint32_t reg = 0U;
  reg = SPI_CTL0(priv->spi_periph);
  reg |= bitorder;
  SPI_CTL0(priv->spi_periph) = (uint32_t)reg;

  features &= ~HWFEAT_LSBFIRST;
#endif

#ifdef CONFIG_SPI_TRIGGER
/* Turn deferred trigger mode on or off.  Only applicable for DMA mode. If a
 * transfer is deferred then the DMA will not actually be triggered until a
 * subsequent call to SPI_TRIGGER to set it off. The thread will be waiting
 * on the transfer completing as normal.
 */
  features &= ~HWFEAT_TRIGGER;
#endif

  /* Other H/W features are not supported */

  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd) {
	FAR struct gd32_spidev_s *priv = (FAR struct gd32_spidev_s*) dev;
	uint32_t regval;
	uint32_t ret;

    while(RESET == spi_i2s_flag_get(priv->spi_periph, SPI_FLAG_TBE));
	spi_i2s_data_transmit(priv->spi_periph, (uint32_t) (wd & 0xffff));
    while(RESET == spi_i2s_flag_get(priv->spi_periph, SPI_FLAG_RBNE));
	ret = (uint32_t) spi_i2s_data_receive(priv->spi_periph);

	regval = SPI_STAT(priv->spi_periph);

	spiinfo ("Sent: %04" PRIx32 " Return: %04" PRIx32
	" Status: %02" PRIx32 "\n", wd, ret, regval);

	return ret;
}

/****************************************************************************
 * Name: spi_exchange (no DMA).  aka spi_exchange_nodma
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 *   REVISIT:
 *   This function could be much more efficient by exploiting (1) RX and TX
 *   FIFOs and (2) the GD32 F3 data packing.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
		FAR void *rxbuffer, size_t nwords)
{
	FAR struct gd32_spidev_s *priv = (FAR struct gd32_spidev_s*) dev;

	spiinfo ("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

	/* 8- or 16-bit mode? */

	if (priv->nbits > 8) {
		/* 16-bit mode */

		const uint16_t *src = (const uint16_t*) txbuffer;
		uint16_t *dest = (uint16_t*) rxbuffer;
		uint16_t word;

		while (nwords-- > 0) {
			/* Get the next word to write.  Is there a source buffer? */

			if (src) {
				word = *src++;
			} else {
				word = 0xffff;
			}

			/* Exchange one word */

			word = (uint16_t) spi_send(dev, (uint32_t) word);

			/* Is there a buffer to receive the return value? */

			if (dest) {
				*dest++ = word;
			}
		}
	} else {
		/* 8-bit mode */

		const uint8_t *src = (const uint8_t*) txbuffer;
		uint8_t *dest = (uint8_t*) rxbuffer;
		uint8_t word;

		while (nwords-- > 0) {
			/* Get the next word to write.  Is there a source buffer? */

			if (src) {
				word = *src++;
			} else {
				word = 0xff;
			}

			/* Exchange one word */

			word = (uint8_t) spi_send(dev, (uint32_t) word);

			/* Is there a buffer to receive the return value? */

			if (dest) {
				*dest++ = word;
			}
		}
	}
}

/****************************************************************************
 * Name: spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   ENOTSUP  - Trigger not fired due to lack of DMA support
 *   EIO      - Trigger not fired because not previously primed
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_TRIGGER
static int spi_trigger(FAR struct spi_dev_s *dev)
{
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of
 *              words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
		size_t nwords) {
	spiinfo ("txbuffer=%p nwords=%d\n", txbuffer, nwords);
	return spi_exchange(dev, txbuffer, NULL, nwords);
}
#endif

/****************************************************************************
 * Name: spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in
 *              number of words.  The wordsize is determined by the number
 *              of bits-per-word selected for the SPI interface.  If
 *              nbits <= 8, the data is packed into uint8_t's; if nbits >8,
 *              the data is packed into uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_SPI_EXCHANGE
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
		size_t nwords) {
	spiinfo ("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
	return spi_exchange(dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_bus_initialize(FAR struct gd32_spidev_s *priv) {
	rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_mosi_periph));
	rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_miso_periph));
	rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_sck_periph));
	rcu_periph_clock_enable(_get_spi_rcu_periph(priv->spi_periph));

#define GPIO_INIT(type) \
do{\
  gpio_af_set(priv->gpio_##type##_periph, priv->gpio_##type##_af, priv->gpio_##type##_pin);\
  gpio_mode_set(priv->gpio_##type##_periph, GPIO_MODE_AF, GPIO_PUPD_NONE, priv->gpio_##type##_pin);\
  gpio_output_options_set(priv->gpio_##type##_periph, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, priv->gpio_##type##_pin);\
}while(0)

	GPIO_INIT(mosi);
	GPIO_INIT(miso);
	GPIO_INIT(sck);
#undef GPIO_INIT

	if (priv->spi_periph == SPI0 || priv->spi_periph == SPI3
			|| priv->spi_periph == SPI4 || priv->spi_periph == SPI5) {
		priv->pclk_clock = rcu_clock_freq_get(CK_APB1);
	} else {
		priv->pclk_clock = rcu_clock_freq_get(CK_APB2);
	}

	spi_struct_para_init(&priv->init_struct);

	priv->init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
	priv->init_struct.device_mode = SPI_MASTER;
	priv->init_struct.frame_size = SPI_FRAMESIZE_8BIT;
	priv->init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
	priv->init_struct.nss = SPI_NSS_SOFT;
	priv->init_struct.prescale = SPI_PSC_2;
	priv->init_struct.endian = SPI_ENDIAN_MSB;

	spi_init(priv->spi_periph, &priv->init_struct);

	spi_setfrequency(priv, 400000); //default 400K

	nxsem_init(&priv->exclsem, 0, 1);

	spi_enable(priv->spi_periph);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s* gd32_spibus_initialize(int bus) {
	FAR struct gd32_spidev_s *priv = NULL;

	irqstate_t flags = enter_critical_section();

#define GD32_SPI_INIT(n) \
case (n):\
  priv=&g_spi##n##dev;\
  if (!priv->initialized)\
  {\
    spi_bus_initialize(priv);\
    priv->initialized = true;\
  }\
  break

	switch (bus) {
#ifdef CONFIG_GD32_SPI0
  GD32_SPI_INIT(0);
#endif
#ifdef CONFIG_GD32_SPI1
  GD32_SPI_INIT(1);
#endif
#ifdef CONFIG_GD32_SPI2
  GD32_SPI_INIT(2);
#endif
#ifdef CONFIG_GD32_SPI3
  GD32_SPI_INIT(3);
#endif
#ifdef CONFIG_GD32_SPI4
  GD32_SPI_INIT(4);
#endif
#ifdef CONFIG_GD32_SPI5
  GD32_SPI_INIT(5);
#endif
	default:
		PANIC();
#undef GD32_SPI_INIT
	}
	leave_critical_section(flags);
	return (FAR struct spi_dev_s*) priv;
}

#endif /* #ifdef CONFIG_GD32_SPI */
