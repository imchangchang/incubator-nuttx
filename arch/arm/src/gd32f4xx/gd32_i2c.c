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

/* Supports:
 *  - Master operation, 100 kHz (standard) and 400 kHz (full speed)
 *  - Multiple instances (shared bus)
 *  - Interrupt based operation
 *
 * Structure naming:
 *  - Device: structure as defined by the nuttx/i2c/i2c.h
 *  - Instance: represents each individual access to the I2C driver, obtained
 *     by the i2c_init(); it extends the Device structure from the
 *     nuttx/i2c/i2c.h;
 *     Instance points to OPS, to common I2C Hardware private data and
 *     contains its own private data, as frequency, address, mode of
 *     operation (in the future)
 *  - Private: Private data of an I2C Hardware
 *
 * TODO
 *  - Check for all possible deadlocks (as BUSY='1' I2C needs to be reset in
 *    HW using the I2C_CR1_SWRST)
 *  - SMBus support (hardware layer timings are already supported) and add
 *    SMBA gpio pin
 *  - Slave support with multiple addresses (on multiple instances):
 *      - 2 x 7-bit address or
 *      - 1 x 10 bit addresses + 1 x 7 bit address (?)
 *      - plus the broadcast address (general call)
 *  - Multi-master support
 *  - DMA (to get rid of too many CPU wake-ups and interventions)
 *  - Be ready for IPMI
 */

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

/* Debug ********************************************************************/

/* I2C event trace logic.  NOTE:  trace uses the internal, non-standard,
 * low-level debug interface syslog() but does not require that any other
 * debug is enabled.
 */

#ifndef CONFIG_I2C_TRACE
#  define gd32_i2c_tracereset(p)
#  define gd32_i2c_tracenew(p,s)
#  define gd32_i2c_traceevent(p,e,a)
#  define gd32_i2c_tracedump(p)
#endif

#ifndef CONFIG_I2C_NTRACE
#  define CONFIG_I2C_NTRACE 32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Interrupt state */

enum gd32_intstate_e
{
  INTSTATE_IDLE = 0,      /* No I2C activity */
  INTSTATE_WAITING,       /* Waiting for completion of interrupt activity */
  INTSTATE_DONE,          /* Interrupt activity complete */
};

/* Trace events */

enum gd32_trace_e
{
  I2CEVENT_NONE = 0,      /* No events have occurred with this status */
  I2CEVENT_SENDADDR,      /* Start/Master bit set and address sent, param = msgc */
  I2CEVENT_SENDBYTE,      /* Send byte, param = dcnt */
  I2CEVENT_ITBUFEN,       /* Enable buffer interrupts, param = 0 */
  I2CEVENT_RCVBYTE,       /* Read more dta, param = dcnt */
  I2CEVENT_REITBUFEN,     /* Re-enable buffer interrupts, param = 0 */
  I2CEVENT_DISITBUFEN,    /* Disable buffer interrupts, param = 0 */
  I2CEVENT_BTFNOSTART,    /* BTF on last byte with no restart, param = msgc */
  I2CEVENT_BTFRESTART,    /* Last byte sent, re-starting, param = msgc */
  I2CEVENT_BTFSTOP,       /* Last byte sten, send stop, param = 0 */
  I2CEVENT_ERROR          /* Error occurred, param = 0 */
};

/* Trace data */

struct gd32_trace_s
{
  uint32_t status;             /* I2C 32-bit SR2|SR1 status */
  uint32_t count;              /* Interrupt count when status change */
  enum gd32_intstate_e event; /* Last event that occurred with this status */
  uint32_t parm;               /* Parameter associated with the event */
  clock_t time;                /* First of event or first status */
};


/* I2C Device Private Data */

struct gd32_i2c_priv_s
{
  /* Standard I2C operations */

  const struct i2c_ops_s *ops;

  uint32_t i2c_periph;

  uint32_t gpio_scl_periph;
  uint32_t gpio_scl_pin;
  uint32_t gpio_scl_af;
  uint32_t gpio_sda_periph;
  uint32_t gpio_sda_pin;
  uint32_t gpio_sda_af;

  uint32_t ev_irq; 				/* event interrupt */
  uint32_t er_irq; 				/* error interrupt */


  int refs;                    /* Reference count */
  sem_t sem_excl;              /* Mutual exclusion semaphore */
  sem_t sem_isr;               /* Interrupt wait semaphore */

  volatile uint8_t intstate;   /* Interrupt handshake (see enum gd32_intstate_e) */

  uint8_t msgc;                /* Message count */
  struct i2c_msg_s *msgv;      /* Message list */
  uint8_t *ptr;                /* Current message buffer */
  uint32_t frequency;          /* Current I2C frequency */
  int dcnt;                    /* Current message length */
  uint16_t flags;              /* Current message flags */

  /* I2C trace support */

#ifdef CONFIG_I2C_TRACE
  int tndx;                    /* Trace array index */
  clock_t start_time;          /* Time when the trace was started */

  /* The actual trace data */

  struct gd32_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;             /* End of transfer SR2|SR1 status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline int  gd32_i2c_sem_waitdone(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_waitstop(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_post(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_init(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sem_destroy(FAR struct gd32_i2c_priv_s *priv);

#ifdef CONFIG_I2C_TRACE
static void gd32_i2c_tracereset(FAR struct gd32_i2c_priv_s *priv);
static void gd32_i2c_tracenew(FAR struct gd32_i2c_priv_s *priv,
                               uint32_t status);
static void gd32_i2c_traceevent(FAR struct gd32_i2c_priv_s *priv,
                                 enum gd32_trace_e event, uint32_t parm);
static void gd32_i2c_tracedump(FAR struct gd32_i2c_priv_s *priv);
#endif /* CONFIG_I2C_TRACE */

static void gd32_i2c_setclock(FAR struct gd32_i2c_priv_s *priv,
                               uint32_t frequency);
static inline void gd32_i2c_sendstart(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_clrstart(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_sendstop(FAR struct gd32_i2c_priv_s *priv);
static inline
uint32_t gd32_i2c_getstatus(FAR struct gd32_i2c_priv_s *priv);

#ifdef I2C1_FSMC_CONFLICT
static inline
uint32_t gd32_i2c_disablefsmc(FAR struct gd32_i2c_priv_s *priv);
static inline void gd32_i2c_enablefsmc(uint32_t ahbenr);
#endif /* I2C1_FSMC_CONFLICT */

static int gd32_i2c_isr_process(struct gd32_i2c_priv_s * priv);

#ifndef CONFIG_I2C_POLLED
static int gd32_i2c_isr(int irq, void *context, FAR void *arg);
#endif /* !CONFIG_I2C_POLLED */

static int gd32_i2c_init(FAR struct gd32_i2c_priv_s *priv);
static int gd32_i2c_deinit(FAR struct gd32_i2c_priv_s *priv);
static int gd32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int gd32_i2c_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Trace events strings */

#ifdef CONFIG_I2C_TRACE
static const char *g_trace_names[] =
{
  "NONE      ",
  "SENDADDR  ",
  "SENDBYTE  ",
  "ITBUFEN   ",
  "RCVBYTE   ",
  "REITBUFEN ",
  "DISITBUFEN",
  "BTFNOSTART",
  "BTFRESTART",
  "BTFSTOP   ",
  "ERROR     "
};
#endif

/* I2C interface */

static const struct i2c_ops_s gd32_i2c_ops =
{
  .transfer = gd32_i2c_transfer
};

/* I2C device structures */

#ifdef CONFIG_GD32_I2C0
static struct gd32_i2c_priv_s gd32_i2c0 =
{
	.ops = &gd32_i2c_ops,
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
 * Name: gd32_i2c_sem_waitdone
 *
 * Description:
 *   Wait for a transfer to complete
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static inline int gd32_i2c_sem_waitdone(FAR struct gd32_i2c_priv_s *priv)
{
  struct timespec abstime;
  irqstate_t flags;
  uint32_t regval;
  int ret;

  flags = enter_critical_section();

  /* Enable I2C interrupts */

  regval  = gd32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval |= (I2C_CR2_ITERREN | I2C_CR2_ITEVFEN);
  gd32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  do
    {
      /* Get the current time */

      clock_gettime(CLOCK_REALTIME, &abstime);

      /* Calculate a time in the future */

#if CONFIG_STM32_I2CTIMEOSEC > 0
      abstime.tv_sec += CONFIG_STM32_I2CTIMEOSEC;
#endif

      /* Add a value proportional to the number of bytes in the transfer */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
      abstime.tv_nsec += 1000 * gd32_i2c_tousecs(priv->msgc, priv->msgv);
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }

#elif CONFIG_STM32_I2CTIMEOMS > 0
      abstime.tv_nsec += CONFIG_STM32_I2CTIMEOMS * 1000 * 1000;
      if (abstime.tv_nsec >= 1000 * 1000 * 1000)
        {
          abstime.tv_sec++;
          abstime.tv_nsec -= 1000 * 1000 * 1000;
        }
#endif

      /* Wait until either the transfer is complete or the timeout expires */

      ret = nxsem_timedwait_uninterruptible(&priv->sem_isr, &abstime);
      if (ret < 0)
        {
          /* Break out of the loop on irrecoverable errors.  This would
           * include timeouts and mystery errors reported by nxsem_timedwait.
           */

          break;
        }
    }

  /* Loop until the interrupt level transfer is complete. */

  while (priv->intstate != INTSTATE_DONE);

  /* Set the interrupt state back to IDLE */

  priv->intstate = INTSTATE_IDLE;

  /* Disable I2C interrupts */

  regval  = gd32_i2c_getreg(priv, STM32_I2C_CR2_OFFSET);
  regval &= ~I2C_CR2_ALLINTS;
  gd32_i2c_putreg(priv, STM32_I2C_CR2_OFFSET, regval);

  leave_critical_section(flags);
  return ret;
}
#else
static inline int gd32_i2c_sem_waitdone(FAR struct gd32_i2c_priv_s *priv)
{
  clock_t timeout;
  clock_t start;
  clock_t elapsed;
  int ret;

  /* Get the timeout value */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
  timeout = USEC2TICK(gd32_i2c_tousecs(priv->msgc, priv->msgv));
#else
  timeout = CONFIG_STM32_I2CTIMEOTICKS;
#endif

  /* Signal the interrupt handler that we are waiting.  NOTE:  Interrupts
   * are currently disabled but will be temporarily re-enabled below when
   * nxsem_timedwait() sleeps.
   */

  priv->intstate = INTSTATE_WAITING;
  start = clock_systime_ticks();

  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Poll by simply calling the timer interrupt handler until it
       * reports that it is done.
       */

      gd32_i2c_isr_process(priv);
    }

  /* Loop until the transfer is complete. */

  while (priv->intstate != INTSTATE_DONE && elapsed < timeout);

  i2cinfo("intstate: %d elapsed: %ld threshold: %ld status: %08" PRIx32 "\n",
          priv->intstate, (long)elapsed, (long)timeout, priv->status);

  /* Set the interrupt state back to IDLE */

  ret = priv->intstate == INTSTATE_DONE ? OK : -ETIMEDOUT;
  priv->intstate = INTSTATE_IDLE;
  return ret;
}
#endif

/****************************************************************************
 * Name: gd32_i2c_sem_waitstop
 *
 * Description:
 *   Wait for a STOP to complete
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_waitstop(FAR struct gd32_i2c_priv_s *priv)
{
  clock_t start;
  clock_t elapsed;
  clock_t timeout;
  uint32_t cr1;
  uint32_t sr1;

  /* Select a timeout */

#ifdef CONFIG_STM32_I2C_DYNTIMEO
  timeout = USEC2TICK(CONFIG_STM32_I2C_DYNTIMEO_STARTSTOP);
#else
  timeout = CONFIG_STM32_I2CTIMEOTICKS;
#endif

  /* Wait as stop might still be in progress; but stop might also
   * be set because of a timeout error: "The [STOP] bit is set and
   * cleared by software, cleared by hardware when a Stop condition is
   * detected, set by hardware when a timeout error is detected."
   */

  start = clock_systime_ticks();
  do
    {
      /* Calculate the elapsed time */

      elapsed = clock_systime_ticks() - start;

      /* Check for STOP condition */

      cr1 = gd32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET);
      if ((cr1 & I2C_CR1_STOP) == 0)
        {
          return;
        }

      /* Check for timeout error */

      sr1 = gd32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);
      if ((sr1 & I2C_SR1_TIMEOUT) != 0)
        {
          return;
        }
    }

  /* Loop until the stop is complete or a timeout occurs. */

  while (elapsed < timeout);

  /* If we get here then a timeout occurred with the STOP condition
   * still pending.
   */

  i2cinfo("Timeout with CR1: %04" PRIx32 " SR1: %04" PRIx32 "\n", cr1, sr1);
}

/****************************************************************************
 * Name: gd32_i2c_sem_post
 *
 * Description:
 *   Release the mutual exclusion semaphore
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_post(struct gd32_i2c_priv_s *priv)
{
  nxsem_post(&priv->sem_excl);
}

/****************************************************************************
 * Name: gd32_i2c_sem_init
 *
 * Description:
 *   Initialize semaphores
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_init(FAR struct gd32_i2c_priv_s *priv)
{
  nxsem_init(&priv->sem_excl, 0, 1);

#ifndef CONFIG_I2C_POLLED
  /* This semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_init(&priv->sem_isr, 0, 0);
  nxsem_set_protocol(&priv->sem_isr, SEM_PRIO_NONE);
#endif
}

/****************************************************************************
 * Name: gd32_i2c_sem_destroy
 *
 * Description:
 *   Destroy semaphores.
 *
 ****************************************************************************/

static inline void gd32_i2c_sem_destroy(FAR struct gd32_i2c_priv_s *priv)
{
  nxsem_destroy(&priv->sem_excl);
#ifndef CONFIG_I2C_POLLED
  nxsem_destroy(&priv->sem_isr);
#endif
}

/****************************************************************************
 * Name: gd32_i2c_trace*
 *
 * Description:
 *   I2C trace instrumentation
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_TRACE
static void gd32_i2c_traceclear(FAR struct gd32_i2c_priv_s *priv)
{
  struct gd32_trace_s *trace = &priv->trace[priv->tndx];

  trace->status = 0;              /* I2C 32-bit SR2|SR1 status */
  trace->count  = 0;              /* Interrupt count when status change */
  trace->event  = I2CEVENT_NONE;  /* Last event that occurred with this status */
  trace->parm   = 0;              /* Parameter associated with the event */
  trace->time   = 0;              /* Time of first status or event */
}

static void gd32_i2c_tracereset(FAR struct gd32_i2c_priv_s *priv)
{
  /* Reset the trace info for a new data collection */

  priv->tndx       = 0;
  priv->start_time = clock_systime_ticks();
  gd32_i2c_traceclear(priv);
}

static void gd32_i2c_tracenew(FAR struct gd32_i2c_priv_s *priv,
                               uint32_t status)
{
  struct gd32_trace_s *trace = &priv->trace[priv->tndx];

  /* Is the current entry uninitialized?  Has the status changed? */

  if (trace->count == 0 || status != trace->status)
    {
      /* Yes.. Was it the status changed?  */

      if (trace->count != 0)
        {
          /* Yes.. bump up the trace index
           * (unless we are out of trace entries)
           */

          if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
            {
              i2cerr("ERROR: Trace table overflow\n");
              return;
            }

          priv->tndx++;
          trace = &priv->trace[priv->tndx];
        }

      /* Initialize the new trace entry */

      gd32_i2c_traceclear(priv);
      trace->status = status;
      trace->count  = 1;
      trace->time   = clock_systime_ticks();
    }
  else
    {
      /* Just increment the count of times that we have seen this status */

      trace->count++;
    }
}

static void gd32_i2c_traceevent(FAR struct gd32_i2c_priv_s *priv,
                                 enum gd32_trace_e event, uint32_t parm)
{
  struct gd32_trace_s *trace;

  if (event != I2CEVENT_NONE)
    {
      trace = &priv->trace[priv->tndx];

      /* Initialize the new trace entry */

      trace->event  = event;
      trace->parm   = parm;

      /* Bump up the trace index (unless we are out of trace entries) */

      if (priv->tndx >= (CONFIG_I2C_NTRACE - 1))
        {
          i2cerr("ERROR: Trace table overflow\n");
          return;
        }

      priv->tndx++;
      gd32_i2c_traceclear(priv);
    }
}

static void gd32_i2c_tracedump(FAR struct gd32_i2c_priv_s *priv)
{
  struct gd32_trace_s *trace;
  int i;

  syslog(LOG_DEBUG, "Elapsed time: %ld\n",
         (long)(clock_systime_ticks() - priv->start_time));

  for (i = 0; i < priv->tndx; i++)
    {
      trace = &priv->trace[i];
      syslog(LOG_DEBUG,
         "%2d. STATUS: %08x COUNT: %3d EVENT: %s(%2d) PARM: %08x TIME: %d\n",
             i + 1, trace->status, trace->count, g_trace_names[trace->event],
             trace->event, trace->parm, trace->time - priv->start_time);
    }
}
#endif /* CONFIG_I2C_TRACE */

/****************************************************************************
 * Name: gd32_i2c_setclock
 *
 * Description:
 *   Set the I2C clock
 *
 ****************************************************************************/

static void gd32_i2c_setclock(FAR struct gd32_i2c_priv_s *priv,
                               uint32_t frequency)
{
  uint16_t cr1;
  uint16_t ccr;
  uint16_t trise;
  uint16_t freqmhz;
  uint16_t speed;

  /* Has the I2C bus frequency changed? */

  if (frequency != priv->frequency)
    {
      /* Disable the selected I2C peripheral to configure TRISE */

      cr1 = gd32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET);
      gd32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1 & ~I2C_CR1_PE);

      /* Update timing and control registers */

      freqmhz = (uint16_t)(STM32_PCLK1_FREQUENCY / 1000000);
      ccr = 0;

      /* Configure speed in standard mode */

      if (frequency <= 100000)
        {
          /* Standard mode speed calculation */

          speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency << 1));

          /* The CCR fault must be >= 4 */

          if (speed < 4)
            {
              /* Set the minimum allowed value */

              speed = 4;
            }

          ccr |= speed;

          /* Set Maximum Rise Time for standard mode */

          trise = freqmhz + 1;
        }

      /* Configure speed in fast mode */

      else /* (frequency <= 400000) */
        {
          /* Fast mode speed calculation with Tlow/Thigh = 16/9 */

#ifdef CONFIG_STM32_I2C_DUTY16_9
          speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 25));

          /* Set DUTY and fast speed bits */

          ccr |= (I2C_CCR_DUTY | I2C_CCR_FS);
#else
          /* Fast mode speed calculation with Tlow/Thigh = 2 */

          speed = (uint16_t)(STM32_PCLK1_FREQUENCY / (frequency * 3));

          /* Set fast speed bit */

          ccr |= I2C_CCR_FS;
#endif

          /* Verify that the CCR speed value is nonzero */

          if (speed < 1)
            {
              /* Set the minimum allowed value */

              speed = 1;
            }

          ccr |= speed;

          /* Set Maximum Rise Time for fast mode */

          trise = (uint16_t)(((freqmhz * 300) / 1000) + 1);
        }

      /* Write the new values of the CCR and TRISE registers */

      gd32_i2c_putreg(priv, STM32_I2C_CCR_OFFSET, ccr);
      gd32_i2c_putreg(priv, STM32_I2C_TRISE_OFFSET, trise);

      /* Bit 14 of OAR1 must be configured and kept at 1 */

      gd32_i2c_putreg(priv, STM32_I2C_OAR1_OFFSET, I2C_OAR1_ONE);

      /* Re-enable the peripheral (or not) */

      gd32_i2c_putreg(priv, STM32_I2C_CR1_OFFSET, cr1);

      /* Save the new I2C frequency */

      priv->frequency = frequency;
    }
}

/****************************************************************************
 * Name: gd32_i2c_sendstart
 *
 * Description:
 *   Send the START conditions/force Master mode
 *
 ****************************************************************************/

static inline void gd32_i2c_sendstart(FAR struct gd32_i2c_priv_s *priv)
{
  /* Disable ACK on receive by default and generate START */

  gd32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                      I2C_CR1_ACK, I2C_CR1_START);
}

/****************************************************************************
 * Name: gd32_i2c_clrstart
 *
 * Description:
 *   Clear the STOP, START or PEC condition on certain error recovery steps.
 *
 ****************************************************************************/

static inline void gd32_i2c_clrstart(FAR struct gd32_i2c_priv_s *priv)
{
  /* "Note: When the STOP, START or PEC bit is set, the software must
   *  not perform any write access to I2C_CR1 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PEC request."
   *
   * "The [STOP] bit is set and cleared by software, cleared by hardware
   *  when a Stop condition is detected, set by hardware when a timeout
   *  error is detected.
   *
   * "This [START] bit is set and cleared by software and cleared by hardware
   *  when start is sent or PE=0."  The bit must be cleared by software if
   *  the START is never sent.
   *
   * "This [PEC] bit is set and cleared by software, and cleared by hardware
   *  when PEC is transferred or by a START or Stop condition or when PE=0."
   */

  gd32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                      I2C_CR1_START | I2C_CR1_STOP | I2C_CR1_PEC, 0);
}

/****************************************************************************
 * Name: gd32_i2c_sendstop
 *
 * Description:
 *   Send the STOP conditions
 *
 ****************************************************************************/

static inline void gd32_i2c_sendstop(FAR struct gd32_i2c_priv_s *priv)
{
  gd32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, I2C_CR1_ACK, I2C_CR1_STOP);
}

/****************************************************************************
 * Name: gd32_i2c_getstatus
 *
 * Description:
 *   Get 32-bit status (SR1 and SR2 combined)
 *
 ****************************************************************************/

static inline uint32_t gd32_i2c_getstatus(FAR struct gd32_i2c_priv_s *priv)
{
  uint32_t status = gd32_i2c_getreg(priv, STM32_I2C_SR1_OFFSET);
  status |= (gd32_i2c_getreg(priv, STM32_I2C_SR2_OFFSET) << 16);
  return status;
}

/****************************************************************************
 * Name: gd32_i2c_disablefsmc
 *
 * Description:
 *   FSMC must be disable while accessing I2C1 because it uses a common
 *   resource (LBAR)
 *
 *  NOTE:
 *  This is an issue with the STM32F103ZE, but may not be an issue with other
 *  STM32s.  You may need to experiment
 *
 ****************************************************************************/

#ifdef I2C1_FSMC_CONFLICT
static inline
uint32_t gd32_i2c_disablefsmc(FAR struct gd32_i2c_priv_s *priv)
{
  uint32_t ret = 0;
  uint32_t regval;

  /* Is this I2C1 */

#if defined(CONFIG_STM32_I2C2) || defined(CONFIG_STM32_I2C3)
  if (priv->config->base == STM32_I2C1_BASE)
#endif
    {
      /* Disable FSMC unconditionally */

      ret    = getreg32(STM32_RCC_AHBENR);
      regval = ret & ~RCC_AHBENR_FSMCEN;
      putreg32(regval, STM32_RCC_AHBENR);
    }

  return ret;
}

/****************************************************************************
 * Name: gd32_i2c_enablefsmc
 *
 * Description:
 *   Re-enable the FSMC
 *
 ****************************************************************************/

static inline void gd32_i2c_enablefsmc(uint32_t ahbenr)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC only if it was previously enabled. */

  if ((ahbenr & RCC_AHBENR_FSMCEN) != 0)
    {
      regval  = getreg32(STM32_RCC_AHBENR);
      regval |= RCC_AHBENR_FSMCEN;
      putreg32(regval, STM32_RCC_AHBENR);
    }
}
#else
#  define gd32_i2c_disablefsmc(priv) (0)
#  define gd32_i2c_enablefsmc(ahbenr)
#endif /* I2C1_FSMC_CONFLICT */

/****************************************************************************
 * Name: gd32_i2c_isr_process
 *
 * Description:
 *  Common Interrupt Service Routine
 *
 ****************************************************************************/

static int gd32_i2c_isr_process(struct gd32_i2c_priv_s *priv)
{
  uint32_t status = gd32_i2c_getstatus(priv);

  /* Check for new trace setup */

  gd32_i2c_tracenew(priv, status);

  /* Was start bit sent */

  if ((status & I2C_SR1_SB) != 0)
    {
      gd32_i2c_traceevent(priv, I2CEVENT_SENDADDR, priv->msgc);

      /* We check for msgc > 0 here as an unexpected interrupt with
       * I2C_SR1_SB set due to noise on the I2C cable can otherwise cause
       * msgc to wrap causing memory overwrite
       */

      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          /* Get run-time data */

          priv->ptr   = priv->msgv->buffer;
          priv->dcnt  = priv->msgv->length;
          priv->flags = priv->msgv->flags;

          /* Send address byte and define addressing mode */

          gd32_i2c_putreg(priv, STM32_I2C_DR_OFFSET,
                           (priv->flags & I2C_M_TEN) ?
                           0 : ((priv->msgv->addr << 1) |
                           (priv->flags & I2C_M_READ)));

          /* Set ACK for receive mode */

          if (priv->dcnt > 1 && (priv->flags & I2C_M_READ) != 0)
            {
              gd32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                                  0, I2C_CR1_ACK);
            }

          /* Increment to next pointer and decrement message count */

          priv->msgv++;
          priv->msgc--;
        }
      else
        {
          /* Clear ISR by writing to DR register */

          gd32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, 0);
        }
    }

  /* In 10-bit addressing mode, was first byte sent */

  else if ((status & I2C_SR1_ADD10) != 0)
    {
      /* TODO: Finish 10-bit mode addressing.
       *
       * For now just clear ISR by writing to DR register. As we don't do
       * 10 bit addressing this must be a spurious ISR
       */

       gd32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, 0);
    }

  /* Was address sent, continue with either sending or reading data */

  else if ((priv->flags & I2C_M_READ) == 0 &&
           (status & (I2C_SR1_ADDR | I2C_SR1_TXE)) != 0)
    {
      if (priv->dcnt > 0)
        {
          /* Send a byte */

          gd32_i2c_traceevent(priv, I2CEVENT_SENDBYTE, priv->dcnt);
          gd32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, *priv->ptr++);
          priv->dcnt--;
        }
    }

  else if ((priv->flags & I2C_M_READ) != 0 && (status & I2C_SR1_ADDR) != 0)
    {
      /* Enable RxNE and TxE buffers in order to receive one or multiple
       * bytes
       */

#ifndef CONFIG_I2C_POLLED
      gd32_i2c_traceevent(priv, I2CEVENT_ITBUFEN, 0);
      gd32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
#endif
    }

  /* More bytes to read */

  else if ((status & I2C_SR1_RXNE) != 0)
    {
      /* Read a byte, if dcnt goes < 0, then read dummy bytes to ack ISRs */

      if (priv->dcnt > 0)
        {
          gd32_i2c_traceevent(priv, I2CEVENT_RCVBYTE, priv->dcnt);

          /* No interrupts or context switches may occur in the following
           * sequence.  Otherwise, additional bytes may be sent by the
           * device.
           */

#ifdef CONFIG_I2C_POLLED
          irqstate_t flags = enter_critical_section();
#endif
          /* Receive a byte */

          *priv->ptr++ = gd32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);

          /* Disable acknowledge when last byte is to be received */

          priv->dcnt--;
          if (priv->dcnt == 1)
            {
              gd32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET,
                                  I2C_CR1_ACK, 0);
            }

#ifdef CONFIG_I2C_POLLED
          leave_critical_section(flags);
#endif
        }
      else
        {
          /* Throw away the unexpected byte */

          gd32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);
        }
    }
  else if (status & I2C_SR1_TXE)
    {
      /* This should never happen, but it does happen occasionally with lots
       * of noise on the bus. It means the peripheral is expecting more data
       * bytes, but we don't have any to give.
       */

      gd32_i2c_putreg(priv, STM32_I2C_DR_OFFSET, 0);
    }
  else if (status & I2C_SR1_BTF)
    {
      /* We should have handled all cases where this could happen above, but
       * just to ensure it gets ACKed, lets clear it here
       */

      gd32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);
    }
  else if (status & I2C_SR1_STOPF)
    {
      /* We should never get this, as we are a master not a slave. Write CR1
       * with its current value to clear the error
       */

      gd32_i2c_modifyreg(priv, STM32_I2C_CR1_OFFSET, 0, 0);
    }

  /* Do we have more bytes to send, enable/disable buffer interrupts
   * (these ISRs could be replaced by DMAs)
   */

#ifndef CONFIG_I2C_POLLED
  if (priv->dcnt > 0)
    {
      gd32_i2c_traceevent(priv, I2CEVENT_REITBUFEN, 0);
      gd32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, 0, I2C_CR2_ITBUFEN);
    }
  else if (priv->dcnt == 0)
    {
      gd32_i2c_traceevent(priv, I2CEVENT_DISITBUFEN, 0);
      gd32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET, I2C_CR2_ITBUFEN, 0);
    }
#endif

  /* Was last byte received or sent?  Hmmm... the F2 and F4 seems to differ
   * from the F1 in that BTF is not set after data is received (only RXNE).
   */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX) || \
    defined(CONFIG_STM32_STM32L15XX)
  if (priv->dcnt <= 0 && (status & (I2C_SR1_BTF | I2C_SR1_RXNE)) != 0)
#else
  if (priv->dcnt <= 0 && (status & I2C_SR1_BTF) != 0)
#endif
    {
      gd32_i2c_getreg(priv, STM32_I2C_DR_OFFSET);    /* ACK ISR */

      /* Do we need to terminate or restart after this byte?
       * If there are more messages to send, then we may:
       *
       *  - continue with repeated start
       *  - or just continue sending writeable part
       *  - or we close down by sending the stop bit
       */

      if (priv->msgc > 0 && priv->msgv != NULL)
        {
          if (priv->msgv->flags & I2C_M_NOSTART)
            {
              gd32_i2c_traceevent(priv, I2CEVENT_BTFNOSTART, priv->msgc);
              priv->ptr   = priv->msgv->buffer;
              priv->dcnt  = priv->msgv->length;
              priv->flags = priv->msgv->flags;
              priv->msgv++;
              priv->msgc--;

              /* Restart this ISR! */

#ifndef CONFIG_I2C_POLLED
              gd32_i2c_modifyreg(priv, STM32_I2C_CR2_OFFSET,
                                  0, I2C_CR2_ITBUFEN);
#endif
            }
          else
            {
              gd32_i2c_traceevent(priv, I2CEVENT_BTFRESTART, priv->msgc);
              gd32_i2c_sendstart(priv);
            }
        }
      else if (priv->msgv)
        {
          gd32_i2c_traceevent(priv, I2CEVENT_BTFSTOP, 0);
          gd32_i2c_sendstop(priv);

          /* Is there a thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
          if (priv->intstate == INTSTATE_WAITING)
            {
              /* Yes.. inform the thread that the transfer is complete
               * and wake it up.
               */

              nxsem_post(&priv->sem_isr);
              priv->intstate = INTSTATE_DONE;
            }
#else
          priv->intstate = INTSTATE_DONE;
#endif

          /* Mark that we have stopped with this transaction */

          priv->msgv = NULL;
        }
    }

  /* Check for errors, in which case, stop the transfer and return
   * Note that in master reception mode AF becomes set on last byte
   * since ACK is not returned. We should ignore this error.
   */

  if ((status & I2C_SR1_ERRORMASK) != 0)
    {
      gd32_i2c_traceevent(priv, I2CEVENT_ERROR, 0);

      /* Clear interrupt flags */

      gd32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);

      /* Is there a thread waiting for this event (there should be) */

#ifndef CONFIG_I2C_POLLED
      if (priv->intstate == INTSTATE_WAITING)
        {
          /* Yes.. inform the thread that the transfer is complete
           * and wake it up.
           */

          nxsem_post(&priv->sem_isr);
          priv->intstate = INTSTATE_DONE;
        }
#else
      priv->intstate = INTSTATE_DONE;
#endif
    }

  priv->status = status;
  return OK;
}

/****************************************************************************
 * Name: gd32_i2c_isr
 *
 * Description:
 *   Common I2C interrupt service routine
 *
 ****************************************************************************/

#ifndef CONFIG_I2C_POLLED
static int gd32_i2c_isr(int irq, void *context, FAR void *arg)
{
  struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);
  return gd32_i2c_isr_process(priv);
}
#endif

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

static uint32_t _get_i2c_rcu_periph(uint32_t i2c_periph)
{
	switch (i2c_periph)
	{
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

static int gd32_i2c_init(FAR struct gd32_i2c_priv_s *priv)
{

    rcu_periph_clock_enable(_get_i2c_rcu_periph(priv->i2c_periph));
    rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_scl_periph));
    rcu_periph_clock_enable(_get_gpio_rcu_periph(priv->gpio_sda_periph));

    gpio_af_set(priv->gpio_scl_periph, priv->gpio_scl_af, priv->gpio_scl_pin);
    gpio_mode_set(priv->gpio_scl_periph, GPIO_MODE_AF, GPIO_PUPD_PULLUP, priv->gpio_scl_pin);
    gpio_output_options_set(priv->gpio_scl_periph, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, priv->gpio_scl_pin);

    gpio_af_set(priv->gpio_sda_periph, priv->gpio_sda_af, priv->gpio_sda_pin);
    gpio_mode_set(priv->gpio_sda_periph, GPIO_MODE_AF, GPIO_PUPD_PULLUP, priv->gpio_sda_pin);
    gpio_output_options_set(priv->gpio_sda_periph, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, priv->gpio_sda_pin);


#ifndef CONFIG_I2C_POLLED
	irq_attach(priv->ev_irq, gd32_i2c_isr, priv);
	irq_attach(priv->er_irq, gd32_i2c_isr, priv);
	up_enable_irq(priv->ev_irq);
	up_enable_irq(priv->er_irq);
#endif

	i2c_clock_config(priv->i2c_periph, priv->i2c_frquency, I2C_DTCY_2);
	i2c_mode_addr_config(priv->i2c_periph, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C0_OWN_ADDRESS7);
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

static int gd32_i2c_deinit(FAR struct gd32_i2c_priv_s *priv)
{
  i2c_deinit(priv->i2c_periph);


#ifndef CONFIG_I2C_POLLED
  up_disable_irq(priv->config->ev_irq);
  up_disable_irq(priv->config->er_irq);
  irq_detach(priv->config->ev_irq);
  irq_detach(priv->config->er_irq);
#endif

    rcu_periph_clock_disable(_get_i2c_rcu_periph(priv->i2c_periph));
    rcu_periph_clock_disable(_get_gpio_rcu_periph(priv->gpio_scl_periph));
    rcu_periph_clock_disable(_get_gpio_rcu_periph(priv->gpio_sda_periph));

  return OK;
}

/****************************************************************************
 * Device Driver Operations
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int gd32_i2c_transfer(FAR struct i2c_master_s *dev,
                              FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  uint32_t status = 0;
#ifdef I2C1_FSMC_CONFLICT
  uint32_t ahbenr;
#endif
  int ret;

  DEBUGASSERT(count > 0);

  /* Ensure that address or flags don't change meanwhile */

  ret = nxsem_wait(&priv->sem_excl);
  if (ret < 0)
    {
      return ret;
    }

#ifdef I2C1_FSMC_CONFLICT
  /* Disable FSMC that shares a pin with I2C1 (LBAR) */

  ahbenr = gd32_i2c_disablefsmc(priv);

#else
  /* Wait for any STOP in progress.  NOTE:  If we have to disable the FSMC
   * then we cannot do this at the top of the loop, unfortunately.  The STOP
   * will not complete normally if the FSMC is enabled.
   */

  gd32_i2c_sem_waitstop(priv);
#endif

  /* Clear any pending error interrupts */

  gd32_i2c_putreg(priv, STM32_I2C_SR1_OFFSET, 0);

  /* "Note: When the STOP, START or PEC bit is set, the software must
   *  not perform any write access to I2C_CR1 before this bit is
   *  cleared by hardware. Otherwise there is a risk of setting a
   *  second STOP, START or PEC request."  However, if the bits are
   *  not cleared by hardware, then we will have to do that from hardware.
   */

  gd32_i2c_clrstart(priv);

  /* Old transfers are done */

  /* Reset ptr and dcnt to ensure an unexpected data interrupt doesn't
   * overwrite stale data.
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  priv->msgv = msgs;
  priv->msgc = count;

  /* Reset I2C trace logic */

  gd32_i2c_tracereset(priv);

  /* Set I2C clock frequency (on change it toggles I2C_CR1_PE !)
   * REVISIT: Note that the frequency is set only on the first message.
   * This could be extended to support different transfer frequencies for
   * each message segment.
   */

  gd32_i2c_setclock(priv, msgs->frequency);

  /* Trigger start condition, then the process moves into the ISR.  I2C
   * interrupts will be enabled within gd32_i2c_waitdone().
   */

  priv->status = 0;
  gd32_i2c_sendstart(priv);

  /* Wait for an ISR, if there was a timeout, fetch latest status to get
   * the BUSY flag.
   */

  if (gd32_i2c_sem_waitdone(priv) < 0)
    {
      status = gd32_i2c_getstatus(priv);
      ret = -ETIMEDOUT;

      i2cerr("ERROR: Timed out: CR1: 0x%04x status: 0x%08" PRIx32 "\n",
             gd32_i2c_getreg(priv, STM32_I2C_CR1_OFFSET), status);

      /* "Note: When the STOP, START or PEC bit is set, the software must
       *  not perform any write access to I2C_CR1 before this bit is
       *  cleared by hardware. Otherwise there is a risk of setting a
       *  second STOP, START or PEC request."
       */

      gd32_i2c_clrstart(priv);

      /* Clear busy flag in case of timeout */

      status = priv->status & 0xffff;
    }
  else
    {
      /* clear SR2 (BUSY flag) as we've done successfully */

      status = priv->status & 0xffff;
    }

  /* Check for error status conditions */

  if ((status & I2C_SR1_ERRORMASK) != 0)
    {
      /* I2C_SR1_ERRORMASK is the 'OR' of the following individual bits: */

      if (status & I2C_SR1_BERR)
        {
          /* Bus Error */

          ret = -EIO;
        }
      else if (status & I2C_SR1_ARLO)
        {
          /* Arbitration Lost (master mode) */

          ret = -EAGAIN;
        }
      else if (status & I2C_SR1_AF)
        {
          /* Acknowledge Failure */

          ret = -ENXIO;
        }
      else if (status & I2C_SR1_OVR)
        {
          /* Overrun/Underrun */

          ret = -EIO;
        }
      else if (status & I2C_SR1_PECERR)
        {
          /* PEC Error in reception */

          ret = -EPROTO;
        }
      else if (status & I2C_SR1_TIMEOUT)
        {
          /* Timeout or Tlow Error */

          ret = -ETIME;
        }

      /* This is not an error and should never happen since SMBus is not
       * enabled
       */

      else /* if (status & I2C_SR1_SMBALERT) */
        {
          /* SMBus alert is an optional signal with an interrupt line for
           * devices that want to trade their ability to master for a pin.
           */

          ret = -EINTR;
        }
    }

  /* This is not an error, but should not happen.  The BUSY signal can hang,
   * however, if there are unhealthy devices on the bus that need to be
   * reset.
   * NOTE:  We will only see this busy indication if gd32_i2c_sem_waitdone()
   * fails above;  Otherwise it is cleared.
   */

  else if ((status & (I2C_SR2_BUSY << 16)) != 0)
    {
      /* I2C Bus is for some reason busy */

      ret = -EBUSY;
    }

  /* Dump the trace result */

  gd32_i2c_tracedump(priv);

#ifdef I2C1_FSMC_CONFLICT
  /* Wait for any STOP in progress.  NOTE:  If we have to disable the FSMC
   * then we cannot do this at the top of the loop, unfortunately.  The STOP
   * will not complete normally if the FSMC is enabled.
   */

  gd32_i2c_sem_waitstop(priv);

  /* Re-enable the FSMC */

  gd32_i2c_enablefsmc(ahbenr);
#endif

  /* Ensure that any ISR happening after we finish can't overwrite any user
   * data
   */

  priv->dcnt = 0;
  priv->ptr = NULL;

  gd32_i2c_sem_post(priv);
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

FAR struct i2c_master_s *gd32_i2cbus_initialize(int port)
{
  struct gd32_i2c_priv_s * priv = NULL;
  irqstate_t flags;

  /* Get I2C private structure */

  switch (port)
    {
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

  if ((volatile int)priv->refs++ == 0)
    {
      gd32_i2c_sem_init(priv);
      gd32_i2c_init(priv);
    }

  leave_critical_section(flags);
  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: gd32_i2cbus_uninitialize
 *
 * Description:
 *   Uninitialize an I2C bus
 *
 ****************************************************************************/

int gd32_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct gd32_i2c_priv_s *priv = (struct gd32_i2c_priv_s *)dev;
  irqstate_t flags;

  DEBUGASSERT(dev);

  /* Decrement reference count and check for underflow */

  if (priv->refs == 0)
    {
      return ERROR;
    }

  flags = enter_critical_section();

  if (--priv->refs)
    {
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
