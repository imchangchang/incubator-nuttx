
#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#include <nuttx/init.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "nvic.h"
#include "barriers.h"

#include "system_gd32f4xx.h"
#include "gd32f4xx_gpio.h"

#define IDLE_STACKSIZE (CONFIG_IDLETHREAD_STACKSIZE & ~7)
#define IDLE_STACK     ((uintptr_t)&_ebss + IDLE_STACKSIZE)
#define HEAP_BASE      ((uintptr_t)&_ebss + IDLE_STACKSIZE)

const uintptr_t g_idle_topstack = HEAP_BASE;

void __start(void)
{
	const uint32_t *src;
	uint32_t *dest;

  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile("sub r10, sp, %0" : :
                   "r"(CONFIG_IDLETHREAD_STACKSIZE - 64) :);

	SystemInit();

	for (dest = _START_BSS; dest < _END_BSS; )
	{
		*dest++ = 0;
	}

    for (src = _DATA_INIT, dest = _START_DATA; dest < _END_DATA; )
	{
	  *dest++ = *src++;
	}

    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_mode_set(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ, GPIO_PIN_6);
    gpio_bit_write(GPIOC, GPIO_PIN_6, SET);
    arm_serialinit();
    nx_start();
    for(;;);
}
