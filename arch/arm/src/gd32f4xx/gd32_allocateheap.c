/****************************************************************************
 * arch/arm/src/stm32/stm32_allocateheap.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>

#include <arch/board/board.h>

#include "chip.h"

#include "mpu.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "gd32f4xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifdef GD32F450
#define SRAM0_START 0x20000000
#define SRAM0_END   0x2001bFFF
#define SRAM1_START 0x2001C000
#define SRAM1_END   0x2001FFFF
#define SRAM2_START 0x2002FFFF
#define SRAM2_END   0x2002FFFF
#endif

#define HEAP_END SRAM0_END

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_heap_color
 *
 * Description:
 *   Set heap memory to a known, non-zero state to checking heap usage.
 *
 ****************************************************************************/

#ifdef CONFIG_HEAP_COLORATION
static inline void up_heap_color(FAR void *start, size_t size)
{
  memset(start, HEAP_COLOR, size);
}
#else
#  define up_heap_color(start,size)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 *   The following memory map is assumed for the flat build:
 *
 *     .data region.  Size determined at link time.
 *     .bss  region  Size determined at link time.
 *     IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Heap.  Extends to the end of SRAM.
 *
 *   The following memory map is assumed for the kernel build:
 *
 *     Kernel .data region       Size determined at link time
 *     Kernel .bss  region       Size determined at link time
 *     Kernel IDLE thread stack  Size determined by
 *                                CONFIG_IDLETHREAD_STACKSIZE
 *     Padding for alignment
 *     User .data region         Size determined at link time
 *     User .bss region          Size determined at link time
 *     Kernel heap               Size determined by
 *                                CONFIG_MM_KERNEL_HEAPSIZE
 *     User heap                 Extends to the end of SRAM
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
  /* Return the heap settings */
  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
  *heap_size  = HEAP_END - g_idle_topstack;

  /* Colorize the heap for debug */
  _info("%x %x %d\n", (uintptr_t)(*heap_start),HEAP_END,*heap_size);

  up_heap_color(*heap_start, *heap_size);
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel-space heap.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM1_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM1_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM1_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((SRAM1_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM1_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (FAR void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

/****************************************************************************
 * Name: arm_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
#ifndef CONFIG_STM32_CCMEXCLUDE
  /* Colorize the heap for debug */

  up_heap_color((FAR void *)SRAM2_START, SRAM2_END - SRAM2_START);

  /* Add the STM32F20xxx/STM32F40xxx CCM SRAM user heap region. */

  kumm_addregion((FAR void *)SRAM2_START, SRAM2_END - SRAM2_START);
#endif

}
#endif
