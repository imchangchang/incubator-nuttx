/****************************************************************************
 * boards/arm/gd32f4xx/gd32f450v_start/include/board.h
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

#ifndef __BOARDS_ARM_GD32F4XX_GD32F450V_START_INCLUDE_BOARD_H
#define __BOARDS_ARM_GD32F4XX_GD32F450V_START_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* From system_gd32f4xx.h */
#define __SYSTEM_CLOCK_200M_PLL_25M_HXTAL         (uint32_t)(200000000)

/* console */
#define GPIO_UART0_RX_PORT GPIOB
#define GPIO_UART0_RX_PIN  GPIO_PIN_3
#define GPIO_UART0_RX_AF   GPIO_AF_7
#define GPIO_UART0_TX_PORT GPIOA
#define GPIO_UART0_TX_PIN  GPIO_PIN_15
#define GPIO_UART0_TX_AF   GPIO_AF_7

/* SBUS */
#define GPIO_UART1_RX_PORT GPIOD
#define GPIO_UART1_RX_PIN  GPIO_PIN_6
#define GPIO_UART1_RX_AF   GPIO_AF_7
#define GPIO_UART1_TX_PORT GPIOD
#define GPIO_UART1_TX_PIN  GPIO_PIN_5
#define GPIO_UART1_TX_AF   GPIO_AF_7


/* AP */
#define GPIO_UART2_RX_PORT GPIOC
#define GPIO_UART2_RX_PIN  GPIO_PIN_10
#define GPIO_UART2_RX_AF   GPIO_AF_7
#define GPIO_UART2_TX_PORT GPIOC
#define GPIO_UART2_TX_PIN  GPIO_PIN_11
#define GPIO_UART2_TX_AF   GPIO_AF_7


#endif /* __BOARDS_ARM_GD32F4XX_GD32F450V_START_INCLUDE_BOARD_H */
