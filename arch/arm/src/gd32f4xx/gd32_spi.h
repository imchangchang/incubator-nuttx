/****************************************************************************
 * arch/arm/src/gd32f4xx/gd32_spi.h
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

#ifndef __ARCH_ARM_SRC_GD32_GD32_SPI_H
#define __ARCH_ARM_SRC_GD32_GD32_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/spi.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct spi_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gd32_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   bus number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *gd32_spibus_initialize(int bus);

/****************************************************************************
 * Name:  gd32_spi0/1/...select and gd32_spi0/1/...status
 *
 * Description:
 *   The external functions, gd32_spi0/1/...select, gd32_spi0/1/...status,
 *   and gd32_spi0/1/...cmddata must be provided by board-specific logic.
 *   These are implementations of the select, status, and cmddata methods of
 *   the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi/spi.h). All other methods (including
 *   gd32_spibus_initialize()) are provided by common GD32 logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in gd32_boardinitialize() to configure SPI chip
 *      select pins.
 *   2. Provide gd32_spi0/1/...select() and gd32_spi0/1/...status()
 *      functions in your board-specific logic.  These functions will
 *      perform chip selection and status operations using GPIOs in the way
 *      your board is configured.
 *   3. If CONFIG_SPI_CMDDATA is defined in your NuttX configuration file,
 *      then provide gd32_spi0/1/...cmddata() functions in your board-
 *      specific logic. These functions will perform cmd/data selection
 *      operations using GPIOs in the way your board is configured.
 *   4. Add a calls to gd32_spibus_initialize() in your low level
 *      application initialization logic
 *   5. The handle returned by gd32_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_GD32_SPI0
void gd32_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t gd32_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
int gd32_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif


#ifdef CONFIG_GD32_SPI1
void gd32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t gd32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
int gd32_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_GD32_SPI2
void gd32_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t gd32_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
int gd32_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_GD32_SPI3
void gd32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t gd32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
int gd32_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_GD32_SPI4
void gd32_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t gd32_spi4status(FAR struct spi_dev_s *dev, uint32_t devid);
int gd32_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#ifdef CONFIG_GD32_SPI5
void gd32_spi5select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected);
uint8_t gd32_spi5status(FAR struct spi_dev_s *dev, uint32_t devid);
int gd32_spi5cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

/****************************************************************************
 * Name: gd32_spi0/1/...register
 *
 * Description:
 *   If the board supports a card detect callback to inform the SPI-based
 *   MMC/SD driver when an SD card is inserted or removed, then
 *   CONFIG_SPI_CALLBACK should be defined and the following function(s)
 *   must be implemented.  These functions implements the registercallback
 *   method of the SPI interface (see include/nuttx/spi/spi.h for details)
 *
 * Input Parameters:
 *   dev -      Device-specific state data
 *   callback - The function to call on the media change
 *   arg -      A caller provided value to return with the callback
 *
 * Returned Value:
 *   0 on success; negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CALLBACK

#ifdef CONFIG_GD32_SPI0
int gd32_spi0register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif
#ifdef CONFIG_GD32_SPI1
int gd32_spi1register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_GD32_SPI2
int gd32_spi2register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_GD32_SPI3
int gd32_spi3register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_GD32_SPI4
int gd32_spi4register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif

#ifdef CONFIG_GD32_SPI5
int gd32_spi5register(FAR struct spi_dev_s *dev, spi_mediachange_t callback,
                       FAR void *arg);
#endif


#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_GD32_GD32_SPI_H */
