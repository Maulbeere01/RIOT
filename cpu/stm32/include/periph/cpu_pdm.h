/*
 * SPDX-FileCopyrightText: 2026 HAW Hamburg
 * SPDX-License-Identifier: LGPL-2.1-only
 */

#pragma once

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       PDM CPU specific definitions for the STM32 family
 *
 * @author      Ben Wehrberger <ben.wehrberger@haw-hamburg.de>
 */

#include "periph/cpu_gpio.h"
#include "periph/cpu_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DFSDM1_BASE
/**
 * @brief   Structure for PDM configuration data
 *
 * On STM32, PDM is implemented with the DFSDM (Digital Filter for Sigma-Delta
 * Modulators) peripheral.
 *
 * @note    A board providing periph_pdm must also define PDM_MIC_CLK_MIN_HZ and
 *          PDM_MIC_CLK_MAX_HZ, matching the normal operating clock range of the
 *          used microphone (in Hz), as given by the microphone's datasheet.
 */
typedef struct {
    gpio_t din_pin;            /**< DIN pin */
    gpio_t clk_pin;            /**< CLK pin */
    gpio_af_t din_af;          /**< DIN pin alternate function */
    gpio_af_t clk_af;          /**< CLK pin alternate function */
    uint8_t chan;              /**< DFSDM channel connected to the DIN pin */
    dma_t dma;                 /**< Logical DMA stream */
    uint8_t dma_chan;          /**< DMA channel */
} pdm_conf_t;
#endif

#ifdef __cplusplus
}
#endif

/** @} */
