/*
 * SPDX-FileCopyrightText: 2026 HAW Hamburg
 * SPDX-License-Identifier: LGPL-2.1-only
 */

/**
 * @ingroup     cpu_stm32
 * @{
 *
 * @file
 * @brief       Implementation of the peripheral PDM interface using the DFSDM
 *              peripheral
 *
 * @author      Ben Wehrberger <ben.wehrberger@haw-hamburg.de>
 *
 * @}
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>

#include "container.h"
#include "periph_conf.h"
#include "periph_cpu.h"
#include "periph/gpio.h"
#include "periph/pdm.h"

#define ENABLE_DEBUG 1
#include "debug.h"

/* Sinc3. FOSR_MAX depends on the filter order (RM0394 21.4.8), change both together */
#define SINC_ORDER 3

/* maybe remove this later, after trying out SINC 3-5, no need if only sinc 3 is applicable */
#define FOSR_MAX 1024

/* The samples buffer is a double buffer */
static int16_t _pdm_buf[PDM_BUF_SIZE * 2] = { 0 };

static pdm_isr_ctx_t _isr_ctx;

static bool _pdm_running = false;

static inline DFSDM_Channel_TypeDef *_channel(unsigned chan)
{
    return (DFSDM_Channel_TypeDef *)(DFSDM1_Channel0_BASE + chan *
            (DFSDM1_Channel1_BASE - DFSDM1_Channel0_BASE));
}

static void _dma_cb(void *arg)
{
    (void)arg;

    /* remaining counts down, more than half left means DMA is still in first
     * half, so second half is ready */
    uint8_t idx = dma_get_remaining(pdm_config.dma) > PDM_BUF_SIZE ? 1 : 0;
    _isr_ctx.cb(_isr_ctx.arg, &_pdm_buf[PDM_BUF_SIZE * idx]);
}

static void _start_dma(void)
{
    dma_acquire(pdm_config.dma);
    dma_setup(pdm_config.dma,
              pdm_config.dma_chan,
              (void *)((uintptr_t)&DFSDM1_Filter0->FLTRDATAR + 2),
              DMA_PERIPH_TO_MEM, DMA_DATA_WIDTH_HALF_WORD,
              false);
    dma_prepare(pdm_config.dma, &_pdm_buf, ARRAY_SIZE(_pdm_buf), true);
    dma_enable_loop(pdm_config.dma);
    dma_set_cb(pdm_config.dma, &_dma_cb, NULL);
    dma_start(pdm_config.dma);
}

static void _stop_dma(void)
{
    dma_stop(pdm_config.dma);
    dma_disable_loop(pdm_config.dma);
    dma_release(pdm_config.dma);
}

static void _enable_dfsdm(void)
{
    /* enable DFSDM, channel and filter, then start continuous conversion */
    DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN;
    _channel(pdm_config.chan)->CHCFGR1 |= DFSDM_CHCFGR1_CHEN;
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_DFEN;
    DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;
}

static void _disable_dfsdm(void)
{
    /* disable DFSDM, channel and filter */
    DFSDM1_Filter0->FLTCR1 &= ~DFSDM_FLTCR1_DFEN;
    _channel(pdm_config.chan)->CHCFGR1 &= ~DFSDM_CHCFGR1_CHEN;
    DFSDM1_Channel0->CHCFGR1 &= ~DFSDM_CHCFGR1_DFSDMEN;
}

static inline void _debug(uint8_t chan)
{
    DEBUG("[pdm] RCC->APB2ENR:   0x%08lx\n", (unsigned long)RCC->APB2ENR);
    DEBUG("[pdm] GPIOC->MODER:   0x%08lx\n", (unsigned long)GPIOC->MODER);
    DEBUG("[pdm] GPIOC->AFR[0]:  0x%08lx\n", (unsigned long)GPIOC->AFR[0]);
    DEBUG("[pdm] GPIOB->MODER:   0x%08lx\n", (unsigned long)GPIOB->MODER);
    DEBUG("[pdm] GPIOB->AFR[1]:  0x%08lx\n", (unsigned long)GPIOB->AFR[1]);
    DEBUG("[pdm] CH0CFGR1:       0x%08lx\n",
          (unsigned long)DFSDM1_Channel0->CHCFGR1);
    DEBUG("[pdm] CH%u CFGR1:      0x%08lx\n", chan,
          (unsigned long)_channel(chan)->CHCFGR1);
    DEBUG("[pdm] FLT0 FLTCR1:    0x%08lx\n",
          (unsigned long)DFSDM1_Filter0->FLTCR1);
    DEBUG("[pdm] FLT0 FLTFCR:    0x%08lx\n",
          (unsigned long)DFSDM1_Filter0->FLTFCR);

    DEBUG("[pdm] DFSDM1EN: %u (expect 1)\n",
          (unsigned)((RCC->APB2ENR & RCC_APB2ENR_DFSDM1EN) != 0));
    DEBUG("[pdm] PC2 mode: %lu (expect 2 = AF), AF: %lu (expect 6)\n",
          (unsigned long)((GPIOC->MODER >> (2 * 2)) & 0x3),
          (unsigned long)((GPIOC->AFR[0] >> (2 * 4)) & 0xf));
    DEBUG("[pdm] PB12 mode: %lu (expect 2 = AF), AF: %lu (expect 6)\n",
          (unsigned long)((GPIOB->MODER >> (12 * 2)) & 0x3),
          (unsigned long)((GPIOB->AFR[1] >> ((12 - 8) * 4)) & 0xf));
    DEBUG("[pdm] CH0CFGR1: 0x%08lx (expect 0x00130000)\n",
          (unsigned long)DFSDM1_Channel0->CHCFGR1);
    DEBUG("[pdm] CH%u CFGR1: 0x%08lx (expect 0x00000005)\n",
          chan, (unsigned long)_channel(chan)->CHCFGR1);
    DEBUG("[pdm] FLTCR1: 0x%08lx (expect 0x21040000)\n",
          (unsigned long)DFSDM1_Filter0->FLTCR1);
    DEBUG("[pdm] FLTFCR: 0x%08lx (expect 0x607C0000)\n",
          (unsigned long)DFSDM1_Filter0->FLTFCR);
}

int32_t pdm_init(pdm_mode_t mode, uint32_t rate, int8_t gain,
                 pdm_data_cb_t cb, void *arg)
{
    assert(pdm_config.chan < 4);

    uint16_t divisor = 20;
    /* FOSR can be 0-1023 (dec_rat = 1-1024), prob need a stricter range for 
     * audio when calc sample rate*/
    uint16_t dec_rat = 125;

    /* Later compute from dec_rat. Filter output grows to about dec_rat^SINC_ORDER,
     * must fit 24 bit: shift = max(0, SINC_ORDER * log2(dec_rat) + 1 - 24)
     * RM0394 21.4.13, table 105  */
    uint8_t rshift = 0;

    /* Configure mode (Mono or Stereo) */
    switch (mode) {
    case PDM_MODE_MONO:
        break;
    case PDM_MODE_STEREO:
        return -ENOTSUP;
        break;
    default:
        DEBUG("[pdm] init: mode not supported\n");
        return -ENOTSUP;
    }

    /* use PCLK2 as DFSDM clock source */
    /* DFSDM clock has to be 4x min of the mic clk */
    RCC->CCIPR &= ~RCC_CCIPR_DFSDM1SEL;
    periph_clk_en(APB2, RCC_APB2ENR_DFSDM1EN);

    /* configure CLK and DIN pins */
    gpio_init(pdm_config.din_pin, GPIO_IN);
    gpio_init_af(pdm_config.din_pin, pdm_config.din_af);
    gpio_init(pdm_config.clk_pin, GPIO_OUT);
    gpio_init_af(pdm_config.clk_pin, pdm_config.clk_af);

    /* reset channel 0 and disable DFSDM */
    DFSDM1_Channel0->CHCFGR1 = 0;
    /* CKOUT = f_DFSDM / (CKOUTDIV + 1) */
    DFSDM1_Channel0->CHCFGR1 = (divisor - 1) << DFSDM_CHCFGR1_CKOUTDIV_Pos;

    DFSDM_Channel_TypeDef *chan = _channel(pdm_config.chan);

    /* reset channel */
    chan->CHCFGR1 = 0;
    chan->CHCFGR2 = 0;

    /* sample on falling CKOUT edge */
    chan->CHCFGR1 |= DFSDM_CHCFGR1_SITP_0 | DFSDM_CHCFGR1_SPICKSEL_0;

    /* Maybe try later to  calibrate OFFSET once after start (average samples in the
    * discard window of pdm_start) to remove the mic DC offset, like nrf does */
    /* right shift so the filter output fits the 24 bit result register, OFFSET = 0 */
    chan->CHCFGR2 |= rshift << DFSDM_CHCFGR2_DTRBS_Pos;


    /* read channel continuously, FAST keeps the filter state between samples,
     * enable DMA channel */
    DFSDM1_Filter0->FLTCR1 = 0;
    DFSDM1_Filter0->FLTCR1 |= (pdm_config.chan << DFSDM_FLTCR1_RCH_Pos) |
                              DFSDM_FLTCR1_RCONT |
                              DFSDM_FLTCR1_FAST |
                              DFSDM_FLTCR1_RDMAEN;

    /* FOSR is dec ratio, FORD is Sinc order (1-5):
       Sinc3 (= 3 moving averages in a row, removes the PDM noise can try out 
       later 3-5, maybe look app note */
    DFSDM1_Filter0->FLTFCR = (dec_rat - 1) << DFSDM_FLTFCR_FOSR_Pos |
                             SINC_ORDER << DFSDM_FLTFCR_FORD_Pos;

    _isr_ctx.cb = cb;
    _isr_ctx.arg = arg;

    (void)rate;
    (void)gain;

    //_debug(pdm_config.chan);
    periph_clk_dis(APB2, RCC_APB2ENR_DFSDM1EN);
    return CLOCK_APB2 / divisor / dec_rat;
}

void pdm_start(void)
{
    if (_pdm_running) {
        return;
    }
    periph_clk_en(APB2, RCC_APB2ENR_DFSDM1EN);
    _start_dma();
    _enable_dfsdm();
    _pdm_running = true;
}

void pdm_stop(void)
{
    if (!_pdm_running) {
        return;
    }
    _disable_dfsdm();
    _stop_dma();
    periph_clk_dis(APB2, RCC_APB2ENR_DFSDM1EN);
    _pdm_running = false;
}

void pdm_deinit(void)
{
    pdm_stop();
}
