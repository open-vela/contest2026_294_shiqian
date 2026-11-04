/****************************************************************************
 * arch/arm/src/stm32n6/stm32n6_sai.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version
 * 2.0 (the "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 * STM32N6 SAI driver for NuttX.
 * Serial Audio Interface for microphone input (MEMS mic via I2S).
 *
 * Adapted from STM32F7 NuttX reference (stm32_sai.c).
 * Reference: STM32N6 HAL (stm32n6xx_hal_sai.c)
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <stdio.h>
#include <syslog.h>
#include <string.h>

#include "arm_internal.h"
#include "stm32n6_dma.h"
#include "stm32n6_gpio.h"
#include "stm32n6_sai.h"

/* memorymap first: the RCC header derives every register address from
 * STM32_RCC_BASE, which that header owns.
 */

#include "hardware/stm32_memorymap.h"
#include "hardware/stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SAI1 base address (per CMSIS stm32n647xx.h). The driver's register
 * offsets below (SAI_CR1_OFFSET=0x04, etc.) are relative to this base
 * and already encode the CMSIS Block A control-register placement
 * (SAI_Block_TypeDef starting at SAI1_BASE + 0x04), matching the
 * Renode STM32N6_SAI model's absolute offsets from the sai1 base.
 */

#define STM32N6_SAI1_BASE   0x42005800

/* SAI register offsets (per block: 0x20 spacing) */

#define SAI_GCR_OFFSET      0x00
#define SAI_CR1_OFFSET      0x04
#define SAI_CR2_OFFSET      0x08
#define SAI_FRCR_OFFSET     0x0C
#define SAI_SLOTR_OFFSET    0x10
#define SAI_IMR_OFFSET      0x14
#define SAI_SR_OFFSET       0x18
#define SAI_CLRFR_OFFSET    0x1C
#define SAI_DR_OFFSET       0x20
#define SAI_PDMCR_OFFSET    0x44

/* GPIO registers needed for the pin check.  The port keeps these private to
 * its own GPIO driver; a diagnostic that has to look at the mode and the
 * alternate-function number spells them out rather than widening that
 * header for one caller.
 */

#define SAI_GPIO_MODER_OFFSET 0x00
#define SAI_GPIO_IDR_OFFSET   0x10
#define SAI_GPIO_AFRL_OFFSET  0x20
#define SAI_GPIO_AFRH_OFFSET  0x24

/* SAI_CR1 bits.  Positions checked against the block's own definitions in
 * STM32Cube_FW_N6's stm32n647xx.h (SAI_xCR1_*_Pos), not carried over from
 * another part's driver: two fields in this file had been transcribed wrong
 * and both are load-bearing.
 */

#define SAI_CR1_SAIEN       (1 << 16)
#define SAI_CR1_DMAEN       (1 << 17)
#define SAI_CR1_MODE_MASK   (3 << 0)
#define SAI_CR1_MODE_MASTER_TX (0 << 0)
#define SAI_CR1_MODE_MASTER_RX (1 << 0)  /* was (2 << 0), which is SLAVE_TX */
#define SAI_CR1_MODE_SLAVE_TX  (2 << 0)
#define SAI_CR1_MODE_SLAVE_RX  (3 << 0)
#define SAI_CR1_MODE_RX     SAI_CR1_MODE_MASTER_RX
#define SAI_CR1_DS_MASK     (7 << 5)
#define SAI_CR1_DS_16BIT    (4 << 5)
#define SAI_CR1_DS_24BIT    (6 << 5)
#define SAI_CR1_LSBFIRST    (1 << 8)     /* was (1 << 12), which is MONO */
#define SAI_CR1_MONO        (1 << 12)
#define SAI_CR1_CKSTR       (1 << 9)
#define SAI_CR1_NODIV       (1 << 19)
#define SAI_CR1_MCKDIV_MASK (0x3F << 20)

/* SAI_FRCR bits (frame control) */

#define SAI_FRCR_FRL_MASK   (0xFF << 0)  /* Frame length */
#define SAI_FRCR_FSALL_MASK (0x3F << 8)  /* FS active level length */

/* SAI_SLOTR bits */

#define SAI_SLOTR_FBOFF_MASK (0x1F << 0)
#define SAI_SLOTR_SLOTSZ_MASK (3 << 6)
#define SAI_SLOTR_SLOTSZ_16B  (1 << 6)  /* one 16-bit sample per slot */
#define SAI_SLOTR_NBSLOT_MASK (0xF << 8)
#define SAI_SLOTR_SLOTEN_MASK (0xFFFF << 16)

/* SAI status bits */

#define SAI_SR_FREQ         (1 << 3)
#define SAI_SR_OVRUDR       (1 << 0)

/* Audio constants */

#define SAI_DEFAULT_MCLK_DIV  4  /* MCLK = HCLK / 4 */

/* --- playback ------------------------------------------------------------ */

/* Register fields the input path never touched.  Transcribed from the SAI
 * block's definitions rather than guessed: CKSTR selects which edge data
 * changes on, MCKEN drives MCLK out to the codec, and FSDEF/FSOFF describe
 * how the frame sync relates to the first bit.
 */

#define SAI_CR1_CKSTR        (1 << 9)    /* data changes on falling edge (I2S) */
#define SAI_CR1_OUTDRIV     (1 << 13)    /* output drive enabled */
#define SAI_CR1_SAIEN       (1 << 16)    /* block enable */
#define SAI_CR1_MCKEN       (1 << 27)    /* master clock generation + output */
#define SAI_FRCR_FSDEF      (1 << 16)    /* FS = channel identification */
#define SAI_FRCR_FSPOL      (1 << 17)    /* FS active low */
#define SAI_FRCR_FSOFF      (1 << 18)    /* FS starts before the first bit */
#define SAI_CR2_FTH_1QF     (1 << 0)     /* FIFO threshold 1/4 */
#define SAI_CR2_FFLUSH      (1 << 3)     /* flush the FIFO, self-clearing */
#define SAI_CR1_MCKDIV(d)   (((d) & 0x3f) << 20)  /* master clock divider */

#define SAI_TX_FRAME_BITS  32            /* 16-bit sample x 2 slots */
#define SAI_TX_ACTIVE_BITS 16
#define SAI_TX_SLOTS       2

#define SAI_TX_DMA_CHAN    1
#define SAI_TX_DMA_REQUEST 91            /* GPDMA1 request line: SAI1_A */

/* RCC bits this path needs.  The port's RCC header lists IC1/2/3/6/11/16/17/18
 * because those are the ones existing drivers use; SAI1 hangs off IC7, which
 * was not among them.  Offsets are the CMSIS struct's.
 */

#define STM32N6_RCC_IC7CFGR      (STM32_RCC_BASE + 0x00dc)

/* PLL reference-source encodings.  The port's RCC header has
 * RCC_PLL1CFGR1_SEL_HSE as (1 << 28), but the vendor HAL writes (2 << 28)
 * for HSE and uses (1 << 28) for MSI, with 0 meaning HSI.  PLL1 on this board
 * reads back SEL = 0 and runs at 64/4*75 = 1200 MHz, which confirms 0 is HSI
 * and therefore that the header's value is wrong.
 */

#define RCC_PLLSEL_HSI           (0 << 28)
#define RCC_PLLSEL_HSE           (2 << 28)
#define RCC_PLL2CFGR1_SEL_MASK   (7 << 28)
#define RCC_PLL2CFGR1_BYP        (1 << 27)
#define RCC_PLL2CFGR1_DIVM_MASK  (0x3f << 20)
#define RCC_PLL2CFGR1_DIVN_MASK  (0xfff << 8)
#define RCC_PLL2CFGR3_PDIV1_MASK (7 << 27)
#define RCC_PLL2CFGR3_PDIV2_MASK (7 << 24)
#define RCC_PLL2CFGR2_FRAC_MASK  (0xffffff)

/* PLL2CFGR3 mode bits.  PLL2CFGR3 shares PLL1CFGR3's layout, so the two
 * names the port header already has are reused and only the rest are added.
 *
 * PDIVEN is the one that matters here.  The vendor HAL sets it together with
 * MODSSRST before switching the PLL on, and its comment says what it does:
 * "Enable PLLx post divider output".  Leaving it clear lets the VCO lock -
 * PLL2RDY reads 1 - while nothing leaves the post dividers, so IC7 is fed no
 * clock at all.  That is precisely what the board showed: pll2 on=1 ready=1
 * with ic7cfgr and DIVENR correct, ic7_ck never producing a frame, and the
 * always-on sources (pclk2, per_ck, hsi_div_ck) working.
 */

#define RCC_PLL2CFGR3_PDIVEN     RCC_PLL1CFGR3_PDIVEN
#define RCC_PLL2CFGR3_MODSSDIS   RCC_PLL1CFGR3_MODSSDIS
#define RCC_PLL2CFGR3_MODSSRST   (1 << 0)   /* modulation spread-spectrum reset */
#define RCC_PLL2CFGR3_DACEN      (1 << 1)   /* noise cancelling DAC (frac mode) */
#define RCC_PLL2CFGR3_MODDSEN    (1 << 3)   /* modulation / fractional enable     */

/* The port's RCC header defines PLL2CFGR1 and PLL2CFGR3 but not CFGR2, which
 * holds the fractional part of DIVN.  Playback runs in integer mode and only
 * ever clears it, so it is spelled out here rather than added to the shared
 * header for a single zero write.
 */

#define STM32N6_RCC_PLL2CFGR2    (STM32_RCC_BASE + 0x0094)
#define RCC_DIVENR_IC7EN         (1 << 6)
#define RCC_APB2ENR_SAI1EN       (1 << 21)

/* SAI1 kernel clock selection, RM0486 "RCC kernel clock register 7".
 *
 * APB2ENR bit 21 above gates the block's bus interface; it does not choose
 * or start the clock that generates frames.  That is CCIPR7.SAI1SEL, and
 * this driver never wrote it, so it sat at its reset value 000 = pclk2 while
 * the whole PLL2 and IC7 chain was being set up for nothing.  Field values
 * are the register's, in order:
 *
 *   000 pclk2     001 per_ck      010 ic7_ck    011 ic8_ck
 *   100 msi_ck    101 hsi_div_ck  110 I2S_CKIN  111 spdif_symb_ck
 */

#define RCC_CCIPR7_SAI1SEL_SHIFT (20)
#define RCC_CCIPR7_SAI1SEL_MASK  (7 << RCC_CCIPR7_SAI1SEL_SHIFT)

#define RCC_SAI1SEL_PCLK2        (0)
#define RCC_SAI1SEL_PER          (1)
#define RCC_SAI1SEL_IC7          (2)
#define RCC_SAI1SEL_IC8          (3)
#define RCC_SAI1SEL_MSI          (4)
#define RCC_SAI1SEL_HSI          (5)
#define RCC_SAI1SEL_I2SCKIN      (6)
#define RCC_SAI1SEL_SPDIF        (7)

/* Bus registers the diagnostic reads back.  Neither is in the port's RCC
 * header, and their offsets are this family's, not the 0x34 that older
 * parts put APB2ENR at: APB2RSTR 0x22c, APB2ENR 0x26c (stm32n647xx.h).
 */

#define STM32N6_RCC_APB2RSTR     (STM32_RCC_BASE + 0x022c)
#define STM32N6_RCC_APB2ENR      (STM32_RCC_BASE + 0x026c)
#define RCC_APB2RSTR_SAI1RST     (1 << 21)

/* PLL2, transcribed from the vendor BSP's 44.1 kHz case.  PLL2 is off on this
 * board and nothing else claims it: the NPU's IC6 stays on PLL1 by a
 * deliberate earlier decision, so borrowing PLL2 here disturbs nothing.
 */

#define SAI_PLL2_DIVM            (6)
#define SAI_PLL2_DIVN            (192)
#define SAI_PLL2_PDIV1           (4)
#define SAI_PLL2_PDIV2           (2)
#define SAI_IC7_DIV              (17)
#define SAI_HSE_HZ               (48000000)

/* Kernel clock reaching SAI1, and the divider its block needs.
 *
 * PLL2 drives IC7 and IC7 drives the block: 48 MHz / 6 * 192 / 4 / 17, which
 * comes to 22.58 MHz.  With NODIV clear the block's master clock is the
 * kernel clock divided by MCKDIV, one audio frame lasts 256 master clocks,
 * and so the divider that puts a frame out at the sample rate is
 *
 *   MCKDIV = kernel clock / (256 * Fs)
 *
 * which is 2 here, and is the same value the vendor HAL computes for the
 * same clock and the same request.  Zero is not a valid divider: the master
 * clock never runs, no frame is ever produced, the FIFO is never drained and
 * the request line is never asserted - the fill that had already reached the
 * FIFO just sits there, which is what a timeout looks like from above.
 */

#define SAI_TX_KERNEL_HZ \
  (SAI_HSE_HZ / SAI_PLL2_DIVM * SAI_PLL2_DIVN / SAI_PLL2_PDIV1 / SAI_IC7_DIV)

#define SAI_TX_RATE_HZ          (44100)
#define SAI_TX_MCKDIV           (SAI_TX_KERNEL_HZ / (SAI_TX_RATE_HZ * 256))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32n6_sai_priv_s
{
  uint32_t base;
  sem_t    lock;
  sem_t    dma_wait;
  bool     initialized;
  struct sai_config_s config;
  sai_buffer_cb_t callback;
  void    *cb_arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32n6_sai_priv_s g_sai1a_priv;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t sai_getreg(
    struct stm32n6_sai_priv_s *priv, uint32_t offset)
{
  return *(volatile uint32_t *)(priv->base + offset);
}

static inline void sai_putreg(
    struct stm32n6_sai_priv_s *priv, uint32_t offset,
    uint32_t value)
{
  *(volatile uint32_t *)(priv->base + offset) = value;
}

/* Kernel clock sources, in SAI1SEL field order (RM0486 "RCC_CCIPR7"). */

static FAR const char *g_sai1sel_name[8] =
{
  "pclk2", "per_ck", "ic7_ck", "ic8_ck",
  "msi_ck", "hsi_div_ck", "I2S_CKIN", "spdif_symb_ck"
};

/****************************************************************************
 * Name: sai_kernel_clock_select
 *
 * Description:
 *   Choose which clock SAI1 generates frames from.  Read-modify-write,
 *   because the same register carries the RTC source and prescaler.
 *
 ****************************************************************************/

static int sai_kernel_clock_get(void);

static void sai_kernel_clock_select(int sel)
{
  uint32_t regval = getreg32(STM32_RCC_CCIPR7);
  int      readback;

  regval &= ~RCC_CCIPR7_SAI1SEL_MASK;
  regval |= ((uint32_t)sel << RCC_CCIPR7_SAI1SEL_SHIFT);
  putreg32(regval, STM32_RCC_CCIPR7);

  /* Read it back.  This field is the one the driver used to leave alone
   * entirely, so it is worth proving on every boot that the write landed and
   * which source the block is actually running from.
   */

  readback = sai_kernel_clock_get();

  syslog(LOG_INFO, "sai: kernel clock set to sel=%d (%s), read back %d"
         "%s\n", sel, g_sai1sel_name[sel & 7], readback,
         readback == sel ? "" : " <- MISMATCH");
}

static int sai_kernel_clock_get(void)
{
  return (int)((getreg32(STM32_RCC_CCIPR7) & RCC_CCIPR7_SAI1SEL_MASK) >>
               RCC_CCIPR7_SAI1SEL_SHIFT);
}

static int sai_config_cr1(struct stm32n6_sai_priv_s *priv)
{
  uint32_t cr1 = 0;

  /* Mode: receiver */

  cr1 |= SAI_CR1_MODE_RX;

  /* Data size */

  if (priv->config.bits_per_sample == 24)
    {
      cr1 |= SAI_CR1_DS_24BIT;
    }
  else
    {
      cr1 |= SAI_CR1_DS_16BIT;
    }

  /* MCLK divider */

  cr1 |= (SAI_DEFAULT_MCLK_DIV << 20) &
          SAI_CR1_MCKDIV_MASK;

  /* Enable DMA */

  cr1 |= SAI_CR1_DMAEN;

  sai_putreg(priv, SAI_CR1_OFFSET, cr1);
  return 0;
}

static void sai_config_frame(struct stm32n6_sai_priv_s *priv)
{
  uint32_t frcr;
  uint32_t slotr;
  uint32_t frame_len;
  uint32_t slot_count;

  if (priv->config.channels == 2)
    {
      frame_len = priv->config.bits_per_sample * 2;
      slot_count = 2;
    }
  else
    {
      frame_len = priv->config.bits_per_sample;
      slot_count = 1;
    }

  /* Frame length and FS active level */

  frcr = ((frame_len - 1) & 0xff) |
         (((priv->config.bits_per_sample - 1) & 0x3f) << 8);
  sai_putreg(priv, SAI_FRCR_OFFSET, frcr);

  /* Slot configuration */

  slotr = (((priv->config.bits_per_sample - 1) & 0x3) << 6) |
          (((slot_count - 1) & 0xf) << 8) |
          (((1 << slot_count) - 1) << 16);
  sai_putreg(priv, SAI_SLOTR_OFFSET, slotr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32n6_sai_input_init(const struct sai_config_s *config)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;

  if (priv->initialized)
    {
      return 0;
    }

  priv->base = STM32N6_SAI1_BASE;
  priv->config = *config;

  nxsem_init(&priv->lock, 0, 1);
  nxsem_init(&priv->dma_wait, 0, 0);

  /* Disable SAI */

  sai_putreg(priv, SAI_CR1_OFFSET, 0);

  /* Configure CR1 */

  sai_config_cr1(priv);

  /* Configure frame and slot */

  sai_config_frame(priv);

  priv->initialized = true;

  syslog(LOG_INFO, "sai: initialized %luHz %ubit %uch\n",
         (unsigned long)config->sample_rate,
         config->bits_per_sample,
         config->channels);
  return 0;
}

int stm32n6_sai_input_start(sai_buffer_cb_t callback,
                              void *arg)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;

  if (!priv->initialized)
    {
      return -ENODEV;
    }

  priv->callback = callback;
  priv->cb_arg = arg;

  /* Enable SAI */

  sai_putreg(priv, SAI_CR1_OFFSET,
             sai_getreg(priv, SAI_CR1_OFFSET) |
             SAI_CR1_SAIEN);

  syslog(LOG_INFO, "sai: capture started\n");
  return 0;
}

int stm32n6_sai_input_stop(void)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;

  if (!priv->initialized)
    {
      return -ENODEV;
    }

  /* Disable SAI */

  sai_putreg(priv, SAI_CR1_OFFSET,
             sai_getreg(priv, SAI_CR1_OFFSET) &
             ~SAI_CR1_SAIEN);

  priv->callback = NULL;
  priv->cb_arg = NULL;

  syslog(LOG_INFO, "sai: capture stopped\n");
  return 0;
}

void stm32n6_sai_input_deinit(void)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;

  if (!priv->initialized)
    {
      return;
    }

  stm32n6_sai_input_stop();

  sai_putreg(priv, SAI_CR1_OFFSET, 0);

  priv->initialized = false;
  nxsem_destroy(&priv->lock);
  nxsem_destroy(&priv->dma_wait);

  syslog(LOG_INFO, "sai: deinitialized\n");
}


/* --- playback ------------------------------------------------------------ *
 *
 * SAI1 block A as master transmitter.  The codec is the slave and takes both
 * its bit clock and its MCLK from here, so the whole audio path hangs off
 * this block's kernel clock.
 */

/****************************************************************************
 * Name: stm32n6_sai_clock_config
 *
 * Description:
 *   Point SAI1's kernel clock at IC7 and start PLL2 feeding it.
 *
 *   PLL2 is off on this board and nothing else claims it: the NPU's IC6 was
 *   deliberately left on PLL1 (crossing to a second PLL cost more than the
 *   frequency bought back), and every other kernel clock in use is routed
 *   elsewhere.  So this is additive.
 *
 ****************************************************************************/

static int stm32n6_sai_clock_config(void)
{
  uint32_t regval;
  int      i;

  /* HSE first.
   *
   * The vendor's PLL2 settings are built around a 48 MHz reference.  This
   * board's PLL1 runs from HSI instead - IC6 and the NPU domain are clocked
   * from PLL1 at 1200 MHz, which is 64/4*75 and only works out from HSI - so
   * HSE is off at boot and has to be started before it can feed anything.
   *
   * Only PLL2 is taken from it.  PLL1, and with it SYSCLK, the NPU and every
   * other kernel clock, is left exactly as the FSBL set it up.
   */

  putreg32(getreg32(STM32_RCC_CR) | RCC_CR_HSEON, STM32_RCC_CR);

  for (i = 0; i < 100000; i++)
    {
      if ((getreg32(STM32_RCC_SR) & RCC_SR_HSERDY) != 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (i >= 100000)
    {
      syslog(LOG_ERR, "sai: HSE never started (SR=%08lx)\n",
             (unsigned long)getreg32(STM32_RCC_SR));
      return -ETIMEDOUT;
    }

  /* PLL2: source, dividers and post-dividers.  Read-modify-write on exactly
   * the fields the vendor HAL touches, so whatever else the FSBL left in
   * these registers survives - a blind write would clear bits nobody here
   * understands.  BYP goes with them: it is set at reset, and a bypassed PLL
   * ignores DIVM and DIVN entirely.
   */

  regval  = getreg32(STM32_RCC_PLL2CFGR1);
  regval &= ~(RCC_PLL2CFGR1_SEL_MASK | RCC_PLL2CFGR1_BYP |
              RCC_PLL2CFGR1_DIVM_MASK | RCC_PLL2CFGR1_DIVN_MASK);
  regval |= RCC_PLLSEL_HSE |
            ((uint32_t)SAI_PLL2_DIVM << 20) |
            ((uint32_t)SAI_PLL2_DIVN << 8);
  putreg32(regval, STM32_RCC_PLL2CFGR1);

  regval  = getreg32(STM32N6_RCC_PLL2CFGR2);
  putreg32(regval & ~RCC_PLL2CFGR2_FRAC_MASK, STM32N6_RCC_PLL2CFGR2);

  /* Integer mode only: spread spectrum and the fractional DAC stay off, and
   * PDIVEN goes in with the dividers.  Order matters -- the output enable has
   * to be in place before PLL2ON, which is how the HAL sequences it.
   */

  regval  = getreg32(STM32_RCC_PLL2CFGR3);
  regval &= ~(RCC_PLL2CFGR3_PDIV1_MASK | RCC_PLL2CFGR3_PDIV2_MASK |
              RCC_PLL2CFGR3_MODDSEN | RCC_PLL2CFGR3_DACEN);
  regval |= ((uint32_t)SAI_PLL2_PDIV1 << 27) |
            ((uint32_t)SAI_PLL2_PDIV2 << 24) |
            RCC_PLL2CFGR3_MODSSRST | RCC_PLL2CFGR3_MODSSDIS |
            RCC_PLL2CFGR3_PDIVEN;
  putreg32(regval, STM32_RCC_PLL2CFGR3);

  /* Enable it.  Nothing locks before this: the previous version configured
   * a PLL that was never switched on, so the wait could only time out.
   */

  putreg32(getreg32(STM32_RCC_CR) | RCC_CR_PLL2ON, STM32_RCC_CR);

  for (i = 0; i < 100000; i++)
    {
      if ((getreg32(STM32_RCC_SR) & RCC_SR_PLL2RDY) != 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (i >= 100000)
    {
      syslog(LOG_ERR, "sai: PLL2 never locked (SR=%08lx)\n",
             (unsigned long)getreg32(STM32_RCC_SR));
      return -ETIMEDOUT;
    }

  /* IC7 takes PLL2.  Its integer field holds the divider *minus one*, which
   * is what the vendor HAL writes.
   */

  putreg32(RCC_ICCFGR_SEL_PLL2 |
           ((uint32_t)(SAI_IC7_DIV - 1) << RCC_ICCFGR_INT_SHIFT),
           STM32N6_RCC_IC7CFGR);

  /* IC dividers have their own gate, off at reset.  Write-1-to-set alias. */

  putreg32(RCC_DIVENR_IC7EN, STM32_RCC_DIVENSR);

  /* And SAI1's APB2 gate. */

  putreg32(RCC_APB2ENR_SAI1EN, STM32_RCC_APB2ENSR);

  /* Point the block at IC7.
   *
   * Everything above builds a 22.58 MHz clock and enables the gate in front
   * of it, but SAI1 only consumes it when CCIPR7.SAI1SEL says so.  Leaving
   * that field alone (the previous state of this function) meant the block
   * ran from pclk2 while every number the dump printed described IC7, which
   * is why the clock read as healthy and the FIFO still never drained.
   */

  sai_kernel_clock_select(RCC_SAI1SEL_IC7);

  return 0;
}

/****************************************************************************
 * Name: stm32n6_sai_output_apply
 *
 * Description:
 *   Program the block.  Frame length 32 bits with 16 active, two 16-bit
 *   slots, FS before the first bit and active low: the standard I2S shape,
 *   and the one the vendor BSP uses.
 *
 *   NODIV stays clear, so the master clock comes from MCKDIV and works
 *   out at 256 * Fs: one frame per 256 master clocks, at 44.1 kHz.
 *
 ****************************************************************************/

static void stm32n6_sai_output_apply(struct stm32n6_sai_priv_s *priv)
{
  uint32_t cr1;
  uint32_t frcr;
  uint32_t slotr;
  int      i;

  cr1 = SAI_CR1_CKSTR | SAI_CR1_OUTDRIV | SAI_CR1_DMAEN | SAI_CR1_MCKEN |
        SAI_CR1_DS_16BIT |
        SAI_CR1_MCKDIV(SAI_TX_MCKDIV);

  frcr = ((SAI_TX_FRAME_BITS - 1) & 0xff) |
      SAI_FRCR_FSDEF | SAI_FRCR_FSPOL | SAI_FRCR_FSOFF |
         (((SAI_TX_ACTIVE_BITS - 1) & 0x3f) << 8);

  slotr = SAI_SLOTR_SLOTSZ_16B |
          (((SAI_TX_SLOTS - 1) & 0xf) << 8) |
          (((1 << SAI_TX_SLOTS) - 1) << 16);

  /* Off while it is being programmed, then on.  MCLK starts with SAIEN, so
   * the codec is listening before the first sample arrives.
   *
   * The flush in between matters for the retry paths: a block that never got
   * a clock keeps whatever it was given, and a FIFO that is already full
   * cannot show whether the new configuration drains.
   */

  sai_putreg(priv, SAI_CR1_OFFSET, 0);
  sai_putreg(priv, SAI_CR2_OFFSET, SAI_CR2_FFLUSH);

  /* FFLUSH is self-clearing.  Writing the next CR2 value before it clears
   * would overwrite the flush request, so wait for the bit to drop.
   */

  for (i = 0; i < 1000; i++)
    {
      if ((sai_getreg(priv, SAI_CR2_OFFSET) & SAI_CR2_FFLUSH) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  sai_putreg(priv, SAI_CR2_OFFSET, SAI_CR2_FTH_1QF);
  sai_putreg(priv, SAI_FRCR_OFFSET, frcr);
  sai_putreg(priv, SAI_SLOTR_OFFSET, slotr);

  /* Master clock first, transfer second.
   *
   * RM0486 spells this order out for MCLK: program MCKDIV, set MCKEN with the
   * block still disabled, and only afterwards set SAIEN.  Writing both bits
   * in one go -- which is what this did until 2026-10-28 -- produced a block
   * whose bit clock ran happily while the master clock pin stayed flat, and
   * a codec with no MCLK answers every register read with 0x00 no matter
   * what was written to it.  Two writes cost nothing.
   */

  sai_putreg(priv, SAI_CR1_OFFSET, cr1 & ~SAI_CR1_SAIEN);
  up_udelay(10);
  sai_putreg(priv, SAI_CR1_OFFSET, cr1 | SAI_CR1_SAIEN);

  /* Clear the sticky status bits the probes left behind.
   *
   * Writing the data register while the block had no clock set the overrun /
   * underrun flag, and that flag is cleared only by writing it back - it
   * survives every reconfiguration.  A block carrying a stale underrun flag
   * into playback is not a block whose behaviour can be predicted from its
   * configuration alone.
   */

  sai_putreg(priv, SAI_CLRFR_OFFSET, 0x7f);
}

/****************************************************************************
 * Name: sai_tx_clock_alive
 *
 * Description:
 *   Did the block consume anything after it was programmed?
 *
 *   Eight words is 181 us of audio at 44.1 kHz, so a block with a live
 *   kernel clock has emptied its FIFO by then and one without a clock still
 *   holds all eight.  The kernel clock sweep and the boot dump have disagreed
 *   on this board about which source works, and playback cannot afford to
 *   pick the wrong one, so the source is chosen here by measurement rather
 *   than by which register reads back what it was given.
 *
 *   Whatever is left of the probe is flushed before returning: those are
 *   zeros, and they must not reach the speaker.
 *
 * Returned Value:
 *   True if the FIFO drained.
 *
 ****************************************************************************/

static bool sai_tx_clock_alive(struct stm32n6_sai_priv_s *priv)
{
  uint32_t filled;
  bool     alive = false;
  int      i;

  for (i = 0; i < 8; i++)
    {
      sai_putreg(priv, SAI_DR_OFFSET, 0);
    }

  /* Read the level back before polling it.
   *
   * A level of zero is ambiguous on this board: it is what a clocked block
   * reports once it has drained, and it is also what a block reports when
   * the writes never reached the FIFO at all.  Only a level that rose and
   * then fell proves the block is consuming what it is given.  The kernel
   * clock sweep already showed the difference: ic7_ck fills the FIFO to five
   * of eight and then sits there.
   */

  filled = (sai_getreg(priv, SAI_SR_OFFSET) >> 16) & 0x7;

  if (filled != 0)
    {
      for (i = 0; i < 40; i++)
        {
          if (((sai_getreg(priv, SAI_SR_OFFSET) >> 16) & 0x7) == 0)
            {
              alive = true;
              break;
            }

          up_udelay(50);
        }
    }

  syslog(LOG_INFO, "sai: fifo filled to %lu of 7, %s\n",
         (unsigned long)filled,
         filled == 0 ? "writes never reached the block" :
         alive ? "drained, the block has a clock" :
                 "still full, no bit clock reached the block");

  sai_putreg(priv, SAI_CR2_OFFSET, SAI_CR2_FFLUSH);
  return alive;
}

static bool g_sai_cpu_feed;

int stm32n6_sai_output_init(uint32_t sample_rate, uint8_t channels)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;
  int ret;

  /* Only the 44.1 kHz family has clock numbers transcribed from the vendor
   * BSP; every other rate wants a different PLL2 setting and divider.
   */

  if (sample_rate != 44100 || channels != 2)
    {
      syslog(LOG_ERR, "sai: playback is wired for 44.1 kHz stereo only\n");
      return -EINVAL;
    }

  /* Clear the fallback from any earlier run.
   *
   * cpu_feed is set when one block timed out, and it used to stay set for the
   * life of the system: every playback after that was fed from the CPU, which
   * cannot keep 44.1 kHz in step with a card read, so the file played in
   * thirteen percent of its length.  A new stream is a new chance for the
   * channel, so the flag belongs to the stream, not to the port.
   */

  g_sai_cpu_feed = false;

  priv->base = STM32N6_SAI1_BASE;
  priv->config.sample_rate    = sample_rate;
  priv->config.channels       = channels;
  priv->config.bits_per_sample = 16;

  ret = stm32n6_sai_clock_config();
  if (ret < 0)
    {
      /* No clock means no MCLK, which means no acknowledge from the codec
       * and a transmit block that never asks the DMA for anything.  Failing
       * here says why, instead of leaving a timeout three layers down.
       */

      return ret;
    }

  stm32n6_configgpio(GPIO_SAI1_FS_A);
  stm32n6_configgpio(GPIO_SAI1_SCK_A);
  stm32n6_configgpio(GPIO_SAI1_SD_A);
  stm32n6_configgpio(GPIO_SAI1_MCLK_A);

  stm32n6_sai_output_apply(priv);

  /* The clock chain above is built so the block runs from IC7.  If that chain
   * is not actually reaching the block, the always-present bus clock still
   * plays the file, at a rate that follows it - and a wrong pitch beats a
   * silent speaker for a board whose audio has never come up.
   */

  if (!sai_tx_clock_alive(priv))
    {
      syslog(LOG_WARNING, "sai: ic7_ck did not clock the block, falling back "
             "to pclk2 for playback (the pitch will be off)\n");

      sai_kernel_clock_select(RCC_SAI1SEL_PCLK2);
      stm32n6_sai_output_apply(priv);

      if (!sai_tx_clock_alive(priv))
        {
          syslog(LOG_ERR, "sai: no kernel clock reaches the block at all\n");
        }
    }

  stm32n6_dma_init(SAI_TX_DMA_CHAN);

  priv->initialized = true;

  stm32n6_sai_output_dump();
  return 0;
}

/* Playback is fed from the CPU, one frame per free FIFO slot.
 *
 * The DMA path further down is kept but is not the default: measured on this
 * board, a 256 byte block reports complete within a couple of microseconds of
 * the channel being enabled - the source pointer has advanced by the whole
 * block - so the channel is not waiting for the block's requests.  A file
 * played that way is compressed into a click: 800 ms of tone was reported as
 * 20 ms, while the block itself was proven to have a clock (the FIFO fills to
 * five of seven and then drains).  The CPU loop below paces itself on the
 * FIFO level, so it cannot run ahead of the sample rate at all, at the cost of
 * holding the calling thread for the duration.
 *
 * The channel path is kept but not used as the default: CTR2.PFREQ and the
 * flag clear register are both now right, and with them the channel is paced
 * by the block at exactly the right rate, but nothing is heard.  The same
 * buffer written to the same data register from the CPU is audible, so the
 * fault is in the channel's access to the peripheral or to memory, not in the
 * data.  The next thing to look at is the RIF attribution of GPDMA1: the
 * board configures RIMC for SDMMC1 and for the NPU, and a channel whose
 * transactions are attributed to the wrong security domain can still be paced
 * by a request line while the peripheral discards everything it writes.
 */


/* One shot: report how long the first block took to leave, in ticks.  A block
 * of 4096 bytes is 23 ms of 44.1 kHz stereo, so a value that is not in that
 * neighbourhood means the channel is still not paced by the block.
 */


/* Per playback: how many channel blocks were started, and how many of those
 * were started with the FIFO already empty - which is exactly the moment that
 * is heard as a click.
 */

static uint32_t g_sai_blocks;
static uint32_t g_sai_starved;

/****************************************************************************
 * Name: stm32n6_sai_output_feed
 *
 * Description:
 *   Hand the block two samples at a time, one frame per data register write.
 *
 *   The FIFO drains at the sample rate, so the hardware paces this by
 *   itself: a write is only made when there is room, and room appears 44100
 *   times a second.  Slower than the channel and it holds the CPU for the
 *   whole file, but it needs no request line, no channel and no memory
 *   attributes, so audio plays even when the channel does not complete.
 *
 ****************************************************************************/

static int stm32n6_sai_output_feed(FAR const int16_t *samples, uint32_t count)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;
  uint32_t last = clock_systime_ticks();
  uint32_t i    = 0;

  while (i < count)
    {
      if (((sai_getreg(priv, SAI_SR_OFFSET) >> 16) & 0x7) <= 4)
        {
          uint32_t word = (uint32_t)(uint16_t)samples[i];

          i++;

          if (i < count)
            {
              word |= (uint32_t)(uint16_t)samples[i] << 16;
              i++;
            }

          sai_putreg(priv, SAI_DR_OFFSET, word);
          last = clock_systime_ticks();
        }
      else if (clock_systime_ticks() - last > MSEC2TICK(500))
        {
          syslog(LOG_ERR, "sai: fifo did not drain, cpu feed abandoned\n");
          return -ETIMEDOUT;
        }
    }

  return 0;
}


int stm32n6_sai_output_write(FAR const int16_t *samples, uint32_t count)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;
  uint32_t bytes;
  uint32_t first;
  int      ret;

  if (!priv->initialized || samples == NULL || count == 0)
    {
      return -EINVAL;
    }

  bytes = count * sizeof(int16_t);

  /* One DMA block per call; the caller streams the file in chunks.  The
   * block length field is 16 bits.
   */

  if (bytes > 0xffff)
    {
      return -E2BIG;
    }

  /* Word width: one beat is one stereo frame, which is exactly the shape the
   * CPU loop below produces and therefore the one the block is known to
   * consume at the configured rate.  A halfword beat delivers a single slot
   * per beat, and the request line asks for one transfer per frame, so that
   * shape runs at exactly half speed - measured as 800 ms of tone taking
   * 1600 ms and 15 s of file taking 31 s.
   */

  bytes &= ~(uint32_t)3;
  count  = bytes / sizeof(int16_t);

  if (bytes == 0)
    {
      return -EINVAL;
    }

  if (g_sai_cpu_feed)
    {
      /* Nobody is serving the request line, so the block is told to stop
       * raising it.  Leaving DMAEN set while feeding from the CPU makes the
       * block keep asking and recording an underrun every frame.
       */

      sai_putreg(priv, SAI_CR1_OFFSET,
                 sai_getreg(priv, SAI_CR1_OFFSET) & ~SAI_CR1_DMAEN);

      return stm32n6_sai_output_feed(samples, count);
    }

  /* The channel reads this memory directly, so it must be in memory and not
   * only in the cache.
   *
   * Clean the block, not the whole cache.  Cleaning all of it was tried while
   * the channel was believed to be reading stale memory, and it costs about a
   * millisecond and a half - longer than the 181 us the transmit FIFO holds,
   * so every block boundary lost that much audio.  Measured as 107% on a
   * fifteen second file, one millisecond and a half per block, while the same
   * block fed in one pass by the tone path ran at 100%.
   */

  up_clean_dcache((uintptr_t)samples, (uintptr_t)samples + bytes);

  /* Word width, not halfword: one beat writes one stereo frame, which is the
   * same shape the CPU loop below produces.  A halfword beat would deliver
   * one sample at a time and pair the channels differently.
   */

  /* Count the boundaries that start with nothing left in the FIFO: those are
   * the ones heard as clicks, so the number says whether the stutter is the
   * channel being restarted late or something else entirely.
   */

  g_sai_blocks++;

  if (((sai_getreg(priv, SAI_SR_OFFSET) >> 16) & 0x7) == 0)
    {
      g_sai_starved++;
    }

  stm32n6_dma_init(SAI_TX_DMA_CHAN);

  /* DATAPATH: the block must be told to ask the channel for data on every
   * start, not just in init.  output_quiet() clears DMAEN deliberately (it
   * wants the FIFO of zeros to sit still), so a later stream - the silence
   * keeper that runs between playbacks - would otherwise enqueue a block
   * that no request line ever drains.
   */

  sai_putreg(priv, SAI_CR1_OFFSET,
             sai_getreg(priv, SAI_CR1_OFFSET) | SAI_CR1_DMAEN);

  ret = stm32n6_dma_start_m2p(SAI_TX_DMA_CHAN, (uint32_t)(uintptr_t)samples,
                              priv->base + SAI_DR_OFFSET, bytes,
                              SAI_TX_DMA_REQUEST, 2);
  if (ret < 0)
    {
      return ret;
    }

  first = (uint32_t)clock_systime_ticks();

  ret = stm32n6_dma_wait(SAI_TX_DMA_CHAN, 1000);

  if (ret == -ETIMEDOUT)
    {
      /* The block has a clock and the codec is listening, so the audio can
       * go out without the channel.  Take the request line and the channel
       * out of the picture first, so nothing lands in the middle of the
       * transfer that follows, then feed the rest of the file from here.
       */

      stm32n6_dma_deinit(SAI_TX_DMA_CHAN);
      sai_putreg(priv, SAI_CR1_OFFSET,
                 sai_getreg(priv, SAI_CR1_OFFSET) & ~SAI_CR1_DMAEN);

      g_sai_cpu_feed = true;

      syslog(LOG_WARNING, "sai: no dma completion, feeding from the cpu "
             "instead\n");

      ret = stm32n6_sai_output_feed(samples, count);
    }

  return ret;
}


uint32_t stm32n6_sai_output_status(void)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;

  if (!priv->initialized)
    {
      return 0;
    }

  return sai_getreg(priv, SAI_SR_OFFSET);
}

/****************************************************************************
 * Name: sai_report_and_reset
 *
 * Description:
 *   Print the per playback counters once and clear them.  Both the stop and
 *   the quiet path end a playback, so both report the same way.
 *
 ****************************************************************************/

static void sai_report_and_reset(struct stm32n6_sai_priv_s *priv)
{
  /* One line, and only one: it is the health check for the whole path.  A
   * block that starts with the FIFO already dry is a click, so the second
   * number staying at zero is what says the silence keeper and the reader are
   * keeping up.  Everything else that used to be printed here measured how
   * the block was fed, which is now a settled question.
   */

  syslog(LOG_INFO, "sai: %lu blocks, %lu of them started with an empty "
         "fifo\n", (unsigned long)g_sai_blocks,
         (unsigned long)g_sai_starved);

  (void)priv;

  g_sai_blocks  = 0;
  g_sai_starved = 0;
}

bool stm32n6_sai_output_ready(void)
{
  return g_sai1a_priv.initialized;
}

void stm32n6_sai_output_stop(void)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;

  if (!priv->initialized)
    {
      return;
    }

  sai_report_and_reset(priv);

  stm32n6_dma_deinit(SAI_TX_DMA_CHAN);

  sai_putreg(priv, SAI_CR1_OFFSET,
             sai_getreg(priv, SAI_CR1_OFFSET) & ~SAI_CR1_SAIEN);

  priv->initialized = false;
}

/****************************************************************************
 * Name: stm32n6_sai_output_quiet
 *
 * Description:
 *   Hand the block back leaving it clocking, with zeros in the FIFO.
 *
 *   This is what the end of a playback wants, not a stop.  The codec stops
 *   answering I2C as soon as the block has run once, so its output stages
 *   cannot be turned off and the amplifier keeps whatever the codec drives
 *   into it.  With the block stopped the codec sees no MCLK, no bit clock
 *   and no frame sync at all, and that idle state is heard as a hiss - which
 *   is why the noise has always been louder after a playback than during
 *   one.  Left running with zeros queued, the codec receives digital silence
 *   at a valid rate and its outputs sit still.
 *
 *   The FIFO is flushed first so the last samples of the clip are not what
 *   gets repeated, and then loaded with zeros: whether the block repeats the
 *   last word or sends zeros once the FIFO runs dry, either way it is silence.
 *
 ****************************************************************************/

void stm32n6_sai_output_quiet(void)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;
  int i;

  if (!priv->initialized)
    {
      return;
    }

  sai_report_and_reset(priv);

  stm32n6_dma_deinit(SAI_TX_DMA_CHAN);

  sai_putreg(priv, SAI_CR1_OFFSET,
             sai_getreg(priv, SAI_CR1_OFFSET) & ~SAI_CR1_DMAEN);

  sai_putreg(priv, SAI_CR2_OFFSET, SAI_CR2_FFLUSH);

  for (i = 0; i < 8; i++)
    {
      sai_putreg(priv, SAI_DR_OFFSET, 0);
    }

  /* initialized stays set: the block is still running, and the next playback
   * starts by reprogramming it from the top anyway.
   */
}

void stm32n6_sai_output_dump(void)
{
  uint32_t cfgr1  = getreg32(STM32_RCC_PLL2CFGR1);
  uint32_t cfgr3  = getreg32(STM32_RCC_PLL2CFGR3);
  uint32_t ic7    = getreg32(STM32N6_RCC_IC7CFGR);
  uint32_t divm   = (cfgr1 >> 20) & 0x3f;
  uint32_t divn   = (cfgr1 >> 8) & 0xfff;
  uint32_t pdiv1  = (cfgr3 >> 27) & 0x7;
  uint32_t icint  = (ic7 >> RCC_ICCFGR_INT_SHIFT) & 0xff;
  uint32_t vco_khz;
  uint32_t ic_khz;
  uint32_t mckdiv;
  uint32_t us;
  uint32_t fflvl;
  int      i;

  if (divm == 0)
    {
      divm = 1;
    }

  if (pdiv1 == 0)
    {
      pdiv1 = 1;
    }

  vco_khz = (SAI_HSE_HZ / 1000 / divm) * divn;
  ic_khz  = vco_khz / pdiv1 / (icint + 1);

  mckdiv = (sai_getreg(&g_sai1a_priv, SAI_CR1_OFFSET) >> 20) & 0x3f;

  syslog(LOG_INFO, "sai: pll2 m=%lu n=%lu pdiv1=%lu -> vco %lu MHz; "
         "ic7 /%lu = %lu kHz; mckdiv %lu -> mclk %lu kHz, fs %lu Hz\n",
         (unsigned long)divm, (unsigned long)divn, (unsigned long)pdiv1,
         (unsigned long)(vco_khz / 1000), (unsigned long)(icint + 1),
         (unsigned long)ic_khz, (unsigned long)mckdiv,
         (unsigned long)(mckdiv != 0 ? ic_khz / mckdiv : 0),
         (unsigned long)(mckdiv != 0 ? ic_khz * 1000 / mckdiv / 256 : 0));

  /* The divider gate lives in DIVENR; DIVENSR is the write-1-to-set alias and
   * reads back as zero, so the previous report of it said nothing at all.
   */


  /* Clock probe.
   *
   * Push more words at the data register than the FIFO can hold, wait a
   * millisecond, and look at the level.  Eight words is under 200
   * microseconds of audio at 44.1 kHz, so a clocked block is empty by the
   * time this reads; an unclocked one is still full.  That one number says
   * whether the remaining problem is the clock or the DMA.
   */

  /* Clock probe: how long the block takes to empty its own FIFO.
   *
   * Eight words is 181 microseconds of audio at 44.1 kHz, so a clocked
   * block is empty by then and an unclocked one never empties at all.  One
   * number, and the range it has to be read over is five decades wide.
   */

  for (i = 0; i < 8; i++)
    {
      sai_putreg(&g_sai1a_priv, SAI_DR_OFFSET, 0);
    }

  for (us = 0; us < 2000; us += 50)
    {
      if (((sai_getreg(&g_sai1a_priv, SAI_SR_OFFSET) >> 16) & 0xf) == 0)
        {
          break;
        }

      up_udelay(50);
    }

  fflvl = (sai_getreg(&g_sai1a_priv, SAI_SR_OFFSET) >> 16) & 0xf;

}

/****************************************************************************
 * Name: sai_fifo_drain_us
 *
 * Description:
 *   Overfill the transmit FIFO and time how long the block needs to empty
 *   it.  Eight words is 181 us of audio at 44.1 kHz once the block has a
 *   clock; a block without one keeps all eight indefinitely.
 *
 *   This is the single measurement that separates "no clock reached the
 *   block" from "everything downstream of the clock".  Polling a status bit
 *   cannot make that distinction, and this part has no clock-ready bit to
 *   poll: the block's status register carries overrun, mute, wrong clock
 *   configuration, FIFO request, codec not ready, and the two frame-sync
 *   detections, and nothing else.
 *
 * Returned Value:
 *   Microseconds to empty, or -1 if nothing left inside 2 ms.
 *
 ****************************************************************************/

static int sai_fifo_drain_us(struct stm32n6_sai_priv_s *priv)
{
  uint32_t filled;
  uint32_t us;
  uint32_t last = 0;
  uint32_t frame;

  /* Write one word at a time and read the level back each time.  A level that
   * stays at zero proves the writes never reached the data register, which
   * would make the "never drains" reading below mean the wrong thing.
   */

  for (filled = 0; filled < 8; filled++)
    {
      sai_putreg(priv, SAI_DR_OFFSET, 0);

      frame = (sai_getreg(priv, SAI_SR_OFFSET) >> 16) & 0x7;
      if (frame != last)
        {
          last = frame;
        }
    }

  syslog(LOG_INFO, "sai-diag: fifo level after eight writes is %lu of 7\n",
         (unsigned long)last);

  for (us = 0; us < 20000; us += 50)
    {
      if (((sai_getreg(priv, SAI_SR_OFFSET) >> 16) & 0x7) == 0)
        {
          return (int)us;
        }

      up_udelay(50);
    }

  return -1;
}

/****************************************************************************
 * Name: sai_pin_edges
 *
 * Description:
 *   Count transitions on an alternate-function output pin over a window.
 *
 *   Every other measurement in this file trusts the block's own status
 *   registers, and those cannot tell "the clock generator is idle" from "the
 *   clock generator is running but the FIFO probe is misreading".  A pin
 *   either moves or it does not, and on STM32 the input data register keeps
 *   reporting the pad level even while the pin is driven by the alternate
 *   function, so this is a scope with no probe and no wire.
 *
 *   Sampled at roughly one microsecond, so a 44.1 kHz frame sync reads as
 *   about 880 edges over ten milliseconds and a stopped pin reads zero.
 *
 ****************************************************************************/

static int sai_pin_edges(uint32_t pinset, int us)
{
  bool last = stm32n6_gpioread(pinset);
  int  edges = 0;
  int  t;

  for (t = 0; t < us; t++)
    {
      bool now = stm32n6_gpioread(pinset);

      if (now != last)
        {
          edges++;
          last = now;
        }

      up_udelay(1);
    }

  return edges;
}

/****************************************************************************
 * Name: sai_pin_mux_dump
 *
 * Description:
 *   Report whether a pin really reached the SAI alternate function.
 *
 *   Counting edges on a pin that is still an input reads a flat zero and
 *   looks exactly like a stopped clock generator, so the mode and the
 *   alternate function number have to be on the record before the edge
 *   count means anything.
 *
 ****************************************************************************/

static void sai_pin_mux_dump(uintptr_t base, unsigned int pin,
                             FAR const char *name)
{
  uint32_t moder = getreg32(base + SAI_GPIO_MODER_OFFSET);
  uint32_t afr   = getreg32(base + (pin < 8 ? SAI_GPIO_AFRL_OFFSET :
                                                 SAI_GPIO_AFRH_OFFSET));
  unsigned int afshift = (pin < 8 ? pin : pin - 8) * 4;
  unsigned int mode = (moder >> (pin * 2)) & 3;
  unsigned int af   = (afr >> afshift) & 0xf;

  syslog(LOG_INFO, "sai-diag: pin %-4s mode=%u (2 is alternate function) "
         "af=%u (6 is SAI1) level=%lu%s\n",
         name, mode, af,
         (unsigned long)((getreg32(base + SAI_GPIO_IDR_OFFSET) >> pin) & 1),
         (mode == 2 && af == 6) ? "" :
           " <- NOT on the SAI alternate function, the edge count below "
           "cannot be trusted");
}

/****************************************************************************
 * Name: sai_pin_selftest
 *
 * Description:
 *   Prove the pin read back path works before trusting a zero.
 *
 *   Grounding the master clock pin, driving it as a plain output and reading
 *   the input register back answers one question: does reading PORTE
 *   actually report the pad?  If it does, a still pin under the SAI
 *   alternate function really is a still clock, and not a read path that
 *   always says zero.
 *
 ****************************************************************************/

#define SAI_MCLK_GPIO_OUT \
  (GPIO_MODE_OUTPUT | GPIO_OTYPE_PP | GPIO_SPEED_HIGH | \
   GPIO_PUPD_NONE | GPIO_PORTE | GPIO_PIN(2))

#define SAI_FS_GPIO_OUT \
  (GPIO_MODE_OUTPUT | GPIO_OTYPE_PP | GPIO_SPEED_HIGH | \
   GPIO_PUPD_NONE | GPIO_PORTB | GPIO_PIN(0))

#define SAI_SD_GPIO_OUT \
  (GPIO_MODE_OUTPUT | GPIO_OTYPE_PP | GPIO_SPEED_HIGH | \
   GPIO_PUPD_NONE | GPIO_PORTB | GPIO_PIN(2))

#define SAI_SCK_GPIO_OUT \
  (GPIO_MODE_OUTPUT | GPIO_OTYPE_PP | GPIO_SPEED_HIGH | \
   GPIO_PUPD_NONE | GPIO_PORTC | GPIO_PIN(2))

/****************************************************************************
 * Name: sai_pin_drive_test
 *
 * Description:
 *   Drive one SAI pin as a plain GPIO output and read the pad back.
 *
 *   This separates "the pad cannot drive" from "the block is not driving the
 *   pad".  They look identical from outside -- a still pin -- but the first is
 *   a supply or wiring fault and the second is a peripheral configuration
 *   fault, and the fixes are in completely different places.  Running it on
 *   every SAI pin says whether the working SCK pin is special or whether the
 *   other three are.
 *
 * Returned Value:
 *   True if the pad followed both levels.
 *
 ****************************************************************************/

static bool sai_pin_drive_test(uint32_t outcfg, uint32_t afcfg)
{
  bool low;
  bool high;

  stm32n6_configgpio(outcfg);

  stm32n6_gpiowrite(outcfg, false);
  up_udelay(20);
  low = stm32n6_gpioread(outcfg);

  stm32n6_gpiowrite(outcfg, true);
  up_udelay(20);
  high = stm32n6_gpioread(outcfg);

  stm32n6_configgpio(afcfg);

  return !low && high;
}

static bool sai_pin_selftest(void)
{
  bool fs  = sai_pin_drive_test(SAI_FS_GPIO_OUT,  GPIO_SAI1_FS_A);
  bool sda = sai_pin_drive_test(SAI_SD_GPIO_OUT,  GPIO_SAI1_SD_A);
  bool sck = sai_pin_drive_test(SAI_SCK_GPIO_OUT, GPIO_SAI1_SCK_A);
  bool mck = sai_pin_drive_test(SAI_MCLK_GPIO_OUT, GPIO_SAI1_MCLK_A);

  syslog(LOG_INFO, "sai-diag: pad drive test: FS(PB0)=%s SD(PB2)=%s "
         "SCK(PC2)=%s MCLK(PE2)=%s\n",
         fs ? "ok" : "DEAD", sda ? "ok" : "DEAD",
         sck ? "ok" : "DEAD", mck ? "ok" : "DEAD");

  if (!fs || !sda || !sck || !mck)
    {
      syslog(LOG_ERR, "sai-diag: a pad that cannot follow both levels has no "
             "supply or no connection; the edge counts for it mean nothing\n");
    }

  return fs && sda && sck && mck;
}

/****************************************************************************
 * Name: sai_pin_pull_test
 *
 * Description:
 *   Decide whether a pin is actually driven by the block or merely floating.
 *
 *   An edge count cannot tell the two apart: a high-impedance pad in
 *   alternate-function mode picks up noise and produces edges that look
 *   exactly like a slow clock.  The internal pull resistors settle it.  On a
 *   driven pin the weak pull cannot move the average level, so the fraction
 *   of high samples is the same with a pull up and with a pull down.  On a
 *   floating pin that fraction follows the pull: near zero with a pull down,
 *   near one with a pull up.
 *
 ****************************************************************************/

static int sai_pin_high_ratio(uint32_t cfgset, int samples)
{
  int high = 0;
  int i;

  stm32n6_configgpio(cfgset);

  for (i = 0; i < samples; i++)
    {
      if (stm32n6_gpioread(cfgset))
        {
          high++;
        }

      up_udelay(2);
    }

  return samples > 0 ? (high * 100) / samples : 0;
}

static void sai_pin_pull_test(uintptr_t port, unsigned int pin,
                              FAR const char *name)
{
  uint32_t base = (uint32_t)(GPIO_MODE_AF | GPIO_OTYPE_PP |
                             GPIO_SPEED_HIGH | GPIO_AF(6) | port |
                             GPIO_PIN(pin));
  int      pd;
  int      pu;

  pd = sai_pin_high_ratio(base | GPIO_PUPD_PD, 400);
  pu = sai_pin_high_ratio(base | GPIO_PUPD_PU, 400);

  /* Four outcomes, and the first version of this only recognised one of
   * them: a pin held low by its driver reads 0% with either pull, which the
   * old wording called "floating".  A genuinely floating pin follows the
   * pull instead -- near 0% up, near 100% pulled up.
   */

  syslog(LOG_INFO, "sai-diag: pull test %-4s pd=%d%% pu=%d%% -> %s\n",
         name, pd, pu,
         (pd <= 5 && pu >= 95) ?
           "nothing is driving it (it follows the pull)" :
         (pd <= 5 && pu <= 5) ?
           "driven or held LOW" :
         (pd >= 95 && pu >= 95) ?
           "driven or held HIGH" :
           "toggling (pull cannot move it)");
}

/****************************************************************************
 * Name: sai_mclk_only_probe
 *
 * Description:
 *   Enable nothing but the master clock and count the edges on its pin.
 *
 *   RM0486 Table 670 says MCLK comes out with MCLKEN set, NODIV clear and
 *   SAIEN clear -- the transfer does not have to be armed for the clock
 *   generator to run.  That makes this the smallest possible experiment: no
 *   FIFO, no DMA, no frame sync, no codec.  If the master clock pin still
 *   does not move, the block is not being clocked at all and every test
 *   downstream of it has been measuring the same fault.
 *
 * Returned Value:
 *   Edges counted on the master clock pin, or a negative errno if the clock
 *   chain could not be set up.
 *
 ****************************************************************************/

static int sai_mclk_only_probe(struct stm32n6_sai_priv_s *priv, int sel,
                               bool nodiv)
{
  uint32_t cr1;

  sai_kernel_clock_select(sel);

  sai_putreg(priv, SAI_CR1_OFFSET, 0);
  sai_putreg(priv, SAI_CR2_OFFSET, SAI_CR2_FFLUSH);
  up_udelay(100);
  sai_putreg(priv, SAI_CR2_OFFSET, SAI_CR2_FTH_1QF);
  sai_putreg(priv, SAI_FRCR_OFFSET,
             ((SAI_TX_FRAME_BITS - 1) & 0xff) |
             SAI_FRCR_FSDEF | SAI_FRCR_FSPOL |
             (((SAI_TX_ACTIVE_BITS - 1) & 0x3f) << 8));
  sai_putreg(priv, SAI_SLOTR_OFFSET,
             SAI_SLOTR_SLOTSZ_16B |
             (((SAI_TX_SLOTS - 1) & 0xf) << 8) |
             (((1 << SAI_TX_SLOTS) - 1) << 16));

  /* MCKEN and the divider only.  SAIEN stays clear. */

  cr1 = SAI_CR1_OUTDRIV | SAI_CR1_MCKEN | SAI_CR1_DS_16BIT;

  if (nodiv)
    {
      /* NODIV set: the master clock *is* the bit clock, so this variant
       * works whenever the block can drive SCK at all.
       */

      cr1 |= SAI_CR1_NODIV | SAI_CR1_MCKDIV(SAI_TX_MCKDIV);
    }
  else
    {
      cr1 |= SAI_CR1_MCKDIV(SAI_TX_MCKDIV);
    }

  sai_putreg(priv, SAI_CR1_OFFSET, cr1);

  syslog(LOG_INFO, "sai-diag: mclk-only on %-12s nodiv=%d cr1=%08lx "
         "read back %08lx\n",
         g_sai1sel_name[sel & 7], nodiv ? 1 : 0, (unsigned long)cr1,
         (unsigned long)sai_getreg(priv, SAI_CR1_OFFSET));

  return sai_pin_edges(GPIO_SAI1_MCLK_A, 2000);
}

/****************************************************************************
 * Name: stm32n6_sai_output_diag
 *
 * Description:
 *   Bring-up diagnosis that finishes in one run instead of one flash cycle
 *   per hypothesis.
 *
 *   Stage 1 reads every stage of the clock chain back and reports what it
 *   actually contains, rather than what the code meant to write.
 *
 *   Stage 2 tries all eight kernel clock sources in turn, reprogramming the
 *   block for each and measuring whether frames appear.  A block that drains
 *   on any of them has a working clock generator, which puts the fault in
 *   the source the driver selected; a block that drains on none of them has
 *   a fault at the block or the pin level, and further clock experiments
 *   cannot help.
 *
 *   The block is left on the first source that worked, so playback can be
 *   tried immediately after this returns.
 *
 * Returned Value:
 *   Zero if some clock source produces frames, -ENODEV if none does.
 *
 ****************************************************************************/

int stm32n6_sai_output_diag(void)
{
  struct stm32n6_sai_priv_s *priv = &g_sai1a_priv;
  int      drained[8];
  int      usable = -1;
  uint32_t cr1;
  uint32_t sr;
  uint32_t regval;
  int      sel;
  int      us;

  if (!priv->initialized)
    {
      priv->base = STM32N6_SAI1_BASE;

      if (stm32n6_sai_clock_config() < 0)
        {
          syslog(LOG_ERR, "sai-diag: clock chain would not come up\n");
          return -EIO;
        }

      stm32n6_configgpio(GPIO_SAI1_FS_A);
      stm32n6_configgpio(GPIO_SAI1_SCK_A);
      stm32n6_configgpio(GPIO_SAI1_SD_A);
      stm32n6_configgpio(GPIO_SAI1_MCLK_A);
      stm32n6_sai_output_apply(priv);
      priv->initialized = true;
    }

  syslog(LOG_INFO, "sai-diag: ==== 1 of 3: clock chain as read back ====\n");

  syslog(LOG_INFO, "sai-diag: pll2 on=%d ready=%d\n",
         (getreg32(STM32_RCC_CR) & RCC_CR_PLL2ON) ? 1 : 0,
         (getreg32(STM32_RCC_SR) & RCC_SR_PLL2RDY) ? 1 : 0);

  /* A locked PLL proves only that the VCO runs.  PDIVEN decides whether the
   * post dividers hand that clock on, and it is the bit this driver used to
   * leave clear, so it belongs on the record next to ic7cfgr.
   */

  regval = getreg32(STM32_RCC_PLL2CFGR3);

  syslog(LOG_INFO, "sai-diag: pll2cfgr3=%08lx pdiven=%d pdiv1=%lu pdiv2=%lu "
         "moddsen=%d\n",
         (unsigned long)regval,
         (regval & RCC_PLL2CFGR3_PDIVEN) ? 1 : 0,
         (unsigned long)((regval >> 27) & 0x7),
         (unsigned long)((regval >> 24) & 0x7),
         (regval & RCC_PLL2CFGR3_MODDSEN) ? 1 : 0);

  regval = getreg32(STM32N6_RCC_IC7CFGR);

  syslog(LOG_INFO, "sai-diag: ic7cfgr=%08lx source=%s divider=%lu\n",
         (unsigned long)regval,
         ((regval & RCC_ICCFGR_SEL_MASK) >> RCC_ICCFGR_SEL_SHIFT) == 0 ?
           "pll1" : "pll2",
         (unsigned long)(((regval & RCC_ICCFGR_INT_MASK) >>
                          RCC_ICCFGR_INT_SHIFT) + 1));

  syslog(LOG_INFO, "sai-diag: divenr.ic7en=%d apb2enr.sai1en=%d "
         "apb2rstr.sai1rst=%d\n",
         (getreg32(STM32_RCC_DIVENR) & RCC_DIVENR_IC7EN) ? 1 : 0,
         (getreg32(STM32N6_RCC_APB2ENR) & RCC_APB2ENR_SAI1EN) ? 1 : 0,
         (getreg32(STM32N6_RCC_APB2RSTR) & RCC_APB2RSTR_SAI1RST) ? 1 : 0);

  sel = sai_kernel_clock_get();
  syslog(LOG_INFO, "sai-diag: ccipr7.sai1sel=%d (%s)\n", sel,
         g_sai1sel_name[sel & 7]);

  cr1 = sai_getreg(priv, SAI_CR1_OFFSET);
  sr  = sai_getreg(priv, SAI_SR_OFFSET);

  syslog(LOG_INFO, "sai-diag: cr1=%08lx mode=%lu saien=%d mcken=%d "
         "nodiv=%d mckdiv=%lu\n",
         (unsigned long)cr1,
         (unsigned long)(cr1 & SAI_CR1_MODE_MASK),
         (cr1 & SAI_CR1_SAIEN) ? 1 : 0,
         (cr1 & SAI_CR1_MCKEN) ? 1 : 0,
         (cr1 & SAI_CR1_NODIV) ? 1 : 0,
         (unsigned long)((cr1 >> 20) & 0x3f));

  syslog(LOG_INFO, "sai-diag: sr=%08lx flvl=%lu%s%s%s%s\n",
         (unsigned long)sr,
         (unsigned long)((sr >> 16) & 0x7),
         (sr & SAI_SR_OVRUDR) ? " overrun" : "",
         (sr & SAI_SR_FREQ) ? " fifo-req" : "",
         (sr & (1 << 2)) ? " wrong-clock-config" : "",
         (sr & (1 << 5)) ? " fs-anticipated" : "");

  /* The frame and slot registers decide whether a frame can be generated at
   * all, and CR1 alone does not tell them apart from a block that silently
   * dropped the writes.  Read them back so the next round has the real
   * shape instead of the intended one.
   */

  syslog(LOG_INFO, "sai-diag: cr2=%08lx frcr=%08lx slotr=%08lx imr=%08lx\n",
         (unsigned long)sai_getreg(priv, SAI_CR2_OFFSET),
         (unsigned long)sai_getreg(priv, SAI_FRCR_OFFSET),
         (unsigned long)sai_getreg(priv, SAI_SLOTR_OFFSET),
         (unsigned long)sai_getreg(priv, SAI_IMR_OFFSET));

  /* Ask the pins rather than the registers.  A frame sync that moves means
   * the clock generator is running whatever the FIFO probe said, and a frame
   * sync that is nailed still means no amount of register programming will
   * produce sound.
   */

  stm32n6_configgpio(GPIO_SAI1_FS_A);
  stm32n6_configgpio(GPIO_SAI1_SCK_A);
  stm32n6_configgpio(GPIO_SAI1_MCLK_A);

  sai_pin_mux_dump(STM32_GPIOB_BASE, 0, "FS");
  sai_pin_mux_dump(STM32_GPIOC_BASE, 2, "SCK");
  sai_pin_mux_dump(STM32_GPIOE_BASE, 2, "MCLK");

  /* GCR sits in front of the block and PDM mode changes what the block even
   * is, so both are worth having on the record when nothing moves.
   */

  syslog(LOG_INFO, "sai-diag: gcr=%08lx pdmcr=%08lx\n",
         (unsigned long)getreg32(priv->base + SAI_GCR_OFFSET),
         (unsigned long)getreg32(priv->base + SAI_PDMCR_OFFSET));

  syslog(LOG_INFO, "sai-diag: pin edges over 10 ms: fs=%d sck=%d mclk=%d "
         "(a 44.1 kHz frame sync gives about 880 on fs, a stopped pin zero)\n",
         sai_pin_edges(GPIO_SAI1_FS_A, 10000),
         sai_pin_edges(GPIO_SAI1_SCK_A, 10000),
         sai_pin_edges(GPIO_SAI1_MCLK_A, 10000));

  /* An edge count cannot tell a real clock from a floating pad picking up
   * noise.  The internal pulls can: a driven pin ignores them.
   */

  sai_pin_pull_test(GPIO_PORTB, 0, "FS");
  sai_pin_pull_test(GPIO_PORTC, 2, "SCK");
  sai_pin_pull_test(GPIO_PORTE, 2, "MCLK");

  /* Same three measurements again with the block completely disabled.
   *
   * Anything that still moves with SAIEN and MCKEN cleared is not coming
   * from the SAI at all, and the earlier "SCK is driven" reading would have
   * been measuring something else on that pin.
   */

  syslog(LOG_INFO, "sai-diag: ---- same pins with the block disabled ----\n");
  sai_putreg(priv, SAI_CR1_OFFSET, 0);
  up_udelay(100);

  sai_pin_pull_test(GPIO_PORTB, 0, "FS");
  sai_pin_pull_test(GPIO_PORTC, 2, "SCK");
  sai_pin_pull_test(GPIO_PORTE, 2, "MCLK");

  sai_kernel_clock_select(RCC_SAI1SEL_IC7);
  stm32n6_sai_output_apply(priv);

  /* Smallest possible experiment: master clock only, transfer not armed.
   * RM0486 Table 670 says this is enough to drive MCLK out of the block.
   */

  syslog(LOG_INFO, "sai-diag: ==== 1b of 3: master clock on its own ====\n");

  /* Before reading a zero as "the block is not clocked", make sure the pin
   * read back reports the pad at all.
   */

  syslog(LOG_INFO, "sai-diag: pin read back self test: %s\n",
         sai_pin_selftest() ? "every SAI pad follows both levels" :
           "at least one SAI pad cannot drive, see the line above");

  syslog(LOG_INFO, "sai-diag: mclk edges pclk2 = %d\n",
         sai_mclk_only_probe(priv, RCC_SAI1SEL_PCLK2, false));
  syslog(LOG_INFO, "sai-diag: mclk edges ic7   = %d\n",
         sai_mclk_only_probe(priv, RCC_SAI1SEL_IC7, false));
  syslog(LOG_INFO, "sai-diag: mclk edges ic7 nodiv=1 = %d (mclk equals sck "
         "in this mode)\n",
         sai_mclk_only_probe(priv, RCC_SAI1SEL_IC7, true));

  /* Back to the real configuration before the sweep measures anything. */

  sai_kernel_clock_select(RCC_SAI1SEL_IC7);
  stm32n6_sai_output_apply(priv);

  /* Stage 2.  Each source gets a clean block and the same probe. */

  syslog(LOG_INFO, "sai-diag: ==== 2 of 3: kernel clock sweep ====\n");

  for (sel = 0; sel < 8; sel++)
    {
      sai_kernel_clock_select(sel);
      stm32n6_sai_output_apply(priv);

      /* Clear DMAEN for the sweep.  The block would ask the DMA for data
       * and, with no channel running, sit in a state that is not
       * representative of the clock question being asked.  Feeding the FIFO
       * from the CPU removes that variable entirely.
       */

      sai_putreg(priv, SAI_CR1_OFFSET,
                 sai_getreg(priv, SAI_CR1_OFFSET) & ~SAI_CR1_DMAEN);

      up_udelay(100);

      drained[sel] = sai_fifo_drain_us(priv);
      us           = drained[sel];

      syslog(LOG_INFO, "sai-diag: sel=%d %-14s %s\n",
             sel, g_sai1sel_name[sel],
             us >= 0 ? "frames appear" : "no frames");

      if (us >= 0 && (usable < 0 || sel == RCC_SAI1SEL_IC7))
        {
          /* Prefer IC7 whenever it works: it is the source the driver was
           * built around and the only one with a known frame rate.  Keeping
           * the first hit -- usually pclk2 -- made the verdict line claim
           * IC7 had failed when the log above plainly showed it working.
           */

          usable = sel;
        }
    }

  /* Stage 3.  Leave the block on something that works, or say plainly that
   * no clock source does.
   */

  syslog(LOG_INFO, "sai-diag: ==== 3 of 3: verdict ====\n");

  if (usable < 0)
    {
      syslog(LOG_ERR, "sai-diag: no clock source produces frames. The block "
             "is programmed, enabled and readable, so the fault is at the "
             "block or the pin level and no clock change will fix it.\n");
      syslog(LOG_ERR, "sai-diag: next: run the vendor bare-metal player "
             "on this board. Silent there too means the fault is outside "
             "the firmware; if it plays, compare its block A values "
             "against the ones above.\n");

      sai_kernel_clock_select(RCC_SAI1SEL_IC7);
      stm32n6_sai_output_apply(priv);
      return -ENODEV;
    }

  if (usable == RCC_SAI1SEL_IC7)
    {
      syslog(LOG_INFO, "sai-diag: ic7_ck works, which is the source this "
             "driver was built around. Try playback now.\n");
    }
  else
    {
      syslog(LOG_WARNING, "sai-diag: ic7_ck produces no frames. The PLL2 "
             "and IC7 chain is not reaching the block; playback is left on "
             "%s, the first source that worked, so the codec and the rest of "
             "the path can be checked.\n", g_sai1sel_name[usable]);
    }

  sai_kernel_clock_select(usable);
  stm32n6_sai_output_apply(priv);
  sai_putreg(priv, SAI_CLRFR_OFFSET, 0x7f);

  syslog(LOG_INFO, "sai-diag: left on %s, fifo emptied in %d us\n",
         g_sai1sel_name[usable], drained[usable]);
  return 0;
}

