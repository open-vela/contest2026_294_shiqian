/****************************************************************************
 * arch/arm/src/stm32n6/stm32n6_ethprobe.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * GMAC bring-up step 0: clock tree, RMII pin mux, PHY reset, MDIO sweep.
 *
 * This lives in its own translation unit on purpose.  stm32n6_ethernet.c is
 * only built when CONFIG_STM32_EMAC is set, and that symbol selects NET, so
 * using it here would drag the whole IP stack into the product image just to
 * ask the PHY for its identity.  Nothing below needs the network stack: it
 * only reads and writes GMAC registers.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <syslog.h>
#include <stdint.h>
#include <errno.h>

#include "arm_internal.h"
#include "stm32n6_gpio.h"

/* memorymap first: the RCC header derives every register address from
 * STM32_RCC_BASE, which that header owns.
 */

#include "hardware/stm32_memorymap.h"
#include "hardware/stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GMAC register base (CMSIS stm32n647xx.h: ETH1_BASE_NS) */

/* Ethernet 1.  AHB5PERIPH_BASE_S(0x58020000) + 0x16000.
 *
 * The secure alias, not the non-secure one.  A secure access to a
 * non-secure peripheral is always allowed, but the reverse is a RIFSC
 * violation, and nothing on this board leaves ETH1 as a non-secure slave:
 * the vendor's own binary references 0x58036000 twice and 0x48036000 not at
 * all, and this tree's working SDMMC driver uses 0x58027000 for the same
 * reason.
 */

#define STM32N6_ETH_BASE     0x58036000

#define ETH_MACMDIOAR_OFFSET 0x0200
#define ETH_MACMDIODR_OFFSET 0x0204

/* DMA mode register, and its software reset bit.  The HAL resets the whole
 * MAC subsystem here before it touches MDIO, and a MAC left un-reset is
 * exactly what a busy bit that never clears looks like: the MDIO front end
 * has no clock to advance with, whatever the CSR clock range says.
 */

/* CSR clock / 102.  HCLK runs at 200 MHz on this board, and 802.3 caps MDC
 * at 2.5 MHz, so the small dividers put MDC well out of spec.
 */

#define ETH_MDIO_DEFAULT_CR  4

#define ETH_DMAMR_OFFSET     0x1000
#define ETH_DMAMR_SWR        (1 << 0)

/* MDIO address register fields */

#define ETH_MACMDIOAR_GB        (1 << 0)
#define ETH_MACMDIOAR_GOC_READ  (3 << 2)
#define ETH_MACMDIOAR_CR_MASK   (0xf << 8)
#define ETH_MACMDIOAR_PA_MASK   (0x1f << 21)
#define ETH_MACMDIOAR_RDA_MASK  (0x1f << 16)

/* Ethernet clock tree (RM0486 "RCC kernel clock register 2", CCIPR2).
 *
 * Bits 18:16 are NOT the kernel clock source, they are the PHY interface
 * type: 000 MII, 001 RGMII, 100 RMII, everything else reserved.  This board
 * is wired for RMII, and the earlier version of this function left the field
 * at 000, so the MAC came up expecting MII TX/RX clocks that are not
 * connected -- which is why its MDIO front end never advanced a single bit
 * and the busy flag stayed set forever.
 *
 * The kernel clock is bits 13:12 instead, and 00 already selects hclke.
 *
 * The reference manual adds one ordering rule that matters: choose the PHY
 * interface while ETH1 is held in reset, before its clocks are enabled.
 */

#define STM32N6_RCC_CCIPR2        (STM32_RCC_BASE + 0x0148)
#define STM32N6_RCC_AHB5RSTR      (STM32_RCC_BASE + 0x0220)
#define STM32N6_RCC_AHB5ENSR      (STM32_RCC_BASE + 0x0a60)

#define RCC_CCIPR2_ETH1SEL_SHIFT  (16)
#define RCC_CCIPR2_ETH1SEL_MASK   (7 << RCC_CCIPR2_ETH1SEL_SHIFT)
#define RCC_ETH1SEL_MII           (0)
#define RCC_ETH1SEL_RGMII         (1)
#define RCC_ETH1SEL_RMII          (4)

#define RCC_CCIPR2_ETH1CLKSEL_SHIFT (12)
#define RCC_CCIPR2_ETH1CLKSEL_MASK  (3 << RCC_CCIPR2_ETH1CLKSEL_SHIFT)
#define RCC_ETH1CLKSEL_HCLKE      (0)

/* Both of these reset to "take the clock from a pin".  See eth_ccipr2_sweep
 * for why that matters on a board that does not route ETH1_CLK125.
 */

#define RCC_CCIPR2_ETH1GTXCLKSEL  (1 << 24)  /* 1 = internal, from RCC */
#define RCC_CCIPR2_ETH1REFCLKSEL  (1 << 20)  /* 1 = internal, from RCC */

#define RCC_AHB5RSTR_ETH1RST      (1 << 25)

#define RCC_AHB5ENR_ETH1MACEN     (1 << 22)
#define RCC_AHB5ENR_ETH1TXEN      (1 << 23)
#define RCC_AHB5ENR_ETH1RXEN      (1 << 24)
#define RCC_AHB5ENR_ETH1EN        (1 << 25)

/* Standard MII registers this step reads */

#define ETH_MII_BMSR    1
#define ETH_MII_PHYID1  2
#define ETH_MII_PHYID2  3

/* RIF / RIFSC.
 *
 * The vendor's NetXDuo example calls SystemIsolation_Config(), which gives
 * ETH1 both an RIMC master attribute (index 6: CID1, secure, privileged) and
 * a RISC slave attribute (REG1 | SEC28).  We never did either, while this
 * board's bring-up has already been bitten twice by leaving RIF alone - the
 * LTDC black screen was RISAF, the SDMMC hang was VDDIO - so it is the one
 * difference left against a known-good reference.
 *
 * Same treatment as dcmipp_rif_config(), which is already proven on this SoC.
 */

#define ETH1_RIF_MASTER_INDEX  6
#define RIFSC_RIMC_ATTR(n)     (STM32_RIFSC_BASE + 0x0c10 + ((n) << 2))
#define RIFSC_ATTR_MCID(n)     ((n) << 4)
#define RIFSC_ATTR_MSEC        (1 << 8)
#define RIFSC_ATTR_MPRIV       (1 << 9)
#define RIFSC_SECCFGR1         (STM32_RIFSC_BASE + 0x14)  /* bit28 = ETH1 */
#define RIFSC_PRIVCFGR1        (STM32_RIFSC_BASE + 0x34)  /* bit28 = ETH1 */
#define ETH1_RISC_BIT          (1u << 28)

/* RCC IC-divider enables.  ck_ker_eth1 passes through a divider before it
 * reaches the MAC (RM0486 Figure 53, the "D" block), and this tree already
 * programs RCC_DIVENR for DCMIPP/LTDC/SAI/ATON - but never for ETH1.  IC12
 * feeds the ETH1 kernel clock and IC13 feeds the ETH1 PTP clock, so both
 * dividers have to be enabled before ETH1CLKSEL=2 (ic12_ck) can carry a
 * clock at all.
 */

#define RCC_DIVENR_IC12EN      (1u << 11)
#define RCC_DIVENR_IC13EN      (1u << 12)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t eth_getreg(uint32_t offset)
{
  return *(volatile uint32_t *)(STM32N6_ETH_BASE + offset);
}

static inline void eth_putreg(uint32_t offset, uint32_t value)
{
  *(volatile uint32_t *)(STM32N6_ETH_BASE + offset) = value;
}

/****************************************************************************
 * Name: eth_clock_config
 *
 * Description:
 *   Point ETH1 at HCLK and open the four AHB5 gates.  Every later step --
 *   including the MDIO transactions -- depends on these being open, so they
 *   come before the pin mux and before the first register access that has to
 *   reach the PHY.
 *
 ****************************************************************************/

static void eth_clock_config(void)
{
  uint32_t regval;

  /* Hold the MAC in reset while its PHY interface is chosen. */

  putreg32(getreg32(STM32N6_RCC_AHB5RSTR) | RCC_AHB5RSTR_ETH1RST,
           STM32N6_RCC_AHB5RSTR);

  /* RMII, kernel clock from hclke.  Writing 000 here -- which is what the
   * first version of this probe did -- selects MII and leaves the MAC
   * waiting for MII clocks the board does not route.
   */

  regval  = getreg32(STM32N6_RCC_CCIPR2);
  regval &= ~RCC_CCIPR2_ETH1SEL_MASK;
  regval |= (RCC_ETH1SEL_RMII << RCC_CCIPR2_ETH1SEL_SHIFT);
  regval &= ~RCC_CCIPR2_ETH1CLKSEL_MASK;
  regval |= (RCC_ETH1CLKSEL_HCLKE << RCC_CCIPR2_ETH1CLKSEL_SHIFT);
  putreg32(regval, STM32N6_RCC_CCIPR2);

  /* Open the four gates through the write-one-to-set alias, which cannot
   * disturb the other AHB5 enables the way a read-modify-write can.
   */

  putreg32(RCC_AHB5ENR_ETH1MACEN | RCC_AHB5ENR_ETH1TXEN |
           RCC_AHB5ENR_ETH1RXEN  | RCC_AHB5ENR_ETH1EN,
           STM32N6_RCC_AHB5ENSR);

  /* Release the reset and give the MAC a moment to settle. */

  putreg32(getreg32(STM32N6_RCC_AHB5RSTR) & ~RCC_AHB5RSTR_ETH1RST,
           STM32N6_RCC_AHB5RSTR);

  up_mdelay(1);
}

/****************************************************************************
 * Name: eth_gpio_config
 *
 * Description:
 *   Bring out the RMII signals and the PHY reset line.
 *
 ****************************************************************************/

static void eth_gpio_config(void)
{
  stm32n6_configgpio(GPIO_ETH1_PHY_RST);
  stm32n6_configgpio(GPIO_ETH1_MDC);
  stm32n6_configgpio(GPIO_ETH1_MDIO);
  stm32n6_configgpio(GPIO_ETH1_REF_CLK);
  stm32n6_configgpio(GPIO_ETH1_CRS_DV);
  stm32n6_configgpio(GPIO_ETH1_TX_EN);
  stm32n6_configgpio(GPIO_ETH1_TXD0);
  stm32n6_configgpio(GPIO_ETH1_TXD1);
  stm32n6_configgpio(GPIO_ETH1_RXD0);
  stm32n6_configgpio(GPIO_ETH1_RXD1);
}

/****************************************************************************
 * Name: eth_phy_reset
 *
 * Description:
 *   Pulse the PHY reset, which is active low on this board.  The datasheet
 *   asks for a few tens of milliseconds before the MDIO interface answers
 *   and the vendor's own bring-up waits a hundred, so this does too.
 *
 ****************************************************************************/

static void eth_phy_reset(void)
{
  stm32n6_gpiowrite(GPIO_ETH1_PHY_RST, false);
  up_mdelay(20);
  stm32n6_gpiowrite(GPIO_ETH1_PHY_RST, true);
  up_mdelay(100);
}

/****************************************************************************
 * Name: eth_soft_reset
 *
 * Description:
 *   Reset the MAC subsystem the way the vendor HAL does before configuring
 *   anything else.
 *
 *   This step is easy to leave out because it looks like housekeeping, and
 *   omitting it produces a very specific symptom: MACMDIOAR accepts the
 *   request and echoes it back perfectly, but the busy bit never clears,
 *   because the logic that would clear it is still held in reset.
 *
 ****************************************************************************/

static bool eth_soft_reset(void)
{
  uint32_t timeout = 200000;

  eth_putreg(ETH_DMAMR_OFFSET, ETH_DMAMR_SWR);

  while (timeout-- > 0)
    {
      if ((eth_getreg(ETH_DMAMR_OFFSET) & ETH_DMAMR_SWR) == 0)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: eth_try_kernel_clock
 *
 * Description:
 *   Point the MAC at one of the four kernel clock sources and see whether
 *   the software reset can complete under it.
 *
 *   The reset completing is the cheapest possible proof that the MAC's
 *   functional clock is actually toggling: the reset is driven by that clock
 *   and the bit clears by itself when the logic runs.  Trying all four
 *   sources turns "no clock" into "which source is alive".
 *
 ****************************************************************************/

static bool eth_try_kernel_clock(int clksel, FAR const char *name)
{
  uint32_t regval;
  bool     ok;

  /* The manual wants the PHY interface and the clock source chosen while
   * the MAC is held in reset, so put it back under reset for each attempt
   * rather than editing CCIPR2 behind a running block.
   */

  putreg32(getreg32(STM32N6_RCC_AHB5RSTR) | RCC_AHB5RSTR_ETH1RST,
           STM32N6_RCC_AHB5RSTR);

  regval  = getreg32(STM32N6_RCC_CCIPR2);
  regval &= ~RCC_CCIPR2_ETH1CLKSEL_MASK;
  regval |= ((uint32_t)clksel << RCC_CCIPR2_ETH1CLKSEL_SHIFT);
  regval &= ~RCC_CCIPR2_ETH1SEL_MASK;
  regval |= (RCC_ETH1SEL_RMII << RCC_CCIPR2_ETH1SEL_SHIFT);
  putreg32(regval, STM32N6_RCC_CCIPR2);

  putreg32(getreg32(STM32N6_RCC_AHB5RSTR) & ~RCC_AHB5RSTR_ETH1RST,
           STM32N6_RCC_AHB5RSTR);

  up_mdelay(1);

  ok = eth_soft_reset();

  syslog(LOG_INFO, "eth-probe: kernel clock %-8s (%d) -> software reset %s "
         "(dmamr=%08lx)\n",
         name, clksel, ok ? "completed" : "still stuck",
         (unsigned long)eth_getreg(ETH_DMAMR_OFFSET));

  return ok;
}

/****************************************************************************
 * Name: eth_mdio_read
 *
 * Description:
 *   One MDIO read.  The clock-divider field is a parameter because the MDC
 *   frequency the PHY tolerates depends on HCLK, and HCLK is not a constant
 *   this file should assume.
 *
 * Returned Value:
 *   The 16-bit register value, or -ETIMEDOUT.
 *
 ****************************************************************************/

static int eth_mdio_read(uint8_t phy_addr, uint8_t reg_addr, int cr)
{
  uint32_t mdioar;
  uint32_t timeout = 100000;

  mdioar  = ((uint32_t)phy_addr << 21) & ETH_MACMDIOAR_PA_MASK;
  mdioar |= ((uint32_t)reg_addr << 16) & ETH_MACMDIOAR_RDA_MASK;
  mdioar |= ETH_MACMDIOAR_GOC_READ;
  mdioar |= ((uint32_t)cr << 8) & ETH_MACMDIOAR_CR_MASK;
  mdioar |= ETH_MACMDIOAR_GB;

  eth_putreg(ETH_MACMDIOAR_OFFSET, mdioar);

  /* Read the register back before waiting on it.  Two different faults look
   * identical from the outside: a MAC that is not clocked never latches the
   * request at all, and a MAC that is clocked clears the busy bit within a
   * few MDC periods and answers with all ones when nobody is there.  The
   * read back tells them apart, and it costs one register access.
   */

  if (cr == ETH_MDIO_DEFAULT_CR && phy_addr == 0)
    {
      syslog(LOG_INFO, "eth-probe: mdioar wrote %08lx read %08lx%s\n",
             (unsigned long)mdioar,
             (unsigned long)eth_getreg(ETH_MACMDIOAR_OFFSET),
             (eth_getreg(ETH_MACMDIOAR_OFFSET) & ETH_MACMDIOAR_GB) ?
               " <- busy bit stuck, the MAC is not running" :
               " (busy bit already cleared, the MAC is running)");
    }

  while (timeout-- > 0)
    {
      if (!(eth_getreg(ETH_MACMDIOAR_OFFSET) & ETH_MACMDIOAR_GB))
        {
          return (int)(eth_getreg(ETH_MACMDIODR_OFFSET) & 0xffff);
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: eth_pin_edges
 *
 * Description:
 *   Count transitions on a pin over a window by sampling its input data
 *   register.
 *
 *   PF7 carries the reference clock the PHY drives back into the MAC.  If
 *   that pin is still, the PHY is not running -- unpowered, held in reset,
 *   or its crystal is not oscillating -- and no amount of MAC programming
 *   will bring MDIO to life.  This is the one measurement that separates a
 *   MAC fault from a PHY fault without a scope.
 *
 ****************************************************************************/

static int eth_pin_edges(uint32_t pinset, int us)
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
 * Name: eth_scan
 *
 * Description:
 *   Sweep every MDC divider and every PHY address once.
 *
 * Returned Value:
 *   The PHY address that answered, or -ENODEV.
 *
 ****************************************************************************/

static int eth_scan(FAR int *out_cr, FAR int *out_id1, FAR int *out_id2)
{
  int cr;
  int addr;

  static const int crs[8] = { 4, 5, 6, 3, 2, 1, 0, 7 };
  int i;

  for (i = 0; i < 8; i++)
    {
      cr = crs[i];
      for (addr = 0; addr < 32; addr++)
        {
          int id1 = eth_mdio_read(addr, ETH_MII_PHYID1, cr);
          int id2 = eth_mdio_read(addr, ETH_MII_PHYID2, cr);

          /* A Motorcomm YT8512C reports PHYID1 = 0x0000 and carries the
           * whole model number in PHYID2 (0x0118), so rejecting a zero
           * PHYID1 - which the first version of this scan did - hides the
           * exact PHY fitted to this board.  Judge on PHYID2 instead, and
           * reject only the readings an empty bus produces: all ones from a
           * floating high line, all zeroes from one held low, or a timeout.
           */

          if (id1 < 0 || id2 < 0 || id2 == 0 || id2 == 0xffff)
            {
              continue;
            }

          *out_cr  = cr;
          *out_id1 = id1;
          *out_id2 = id2;
          return addr;
        }
    }

  return -ENODEV;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: eth_hal_literal
 *
 * Description:
 *   Bring the MAC up exactly the way the vendor's own example does, and see
 *   whether the software reset completes under that sequence.
 *
 *   The vendor MspInit (Projects/61_NetXDuo_TCP_Client) enables the four
 *   AHB5 gates with a read-modify-write on the plain enable register, never
 *   asserts ETH1RST, and lets HAL_ETH_Init() followed by SET_BIT on DMAMR do
 *   the rest.  Everything this probe did before that point pulsed the reset
 *   first, so if the reset bit is write-one-to-set on this part and cannot be
 *   released by writing a zero, the MAC would sit in reset while the bus
 *   still answers register reads - which is exactly the shape of the failure
 *   we have been chasing.
 *
 *   Running this first means the log separates "our sequence is wrong" from
 *   "the MAC has no clock at all" without another round of guessing.
 *
 ****************************************************************************/

static void eth_hal_literal(void)
{
  uint32_t regval;
  int      waited;

  /* __HAL_RCC_ETH1_CLK_ENABLE() and friends: plain read-modify-write on
   * AHB5ENR, preserving the other peripherals' gates.
   */

  regval  = getreg32(STM32_RCC_AHB5ENR);
  regval |= RCC_AHB5ENR_ETH1MACEN | RCC_AHB5ENR_ETH1TXEN |
            RCC_AHB5ENR_ETH1RXEN  | RCC_AHB5ENR_ETH1EN;
  putreg32(regval, STM32_RCC_AHB5ENR);

  /* SET_BIT(RCC->CCIPR2, RCC_ETH1PHYIF_RMII): OR the interface in, leave
   * the kernel clock at its reset value (00 = hclke).
   */

  regval  = getreg32(STM32N6_RCC_CCIPR2);
  regval &= ~RCC_CCIPR2_ETH1CLKSEL_MASK;
  regval |= (RCC_ETH1SEL_RMII << RCC_CCIPR2_ETH1SEL_SHIFT);
  putreg32(regval, STM32N6_RCC_CCIPR2);

  up_mdelay(1);

  /* SET_BIT(DMAMR, SWR), then up to 5 ms for it to clear by itself.  The
   * vendor HAL allows ETH_SWRESET_TIMEOUT here; 5 ms is far more than the
   * few CSR cycles the reset actually takes when the clock is running.
   */

  regval  = eth_getreg(ETH_DMAMR_OFFSET);
  regval |= ETH_DMAMR_SWR;
  eth_putreg(ETH_DMAMR_OFFSET, regval);

  for (waited = 0; waited < 500; waited++)
    {
      if ((eth_getreg(ETH_DMAMR_OFFSET) & ETH_DMAMR_SWR) == 0)
        {
          break;
        }

      up_udelay(10);
    }

  syslog(LOG_INFO, "eth-probe: hal-literal (no ETH1RST pulse) -> "
         "software reset %s after %d us (dmamr=%08lx ccipr2=%08lx "
         "ahb5enr=%08lx rstr=%08lx)\n",
         waited < 500 ? "completed" : "STILL STUCK",
         waited * 10,
         (unsigned long)eth_getreg(ETH_DMAMR_OFFSET),
         (unsigned long)getreg32(STM32N6_RCC_CCIPR2),
         (unsigned long)getreg32(STM32_RCC_AHB5ENR),
         (unsigned long)getreg32(STM32N6_RCC_AHB5RSTR));

  /* Whether the reset completed or not, one MDIO read tells us whether the
   * PHY front end is running.  CR=4 is CSR/102, which is 1.96 MHz at the
   * 200 MHz HCLK this board runs; the smaller dividers put MDC far above
   * the 2.5 MHz the standard allows and the PHY would never answer.
   */

  if (waited < 500)
    {
      int id1 = eth_mdio_read(0, ETH_MII_PHYID1, 4);
      int id2 = eth_mdio_read(0, ETH_MII_PHYID2, 4);

      syslog(LOG_INFO, "eth-probe: hal-literal MDIO CR=4 (CSR/102) -> "
             "phyid1=%04x phyid2=%04x%s\n",
             id1 & 0xffff, id2 & 0xffff,
             (id1 == 0xffff || id1 < 0) ?
               " (all ones or timeout: no PHY answer)" : "");
    }
}

/****************************************************************************
 * Name: eth_ccipr2_sweep
 *
 * Description:
 *   The four kernel clock sources all failed to complete the software reset,
 *   so the fault is downstream of the selector rather than in the choice of
 *   source.  Two bits of CCIPR2 sit on that downstream path and both of them
 *   reset to "take the clock from a pin":
 *
 *     bit 24 ETH1GTXCLKSEL  0 = external ETH1_CLK125 (needs the AF mux)
 *                           1 = internal, generated from ck_ker_eth1
 *     bit 20 ETH1REFCLKSEL  0 = external ETH_RX/REF_CLK (needs the AF mux)
 *                           1 = internal, ck_ker_eth1
 *
 *   This board routes the PHY's 50 MHz reference into PF7 for the RMII
 *   REF clock, but it does not route ETH1_CLK125 at all: PF5 carries the PHY
 *   reset instead of the MAC's 125 MHz output.  If the MAC's 125 MHz domain
 *   is fed from that missing pin, its logic never runs and the reset request
 *   sits in DMAMR forever -- exactly what the logs show.
 *
 *   Each variant is tried from a clean reset so the result is attributable.
 *
 ****************************************************************************/

static void eth_ccipr2_sweep(void)
{
  static const struct
  {
    uint32_t      bits;
    FAR const char *name;
  } variant[] =
  {
    { 0,                        "baseline ext GTX + ext REF" },
    { (1u << 24),               "GTX from RCC" },
    { (1u << 20),               "REF from RCC" },
    { (1u << 24) | (1u << 20),  "GTX + REF from RCC" },
  };

  uint32_t regval;
  bool     ok;
  int      i;

  syslog(LOG_INFO, "eth-probe: ==== CCIPR2 sweep (GTX/REF clock sources) "
         "====\n");

  for (i = 0; i < (int)(sizeof(variant) / sizeof(variant[0])); i++)
    {
      putreg32(getreg32(STM32N6_RCC_AHB5RSTR) | RCC_AHB5RSTR_ETH1RST,
               STM32N6_RCC_AHB5RSTR);

      regval  = getreg32(STM32N6_RCC_CCIPR2);
      regval &= ~RCC_CCIPR2_ETH1CLKSEL_MASK;
      regval |= (RCC_ETH1SEL_RMII << RCC_CCIPR2_ETH1SEL_SHIFT);
      regval &= ~(RCC_CCIPR2_ETH1GTXCLKSEL | RCC_CCIPR2_ETH1REFCLKSEL);
      regval |= variant[i].bits;
      putreg32(regval, STM32N6_RCC_CCIPR2);

      putreg32(getreg32(STM32N6_RCC_AHB5RSTR) & ~RCC_AHB5RSTR_ETH1RST,
               STM32N6_RCC_AHB5RSTR);

      up_mdelay(1);

      ok = eth_soft_reset();

      syslog(LOG_INFO, "eth-probe: ccipr2=%08lx %-26s -> reset %s "
             "(dmamr=%08lx maccr=%08lx dmaisr=%08lx dmadsr=%08lx)\n",
             (unsigned long)getreg32(STM32N6_RCC_CCIPR2),
             variant[i].name,
             ok ? "completed" : "stuck",
             (unsigned long)eth_getreg(ETH_DMAMR_OFFSET),
             (unsigned long)eth_getreg(0x0000),
             (unsigned long)eth_getreg(0x1008),
             (unsigned long)eth_getreg(0x100c));
    }
}

static void eth_rif_config(void)
{
  /* The RIFSC clock lives on AHB3 and has to be running before its registers
   * can be touched at all.
   */

  putreg32(RCC_AHB3ENR_RIFSCEN, STM32_RCC_AHB3ENSR);

  putreg32(RIFSC_ATTR_MCID(1) | RIFSC_ATTR_MSEC | RIFSC_ATTR_MPRIV,
           RIFSC_RIMC_ATTR(ETH1_RIF_MASTER_INDEX));

  modifyreg32(RIFSC_SECCFGR1, 0, ETH1_RISC_BIT);
  modifyreg32(RIFSC_PRIVCFGR1, 0, ETH1_RISC_BIT);

  syslog(LOG_INFO, "eth-probe: rif eth1 master=%08lx secfgr1=%08lx "
         "privcfgr1=%08lx\n",
         (unsigned long)getreg32(RIFSC_RIMC_ATTR(ETH1_RIF_MASTER_INDEX)),
         (unsigned long)getreg32(RIFSC_SECCFGR1),
         (unsigned long)getreg32(RIFSC_PRIVCFGR1));
}

/****************************************************************************
 * Name: eth_mdc_activity
 *
 * Description:
 *   Ask for a burst of MDIO reads and count transitions on MDC while they
 *   are in flight.
 *
 *   This is the one measurement that separates "the MAC is not running" from
 *   "the MAC is running but nobody answers".  A PHY that is absent, held in
 *   reset or unpowered still leaves 64 MDC clocks on the pin for every
 *   transaction, so a silent MDC means the failure is inside the MAC - which
 *   is what the stuck busy bit already suggests - while a busy MDC means the
 *   transactions really are going out and the PHY is the one not replying.
 *
 ****************************************************************************/

static void eth_mdc_activity(void)
{
  uint32_t mdioar = ETH_MACMDIOAR_GOC_READ | ETH_MACMDIOAR_GB | (4u << 8);
  int      frames = 64;
  int      edges  = 0;
  int      i;

  syslog(LOG_INFO, "eth-probe: ==== MDC pin activity (PH5) ====\n");

  for (i = 0; i < frames; i++)
    {
      eth_putreg(ETH_MACMDIOAR_OFFSET, mdioar);
      edges += eth_pin_edges(GPIO_ETH1_MDC, 100);
    }

  syslog(LOG_INFO, "eth-probe: MDC(PH5) edges during %d MDIO requests: %d "
         "%s\n", frames, edges,
         edges == 0 ?
           "-> the SMA front end is dead, the MAC is not clocked" :
           "-> MDC is running, the transactions are going out");
}

/****************************************************************************
 * Name: eth_divenr_enable
 *
 * Description:
 *   Open the IC12/IC13 dividers that stand between ck_ker_eth1 and the MAC.
 *
 ****************************************************************************/

static void eth_divenr_enable(void)
{
  uint32_t before = getreg32(STM32_RCC_DIVENR);

  modifyreg32(STM32_RCC_DIVENR, 0,
              RCC_DIVENR_IC12EN | RCC_DIVENR_IC13EN);

  syslog(LOG_INFO, "eth-probe: divenr %08lx -> %08lx (IC12EN|IC13EN)\n",
         (unsigned long)before,
         (unsigned long)getreg32(STM32_RCC_DIVENR));
}

/* GPIO register offsets used by the pin diagnostics below */

#define ETH_GPIO_MODER   0x00
#define ETH_GPIO_AFRL    0x20
#define ETH_GPIO_AFRH    0x24

/****************************************************************************
 * Name: eth_af_dump
 *
 * Description:
 *   Print the mode and alternate function that actually landed in the GPIO
 *   registers for the RMII pins.
 *
 *   configgpio() returning success only means the write was issued.  This
 *   board has already produced one peripheral whose pins read back "mode=2,
 *   af=6" and never moved at all, so the register value is worth seeing
 *   rather than assuming.
 *
 ****************************************************************************/

static void eth_af_dump(void)
{
  uint32_t fmoder = getreg32(STM32_GPIOF_BASE + ETH_GPIO_MODER);
  uint32_t fafrl  = getreg32(STM32_GPIOF_BASE + ETH_GPIO_AFRL);
  uint32_t fafrh  = getreg32(STM32_GPIOF_BASE + ETH_GPIO_AFRH);
  uint32_t hmoder = getreg32(STM32_GPIOH_BASE + ETH_GPIO_MODER);
  uint32_t hafrl  = getreg32(STM32_GPIOH_BASE + ETH_GPIO_AFRL);

  syslog(LOG_INFO, "eth-probe: gpio F.moder=%08lx F.afrl=%08lx F.afrh=%08lx "
         "H.moder=%08lx H.afrl=%08lx\n",
         (unsigned long)fmoder, (unsigned long)fafrl, (unsigned long)fafrh,
         (unsigned long)hmoder, (unsigned long)hafrl);

  syslog(LOG_INFO, "eth-probe: decode mod=%lu PF4=%lu PF5=%lu PF7=%lu "
         "PF10=%lu | mod=%lu PH5=%lu  (mode 2 = AF, af should be 11)\n",
         (unsigned long)((fmoder >> 8) & 3),
         (unsigned long)((fafrl >> 16) & 0xf),
         (unsigned long)((fafrl >> 20) & 0xf),
         (unsigned long)((fafrh >> 28) & 0xf),
         (unsigned long)((fafrh >> 8) & 0xf),
         (unsigned long)((hmoder >> 10) & 3),
         (unsigned long)((hafrl >> 20) & 0xf));
}

/****************************************************************************
 * Name: eth_pad_test
 *
 * Description:
 *   Drive MDC (PH5) as a plain push-pull GPIO and read it back.
 *
 *   A pad in a powered I/O domain follows whatever it is told to output; a
 *   pad whose domain is off reads the same value both ways.  That is the
 *   difference between "the MAC is not clocking MDC" and "MDC cannot move at
 *   all", and MDC reading zero edges makes the two indistinguishable.
 *
 *   The pad is put back into its ETH alternate function before returning.
 *
 ****************************************************************************/

static void eth_pad_test(void)
{
  uint32_t out = GPIO_MODE_OUTPUT | GPIO_OTYPE_PP | GPIO_SPEED_LOW |
                 GPIO_PUPD_NONE | GPIO_PORTH | GPIO_PIN(5);
  bool     hi;
  bool     lo;

  stm32n6_configgpio(out);

  stm32n6_gpiowrite(out, true);
  up_udelay(20);
  hi = stm32n6_gpioread(out);

  stm32n6_gpiowrite(out, false);
  up_udelay(20);
  lo = stm32n6_gpioread(out);

  syslog(LOG_INFO, "eth-probe: pad test PH5(MDC): drive-high reads %d, "
         "drive-low reads %d %s\n", hi, lo,
         (hi && !lo) ?
           "-> pad follows, the I/O domain is powered" :
           "-> pad does not follow, the pin or its I/O domain is dead");

  stm32n6_configgpio(GPIO_ETH1_MDC);
}

/****************************************************************************
 * Name: eth_scratch_test
 *
 * Description:
 *   Prove the register file really is a register file.
 *
 *   Every "the MAC is not running" conclusion so far rests on one assumption
 *   that was never tested: that a value written to an ETH1 register and read
 *   back came from the ETH1 register file rather than from a write buffer
 *   echoing the store.  Two write-only MAC hash registers are a clean test -
 *   they are independent, they have no reset-value significance, and a real
 *   register file keeps them apart.
 *
 ****************************************************************************/

static void eth_scratch_test(void)
{
  uint32_t a;
  uint32_t b;

  eth_putreg(0x0010, 0x11223344);   /* MACHT0R */
  eth_putreg(0x0014, 0x55667788);   /* MACHT1R */

  a = eth_getreg(0x0010);
  b = eth_getreg(0x0014);

  syslog(LOG_INFO, "eth-probe: scratch ht0=%08lx ht1=%08lx (want 11223344 / "
         "55667788) %s\n", (unsigned long)a, (unsigned long)b,
         (a == 0x11223344 && b == 0x55667788) ?
           "-> real register file, ETH1 is there" :
           "-> does NOT track, we are not talking to an ETH1 register file");

  eth_putreg(0x0010, 0xaabbccdd);
  a = eth_getreg(0x0010);
  b = eth_getreg(0x0014);

  syslog(LOG_INFO, "eth-probe: scratch2 ht0=%08lx ht1=%08lx (want aabbccdd / "
         "55667788)\n", (unsigned long)a, (unsigned long)b);
}

/****************************************************************************
 * Name: eth_block_existence
 *
 * Description:
 *   Ask the block whether it is really there.
 *
 *   MACHWF0R..3R are read-only registers whose reset values report the IP's
 *   feature set - a real datapath leaves them non-zero, while an address
 *   that decodes to nothing reads back zero.  That is a stronger existence
 *   test than a scratch write, which can look "not tracked" for reasons that
 *   have nothing to do with the block being absent.
 *
 *   DMAMR is also written with zero here.  If bit 0 reads back as one after
 *   being explicitly cleared, then SWR is not a self-clearing flag on this
 *   part at all, and every "software reset stuck" conclusion drawn so far
 *   would have been reading a tied bit rather than a stalled reset.
 *
 ****************************************************************************/

static void eth_block_existence(void)
{
  uint32_t hw0 = eth_getreg(0x011c);
  uint32_t hw1 = eth_getreg(0x0120);
  uint32_t hw2 = eth_getreg(0x0124);
  uint32_t hw3 = eth_getreg(0x0128);
  uint32_t maca;
  uint32_t dmamr;

  syslog(LOG_INFO, "eth-probe: hwfeat %08lx %08lx %08lx %08lx %s\n",
         (unsigned long)hw0, (unsigned long)hw1,
         (unsigned long)hw2, (unsigned long)hw3,
         (hw0 | hw1 | hw2 | hw3) != 0 ?
           "-> non-zero, an ETH datapath is answering" :
           "-> all zero, nothing decodes at this address");

  eth_putreg(0x0300, 0xdeadbeef);
  maca = eth_getreg(0x0300);
  syslog(LOG_INFO, "eth-probe: MACA0HR wrote deadbeef read %08lx %s\n",
         (unsigned long)maca,
         maca == 0xdeadbeef ? "-> tracks" : "-> does NOT track");

  eth_putreg(ETH_DMAMR_OFFSET, 0);
  dmamr = eth_getreg(ETH_DMAMR_OFFSET);
  syslog(LOG_INFO, "eth-probe: DMAMR written 0, reads %08lx %s\n",
         (unsigned long)dmamr,
         dmamr == 0 ? "-> bit0 is a real flag" :
           "-> bit0 stays set even when cleared, it is tied");
}

/****************************************************************************
 * Name: eth_mac_alive
 *
 * Description:
 *   Is the MAC register file answering?
 *
 *   Everything before this point judged the block by whether DMAMR.SWR clears
 *   by itself.  That was the wrong yardstick: the software reset covers the
 *   MAC and the DMA together, so a MAC whose clock is missing keeps SWR set
 *   forever no matter what the DMA half is doing, and the DMA half answers
 *   register accesses either way.  A scratch write into the MAC register
 *   file asks the question directly.
 *
 ****************************************************************************/

static bool eth_mac_alive(void)
{
  eth_putreg(0x0010, 0x11223344);            /* MACHT0R */
  return eth_getreg(0x0010) == 0x11223344;
}

/****************************************************************************
 * Name: eth_mac_clock_search
 *
 * Description:
 *   Sweep the two clock controls that describe the MAC's 125 MHz domain
 *   against all four kernel clock sources, and judge each one by whether the
 *   MAC register file starts answering.
 *
 *     bit 24 ETH1GTXCLKSEL  0 = external ETH1_CLK125 (needs the AF mux)
 *                           1 = internal from ck_ker_eth1
 *     bit 20 ETH1REFCLKSEL  0 = external REF clock (the PHY drives PF7)
 *                           1 = internal from ck_ker_eth1
 *
 ****************************************************************************/

static void eth_mac_clock_search(void)
{
  static const struct
  {
    uint32_t      bits;
    FAR const char *name;
  } combo[] =
  {
    { 0,                        "ext GTX + ext REF" },
    { (1u << 24),               "GTX=RCC" },
    { (1u << 24) | (1u << 20),  "GTX=RCC REF=RCC" },
  };

  static const struct
  {
    uint32_t      sel;
    FAR const char *name;
  } src[] =
  {
    { 0, "hclke"   },
    { 1, "per_ck"  },
    { 2, "ic12_ck" },
    { 3, "hse_ck"  },
  };

  uint32_t regval;
  bool     alive;
  int      i;
  int      j;

  syslog(LOG_INFO, "eth-probe: ==== MAC register file liveness sweep "
         "(MACA0HR/MACHT0R, not DMAMR) ====\n");

  for (i = 0; i < (int)(sizeof(src) / sizeof(src[0])); i++)
    {
      for (j = 0; j < (int)(sizeof(combo) / sizeof(combo[0])); j++)
        {
          putreg32(getreg32(STM32N6_RCC_AHB5RSTR) | RCC_AHB5RSTR_ETH1RST,
                   STM32N6_RCC_AHB5RSTR);

          regval  = getreg32(STM32N6_RCC_CCIPR2);
          regval &= ~RCC_CCIPR2_ETH1CLKSEL_MASK;
          regval |= (src[i].sel << RCC_CCIPR2_ETH1CLKSEL_SHIFT);
          regval &= ~RCC_CCIPR2_ETH1SEL_MASK;
          regval |= (RCC_ETH1SEL_RMII << RCC_CCIPR2_ETH1SEL_SHIFT);
          regval &= ~(RCC_CCIPR2_ETH1GTXCLKSEL | RCC_CCIPR2_ETH1REFCLKSEL);
          regval |= combo[j].bits;
          putreg32(regval, STM32N6_RCC_CCIPR2);

          putreg32(getreg32(STM32N6_RCC_AHB5RSTR) & ~RCC_AHB5RSTR_ETH1RST,
                   STM32N6_RCC_AHB5RSTR);

          up_mdelay(1);

          alive = eth_mac_alive();

          syslog(LOG_INFO, "eth-probe: mac %-8s + %-16s -> %s "
                 "(ccipr2=%08lx maca0hr=%08lx hwfeat0=%08lx)\n",
                 src[i].name, combo[j].name,
                 alive ? "MAC IS ALIVE" : "mac silent",
                 (unsigned long)getreg32(STM32N6_RCC_CCIPR2),
                 (unsigned long)eth_getreg(0x0300),
                 (unsigned long)eth_getreg(0x011c));
        }
    }
}

/****************************************************************************
 * Name: eth_page_scan
 *
 * Description:
 *   Read every 32-bit word of the first two 4 KB pages of the ETH1 window
 *   and report which offsets return anything other than zero.
 *
 *   The MAC half of the register file has not answered a single read while
 *   the DMA half at +0x1000 stores and returns values, and those two offsets
 *   fall in different 4 KB pages.  A read-only map of what actually responds
 *   says whether the window is one peripheral with a dead half, or two
 *   different decodes that were never the same block.
 *
 ****************************************************************************/

static void eth_page_scan(void)
{
  int page;
  int off;
  int shown;
  int hits[2] = { 0, 0 };

  /* Stop at 0x1FFF.  Reading 0x58038000 raises a precise data bus error
   * (CFSR.PRECISERR, BFAR=0x58038000), so ETH1 decodes 8 KB and the next
   * 4 KB page falls into the undefined hole before USB1_OTG_HS.  Scanning
   * past the end of the window panics the kernel and kills the rest of the
   * probe, so the range stays inside the block.
   */

  for (page = 0; page < 2; page++)
    {
      shown = 0;
      syslog(LOG_INFO, "eth-probe: page %d non-zero words:\n", page);

      for (off = 0; off < 0x400; off += 4)
        {
          uint32_t v = eth_getreg((uint32_t)(page * 0x1000 + off));

          if (v != 0)
            {
              hits[page]++;

              if (shown < 12)
                {
                  syslog(LOG_INFO, "eth-probe:   +0x%04x = %08lx\n",
                         (unsigned)(page * 0x1000 + off),
                         (unsigned long)v);
                  shown++;
                }
            }
        }

      syslog(LOG_INFO, "eth-probe: page %d: %d non-zero words out of 256%s\n",
             page, hits[page],
             hits[page] == 0 ? "  (the whole page reads back zero)" : "");
    }
}

/****************************************************************************
 * Name: eth_swr_report
 *
 * Description:
 *   Run the DMA software reset and report it in full.
 *
 *   DMAMR read 0x00000001 during the page scan, before this probe had
 *   written the bit, so the flag was already set on entry.  Knowing whether
 *   it clears now that the register file is proven to answer decides between
 *   "the reset was a leftover request" and "the DMA really cannot finish".
 *
 ****************************************************************************/

static void eth_swr_report(void)
{
  uint32_t csrswcr = eth_getreg(0x0230);          /* MACCSRSWCR */
  uint32_t before  = eth_getreg(ETH_DMAMR_OFFSET);
  bool     ok      = eth_soft_reset();
  uint32_t after   = eth_getreg(ETH_DMAMR_OFFSET);

  syslog(LOG_INFO, "eth-probe: swr maccsrswcr=%08lx dmamr %08lx -> %08lx "
         "(%s)\n", (unsigned long)csrswcr,
         (unsigned long)before, (unsigned long)after,
         ok ? "completed, the DMA is clocked" : "STILL SET");
}

/****************************************************************************
 * Name: eth_mdio_quick
 *
 * Description:
 *   Walk the eight low PHY addresses once at the in-spec divider and print
 *   what came back.
 *
 *   Every failure up to this point was traced to a closed clock gate, so the
 *   PHY has never actually been asked a question.  This is that question: a
 *   YT8512C answers at address 0 with a non-FFFF PHYID1.
 *
 ****************************************************************************/

static void eth_mdio_quick(FAR const char *tag)
{
  int addr;
  int found = 0;

  syslog(LOG_INFO, "eth-probe: ---- MDIO quick scan CR=%d (%s) ----\n",
         ETH_MDIO_DEFAULT_CR, tag);

  for (addr = 0; addr < 8; addr++)
    {
      int id1 = eth_mdio_read((uint8_t)addr, ETH_MII_PHYID1,
                              ETH_MDIO_DEFAULT_CR);
      int id2 = eth_mdio_read((uint8_t)addr, ETH_MII_PHYID2,
                              ETH_MDIO_DEFAULT_CR);

      if (id1 < 0)
        {
          syslog(LOG_INFO, "eth-probe:   addr %d: no answer, busy bit never "
                 "cleared (%d)\n", addr, id1);
        }
      else if (id1 == 0xffff)
        {
          syslog(LOG_INFO, "eth-probe:   addr %d: all ones, nothing drives "
                 "MDIO\n", addr);
        }
      else
        {
          found++;
          syslog(LOG_INFO, "eth-probe:   addr %d: phyid1=%04x phyid2=%04x\n",
                 addr, id1 & 0xffff, id2 & 0xffff);
        }
    }

  syslog(LOG_INFO, "eth-probe: MDIO quick scan: %d device(s) answered\n",
         found);
}

/****************************************************************************
 * Name: eth_phy_dump
 *
 * Description:
 *   Dump the standard MII registers plus the Motorcomm-specific status
 *   register of the PHY at address 0.
 *
 *   Two questions are being settled here.  BMSR bit 2 is latched low by the
 *   standard: the first read after a link loss returns zero and the read
 *   itself clears the latch, so a single read cannot tell "no link" from
 *   "link came up since the last read".  Reading it twice does.
 *
 *   The second question is whether anything from the far end is arriving at
 *   all.  ANLPAR is all zeroes until the PHY has received at least one fast
 *   link pulse from the partner, so a non-zero ANLPAR proves the cable, the
 *   magnetics and the partner's transmitter are all working, and moves the
 *   fault to the auto-negotiation configuration.
 *
 ****************************************************************************/

static void eth_phy_dump(void)
{
  int bmcr    = eth_mdio_read(0, 0x00, ETH_MDIO_DEFAULT_CR);
  int bmsr1   = eth_mdio_read(0, 0x01, ETH_MDIO_DEFAULT_CR);
  int bmsr2   = eth_mdio_read(0, 0x01, ETH_MDIO_DEFAULT_CR);
  int id1     = eth_mdio_read(0, 0x02, ETH_MDIO_DEFAULT_CR);
  int id2     = eth_mdio_read(0, 0x03, ETH_MDIO_DEFAULT_CR);
  int anar    = eth_mdio_read(0, 0x04, ETH_MDIO_DEFAULT_CR);
  int anlpar  = eth_mdio_read(0, 0x05, ETH_MDIO_DEFAULT_CR);
  int aner    = eth_mdio_read(0, 0x06, ETH_MDIO_DEFAULT_CR);
  int spec    = eth_mdio_read(0, 0x11, ETH_MDIO_DEFAULT_CR);

  syslog(LOG_INFO, "eth-probe: phy bmcr=%04x (an_en=%d restart_an=%d "
         "loopback=%d pdown=%d)\n",
         bmcr & 0xffff,
         (bmcr >> 12) & 1, (bmcr >> 9) & 1,
         (bmcr >> 14) & 1, (bmcr >> 11) & 1);

  syslog(LOG_INFO, "eth-probe: phy bmsr #1=%04x #2=%04x (link=%d an_done=%d)\n",
         bmsr1 & 0xffff, bmsr2 & 0xffff,
         (bmsr2 >> 2) & 1, (bmsr2 >> 5) & 1);

  syslog(LOG_INFO, "eth-probe: phy id %04x:%04x anar=%04x anlpar=%04x "
         "aner=%04x\n",
         id1 & 0xffff, id2 & 0xffff,
         anar & 0xffff, anlpar & 0xffff, aner & 0xffff);

  syslog(LOG_INFO, "eth-probe: phy spec11=%04x (link=%d speed=%d duplex=%d)%s\n",
         spec & 0xffff, (spec >> 10) & 1, (spec >> 14) & 3, (spec >> 13) & 1,
         anlpar == 0 ?
           " -> ANLPAR is zero, nothing is arriving from the far end" :
           " -> ANLPAR is non-zero, the PHY hears the partner");
}

/****************************************************************************
 * Name: eth_phy_wait_link
 *
 * Description:
 *   Wait for the PHY to report link up and auto-negotiation complete.
 *
 *   The earlier revisions of this probe sampled BMSR about 100 ms after
 *   pulsing the PHY reset and always read link=0, ANLPAR=0.  That was not a
 *   fault: 802.3 clause 28 auto-negotiation needs two to three seconds, and
 *   the reset the probe issues tears down whatever link Windows had already
 *   established.  Polling turns a guaranteed "not yet" into a real answer.
 *
 *   BMSR bit 2 is latched low, so each sample reads it twice.
 *
 ****************************************************************************/

static void eth_phy_wait_link(void)
{
  int i;

  for (i = 0; i < 60; i++)
    {
      eth_mdio_read(0, 0x01, ETH_MDIO_DEFAULT_CR);      /* clear latch */
      int bmsr = eth_mdio_read(0, 0x01, ETH_MDIO_DEFAULT_CR);

      if (bmsr > 0 && (bmsr & 0x0004) != 0)
        {
          int anlpar = eth_mdio_read(0, 0x05, ETH_MDIO_DEFAULT_CR);
          int aner   = eth_mdio_read(0, 0x06, ETH_MDIO_DEFAULT_CR);

          syslog(LOG_INFO, "eth-probe: link up after %d00 ms "
                 "(bmsr=%04x an_done=%d anlpar=%04x aner=%04x)\n",
                 i + 1, bmsr & 0xffff, (bmsr >> 5) & 1,
                 anlpar & 0xffff, aner & 0xffff);
          return;
        }

      up_mdelay(100);
    }

  syslog(LOG_INFO, "eth-probe: no link after 6000 ms (bmsr=%04x) - the PHY "
         "never negotiated, check the cable, the partner's speed setting and "
         "the magnetics\n",
         eth_mdio_read(0, 0x01, ETH_MDIO_DEFAULT_CR) & 0xffff);
}

int stm32n6_eth_probe(void)
{
  int found_cr   = -1;
  int found_addr = -1;
  int id1        = 0;
  int id2        = 0;
  int bmsr       = 0;
  uint32_t gate;

  /* Try the vendor's sequence before anything else touches the reset. */

  /* The AHB5 gates have to be open before any ETH1 register can answer.
   * The previous revision ran the register file tests first and only opened
   * the gates further down, so every reading it produced came from a window
   * whose clock was gated off: MACHWF0R, MACA0HR and MACHT0R all read back
   * zero no matter what the hardware was doing, and all twelve combinations
   * of the MAC liveness sweep were guaranteed to report "mac silent".
   *
   * Open the gates, select the PHY interface and release the reset first,
   * then measure.
   */

  eth_rif_config();
  eth_clock_config();
  eth_gpio_config();

  syslog(LOG_INFO, "eth-probe: gates open: ahb5enr=%08lx ccipr2=%08lx "
         "ahb5rstr=%08lx\n",
         (unsigned long)getreg32(STM32_RCC_AHB5ENR),
         (unsigned long)getreg32(STM32N6_RCC_CCIPR2),
         (unsigned long)getreg32(STM32N6_RCC_AHB5RSTR));

  eth_page_scan();
  eth_scratch_test();
  eth_block_existence();
  eth_mdc_activity();
  eth_swr_report();
  eth_mdio_quick("PHY reset not pulsed yet");
  eth_phy_reset();
  eth_mdio_quick("after pulsing the PHY reset on PF5");
  eth_phy_wait_link();
  eth_phy_dump();
  eth_af_dump();
  eth_pad_test();
  eth_hal_literal();
  eth_ccipr2_sweep();
  eth_mac_clock_search();

  /* The reference clock on PF7 already proves the PHY is alive, so the
   * remaining question is whether the MAC's own functional clock is
   * running.  Ask all four kernel clock sources; the one whose software
   * reset completes is the one that is actually toggling.
   */

  syslog(LOG_INFO, "eth-probe: ==== kernel clock sweep (reset completes = "
         "clock alive) ====\n");

  eth_divenr_enable();

  if (!eth_try_kernel_clock(0, "hclke"))
    {
      /* ic12_ck only exists once its IC divider is enabled, so it is tried
       * after eth_divenr_enable() and not before.
       */

      eth_try_kernel_clock(2, "ic12_ck");
      eth_try_kernel_clock(3, "hse_ck");
      eth_try_kernel_clock(1, "per_ck");
      eth_try_kernel_clock(0, "hclke");
    }

  eth_phy_reset();

  syslog(LOG_INFO, "eth-probe: stage 1 clocks: ccipr2=%08lx "
         "eth1sel=%lu (4 is RMII) clksel=%lu ahb5enr=%08lx rstr=%08lx\n",
         (unsigned long)getreg32(STM32N6_RCC_CCIPR2),
         (unsigned long)((getreg32(STM32N6_RCC_CCIPR2) &
                          RCC_CCIPR2_ETH1SEL_MASK) >>
                         RCC_CCIPR2_ETH1SEL_SHIFT),
         (unsigned long)((getreg32(STM32N6_RCC_CCIPR2) &
                          RCC_CCIPR2_ETH1CLKSEL_MASK) >>
                         RCC_CCIPR2_ETH1CLKSEL_SHIFT),
         (unsigned long)getreg32(STM32_RCC_AHB5ENR),
         (unsigned long)getreg32(STM32N6_RCC_AHB5RSTR));

  /* Decode the four gates individually.  A partial write is the failure that
   * looks like "no PHY answered": ETH1MAC without a clock leaves the MDIO
   * front end dead even though the other three gates read back set.
   */

  gate = getreg32(STM32_RCC_AHB5ENR);

  syslog(LOG_INFO, "eth-probe: gate eth1en=%d macen=%d txen=%d rxen=%d%s\n",
         (gate & RCC_AHB5ENR_ETH1EN) ? 1 : 0,
         (gate & RCC_AHB5ENR_ETH1MACEN) ? 1 : 0,
         (gate & RCC_AHB5ENR_ETH1TXEN) ? 1 : 0,
         (gate & RCC_AHB5ENR_ETH1RXEN) ? 1 : 0,
         ((gate & (RCC_AHB5ENR_ETH1EN | RCC_AHB5ENR_ETH1MACEN |
                   RCC_AHB5ENR_ETH1TXEN | RCC_AHB5ENR_ETH1RXEN)) ==
          (RCC_AHB5ENR_ETH1EN | RCC_AHB5ENR_ETH1MACEN |
           RCC_AHB5ENR_ETH1TXEN | RCC_AHB5ENR_ETH1RXEN)) ?
           "" : " <- SOME GATES DID NOT STICK");

  syslog(LOG_INFO, "eth-probe: stage 2 pins: MDC=PH5 MDIO=PF4 REF_CLK=PF7 "
         "CRS_DV=PF10 TX_EN=PF11 TXD0=PF12 TXD1=PF13 RXD0=PF14 RXD1=PF15\n");

  /* Ask the PHY whether it is alive at all before blaming the MAC.  PF7 is
   * the 50 MHz reference the PHY drives back when it is out of reset and its
   * crystal is running.  A still pin means the fault is on the PHY side and
   * no MAC register will change that.
   */

  stm32n6_configgpio(GPIO_ETH1_REF_CLK);

  syslog(LOG_INFO, "eth-probe: stage 3b ref clock on PF7: %d edges over 2 ms "
         "(a running 50 MHz PHY gives thousands, a dead one zero)\n",
         eth_pin_edges(GPIO_ETH1_REF_CLK, 2000));

  syslog(LOG_INFO, "eth-probe: stage 4 scan: 8 MDC dividers x 32 PHY "
         "addresses\n");

  /* Sweep the MDC divider first, then the five-bit PHY address.  A PHY that
   * is absent, unpowered or held in reset reads all ones, so that is what
   * "nobody there" looks like on this bus.
   */

  found_addr = eth_scan(&found_cr, &id1, &id2);

  if (found_addr < 0)
    {
      /* The schematic shows ETH_RESET as a 4.7K pull up with 100nF to
       * ground, which makes it active low -- but the vendor's own MSP drives
       * PF5 low and never releases it, so the polarity is worth settling on
       * the board rather than in a comment.  Try the other level too.
       */

      syslog(LOG_WARNING, "eth-probe: no answer with PF5 high, trying PF5 "
             "low (the vendor's MSP leaves it low)\n");

      stm32n6_gpiowrite(GPIO_ETH1_PHY_RST, false);
      up_mdelay(100);

      found_addr = eth_scan(&found_cr, &id1, &id2);
      if (found_addr >= 0)
        {
          syslog(LOG_WARNING, "eth-probe: PF5 low is the released level on "
                 "this board; the reset is active high\n");
        }

      if (found_addr < 0)
        {
          syslog(LOG_ERR, "eth-probe: no PHY answered on either PF5 level. "
                 "With the reference clock reading above, that points at PHY "
                 "power or the crystal, not at the MAC.\n");
          return -ENODEV;
        }
    }

  bmsr = eth_mdio_read(found_addr, ETH_MII_BMSR, found_cr);

  syslog(LOG_INFO, "eth-probe: phy at address %d, id %04x:%04x, "
         "mdc divider %d, bmsr %04x\n",
         found_addr, id1, id2, found_cr, bmsr);

  syslog(LOG_INFO, "eth-probe: the vendor BSP expects a YT8512C at address "
         "0; bmsr bit 2 means link up and bit 5 means autonegotiation "
         "complete\n");

  return found_addr;
}

/* The real arm_netinitialize() now lives in stm32n6_ethernet.c. */

