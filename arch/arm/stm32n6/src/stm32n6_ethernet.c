/****************************************************************************
 * arch/arm/src/stm32n6/stm32n6_ethernet.c
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
 ****************************************************************************/

/* ETH1 driver for the STM32N6 (Synopsys GMAC5 with the multi-channel DMA).
 *
 * Polled, no interrupts.  A kernel thread walks the Rx ring and offers the
 * Tx ring to the network stack every 20 ms, which is far cheaper to get right
 * on a first bring-up than the descriptor/interrupt choreography and still
 * moves a full 100 Mbit/s stream in practice.
 *
 * Two facts drive the initialisation order and both were established on the
 * board rather than read out of a manual:
 *
 *   1. In RMII mode the 50 MHz reference clock comes from the PHY on PF7.
 *      Until the PHY is out of reset and running there is no clock, and the
 *      DMA refuses to come out of its software reset: DMAMR.SWR stays set
 *      forever while the register file happily answers reads and writes.
 *
 *   2. Auto-negotiation needs about half a second.  Sampling BMSR right after
 *      the reset pulse always reports "no link", which is what made an
 *      earlier revision of the probe believe the MAC was dead.
 *
 * So the order is: gates, interface select, PHY reset, wait for link, and
 * only then touch the MAC and the DMA.
 *
 * Register offsets and descriptor layouts come from the STM32N647 CMSIS
 * header and RM0486 Tables 819/822: TDES0/RDES0 hold buffer-1 address,
 * TDES2 holds the buffer-1 length in bits 13:0, and ownership lives in
 * TDES3/RDES3 bit 31 with BUF1V at bit 24 on the receive side.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/kthread.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/netconfig.h>

#include "arm_internal.h"
#include "stm32n6_gpio.h"
#include "hardware/stm32_memorymap.h"
#include "hardware/stm32_rcc.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ETHERNET)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ETH1 register block.
 *
 * The secure alias is used on purpose: the FSBL leaves the peripheral in its
 * reset SECURE state and NuttX runs secure, so this is the alias the CPU is
 * guaranteed to be allowed to reach.
 */

#define ETH_BASE              0x58036000u

/* MAC registers */

#define ETH_MACCR             (ETH_BASE + 0x0000)
#define ETH_MACPFR            (ETH_BASE + 0x0008)
#define ETH_MACMDIOAR         (ETH_BASE + 0x0200)
#define ETH_MACMDIODR         (ETH_BASE + 0x0204)
#define ETH_MACA0HR           (ETH_BASE + 0x0300)
#define ETH_MACA0LR           (ETH_BASE + 0x0304)
#define ETH_MACRXQC0R         (ETH_BASE + 0x00a0)
#define ETH_MTLOMR            (ETH_BASE + 0x0c00)
#define ETH_MTLTXQOMR         (ETH_BASE + 0x0d00)
#define ETH_MTLTXQDR          (ETH_BASE + 0x0d08)
#define ETH_MTLRXQDMAMR       (ETH_BASE + 0x0c30)
#define ETH_MTLRXQOMR         (ETH_BASE + 0x0d30)
#define ETH_MTLRXQMPOCR       (ETH_BASE + 0x0d34)
#define ETH_MTLRXQDR          (ETH_BASE + 0x0d38)

/* MMC (RMON) receive error counters.  They separate "frames never arrived"
 * from "frames arrived but the MAC rejected them", which the queue debug
 * register alone cannot tell apart.
 */

#define ETH_MMCCR             (ETH_BASE + 0x0700)
#define ETH_MMCCR_CR          (1u << 0)
#define ETH_MMCRIR            (ETH_BASE + 0x0704)
#define ETH_MMCRCRCEPR        (ETH_BASE + 0x0794)
#define ETH_MMCTPCGR          (ETH_BASE + 0x0768)
#define ETH_MMCRAEPR          (ETH_BASE + 0x0798)

/* MAC_RxQ_Ctrl0.RXQ0EN: 0 = queue 0 not enabled, 1 = enabled for AV,
 * 2 = enabled for generic traffic.  The MTL queue settings and the DMA rings
 * are useless until this is set, and with it left at zero the MAC accepts
 * nothing at all: no FIFO occupancy, no MMC error counters, and a receive
 * DMA that sits on the first descriptor forever.
 */

#define ETH_RXQC0R_RXQ0EN_GT  (2u << 0)

#define ETH_MACCR_RE          (1u << 0)
#define ETH_MACCR_TE          (1u << 1)
#define ETH_MACCR_DM          (1u << 13)
#define ETH_MACCR_FES         (1u << 14)

/* Port Select.  The vendor HAL forces this on for every interface except
 * RGMII ("force it for 10/100m"), and with it clear the MAC listens on the
 * wrong port: the packet filter, the queues and the rings all look correct
 * while MTLRXQDR stays at zero because no frame ever reaches the MAC.
 */

#define ETH_MACCR_PS          (1u << 15)
#define ETH_MACPFR_PR         (1u << 0)

/* MTL queue operation modes.  The register names on this part carry no queue
 * number (MTLTXQOMR, not MTLTXQ0OMR); the bit layout is the usual one.
 */

#define ETH_MTLTXQOMR_FTQ     (1u << 0)
#define ETH_MTLTXQOMR_TSF     (1u << 1)
#define ETH_MTLTXQOMR_TXQEN   (2u << 2)
#define ETH_MTLRXQOMR_RSF     (1u << 5)

/* Queue sizes.  TQS is bits[19:16] of MTLTXQOMR and RQS is bits[23:20] of
 * MTLRXQOMR; zero is not "no limit" but the smallest possible queue, 256
 * bytes.  Combined with store-and-forward that is fatal on receive: a frame
 * larger than 256 bytes can never be stored completely, so the MAC drops it
 * without filling the FIFO and without charging any MMC error, and the DMA
 * waits forever on a descriptor that never completes.  The vendor default is
 * 4096 bytes of receive queue and 2048 of transmit queue.
 */

#define ETH_MTLTXQOMR_TQS_2K  (0x3u << 16)
#define ETH_MTLRXQOMR_RQS_4K  (0xfu << 20)
/* MTL_RxQ_DMA_Map0.RXQ0DCM is a 4-bit DMA channel *number*, not a bit mask.
 * Writing 1 there points receive queue 0 at DMA channel 1, which has no
 * descriptor ring at all.  Channel 0 is the reset value and the wanted one.
 */

#define ETH_MTLRXQDMAMR_CH0   0u

/* The programmable burst length is not optional.  RM0486 on DMACxRXCR.RXPBL:
 * "You can program PBL with any of the following values: 1, 2, 4, 8, 16, or
 * 32.  Any other value results in undefined behavior."  The field resets to
 * zero, and the MTL only forwards a queue once the number of bytes available
 * there matches what the DMA asks for, so a zero RXPBL leaves the receive FIFO
 * full for ever while the DMA reports that it is running and raises no error.
 * The vendor HAL always programs 32 beats.
 */

#define ETH_DMACRXCR_RXPBL_32 (32u << 16)
#define ETH_DMACTXCR_TXPBL_32 (32u << 16)

/* Vendor defaults lifted from HAL_ETH's ETH_MACDMAConfig() so the controller
 * is programmed exactly like the ST example: address aligned beats, fixed
 * burst, AXI burst length 4 and three outstanding read/write requests.
 */

#define ETH_DMASBMR_VENDOR    0x03031003u

/* Strict priority arbitration both ways and drop-transmit-status on overflow,
 * again the vendor default.  DMACxCR is left at zero: its descriptor skip
 * length is ignored while the ring length registers are non-zero.
 */

#define ETH_MTLOMR_VENDOR     0x00000066u

/* MAC_1us_Tic_Counter has to match the CSR clock or every internal microsecond
 * based timer is wrong.  HCLK is 200 MHz on this board.
 */

#define ETH_MAC1USTCR_VAL     199u

/* DMA global registers */

#define ETH_MAC1USTCR         (ETH_BASE + 0x00dc)
#define ETH_DMAMR             (ETH_BASE + 0x1000)
#define ETH_DMAMR_SWR         (1u << 0)
#define ETH_DMASBMR           (ETH_BASE + 0x1004)
#define ETH_DMADSR            (ETH_BASE + 0x100c)

/* DMA channel 0.  Note that the tail-pointer offsets differ from the STM32H7
 * driver (0x1120/0x1128 here against 0x1110/0x1118 there), so its register
 * map cannot be reused verbatim.
 */

#define ETH_DMACTXCR          (ETH_BASE + 0x1104)
#define ETH_DMACRXCR          (ETH_BASE + 0x1108)
#define ETH_DMACTXDLAR        (ETH_BASE + 0x1114)
#define ETH_DMACRXDLAR        (ETH_BASE + 0x111c)
#define ETH_DMACTXDTPR        (ETH_BASE + 0x1120)
#define ETH_DMACRXDTPR        (ETH_BASE + 0x1128)
#define ETH_DMACTXRLR         (ETH_BASE + 0x112c)
#define ETH_DMACRXRLR         (ETH_BASE + 0x1130)
#define ETH_DMACCATXDR        (ETH_BASE + 0x1144)
#define ETH_DMACCARXDR        (ETH_BASE + 0x114c)
#define ETH_DMACCR            (ETH_BASE + 0x1100)

#define ETH_DMACS            (ETH_BASE + 0x1160)
#define ETH_DMACS_TPS        (1u << 1)
#define ETH_DMACS_RPS        (1u << 8)

#define ETH_DMACTXCR_ST       (1u << 0)
#define ETH_DMACRXCR_SR       (1u << 0)
#define ETH_DMACRXCR_RBSZ(s)  (((s) & 0x3fffu) << 1)

/* Descriptors: four words each, buffer-1 address in des0, buffer-1 length in
 * des2[13:0], ownership in des3[31].
 */

#define ETH_DES2_LEN_MASK     0x00003fffu
#define ETH_TDES3_OWN         (1u << 31)
#define ETH_TDES3_FD          (1u << 29)
#define ETH_TDES3_LD          (1u << 28)
#define ETH_RDES3_OWN         (1u << 31)
#define ETH_RDES3_BUF1V       (1u << 24)
#define ETH_RDES3_ES          (1u << 15)
#define ETH_RDES3_FL_MASK     0x00007fffu

/* RCC: everything below was verified on the board against a working link. */

#define RCC_CCIPR2            (STM32_RCC_BASE + 0x0148)
#define RCC_AHB5RSTR          (STM32_RCC_BASE + 0x0220)
#define RCC_AHB5ENSR          (STM32_RCC_BASE + 0x0a60)

#define CCIPR2_ETH1CLKSEL_M   0x00003000u  /* bits 13:12 */
#define CCIPR2_ETH1SEL_M      0x00070000u  /* bits 18:16 */
#define CCIPR2_ETH1REFCLKSEL  (1u << 20)
#define CCIPR2_ETH1GTXCLKSEL  (1u << 24)
#define ETH1SEL_RMII          4

#define AHB5RSTR_ETH1RST      (1u << 25)
#define AHB5EN_ETH1MACEN      (1u << 22)
#define AHB5EN_ETH1TXEN       (1u << 23)
#define AHB5EN_ETH1RXEN       (1u << 24)
#define AHB5EN_ETH1EN         (1u << 25)

/* RIF: the four AHB5 gates are RIF-protected, and a secure CPU only reaches
 * SECURE regions, so this has to be opened before the gates are written.
 */

/* RISAF instances guarding the RAM the descriptors and buffers live in.
 * AHB3PERIPH_BASE_S = 0x54020000; IASR sits at offset 0x008.  A non-zero bit
 * here means a master was refused access to that region, which is how a DMA
 * can stall forever without ever raising a bus error of its own.
 */

#define RISAF2_IASR           0x54027008u
#define RISAF3_IASR           0x54028008u
#define RISAF7_IASR           0x5402c008u

#define ETH1_RIF_MASTER_INDEX 6
#define RIFSC_RIMC_ATTR(n)    (STM32_RIFSC_BASE + 0x0c10 + ((n) << 2))
#define RIFSC_SECCFGR1        (STM32_RIFSC_BASE + 0x0014)
#define RIFSC_PRIVCFGR1       (STM32_RIFSC_BASE + 0x0034)
#define ETH1_RISC_BIT         (1u << 28)
#define RIFSC_ATTR_MCID(n)    ((n) << 4)
#define RIFSC_ATTR_MSEC       (1u << 8)
#define RIFSC_ATTR_MPRIV      (1u << 9)

/* MDIO: CR=4 divides the 200 MHz CSR clock by 102, giving 1.96 MHz, which is
 * the fastest setting inside the 2.5 MHz the standard permits.  CR=0 would
 * run MDC at 4.76 MHz and the PHY would never answer.
 */

#define MDIOAR_GB             (1u << 0)
#define MDIOAR_GOC_READ       (3u << 2)
#define MDIOAR_GOC_WRITE      (1u << 1)
#define MDIOAR_CR_SHIFT       8
#define MDIOAR_RDA_SHIFT      16
#define MDIOAR_PA_SHIFT       21
#define MDIOAR_RDA_M          0x001f0000u
#define MDIOAR_PA_M           0x03e00000u

#define ETH_MDIO_CR           4
#define ETH_MDIO_TIMEOUT      100000

/* PHY (Motorcomm YT8512C at address 0: PHYID1 reads 0x0000 and the model
 * number lives entirely in PHYID2, which reads 0x0128).
 */

#define MII_BMCR              0x00
#define MII_BMSR              0x01
#define MII_ANAR              0x04
#define MII_ANLPAR            0x05

#define BMCR_RESET            (1u << 15)
#define BMCR_ANENABLE         (1u << 12)
#define BMCR_ANRESTART        (1u << 9)

#define BMSR_LSTATUS          (1u << 2)
#define BMSR_ANEGCOMPLETE     (1u << 5)

#define ETH_PHY_ADDR          0
#define ETH_LINK_POLLS        60
#define ETH_LINK_POLL_MS      100

/* Rings and buffers */

#define ETH_NTXDESC           4
#define ETH_NRXDESC           4
#define ETH_PKTBUF            1536

#define ETH_WORKER_STACKSIZE  2048
#define ETH_WORKER_PRIORITY   100
#define ETH_WORKER_INTERVAL   20

/* Fixed station address.  The board has no EEPROM for one and NuttX needs an
 * address that does not change between boots, so this is the locally
 * administered range and the last two bytes are the only ones that vary.
 */

static const uint8_t g_eth_macaddr[6] =
{
  0x02, 0x00, 0x00, 0x12, 0x34, 0x56
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The channel takes the low three bits of the descriptor list address as
 * zero - RM0486, Rx/Tx Descriptor List Address: "The DMA ignores the LSB bits
 * (2:0) for 64-bit bus width and internally takes these bits as all-zero" - so
 * a ring that lands on a four byte boundary, which is all a plain uint32_t
 * array asks for, is fetched from four bytes earlier than intended.  The
 * channel then reads a shifted word, finds OWN clear and suspends with
 * "receive buffer unavailable" while every descriptor in memory looks perfect.
 * The alignment below is what stops that from depending on where the linker
 * happened to put the structure.
 */

struct stm32n6_desc_s
{
  uint32_t des0;                        /* Buffer 1 address             */
  uint32_t des1;                        /* Buffer 2 address (unused)    */
  uint32_t des2;                        /* buffer 1 length in [13:0]    */
  uint32_t des3;                        /* control and status           */
};

/* Sixteen bytes is the stride the channel walks, so the type itself must stay
 * sixteen bytes.  Aligning the type instead rounds sizeof() up to the
 * alignment and silently doubles the spacing between descriptors: the channel
 * then reads half of every entry and reports "receive buffer unavailable"
 * while the software view looks perfect.
 */

_Static_assert(sizeof(struct stm32n6_desc_s) == 16,
               "the channel walks sixteen byte descriptors");

struct stm32n6_ethmac_s
{
  struct net_driver_s dev;              /* NuttX network device         */

  struct stm32n6_desc_s txdesc[ETH_NTXDESC] __attribute__((aligned(16)));
  struct stm32n6_desc_s rxdesc[ETH_NRXDESC] __attribute__((aligned(16)));

  uint8_t txbuf[ETH_NTXDESC][ETH_PKTBUF] __attribute__((aligned(32)));
  uint8_t rxbuf[ETH_NRXDESC][ETH_PKTBUF] __attribute__((aligned(32)));

  uint8_t txhead;                       /* next descriptor to fill      */
  uint8_t rxhead;                       /* next descriptor to reclaim   */
  uint32_t rxcnt;                       /* frames handed to the stack   */
  uint32_t txcnt;                       /* frames handed to the DMA     */
  uint32_t diag;                        /* worker tick counter          */
  uint32_t crsdv_edges;                 /* accumulated CRS_DV transitions */
  uint32_t rxd0_edges;                  /* accumulated RXD0 transitions  */
  uint32_t rxd1_edges;                  /* accumulated RXD1 transitions  */
  bool    bifup;                        /* interface is up              */
  bool    running;                      /* worker keeps going           */
};

static struct stm32n6_ethmac_s g_ethmac;

/* A handful of frames are reported in full, so that what is on the wire can be
 * compared with what the peer says it sent.  Bounded on purpose: a log line
 * costs milliseconds and the receive ring is only four deep.
 */

static int g_eth_rxlogs;
static int g_eth_txlogs;
static uint32_t g_eth_txbusy;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void stm32n6_ethreset(void);
static void stm32n6_ethgpio(void);
static int  stm32n6_mdio_read(int phy, int reg);
static int  stm32n6_mdio_write(int phy, int reg, uint16_t value);
static void stm32n6_phy_reset(void);
static int  stm32n6_phy_wait_link(void);
static int  stm32n6_soft_reset(void);
static void stm32n6_mac_config(void);
static int  stm32n6_ring_init(void);
static int  stm32n6_transmit(struct stm32n6_ethmac_s *priv);
static void stm32n6_receive(struct stm32n6_ethmac_s *priv);
static int  stm32n6_txpoll(struct net_driver_s *dev);
static int  stm32n6_ifup(struct net_driver_s *dev);
static int  stm32n6_ifdown(struct net_driver_s *dev);
static void stm32n6_txavail(struct net_driver_s *dev);
static int  stm32n6_worker(int argc, char *argv[]);
static int  stm32n6_pin_edges(uint32_t pinset, int us);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Bring ETH1 out of reset on the HCLK-derived kernel clock, RMII. */

static void stm32n6_ethreset(void)
{
  uint32_t regval;

  /* RIFSC clock, then open the ETH1 master and slave attributes.  Without
   * this the gate writes below are dropped and every ETH1 access reads zero,
   * which is how this whole bring-up started.
   */

  putreg32(RCC_AHB3ENR_RIFSCEN, STM32_RCC_AHB3ENSR);

  putreg32(RIFSC_ATTR_MCID(1) | RIFSC_ATTR_MSEC | RIFSC_ATTR_MPRIV,
           RIFSC_RIMC_ATTR(ETH1_RIF_MASTER_INDEX));
  modifyreg32(RIFSC_SECCFGR1, 0, ETH1_RISC_BIT);
  modifyreg32(RIFSC_PRIVCFGR1, 0, ETH1_RISC_BIT);

  /* Hold the MAC in reset while the PHY interface is chosen. */

  putreg32(getreg32(RCC_AHB5RSTR) | AHB5RSTR_ETH1RST, RCC_AHB5RSTR);

  regval  = getreg32(RCC_CCIPR2);
  regval &= ~CCIPR2_ETH1CLKSEL_M;
  regval &= ~CCIPR2_ETH1SEL_M;
  regval |= (ETH1SEL_RMII << 16);
  regval &= ~(CCIPR2_ETH1GTXCLKSEL | CCIPR2_ETH1REFCLKSEL);
  putreg32(regval, RCC_CCIPR2);

  /* Open the four gates through the write-one-to-set alias. */

  putreg32(AHB5EN_ETH1MACEN | AHB5EN_ETH1TXEN |
           AHB5EN_ETH1RXEN  | AHB5EN_ETH1EN, RCC_AHB5ENSR);

  putreg32(getreg32(RCC_AHB5RSTR) & ~AHB5RSTR_ETH1RST, RCC_AHB5RSTR);
  up_mdelay(1);
}

static void stm32n6_ethgpio(void)
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

static int stm32n6_mdio_read(int phy, int reg)
{
  uint32_t mdioar;
  uint32_t timeout = ETH_MDIO_TIMEOUT;

  mdioar  = ((uint32_t)phy << MDIOAR_PA_SHIFT) & MDIOAR_PA_M;
  mdioar |= ((uint32_t)reg << MDIOAR_RDA_SHIFT) & MDIOAR_RDA_M;
  mdioar |= MDIOAR_GOC_READ;
  mdioar |= ((uint32_t)ETH_MDIO_CR << MDIOAR_CR_SHIFT);
  mdioar |= MDIOAR_GB;

  putreg32(mdioar, ETH_MACMDIOAR);

  while (timeout-- > 0)
    {
      if ((getreg32(ETH_MACMDIOAR) & MDIOAR_GB) == 0)
        {
          return (int)(getreg32(ETH_MACMDIODR) & 0xffff);
        }
    }

  return -ETIMEDOUT;
}

static int stm32n6_mdio_write(int phy, int reg, uint16_t value)
{
  uint32_t mdioar;
  uint32_t timeout = ETH_MDIO_TIMEOUT;

  putreg32(value, ETH_MACMDIODR);

  mdioar  = ((uint32_t)phy << MDIOAR_PA_SHIFT) & MDIOAR_PA_M;
  mdioar |= ((uint32_t)reg << MDIOAR_RDA_SHIFT) & MDIOAR_RDA_M;
  mdioar |= MDIOAR_GOC_WRITE;
  mdioar |= ((uint32_t)ETH_MDIO_CR << MDIOAR_CR_SHIFT);
  mdioar |= MDIOAR_GB;

  putreg32(mdioar, ETH_MACMDIOAR);

  while (timeout-- > 0)
    {
      if ((getreg32(ETH_MACMDIOAR) & MDIOAR_GB) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/* PF5 is the PHY reset, active low, with a 4.7k pull-up on the board. */

static void stm32n6_phy_reset(void)
{
  stm32n6_gpiowrite(GPIO_ETH1_PHY_RST, false);
  up_mdelay(20);
  stm32n6_gpiowrite(GPIO_ETH1_PHY_RST, true);
  up_mdelay(100);
}

/* Wait for auto-negotiation.  BMSR bit 2 latches low, so every sample reads
 * the register twice: the first read clears the latch and the second reports
 * the current state.
 */

static int stm32n6_phy_wait_link(void)
{
  int i;

  for (i = 0; i < ETH_LINK_POLLS; i++)
    {
      int bmsr;

      stm32n6_mdio_read(ETH_PHY_ADDR, MII_BMSR);
      bmsr = stm32n6_mdio_read(ETH_PHY_ADDR, MII_BMSR);

      if (bmsr > 0 && (bmsr & BMSR_LSTATUS) != 0)
        {
          return OK;
        }

      up_mdelay(ETH_LINK_POLL_MS);
    }

  return -ETIMEDOUT;
}

static void stm32n6_mac_config(void)
{
  uint32_t regval;

  /* Station address.  AE (bit 31 of the high register) enables the filter. */

  putreg32(((uint32_t)g_eth_macaddr[4] << 8) | g_eth_macaddr[5] | (1u << 31),
           ETH_MACA0HR);
  putreg32(((uint32_t)g_eth_macaddr[0] << 24) |
           ((uint32_t)g_eth_macaddr[1] << 16) |
           ((uint32_t)g_eth_macaddr[2] << 8)  |
           (uint32_t)g_eth_macaddr[3], ETH_MACA0LR);

  /* Promiscuous, so ARP and DHCP replies are not filtered before the driver
   * is known to work.  Tighten to a hash filter later if desired.
   */

  putreg32(ETH_MACPFR_PR, ETH_MACPFR);

  /* Store and forward on both queues so a descriptor never completes on a
   * partial frame, then enable queue 0.
   */

  putreg32(ETH_RXQC0R_RXQ0EN_GT, ETH_MACRXQC0R);
  putreg32(ETH_MTLRXQDMAMR_CH0, ETH_MTLRXQDMAMR);

  putreg32(ETH_MTLTXQOMR_TSF | ETH_MTLTXQOMR_TXQEN | ETH_MTLTXQOMR_TQS_2K,
           ETH_MTLTXQOMR);
  putreg32(ETH_MTLRXQOMR_RSF | 3u | ETH_MTLRXQOMR_RQS_4K, ETH_MTLRXQOMR);

  /* MTL arbitration and the DMA system bus mode, both left at reset until
   * now.  Neither is obviously receive related, but the vendor programs them
   * on every initialisation and matching the known-good example costs
   * nothing.
   */

  putreg32(ETH_MTLOMR_VENDOR, ETH_MTLOMR);
  putreg32(ETH_DMASBMR_VENDOR, ETH_DMASBMR);
  putreg32(ETH_MAC1USTCR_VAL, ETH_MAC1USTCR);

  /* 100 Mbit/s full duplex, transmitter and receiver on. */

  regval = getreg32(ETH_MACCR);
  regval |= ETH_MACCR_DM | ETH_MACCR_FES | ETH_MACCR_PS;
  putreg32(regval, ETH_MACCR);

  /* Start the RMON receive counters from zero so the diagnostic below only
   * reports traffic seen during this boot.
   */

  putreg32(ETH_MMCCR_CR, ETH_MMCCR);
}

/* Software reset of the whole ETH1 core.
 *
 * DMAMR.SWR is not a DMA-only reset: it resets the MAC registers too.
 * Running it after the MAC has been configured silently wipes MACCR, the
 * packet filter and both MTL queue modes back to zero, which leaves the
 * receivers disabled while every ring and descriptor still looks healthy.
 * That is exactly how the first version of this driver failed: rxqdr stayed
 * at zero with no DMA error at all, because the MAC was not receiving.
 *
 * It also only completes once the RMII reference clock is running, so the
 * caller has to have released the PHY reset and waited for link first.
 */

static int stm32n6_soft_reset(void)
{
  uint32_t timeout = ETH_MDIO_TIMEOUT;

  putreg32(ETH_DMAMR_SWR, ETH_DMAMR);

  while (timeout-- > 0)
    {
      if ((getreg32(ETH_DMAMR) & ETH_DMAMR_SWR) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

static int stm32n6_ring_init(void)
{
  int i;

  /* Build both rings.  Every buffer starts out owned by the application and
   * every address is written before ownership is handed over.
   */

  for (i = 0; i < ETH_NRXDESC; i++)
    {
      g_ethmac.rxdesc[i].des0 = (uint32_t)&g_ethmac.rxbuf[i][0];
      g_ethmac.rxdesc[i].des1 = 0;
      g_ethmac.rxdesc[i].des2 = 0;
      g_ethmac.rxdesc[i].des3 = 0;
    }

  for (i = 0; i < ETH_NTXDESC; i++)
    {
      g_ethmac.txdesc[i].des0 = 0;
      g_ethmac.txdesc[i].des1 = 0;
      g_ethmac.txdesc[i].des2 = 0;

      /* Owned by software, not by the channel.  Handing the whole transmit
       * ring to the DMA at start-up - which is what this did - makes every
       * later transmit() find the next descriptor still owned and bail out
       * with -EBUSY, so nothing is ever sent and the count stays at zero.
       * Ownership moves to the channel one descriptor at a time, when the
       * buffer in it is filled.
       */

      g_ethmac.txdesc[i].des3 = 0;
    }

  g_ethmac.txhead = 0;
  g_ethmac.rxhead = 0;

  up_clean_dcache((uintptr_t)g_ethmac.txdesc,
                  (uintptr_t)g_ethmac.txdesc + sizeof(g_ethmac.txdesc));
  up_clean_dcache((uintptr_t)g_ethmac.rxdesc,
                  (uintptr_t)g_ethmac.rxdesc + sizeof(g_ethmac.rxdesc));

  /* Program the rings, then hand the receive ring to the DMA by giving it
   * ownership of every buffer.
   */

  putreg32((ETH_NTXDESC - 1), ETH_DMACTXRLR);
  putreg32((ETH_NRXDESC - 1), ETH_DMACRXRLR);

  putreg32((uint32_t)g_ethmac.txdesc, ETH_DMACTXDLAR);
  putreg32((uint32_t)g_ethmac.rxdesc, ETH_DMACRXDLAR);

  for (i = 0; i < ETH_NRXDESC; i++)
    {
      g_ethmac.rxdesc[i].des3 = ETH_RDES3_OWN | ETH_RDES3_BUF1V;
    }

  up_clean_dcache((uintptr_t)g_ethmac.rxdesc,
                  (uintptr_t)g_ethmac.rxdesc + sizeof(g_ethmac.rxdesc));

  /* Receive buffer size, then start both engines. */

  putreg32(ETH_DMACRXCR_RXPBL_32 | ETH_DMACRXCR_RBSZ(ETH_PKTBUF),
           ETH_DMACRXCR);

  /* The tail pointer is the address of the last descriptor the DMA may take
   * and the DMA suspends the moment its current descriptor pointer equals the
   * tail (RM0486: "the DMA enters the Suspend state when this condition
   * occurs").  Writing the ring base there therefore says "you own nothing":
   * the FIFO fills with frames, no descriptor is ever completed and no error
   * bit is raised.  One past the end means "one complete ring is available"
   * and the DMA wraps around after the last descriptor, which is what a ring
   * that is permanently full of free buffers wants.
   */

  putreg32((uint32_t)&g_ethmac.rxdesc[ETH_NRXDESC], ETH_DMACRXDTPR);

  /* Start in the same order the vendor HAL does: flush the transmit queue,
   * start both DMA engines, clear the stopped flags, and only then let the
   * MAC transmit and receive.  Enabling the MAC first, as the earlier version
   * did, leaves the receive path enabled before the DMA is ready for it.
   */

  modifyreg32(ETH_MTLTXQOMR, 0, ETH_MTLTXQOMR_FTQ);
  putreg32(ETH_DMACTXCR_TXPBL_32 | ETH_DMACTXCR_ST, ETH_DMACTXCR);
  putreg32(ETH_DMACRXCR_RXPBL_32 | ETH_DMACRXCR_RBSZ(ETH_PKTBUF) |
           ETH_DMACRXCR_SR, ETH_DMACRXCR);
  modifyreg32(ETH_DMACS, 0, ETH_DMACS_TPS | ETH_DMACS_RPS);

  modifyreg32(ETH_MACCR, 0, ETH_MACCR_RE | ETH_MACCR_TE);

  /* Read the two list addresses back.  The channel clears the low three bits
   * and does not complain, so a ring that is only four byte aligned looks
   * perfectly healthy from software and receives nothing at all.  Comparing
   * the readback against the address the compiler chose catches that at
   * start-up instead of after a round of register archaeology.
   */

  if (getreg32(ETH_DMACRXDLAR) != (uint32_t)(uintptr_t)g_ethmac.rxdesc ||
      getreg32(ETH_DMACTXDLAR) != (uint32_t)(uintptr_t)g_ethmac.txdesc)
    {
      syslog(LOG_ERR, "eth1: ring address truncated by the channel "
             "(rx soft=%08lx hard=%08lx, tx soft=%08lx hard=%08lx)\n",
             (unsigned long)(uintptr_t)g_ethmac.rxdesc,
             (unsigned long)getreg32(ETH_DMACRXDLAR),
             (unsigned long)(uintptr_t)g_ethmac.txdesc,
             (unsigned long)getreg32(ETH_DMACTXDLAR));
    }

  return OK;
}

static int stm32n6_transmit(struct stm32n6_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct stm32n6_desc_s *txdesc;
  uint32_t txlen;
  int ndx;

  if (dev->d_len == 0)
    {
      return OK;
    }

  ndx    = priv->txhead;
  txdesc = &priv->txdesc[ndx];

  /* A descriptor that still carries OWN is owned by the DMA. */

  if ((txdesc->des3 & ETH_TDES3_OWN) != 0)
    {
      g_eth_txbusy++;
      return -EBUSY;
    }

  if (dev->d_len > ETH_PKTBUF)
    {
      nerr("ERROR: packet too large: %u\n", (unsigned)dev->d_len);
      return -EMSGSIZE;
    }

  memcpy(&priv->txbuf[ndx][0], dev->d_buf, dev->d_len);

  /* A minimum size frame is sixty bytes without the checksum.  An ARP reply
   * is forty two, and a receiver is entitled to drop anything shorter as a
   * runt, so the tail is padded here rather than trusted to the MAC.
   */

  if (dev->d_len < 60)
    {
      memset(&priv->txbuf[ndx][dev->d_len], 0, 60 - dev->d_len);
      txlen = 60;
    }
  else
    {
      txlen = dev->d_len;
    }

  /* The buffer contents must reach memory before the descriptor points at
   * them and before ownership passes to the DMA.
   */

  up_clean_dcache((uintptr_t)&priv->txbuf[ndx][0],
                  (uintptr_t)&priv->txbuf[ndx][0] + txlen);

  txdesc->des0 = (uint32_t)&priv->txbuf[ndx][0];
  txdesc->des2 = txlen & ETH_DES2_LEN_MASK;
  txdesc->des3 = ETH_TDES3_OWN | ETH_TDES3_FD | ETH_TDES3_LD;

  up_clean_dcache((uintptr_t)txdesc, (uintptr_t)txdesc + sizeof(*txdesc));

  if (g_eth_txlogs < 8 && priv->dev.d_ipaddr != 0)
    {
      g_eth_txlogs++;
      syslog(LOG_INFO, "eth1: tx %d len=%u\n", g_eth_txlogs,
             (unsigned)dev->d_len);
    }

  priv->txhead = (ndx + 1) % ETH_NTXDESC;
  priv->txcnt++;

  /* Kick the transmit engine.  Writing the tail pointer is the doorbell. */

  /* One past the last descriptor given to the DMA.  When the index wraps the
   * ring is completely filled, so the address just past the array is the
   * right doorbell; handing over the base instead would put the tail behind
   * the current pointer and the DMA would quietly stop.
   */

  putreg32((uint32_t)&priv->txdesc[priv->txhead == 0 ?
                                   ETH_NTXDESC : priv->txhead],
           ETH_DMACTXDTPR);

  dev->d_len = 0;
  dev->d_buf = NULL;
  return OK;
}

static void stm32n6_receive(struct stm32n6_ethmac_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  int i;

  for (i = 0; i < ETH_NRXDESC; i++)
    {
      struct stm32n6_desc_s *rxdesc = &priv->rxdesc[priv->rxhead];
      uint32_t des3;
      uint32_t len;

      up_invalidate_dcache((uintptr_t)rxdesc,
                           (uintptr_t)rxdesc + sizeof(*rxdesc));
      des3 = rxdesc->des3;

      if ((des3 & ETH_RDES3_OWN) != 0)
        {
          break;                        /* DMA still owns it */
        }

      len = des3 & ETH_RDES3_FL_MASK;

      if ((des3 & ETH_RDES3_ES) != 0 || len == 0 || len > ETH_PKTBUF)
        {
          /* Error or nonsense length: recycle and skip. */

          goto recycle;
        }

      up_invalidate_dcache((uintptr_t)&priv->rxbuf[priv->rxhead][0],
                           (uintptr_t)&priv->rxbuf[priv->rxhead][0] + len);

      dev->d_buf = &priv->rxbuf[priv->rxhead][0];
      dev->d_len = len;

      /* Dispatch by Ethernet type, exactly as the other NuttX MAC drivers in
       * this tree do; the network layer expects each protocol to be handed
       * its own frames.
       */

      {
        uint16_t etype = ((struct eth_hdr_s *)dev->d_buf)->type;

        if (g_eth_rxlogs < 8 && len >= 14 && g_ethmac.dev.d_ipaddr != 0)
          {
            FAR const uint8_t *p = (FAR const uint8_t *)dev->d_buf;

            g_eth_rxlogs++;
            syslog(LOG_INFO, "eth1: rx %d len=%u type=%04x "
                   "dst=%02x:%02x:%02x:%02x:%02x:%02x "
                   "src=%02x:%02x:%02x:%02x:%02x:%02x\n",
                   g_eth_rxlogs, (unsigned)len, (unsigned)etype,
                   p[0], p[1], p[2], p[3], p[4], p[5],
                   p[6], p[7], p[8], p[9], p[10], p[11]);
          }

        if (etype == HTONS(ETHTYPE_IP))
          {
            ipv4_input(dev);
          }
#ifdef CONFIG_NET_IPv6
        else if (etype == HTONS(ETHTYPE_IP6))
          {
            ipv6_input(dev);
          }
#endif
#ifdef CONFIG_NET_ARP
        else if (etype == HTONS(ETHTYPE_ARP))
          {
            arp_input(dev);
          }
#endif
      }

      if (dev->d_len > 0)
        {
          stm32n6_transmit(priv);
        }

      dev->d_buf = NULL;
      dev->d_len = 0;

recycle:
      rxdesc->des0 = (uint32_t)&priv->rxbuf[priv->rxhead][0];
      rxdesc->des2 = 0;
      rxdesc->des3 = ETH_RDES3_OWN | ETH_RDES3_BUF1V;
      up_clean_dcache((uintptr_t)rxdesc,
                      (uintptr_t)rxdesc + sizeof(*rxdesc));

      priv->rxhead = (priv->rxhead + 1) % ETH_NRXDESC;
      priv->rxcnt++;
    }

  /* Same "one past the end" doorbell as at start-up: every descriptor has
   * been handed back by now.
   */

  putreg32((uint32_t)&priv->rxdesc[ETH_NRXDESC], ETH_DMACRXDTPR);
}

static int stm32n6_txpoll(struct net_driver_s *dev)
{
  struct stm32n6_ethmac_s *priv = (struct stm32n6_ethmac_s *)dev->d_private;

  if (dev->d_len > 0)
    {
      stm32n6_transmit(priv);
    }

  /* Returning non-zero stops the poll. */

  return -EBUSY;
}

static int stm32n6_ifup(struct net_driver_s *dev)
{
  struct stm32n6_ethmac_s *priv = (struct stm32n6_ethmac_s *)dev->d_private;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)(dev->d_ipaddr >> 24) & 0xff, (int)(dev->d_ipaddr >> 16) & 0xff,
        (int)(dev->d_ipaddr >> 8) & 0xff,  (int)dev->d_ipaddr & 0xff);

  if (stm32n6_ring_init() < 0)
    {
      syslog(LOG_ERR, "eth1: ring setup failed\n");
      return -EIO;
    }

  syslog(LOG_INFO, "eth1: rings started "
         "(txdlAR=%08lx rxdlAR=%08lx dmasbmr=%08lx)\n",
         (unsigned long)getreg32(ETH_DMACTXDLAR),
         (unsigned long)getreg32(ETH_DMACRXDLAR),
         (unsigned long)getreg32(ETH_BASE + 0x1004));

  priv->bifup = true;
  return OK;
}

static int stm32n6_ifdown(struct net_driver_s *dev)
{
  struct stm32n6_ethmac_s *priv = (struct stm32n6_ethmac_s *)dev->d_private;

  ninfo("Taking down\n");

  priv->bifup = false;
  putreg32(0, ETH_DMACTXCR);
  putreg32(0, ETH_DMACRXCR);
  return OK;
}

static void stm32n6_txavail(struct net_driver_s *dev)
{
  struct stm32n6_ethmac_s *priv = (struct stm32n6_ethmac_s *)dev->d_private;

  if (!priv->bifup)
    {
      return;
    }

  stm32n6_receive(priv);

  dev->d_buf = &priv->txbuf[priv->txhead][0];
  dev->d_len = 0;
  devif_poll(dev, stm32n6_txpoll);
  dev->d_buf = NULL;
  dev->d_len = 0;
}

/* Count transitions on a pin by sampling its input data register.  CRS_DV
 * and the two receive data lines are driven by the PHY, so activity there
 * proves the receive path is alive up to the MAC and moves the fault into
 * the MAC configuration, while silence means the PHY never passed a frame on.
 */

static int stm32n6_pin_edges(uint32_t pinset, int us)
{
  bool last = stm32n6_gpioread(pinset);
  int  edges = 0;

  while (us-- > 0)
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

static int stm32n6_worker(int argc, char *argv[])
{
  struct stm32n6_ethmac_s *priv = &g_ethmac;

  while (priv->running)
    {
      /* While nothing has been received, spend almost the whole tick
       * watching the RMII receive lines.  The earlier version sampled for
       * 500 us every five seconds - a duty cycle of 0.01% - so a 5 us frame
       * would essentially never be caught and a silent counter proved
       * nothing.  Accumulating across the whole interval turns the counter
       * into a real instrument.
       */

      /* Drain the receive ring first, and do it on every pass.
       *
       * This call used to be missing altogether: the loop only chose between
       * two paths and neither of them touched the ring, so the receive count
       * could never leave zero and the descriptors stayed owned by the channel
       * for ever.  Everything the hardware reported looked healthy while that
       * was so - the early receive interrupt was set and the descriptor ring
       * had wrapped back to its first entry - because the DMA had already
       * filled and closed all four buffers and was simply waiting to be given
       * more.
       */

      stm32n6_receive(priv);

      /* Then let the stack hand over anything it has queued. */

      stm32n6_txavail(&priv->dev);

      if (priv->rxcnt == 0)
        {
          /* Nothing has arrived yet, so keep measuring the RMII lines.  The
           * sampling paces the loop; the counters are the only evidence that
           * the pads are alive when the channel says nothing is coming.
           */

          priv->crsdv_edges += stm32n6_pin_edges(GPIO_ETH1_CRS_DV, 8000);
          priv->rxd0_edges   += stm32n6_pin_edges(GPIO_ETH1_RXD0, 5000);
          priv->rxd1_edges   += stm32n6_pin_edges(GPIO_ETH1_RXD1, 5000);
        }
      else
        {
          usleep(ETH_WORKER_INTERVAL * 1000);
        }

      /* Every two seconds report whether the DMA is actually moving.  A
       * receive count that stays at zero together with an owned descriptor
       * in every slot means the ring was never handed over, while a stuck
       * DMACSR usually means the channel was started before the queue.
       */

      priv->diag++;

      if (priv->dev.d_ipaddr != 0 && (priv->diag % 250) == 0)
        {
          /* Every 30 s, not every 2 s: the earlier cadence flooded the
           * console.  MTLRXQDR is the decisive one - a non-zero packet
           * count there means frames reach the MAC FIFO but the DMA never
           * picks them up, while zero means nothing is arriving at all.
           */

          syslog(LOG_INFO, "eth1: rx=%lu tx=%lu dmacsr=%08lx "
                 "maccr=%08lx pfr=%08lx rxqomr=%08lx rxqmap=%08lx "
                 "rxqdr=%08lx des3=%08lx\n",
                 (unsigned long)priv->rxcnt, (unsigned long)priv->txcnt,
                 (unsigned long)getreg32(ETH_BASE + 0x1160),
                 (unsigned long)getreg32(ETH_MACCR),
                 (unsigned long)getreg32(ETH_MACPFR),
                 (unsigned long)getreg32(ETH_MTLRXQOMR),
                 (unsigned long)getreg32(ETH_MTLRXQDMAMR),
                 (unsigned long)getreg32(ETH_MTLRXQDR),
                 (unsigned long)priv->rxdesc[0].des3);

          /* PF7 carries the PHY's 50 MHz reference clock and is known to be
           * running, so measuring it with the same counter is a self-check on
           * the instrument: if refclk reads zero then the sampling loop or the
           * pin read is at fault, not the receive path.
           */

          syslog(LOG_INFO, "eth1: pins over the last 5 s: crsdv=%lu "
                 "rxd0=%lu rxd1=%lu (refclk=%d over 500 us)\n",
                 (unsigned long)priv->crsdv_edges,
                 (unsigned long)priv->rxd0_edges,
                 (unsigned long)priv->rxd1_edges,
                 stm32n6_pin_edges(GPIO_ETH1_REF_CLK, 500));

          syslog(LOG_INFO, "eth1: mmc crcerr=%08lx alignerr=%08lx "
                 "rir=%08lx txgood=%08lx txqdr=%08lx txbusy=%lu\n",
                 (unsigned long)getreg32(ETH_MMCRCRCEPR),
                 (unsigned long)getreg32(ETH_MMCRAEPR),
                 (unsigned long)getreg32(ETH_MMCRIR),
                 (unsigned long)getreg32(ETH_MMCTPCGR),
                 (unsigned long)getreg32(ETH_MTLTXQDR),
                 (unsigned long)g_eth_txbusy);

          /* The current-descriptor pointers say whether the channels are
           * really running: an RX pointer equal to the ring address means the
           * engine is armed and waiting for a frame, while zero means it was
           * never started.
           */

          syslog(LOG_INFO, "eth1: rif iasr af2=%08lx af3=%08lx af7=%08lx "
                 "mtlomr=%08lx\n",
                 (unsigned long)getreg32(RISAF2_IASR),
                 (unsigned long)getreg32(RISAF3_IASR),
                 (unsigned long)getreg32(RISAF7_IASR),
                 (unsigned long)getreg32(ETH_MTLOMR));

          syslog(LOG_INFO, "eth1: dma daccr=%08lx cur_rx=%08lx cur_tx=%08lx "
                 "ring_rx=%08lx\n",
                 (unsigned long)getreg32(ETH_DMACCR),
                 (unsigned long)getreg32(ETH_DMACCARXDR),
                 (unsigned long)getreg32(ETH_DMACCATXDR),
                 (unsigned long)(uintptr_t)priv->rxdesc);

          syslog(LOG_INFO, "eth1: dma rxcr=%08lx rxlar=%08lx rxdtpr=%08lx "
                 "rxrlr=%08lx\n",
                 (unsigned long)getreg32(ETH_DMACRXCR),
                 (unsigned long)getreg32(ETH_DMACRXDLAR),
                 (unsigned long)getreg32(ETH_DMACRXDTPR),
                 (unsigned long)getreg32(ETH_DMACRXRLR));

          /* The process state in the DMA debug register is the one reading
           * that says where the receive engine actually sits: 0 stopped,
           * 1 fetching a descriptor, 3 waiting for the queue to have data,
           * 4 suspended with no descriptor, 7 transferring packet data.
           */

          syslog(LOG_INFO, "eth1: dma dsr=%08lx rps0=%lu tps0=%lu "
                 "mpocr=%08lx sbmsr=%08lx u1us=%08lx\n",
                 (unsigned long)getreg32(ETH_DMADSR),
                 (unsigned long)((getreg32(ETH_DMADSR) >> 8) & 0xf),
                 (unsigned long)((getreg32(ETH_DMADSR) >> 12) & 0xf),
                 (unsigned long)getreg32(ETH_MTLRXQMPOCR),
                 (unsigned long)getreg32(ETH_DMASBMR),
                 (unsigned long)getreg32(ETH_MAC1USTCR));

          /* The channel writes the descriptors behind the cache, so drop the
           * lines before looking at them; a cached copy keeps reporting OWN
           * long after the DMA has taken the buffer back.
           */

          up_invalidate_dcache((uintptr_t)priv->rxdesc,
                               (uintptr_t)priv->rxdesc +
                               sizeof(priv->rxdesc));

          syslog(LOG_INFO, "eth1: des3 %08lx %08lx %08lx %08lx\n",
                 (unsigned long)priv->rxdesc[0].des3,
                 (unsigned long)priv->rxdesc[1].des3,
                 (unsigned long)priv->rxdesc[2].des3,
                 (unsigned long)priv->rxdesc[3].des3);

          priv->crsdv_edges = 0;
          priv->rxd0_edges   = 0;
          priv->rxd1_edges   = 0;
        }

      usleep(ETH_WORKER_INTERVAL * 1000);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32n6_ethinitialize(void)
{
  struct stm32n6_ethmac_s *priv = &g_ethmac;
  int bmsr;

  memset(priv, 0, sizeof(*priv));

  stm32n6_ethreset();
  stm32n6_ethgpio();
  stm32n6_phy_reset();

  bmsr = stm32n6_phy_wait_link();
  syslog(LOG_INFO, "eth1: PHY %s (bmsr=%04x)\n",
         bmsr == OK ? "link up" : "did NOT negotiate",
         stm32n6_mdio_read(ETH_PHY_ADDR, MII_BMSR) & 0xffff);

  /* The MAC and the DMA are only touched once the reference clock exists.
   * The soft reset has to come first: it clears the MAC registers as well,
   * so anything written before it is lost.
   */

  if (stm32n6_soft_reset() < 0)
    {
      syslog(LOG_ERR, "eth1: soft reset never completed (dmamr=%08lx) - "
             "no RMII reference clock\n", (unsigned long)getreg32(ETH_DMAMR));
    }

  stm32n6_mac_config();

  syslog(LOG_INFO, "eth1: MAC configured: maccr=%08lx pfr=%08lx "
         "txqomr=%08lx rxqomr=%08lx rxqmap=%08lx rxqc0r=%08lx "
         "maca0hr=%08lx maca0lr=%08lx\n",
         (unsigned long)getreg32(ETH_MACCR),
         (unsigned long)getreg32(ETH_MACPFR),
         (unsigned long)getreg32(ETH_MTLTXQOMR),
         (unsigned long)getreg32(ETH_MTLRXQOMR),
         (unsigned long)getreg32(ETH_MTLRXQDMAMR),
         (unsigned long)getreg32(ETH_MACRXQC0R),
         (unsigned long)getreg32(ETH_MACA0HR),
         (unsigned long)getreg32(ETH_MACA0LR));

  priv->dev.d_ifup    = stm32n6_ifup;
  priv->dev.d_ifdown  = stm32n6_ifdown;
  priv->dev.d_txavail = stm32n6_txavail;
  priv->dev.d_private = priv;
  priv->dev.d_buf     = NULL;
  priv->dev.d_len     = 0;

  memcpy(priv->dev.d_mac.ether.ether_addr_octet, g_eth_macaddr, 6);

  netdev_register(&priv->dev, NET_LL_ETHERNET);

  syslog(LOG_INFO, "eth1: netdev registered, MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
         g_eth_macaddr[0], g_eth_macaddr[1], g_eth_macaddr[2],
         g_eth_macaddr[3], g_eth_macaddr[4], g_eth_macaddr[5]);

  priv->running = true;
  kthread_create("eth1", ETH_WORKER_PRIORITY, ETH_WORKER_STACKSIZE,
                 stm32n6_worker, NULL);
  return OK;
}

/****************************************************************************
 * Name: arm_netinitialize
 ****************************************************************************/

void arm_netinitialize(void)
{
  stm32n6_ethinitialize();
}

#endif /* CONFIG_NET && CONFIG_NET_ETHERNET */
