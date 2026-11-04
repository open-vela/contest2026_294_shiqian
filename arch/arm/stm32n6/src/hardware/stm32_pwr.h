/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32_pwr.h
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
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32_PWR_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32_PWR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWR register addresses (AHB4 + 0x4800) */

#define STM32_PWR_BASE         (STM32_AHB4_BASE + 0x4800)

/* PWR register offsets (from CMSIS stm32n647xx.h PWR_TypeDef) */

#define STM32_PWR_CR1_OFFSET   0x000
#define STM32_PWR_CR2_OFFSET   0x004
#define STM32_PWR_CR3_OFFSET   0x008
#define STM32_PWR_CR4_OFFSET   0x00c
#define STM32_PWR_VOSCR_OFFSET 0x020
#define STM32_PWR_BDCR1_OFFSET 0x024
#define STM32_PWR_BDCR2_OFFSET 0x028
#define STM32_PWR_DBPCR_OFFSET 0x02c
#define STM32_PWR_CPUCR_OFFSET 0x030
#define STM32_PWR_SVMCR1_OFFSET 0x034
#define STM32_PWR_SVMCR2_OFFSET 0x038
#define STM32_PWR_SVMCR3_OFFSET 0x03c

/* PWR register addresses */

#define STM32_PWR_CR1          (STM32_PWR_BASE + STM32_PWR_CR1_OFFSET)
#define STM32_PWR_CR2          (STM32_PWR_BASE + STM32_PWR_CR2_OFFSET)
#define STM32_PWR_CR3          (STM32_PWR_BASE + STM32_PWR_CR3_OFFSET)
#define STM32_PWR_CR4          (STM32_PWR_BASE + STM32_PWR_CR4_OFFSET)
#define STM32_PWR_VOSCR        (STM32_PWR_BASE + STM32_PWR_VOSCR_OFFSET)
#define STM32_PWR_BDCR1        (STM32_PWR_BASE + STM32_PWR_BDCR1_OFFSET)
#define STM32_PWR_BDCR2        (STM32_PWR_BASE + STM32_PWR_BDCR2_OFFSET)
#define STM32_PWR_DBPCR        (STM32_PWR_BASE + STM32_PWR_DBPCR_OFFSET)
#define STM32_PWR_CPUCR        (STM32_PWR_BASE + STM32_PWR_CPUCR_OFFSET)
#define STM32_PWR_SVMCR1       (STM32_PWR_BASE + STM32_PWR_SVMCR1_OFFSET)
#define STM32_PWR_SVMCR2       (STM32_PWR_BASE + STM32_PWR_SVMCR2_OFFSET)
#define STM32_PWR_SVMCR3       (STM32_PWR_BASE + STM32_PWR_SVMCR3_OFFSET)

/* PWR_DBPCR bits (from CMSIS PWR_DBPCR) */

#define PWR_DBPCR_DBP           (1 << 0)  /* Disable backup domain
                                            * write protection */

/* PWR_SVMCR3 bits (VddIO2/VddIO3 supply valid + voltage range,
 * from CMSIS PWR_SVMCR3)
 */

#define PWR_SVMCR3_AVMEN        (1 << 4)  /* VDDA18ADC voltage monitor en */
#define PWR_SVMCR3_VDDIO2SV     (1 << 8)
#define PWR_SVMCR3_VDDIO3SV     (1 << 9)
#define PWR_SVMCR3_ASV          (1 << 12) /* VDDA18ADC supply valid */

/* Voltage-range selects sit in the high half of the register, not next to the
 * supply-valid bits.  Checked against CMSIS stm32n647xx.h:
 *
 *   PWR_SVMCR3_VDDIOVRSEL_Pos  = 24
 *   PWR_SVMCR3_VDDIO2VRSEL_Pos = 25
 *   PWR_SVMCR3_VDDIO3VRSEL_Pos = 26
 *
 * The earlier values here (16 and 17) are VDDIO2RDY and VDDIO3RDY, which are
 * read-only status bits, so a write of VRSEL through them was silently
 * dropped and the voltage range never got configured at all.
 */

#define PWR_SVMCR3_VDDIOVRSEL   (1 << 24)
#define PWR_SVMCR3_VDDIO2VRSEL  (1 << 25)
#define PWR_SVMCR3_VDDIO3VRSEL  (1 << 26)
#define PWR_SVMCR3_VDDIO2RDY    (1 << 16)
#define PWR_SVMCR3_VDDIO3RDY    (1 << 17)

/* PWR_SVMCR1 bits: VDDIO4 only.
 *
 * VDDIO5 used to be described here as SVMCR1 bits 9 and 25.  It is not:
 * CMSIS puts it in PWR_SVMCR2 at bit 8 (supply valid) and bit 24 (range).
 * Using the SVMCR1 spellings wrote a reserved bit and left the VDDIO5 pads
 * without supply, which is the same class of failure as the SDMMC1 puzzle in
 * 2026-09: every register reads back set and the pin still never drives.
 */

#define PWR_SVMCR1_VDDIO4SV     (1 << 8)
#define PWR_SVMCR1_VDDIO4VRSEL  (1 << 24)
#define PWR_SVMCR1_VDDIO4RDY    (1 << 16)

/* PWR_SVMCR2 bits (VDDIO5) */

#define PWR_SVMCR2_VDDIO5SV     (1 << 8)
#define PWR_SVMCR2_VDDIO5VRSEL  (1 << 24)
#define PWR_SVMCR2_VDDIO5RDY    (1 << 16)

/* PWR_VOSCR bits */

#define PWR_VOSCR_VOS           (1 << 0)  /* Voltage scaling selection */
#define PWR_VOSCR_VOSRDY        (1 << 1)  /* VOS ready */
#define PWR_VOSCR_ACTVOS        (1 << 16) /* Currently applied VOS */
#define PWR_VOSCR_ACTVOSRDY     (1 << 17) /* ACTVOS ready */

/* PWR_CPUCR bits (from CMSIS PWR_CPUCR) */

#define PWR_CPUCR_PDDS          (1 << 0)  /* Power-down deepsleep select
                                            * (0 = Stop, 1 = Standby) */
#define PWR_CPUCR_CSSF          (1 << 1)  /* Clear Standby/Stop flags
                                            * (write 1; reads as 0) */
#define PWR_CPUCR_STOPF         (1 << 8)  /* Stop flag (set by HW when Stop
                                            * mode was entered) */
#define PWR_CPUCR_SBF           (1 << 9)  /* System Standby flag */
#define PWR_CPUCR_SVOS_Pos      16        /* Stop-mode voltage-scaling sel */
#define PWR_CPUCR_SVOS_Msk      (0x1 << PWR_CPUCR_SVOS_Pos)
#define PWR_CPUCR_SVOS          PWR_CPUCR_SVOS_Msk

/* Voltage scaling levels */

#define PWR_VOS_SCALE0          0  /* Highest performance (800MHz) */
#define PWR_VOS_SCALE1          1  /* High performance */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32_PWR_H */
