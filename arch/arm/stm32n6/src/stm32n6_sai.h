/****************************************************************************
 * arch/arm/src/stm32n6/stm32n6_sai.h
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
 * STM32N6 SAI driver header.
 * Serial Audio Interface for microphone input and speaker output.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32N6_STM32N6_SAI_H
#define __ARCH_ARM_SRC_STM32N6_STM32N6_SAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SAI audio format */

struct sai_config_s
{
  uint32_t sample_rate;    /* e.g. 16000, 44100, 48000 */
  uint8_t bits_per_sample; /* 16 or 24 */
  uint8_t  channels;       /* 1=mono, 2=stereo */
  uint8_t  block_count;    /* DMA block count (2=double buffer) */
  uint16_t block_samples;  /* Samples per block */
};

/* SAI buffer callback */

typedef void (*sai_buffer_cb_t)(void *buffer, uint32_t size,
                                 void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief Initialize SAI for audio input
 * @param config Audio configuration
 * @return 0 on success
 */

int stm32n6_sai_input_init(const struct sai_config_s *config);

/**
 * @brief Start audio capture
 * @param callback Buffer-ready callback
 * @param arg Callback argument
 * @return 0 on success
 */

int stm32n6_sai_input_start(sai_buffer_cb_t callback,
                              void *arg);

/**
 * @brief Stop audio capture
 * @return 0 on success
 */

int stm32n6_sai_input_stop(void);

/**
 * @brief Deinitialize SAI input
 */

void stm32n6_sai_input_deinit(void);

/**
 * @brief Initialize SAI1 block A for playback (master transmitter)
 *
 * Drives MCLK out for the codec, so the codec needs no crystal of its own.
 * Also brings up the clock chain the block hangs off.
 *
 * @param sample_rate Sample rate in Hz (44100)
 * @param channels    1 = mono, 2 = stereo
 * @return 0 on success
 */

int stm32n6_sai_output_init(uint32_t sample_rate, uint8_t channels);

/**
 * @brief Play a buffer, blocking until the hardware has consumed it
 *
 * Samples are 16-bit and interleaved; the transfer is handed to the DMA and
 * this returns once the last one has left the shift register.
 *
 * @param samples Interleaved 16-bit samples
 * @param count   Number of samples (not bytes)
 * @return 0 on success
 */

int stm32n6_sai_output_write(FAR const int16_t *samples, uint32_t count);

/**
 * @brief Whether the transmit block is currently programmed and running
 */

bool stm32n6_sai_output_ready(void);

/**
 * @brief Stop playback and release the DMA channel
 */

void stm32n6_sai_output_stop(void);

/**
 * @brief End playback but leave the block clocking on silence
 *
 * Used instead of a stop where the codec cannot be muted: a stopped block
 * leaves the codec without MCLK, bit clock or frame sync, and that idle
 * state is heard as a hiss.  The FIFO is flushed and reloaded with zeros,
 * and SAIEN stays set.
 */

void stm32n6_sai_output_quiet(void);

/**
 * @brief Report the clock chain, for the boot log
 *
 * The audio path depends on a PLL that nothing else in the system uses, so
 * when a tone comes out at the wrong pitch these are the numbers that say
 * which stage disagrees.
 */

void stm32n6_sai_output_dump(void);

/**
 * @brief Diagnose the transmit path in one run: clock chain readback, then
 *        every kernel clock source, then a verdict
 *
 * Fills the transmit FIFO past its capacity and times how long the block
 * takes to empty it, once per kernel clock source.  A block that drains on
 * some source has a working clock generator; one that drains on none has a
 * fault at the block or pin level.
 *
 * Leaves the block enabled on the first source that produced frames.
 *
 * @return 0 if any source produces frames, -ENODEV if none does
 */

int stm32n6_sai_output_diag(void);

/**
 * @brief Status register of the transmit block, for post-transfer checks
 *
 * Read before stopping the block: stopping clears it.  Bit 0 is the overrun /
 * underrun flag, bit 2 the wrong-clock-configuration flag, bit 3 the FIFO
 * request flag; anything but the request bit set means the transfer did not
 * run cleanly.
 *
 * @return SAI_SR as read, or 0 when the block is not initialised
 */

uint32_t stm32n6_sai_output_status(void);

#endif /* __ARCH_ARM_SRC_STM32N6_STM32N6_SAI_H */
