/****************************************************************************
 * vendor/openvela/boards/atk-dnn647/src/stm32n6_audio.h
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
 * ATK-DNN647 audio output: ES8388 codec on SAI1.
 *
 ****************************************************************************/

#ifndef __VENDOR_OPENVELA_BOARDS_ATK_DNN647_SRC_STM32N6_AUDIO_H
#define __VENDOR_OPENVELA_BOARDS_ATK_DNN647_SRC_STM32N6_AUDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32n6_audio_init
 *
 * Description:
 *   Bring up the codec and the transmit path.  Idempotent: the codec is
 *   configured once and left running, because its own startup takes a
 *   hundred milliseconds and a prompt should not wait for that.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure.
 *
 ****************************************************************************/

int stm32n6_audio_init(void);

/****************************************************************************
 * Name: stm32n6_audio_play_wav
 *
 * Description:
 *   Play a 16-bit PCM WAV file from the filesystem.
 *
 *   Blocks until the file has been sent.  Playback is a single writer: a
 *   prompt is short and the speaker has one channel, so overlapping calls
 *   would only garble each other.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure.
 *
 ****************************************************************************/

int stm32n6_audio_play_wav(FAR const char *path);

/****************************************************************************
 * Name: stm32n6_audio_set_volume
 *
 * Description:
 *   Set the speaker volume, 0 to 33.  The codec's own range, not a
 *   percentage.
 *
 ****************************************************************************/

void stm32n6_audio_set_volume(uint8_t volume);

/****************************************************************************
 * Name: stm32n6_audio_dump_codec
 *
 * Description:
 *   Read every register the bring-up sequence writes and print it next to
 *   the value that was supposed to land there.
 *
 *   Writing a codec register only proves the codec acknowledged its address;
 *   it does not prove the field took effect, and the ES8388 has a documented
 *   pulse the datasheet calls a soft reset that needs settling time.  A read
 *   back is the only way to tell "configured and quiet" from "configured
 *   correctly and silent because of what comes after it".
 *
 ****************************************************************************/

void stm32n6_audio_dump_codec(void);

/****************************************************************************
 * Name: stm32n6_audio_bus_scan
 *
 * Description:
 *   Walk the codec's bit-banged control bus and print every address that
 *   acknowledges, together with the idle levels of both lines.
 *
 *   The light sensor answers at 0x1e on the same two wires through the
 *   hardware I2C4 controller, so it acts as the control: 0x1e present and
 *   0x10 absent means the wires and the bit timing are fine and the codec
 *   itself is not answering; nothing present means the software bus is at
 *   fault and the codec was never spoken to at all.
 *
 * Returned Value:
 *   The number of devices that acknowledged.
 *
 ****************************************************************************/

int stm32n6_audio_bus_scan(void);

/****************************************************************************
 * Name: stm32n6_audio_tone
 *
 * Description:
 *   Play a square wave straight out of RAM.
 *
 *   No card, no filesystem and no WAV parsing: the speaker path can be
 *   exercised on a board with nothing plugged in.  The elapsed time against
 *   the requested duration doubles as a clock check, because a wrong PLL2
 *   divider shifts it proportionally.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure.
 *
 ****************************************************************************/

int stm32n6_audio_tone(int freq_hz, int ms);

#endif /* __VENDOR_OPENVELA_BOARDS_ATK_DNN647_SRC_STM32N6_AUDIO_H */
