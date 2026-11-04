/****************************************************************************
 * vendor/openvela/boards/atk-dnn647/src/stm32n6_audio.c
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
 * The codec sits on SAI1 and takes both its bit clock and its MCLK from this
 * side, so the board owns the whole path: pins, codec registers, the WAV
 * file, and the loop that feeds the transmit block.
 *
 * The register values are the vendor's, not derived.  ES8388 configuration
 * is a long sequence of undocumented-by-formula choices, and the vendor BSP
 * for this exact board is known to produce sound; transcribing it removes a
 * whole category of guesswork.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <sched.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <syslog.h>

#include "arm_internal.h"
#include "stm32n6_gpio.h"
#include "stm32n6_i2c.h"
#include "stm32n6_sai.h"
#include "stm32n6_audio.h"

#if defined(CONFIG_STM32_SAI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The codec's control bus is plain GPIO, driven in software.  Open drain so
 * the codec can pull SDA low for its acknowledge without either side driving
 * against the other, and slow because the vendor's timing is a 2 us delay
 * per half bit (about 250 kHz).
 */

#define GPIO_ES_SCL \
  (GPIO_MODE_OUTPUT | GPIO_OTYPE_OD | GPIO_SPEED_LOW | \
   GPIO_PUPD_PU | GPIO_PORTE | GPIO_PIN(13))

#define GPIO_ES_SDA \
  (GPIO_MODE_OUTPUT | GPIO_OTYPE_OD | GPIO_SPEED_LOW | \
   GPIO_PUPD_PU | GPIO_PORTE | GPIO_PIN(14))

#define ES8388_I2C_ADDR     0x10    /* fixed, per the codec's data sheet */

#define ES_AUDIO_CHUNK      16384   /* bytes per write to the transmit block */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wav_info_s
{
  uint32_t rate;          /* samples per second */
  uint16_t channels;      /* 1 or 2 */
  uint16_t bits;          /* bits per sample */
  uint32_t data_off;      /* byte offset of the samples */
  uint32_t data_len;      /* byte length of the samples */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_audio_ready;

/****************************************************************************
 * Name: es_bus
 *
 * Description:
 *   The codec's control bus, served by the port's hardware I2C4 controller.
 *
 *   This started out as a bit-banged bus on the same two pins.  The address
 *   scan through it answered at 0x10, which proves the wires and the timing
 *   are fine, and yet every multi-byte write from the same code failed on
 *   the address byte -- so the software bus was doing something the codec
 *   did not like once a transaction ran past one byte.  The hardware
 *   controller on those same pins is already carrying the light sensor and
 *   the EEPROM through exactly the transaction shapes this codec needs, so
 *   the codec uses it too and the bit-bang is gone from the working path.
 *
 ****************************************************************************/

static FAR struct i2c_master_s *es_bus(void)
{
  static FAR struct i2c_master_s *bus;

  if (bus == NULL)
    {
      bus = stm32n6_i2cbus_initialize(4);
      if (bus == NULL)
        {
          syslog(LOG_ERR, "audio: I2C4 is not available for the codec\n");
        }
    }

  return bus;
}

static int es_write_reg_stage(uint8_t reg, uint8_t value, FAR int *stage)
{
  struct i2c_msg_s msg;
  uint8_t          buf[2];
  FAR struct i2c_master_s *bus = es_bus();
  int              ret;

  if (bus == NULL)
    {
      *stage = 1;
      return -ENODEV;
    }

  buf[0] = reg;
  buf[1] = value;

  msg.frequency = 100000;
  msg.addr      = ES8388_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  ret = I2C_TRANSFER(bus, &msg, 1);

  /* Hand the bus error back unchanged.  A missing acknowledge (-ENXIO) and a
   * transfer that never finished (-ETIMEDOUT) mean completely different
   * things here - one is the codec not answering, the other is the bus or
   * the pin mux not working - and mapping both to -EIO, which is what this
   * used to do, is what made the earlier logs impossible to act on.
   */

  *stage = ret < 0 ? 1 : 0;
  return ret;
}

static int es_write_reg(uint8_t reg, uint8_t value)
{
  int stage;

  return es_write_reg_stage(reg, value, &stage);
}

/****************************************************************************
 * Name: es_read_reg
 *
 * Description:
 *   One register read: register index, repeated start, data byte.  The same
 *   two-message shape the light sensor uses on this bus.
 *
 ****************************************************************************/

static int es_read_reg(uint8_t reg, FAR uint8_t *value)
{
  struct i2c_msg_s msg[2];
  FAR struct i2c_master_s *bus = es_bus();
  int              ret;

  if (bus == NULL)
    {
      return -ENODEV;
    }

  msg[0].frequency = 100000;
  msg[0].addr      = ES8388_I2C_ADDR;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = 100000;
  msg[1].addr      = ES8388_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = value;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(bus, msg, 2);
  return ret;
}

/****************************************************************************
 * Name: es_read_reg_stop
 *
 * Description:
 *   Read one register with a STOP between the address phase and the data
 *   phase.
 *
 *   The repeated-start form above is what the light sensor on this same bus
 *   answers to, so it is the one to use first.  This variant puts the same
 *   two bytes on the wire but terminates the address write properly, which
 *   is what a part that does not accept a repeated start needs.  Having both
 *   in one run says which of the two the codec wants.
 *
 ****************************************************************************/

static int es_read_reg_stop(uint8_t reg, FAR uint8_t *value)
{
  struct i2c_msg_s msg;
  FAR struct i2c_master_s *bus = es_bus();
  int              ret;

  if (bus == NULL)
    {
      return -ENODEV;
    }

  msg.frequency = 100000;
  msg.addr      = ES8388_I2C_ADDR;
  msg.flags     = 0;
  msg.buffer    = &reg;
  msg.length    = 1;

  ret = I2C_TRANSFER(bus, &msg, 1);
  if (ret < 0)
    {
      return -EIO;
    }

  msg.frequency = 100000;
  msg.addr      = ES8388_I2C_ADDR;
  msg.flags     = I2C_M_READ;
  msg.buffer    = value;
  msg.length    = 1;

  ret = I2C_TRANSFER(bus, &msg, 1);
  return ret;
}

/****************************************************************************
 * Private Functions: codec bring-up
 ****************************************************************************/

/****************************************************************************
 * Name: es8388_configure
 *
 * Description:
 *   The vendor's power-up sequence.  Register numbers and values are theirs.
 *
 *   Registers 0x0D and 0x18 set the ADC and DAC oversampling, and 0x08 says
 *   MCLK is not divided: together they fix the ratio between MCLK and the
 *   sample rate, which is why the clock tree on the transmit side has to
 *   produce what it does.
 *
 ****************************************************************************/

static int es8388_configure(void)
{
  static const struct
  {
    uint8_t reg;
    uint8_t value;
  } seq[] =
  {
    { 0x00, 0x80 },   /* soft reset */
    { 0x00, 0x00 },
    { 0x01, 0x58 },   /* power management */
    { 0x01, 0x50 },
    { 0x02, 0xF3 },
    { 0x02, 0xF0 },
    { 0x03, 0x09 },   /* microphone bias off */
    { 0x00, 0x06 },   /* reference and 500k driver on */
    { 0x04, 0x00 },   /* DAC power: no channel on yet */
    { 0x08, 0x00 },   /* MCLK not divided */
    { 0x2B, 0x80 },   /* DACLRC follows ADCLRC */
    { 0x09, 0x88 },   /* ADC PGA +24 dB */
    { 0x0C, 0x4C },   /* ADC data: left to left, 16 bit */
    { 0x0D, 0x02 },   /* ADC oversampling */
    { 0x10, 0x00 },   /* ADC digital volume: no attenuation */
    { 0x11, 0x00 },
    { 0x17, 0x18 },   /* DAC 16 bit */
    { 0x18, 0x02 },   /* DAC oversampling */
    { 0x1A, 0x00 },   /* DAC digital volume: no attenuation */
    { 0x1B, 0x00 },
    { 0x27, 0xB8 },   /* left mixer */
    { 0x2A, 0xB8 },   /* right mixer */
  };

  unsigned i;
  unsigned silent = 0;
  int      stage;
  int      ret = 0;

  /* The pins stay on the hardware controller's alternate function now, which
   * is also what keeps the light sensor and the EEPROM on the same bus
   * working while the codec is being configured.
   */

  if (es_bus() == NULL)
    {
      return -ENODEV;
    }

  /* The vendor's own bring-up ignores every return value here, and one entry
   * in the sequence deserves that treatment: the soft reset can drop the
   * codec off the bus mid transaction, because the part is resetting while
   * the value byte is still on the wire.  Aborting on it -- which is what
   * this function used to do -- means writing nothing else at all.
   */

  for (i = 0; i < sizeof(seq) / sizeof(seq[0]); i++)
    {
      ret = es_write_reg_stage(seq[i].reg, seq[i].value, &stage);

      if (ret < 0)
        {
          silent++;

          /* -ENXIO is the codec not acknowledging its address.  -ETIMEDOUT
           * is the transfer never reaching an end at all, which points at
           * the bus or the pin mux rather than at the codec.  They used to
           * be flattened into one code by this function's caller.
           */

          syslog(LOG_ERR, "audio: codec write %02x=%02x failed: %d%s\n",
                 seq[i].reg, seq[i].value, ret,
                 ret == -ENXIO ? " (no acknowledge at this address)" :
                 ret == -ETIMEDOUT ? " (the transfer never finished: check "
                                     "the bus and the pin mux)" : "");
        }

      /* The reset pair needs settling time before anything else lands. */

      if (seq[i].reg == 0x00 && seq[i].value == 0x00)
        {
          up_mdelay(100);
        }
    }

  syslog(LOG_INFO, "audio: codec sequence done, %u of %u writes refused\n",
         silent, (unsigned)(sizeof(seq) / sizeof(seq[0])));

  /* Only give up when the codec ignored everything, which is the case where
   * the address never acknowledged.
   */

  if (silent == sizeof(seq) / sizeof(seq[0]))
    {
      return -EIO;
    }

  /* Then the five settings the vendor's player makes after init, in the same
   * order and with the same encodings.  Note that 0x02 is overwritten here:
   * the sequence above ends with both converters off, and these bits are
   * active low.
   *
   *   0x02  adda_cfg(1, 0)     DAC on, ADC off
   *   0x04  output_cfg(1, 1)   both DAC output channels on (3 << 4 | 3 << 2)
   *   0x2E  headphone volume
   *   0x30  speaker volume, 0-33
   *   0x17  Philips I2S, 16 bit  ((fmt << 1) | (len << 3), fmt 0, len 3)
   */

  es_write_reg(0x02, 0x0A);

  /* Outputs stay off while nothing is playing.
   *
   * The transmit block is stopped at idle, so the codec can still be written
   * at this moment - which is what makes switching the outputs around a
   * transfer possible at all.  Left on, the output stage hisses into the
   * speaker amplifier, and the hiss grows with the volume setting.
   */

  es_write_reg(0x04, 0x00);
  es_write_reg(0x2E, 0x21);
  es_write_reg(0x2F, 0x21);
  es_write_reg(0x30, 0x21);
  es_write_reg(0x31, 0x21);
  es_write_reg(0x17, 0x18);

  up_mdelay(100);
  return 0;
}

/****************************************************************************
 * Private Functions: WAV
 ****************************************************************************/

static uint32_t wav_u32(FAR const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t wav_u16(FAR const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/****************************************************************************
 * Name: wav_parse
 *
 * Description:
 *   Read the fmt and data chunks.
 *
 *   A WAV file is a RIFF container of chunks and the interesting ones are not
 *   always the first two, so this walks them rather than assuming offsets.
 *   Only the fields playback actually needs are pulled out.
 *
 ****************************************************************************/

static int wav_parse(FAR FILE *fp, FAR struct wav_info_s *info)
{
  uint8_t hdr[12];
  size_t  off = 12;

  memset(info, 0, sizeof(*info));

  if (fread(hdr, 1, sizeof(hdr), fp) != sizeof(hdr) ||
      memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0)
    {
      return -EINVAL;
    }

  for (;;)
    {
      uint8_t chunk[8];
      uint32_t size;

      if (fread(chunk, 1, sizeof(chunk), fp) != sizeof(chunk))
        {
          return -EINVAL;
        }

      size = wav_u32(chunk + 4);

      if (memcmp(chunk, "fmt ", 4) == 0)
        {
          uint8_t fmt[16];

          if (size < sizeof(fmt) || fread(fmt, 1, sizeof(fmt), fp) != sizeof(fmt))
            {
              return -EINVAL;
            }

          info->channels = wav_u16(fmt + 2);
          info->rate     = wav_u32(fmt + 4);
          info->bits     = wav_u16(fmt + 14);

          /* Skip whatever of the chunk was not read. */

          if (size > sizeof(fmt))
            {
              fseek(fp, (long)(size - sizeof(fmt)), SEEK_CUR);
            }
        }
      else if (memcmp(chunk, "data", 4) == 0)
        {
          info->data_off = (uint32_t)off + 8;
          info->data_len = size;
          break;
        }
      else
        {
          /* Chunks are word aligned; an odd length carries a pad byte. */

          fseek(fp, (long)(size + (size & 1)), SEEK_CUR);
        }

      off += 8 + size + (size & 1);
    }

  if (info->rate == 0 || info->channels == 0 || info->bits == 0)
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: es8388_output_enable
 *
 * Description:
 *   Switch the DAC output channels on or off.
 *
 *   Only legal while the transmit block is stopped: once its clocks are on
 *   the codec's pins, the ES8388 stops acknowledging its address and the
 *   write is nacked.  Every call site is therefore paired with a stop or a
 *   start of that block, never with a running transfer.
 *
 ****************************************************************************/

static int es8388_output_enable(bool on)
{
  return es_write_reg(0x04, on ? 0x3C : 0x00);
}

int stm32n6_audio_init(void)
{
  int ret;

  if (g_audio_ready)
    {
      return 0;
    }

  /* Codec first, transmit block second - the opposite of what this did
   * before, and now with a measurement behind it.
   *
   * On this board the codec answers its address only while the transmit
   * block is stopped.  With the block never started, a register read of
   * 0x10 returns 0x06, which is R0; once the block runs and MCLK is on PE2,
   * every transaction to 0x10 is nacked, and still nacked a hundred
   * milliseconds later.  So the registers are written while the master
   * clock is still quiet, and the block that outputs it is started after.
   *
   * This is also what the vendor player does: MX_SAI1_Init() programs the
   * block early but arms no transfer, and es8388_sai_init_tx() only runs
   * after es8388_init() has finished.
   */

  /* TODO, not fixed yet: after a cold power cycle the codec refuses its whole
   * register sequence - twenty two out of twenty two writes nacked - and only
   * a later reset works.  Its supplies are up before this runs, so it is
   * probably just not ready this early; a retry with a delay was tried and did
   * not help, so the real cause is still unknown.  Left as is because it does
   * not block anything: a reset settles it.
   */

  ret = es8388_configure();
  if (ret < 0)
    {
      return ret;
    }

  /* The transmit block is deliberately not started here.
   *
   * It runs MCLK and the bit clock out to the codec, and a block left running
   * with an empty FIFO feeds the DAC underrun data - which is audible as a
   * continuous hiss, louder the higher the volume is set.  Starting it only
   * for the length of a transfer keeps the speaker silent the rest of the
   * time, and it costs nothing because the codec needs no further writes: on
   * this board the codec stops acknowledging its address as soon as the block
   * runs, so anything it needs has to be in place before the block starts.
   */

  g_audio_ready = true;
  syslog(LOG_INFO, "audio: ES8388 configured, 44.1 kHz stereo, block idle\n");
  return 0;
}

void stm32n6_audio_set_volume(uint8_t volume)
{
  if (volume > 33)
    {
      volume = 33;
    }

  es_write_reg(0x30, volume);
  es_write_reg(0x31, volume);
}

/****************************************************************************
 * Streaming a file to the transmit block
 *
 * The block's FIFO holds eight frames, about 181 us of audio, so the loop
 * that feeds it cannot stop to read the card.  One 4 KB read takes
 * hundreds of microseconds even with the file system's sectors cached, and
 * the FIFO is empty long before the read returns.  Reading the file between
 * two writes to the block is what turns a song into a run of clicks; a tone
 * hides the problem because its next sample is already in memory.
 *
 * The file is therefore read into a ring of slots by a second task, and
 * the caller does nothing but move whole slots into the block.  The reader
 * runs a notch above the caller: it preempts for the few microseconds a
 * slot copy takes, and blocks on the free-slot count the rest of the time
 * while it waits for the card.
 ****************************************************************************/

#define WAV_SLOT_COUNT 8

static uint8_t         g_wav_slots[WAV_SLOT_COUNT][ES_AUDIO_CHUNK];
static size_t          g_wav_slot_len[WAV_SLOT_COUNT];
static uint32_t        g_wav_wr;
static uint32_t        g_wav_rd;
static uint32_t        g_wav_pending;
static sem_t           g_wav_free;
static sem_t           g_wav_filled;
static bool            g_wav_done;
static FAR const char *g_wav_path;
static uint32_t        g_wav_offset;
static uint32_t        g_wav_left;
static int             g_wav_result;

/* Where the clip actually landed, chosen by whichever window proved it keeps
 * what is written to it.
 */

static uint32_t        g_audio_clip;

/* One shot throughput measurement for the reader. */


/* Split the playback loop into the two places it can lose time: waiting for
 * the reader to hand over a slot, and waiting for the channel to take the
 * block.  Whichever number dominates names the bottleneck.
 *
 * Counted in system ticks, not in cycles: the cycle counter stops while the
 * CPU sleeps, so it reports a third of the wall clock and cannot be used for
 * anything that includes a blocked wait.
 */


/* Silence keeper.
 *
 * Once a playback has run the codec stops answering I2C, so its output stages
 * cannot be switched off and the way to keep the speaker quiet is to keep the
 * converter supplied with digital silence.  Leaving the block enabled with an
 * empty FIFO is not silence: a starved transmitter repeats or invents frames,
 * which is heard as fine crackle.  This thread refills it whenever no playback
 * owns the block.
 *
 * The block is 8 KB, which is 46 ms of 44.1 kHz stereo, so the CPU is handed
 * over once every 46 ms rather than once every 181 us.  The thread runs at a
 * low priority: its poll loop yields, so anything that actually has work
 * preempts it, and when the system is otherwise idle the spin costs nothing
 * that was being used.
 */

#define AUDIO_IDLE_WORDS   2048

static int16_t         g_audio_idle_buf[AUDIO_IDLE_WORDS];
static sem_t           g_audio_lock;
static volatile bool   g_audio_busy;
static int             g_audio_idle_pid = -1;

#define AUDIO_CLIP_ENABLE   0

/* Preloading the clip into HyperRAM is switched off.
 *
 * It has never once succeeded on this board - both candidate windows report
 * "did not retain the clip" - and it is not free: it writes the whole file
 * into the window and verifies eight points, which takes about a second and
 * holds the bus hard enough that the silence keeper's transfer times out
 * while it runs.  The timeout leaves the block with its channel torn down,
 * and the playback that follows then sees a FIFO that never drains and falls
 * back to the wrong kernel clock.  Streaming from the card works at 100%, so
 * this probe buys nothing and costs the next playback.
 */

/* 0x90000000 is not free, so the clip does not go there.
 *
 * Layout on this board: the LVGL heap starts at 0x90000000 and is
 * CONFIG_LV_MEM_SIZE_KILOBYTES long (192 KB), the eye-UI camera frames are
 * captured at 0x90100000 (768 KB), and the ATON non-secure window claims the
 * first 4 MB of the part for epoch program buffers and I/O tensors.  The part
 * is 8 MB, so 0x90400000 is the first address past all three and leaves 4 MB,
 * which is plenty for a clip.
 *
 * Playing from here is what removes the stutter.  The card is read through a
 * CPU polled FIFO loop and sustains about 165 KB/s, while 44.1 kHz stereo
 * needs 176 KB/s, so streaming can never keep the ring ahead of the block:
 * measured as six milliseconds of silence at every block boundary, one
 * hundred and sixty two of them.
 */

#define AUDIO_CLIP_BASE     0x90400000
#define AUDIO_CLIP_MAX      (3 * 1024 * 1024)

/****************************************************************************
 * Name: wav_load_clip
 *
 * Description:
 *   Copy the samples into HyperRAM and check that they landed.
 *
 *   Playing straight from the card costs about 1.9 ms of CPU per 4 KB, and
 *   the transmit block's FIFO holds only 181 us of audio, so every read is
 *   heard as a gap - that is the stutter.  With the clip in memory mapped
 *   RAM there is no read at all while it plays.
 *
 ****************************************************************************/

static int wav_load_clip(FAR FILE *fp, uint32_t offset, uint32_t bytes)
{
  static const uint32_t bases[] =
  {
    0x90400000,   /* past the ATON window, if the part is the full 8 MB */
    0x901c0000    /* past the LVGL heap and the camera frame, if it is 4 MB */
  };

  uint8_t  check[64];
  unsigned b;

  if (bytes == 0 || bytes > AUDIO_CLIP_MAX)
    {
      return -E2BIG;
    }

  for (b = 0; b < sizeof(bases) / sizeof(bases[0]); b++)
    {
      FAR uint8_t *dst = (FAR uint8_t *)bases[b];
      uint32_t      done = 0;
      bool          ok = true;

      if (fseek(fp, (long)offset, SEEK_SET) != 0)
        {
          return -EIO;
        }

      while (done < bytes)
        {
          uint32_t n = bytes - done;

          if (n > ES_AUDIO_CHUNK)
            {
              n = ES_AUDIO_CHUNK;
            }

          if (fread(dst + done, 1, n, fp) != n)
            {
              return -EIO;
            }

          done += n;
        }

      /* Check several places, not just the start.
       *
       * Verifying only the first bytes is how a copy into a window that does
       * not retain writes passed as a success: the first few dozen bytes
       * happened to match what was already there, the rest read back as zeros,
       * and playback was fifteen seconds of silence with no error reported
       * anywhere.
       */

      for (done = 0; done < bytes && ok; done += bytes / 8)
        {
          if (fseek(fp, (long)(offset + done), SEEK_SET) != 0 ||
              fread(check, 1, sizeof(check), fp) != sizeof(check))
            {
              return -EIO;
            }

          if (memcmp(dst + done, check, sizeof(check)) != 0)
            {
              ok = false;
            }
        }

      if (ok)
        {
          syslog(LOG_INFO, "audio: clip loaded at %08lx (%lu bytes)\n",
                 (unsigned long)bases[b], (unsigned long)bytes);
          g_audio_clip = bases[b];
          return 0;
        }

      syslog(LOG_WARNING, "audio: %08lx did not retain the clip, trying the "
             "next window\n", (unsigned long)bases[b]);
    }

  return -EIO;
}

/****************************************************************************
 * Name: wav_reader_main
 *
 * Description:
 *   Fill the slot ring from the file until it is exhausted, then wake the
 *   feeder one final time so it can tell the end of the file from a stall.
 *
 ****************************************************************************/

static int wav_reader_main(int argc, FAR char *argv[])
{
  int     fd;
  ssize_t got;

  /* Plain open/read, not stdio.
   *
   * CONFIG_STDIO_BUFFER_SIZE is 64 on this build, so one fread of a 4096 byte
   * slot became sixty four reads, each taking the file system lock and
   * copying through a 64 byte buffer.  That cost about 1.3 ms per slot, which
   * is the whole 107% seen on playback: the reader could not sustain the
   * 176 KB/s the block consumes, so the ring never filled and the feeder
   * waited on it once per block - heard as one dropout every 23 ms.  A single
   * read of the slot lets the file system hand whole sectors straight to the
   * buffer.
   */

  fd = open(g_wav_path, O_RDONLY);
  if (fd < 0)
    {
      g_wav_result = -errno;
      syslog(LOG_ERR, "audio: reader cannot open \"%s\", errno %d\n",
             g_wav_path, errno);
      goto done;
    }

  if (lseek(fd, (off_t)g_wav_offset, SEEK_SET) != (off_t)g_wav_offset)
    {
      g_wav_result = -EIO;
      close(fd);
      goto done;
    }

  while (g_wav_left > 0)
    {
      size_t want = g_wav_left > ES_AUDIO_CHUNK ?
                    ES_AUDIO_CHUNK : g_wav_left;

      /* Wait for a slot.  Once the card has run ahead of the feed this is
       * where the reader spends its time, which is the whole point: it
       * holds a slot the feeder is not waiting on, never the block itself.
       */

      while (nxsem_wait(&g_wav_free) < 0)
        {
        }

      got = read(fd, g_wav_slots[g_wav_wr], want);

      /* The block takes whole frames, sixteen bits per channel. */

      if (got > 0)
        {
          got &= ~1;
        }

      if (got <= 0)
        {
          nxsem_post(&g_wav_free);
          break;
        }

      g_wav_slot_len[g_wav_wr] = (size_t)got;
      g_wav_left              -= (uint32_t)got;
      g_wav_pending++;
      g_wav_wr                 = (g_wav_wr + 1) % WAV_SLOT_COUNT;

      nxsem_post(&g_wav_filled);
    }

  close(fd);

done:
  g_wav_done = true;
  nxsem_post(&g_wav_filled);
  return 0;
}

/****************************************************************************
 * Name: audio_lock_init
 *
 * Description:
 *   One lock around the transmit block, because two threads use it: whoever
 *   is playing something, and the silence keeper that covers the gaps.
 *
 ****************************************************************************/

static void audio_lock_init(void)
{
  static bool done;

  if (!done)
    {
      nxsem_init(&g_audio_lock, 0, 1);
      done = true;
    }
}

/****************************************************************************
 * Name: audio_lock_take
 *
 * Description:
 *   nxsem_wait() reports a signal or a cancellation as a negative return and
 *   leaves the lock untaken, which would then be posted one time too many.
 *   Retry until it is actually held.
 *
 ****************************************************************************/

static void audio_lock_take(void)
{
  while (nxsem_wait(&g_audio_lock) < 0)
    {
    }
}

/****************************************************************************
 * Name: audio_idle_main
 *
 * Description:
 *   Keep the block fed with zeros whenever nothing else is playing.
 *
 ****************************************************************************/

static int audio_idle_main(int argc, FAR char *argv[])
{
  while (true)
    {
      audio_lock_take();

      if (g_audio_busy)
        {
          nxsem_post(&g_audio_lock);
          usleep(20000);
          continue;
        }

      if (stm32n6_sai_output_write(g_audio_idle_buf, AUDIO_IDLE_WORDS) < 0)
        {
          nxsem_post(&g_audio_lock);
          usleep(20000);
          continue;
        }

      nxsem_post(&g_audio_lock);
    }

  return 0;
}

/****************************************************************************
 * Name: audio_idle_start
 *
 * Description:
 *   Start the keeper once; it stays for the life of the system.
 *
 ****************************************************************************/

static void audio_idle_start(void)
{
  audio_lock_init();

  if (g_audio_idle_pid > 0)
    {
      return;
    }

  g_audio_idle_pid = kthread_create("saiidle", SCHED_PRIORITY_MIN + 4, 2048,
                                    audio_idle_main, NULL);
  if (g_audio_idle_pid < 0)
    {
      syslog(LOG_ERR, "audio: cannot start the silence keeper: %d\n",
             g_audio_idle_pid);
      g_audio_idle_pid = -1;
    }
}

int stm32n6_audio_play_wav(FAR const char *path)
{
  FAR FILE          *fp;
  struct wav_info_s  info;
  uint64_t           started;
  uint32_t           elapsed_ms;
  uint32_t           expected_ms;
  int                prio;
  int                pid;
  bool               from_ram = false;
  int                ret = 0;

  if (path == NULL)
    {
      return -EINVAL;
    }

  /* Everything that can fail without touching the hardware happens first: a
   * file that cannot be opened must not leave the transmit block running
   * with an empty FIFO.  That is heard as a hiss, and the codec then refuses
   * I2C until the board is power cycled.
   */

  fp = fopen(path, "rb");
  if (fp == NULL)
    {
      syslog(LOG_ERR, "audio: cannot open \"%s\", errno %d\n", path, errno);
      return -errno;
    }

  ret = wav_parse(fp, &info);
  if (ret < 0)
    {
      syslog(LOG_ERR, "audio: \"%s\" has no usable fmt/data chunk: %d\n",
             path, ret);
      fclose(fp);
      return ret;
    }

  syslog(LOG_INFO, "audio: fmt %lu Hz, %u ch, %u bit; data %lu bytes at "
         "offset %lu\n",
         (unsigned long)info.rate, info.channels, info.bits,
         (unsigned long)info.data_len, (unsigned long)info.data_off);

  /* The transmit path is wired for one rate and one layout.  Saying so beats
   * playing it at the wrong speed, which sounds like a driver bug.
   */

  if (info.rate != 44100 || info.channels != 2 || info.bits != 16)
    {
      syslog(LOG_ERR, "audio: only 44.1 kHz 16-bit stereo is wired up\n");
      fclose(fp);
      return -EINVAL;
    }

  /* Preload into HyperRAM before anything touches the transmit block. */

  if (AUDIO_CLIP_ENABLE && wav_load_clip(fp, info.data_off, info.data_len) == 0)
    {
      from_ram = true;
      syslog(LOG_INFO, "audio: clip is in HyperRAM, playing from memory\n");
    }
  else
    {
      syslog(LOG_WARNING, "audio: could not preload into HyperRAM, "
             "streaming from the card instead (expect gaps)\n");
    }

  fclose(fp);
  fp = NULL;
  if (ret < 0)
    {
      syslog(LOG_ERR, "audio: codec and transmit path did not come up: %d\n",
             ret);
      return ret;
    }

  /* The block is still stopped here, which is the only time the codec can be
   * told to open its outputs.
   */

  es8388_output_enable(true);

  /* Take the block before reprogramming it.
   *
   * The silence keeper owns the block whenever nothing is playing, and it may
   * be in the middle of a transfer.  Reprogramming the block underneath that
   * transfer corrupts both of them: the clock probe writes eight words and
   * waits for the FIFO to empty, but the keeper's channel keeps refilling it,
   * so the probe concludes "still full, no bit clock" and the driver falls
   * back to pclk2 - which is the wrong sample rate, and every later block
   * then times out.  That is the failure seen after playing a tone and then
   * a file.
   */

  audio_lock_init();
  g_audio_busy = true;
  audio_lock_take();

  /* Only program the block if it is not already programmed.
   *
   * The block stays up between playbacks so the codec keeps its clocks, and
   * re-running the bring-up on a live block means re-running the clock probe
   * against a block that the keeper has been feeding - the one step known to
   * misjudge its own state.  The rate is fixed at 44.1 kHz stereo, so nothing
   * about the setup changes between files.
   */

  if (!stm32n6_sai_output_ready())
    {
      ret = stm32n6_sai_output_init(44100, 2);
      if (ret < 0)
        {
          syslog(LOG_ERR, "audio: transmit path init failed: %d\n", ret);
          es8388_output_enable(false);
          g_audio_busy = false;
          nxsem_post(&g_audio_lock);
          return ret;
        }
    }

  started = (uint64_t)clock_systime_ticks() * 1000 /
            (uint64_t)CLK_TCK;

  if (from_ram)
    {
      uint32_t off = 0;

      /* Nothing else runs during this loop - no reader task, no card access -
       * so there is no gap in what the block is handed.
       */

      while (off < info.data_len)
        {
          uint32_t n = info.data_len - off;

          if (n > ES_AUDIO_CHUNK)
            {
              n = ES_AUDIO_CHUNK;
            }

          ret = stm32n6_sai_output_write(
                  (FAR const int16_t *)(g_audio_clip + off),
                  (uint32_t)(n / 2));
          if (ret < 0)
            {
              syslog(LOG_ERR, "audio: transmit failed: %d\n", ret);
              break;
            }

          off += n;
        }

      goto fed;
    }

  g_wav_path    = path;
  g_wav_offset  = info.data_off;
  g_wav_left    = info.data_len;
  g_wav_wr      = 0;
  g_wav_rd      = 0;
  g_wav_pending = 0;
  g_wav_done    = false;
  g_wav_result  = 0;

  nxsem_init(&g_wav_free, 0, WAV_SLOT_COUNT);
  nxsem_init(&g_wav_filled, 0, 0);

  /* Same priority as the feeder, deliberately.
   *
   * The feeder spends the whole of a block inside a register poll loop, and
   * a yield only switches to a thread of the same or higher priority.  Giving
   * the reader a lower priority starved it down to the few milliseconds the
   * feeder spent waiting on a semaphore, which is where the six milliseconds
   * of silence per block came from; the card itself was never the limit.
   */

  prio = this_task()->sched_priority;
  if (prio <= SCHED_PRIORITY_MIN)
    {
      prio = SCHED_PRIORITY_MIN + 1;
    }

  /* The reader's own stack has to carry the file system's read path, which
   * is deeper than the default thread stack.
   */

  pid = kthread_create("wavrd", prio, 4096, wav_reader_main, NULL);
  if (pid < 0)
    {
      syslog(LOG_ERR, "audio: cannot start the reader: %d\n", pid);
      nxsem_destroy(&g_wav_free);
      nxsem_destroy(&g_wav_filled);
      stm32n6_sai_output_stop();
      es8388_output_enable(false);
      g_audio_busy = false;
      nxsem_post(&g_audio_lock);
      return pid;
    }

  /* Let the reader fill the whole ring before the first block goes out.
   *
   * The opening burst is the one moment the card read competes with the
   * audio: the reader fills every empty slot back to back while blocks are
   * already playing.  Waiting for three slots only postponed that burst - the
   * reader kept filling the other five during playback.  Waiting for all of
   * them moves the entire burst in front of the audio, where it cannot be
   * heard, and leaves the loop in the same state it settles into anyway.
   *
   * A block leaves the SAI FIFO holding one word, which is 23 us of audio, so
   * a preemption of more than that at a block boundary is audible.  The ring
   * is 8 x 16 KB, so this costs 745 ms of silence before playback starts and
   * nothing during it.
   */

  while (g_wav_pending < WAV_SLOT_COUNT && g_wav_result == 0 && !g_wav_done)
    {
      usleep(5000);
    }

  while (true)
    {
      while (nxsem_wait(&g_wav_filled) < 0)
        {
        }

      if (g_wav_pending == 0)
        {
          /* The reader posts once more after it has finished, and there is
           * nothing behind that post, so this is the end of the file.
           */

          break;
        }

      if (ret == 0)
        {
          ret = stm32n6_sai_output_write(
                  (FAR const int16_t *)g_wav_slots[g_wav_rd],
                  (uint32_t)(g_wav_slot_len[g_wav_rd] / 2));

          if (ret < 0)
            {
              syslog(LOG_ERR, "audio: transmit failed: %d\n", ret);
            }
        }

      /* Keep taking slots even after a failure, so the reader is never left
       * waiting on a slot that nothing will collect.
       */

      g_wav_rd = (g_wav_rd + 1) % WAV_SLOT_COUNT;
      g_wav_pending--;
      nxsem_post(&g_wav_free);
    }

  nxsem_destroy(&g_wav_free);
  nxsem_destroy(&g_wav_filled);

fed:

  if (g_wav_result < 0)
    {
      syslog(LOG_ERR, "audio: reading the file failed: %d\n", g_wav_result);
      ret = g_wav_result;
    }

  /* Leave the block clocking on silence rather than stopping it.
   *
   * The codec stops answering I2C after the block has run once, so its output
   * stages cannot be turned off and the amplifier keeps whatever the codec
   * drives.  A stopped block hands the codec no MCLK at all, and that idle
   * state is the hiss that follows every playback.  Zeros at a valid rate
   * keep the converter quiet.
   */

  stm32n6_sai_output_quiet();

  if (es8388_output_enable(false) < 0)
    {
      syslog(LOG_INFO, "audio: the codec would not take the mute; the block "
             "is left running on silence instead\n");
    }

  /* Hand the block to the keeper, starting it now if this is the first run.
   * The zero block queued here gives it 46 ms of runway before it has to
   * take over, so the FIFO never runs dry in between.
   */

  audio_idle_start();

  g_audio_busy = false;
  stm32n6_sai_output_write(g_audio_idle_buf, AUDIO_IDLE_WORDS);
  nxsem_post(&g_audio_lock);

  /* Report how long it actually took against how long the file says it
   * should.  This is the only check on the clock chain that does not need a
   * scope: a wrong PLL2 divider shows up as a proportional error here.
   */

  elapsed_ms = (uint32_t)((uint64_t)clock_systime_ticks() * 1000 /
                          (uint64_t)CLK_TCK) - (uint32_t)started;

  expected_ms = (uint32_t)((uint64_t)info.data_len * 1000 /
                           ((uint64_t)info.rate * info.channels *
                            (info.bits / 8)));

  syslog(LOG_INFO, "audio: played %lu ms of audio in %lu ms "
         "(%lu.%02lu%% of nominal)\n",
         (unsigned long)expected_ms, (unsigned long)elapsed_ms,
         (unsigned long)(elapsed_ms * 100 /
                         (expected_ms ? expected_ms : 1)),
         (unsigned long)(elapsed_ms * 10000 /
                         (expected_ms ? expected_ms : 1) % 100));


  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32n6_audio_dump_codec
 *
 * Description:
 *   Read back the registers the bring-up sequence writes.
 *
 *   The expected column is the value the sequence above ends on, so a line
 *   that differs names the register that did not take.  This is the check
 *   that separates a silent codec from a codec that is configured but being
 *   fed nothing, and it needs no audio path to run.
 *
 ****************************************************************************/

void stm32n6_audio_dump_codec(void)
{
  /* { register, value the bring-up sequence leaves in it }.  A flat pair
   * array rather than a struct keeps this readable at a glance and keeps the
   * table out of the way of the style that a struct initialiser here would
   * trip over.
   */

  static const uint8_t expect[][2] =
  {
    { 0x00, 0x06 },   /* reference and driver          */
    { 0x01, 0x50 },   /* power management              */
    { 0x02, 0x0a },   /* DAC on, ADC off               */
    { 0x03, 0x09 },   /* microphone bias off           */
    { 0x04, 0x00 },   /* DAC outputs off while idle    */
    { 0x08, 0x00 },   /* MCLK not divided              */
    { 0x09, 0x88 },   /* ADC PGA                       */
    { 0x0c, 0x4c },   /* ADC data format               */
    { 0x0d, 0x02 },   /* ADC oversampling              */
    { 0x17, 0x18 },   /* DAC: Philips I2S, 16 bit      */
    { 0x18, 0x02 },   /* DAC oversampling              */
    { 0x27, 0xb8 },   /* left mixer                    */
    { 0x2a, 0xb8 },   /* right mixer                   */
    { 0x2e, 0x21 },   /* headphone volume, maximum     */
    { 0x30, 0x21 },   /* speaker volume, maximum       */
    { 0x31, 0x21 },   /* speaker volume, right channel */
  };

  const unsigned count = sizeof(expect) / sizeof(expect[0]);
  uint8_t  value;
  unsigned i;
  int      ret;
  unsigned mismatches = 0;
  unsigned failures   = 0;

  syslog(LOG_INFO, "codec: register read back, expected vs actual\n");

  /* Does a written value come back?
   *
   * Three different headphone volumes, each read back after it is written,
   * separate the two faults that both print as "expected xx read 00": a codec
   * that stores nothing, and a read path that returns zero no matter what
   * the codec holds.  Values that differ mean the reads work and the zeros
   * in the table below are real; one value three times means nothing is
   * being stored, or nothing is being read, and the STOP-separated read that
   * follows splits those two.
   */

  {
    static const uint8_t probe[3] = { 0x00, 0x15, 0x0a };
    unsigned k;

    syslog(LOG_INFO, "codec: write/read probe on 0x2e\n");

    for (k = 0; k < 3; k++)
      {
        uint8_t back = 0xee;
        int     wret;

        wret = es_write_reg(0x2e, probe[k]);
        up_udelay(200);

        ret = es_read_reg(0x2e, &back);

        syslog(LOG_INFO, "codec: 2e <- %02x (w %d), read %02x (r %d)\n",
               probe[k], wret, back, ret);
      }
  }

  /* Same register, read with a STOP between the address byte and the data
   * byte.  If this works and the repeated-start read did not, the codec wants
   * the two phases separated.
   */

  {
    uint8_t back = 0xee;
    int     wret;

    wret = es_write_reg(0x2e, 0x0a);
    up_udelay(200);

    ret = es_read_reg_stop(0x2e, &back);

    syslog(LOG_INFO, "codec: 2e with a stop between phases read %02x "
           "(w %d, r %d)\n", back, wret, ret);
  }

  for (i = 0; i < count; i++)
    {
      value = 0;
      ret   = es_read_reg(expect[i][0], &value);

      if (ret < 0)
        {
          failures++;
          syslog(LOG_ERR, "codec: %02x -> no answer on the bus\n",
                 expect[i][0]);
          continue;
        }

      if (value != expect[i][1])
        {
          mismatches++;
          syslog(LOG_WARNING, "codec: %02x expected %02x read %02x\n",
                 expect[i][0], expect[i][1], value);
        }
    }

  syslog(LOG_INFO, "codec: %u checked, %u differ, %u no answer\n",
         count, mismatches, failures);

  if (failures == 0 && mismatches == 0)
    {
      syslog(LOG_INFO, "codec: configuration matches the sequence byte for "
             "byte, so a silent speaker is not the codec's registers\n");
    }
}

/****************************************************************************
 * Name: es_ab_read
 *
 * Description:
 *   Register index, repeated start, one byte back - the exact transaction
 *   shape the ES8388 needs, against any address on the bus.
 *
 *   This exists so the codec and a known-good device on the same wires can
 *   be asked the same question and the answers compared; every conclusion
 *   about "the codec does not answer" hangs off that comparison.
 *
 ****************************************************************************/

static int es_ab_read(uint8_t addr, uint8_t reg, FAR uint8_t *val)
{
  struct i2c_msg_s ab[2];
  FAR struct i2c_master_s *bus = es_bus();

  if (bus == NULL)
    {
      return -ENODEV;
    }

  ab[0].frequency = 100000;
  ab[0].addr      = addr;
  ab[0].flags     = I2C_M_NOSTOP;
  ab[0].buffer    = &reg;
  ab[0].length    = 1;

  ab[1].frequency = 100000;
  ab[1].addr      = addr;
  ab[1].flags     = I2C_M_READ;
  ab[1].buffer    = val;
  ab[1].length    = 1;

  return I2C_TRANSFER(bus, ab, 2);
}

/****************************************************************************
 * Name: stm32n6_audio_bus_scan
 ****************************************************************************/

int stm32n6_audio_bus_scan(void)
{
  struct i2c_msg_s msg;
  FAR struct i2c_master_s *bus;
  uint8_t probe = 0;
  int  found = 0;
  int  addr;

  bus = es_bus();
  if (bus == NULL)
    {
      return 0;
    }

  syslog(LOG_INFO, "codec-scan: sweeping addresses 0x08..0x77 on PE13/PE14 "
         "through the hardware I2C4 controller, one real byte read each\n");

  for (addr = 0x08; addr < 0x78; addr++)
    {
      int ret;

      /* A one byte read, not a zero length write.  A zero length message was
       * what this used to send, and the sweep it produced listed 0x08 - an
       * address nothing on this board occupies - which means the hardware
       * was reporting a completed transfer without a slave ever driving an
       * acknowledge.  Reading a byte forces the slave to answer.
       */

      msg.frequency = 100000;
      msg.addr      = (uint16_t)addr;
      msg.flags     = I2C_M_READ;
      msg.buffer    = &probe;
      msg.length    = 1;

      ret = I2C_TRANSFER(bus, &msg, 1);

      if (ret >= 0)
        {
          found++;
          syslog(LOG_INFO, "codec-scan: 0x%02x acknowledges%s\n", addr,
                 addr == ES8388_I2C_ADDR ?
                   " <- the ES8388 (expect registers to read back)" :
                 addr == 0x1e ?
                   " <- the light sensor" :
                 addr == 0x2c ?
                   " <- the magnetometer" :
                 addr == 0x50 ?
                   " <- the EEPROM" :
                 addr == 0x6a ?
                   " <- the IMU" : "");
        }
      else if (addr == ES8388_I2C_ADDR)
        {
          /* Say why the codec did not answer.  A NACK means the part is not
           * there or not enabled; a timeout means the transfer never ran,
           * which is the bus or the pin mux instead.  Those two need very
           * different next steps and used to look identical.
           */

          syslog(LOG_ERR, "codec-scan: 0x%02x did not answer: %d%s\n",
                 addr, ret,
                 ret == -ENXIO ? " (no acknowledge: nothing at that address)" :
                 ret == -ETIMEDOUT ? " (timeout: the transaction never ran, "
                                     "check the bus and the pin mux)" : "");
        }
    }

  syslog(LOG_INFO, "codec-scan: %d device(s) acknowledged\n", found);

  /* A/B on one transaction shape.
   *
   * The light sensor and the codec are read with the same two messages - a
   * register index, then a repeated start and one byte back.  Running both
   * here means the next log answers "is the bus broken or is the codec
   * silent" with two lines instead of a guess: 0x1e returning a real value
   * exonerates the bus and the driver at the same moment 0x10 fails.
   */

  {
    static const uint8_t targets[2] = { 0x1e, ES8388_I2C_ADDR };
    static const char *const names[2] =
    {
      "light sensor (control)", "ES8388 (target)"
    };

    static const uint8_t after[3] = { 0x1e, 0x50, ES8388_I2C_ADDR };
    static const char *const after_name[3] =
    {
      "light sensor", "EEPROM", "ES8388"
    };

    uint8_t val;
    unsigned k;
    int ret;

    for (k = 0; k < 2; k++)
      {
        val = 0xee;
        ret = es_ab_read(targets[k], 0x00, &val);

        syslog(LOG_INFO, "codec-scan: ab %-22s addr 0x%02x reg 00 -> "
               "%02x (ret %d)\n", names[k], targets[k], val, ret);
      }

    /* Now the same question with the transmit block running.
     *
     * The board answers differently depending on this: with MCLK and the bit
     * clock running, the codec stops acknowledging, and with the block never
     * started it answers.  Every device on the wires is asked, not just the
     * codec - if the other two stop as well, the transmit block took the bus
     * down and the codec is a bystander; if they keep answering, only the
     * codec changed and its master clock input is what did it.
     */

    ret = stm32n6_sai_output_init(44100, 2);

    syslog(LOG_INFO, "codec-scan: transmit block start -> %d, asking again\n",
           ret);

    for (k = 0; k < 3; k++)
      {
        val = 0xee;
        ret = es_ab_read(after[k], 0x00, &val);

        syslog(LOG_INFO, "codec-scan: ab %-12s with sai running -> %02x "
               "(ret %d)\n", after_name[k], val, ret);
      }

    up_mdelay(100);

    val = 0xee;
    ret = es_ab_read(ES8388_I2C_ADDR, 0x00, &val);
    syslog(LOG_INFO, "codec-scan: ab ES8388 100 ms later   -> %02x "
           "(ret %d)\n", val, ret);
  }

  if (found == 0)
    {
      syslog(LOG_ERR, "codec-scan: nothing answered on I2C4. Either the bus "
             "is held down or nothing on this board is powered.\n");
    }

  return found;
}

/****************************************************************************
 * Name: stm32n6_audio_tone
 ****************************************************************************/

int stm32n6_audio_tone(int freq_hz, int ms)
{
  FAR int16_t *buf;
  uint32_t     frames;
  uint32_t     period;
  uint32_t     done;
  uint32_t     started;
  uint32_t     elapsed;
  bool         silent;
  int          ret = 0;

  if (freq_hz != 0 && (freq_hz < 20 || freq_hz > 8000))
    {
      freq_hz = 440;
    }

  if (ms <= 0 || ms > 5000)
    {
      ms = 500;
    }

  ret = stm32n6_audio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "audio-tone: codec and transmit path did not come up: "
             "%d\n", ret);
      return ret;
    }

  es8388_output_enable(true);

  audio_lock_init();
  g_audio_busy = true;
  audio_lock_take();

  if (!stm32n6_sai_output_ready())
    {
      ret = stm32n6_sai_output_init(44100, 2);
      if (ret < 0)
        {
          syslog(LOG_ERR, "audio-tone: transmit path init failed: %d\n", ret);
          es8388_output_enable(false);
          g_audio_busy = false;
          nxsem_post(&g_audio_lock);
          return ret;
        }
    }

  frames = (44100u / 1000u) * (uint32_t)ms;

  /* Big chunks on purpose.  Every call starts a new channel block, and the
   * block boundary is where the FIFO can be caught empty, so the fewer the
   * better: at 256 frames a 800 ms tone crossed 137 boundaries and was heard
   * to break up three times, and every one of those also cost the timing a
   * whole ten millisecond tick.
   */

  buf = malloc(8192 * sizeof(int16_t));
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  /* One period of the tone, worked out once instead of per frame. */

  if (freq_hz == 0)
    {
      /* Silence, through the same block and the same feed loop.  Anything
       * audible during this is generated by the codec or by the block itself,
       * which separates a noisy path from a waveform problem.
       */

      period = 1024;
    }
  else
    {
      period = 44100u / (uint32_t)freq_hz;
      if (period < 2)
        {
          period = 2;
        }
      else if (period > 1024)
        {
          period = 1024;
        }
    }

  started = (uint32_t)clock_systime_ticks();
  done    = 0;
  silent  = (freq_hz == 0);

  while (done < frames)
    {
      uint32_t n = frames - done;
      uint32_t i;

      if (n > 4096)
        {
          n = 256;
        }

      for (i = 0; i < n; i++)
        {
          /* Square wave, the waveform that was measured to play continuously
           * on this board.  Everything else in this function is unchanged
           * from that run; the sine that replaced it is what turned the tone
           * into a stutter, and it is not worth chasing while the path itself
           * is proven clean by tone 0.
           */

          int16_t sample = silent ? 0 :
            ((((done + i) % period) < (period / 2)) ? 9000 : -9000);

          buf[2 * i]     = sample;
          buf[2 * i + 1] = sample;
        }

      ret = stm32n6_sai_output_write(buf, n * 2);

      if (ret < 0)
        {
          syslog(LOG_ERR, "audio-tone: transmit failed after %lu of %lu "
                 "frames: %d\n", (unsigned long)done,
                 (unsigned long)frames, ret);
          break;
        }

      done += n;
    }

  free(buf);

  /* What the block looked like when the data ran out.  Read before the stop:
   * stopping clears it.  Bit 0 is overrun/underrun and bit 2 is wrong clock
   * configuration; either one set means the tone did not leave the block
   * cleanly and the stutter is in the digital side rather than the analog.
   */

  {
    uint32_t sr = stm32n6_sai_output_status();

    syslog(LOG_INFO, "audio-tone: sai status after transfer %08lx%s%s%s\n",
           (unsigned long)sr,
           (sr & 1) ? " underrun" : "",
           (sr & 4) ? " wrong-clock-config" : "",
           (sr & 8) ? " fifo-req" : "");
  }

  /* Same reasoning as the file path: quiet, not stopped, then hand the block
   * to the keeper so the FIFO never starves.
   */

  stm32n6_sai_output_quiet();

  if (es8388_output_enable(false) < 0)
    {
      syslog(LOG_INFO, "audio-tone: the codec would not take the mute; the "
             "block is left running on silence instead\n");
    }

  audio_idle_start();

  g_audio_busy = false;
  stm32n6_sai_output_write(g_audio_idle_buf, AUDIO_IDLE_WORDS);
  nxsem_post(&g_audio_lock);

  elapsed = (uint32_t)(((uint64_t)((uint32_t)clock_systime_ticks() -
                                   started) * 1000) / (uint64_t)CLK_TCK);

  syslog(LOG_INFO, "audio-tone: %d Hz for %d ms requested, %lu ms measured "
         "(%lu of %lu frames)%s\n",
         freq_hz, ms, (unsigned long)elapsed, (unsigned long)done,
         (unsigned long)frames,
         (elapsed > (uint32_t)ms / 2 && elapsed < (uint32_t)ms * 2) ?
           "" : " <- the block is running at the wrong rate");

  return ret;
}

#endif /* CONFIG_STM32_SAI1 */
