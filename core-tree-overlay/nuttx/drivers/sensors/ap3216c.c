/****************************************************************************
 * drivers/sensors/ap3216c.c
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

/* Driver for the Lite-On AP3216C ambient light / proximity / IR sensor.
 *
 * The register numbers and the decoding of the six data bytes come from the
 * vendor BSP (SoftwarePackage/Drivers/BSP/AP3216C), which is the only
 * reference for this part that has actually run on this board.
 *
 * Units, stated plainly because the uORB topics have fixed meanings:
 *
 *   light      ALS counts scaled by AP3216C_ALS_LUX_SCALE.  The datasheet
 *              gives a typical figure rather than a transfer function, so
 *              read the result as indicative, not calibrated.
 *   ir         IR counts, unscaled.  The topic field is documented as lux;
 *              this driver puts counts there and says so.
 *   proximity  PS counts, NOT centimetres.  The PS channel has no linear
 *              distance mapping without board-level characterisation, and
 *              this driver does not invent one.
 *
 * Every I2C transaction and every bring-up stage logs a line.  That is
 * deliberate: in this product the sensor exists mainly to prove that
 * I2C4 and its peripherals are alive, so a silent failure would defeat
 * the purpose.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <syslog.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/sensor.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AP3216C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AP3216C_REG_SYSTEM      0x00
#define AP3216C_REG_IR_DATA     0x0a
#define AP3216C_REG_ALS_DATA    0x0c
#define AP3216C_REG_PS_DATA     0x0e

#define AP3216C_SYS_SOFTRESET   0x04
#define AP3216C_SYS_ALS_PS_IR   0x03

#define AP3216C_IR_OVERFLOW     0x80
#define AP3216C_PS_OVERFLOW     0x40

#define AP3216C_MIN_INTERVAL    100000   /* 100 ms */
#define AP3216C_RESET_WAIT_MS   10
#define AP3216C_FIRST_READ_MS   100      /* first conversion after enable */

/* ALS counts to lux.  Datasheet-typical, not measured on this board. */

#define AP3216C_ALS_LUX_SCALE   0.35f

/* Log the first few samples and then one in every N, so a running sensor
 * does not flood the console but a failing one is still visible
 * immediately.
 */

#define AP3216C_LOG_FIRST       3
#define AP3216C_LOG_EVERY       100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ap3216c_dev_s
{
  struct sensor_lowerhalf_s lower_light;   /* /dev/uorb/sensor_lightN */
  struct sensor_lowerhalf_s lower_prox;    /* /dev/uorb/sensor_proxN  */
  struct work_s             work;
  FAR struct i2c_master_s  *i2c;
  uint32_t                  interval;
  uint32_t                  samples;
  uint16_t                  ir;
  uint16_t                  als;
  uint16_t                  ps;
  bool                      light_active;
  bool                      prox_active;
  uint8_t                   addr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* One sensor on this board.  The activate/set_interval callbacks are handed
 * a sensor_lowerhalf_s pointer and have to get back to the device from it;
 * with a single instance a file-scope pointer does that without pointer
 * arithmetic on a struct that has two lower halves.
 */

static FAR struct ap3216c_dev_s *g_ap3216c;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ap3216c_getregs
 *
 * Description:
 *   Read a run of registers.  Logs the transaction unconditionally: a
 *   failed read here is the single most likely reason the sensor looks
 *   dead, and the address and register number are what make it
 *   diagnosable.
 *
 ****************************************************************************/

static int ap3216c_getregs(struct ap3216c_dev_s *priv, uint8_t reg,
                           FAR uint8_t *buf, size_t n, bool quiet)
{
  struct i2c_msg_s msg[2];
  int ret;

  msg[0].frequency = CONFIG_SENSORS_AP3216C_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = I2C_M_NOSTOP;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_SENSORS_AP3216C_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = n;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);

  /* I2C_TRANSFER returns the number of messages moved, not zero, so a bare
   * "-> 2" in the log reads like an error when it is a success.  Print the
   * verdict in words and keep the count as detail.
   */

  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: i2c read  addr=%02x reg=%02x len=%zu "
             "FAILED errno %d\n", priv->addr, reg, n, ret);
      return ret;
    }

  if (!quiet)
    {
      syslog(LOG_INFO, "ap3216c: i2c read  addr=%02x reg=%02x len=%zu ok "
             "(%d msgs)\n", priv->addr, reg, n, ret);
    }

  return OK;
}

/****************************************************************************
 * Name: ap3216c_putreg
 *
 ****************************************************************************/

static int ap3216c_putreg(struct ap3216c_dev_s *priv, uint8_t reg,
                          uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buf[2];
  int ret;

  buf[0] = reg;
  buf[1] = value;

  msg.frequency = CONFIG_SENSORS_AP3216C_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  ret = I2C_TRANSFER(priv->i2c, &msg, 1);

  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: i2c write addr=%02x reg=%02x val=%02x "
             "FAILED errno %d\n", priv->addr, reg, value, ret);
      return ret;
    }

  syslog(LOG_INFO, "ap3216c: i2c write addr=%02x reg=%02x val=%02x ok\n",
         priv->addr, reg, value);

  return OK;
}

/****************************************************************************
 * Name: ap3216c_readdata
 *
 * Description:
 *   Read the six data registers, one register per transfer.
 *
 *   The vendor BSP does exactly this -- a loop of single-byte reads at
 *   0x0A..0x0F -- rather than one six-byte burst.  The burst form was tried
 *   first here and returned nonsense (ir=als=ps=3 on a lit desk), so this
 *   follows the reference that has actually run on the board.
 *
 *   Only failures are logged: at a 100 ms interval a line per byte would be
 *   six lines per sample, and the useful signal is "which register failed".
 *
 ****************************************************************************/

static int ap3216c_readdata(struct ap3216c_dev_s *priv, FAR uint8_t *data)
{
  int i;
  int ret;

  for (i = 0; i < 6; i++)
    {
      ret = ap3216c_getregs(priv, AP3216C_REG_IR_DATA + i, &data[i], 1, true);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ap3216c: data read failed at register %02x: %d\n",
                 AP3216C_REG_IR_DATA + i, ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ap3216c_probe
 *
 * Description:
 *   The vendor's bring-up: soft reset, enable all three channels, read the
 *   mode register back and expect it to hold what was written.
 *
 *   Reading the register back matters.  An I2C acknowledge only proves the
 *   address answered; it does not prove the write landed.  Checking the
 *   value is the difference between "the bus works" and "the sensor is
 *   configured".
 *
 ****************************************************************************/

static int ap3216c_probe(struct ap3216c_dev_s *priv)
{
  uint8_t value = 0;
  int ret;

  syslog(LOG_INFO, "ap3216c: --- bring-up: soft reset ---\n");

  ret = ap3216c_putreg(priv, AP3216C_REG_SYSTEM, AP3216C_SYS_SOFTRESET);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: soft reset write failed: %d\n", ret);
      return ret;
    }

  up_mdelay(AP3216C_RESET_WAIT_MS);

  syslog(LOG_INFO, "ap3216c: --- bring-up: enable ALS+PS+IR (0x%02x) ---\n",
         AP3216C_SYS_ALS_PS_IR);

  ret = ap3216c_putreg(priv, AP3216C_REG_SYSTEM, AP3216C_SYS_ALS_PS_IR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: mode write failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "ap3216c: --- bring-up: read mode register back ---\n");

  ret = ap3216c_getregs(priv, AP3216C_REG_SYSTEM, &value, 1, false);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: mode readback failed: %d\n", ret);
      return ret;
    }

  if (value != AP3216C_SYS_ALS_PS_IR)
    {
      syslog(LOG_ERR, "ap3216c: mode readback %02x, expected %02x\n",
             value, AP3216C_SYS_ALS_PS_IR);
      return -ENODEV;
    }

  syslog(LOG_INFO, "ap3216c: sensor answered correctly at address %02x\n",
         priv->addr);
  return OK;
}

/****************************************************************************
 * Name: ap3216c_decode
 *
 * Description:
 *   Turn the six data bytes into IR, ALS and PS counts.  A channel that
 *   reports overflow reads as zero, which is what the vendor driver does and
 *   what the datasheet asks for.
 *
 ****************************************************************************/

static void ap3216c_decode(FAR const uint8_t *data, FAR uint16_t *ir,
                           FAR uint16_t *als, FAR uint16_t *ps)
{
  *ir  = (data[0] & AP3216C_IR_OVERFLOW) ? 0 :
         (uint16_t)(((uint16_t)data[1] << 2) | (data[0] & 0x03));

  *als = (uint16_t)(((uint16_t)data[3] << 8) | data[2]);

  *ps  = (data[4] & AP3216C_PS_OVERFLOW) ? 0 :
         (uint16_t)(((uint16_t)(data[5] & 0x3f) << 4) | (data[4] & 0x0f));
}

/****************************************************************************
 * Name: ap3216c_worker
 *
 * Description:
 *   Read the sensor, publish what is enabled, and put itself back on the
 *   queue.  Publishing happens here rather than in an interrupt because an
 *   I2C transfer cannot be made from one.
 *
 ****************************************************************************/

static void ap3216c_worker(FAR void *arg)
{
  FAR struct ap3216c_dev_s *priv = arg;
  struct sensor_light light;
  struct sensor_prox  prox;
  uint8_t data[6];
  uint64_t stamp;
  int ret;

  DEBUGASSERT(priv != NULL);

  work_queue(HPWORK, &priv->work, ap3216c_worker, priv,
             priv->interval / USEC_PER_TICK);

  memset(data, 0, sizeof(data));

  ret = ap3216c_readdata(priv, data);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: sample read failed: %d\n", ret);
      return;
    }

  ap3216c_decode(data, &priv->ir, &priv->als, &priv->ps);

  priv->samples++;
  stamp = sensor_get_timestamp();

  if (priv->samples <= AP3216C_LOG_FIRST ||
      (priv->samples % AP3216C_LOG_EVERY) == 0)
    {
      syslog(LOG_INFO, "ap3216c: sample %lu: raw %02x %02x %02x %02x %02x "
             "%02x -> ir %u als %u ps %u\n",
             (unsigned long)priv->samples,
             data[0], data[1], data[2], data[3], data[4], data[5],
             priv->ir, priv->als, priv->ps);
    }

  if (priv->light_active)
    {
      light.timestamp = stamp;
      light.light     = (float)priv->als * AP3216C_ALS_LUX_SCALE;
      light.ir        = (float)priv->ir;

      priv->lower_light.push_event(priv->lower_light.priv, &light,
                                   sizeof(light));
    }

  if (priv->prox_active)
    {
      prox.timestamp = stamp;
      prox.proximity = (float)priv->ps;

      priv->lower_prox.push_event(priv->lower_prox.priv, &prox,
                                  sizeof(prox));
    }
}

/****************************************************************************
 * Name: ap3216c_set_interval
 *
 ****************************************************************************/

static int ap3216c_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct ap3216c_dev_s *priv = g_ap3216c;

  if (priv == NULL)
    {
      return -ENODEV;
    }

  if (*period_us < AP3216C_MIN_INTERVAL)
    {
      priv->interval = AP3216C_MIN_INTERVAL;
      *period_us     = priv->interval;
    }
  else
    {
      priv->interval = *period_us;
    }

  syslog(LOG_INFO, "ap3216c: interval set to %lu us\n",
         (unsigned long)priv->interval);

  return OK;
}

/****************************************************************************
 * Name: ap3216c_activate
 *
 * Description:
 *   Start or stop sampling.  The sensor keeps converting while idle; only
 *   the publishing and the work queue are gated, which keeps the first
 *   sample after an enable from being stale.
 *
 ****************************************************************************/

static int ap3216c_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct ap3216c_dev_s *priv = g_ap3216c;
  bool wake;

  if (priv == NULL)
    {
      return -ENODEV;
    }

  if (lower == &priv->lower_light)
    {
      priv->light_active = enable;
      syslog(LOG_INFO, "ap3216c: light channel %s\n",
             enable ? "enabled" : "disabled");
    }
  else if (lower == &priv->lower_prox)
    {
      priv->prox_active = enable;
      syslog(LOG_INFO, "ap3216c: proximity channel %s\n",
             enable ? "enabled" : "disabled");
    }
  else
    {
      syslog(LOG_ERR, "ap3216c: activate called for an unknown channel\n");
      return -EINVAL;
    }

  wake = priv->light_active || priv->prox_active;

  if (wake)
    {
      if (work_available(&priv->work))
        {
          syslog(LOG_INFO, "ap3216c: sampling starts, first read in %d ms\n",
                 AP3216C_FIRST_READ_MS);

          work_queue(HPWORK, &priv->work, ap3216c_worker, priv,
                     MSEC2TICK(AP3216C_FIRST_READ_MS));
        }
    }
  else
    {
      syslog(LOG_INFO, "ap3216c: both channels off, sampling stops\n");
      work_cancel(HPWORK, &priv->work);
    }

  return OK;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_ap3216c_ops =
{
  .activate     = ap3216c_activate,
  .set_interval = ap3216c_set_interval,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ap3216c_register
 *
 * Description:
 *   Bind the sensor on the given bus to two uORB topics: light and
 *   proximity.
 *
 * Input Parameters:
 *   devno - Device number, used to name /dev/uorb/sensor_lightN and _proxN
 *   i2c   - An instance of the I2C master to use
 *   addr  - 7-bit I2C address (0x1e on this board)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ap3216c_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct ap3216c_dev_s *priv;
  int ret;

  syslog(LOG_INFO, "ap3216c: register start, devno %d addr %02x\n",
         devno, addr);

  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ap3216c: no I2C bus supplied\n");
      return -EINVAL;
    }

  priv = kmm_zalloc(sizeof(struct ap3216c_dev_s));
  if (priv == NULL)
    {
      syslog(LOG_ERR, "ap3216c: out of memory\n");
      return -ENOMEM;
    }

  priv->i2c      = i2c;
  priv->addr     = addr;
  priv->interval = AP3216C_MIN_INTERVAL;

  priv->lower_light.ops     = &g_ap3216c_ops;
  priv->lower_light.type    = SENSOR_TYPE_LIGHT;
  priv->lower_light.nbuffer = 1;

  priv->lower_prox.ops      = &g_ap3216c_ops;
  priv->lower_prox.type     = SENSOR_TYPE_PROXIMITY;
  priv->lower_prox.nbuffer  = 1;

  ret = ap3216c_probe(priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: probe failed, not registering: %d\n", ret);
      kmm_free(priv);
      return ret;
    }

  g_ap3216c = priv;

  syslog(LOG_INFO, "ap3216c: registering uORB light topic\n");

  ret = sensor_register(&priv->lower_light, devno);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: light registration failed: %d\n", ret);
      g_ap3216c = NULL;
      kmm_free(priv);
      return ret;
    }

  syslog(LOG_INFO, "ap3216c: registering uORB proximity topic\n");

  ret = sensor_register(&priv->lower_prox, devno);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ap3216c: proximity registration failed: %d\n", ret);
      g_ap3216c = NULL;
      kmm_free(priv);
      return ret;
    }

  syslog(LOG_INFO, "ap3216c: ready, light and proximity topics up\n");
  return OK;
}

/****************************************************************************
 * Name: ap3216c_read_now
 *
 * Description:
 *   Read the sensor once and return the raw counts.  Used by the board
 *   self-test, which must work whether or not anything has subscribed to
 *   the uORB topics.
 *
 ****************************************************************************/

int ap3216c_read_now(FAR uint16_t *ir, FAR uint16_t *als, FAR uint16_t *ps)
{
  FAR struct ap3216c_dev_s *priv = g_ap3216c;
  uint8_t data[6];
  int ret;

  if (priv == NULL)
    {
      syslog(LOG_ERR, "ap3216c: read_now called before register\n");
      return -ENODEV;
    }

  memset(data, 0, sizeof(data));

  ret = ap3216c_readdata(priv, data);
  if (ret < 0)
    {
      return ret;
    }

  ap3216c_decode(data, ir, als, ps);

  priv->ir  = *ir;
  priv->als = *als;
  priv->ps  = *ps;

  syslog(LOG_INFO, "ap3216c: read_now raw %02x %02x %02x %02x %02x %02x "
         "-> ir %u als %u ps %u\n",
         data[0], data[1], data[2], data[3], data[4], data[5],
         *ir, *als, *ps);

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_AP3216C */
