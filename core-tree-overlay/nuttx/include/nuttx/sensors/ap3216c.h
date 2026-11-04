/****************************************************************************
 * include/nuttx/sensors/ap3216c.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_AP3216C_H
#define __INCLUDE_NUTTX_SENSORS_AP3216C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_AP3216C)

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ap3216c_register
 *
 * Description:
 *   Register the AP3216C ambient light / proximity / IR sensor on the given
 *   I2C bus.  Publishes two uORB topics: sensor_lightN (ALS scaled to
 *   indicative lux, IR counts) and sensor_proxN (PS counts).
 *
 *   The bring-up sequence reads the mode register back and refuses to
 *   register if the value did not land, so a device that acknowledges its
 *   address but ignores writes cannot masquerade as a working sensor.
 *
 * Input Parameters:
 *   devno - Device number, used to name /dev/uorb/sensor_lightN and _proxN
 *   i2c   - An instance of the I2C master to use
 *   addr  - 7-bit I2C address (0x1e on the ATK-DNN647)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ap3216c_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr);

/****************************************************************************
 * Name: ap3216c_read_now
 *
 * Description:
 *   Read the sensor once and return the raw counts.
 *
 *   The uORB topics only publish while something is subscribed, so a first
 *   bring-up check that relied on them would show nothing at all until an
 *   application opened them.  This reads the six data registers directly,
 *   which makes it usable as a self-test straight after boot.
 *
 * Input Parameters:
 *   ir, als, ps - Receivers for the three raw channels
 *
 * Returned Value:
 *   Zero (OK) on success; -ENODEV if the driver is not registered yet.
 *
 ****************************************************************************/

int ap3216c_read_now(FAR uint16_t *ir, FAR uint16_t *als, FAR uint16_t *ps);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_AP3216C */
#endif /* __INCLUDE_NUTTX_SENSORS_AP3216C_H */
