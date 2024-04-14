/*
 * Copyright (c) 2016 Brian Schwind <https://github.com/bschwind>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2023 Michael D. Naish <mnaish@uwo.ca>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file tsl2561.h
 * @defgroup tsl2561 tsl2561
 * @{
 *
 * ESP-IDF driver for TSL2561 light-to-digital converter
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2016 Brian Schwind <https://github.com/bschwind>\n
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 * Copyright (c) 2023 Michael D. Naish <mnaish@uwo.ca>
 *
 * BSD Licensed as described in the file LICENSE
 */

#ifndef __TSL2561_H__
#define __TSL2561_H__

#include <i2cdev.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSL2561_I2C_ADDR_GND   0x29
#define TSL2561_I2C_ADDR_FLOAT 0x39 //!< Default I2C address
#define TSL2561_I2C_ADDR_VCC   0x49


/**
 * Approximate ratios for CH1 to CH0 under different lighting conditions
 */
#define TSL2561_APPROX_CH_RATIO_SUN 0.325
#define TSL2561_APPROX_CH_RATIO_LED 0.110

/**
 * Device package
 */
typedef enum
{
    TSL2561_PACKAGE_CS = 0,
    TSL2561_PACKAGE_T_FN_CL
} tsl2561_package_t;

/**
 * Integration time
 */
typedef enum
{
    TSL2561_INTEGRATION_13MS = 0, //!< 13ms
    TSL2561_INTEGRATION_101MS,    //!< 101ms
    TSL2561_INTEGRATION_402MS     //!< 402ms, default
} tsl2561_integration_time_t;

/**
 * Gain
 */
typedef enum
{
    TSL2561_GAIN_1X = 0x00, //!< Default
    TSL2561_GAIN_16X = 0x10
} tsl2561_gain_t;

/**
 * Interrupt control
 */
typedef enum
{
    TSL2561_INTR_DISABLED = 0x00,
    TSL2561_INTR_LEVEL    = 0x10,
    TSL2561_INTR_SMBALERT = 0x20,
    TSL2561_INTR_TEST     = 0x30
} tsl2561_interrupt_control_t;

/**
 * Interrupt persistence
 */
typedef enum
{
    TSL2561_PERSIST_EVERY      = 0x0,
    TSL2561_PERSIST_ANY        = 0x1,
    TSL2561_PERSIST_2_PERIODS  = 0x2,
    TSL2561_PERSIST_3_PERIODS  = 0x3,
    TSL2561_PERSIST_4_PERIODS  = 0x4,
    TSL2561_PERSIST_5_PERIODS  = 0x5,
    TSL2561_PERSIST_6_PERIODS  = 0x6,
    TSL2561_PERSIST_7_PERIODS  = 0x7,
    TSL2561_PERSIST_8_PERIODS  = 0x8,
    TSL2561_PERSIST_9_PERIODS  = 0x9,
    TSL2561_PERSIST_10_PERIODS = 0xA,
    TSL2561_PERSIST_11_PERIODS = 0xB,
    TSL2561_PERSIST_12_PERIODS = 0xC,
    TSL2561_PERSIST_13_PERIODS = 0xD,
    TSL2561_PERSIST_14_PERIODS = 0xE,
    TSL2561_PERSIST_15_PERIODS = 0xF
} tsl2561_interrupt_persistence_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    tsl2561_integration_time_t integration_time;
    tsl2561_gain_t gain;
    tsl2561_interrupt_control_t interrupt_control;
    tsl2561_interrupt_persistence_t interrupt_persistence;
    uint16_t low_threshold;
    uint16_t high_threshold;
    tsl2561_package_t package_type;
} tsl2561_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr I2C device address, `TSL2561_I2C_ADDR_...` const
 * @param port I2C port
 * @param sda_gpio SDA GPIO pin
 * @param scl_gpio SCL GPIO pin
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_init_desc(tsl2561_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_free_desc(tsl2561_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_init(tsl2561_t *dev);

/**
 * @brief Set device integration time
 *
 * @param dev Device descriptor
 * @param integration_time Integration time
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_integration_time(tsl2561_t *dev, tsl2561_integration_time_t integration_time);

/**
 * @brief Set device gain
 *
 * @param dev Device descriptor
 * @param gain Gain
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_gain(tsl2561_t *dev, tsl2561_gain_t gain);

/**
 * @brief Set device interrupt control
 *
 * @param dev Device descriptor
 * @param interrupt_control Interrupt control
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_interrupt_control(tsl2561_t *dev, tsl2561_interrupt_control_t interrupt_control);

/**
 * @brief Set device interrupt persistence
 *
 * @param dev Device descriptor
 * @param interrupt_persistence Interrupt persistence
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_interrupt_persistence(tsl2561_t *dev, tsl2561_interrupt_persistence_t interrupt_persistence);

/**
 * @brief Set device low interrupt threshold
 *
 * @param dev Device descriptor
 * @param low_threshold Low interrupt threshold value
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_interrupt_low_threshold(tsl2561_t *dev, uint16_t low_threshold);

/**
 * @brief Set device high interrupt threshold
 *
 * @param dev Device descriptor
 * @param high_threshold High interrupt threshold value
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_set_interrupt_high_threshold(tsl2561_t *dev, uint16_t high_threshold);

/**
 * @brief Clear interrupt
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_clear_interrupt(tsl2561_t *dev);

/**
 * @brief Read light intensity from device
 *
 * @param dev Device descriptor
 * @param[out] lux Light intensity, lux
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_read_lux(tsl2561_t *dev, uint32_t *lux);

/**
 * @brief Calculate raw 16-bit value for channel 0
 *
 * Utility function to approximate raw channel 0 value for setting interrupt thresholds. Threshold
 * detection is based on the broad spectrum (channel 0) sensor only. The calculation of lux involves
 * the ratio of channel 0 and channel 1 (IR spectrum). This function will generate an estimate on the 
 * channel 0 value for a specified light level in lux, given an estimate of the channel 1/channel 0
 * ratio. In sunlight, the ratio is about 0.325; under LED light, the ratio is about 0.110.
 * 
 * @param dev Device descriptor
 * @param lux Light intensity, lux
 * @param approx_channel_ratio Ratio of CH0 and CH1 for environment
 * @param[out] raw Estimated 16-bit threshold value
 * @return `ESP_OK` on success
 */
esp_err_t tsl2561_calc_raw_ch0(tsl2561_t *dev, uint32_t lux, float approx_channel_ratio, uint16_t *raw);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TSL2561_H__
