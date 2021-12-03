/**
 * @file magAK8963.h
 * @author Hans Burmester (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "mpu9250.h"

#ifndef __MAGAK8963_H
#define __MAGAK8963_H

#define AK8963_ADDRESS (0x0c)
#define AK8963_WHO_AM_I (0x00)
#define AK8963_WHO_AM_I_RESPONSE (0x48)

#define AK8963_XOUT_L (0x03) // data
#define AK8963_XOUT_H (0x04)
#define AK8963_YOUT_L (0x05)
#define AK8963_YOUT_H (0x06)
#define AK8963_ZOUT_L (0x07)
#define AK8963_ZOUT_H (0x08)
#define AK8963_ST2 (0x09)    // Data overflow bit 3 and data read error status bit 2
#define AK8963_ASAX (0x10)   // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY (0x11)   // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ (0x12)   // Fuse ROM z-axis sensitivity adjustment value

#define AK8963_CNTL (0x0a)   // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_CNTL_MODE_FUSE_ROM_ACCESS (0x0f)    // Fuse ROM access mode

#define AK8963_CNTL_MODE_CONTINUE_MEASURE_1 (0x02) // Continuous measurement mode 1 - Sensor is measured periodically at 8Hz
#define AK8963_CNTL_MODE_CONTINUE_MEASURE_2 (0x06) // Continuous measurement mode 2 - Sensor is measured periodically at 100Hz

esp_err_t AK8963_init(i2c_port_t i2c_number, calibration_t *c);
esp_err_t ak8963_get_device_id(uint8_t *val);
esp_err_t ak8963_get_cntl(uint8_t *mode);
esp_err_t ak8963_set_cntl(uint8_t mode);

esp_err_t ak8963_get_sensitivity_adjustment_values();

esp_err_t ak8963_get_mag_raw(uint8_t bytes[6]);
esp_err_t ak8963_get_mag(vector_t *v);

void ak8963_print_settings(void);

#endif