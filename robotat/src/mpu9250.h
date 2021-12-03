/**
 * @file mpu9250.h
 * @author Hans Burmester
 * @brief 
 * @version 0.1
 * @date 2021-08-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __MPU9250_H
#define __MPU9250_H

#define I2C_MASTER_SCL_IO 22                                /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21                                /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM  I2C_NUM_0                            /*!< I2C port number for master dev */
#define MPU9250_I2C_ADDRESS (0x68)                          // MPU9250 I2C address
#define MPU9250_WHO_AM_I (0x75)

#define MPU9250_RA_PWR_MGMT_1 (0x6B)
#define MPU9250_RA_PWR_MGMT_2 (0x6C)
#define MPU9250_PWR1_DEVICE_RESET_BIT (7)
#define MPU9250_PWR1_CLKSEL_BIT (0)
#define MPU9250_PWR1_CLKSEL_LENGTH (3)
#define MPU9250_PWR1_SLEEP_BIT (6)

//Acelerómetro
#define MPU9250_ACONFIG_FS_SEL_BIT (3)
#define MPU9250_ACONFIG_FS_SEL_LENGTH (2)
#define MPU9250_RA_ACCEL_CONFIG_1 (0x1C)
#define MPU9250_ACCEL_FS_2 (0x00)
#define MPU9250_ACCEL_FS_4 (0x01)
#define MPU9250_ACCEL_FS_8 (0x02)
#define MPU9250_ACCEL_FS_16 (0x03)
#define MPU9250_ACCEL_SCALE_FACTOR_0 (16384)
#define MPU9250_ACCEL_SCALE_FACTOR_1 (8192)
#define MPU9250_ACCEL_SCALE_FACTOR_2 (4096)
#define MPU9250_ACCEL_SCALE_FACTOR_3 (2048)

//Giroscópio
#define MPU9250_GCONFIG_FS_SEL_LENGTH (2)
#define MPU9250_GCONFIG_FS_SEL_BIT (3)
#define MPU9250_RA_GYRO_CONFIG (0x1B)
#define MPU9250_GYRO_FS_250 (0x00)
#define MPU9250_GYRO_FS_500 (0x01)
#define MPU9250_GYRO_FS_1000 (0x02)
#define MPU9250_GYRO_FS_2000 (0x03)
#define MPU9250_GYRO_SCALE_FACTOR_0 (131)
#define MPU9250_GYRO_SCALE_FACTOR_1 (65.5)
#define MPU9250_GYRO_SCALE_FACTOR_2 (32.8)
#define MPU9250_GYRO_SCALE_FACTOR_3 (16.4)

//Clock Sources
#define MPU9250_CLOCK_INTERNAL (0x00)
#define MPU9250_CLOCK_PLL_XGYRO (0x01)
#define MPU9250_CLOCK_PLL_YGYRO (0x02)
#define MPU9250_CLOCK_PLL_ZGYRO (0x03)
#define MPU9250_CLOCK_KEEP_RESET (0x07)
#define MPU9250_CLOCK_PLL_EXT32K (0x04)
#define MPU9250_CLOCK_PLL_EXT19M (0x05)

#define MPU9250_RA_USER_CTRL (0x6A)
#define MPU9250_USERCTRL_I2C_MST_EN_BIT (5)

#define MPU9250_ACCEL_XOUT_H (0x3B)
#define MPU9250_GYRO_XOUT_H (0x43)

#define MPU9250_RA_INT_PIN_CFG (0x37)
#define MPU9250_INTCFG_BYPASS_EN_BIT (1)

#define BYTE_2_INT_BE(byte, i) ((int16_t)((byte[i] << 8) + (byte[i + 1])))
#define BYTE_2_INT_LE(byte, i) ((int16_t)((byte[i + 1] << 8) + (byte[i])))

typedef struct
{
  float x, y, z;
} vector_t;

typedef struct
{
  // Magnetometer
  vector_t mag_offset;
  vector_t mag_scale;

  // Gryoscope
  vector_t gyro_bias_offset;

  // Accelerometer
  vector_t accel_offset;
  vector_t accel_scale_lo;
  vector_t accel_scale_hi;

} calibration_t;


esp_err_t set_clock_source(uint8_t adrs);
esp_err_t i2c_mpu9250_init(calibration_t *c);

float get_accel_scale_inv(uint8_t scale_factor_3);
esp_err_t set_full_scale_accel_range(uint8_t scale_factor_4);

float get_gyro_scale_inv(uint8_t scale_factor);
esp_err_t set_full_scale_gyro_range(uint8_t scale_factor_2);

esp_err_t set_sleep_enabled(bool state);

float scale_accel(float value, float offset, float scale_lo, float scale_hi);
void accel(uint8_t bytes[6], vector_t *v);
void gryo(uint8_t bytes[6], vector_t *v);
esp_err_t get_accel(vector_t *v);
esp_err_t get_gyro(vector_t *v);
esp_err_t get_accel_gyro_mag(vector_t *va, vector_t *vg, vector_t *vm);

esp_err_t set_i2c_master_mode(bool state);
esp_err_t set_bypass_mux_enabled(bool state);
esp_err_t get_bypass_mux_enabled(bool *state);
esp_err_t get_mag(vector_t *v);

void mpu9250_print_settings(void);
void print_accel_settings(void);
void print_gyro_settings(void);
esp_err_t get_device_id(uint8_t *val);
esp_err_t get_sleep_enabled(bool *state);
esp_err_t get_i2c_master_mode(bool *state);
esp_err_t get_clock_source(uint8_t *clock_source);
esp_err_t get_gyro_settings(uint8_t bytes[3]);
esp_err_t get_full_scale_gyro_range(uint8_t *full_scale_gyro_range);
esp_err_t get_accel_settings(uint8_t bytes[3]);
esp_err_t get_full_scale_accel_range(uint8_t *full_scale_accel_range);

#endif // __MPU9250_H