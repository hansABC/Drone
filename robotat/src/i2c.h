/**
 * @file i2c.h
 * @author Hans Burmester
 * @brief 
 * @version 0.1
 * @date 2021-08-15
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef __I2C_H
#define __I2C_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t i2c_master_init(uint8_t i2c_num, gpio_num_t gpio_sda, gpio_num_t gpio_scl);

esp_err_t i2c_write_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_write_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t data);

esp_err_t i2c_read_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_read_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data);

esp_err_t i2c_write_bits(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t length, uint8_t value);
esp_err_t i2c_write_bit(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t value);

esp_err_t i2c_read_bit(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t *result);
 
#endif