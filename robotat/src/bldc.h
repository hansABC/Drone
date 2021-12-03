/**
 * @file bldc.h
 * @author Hans Alexander Burmester Campos
 * @brief Header file para la librería de motores BLDC
 * @version 0.1
 * @date 2021-08-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef BLDC_H
#define BLDC_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"


typedef struct 
{
    int KV2RPM;
    float V_battery;
    float pulse_width_max;
    float pulse_witdth_min;
} bldc_esc_t;

// Prototipos de funciones

void bldc_esc_init(bldc_esc_t *const info_bldc_esc, int relacionRPM, float battery, float pulse_width_max, float pulse_width_min);
void config_gpio(gpio_num_t gpio_num_2);
void PWM_Init(gpio_num_t gpio_num, ledc_channel_t channel, ledc_timer_t timer);
void PWM_output(bldc_esc_t *const info_bldc_esc, ledc_channel_t channel, float input_controller);
void inicializacion_ESC(ledc_channel_t channel);
void inicializacion_ESC_drone(void);

#endif
