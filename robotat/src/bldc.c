/**
 * @file bldc.c
 * @author Hans Alexander Burmester Campos
 * @brief Librería para inicialización y control de motores BLDC
 * @version 0.1
 * @date 2021-08-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "bldc.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#define LEDC_HS_MODE  LEDC_HIGH_SPEED_MODE

void bldc_esc_init(bldc_esc_t *const info_bldc_esc, int relacionRPM, float battery, float pulse_width_max, float pulse_width_min){

    if (info_bldc_esc){
        info_bldc_esc->KV2RPM = relacionRPM;
        info_bldc_esc->V_battery = battery;
        info_bldc_esc->pulse_width_max = pulse_width_max;
        info_bldc_esc->pulse_witdth_min = pulse_width_min;
    }
    else{
        printf("Estructura vacía");
    }
}

void config_gpio(gpio_num_t gpio_num_2){
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;                            //Set as output mode
    io_conf.intr_type = GPIO_INTR_DISABLE;                      //Disable interrupt
    io_conf.pin_bit_mask = gpio_num_2;                          //Bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = 0;                                   //Disable pull-down mode
    io_conf.pull_up_en = 0;                                     //Disable pull-up mode
    gpio_config(&io_conf);                                      //Configure GPIO with the given settings
}

void PWM_Init(gpio_num_t gpio_num, ledc_channel_t channel, ledc_timer_t timer){
    ledc_timer_config_t ledc_timer = {                                              // Timer Configuration
        .duty_resolution = LEDC_TIMER_12_BIT,                                       // Resolution of PWM duty
        .freq_hz = 50,                                                              // Frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,                                                 // Timer mode
        .timer_num = timer                                                          // Timer index
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {                                          // Configuration of channel
        .channel    = channel,
        .duty       = 0,
        .gpio_num   = gpio_num,
        .speed_mode = LEDC_HS_MODE,
        .hpoint     = 0,
        .timer_sel  = timer
    };
    ledc_channel_config(&ledc_channel);
}

void PWM_output(bldc_esc_t *const info_bldc_esc, ledc_channel_t channel, float input_controller){
    float duty_in;

    //------------ VERSIÓN 1
    //duty_in = ((info_bldc_esc->pulse_width_max - info_bldc_esc->pulse_witdth_min)/(info_bldc_esc->V_battery*info_bldc_esc->KV2RPM))*input_controller + info_bldc_esc->pulse_witdth_min;
    
    //----------- VERSIÓN 2
    duty_in = ((4095)/(20*info_bldc_esc->V_battery*info_bldc_esc->KV2RPM))*(info_bldc_esc->pulse_width_max - info_bldc_esc->pulse_witdth_min)*input_controller + ((4095*info_bldc_esc->pulse_witdth_min)/20);

    //------------VERSIÓN 3
    //duty_in = 4095*((info_bldc_esc->pulse_width_max - info_bldc_esc->pulse_witdth_min)*input_controller + 30000*info_bldc_esc->pulse_witdth_min)/600000; 

    ledc_set_duty(LEDC_HS_MODE, channel, (int) duty_in);
    ledc_update_duty(LEDC_HS_MODE, channel);
}


void inicializacion_ESC(ledc_channel_t channel){
    ledc_set_duty(LEDC_HS_MODE, channel, 205);
    ledc_update_duty(LEDC_HS_MODE, channel);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
};


void inicializacion_ESC_drone(void){
    ledc_set_duty(LEDC_HS_MODE, LEDC_CHANNEL_0, 205);
    ledc_update_duty(LEDC_HS_MODE, LEDC_CHANNEL_0);

    ledc_set_duty(LEDC_HS_MODE, LEDC_CHANNEL_1, 205);
    ledc_update_duty(LEDC_HS_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(LEDC_HS_MODE, LEDC_CHANNEL_2, 205);
    ledc_update_duty(LEDC_HS_MODE, LEDC_CHANNEL_2);

    ledc_set_duty(LEDC_HS_MODE, LEDC_CHANNEL_3, 205);
    ledc_update_duty(LEDC_HS_MODE, LEDC_CHANNEL_3);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
};

