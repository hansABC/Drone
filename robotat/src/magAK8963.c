/**
 * @file magAK8963.c
 * @author Hans Burmester (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-08-29
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c.h"
#include "mpu9250.h"
#include "magAK8963.h"

static const char *TAG = "AK8963";

static bool initialised = false;
static uint8_t i2c_num;
calibration_t *cal;
vector_t asa;

esp_err_t AK8963_init(i2c_port_t i2c_number, calibration_t *c){
  if (initialised){
    ESP_LOGE(TAG, "AK8963_init has already been called");
    return ESP_ERR_INVALID_STATE;
  }
  i2c_num = i2c_number;
  cal = c;

  //Connection with magnetometer
  uint8_t id;
  ak8963_get_device_id(&id);

  if (id & AK8963_WHO_AM_I_RESPONSE){
    ak8963_get_sensitivity_adjustment_values();
    vTaskDelay(10 / portTICK_RATE_MS);
    ak8963_set_cntl(AK8963_CNTL_MODE_CONTINUE_MEASURE_2);
    initialised = true;
    return ESP_OK;
  }
  else{
    ESP_LOGI(TAG, "AK8963: Device ID is not equal to 0x%02x, device value is 0x%02x", AK8963_WHO_AM_I_RESPONSE, id);
    return ESP_ERR_INVALID_STATE;
  }
}

esp_err_t ak8963_get_device_id(uint8_t *val){
  return i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_WHO_AM_I, val);
}

esp_err_t ak8963_get_cntl(uint8_t *mode){
  return i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_CNTL, mode);
}

esp_err_t ak8963_set_cntl(uint8_t mode){
  return i2c_write_byte(i2c_num, AK8963_ADDRESS, AK8963_CNTL, mode);
}

esp_err_t ak8963_get_sensitivity_adjustment_values(){

  esp_err_t ret;

  // Need to set to Fuse mode to get valid values from this.
  uint8_t current_mode;
  ret = ak8963_get_cntl(&current_mode);
  if (ret != ESP_OK)
    return ret;

  ret = ak8963_set_cntl(AK8963_CNTL_MODE_FUSE_ROM_ACCESS);
  if (ret != ESP_OK)
    return ret;

  vTaskDelay(20 / portTICK_RATE_MS);

  uint8_t xi, yi, zi;
  ret = i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ASAX, &xi);
  if (ret != ESP_OK)
    return ret;

  ret = i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ASAY, &yi);
  if (ret != ESP_OK)
    return ret;

  ret = i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ASAZ, &zi);
  if (ret != ESP_OK)
    return ret;

  // Get the ASA* values
  asa.x = (((float)xi - 128.0) * 0.5) / 128.0 + 1.0;
  asa.y = (((float)yi - 128.0) * 0.5) / 128.0 + 1.0;
  asa.z = (((float)zi - 128.0) * 0.5) / 128.0 + 1.0;

  return ak8963_set_cntl(current_mode);
}


//------------------------------------------------LECTURA DE DATOS------------------------------------------------------
esp_err_t ak8963_get_mag_raw(uint8_t bytes[6]){
  i2c_read_bytes(i2c_num, AK8963_ADDRESS, AK8963_XOUT_L, bytes, 6);
  vTaskDelay(1 / portTICK_RATE_MS);
  uint8_t b;
  i2c_read_byte(i2c_num, AK8963_ADDRESS, AK8963_ST2, &b);

  return ESP_OK;
}

esp_err_t ak8963_get_mag(vector_t *v){

  esp_err_t ret;
  uint8_t bytes[6];

  ret = ak8963_get_mag_raw(bytes);
  if (ret != ESP_OK)
  {
    return ret;
  }

  float xi = (float)BYTE_2_INT_LE(bytes, 0);
  float yi = (float)BYTE_2_INT_LE(bytes, 2);
  float zi = (float)BYTE_2_INT_LE(bytes, 4);

  v->x = (xi * asa.x - cal->mag_offset.x) * cal->mag_scale.x;
  v->y = (yi * asa.y - cal->mag_offset.y) * cal->mag_scale.y;
  v->z = (zi * asa.z - cal->mag_offset.z) * cal->mag_scale.z;

  return ESP_OK;
}

//------------------------------------------------IMPRESIÓN DE CONFIGURACIÓN--------------------------------------------

void ak8963_print_settings(void){
  char *cntl_modes[] = {"0x00 (Power-down mode)",
                        "0x01 (Single measurement mode)",
                        "0x02 (Continuous measurement mode 1: 8Hz)",
                        "0x03 Invalid mode",
                        "0x04 (External trigger measurement mode)",
                        "0x05 Invalid mode",
                        "0x06 (Continuous measurement mode 2: 100Hz)",
                        "0x07 Invalid mode",
                        "0x08 (Self-test mode)",
                        "0x09 Invalid mode",
                        "0x0A Invalid mode",
                        "0x0B Invalid mode",
                        "0x0C Invalid mode",
                        "0x0D Invalid mode",
                        "0x0E Invalid mode",
                        "0x0F Invalid mode",
                        "0x0F (Fuse ROM access mode)"};

  uint8_t device_id;
  ESP_ERROR_CHECK(ak8963_get_device_id(&device_id));

  uint8_t cntl;
  ESP_ERROR_CHECK(ak8963_get_cntl(&cntl));

  ESP_LOGI(TAG, "Magnetometer (Compass):");
  ESP_LOGI(TAG, "--> i2c address: 0x%02d", i2c_num);
  ESP_LOGI(TAG, "--> initialised: %s", initialised ? "true" : "false");
  ESP_LOGI(TAG, "--> Device ID: 0x%02x", device_id);
  ESP_LOGI(TAG, "--> Mode: %s", cntl_modes[cntl]);
  ESP_LOGI(TAG, "--> ASA Scalars:");
  ESP_LOGI(TAG, "  --> x: %f", asa.x);
  ESP_LOGI(TAG, "  --> y: %f", asa.y);
  ESP_LOGI(TAG, "  --> z: %f", asa.z);
  ESP_LOGI(TAG, "--> Offset:");
  ESP_LOGI(TAG, "  --> x: %f", cal->mag_offset.x);
  ESP_LOGI(TAG, "  --> y: %f", cal->mag_offset.y);
  ESP_LOGI(TAG, "  --> z: %f", cal->mag_offset.z);
  ESP_LOGI(TAG, "--> Scale:");
  ESP_LOGI(TAG, "  --> x: %f", cal->mag_scale.x);
  ESP_LOGI(TAG, "  --> y: %f", cal->mag_scale.y);
  ESP_LOGI(TAG, "  --> z: %f", cal->mag_scale.z);
}