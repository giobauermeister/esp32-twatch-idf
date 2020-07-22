#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define MOTOR_PIN             4
#define INFRARED_PIN          13

#define I2C_PORT_NUMBER       I2C_NUM_1
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22
#define I2C_FREQ_HZ		        100000 //I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0  // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0  // I2C master doesn't need buffer

#define BMA423_SENSOR_ADDR    0x13

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
  int i2c_master_port = I2C_PORT_NUMBER;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_SDA_PIN;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_SCL_PIN;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void) {
  printf("Hello TWatch\n");

  printf("Initializing I2C...\n");
  i2c_master_init();
  printf("Initializing I2C OK\n");

  gpio_reset_pin(MOTOR_PIN);
  gpio_reset_pin(INFRARED_PIN);
  gpio_set_direction(MOTOR_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(INFRARED_PIN, GPIO_MODE_OUTPUT);

  gpio_set_level(MOTOR_PIN, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  gpio_set_level(MOTOR_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);

  gpio_set_level(MOTOR_PIN, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  gpio_set_level(MOTOR_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);

  gpio_set_level(MOTOR_PIN, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  gpio_set_level(MOTOR_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);

  gpio_set_level(MOTOR_PIN, 1);
  vTaskDelay(50 / portTICK_PERIOD_MS);
  gpio_set_level(MOTOR_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);

  while(1)
  {
    gpio_set_level(INFRARED_PIN, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(INFRARED_PIN, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  
}