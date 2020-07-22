#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define MOTOR_PIN     (gpio_num_t)4
#define INFRARED_PIN  (gpio_num_t)13

extern "C" {
    void app_main(void);
}

void app_main(void) {
  printf("Hello TWatch\n");

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