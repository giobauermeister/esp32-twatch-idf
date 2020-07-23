#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "bma423_registers.h"

#define MOTOR_PIN             4
#define INFRARED_PIN          13

#define I2C_PORT_NUMBER       I2C_NUM_0
#define I2C_SDA_PIN           21
#define I2C_SCL_PIN           22
#define I2C_FREQ_HZ		        100000 //I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0  // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0  // I2C master doesn't need buffer

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

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

// static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
// {
//     int ret;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, BMA4_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     if (ret != ESP_OK) {
//         return ret;
//     }
//     vTaskDelay(30 / portTICK_RATE_MS);
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, BMA4_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
//     i2c_master_read_byte(cmd, data_h, ACK_VAL);
//     i2c_master_read_byte(cmd, data_l, NACK_VAL);
//     i2c_master_stop(cmd);
//     ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }

static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t reg_addr)
{
  esp_err_t ret;
  uint8_t data=0;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  //printf("[%s] i2c first block complete\n", esp_err_to_name(ret));

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, slave_addr << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  //printf("[%s] i2c second block complete\n", esp_err_to_name(ret));
  printf("data from sensor: 0x%02x\n", data);
  if(data == 0x13)
  {
    printf("BMA423 at address 0x%02x is working!\n", slave_addr);
  } else {
    printf("Could not connect to 0x%02x\n", slave_addr);
  }
  return ret;
}

void scan_i2c_devices(void)
{
  printf("scanning the bus...\n");
	int devices_found = 0;
	
	for(int address = 1; address < 127; address++) {
	
		// create and execute the command link
		i2c_cmd_handle_t cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
		i2c_master_stop(cmd);
		if(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS) == ESP_OK) {
			printf("-> found device with address 0x%02x\r\n", address);
			devices_found++;
		}
		i2c_cmd_link_delete(cmd);
	}
	if(devices_found == 0) printf("\r\n-> no devices found\r\n");
	printf("...scan completed!\n");
}

/************  MAIN APP *************/
void app_main(void) {
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  printf("Hello TWatch\n");

  printf("Initializing I2C...\n");
  i2c_master_init();
  printf("Initializing I2C OK\n");

  scan_i2c_devices();  

  i2c_master_sensor_test(I2C_PORT_NUMBER, BMA423_ADDR, BMA423_CHIP_ID);

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