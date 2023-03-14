/*
 * OpenThread Sensor Node
 * Sleepy End Device
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

//#include <openthread/thread.h>
//#include <openthread/udp.h>

#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

//#define DEBUG
#ifdef DEBUG
	#include <zephyr/drivers/uart.h>
	#include <zephyr/usb/usb_device.h>
	#include <zephyr/logging/log.h>
	LOG_MODULE_REGISTER(cdc_acm_echo, LOG_LEVEL_INF);
#endif

// the devicetree node identifier for the "led1_green" alias
#define LED1_GREEN_NODE DT_ALIAS(led1_green)
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(LED1_GREEN_NODE, gpios);

#if DT_NODE_HAS_STATUS(LED1_GREEN_NODE, okay)
	#define LED1_GREEN_PIN DT_GPIO_PIN(LED1_GREEN_NODE, gpios)
#endif

// the devicetree node identifier for the "led1_blue" alias
#define LED1_BLUE_NODE DT_ALIAS(led1_blue)
static const struct gpio_dt_spec led_blue = GPIO_DT_SPEC_GET(LED1_BLUE_NODE, gpios);

#if DT_NODE_HAS_STATUS(LED1_BLUE_NODE, okay)
	#define LED1_BLUE_PIN DT_GPIO_PIN(LED1_BLUE_NODE, gpios)
#endif

// the devicetree node identifier for our self-defined "pwr" alias.
#define PWR_IO_NODE DT_ALIAS(pwr)
static const struct gpio_dt_spec bme280_power = GPIO_DT_SPEC_GET(PWR_IO_NODE, gpios);

#if DT_NODE_HAS_STATUS(PWR_IO_NODE, okay)
#define PWR_IO_PIN DT_GPIO_PIN(PWR_IO_NODE, gpios)
#endif

#define HIGH 1
#define LOW 0

#define SLEEP_TIME 10
#define SYS_REBOOT_COLD   1
#define SYS_REBOOT_WARM   0

// define the i2c bus
#define I2C_NODE DT_NODELABEL(i2c0)
static const struct device *i2c_device = DEVICE_DT_GET(I2C_NODE);

// define the BME280 sensor
#include "bme280.h"

//-----------------------------
void main(void)
//-----------------------------
{
	char json_buf[100];
	int err;

	uint8_t eui64[8];
	char eui64_id[17];
	char buf[3];

	#ifdef DEBUG
		err = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_HIGH);
		err = gpio_pin_set_dt(&led_green, 1);
		k_sleep(K_MSEC(500));

		const struct device *usbdev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
		uint32_t dtr = 0;

		if (usb_enable(NULL)) {
			LOG_ERR("Failed to enable USB");
			return;
		}

/*		// wait for serial connection !!! blocking !!!
		while (!dtr) {
			uart_line_ctrl_get(usbdev, UART_LINE_CTRL_DTR, &dtr);
			k_sleep(K_MSEC(100));
		} 
*/
		err = gpio_pin_set_dt(&led_green, 0);
		printk("--- ot-sensor-lp ---\n");
	#endif

	//------------------------------------
	// go to sleep
	//------------------------------------
	#ifdef DEBUG
		printk("sleep...\n");
		err = pm_device_action_run(usbdev, PM_DEVICE_ACTION_SUSPEND);
	#endif
	pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);
	k_sleep(K_SECONDS(SLEEP_TIME));
	pm_device_action_run(i2c_device, PM_DEVICE_ACTION_RESUME);
	#ifdef DEBUG
		err = pm_device_action_run(usbdev, PM_DEVICE_ACTION_RESUME);
		printk("...wake up\n");
	#endif
	
	//-----------------------------
	// init sensor
	//-----------------------------
	#ifdef DEBUG
		printk("init sensor\n");
	#endif
	// power up i2c sensor
	err = gpio_pin_configure_dt(&bme280_power, GPIO_OUTPUT_HIGH);
	err = gpio_pin_set_dt(&bme280_power, HIGH);
	k_sleep(K_MSEC(100)); // sensor boot time

	if(!device_is_ready(i2c_device)) {
		#ifdef DEBUG
			printk("I2C bus not ready!\n");
		#endif
		return;
	}

	// read chip id
	write_reg_buffer[0] = BME280_CHIP_ID;
	err = i2c_write_read(i2c_device, BME280_I2C_ADDRESS, write_reg_buffer, 1, read_buf_id, 1);
	if(err != 0) {
		#ifdef DEBUG
			printk("BME280 chip id read failed.\n");
		#endif
		return;
	}
	bme_data.chip_id = read_buf_id[0];
	err = gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_HIGH);
	err = gpio_pin_set_dt(&led_blue, 1);
	#ifdef DEBUG
		printk("Chip ID 0x%02X \n", bme_data.chip_id);
		printk("Sensor started\n");
	#endif
	k_sleep(K_MSEC(500));
	err = gpio_pin_set_dt(&led_blue, 0);

	//------------------------------------
	// take measurement
	//------------------------------------
/*
	struct sensor_value temp, humidity;

	if (sensor_sample_fetch(sht)) {
		#ifdef DEBUG
			printk("Failed to fetch sample from SHT4X device\n");
		#endif
		return;
	}
	sensor_channel_get(sht, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	sensor_channel_get(sht, SENSOR_CHAN_HUMIDITY, &humidity);

	//------------------------------------
	// construct json message
	//------------------------------------
	snprintf(json_buf, sizeof(json_buf),
		"{ \"id\": \"%s\", \"temp\": %d.%d, \"hum\": %d.%d }",
		eui64_id, temp.val1, temp.val2, humidity.val1, humidity.val2);

	#ifdef DEBUG
		printk("JSON message: %s",json_buf);
	#endif
*/
	snprintf(json_buf, sizeof(json_buf),
		"{ \"id\": \"AABBCCDDEEFF0011\", \"temp\": 21.00, \"hum\": 47.00 }");
	#ifdef DEBUG
		printk("JSON message: %s \n",json_buf);
	#endif

	// clear json buffer
	json_buf[0] = "\0";

	//------------------------------------
	// hard reset
	//------------------------------------
	#ifdef DEBUG
		printk("reboot system\n");
	#endif
	err = gpio_pin_set_dt(&bme280_power, LOW);

	sys_reboot(SYS_REBOOT_WARM);

	while (true) {
		k_sleep(K_MSEC(100));
	};
}
