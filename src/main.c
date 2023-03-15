/*
 * OpenThread Sensor Node
 * Sleepy End Device
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
//#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>

//#include <openthread/thread.h>
//#include <openthread/udp.h>

#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

#define DEBUG
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

// define the BME280 sensor
#include "bme280.h"
struct bme280_result_type bme280_result;

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

		// wait for serial connection !!! blocking !!!
		while (!dtr) {
			uart_line_ctrl_get(usbdev, UART_LINE_CTRL_DTR, &dtr);
			k_sleep(K_MSEC(100));
		} 

		err = gpio_pin_set_dt(&led_green, 0);
		printk("--- ot-sensor-lp ---\n");
	#endif

	while(true) {
		//------------------------------------
		// go to sleep
		//------------------------------------
		#ifdef DEBUG
			printk("sleep...\n");
			//err = pm_device_action_run(usbdev, PM_DEVICE_ACTION_SUSPEND);
		#endif
		pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);
		k_sleep(K_SECONDS(SLEEP_TIME));
		pm_device_action_run(i2c_device, PM_DEVICE_ACTION_RESUME);
		#ifdef DEBUG
			//err = pm_device_action_run(usbdev, PM_DEVICE_ACTION_RESUME);
			printk("...wake up\n");
		#endif
		
		//-----------------------------
		// init sensor
		//-----------------------------
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

		// init sensor
		err = bme280_chip_init();

		// read chip id
		err = gpio_pin_configure_dt(&led_blue, GPIO_OUTPUT_HIGH);
		err = gpio_pin_set_dt(&led_blue, 1);
		#ifdef DEBUG
			printk("Chip ID 0x%02X \n", bme280_read_chip_id());
		#endif

		//------------------------------------
		// take measurement
		//------------------------------------
		bme280_result = bme280_read_values();
		err = gpio_pin_set_dt(&led_blue, 1);

		// power down BME280 sensor
		err = gpio_pin_set_dt(&bme280_power, LOW);

		//------------------------------------
		// construct json message
		//------------------------------------
		snprintf(json_buf, sizeof(json_buf),
			"{ \"id\": \"AABBCCDDEEFF0011\", \"temp\": %.2f, \"press\": %.2f, \"hum\": %.2f }",
			bme280_result.temp, bme280_result.press, bme280_result.hum);

		#ifdef DEBUG
			printk("JSON message: %s \n",json_buf);
		#endif
	
		// clear json buffer
		json_buf[0] = "\0";

	};
}
