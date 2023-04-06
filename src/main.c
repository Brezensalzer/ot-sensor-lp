/*
 * OpenThread Sensor Node
 * Sleepy End Device
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <string.h>
#include <stdio.h>

#include <openthread/thread.h>
#include <openthread/udp.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <openthread/coap.h>

#define DEBUG
#ifdef DEBUG
	#include <zephyr/drivers/uart.h>
	#include <zephyr/usb/usb_device.h>
	#include <zephyr/logging/log.h>
	LOG_MODULE_REGISTER(ot_sensor, LOG_LEVEL_INF);
#endif

// the devicetree node identifier for the "led1_green" alias
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
	#define LED0_PIN DT_GPIO_PIN(LED0_NODE, gpios)
#endif

// the devicetree node identifier for our self-defined "pwr" alias.
#define PWR_IO_NODE DT_ALIAS(pwr)
static const struct gpio_dt_spec bme280_power = GPIO_DT_SPEC_GET(PWR_IO_NODE, gpios);

#if DT_NODE_HAS_STATUS(PWR_IO_NODE, okay)
#define PWR_IO_PIN DT_GPIO_PIN(PWR_IO_NODE, gpios)
#endif

// helper
#define HIGH 1
#define LOW 0
#define SLEEP_TIME 60

// define the BME280 sensor
#include "bme280.h"
struct bme280_result_type bme280_result;

//-----------------------------
void coap_response_handler(void * p_context, otMessage * p_message, const otMessageInfo * p_msginfo, otError result)
//-----------------------------
{
	if (result == OT_ERROR_NONE) {
		#ifdef DEBUG
			LOG_INF("Message delivery confirmed");
		#endif
	}
	else {
		#ifdef DEBUG
			LOG_INF("Message delivery not confirmed rc: %d", result);
		#endif
	}
}

//-----------------------------
void coap_send(otInstance *ot_instance, char *jsonbuf)
//-----------------------------
{
	otError ot_error = OT_ERROR_NONE;
	char ip6buf[40];

	otMessageInfo msgInfo;
	memset(&msgInfo, 0, sizeof(msgInfo)); // why?
	msgInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
	otIp6AddressFromString("fdd0:15d6:9e7f:2:0:0:c0a8:10b", &msgInfo.mPeerAddr);

	otMessage *msg = otCoapNewMessage( ot_instance, NULL);
	otCoapMessageInit(msg, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT);

	ot_error = otCoapMessageAppendUriPathOptions(msg, "openthread");
	#ifdef DEBUG
		LOG_INF("UriPathOption rc: %s", otThreadErrorToString(ot_error));
	#endif

	ot_error = otCoapMessageAppendContentFormatOption(msg, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
	#ifdef DEBUG
		LOG_INF("ContentFormatOption rc: %s", otThreadErrorToString(ot_error));
	#endif

	ot_error = otCoapMessageSetPayloadMarker(msg);
	#ifdef DEBUG
		LOG_INF("SetPayloadMarker rc: %s", otThreadErrorToString(ot_error));
	#endif

	ot_error = otMessageAppend(msg, jsonbuf, strlen(jsonbuf));
	#ifdef DEBUG
		LOG_INF("MessageAppend rc: %s", otThreadErrorToString(ot_error));
	#endif

	ot_error = otCoapSendRequest(ot_instance, msg, &msgInfo, coap_response_handler, NULL);
	#ifdef DEBUG
		otIp6AddressToString(&msgInfo.mPeerAddr, ip6buf, 39);
		LOG_INF("COAP endpoint IP: %s, Port: %d", ip6buf, msgInfo.mPeerPort);
		LOG_INF("COAP message rc: %s", otThreadErrorToString(ot_error));
	#endif

	otMessageFree(msg);
}

//-----------------------------
void main(void)
//-----------------------------
{
	char json_buf[80];
	int err;
	
	otExtAddress eui64;
	char eui64_id[25] = "\0";
	char buf[3];

	#ifdef DEBUG
		err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_HIGH);
		err = gpio_pin_set_dt(&led, HIGH);
		k_sleep(K_MSEC(5000));

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
		
		err = gpio_pin_set_dt(&led, LOW);
		LOG_INF("--------------------");
		LOG_INF("--- ot-sensor-lp ---");
		LOG_INF("--------------------");
	#endif

	//-------------------------------------
	// fetch OpenThread instance and EUI64
	//-------------------------------------
	otInstance *ot_instance;
	ot_instance = openthread_get_default_instance();

	otPlatRadioGetIeeeEui64(ot_instance, eui64.m8);

	// convert EUI64 to hex string
	for (uint8_t i=0; i < 8; i++) {
		snprintk(buf, sizeof(buf),"%02X",eui64.m8[i]);
		#ifdef DEBUG
			LOG_INF("byte nr. %d: %s", i, buf);
		#endif
		strcat(eui64_id,buf);
	}
	
	// set TX power to +8dbm
	err = otPlatRadioSetTransmitPower(ot_instance, 8);

	// check if we are connected to a network
	int device_state = otThreadGetDeviceRole(ot_instance);
	while ((device_state == OT_DEVICE_ROLE_DETACHED)
			||(device_state == OT_DEVICE_ROLE_DISABLED)) {
		err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_HIGH);
		err = gpio_pin_set_dt(&led, HIGH);
		k_sleep(K_MSEC(1000));
		device_state = otThreadGetDeviceRole(ot_instance);
	}
	err = gpio_pin_set_dt(&led, LOW);


	//------------------------------------
	// main loop
	//------------------------------------
	while(true) {
		//------------------------------------
		// go to sleep
		//------------------------------------
		#ifdef DEBUG
			LOG_INF("sleep...\n");
		#endif
		pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);
		k_sleep(K_SECONDS(SLEEP_TIME));
		pm_device_action_run(i2c_device, PM_DEVICE_ACTION_RESUME);
		#ifdef DEBUG
			LOG_INF("...wake up");
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
				LOG_ERR("I2C bus not ready!");
			#endif
			return;
		}

		// init sensor
		err = bme280_chip_init();

		// read chip id
		#ifdef DEBUG
			LOG_INF("I2C Chip ID: 0x%02X", bme280_read_chip_id());
		#endif

		//------------------------------------
		// take measurement
		//------------------------------------
		bme280_result = bme280_read_values();

		// power down BME280 sensor
		err = gpio_pin_set_dt(&bme280_power, LOW);

		//------------------------------------
		// construct json message
		//------------------------------------
		snprintk(json_buf, sizeof(json_buf),
			"{ \"id\": \"%s\", \"temp\": %.2f, \"press\": %.2f, \"hum\": %.2f }",
			eui64_id, bme280_result.temp, bme280_result.press, bme280_result.hum);

		#ifdef DEBUG
			LOG_INF("JSON message: %s",json_buf);
		#endif
	
		//------------------------------------
		// send COAP message 
		// -- only if we are connected
		//------------------------------------
		if(OT_DEVICE_ROLE_CHILD == otThreadGetDeviceRole(ot_instance)) {
			coap_send(ot_instance, json_buf);
		}
		else {
			#ifdef DEBUG
				LOG_ERR("not connected to a network, message not sent");
			#endif
		}

		// clear json buffer
		json_buf[0] = "\0";
	};
}
