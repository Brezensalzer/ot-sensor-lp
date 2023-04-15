/*
 * OpenThread Sensor Node
 * Sleepy End Device
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <string.h>
#include <stdio.h>

#include <openthread/thread.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <openthread/coap.h>
#include <openthread/dns_client.h>

//#define DEBUG
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

// ADC Voodoo
#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

// helper
#define HIGH 1
#define LOW 0
#define SLEEP_TIME 60

// define the BME280 sensor
#include "bme280.h"
struct bme280_result_type bme280_result;

// hostname and ip address of coap server
const char *dnsResolver = "ff03::1"; // mesh-local multicast address
const char *serviceLabel = "coap2mqtt";
const char *serviceName = "_coap._udp.default.service.arpa.";
const char *serviceHostname = "sensordata.default.service.arpa.";
otIp6Address coapServer;
bool resolved = false;

//--------------------------------------------------------------------------
void coap_send(otInstance *ot_instance, char *jsonbuf)
//--------------------------------------------------------------------------
{
	otError ot_error = OT_ERROR_NONE;
	char ip6buf[40];
	otMessage *msg = NULL;
	otMessageInfo msgInfo;
	memset(&msgInfo, 0, sizeof(msgInfo)); // why?
	msgInfo.mPeerPort = OT_DEFAULT_COAP_PORT;
	msgInfo.mPeerAddr = coapServer;

	msg = otCoapNewMessage( ot_instance, NULL);
	otCoapMessageInit(msg, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT);
	otCoapMessageGenerateToken(msg, OT_COAP_MAX_TOKEN_LENGTH);

	ot_error = otCoapMessageAppendUriPathOptions(msg, "sensordata");
	#ifdef DEBUG
		if (ot_error != OT_ERROR_NONE)
		 { LOG_INF("UriPathOption rc: %s", otThreadErrorToString(ot_error)); }
	#endif

	ot_error = otCoapMessageAppendContentFormatOption(msg, OT_COAP_OPTION_CONTENT_FORMAT_JSON);
	#ifdef DEBUG
		if (ot_error != OT_ERROR_NONE)
		{ LOG_INF("ContentFormatOption rc: %s", otThreadErrorToString(ot_error)); }
	#endif

	ot_error = otCoapMessageSetPayloadMarker(msg);
	#ifdef DEBUG
		if (ot_error != OT_ERROR_NONE)
		{ LOG_INF("SetPayloadMarker rc: %s", otThreadErrorToString(ot_error)); }
	#endif

	ot_error = otMessageAppend(msg, jsonbuf, strlen(jsonbuf));
	#ifdef DEBUG
		if (ot_error != OT_ERROR_NONE)
		{ LOG_INF("MessageAppend rc: %s", otThreadErrorToString(ot_error)); }
	#endif

	ot_error = otCoapSendRequest(ot_instance, msg, &msgInfo, NULL, NULL);
	#ifdef DEBUG
		otIp6AddressToString(&msgInfo.mPeerAddr, ip6buf, 39);
		LOG_INF("COAP endpoint IP: %s, Port: %d", ip6buf, msgInfo.mPeerPort);
		LOG_INF("COAP message rc: %s", otThreadErrorToString(ot_error));
	#endif
	// only call otMessageFree in case of error!
	// otherwise no message is sent and the system hangs!
	if (ot_error != OT_ERROR_NONE) {
		otMessageFree(msg);	
	}
}

//--------------------------------------------------------------------------
static void resolve_callback(otError aError, const otDnsServiceResponse *aResponse, void *aContext)
//--------------------------------------------------------------------------
{
	otError ot_error;
	char *buf[40];
	#ifdef DEBUG
		LOG_INF("DNS resolver callback");
	#endif

	if (aError == OT_ERROR_NONE) {
		ot_error = otDnsServiceResponseGetHostAddress(aResponse, serviceHostname, 0, &coapServer, NULL);
		#ifdef DEBUG
			otIp6AddressToString(&coapServer, buf, 39);
			LOG_INF("service hostname: %s", serviceHostname);
			LOG_INF("service hostname successfully resolved %s", buf);
		#endif
		resolved = true;
	}
	else {
		#ifdef DEBUG
			LOG_ERR("service hostname not resolved!");
		#endif
	}
}

//--------------------------------------------------------------------------
static void resolveCoapServer(otInstance *ot_instance)
//--------------------------------------------------------------------------
{
	otError ot_error;
	char *buf_old[40];
	struct openthread_context *ctx = openthread_get_default_context();

	otDnsQueryConfig *queryConfig = NULL;
	queryConfig = otDnsClientGetDefaultConfig(ot_instance);
	otIp6AddressToString(&queryConfig->mServerSockAddr.mAddress, buf_old, 39);
	#ifdef DEBUG
		LOG_INF("default mDNS server: %s", buf_old);
	#endif

	otIp6AddressFromString(dnsResolver, &queryConfig->mServerSockAddr.mAddress);
	queryConfig->mResponseTimeout = 1000;	
	otDnsClientSetDefaultConfig(ot_instance, queryConfig);
	#ifdef DEBUG
		otIp6AddressToString(&queryConfig->mServerSockAddr.mAddress, dnsResolver, 39);
		LOG_INF("new mDNS server: %s", dnsResolver);
	#endif

	openthread_api_mutex_lock(ctx);
	ot_error = otDnsClientResolveService(ot_instance, serviceLabel, serviceName, resolve_callback, ctx, queryConfig);
	openthread_api_mutex_unlock(ctx);
	#ifdef DEBUG
		LOG_INF("started DNS resolving service rc: %s", otThreadErrorToString(ot_error));
	#endif

	while (resolved == false) {
		k_sleep(K_MSEC(500));
	}
}

//--------------------------------------------------------------------------
void main(void)
//--------------------------------------------------------------------------
{
	char json_buf[100];
	int err;
	
	otExtAddress eui64;
	char eui64_id[25] = "\0";
	char buf[3];

	// USB Voodoo
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

	// configure i2c power pin
	err = gpio_pin_configure_dt(&bme280_power, GPIO_OUTPUT_HIGH);

	// DNS resolve hostname of COAP Server
	//otError ot_error = otIp6AddressFromString(coapIpv6, &coapServer);
	resolveCoapServer(ot_instance);

	//-------------------------------------
	// Setup ADC
	//-------------------------------------
	int16_t adc_buf;
	int32_t bat_mv;
	struct adc_sequence sequence = {
		.buffer = &adc_buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(adc_buf),
	};

	/* Configure channel prior to sampling. */
	if (!device_is_ready(adc_channels[0].dev)) {
		#ifdef DEBUG
			LOG_ERR("ADC controller device not ready");
		#endif
		return;
	}

	err = adc_channel_setup_dt(&adc_channels[0]);
	if (err < 0) {
		#ifdef DEBUG
			LOG_ERR("Could not setup channel #0 (%d)", err);
		#endif
		return;
	}

	// power down all devices
	pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);
	pm_device_action_run(adc_channels[0].dev, PM_DEVICE_ACTION_SUSPEND);

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
		//err = gpio_pin_set_dt(&led, LOW);
		k_sleep(K_SECONDS(SLEEP_TIME));
		//err = gpio_pin_set_dt(&led, HIGH);
		#ifdef DEBUG
			LOG_INF("...wake up");
		#endif
		
		//-----------------------------
		// i2c sensor
		//-----------------------------
		// power up i2c sensor
		pm_device_action_run(i2c_device, PM_DEVICE_ACTION_RESUME);
		err = gpio_pin_set_dt(&bme280_power, HIGH);
		k_sleep(K_MSEC(50)); // sensor boot time

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

		// take measurement
		bme280_result = bme280_read_values();

		// power down BME280 sensor
		err = gpio_pin_set_dt(&bme280_power, LOW);

		// power down i2c bus
		pm_device_action_run(i2c_device, PM_DEVICE_ACTION_SUSPEND);

		//-----------------------------
		// ADC
		//-----------------------------
		// power up ADC
		pm_device_action_run(adc_channels[0].dev, PM_DEVICE_ACTION_RESUME);

		// get battery voltage
		(void)adc_sequence_init_dt(&adc_channels[0], &sequence);
		err = adc_read(adc_channels[0].dev, &sequence);
		bat_mv = adc_buf;
		err = adc_raw_to_millivolts_dt(&adc_channels[0], &bat_mv);

		// power down ADC
		pm_device_action_run(adc_channels[0].dev, PM_DEVICE_ACTION_SUSPEND);

		//------------------------------------
		// construct json message
		//------------------------------------
		snprintk(json_buf, sizeof(json_buf),
			"{ \"id\": \"%s\", \"batt\": %d, \"temp\": %.2f, \"press\": %.2f, \"hum\": %.2f }",
			eui64_id, bat_mv, bme280_result.temp, bme280_result.press, bme280_result.hum);

		#ifdef DEBUG
			LOG_INF("JSON message: %s",json_buf);
		#endif
	
		//------------------------------------
		// send COAP message 
		// -- only if we are connected
		//------------------------------------
		// start COAP
		err = otCoapStart(ot_instance, OT_DEFAULT_COAP_PORT);
		#ifdef DEBUG
			LOG_INF("COAP started, rc: %s", otThreadErrorToString(err));
		#endif

		if(OT_DEVICE_ROLE_CHILD == otThreadGetDeviceRole(ot_instance)) {
			coap_send(ot_instance, json_buf);
		}
		else {
			#ifdef DEBUG
				LOG_ERR("not connected to a network, message not sent");
			#endif
		}

		// stop COAP
		err = otCoapStop(ot_instance);
		#ifdef DEBUG
			LOG_INF("COAP started, rc: %s", otThreadErrorToString(err));
		#endif

		// clear json buffer
		json_buf[0] = "\0";

	};
}
