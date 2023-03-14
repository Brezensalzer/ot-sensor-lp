/*
*  BME280 driver without device tree
*/
#ifndef BME280_H_
#define BME_280_H_

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <inttypes.h>

// define the BME280 sensor
#define BME280_I2C_ADDRESS 			0x76
#define BME280_REG_ID 				0xD0
#define BME280_REG_TEMP_COMP_START	0x88
#define BME280_REG_PRESS_COMP_START	0x8E
#define BME280_REG_PRESS_MSB		0xF7
#define BME280_REG_HUM_COMP_PART1	0xA1
#define BME280_REG_HUM_COMP_PART2	0xE1
#define BME280_CHIP_ID				0x60

extern struct bme280_data_type {
	// compensation parameters
	uint16_t	dig_t1;
	int16_t		dig_t2;
	int16_t		dig_t3;
	uint16_t	dig_p1;
	int16_t		dig_p2;
	int16_t		dig_p3;
	int16_t		dig_p4;
	int16_t		dig_p5;
	int16_t		dig_p6;
	int16_t		dig_p7;
	int16_t		dig_p8;
	int16_t		dig_p9;
	uint8_t		dig_h1;
	int16_t		dig_h2;
	uint8_t		dig_h3;
	int16_t		dig_h4;
	int16_t		dig_h5;
	int8_t		dig_h6;

	// uncompensated values
	uint32_t	ucomp_temp;
	uint32_t	ucomp_press;
	uint32_t	ucomp_hum;

	// compensated values
	uint32_t	comp_temp;
	uint32_t	comp_press;
	uint32_t	comp_hum;

	// carryover
	int32_t		t_fine;

	uint8_t		chip_id;
};

extern struct bme280_result_type {
	uint32_t	temp;
	uint32_t	press;
	uint32_t	hum;
};

static uint8_t read_buf_id[1];
static uint8_t read_buf_temp[6];
static uint8_t read_buf_press[18];
static uint8_t read_buf_hum[7];
static uint8_t write_reg_buffer[1];
static uint8_t read_meas_buf[9];

// define the i2c bus
#define I2C_NODE DT_NODELABEL(i2c0)
static const struct device *i2c_device = DEVICE_DT_GET(I2C_NODE);

//------------------------------------
// read chip id -> connection test
//------------------------------------
uint8_t bme280_read_chip_id(void);

//------------------------------------
// read final values
//------------------------------------
struct bme280_result_type bme280_read_values(void);

#endif