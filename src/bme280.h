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
#define BME280_I2C_ADDRESS              0x76
#define BME280_REG_TEMP_COMP_START	    0x88
#define BME280_REG_PRESS_COMP_START	    0x8E

#define BME280_REG_PRESS_MSB            0xF7
#define BME280_REG_COMP_START           0x88
#define BME280_REG_HUM_COMP_PART1       0xA1
#define BME280_REG_HUM_COMP_PART2       0xE1
#define BME280_REG_ID                   0xD0
#define BME280_REG_CONFIG               0xF5
#define BME280_REG_CTRL_MEAS            0xF4
#define BME280_REG_CTRL_HUM             0xF2
#define BME280_REG_STATUS               0xF3
#define BME280_REG_RESET                0xE0

#define BMP280_CHIP_ID_SAMPLE_1         0x56
#define BMP280_CHIP_ID_SAMPLE_2         0x57
#define BMP280_CHIP_ID_MP               0x58
#define BME280_CHIP_ID                  0x60
#define BME280_MODE_SLEEP               0x00
#define BME280_MODE_FORCED              0x01
#define BME280_MODE_NORMAL              0x03
#define BME280_SPI_3W_DISABLE           0x00
#define BME280_CMD_SOFT_RESET           0xB6
#define BME280_STATUS_MEASURING         0x08
#define BME280_STATUS_IM_UPDATE         0x01

#if defined CONFIG_BME280_MODE_NORMAL
#define BME280_MODE BME280_MODE_NORMAL
#elif defined CONFIG_BME280_MODE_FORCED
#define BME280_MODE BME280_MODE_FORCED
#endif

#if defined CONFIG_BME280_TEMP_OVER_1X
#define BME280_TEMP_OVER                (1 << 5)
#elif defined CONFIG_BME280_TEMP_OVER_2X
#define BME280_TEMP_OVER                (2 << 5)
#elif defined CONFIG_BME280_TEMP_OVER_4X
#define BME280_TEMP_OVER                (3 << 5)
#elif defined CONFIG_BME280_TEMP_OVER_8X
#define BME280_TEMP_OVER                (4 << 5)
#elif defined CONFIG_BME280_TEMP_OVER_16X
#define BME280_TEMP_OVER                (5 << 5)
#endif

#if defined CONFIG_BME280_PRESS_OVER_1X
#define BME280_PRESS_OVER               (1 << 2)
#elif defined CONFIG_BME280_PRESS_OVER_2X
#define BME280_PRESS_OVER               (2 << 2)
#elif defined CONFIG_BME280_PRESS_OVER_4X
#define BME280_PRESS_OVER               (3 << 2)
#elif defined CONFIG_BME280_PRESS_OVER_8X
#define BME280_PRESS_OVER               (4 << 2)
#elif defined CONFIG_BME280_PRESS_OVER_16X
#define BME280_PRESS_OVER               (5 << 2)
#endif

#if defined CONFIG_BME280_HUMIDITY_OVER_1X
#define BME280_HUMIDITY_OVER            1
#elif defined CONFIG_BME280_HUMIDITY_OVER_2X
#define BME280_HUMIDITY_OVER            2
#elif defined CONFIG_BME280_HUMIDITY_OVER_4X
#define BME280_HUMIDITY_OVER            3
#elif defined CONFIG_BME280_HUMIDITY_OVER_8X
#define BME280_HUMIDITY_OVER            4
#elif defined CONFIG_BME280_HUMIDITY_OVER_16X
#define BME280_HUMIDITY_OVER            5
#endif

#if defined CONFIG_BME280_STANDBY_05MS
#define BME280_STANDBY                  0
#elif defined CONFIG_BME280_STANDBY_62MS
#define BME280_STANDBY                  (1 << 5)
#elif defined CONFIG_BME280_STANDBY_125MS
#define BME280_STANDBY                  (2 << 5)
#elif defined CONFIG_BME280_STANDBY_250MS
#define BME280_STANDBY                  (3 << 5)
#elif defined CONFIG_BME280_STANDBY_500MS
#define BME280_STANDBY                  (4 << 5)
#elif defined CONFIG_BME280_STANDBY_1000MS
#define BME280_STANDBY                  (5 << 5)
#elif defined CONFIG_BME280_STANDBY_2000MS
#define BME280_STANDBY                  (6 << 5)
#elif defined CONFIG_BME280_STANDBY_4000MS
#define BME280_STANDBY                  (7 << 5)
#endif

#if defined CONFIG_BME280_FILTER_OFF
#define BME280_FILTER                   0
#elif defined CONFIG_BME280_FILTER_2
#define BME280_FILTER                   (1 << 2)
#elif defined CONFIG_BME280_FILTER_4
#define BME280_FILTER                   (2 << 2)
#elif defined CONFIG_BME280_FILTER_8
#define BME280_FILTER                   (3 << 2)
#elif defined CONFIG_BME280_FILTER_16
#define BME280_FILTER                   (4 << 2)
#endif

#define BME280_CTRL_MEAS_VAL            (BME280_PRESS_OVER | \
					 BME280_TEMP_OVER |  \
					 BME280_MODE)
#define BME280_CONFIG_VAL               (BME280_STANDBY | \
					 BME280_FILTER |  \
					 BME280_SPI_3W_DISABLE)

#define BME280_CTRL_MEAS_OFF_VAL	(BME280_PRESS_OVER | \
					 BME280_TEMP_OVER |  \
					 BME280_MODE_SLEEP)

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
	int32_t		comp_temp;
	uint32_t	comp_press;
	uint32_t	comp_hum;

	// carryover
	int32_t		t_fine;

	uint8_t		chip_id;
};

extern struct bme280_result_type {
	float	temp;
	float	press;
	float	hum;
};

// define the i2c bus
#define I2C_NODE DT_ALIAS(i2c)
static const struct device *i2c_device = DEVICE_DT_GET(I2C_NODE);

//------------------------------------
// initialize sensor
//------------------------------------
uint8_t bme280_chip_init(void);

//------------------------------------
// read chip id
//------------------------------------
uint8_t bme280_read_chip_id(void);

//------------------------------------
// read final values
//------------------------------------
struct bme280_result_type bme280_read_values(void);

#endif