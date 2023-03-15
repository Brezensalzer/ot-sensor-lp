/*
*  BME280 driver without device tree
*/
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>

#define CONFIG_BME280_MODE_FORCED
#define CONFIG_BME280_TEMP_OVER_1X
#define CONFIG_BME280_PRESS_OVER_1X
#define CONFIG_BME280_HUMIDITY_OVER_1X
#define CONFIG_BME280_STANDBY_500MS
#define CONFIG_BME280_FILTER_OFF
#include "bme280.h"

#define DEBUG
struct bme280_data_type bme280_data;

//------------------------------------
// wait until ready
//------------------------------------
int bme280_wait_until_ready(void)
{
	uint8_t status = 0;
	int ret;

	/* Wait for NVM to copy and measurement to be completed */
	do {
		k_sleep(K_MSEC(3));
		ret = i2c_reg_read_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_STATUS, &status);
		if (ret < 0) {
			return ret;
		}
	} while (status & (BME280_STATUS_MEASURING | BME280_STATUS_IM_UPDATE));

	return 0;
}

//------------------------------------
// read chip id 
//------------------------------------
uint8_t bme280_read_chip_id(void) {
    int rc = 0;
    uint8_t read_buf_id[1];

    rc = i2c_reg_read_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_ID, read_buf_id);
    if(rc == 0) {
        bme280_data.chip_id = read_buf_id[0];
        return bme280_data.chip_id;
    }
    else {
        return 0;
    }
}

//------------------------------------
// initialize sensor
//------------------------------------
uint8_t bme280_chip_init(void) {
    int rc = 0;
    // soft reset
    rc = i2c_reg_write_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_RESET, BME280_CMD_SOFT_RESET);
    if (rc != 0) {
        #ifdef DEBUG
            printk("BME280_CMD_SOFT_RESET err: %d", rc);
        #endif
        return rc;
    }
    rc = bme280_wait_until_ready();
    if (rc != 0) {
        #ifdef DEBUG
            printk("wait until ready err: %d", rc);
        #endif
        return rc;
    }
    // read compensation
    rc = bme280_read_compensation();
    if (rc != 0) {
        #ifdef DEBUG
            printk("read compensation err: %d", rc);
        #endif
        return rc;
    }
    // set oversampling
    rc = i2c_reg_write_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_CTRL_HUM, BME280_HUMIDITY_OVER);
    if (rc != 0) {
        #ifdef DEBUG
            printk("BME280_HUMIDITY_OVER err: %d", rc);
        #endif
        return rc;
    }
    rc = i2c_reg_write_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_VAL);
    if (rc != 0) {
        #ifdef DEBUG
            printk("BME280_CTRL_MEAS_VAL err: %d", rc);
        #endif
        return rc;
    }
    // more configurations
    rc = i2c_reg_write_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_CONFIG, BME280_CONFIG_VAL);
    if (rc != 0) {
        #ifdef DEBUG
            printk("BME280_CONFIG_VAL err: %d", rc);
        #endif
        return rc;
    }
    return 0;
}

//------------------------------------
// read compensation
//------------------------------------
int bme280_read_compensation(void) {
    int rc = 0;
    uint8_t read_buf_temp[6];
    uint8_t read_buf_press[18];
    uint8_t read_buf_hum[7];

    rc = i2c_burst_read(i2c_device, BME280_I2C_ADDRESS, BME280_REG_TEMP_COMP_START, read_buf_temp, sizeof(read_buf_temp));
    if (rc != 0) {
        return rc;
    }
    else {
        bme280_data.dig_t1 = (uint16_t)read_buf_temp[0];
        bme280_data.dig_t1 |= ((uint16_t)read_buf_temp[1])<<8;
        bme280_data.dig_t2 = (uint16_t)read_buf_temp[2];
        bme280_data.dig_t2 |= ((uint16_t)read_buf_temp[3])<<8;
        bme280_data.dig_t3 = (uint16_t)read_buf_temp[4];
        bme280_data.dig_t3 |= ((uint16_t)read_buf_temp[5])<<8;
    }
    
    rc = i2c_burst_read(i2c_device, BME280_I2C_ADDRESS, BME280_REG_PRESS_COMP_START, read_buf_press, sizeof(read_buf_press));
    if (rc != 0) {
        return rc;
    }
    else {
        bme280_data.dig_p1 = (uint16_t)read_buf_press[0];
        bme280_data.dig_p1 |= ((uint16_t)read_buf_press[1])<<8;
        bme280_data.dig_p2 = (uint16_t)read_buf_press[2];
        bme280_data.dig_p2 |= ((uint16_t)read_buf_press[3])<<8;
        bme280_data.dig_p3 = (uint16_t)read_buf_press[4];
        bme280_data.dig_p3 |= ((uint16_t)read_buf_press[5])<<8;
        bme280_data.dig_p4 = (uint16_t)read_buf_press[6];
        bme280_data.dig_p4 |= ((uint16_t)read_buf_press[7])<<8;
        bme280_data.dig_p5 = (uint16_t)read_buf_press[8];
        bme280_data.dig_p5 |= ((uint16_t)read_buf_press[9])<<8;
        bme280_data.dig_p6 = (uint16_t)read_buf_press[10];
        bme280_data.dig_p6 |= ((uint16_t)read_buf_press[11])<<8;
        bme280_data.dig_p7 = (uint16_t)read_buf_press[12];
        bme280_data.dig_p7 |= ((uint16_t)read_buf_press[13])<<8;
        bme280_data.dig_p8 = (uint16_t)read_buf_press[14];
        bme280_data.dig_p8 |= ((uint16_t)read_buf_press[15])<<8;
        bme280_data.dig_p9 = (uint16_t)read_buf_press[16];
        bme280_data.dig_p9 |= ((uint16_t)read_buf_press[17])<<8;
    }

    rc = i2c_reg_read_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_HUM_COMP_PART1, &bme280_data.dig_h1);
    if (rc != 0) {
        return rc;
    }

    rc = i2c_burst_read(i2c_device, BME280_I2C_ADDRESS, BME280_REG_HUM_COMP_PART2, read_buf_hum, sizeof(read_buf_hum));
    if (rc != 0) {
        return rc;
    }
    else {
        bme280_data.dig_h2 = (read_buf_hum[1] << 8) | read_buf_hum[0];
        bme280_data.dig_h3 = read_buf_hum[2];
        bme280_data.dig_h4 = (read_buf_hum[3] << 4) | (read_buf_hum[4] & 0x0F);
        bme280_data.dig_h5 = ((read_buf_hum[4] >> 4) & 0x0F) | (read_buf_hum[5] << 4);
        bme280_data.dig_h6 = read_buf_hum[6];
    }
    return rc;
}

/*
 * Compensation code taken from BME280 datasheet, Section 4.2.3
 * "Compensation formula".
 */
void bme280_compensate_temp(void)
{
	int32_t var1, var2;

	var1 = (((bme280_data.ucomp_temp >> 3) - ((int32_t)bme280_data.dig_t1 << 1)) *
		((int32_t)bme280_data.dig_t2)) >> 11;
	var2 = (((((bme280_data.ucomp_temp >> 4) - ((int32_t)bme280_data.dig_t1)) *
		  ((bme280_data.ucomp_temp >> 4) - ((int32_t)bme280_data.dig_t1))) >> 12) *
		((int32_t)bme280_data.dig_t3)) >> 14;
	bme280_data.t_fine = var1 + var2;
	bme280_data.comp_temp = (bme280_data.t_fine * 5 + 128) >> 8;
}

void bme280_compensate_press(void)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)bme280_data.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)bme280_data.dig_p6;
	var2 = var2 + ((var1 * (int64_t)bme280_data.dig_p5) << 17);
	var2 = var2 + (((int64_t)bme280_data.dig_p4) << 35);
	var1 = ((var1 * var1 * (int64_t)bme280_data.dig_p3) >> 8) +
		((var1 * (int64_t)bme280_data.dig_p2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme280_data.dig_p1) >> 33;

	/* Avoid exception caused by division by zero. */
	if (var1 == 0) {
		bme280_data.comp_press = 0U;
		return;
	}

	p = 1048576 - bme280_data.ucomp_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)bme280_data.dig_p9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)bme280_data.dig_p8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_data.dig_p7) << 4);

	bme280_data.comp_press = (uint32_t)p;
}

void bme280_compensate_humidity(void)
{
	int32_t h;

	h = (bme280_data.t_fine - ((int32_t)76800));
	h = ((((bme280_data.ucomp_hum << 14) - (((int32_t)bme280_data.dig_h4) << 20) -
		(((int32_t)bme280_data.dig_h5) * h)) + ((int32_t)16384)) >> 15) *
		(((((((h * ((int32_t)bme280_data.dig_h6)) >> 10) * (((h *
		((int32_t)bme280_data.dig_h3)) >> 11) + ((int32_t)32768))) >> 10) +
		((int32_t)2097152)) * ((int32_t)bme280_data.dig_h2) + 8192) >> 14);
	h = (h - (((((h >> 15) * (h >> 15)) >> 7) *
		((int32_t)bme280_data.dig_h1)) >> 4));
	h = (h > 419430400 ? 419430400 : h);

	bme280_data.comp_hum = (uint32_t)(h >> 12);
}

//------------------------------------
// read final values
//------------------------------------
struct bme280_result_type bme280_read_values(void) {
    int rc = 0;
    struct bme280_result_type result;
    uint8_t write_buf[1];
    uint8_t read_meas_buf[8];

    #ifdef CONFIG_BME280_MODE_FORCED
        rc = i2c_reg_write_byte(i2c_device, BME280_I2C_ADDRESS, BME280_REG_CTRL_MEAS, BME280_CTRL_MEAS_VAL);
        if (rc < 0) { 
            #ifdef DEBUG
                printk("forced mode failed! \n"); 
            #endif
        }
    #endif

    rc = bme280_wait_until_ready();
    if (rc != 0) { 
        #ifdef DEBUG
            printk("wait failed! \n"); 
        #endif
    }

    write_buf[0] = BME280_REG_PRESS_MSB;
    rc = i2c_burst_read(i2c_device, BME280_I2C_ADDRESS, BME280_REG_PRESS_MSB, read_meas_buf, 8);
    if (rc != 0) { 
        #ifdef DEBUG
            printk("read meas_buf failed! \n"); 
        #endif
    }

    bme280_data.ucomp_press = (read_meas_buf[0] << 12) | (read_meas_buf[1] << 4) | (read_meas_buf[2] >> 4);
    bme280_data.ucomp_temp = (read_meas_buf[3] << 12) | (read_meas_buf[4] << 4) | (read_meas_buf[5] >> 4);
    bme280_data.ucomp_hum = (read_meas_buf[6] << 8) | read_meas_buf[7];

    bme280_compensate_temp();
    bme280_compensate_press();
    bme280_compensate_humidity();

	// bme280_data.comp_temp has a resolution of 0.01 degC.
	// So 5123 equals 51.23 degC.
    result.temp  = bme280_data.comp_temp / 100.0;

    // bme280_data.comp_press has 24 integer bits and 8 fractional.
    // Output value of 24674867 represents
	// 24674867/256 = 96386.2 Pa = 963.862 hPa
    result.press = bme280_data.comp_press / 25600.0;

    // data->comp_humidity has 22 integer bits and 10 fractional.
    // Output value of 47445 represents 
    // 47445/1024 = 46.333 %RH
    result.hum   = bme280_data.comp_hum / 1024.0;

    return result;
}