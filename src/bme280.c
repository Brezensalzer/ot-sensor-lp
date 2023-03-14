/*
*  BME280 driver without device tree
*/
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include "bme280.h"

//------------------------------------
// read chip id -> connection test
//------------------------------------
int bme280_read_chip_id(void) {
    int rc = 0;
    write_reg_buffer[0] = BME280_CHIP_ID;
    rc = i2c_write_read(i2c_device, BME280_I2C_ADDRESS, write_reg_buffer, 1, read_buf_id, 1);
    if(rc == 0) {
        bme_data.chip_id = read_buf_id[0];
    }
    return rc;
}