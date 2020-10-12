/*
 * freertos_i2c.h
 *
 *  Created on: 11 oct. 2020
 *      Author: kcdia
 */


#ifndef FREERTOS_I2C_H_
#define FREERTOS_I2C_H_

#include "stdint.h"
#include "fsl_i2c.h"

typedef enum {
    freertos_i2c_success,
    freertos_i2c_fail
} freertos_i2c_flag_t;

freertos_i2c_flag_t init_i2c0_with_default_config(void);

void fsl_i2c_callback(I2C_Type *base, i2c_master_handle_t *handle,
                             status_t status, void *userData);

freertos_i2c_flag_t i2c_multiple_write(uint8_t slave_addr, uint8_t * write_data,
		                               uint8_t write_size);

freertos_i2c_flag_t i2c_multiple_read(uint8_t slave_addr, uint8_t wr_command,
		                              uint8_t * read_data, uint8_t read_size);

#endif /* FREERTOS_I2C_H_ */
