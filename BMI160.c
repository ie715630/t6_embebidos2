/*
 * BMI160.c
 *
 *  Created on: Oct 13, 2020
 *      Author: sergio_mndz
 */

#define BMI160_PRINTF_OFF 1

#include "BMI160.h"

freertos_bmi_flag_t bmi160_i2c_initialization(void)
{
	freertos_bmi_flag_t bmi_flag = freertos_bmi160_fail;
	freertos_i2c_flag_t i2c_flag;
	i2c_flag = init_i2c0_with_default_config();

	vTaskDelay(pdMS_TO_TICKS(10));

	if (freertos_i2c_success == i2c_flag)
	{
		bmi_flag = freertos_bmi160_success;
	}


	uint8_t dataToWrite[2] = {0x7E, 0x00};

	dataToWrite[1] = ACCEL_NORMAL_MODE;
	i2c_multiple_write(BMI160_SLAVE_ADDR, dataToWrite, 2);
	vTaskDelay(pdMS_TO_TICKS(100));

	dataToWrite[1] = GYROS_NORMAL_MODE;
	i2c_multiple_write(BMI160_SLAVE_ADDR, dataToWrite, 2);
	vTaskDelay(pdMS_TO_TICKS(100));

	return bmi_flag;
}

bmi160_raw_data_t bmi160_i2c_read_acc(void)
{
	bmi160_raw_data_t data_acc;
	uint8_t raw_acc_data[6];
	uint16_t x_data;
	uint16_t y_data;
	uint16_t z_data;

	i2c_multiple_read(BMI160_SLAVE_ADDR, reg_acc_x_lo, raw_acc_data , 6);
	x_data = raw_acc_data[0] | raw_acc_data[1]<<8;
	y_data = raw_acc_data[2] | raw_acc_data[3]<<8;
	z_data = raw_acc_data[4] | raw_acc_data[5]<<8;
	data_acc.x = x_data;
	data_acc.y = y_data;
	data_acc.z = z_data;

#ifndef BMI160_PRINTF_OFF
	PRINTF("BMI160 - Accel en x: %d \n", data_acc.x);
	PRINTF("BMI160 - Accel en y: %d \n", data_acc.y);
	PRINTF("BMI160 - Accel en z: %d \n", data_acc.z);
#endif

	return data_acc;
}

bmi160_raw_data_t bmi160_i2c_read_gyr(void) {
	bmi160_raw_data_t data_gyro;
	uint8_t raw_gyro_data[6];
	uint16_t x_data;
	uint16_t y_data;
	uint16_t z_data;
	i2c_multiple_read(BMI160_SLAVE_ADDR, reg_gyro_x_lo, raw_gyro_data , 6);
	x_data = raw_gyro_data[0] | raw_gyro_data[1]<<8;
	y_data = raw_gyro_data[2] | raw_gyro_data[3]<<8;
	z_data = raw_gyro_data[4] | raw_gyro_data[5]<<8;
	data_gyro.x = x_data;
	data_gyro.y = y_data;
	data_gyro.z = z_data;

#ifndef BMI160_PRINTF_OFF
	PRINTF("BMI160 - Gyro en x: %d \n", data_acc.x);
	PRINTF("BMI160 - Gyro en y: %d \n", data_acc.y);
	PRINTF("BMI160 - Gyro en z: %d \n", data_acc.z);
#endif

	return data_gyro;
}

