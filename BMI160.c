/*
 * BMI160.c
 *
 *  Created on: Oct 13, 2020
 *      Author: sergio_mndz
 */

#define BMI160_PRINTF_OFF 1

#include "BMI160.h"

void bmi160_I2C_initialization(void *args)
{
	parameters_task_t parameters_task = *((parameters_task_t*)args);

	init_i2c0_with_default_config();

	uint8_t dataToWrite = 0x00;
	uint8_t dataRead = 0x00;

	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, &dataRead, 1);

	dataToWrite = ACCEL_NORMAL_MODE;
	i2c_multiple_write(BMI160_SLAVE_ADDR, &dataToWrite, 1);

	dataToWrite = GYROS_NORMAL_MODE;
	i2c_multiple_write(BMI160_SLAVE_ADDR, &dataToWrite, 1);

	xTaskCreate(data_acquisition_task, 		"data_acquisition_task", 	  800, (void*)&parameters_task, configMAX_PRIORITIES, NULL);
	xTaskCreate(Ahrs_send_UART_angles_task, "Ahrs_send_UART_angles_task", 800, (void*)&parameters_task, configMAX_PRIORITIES, NULL);

	vTaskDelay(portMAX_DELAY);

}

bmi160_raw_data_t bmi160_i2c_read_acc(void)
{
	bmi160_raw_data_t data_acc;
	uint64_t raw_acc_data = 0;
	uint16_t x_data;
	uint16_t y_data;
	uint16_t z_data;

	i2c_multiple_read(BMI160_SLAVE_ADDR, reg_acc_x_lo, &raw_acc_data , 6);
	x_data = raw_acc_data & 0xffff;
	y_data = (raw_acc_data >> 16) & 0xffff;
	z_data = (raw_acc_data >> 32) & 0xffff;
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
	uint64_t raw_gyro_data = 0;
	uint16_t x_data;
	uint16_t y_data;
	uint16_t z_data;
	i2c_multiple_read(BMI160_SLAVE_ADDR, reg_gyro_x_lo, &raw_gyro_data , 6);
	x_data = raw_gyro_data & 0xffff;
	y_data = (raw_gyro_data >> 16) & 0xffff;
	z_data = (raw_gyro_data >> 32) & 0xffff;
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
