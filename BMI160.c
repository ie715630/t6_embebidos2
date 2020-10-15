/*
 * BMI160.c
 *
 *  Created on: Oct 13, 2020
 *      Author: sergio_mndz
 */

#define BMI160_PRINTF_OFF 1

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

	uint8_t data_x_lo = 0;
	uint8_t data_x_hi = 0;
	uint8_t data_y_lo = 0;
	uint8_t data_y_hi = 0;
	uint8_t data_z_lo = 0;
	uint8_t data_z_hi = 0;

	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_acc_x_lo , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_acc_x_hi , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_acc_y_lo , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_acc_y_hi , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_acc_z_lo , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_acc_z_hi , 1);

	data_acc.x = (data_x_hi << 8) + data_x_lo;
	data_acc.y = (data_y_hi << 8) + data_y_lo;
	data_acc.z = (data_z_hi << 8) + data_z_lo;

#ifndef BMI160_PRINTF_OFF
	PRINTF("BMI160 - Accel en x: %d \n", data_acc.x);
	PRINTF("BMI160 - Accel en y: %d \n", data_acc.y);
	PRINTF("BMI160 - Accel en z: %d \n", data_acc.z);
#endif

	return data_acc;
}

bmi160_raw_data_t bmi160_i2c_read_gyr(void) {
	bmi160_raw_data_t data_gyro;

	uint8_t data_x_lo = 0;
	uint8_t data_x_hi = 0;
	uint8_t data_y_lo = 0;
	uint8_t data_y_hi = 0;
	uint8_t data_z_lo = 0;
	uint8_t data_z_hi = 0;

	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_gyro_x_lo , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_gyro_x_hi , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_gyro_y_lo , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_gyro_y_hi , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_gyro_z_lo , 1);
	i2c_multiple_read(BMI160_SLAVE_ADDR, 0, reg_gyro_z_hi , 1);

	data_gyro.x = (data_x_hi << 8) + data_x_lo;
	data_gyro.y = (data_y_hi << 8) + data_y_lo;
	data_gyro.z = (data_z_hi << 8) + data_z_lo;

#ifndef BMI160_PRINTF_OFF
	PRINTF("BMI160 - Gyro en x: %d \n", data_acc.x);
	PRINTF("BMI160 - Gyro en y: %d \n", data_acc.y);
	PRINTF("BMI160 - Gyro en z: %d \n", data_acc.z);
#endif

	return data_gyro;
}
