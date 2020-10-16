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

	xTaskCreate(data_acquisition_task,		"data_acquisition_task",	800, (void*)&parameters_task, configMAX_PRIORITIES, NULL);
	xTaskCreate(uart_initialization_task, 	"uart_initialization_task",	800, (void*)&parameters_task, configMAX_PRIORITIES, NULL);

	vTaskDelay(portMAX_DELAY);
}

bmi160_raw_data_t bmi160_i2c_read_acc(void)
{
	bmi160_raw_data_t data_acc;
	uint8_t raw_acc_data[6];
	uint16_t x_data;
	uint16_t y_data;
	uint16_t z_data;

	i2c_multiple_read(BMI160_SLAVE_ADDR, reg_acc_x_lo, &raw_acc_data , 6);
	x_data = raw_acc_data[0] & raw_acc_data[1]<<8;
	y_data = raw_acc_data[2] & raw_acc_data[3]<<8;
	z_data = raw_acc_data[4] & raw_acc_data[5]<<8;
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
	i2c_multiple_read(BMI160_SLAVE_ADDR, reg_gyro_x_lo, &raw_gyro_data , 6);
	x_data = raw_gyro_data[0] & raw_gyro_data[1]<<8;
	y_data = raw_gyro_data[2] & raw_gyro_data[3]<<8;
	z_data = raw_gyro_data[4] & raw_gyro_data[5]<<8;
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

bmi160_raw_data_t g_int16data_acc;
bmi160_raw_data_t g_int16data_gyr;

void data_acquisition_task(void * args)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	parameters_task_t parameters_task = *((parameters_task_t*)args);

	while (1) {
		xSemaphoreTake(parameters_task.mutex_ADQUISITION_freertos, portMAX_DELAY);

		g_int16data_acc = bmi160_i2c_read_acc();	// Read IMU accelerometer
		g_int16data_gyr = bmi160_i2c_read_gyr();	// Read IMU gyroscope

#ifndef AHRS_PRINTF_OFF
		PRINTF("AHRS  -  Accel en x: %i \n", g_int16data_acc.x);
		PRINTF("AHRS  -  Accel en y: %i \n", g_int16data_acc.y);
		PRINTF("AHRS  -  Accel en z: %i \n", g_int16data_acc.z);

		PRINTF("AHRS  -  Gyros en x: %i \n", g_int16data_gyr.x);
		PRINTF("AHRS  -  Gyros en y: %i \n", g_int16data_gyr.y);
		PRINTF("AHRS  -  Gyros en z: %i \n", g_int16data_gyr.z);
#endif

		xSemaphoreGive(parameters_task.mutex_ADQUISITION_freertos);
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(AHRS_IMU_SAMPLE_TIME));
	}
}

void uart_initialization_task(void *args)
{
	parameters_task_t parameters_task = *((parameters_task_t*)args);

	rtos_uart_config_t config;		// Configuration UART0
	config.baudrate = 115200;
	config.rx_pin = 16;
	config.tx_pin = 17;
	config.pin_mux = kPORT_MuxAlt3;
	config.uart_number = rtos_uart0;
	config.port = rtos_uart_portB;
	rtos_uart_init(config);

	while (1)
	{
		xSemaphoreTake(parameters_task.mutex_SEND_UART_freertos, portMAX_DELAY);

		xSemaphoreGive(parameters_task.mutex_SEND_UART_freertos);
	}

}
