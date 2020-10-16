/*
 * BMI160.h
 *
 *  Created on: Oct 13, 2020
 *      Author: sergio_mndz
 */

#ifndef T6_EMBEBIDOS2_BMI160_H_
#define T6_EMBEBIDOS2_BMI160_H_

#include <stdint.h>
#include "freertos_i2c.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "event_groups.h"
#include "freertos_uart.h"


#define PIN_SCL_PTC10 (10U)
#define PIN_SDA_PTC11 (11)

// Default I2C 7-bit address of device if SDO = GND
#define BMI160_SLAVE_ADDR 0x68

#define ACCEL_NORMAL_MODE 0x11
#define GYROS_NORMAL_MODE 0x15

#define reg_gyro_x_lo 0x0C
#define reg_gyro_x_hi 0x0D
#define reg_gyro_y_lo 0x0E
#define reg_gyro_y_hi 0x0F
#define reg_gyro_z_lo 0x10
#define reg_gyro_z_hi 0x11
#define reg_acc_x_lo  0x12
#define reg_acc_x_hi  0x13
#define reg_acc_y_lo  0x14
#define reg_acc_y_hi  0x15
#define reg_acc_z_lo  0x16
#define reg_acc_z_hi  0x17

#define AHRS_IMU_SAMPLE_TIME 	20

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}bmi160_raw_data_t;

typedef struct
{
	SemaphoreHandle_t mutex_ADQUISITION_freertos;
	SemaphoreHandle_t mutex_SEND_UART_freertos;
	EventGroupHandle_t event_FreeRTOs;
} parameters_task_t;

/* Inicialización de BMI160 */
void bmi160_I2C_initialization(void *args);

/* Lectura de los valores del acelerometro */
bmi160_raw_data_t bmi160_i2c_read_acc(void);

/* Lectura de los valores del girsocopio */
bmi160_raw_data_t bmi160_i2c_read_gyr(void);

#endif /* T6_EMBEBIDOS2_BMI160_H_ */
