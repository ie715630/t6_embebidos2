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

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}bmi160_raw_data_t;

/* Inicialización de BMI160 */

void BMI160_I2C_initialization(void *args);

/* Lectura de los valores del acelerometro */
bmi160_raw_data_t BMI160_i2c_read_acc(void);

/* Lectura de los valores del girsocopio */
bmi160_raw_data_t BMI160_i2c_read_gyr(void);

#endif /* T6_EMBEBIDOS2_BMI160_H_ */
