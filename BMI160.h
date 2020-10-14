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
