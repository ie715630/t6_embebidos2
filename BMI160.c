/*
 * BMI160.c
 *
 *  Created on: Oct 13, 2020
 *      Author: sergio_mndz
 */

#define BMI160_PRINTF_OFF 1

bmi160_raw_data_t bmi160_i2c_read_acc(void) {
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
