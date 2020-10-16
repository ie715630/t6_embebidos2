/**
 * @file    BMI160.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "semphr.h"
#include "freertos_i2c.h"
#include "BMI160.h"

//static void init_i2c_module(void * args);
//static void transfer_data(void * args);

SemaphoreHandle_t i2c_initilized;


void init_BMI160(void *parameters)
{

	freertos_bmi_flag_t init_flag = freertos_bmi160_success;
	init_flag = bmi160_i2c_initialization();

	xSemaphoreGive(i2c_initilized);
	vTaskSuspend(NULL);
}

void read_data(void *parameters)
{
	bmi160_raw_data_t acc_data;
	bmi160_raw_data_t gyro_data;

	xSemaphoreTake(i2c_initilized, portMAX_DELAY);

	while(1)
	{
		acc_data = bmi160_i2c_read_acc();
		gyro_data = bmi160_i2c_read_gyr();
		PRINTF("Data from acc:  x = %d  y = %d  z = %d \n\r",
			    acc_data.x, acc_data.y, acc_data.z );
		PRINTF("Data from gyro:  x = %d  y = %d  z = %d \n\r",
				gyro_data.x, gyro_data.y, gyro_data.z );
		vTaskDelay(pdMS_TO_TICKS(300));
	}
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    i2c_initilized = xSemaphoreCreateBinary();

    xTaskCreate(init_BMI160, "init_BMI160", 110, NULL, 1, NULL);
    xTaskCreate(read_data, "read_data", 110, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1) {
        __asm volatile ("nop");
    }
    return 0 ;
}
