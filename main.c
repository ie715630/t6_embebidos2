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

SemaphoreHandle_t semaphoreInit;

void init_BMI160(void *parameters)
{
	freertos_bmi_flag_t init_flag = freertos_bmi160_success;
	init_flag = bmi160_i2c_initialization();
	if(freertos_bmi160_success == init_flag)
	{
		PRINTF("BMI160 Inicializado\n");
		xSemaphoreGive(semaphoreInit);
	}
	else
	{
		PRINTF("ERROR AL INICIALIZAR\n");
	}
	vTaskSuspend(NULL);
}

void read_data(void *parameters)
{
	bmi160_raw_data_t acc_data;
	bmi160_raw_data_t gyro_data;
	xSemaphoreTake(semaphoreInit, portMAX_DELAY);
	while(1)
	{
		acc_data = bmi160_i2c_read_acc();
		gyro_data = bmi160_i2c_read_gyr();
		PRINTF("Data from acc:  x = %d  y = %d  z = %d \n", acc_data.x, acc_data.y, acc_data.z );
		PRINTF("Data from gyro:  x = %d  y = %d  z = %d \n", gyro_data.x, gyro_data.y, gyro_data.z );
	}
}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    xTaskCreate(init_BMI160, "init_BMI160", 110, NULL, 1, NULL);
    xTaskCreate(read_data, "read_data", 110, NULL, 1, NULL);

    vTaskStartScheduler();

    while(1) {
        __asm volatile ("nop");
    }
    return 0 ;
}

/*
static void init_i2c_module(void * args)
{
	init_i2c0_with_default_config();
	// 10mS for BMI startup
	vTaskDelay(pdMS_TO_TICKS(10));

	for(;;)
	{
		xSemaphoreGive(i2c_initilized);
		vTaskDelay(portMAX_DELAY);
	}
}

static void transfer_data(void * args)
{
	uint8_t data[10] = {0x7E, 0x15}; // 0x11 Start accel, 0x15 for gyro
	uint8_t slave_addr = 0x68;
	uint8_t wr_size = 2;
	uint8_t rx_data[10];
	uint8_t rd_size = 6;
	uint8_t wr_command = 0x0C;

	xSemaphoreTake(i2c_initilized, portMAX_DELAY);

	i2c_multiple_write(slave_addr, data, wr_size);
	vTaskDelay(300);

	for(;;)
	{
		i2c_multiple_read(slave_addr, wr_command, rx_data, rd_size);
		PRINTF("%X\t%X\t%X\t%X\t%X\t%X\n\r", rx_data[0], rx_data[1], rx_data[2], rx_data[3], rx_data[4], rx_data[5]);
		vTaskDelay(pdMS_TO_TICKS(300));
	}
}
*/
