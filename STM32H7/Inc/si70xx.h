#include "main.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "queue.h"


enum _si70xx{
	si70xx_ADDR  				  = 0x40,
	si70xx_READ_TEMP_LAST_RH_MEAS = 0xE0,
	si70xx_MEAS_TEMP_NO_HOLD 	  = 0xF3,
	si70xx_MEAS_HUMI_NO_HOLD 	  = 0xF5,

	si70xx_RATE_MEASUREMENT		  = 1000
};

/*
#define si70xx_ADDR 0x40
#define si70xx_MEAS_HUM_NO_HOLD 0xF5
#define si70xx_MEAS_TEM_NO_HOLD 0xF3
#define si70xx_READ_TEMP_F_RH_MEASUREMENT 0xE0

#define RATE_MEASUREMENT 1000 //Rate in milliseconds
*/
QueueHandle_t QueueSi70xx_data;
SemaphoreHandle_t xI2CSemaphore; // V-E-R-I-F-I-C-A-R

void t_si70xx_read_TEMP_HUMI(void * pvParameters);
