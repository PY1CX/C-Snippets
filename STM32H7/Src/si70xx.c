/*
 *  Si70xx Humidity and Temperature Driver
 *  Author: Felipe Navarro
 *  Date: 02/04/2018
 *  This driver uses features from FreeRTOS, but can be modified to be used bare-metal
 *  but if you're using a ARM, why Baremetal?
 */

#include "si70xx.h"

/*
 *  si70xx read Temperature and Humidity Task
 */
void t_si70xx_read_TEMP_HUMI(void * pvParameters){
	static TickType_t xLastWakeTime;
	static uint8_t _REG;
	static uint8_t si70xx_data[2];

	struct si70xx_{
		float humidity;
		float temperature;
	};

	static struct si70xx_ si70xx_reading;

	QueueSi70xx_data = xQueueCreate(2, sizeof(si70xx_reading));

	if(QueueSi70xx_data == NULL){/*Think about catch the error of queue not beeing created*/}

	for(;;){
		xLastWakeTime = xTaskGetTickCount();

		//Measure Humidity (WIHTOUT CLOCK STRETCHING - NO HOLD REG TYPE)
		if( xSemaphoreTake( xI2CSemaphore, (TickType_t) 2) == pdTRUE ){
			_REG = si70xx_MEAS_HUMI_NO_HOLD;
			HAL_I2C_Master_Transmit( &hi2c1, si70xx_ADDR, &_REG, 1, 100);
			//As it is a Blocking transmit, we can give semaphore back now
			xSemaphoreGive( xI2CSemaphore );
		}
		else{
			goto _sleep; //Jump to _sleep if we couldn't acquire the Semaphore
		}

		vTaskDelay(pdMS_TO_TICKS(500)); //500ms is a common time for getting the humidity reading!

		//Get the result of the Humidity Reading and also the temperature
		if( xSemaphoreTake( xI2CSemaphore, (TickType_t) 2) == pdTRUE ){
			//Receive 2 bytes from the Si70xx and store in the data array
			HAL_I2C_Master_Receive( &hi2c1, si70xx_ADDR, si70xx_data, 2, 100);

			/*
			 *  Convert humidity read to float
			 */

			//Get the temperature from last RH measurement
			_REG = si70xx_READ_TEMP_LAST_RH_MEAS;
			HAL_I2C_Master_Transmit( &hi2c1, si70xx_ADDR, &_REG, 1, 100);
			HAL_I2C_Master_Receive( &hi2c1, si70xx_ADDR, si70xx_data, 2, 100);

			/*
			 *  Convert temperature read to float
			 */

			xSemaphoreGive( xI2CSemaphore );
		}
		else{
			goto _sleep; //Jump to _sleep if we couldn't acquire the Semaphore
		}

		xQueueSendToBack(QueueSi70xx_data, &si70xx_reading, 0);

_sleep:
		// Using TaskDelayUntil to make it really 1s between sensor acquisition
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(si70xx_RATE_MEASUREMENT));
	}
}
