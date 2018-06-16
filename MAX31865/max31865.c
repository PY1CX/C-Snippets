/*
 *	max31865.c
 *
 *	MAX31865 Driver
 *	Author: Felipe Navarro
 *
 *	Fault Common Numbers:
 *	-3 Semaphore
 *	-2 SPI
 *	-1 Wrong Configuration
 *
 *	 1 Represent Success
 *
 */

#include "max31865.h"

// Global vars for Debug purpose
int8_t result;
uint8_t regx;

uint8_t res1,res2,res3;
uint16_t res_rtd;

/*
 * 	Function Blocks
 */

/*
 * Function to start a new acquisition of the temperature
 *
 * Returns:
 * Error level:
 * -3 - If couldn't get the Semaphore
 * -2 - If couldn't use SPI
 * -1 - If conversion mode is automatic
 *
 *  1 - Success
 */

int8_t begin_one_shot_cnv(SemaphoreHandle_t * Mutex_SPI){

	//Take Semaphore
	if( xSemaphoreTake( Mutex_SPI, (TickType_t) 5) == pdTRUE ){
		uint8_t __areg = 0;
		uint8_t __reg_addr = CONFIG_R;
		HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_RESET); // Lower CS before writing to SPI
		//Try to read the config register
		if(HAL_SPI_Transmit(&hspi1, &__reg_addr, 1, 10) != HAL_OK ){
			goto spi_one_shot_error;
		}

		//Rx Config Register and set ONE_SHOT bit and VBIAS bit on config register
		if(HAL_SPI_Receive(&hspi1, &__areg, 1, 10) != HAL_OK){
			goto spi_one_shot_error;
		}
		//Need to rise CS before sending new command
		HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);

		//Check before sending ONE_SHOT command if AUTOMATIC isn't enabled
		if( (__areg & (1 << CONVERSION_MODE)) == 0 ){
			__reg_addr = CONFIG_W;
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_RESET);
			if(HAL_SPI_Transmit(&hspi1, &__reg_addr, 1, 10) != HAL_OK ){
				goto spi_one_shot_error;
			}
			__areg |= ( 1UL << ONE_SHOT ); //Set ONE_SHOT conversion

		}
		else
		{
			// If Conversion is Automatic, no need to do Single Conversion
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
			xSemaphoreGive(Mutex_SPI);
			return -1; //Exit point wrong conversion mode
		}

		//Transmit new values to the config register
		if(HAL_SPI_Transmit(&hspi1, &__areg, 1, 10) != HAL_OK){
			goto spi_one_shot_error;
		}
		//Get CS down as fast as possible so the conversion starts.
		HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
		xSemaphoreGive(Mutex_SPI);
		return 1;
	}
	else
	{
		return -3;
	}

	//Reset CS pin, give semaphore and free _reg if a failure occur, also send
	//a 0 to signal that we couldn't start a conversion
spi_one_shot_error:
	HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_RESET);
	xSemaphoreGive(Mutex_SPI);
	return -2;
}



/*
 * Function to config the MAX31865 with the following parameters:
 * 3-Wire connection, One Shot conversion and 60hz Filter
 *
 * Returns:
 * Error level:
 * -3 - If couldn't get the Semaphore
 * -2 - If couldn't use SPI
 *
 *  1 - Success
 */

int8_t MAX31865_config_hard_coded(SemaphoreHandle_t * Mutex_SPI){

	if( xSemaphoreTake( Mutex_SPI, (TickType_t) 5) == pdTRUE ){
		uint8_t __areg = CONFIG_W;
		HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_RESET); // Rise CS before writing to SPI

		if(HAL_SPI_Transmit(&hspi1, &__areg, 1, 10) != HAL_OK){
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
			xSemaphoreGive(Mutex_SPI);
			return -2;
		}
		__areg = 0;
		__areg |= ( 1UL << WIRE_NUMBER ); // 3-Wire Config as schematic
		__areg |= ( 1UL << VBIAS ); //Vbias always enabled
		if(HAL_SPI_Transmit(&hspi1, &__areg, 1, 10) != HAL_OK){
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
			xSemaphoreGive(Mutex_SPI);
			return -2;
		}
	}
	else
	{
		return -3;
	}
	HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
	xSemaphoreGive(Mutex_SPI);
	return 1;
}

/*
 *  TASK Function Read_Temp
 *  Function responsible for implementing the config (hard coded in the code)
 *  and also getting one shot conversions.
 *
 */
void t_read_temp(void * pvParameters){

	SemaphoreHandle_t Mutex_SPI = (SemaphoreHandle_t) pvParameters;

	//Config MAX31865
	result = MAX31865_config_hard_coded(Mutex_SPI);

	/*
	 * Guard Clause - Get conversion result if for some reason there is some results
	 * signaled with the DRDY low.
	 */
	if(HAL_GPIO_ReadPin(DRDY_MAX31865_GPIO_Port, DRDY_MAX31865_Pin) == GPIO_PIN_RESET){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(RTD_RX_Task, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}


	for(;;){
		result = begin_one_shot_cnv(Mutex_SPI);
		vTaskDelay(pdMS_TO_TICKS(300));
	}
}

/*
 *  Task to Receive the Signal that we have Data Ready (signaled by the DRDY low)
 *  and get the results.
 *
 */
void t_rx_temp(void * pvParameters){
	SemaphoreHandle_t Mutex_SPI = (SemaphoreHandle_t) pvParameters;

	for(;;){
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY  ); //Wait until we get a Task Notification

		//Verify if the DRDY is really LOW before getting the Semaphore
		if(HAL_GPIO_ReadPin(DRDY_MAX31865_GPIO_Port, DRDY_MAX31865_Pin) == GPIO_PIN_RESET){

			//If we really have a Conversion to acquire, try getting the semaphore
			if( xSemaphoreTake( Mutex_SPI, (TickType_t) 5) == pdTRUE ){

			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_RESET);
			regx = RTD_MSB_R;
			HAL_SPI_Transmit(&hspi1, &regx, 1, 10);

			HAL_SPI_Receive(&hspi1, &res1, 1, 10);
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
			regx = RTD_LSB_R;
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, &regx, 1, 10);

			HAL_SPI_Receive(&hspi1, &res2, 1, 10);
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
			regx = FAULT_STATUS_R;
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, &regx, 1, 10);

			HAL_SPI_Receive(&hspi1, &res3, 1, 10);
			HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
			xSemaphoreGive(Mutex_SPI);

			res_rtd = res1 << 8;
			res_rtd |= res2;
			res_rtd >>= 1;
			res_rtd /= 32;
			res_rtd -= 256;
			}
		}



	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_5)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR(RTD_RX_Task, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}


}
