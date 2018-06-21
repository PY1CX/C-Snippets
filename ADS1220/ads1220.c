/*
 *	ads1220.c
 *
 *	ADS1220 Driver
 *	Author: Felipe Navarro
 *
 */
#include "ads1220.h"

/*
 *  Function to Start a new conversion on Single Shot Mode of ADS1220
 */
int8_t ADS1220_start_conversion(SemaphoreHandle_t * Mutex_SPI){

	//Take SPI Semaphore before using the SPI
	if( xSemaphoreTake( Mutex_SPI, (TickType_t) 5) == pdTRUE ){
			HAL_GPIO_WritePin(CS_ADS1220_GPIO_Port , CS_ADS1220_Pin, GPIO_PIN_RESET); // Lower CS before writing to SPI
			uint8_t _reg_data = ADS1220_START_SYNC;
			//Try to TX the START/SYNC command
			if(HAL_SPI_Transmit(&hspi1, &_reg_data, 1, 10) != HAL_OK ){

				//If we doesn't get a HAL_OK - Free Mutex, SET CS_ADS1220 and return SPI_FAULT
				HAL_GPIO_WritePin(CS_MAX31865_GPIO_Port, CS_MAX31865_Pin, GPIO_PIN_SET);
				xSemaphoreGive(Mutex_SPI);
				return ADS1220_SPI_FAULT;
			}
			//In case of Success
			HAL_GPIO_WritePin(CS_ADS1220_GPIO_Port, CS_ADS1220_Pin, GPIO_PIN_SET);

			xSemaphoreGive(Mutex_SPI);
			return ADS1220_SUCCESS;
	}
	else
	{
		//If couldn't acquire the Semaphore, signal Semaphore Fault
		return ADS1220_SEMAPHORE_FAULT;
	}
}

/*
 * Function to WRITE in the ADS1220 config registers
 */
int8_t WRITE_REG_ADS1220(uint8_t * _reg_addr, uint8_t * _reg_data){
	HAL_GPIO_WritePin(CS_ADS1220_GPIO_Port , CS_ADS1220_Pin, GPIO_PIN_RESET); // Take CS LOW before writing to SPI

	if(HAL_SPI_Transmit(&hspi1, _reg_addr, 1, 10) != HAL_OK ){
		return ADS1220_SPI_FAULT;
	}

	if(HAL_SPI_Transmit(&hspi1, _reg_data, 1, 10) != HAL_OK ){
		return ADS1220_SPI_FAULT;
	}

	HAL_GPIO_WritePin(CS_ADS1220_GPIO_Port , CS_ADS1220_Pin, GPIO_PIN_SET);  // Rise CS after write
	return ADS1220_SUCCESS;
}

/*
 * Function to config the ADS1220 with hard-coded parameters
 */
int8_t ADS1220_config(SemaphoreHandle_t * Mutex_SPI){

	//Take SPI Semaphore before using the SPI
	if( xSemaphoreTake( Mutex_SPI, (TickType_t) 5) == pdTRUE ){
		uint8_t _reg_addr = 0;
		uint8_t _reg_data = 0;

		/*
		 * For this project just need to config register 2
		 * other values are the same as default
		 */

		//Config Register 2
		_reg_addr = ADS1220_REG2_W;

		_reg_data = ( _reg_data &  ADS1220_VREF_BITMASK ) | ADS1220_VREF_REFP0_REFN0;
		_reg_data = ( _reg_data &  ADS1220_FIR_FILTER_BITMASK ) | ADS1220_FIR_FILTER_60;

		if(WRITE_REG_ADS1220( &_reg_addr, &_reg_data) == ADS1220_SUCCESS){
			xSemaphoreGive(Mutex_SPI);
			return ADS1220_SUCCESS;
		}
		else
		{
			xSemaphoreGive(Mutex_SPI);
			return ADS1220_SPI_FAULT;
		}
	}
	else
	{
		return ADS1220_SEMAPHORE_FAULT;
	}
}

/*  TASK FUNCTION
 *
 *  t_one_shot_ADS1220
 *
 *  This task handles the initial config of ADS1220 and dispatches the one-shot
 *  conversions
 */
void t_one_shot_ADS1220(void * pvParameters){

	SemaphoreHandle_t Mutex_SPI = (SemaphoreHandle_t) pvParameters;

	//Config ADS1220
	ADS1220_config(Mutex_SPI);

	for(;;){

		ADS1220_start_conversion(Mutex_SPI);
		vTaskDelay(pdMS_TO_TICKS(PERIOD_PH_MEASUREMENT));
	}
}



/* 	TASK FUNCTION
 *
 *  t_RX_ADS1220
 *
 *  Task to RX the ADS1220 data
 *  This task receives a Task Notification to get out of SUSPENDED mode
 *  and do the RX of the ADS1220 data.
 *
 *  You need to pass the SPI Semaphore Handle thru the pvParameters when
 *  you create the task
 *
 */
void t_RX_ADS1220(void * pvParameters){

	SemaphoreHandle_t Mutex_SPI = (SemaphoreHandle_t) pvParameters;

	for(;;){
		//Wait forever until we receive a signal that the ADS1220 has some data to be received
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY );


	}
}
