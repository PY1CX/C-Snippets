/*
 *	max31865.c
 *
 *	MAX31865 Driver
 *	Author: Felipe Navarro
 *
 */

#include "max31865.h"

/*
 * 	Function Blocks
 *
 */


/*
 * Function to start a new acquisition of the temperature
 *
 * Returns:
 * Error level:
 * -3 - If couldn't get the Semaphore
 * -2 - If couldn't use SPI
 * -1 - If conversion mode isn't enabled
 *
 *  1 - Success
 */
int8_t begin_one_shot_cnv(void){

	//Take Semaphore
	if( xSemaphoreTake( xSPI1_Semaphore, (TickType_t) 5) == pdTrue ){
		uint8_t _reg = pvPortMalloc(sizeof(uint8_t));

		HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_SET); // Rise CS before writing to SPI
		//Try to read the config register
		if(HAL_SPI_Transmit(&hspi1, CONFIG_R, 1, 10) != HAL_OK ){
			goto spi_one_shot_error;
		}

		//Rx Config Register and set ONE_SHOT bit and VBIAS bit on config register
		if(HAL_SPI_Receive(&hspi1, _reg, 1, 10) != HAL_OK){
			goto spi_one_shot_error;
		}

		if( (_reg & (1 << CONVERSION_MODE)) == 0 ){
			_reg |= ( 1UL << ONE_SHOT ); //Set ONE_SHOT conversion
			_reg |= ( 1UL << VBIAS    ); //Enable VBIAS
		}
		else
		{
			HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET);
			xSemaphoreGive(xSPI1_Semaphore);
			pvPortFree(_reg);
			return -1; //Exit point wrong conversion mode
		}

		//Transmit new values to the config register
		if(HAL_SPI_Transmit(&hspi1, _reg, 1, 10) != HAL_OK){
			goto spi_one_shot_error;
		}

		HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET); //Get CS down as fast as possible so the conversion starts.
		xSemaphoreGive(xSPI1_Semaphore);
		pvPortFree(_reg);
		return 1;
	}
	else
	{
		return -3;
	}

	//Reset CS pin, give semaphore and free _reg if a failure occur, also send
	//a 0 to signal that we couldn't start a conversion
spi_one_shot_error:
	HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET);
	xSemaphoreGive(xSPI1_Semaphore);
	pvPortFree(_reg);
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
int8_t MAX31865_config_hard_coded(void){
	if( xSemaphoreTake( xSPI1_Semaphore, (TickType_t) 5) == pdTrue ){

		HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_SET); // Rise CS before writing to SPI
		//0x12 is: 00010010
		if(HAL_SPI_Transmit(&hspi1, 0x12, 1, 10) != HAL_OK){
			HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET);
			xSemaphoreGive(xSPI1_Semaphore);
			return -2;
		}
	}
	else
	{
		return -3;
	}
	HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET);
	xSemaphoreGive(xSPI1_Semaphore);
	return 1;
}

//TODO:
int8 MAX31865_clear_fault(void){

}

/*
 * Function to write to MAX31865 Registers
 * Inputs:
 * _reg_add - Register Address to write to
 * value    - Value to write to the register address
 *
 * Returns:
 * Error level:
 * -3 - If couldn't get the Semaphore
 * -2 - If couldn't use SPI
 *
 *  1 - Success
 */
int8_t MAX31865_write_reg(uint8_t _reg_add, uint8_t value){

	//Take Semaphore
	if( xSemaphoreTake( xSPI1_Semaphore, (TickType_t) 5) == pdTrue ){

		HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_SET); // Rise CS before writing to SPI
		//Try to read the register
		if(HAL_SPI_Transmit(&hspi1, _reg_add, 1, 10) != HAL_OK ){
			goto spi_wr_error;
		}

		//Transmit value to the register
		if(HAL_SPI_Transmit(&hspi1, value, 1, 10) != HAL_OK){
			goto spi_wr_error;
		}

		HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET); //Get CS down as fast as possible so the conversion starts.
		xSemaphoreGive(xSPI1_Semaphore);
		return 1;
	}
	else
	{
		return -3;
	}

//Return point if we couldn't proper use the SPI
spi_wr_error:
	HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET);
	xSemaphoreGive(xSPI1_Semaphore);
	return -2;
}

/*
 * Function to read MAX31865 Registers
 * Inputs:
 * _reg_add   - Register Address to write to
 * *result    - Pointer to the address that we're going to write the value that we read
 *
 * Returns:
 * Error level:
 * -3 - If couldn't get the Semaphore
 * -2 - If couldn't use SPI
 *
 *  1 - Success
 */
int8_t MAX31865_read_reg(uint8_t _reg_add, uint8_t * result){

	//Take Semaphore
	if( xSemaphoreTake( xSPI1_Semaphore, (TickType_t) 5) == pdTrue ){

		HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_SET); // Rise CS before writing to SPI
		//Try to read the register
		if(HAL_SPI_Transmit(&hspi1, _reg_add, 1, 10) != HAL_OK ){
			goto spi_rr_error;
		}

		//Transmit value to the register
		if(HAL_SPI_Receive(&hspi1, &result, 1, 10) != HAL_OK){
			goto spi_rr_error;
		}

		HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET); //Get CS down as fast as possible so the conversion starts.
		xSemaphoreGive(xSPI1_Semaphore);
		return 1;
	}
	else
	{
		return -3;
	}

//Return point if we couldn't proper use the SPI
spi_rr_error:
	HAL_GPIO_WritePin(CS_MAX31865, GPIO_PIN_RESET);
	xSemaphoreGive(xSPI1_Semaphore);
	return -2;
}


/*
 * Function to Generic Config the MAX31865
 * Inputs:
 * cnv_mode
 * 		1 -> Automatic Conversion
 * 		0 -> Single Shot Conversion
 *
 * wire_conf
 * 		1 -> 3-Wire RTD
 * 		0 -> 2-Wire or 4-Wire
 * filter
 * 		1 -> 50Hz
 * 		0 -> 60Hz
 *
 * Returns:
 * Error level:
 * -3 - If couldn't get the Semaphore
 * -2 - If couldn't use SPI
 *
 *  1 - Success
 */
int8_t MAX31865_generic_config(int8_t cnv_mode, int8_t wire_conf, int8_t filter){

	//Conversion Mode
	if(cnv_mode == 1){
		_reg |= ( 1UL << VBIAS );
		_reg |= ( 1UL << CONVERSION_MODE );
	}
	else if(cnv_mode == 0){
		_reg &= ~( 1UL << CONVERSION_MODE );
	}

	// 3-Wire or Even Number Config
	if(wire_conf == 1){
		_reg |= ( 1UL << WIRE_NUMBER );
	}else if( wire_conf == 0){
		_reg &= ~( 1UL << WIRE_NUMBER );
	}

	//50-60Hz Filter Config
	if(filter == 1){
		_reg |= ( 1UL << LINEFILTER );
	}else if( filter == 0 ){
		_reg &= ~( 1UL << LINEFILTER );
	}

	return MAX31865_write_reg(CONFIG_W, _reg);
}

//TODO:
int8_t MAX31865_read_temp_reg(void){

}


/*
 * 	Task Blocks
 *
 */

//TODO:
void t_read_temp(void *pvParameters){

	MAX31865_config_hard_coded();

	while(1){
		begin_one_shot_cnv();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

}




/*
 *  	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(th_adc, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

 */


/*

 	if( xSemaphoreTake( xSPI1_Semaphore, (TickType_t) 5) == pdTrue ){

		uint8_t _reg = pvPortMalloc(sizeof(uint8_t));
		//Get the values in the CONFIG REGISTER
		if(HAL_SPI_Transmit(&hspi1, MAX31865_CONFIG_REG_R, 1, 10) == HAL_OK ){
			if(HAL_SPI_Receive(&hspi1, _reg, 1, 10) == HAL_OK){
				_reg |= ( 1UL << ONE_SHOT ); //Set ONE_SHOT conversion
				if(HAL_SPI_Transmit(&hspi1, _reg, 1, 10) == HAL_OK){
						xSemaphoreGive(xSPI1_Semaphore);
						pvPortFree(_reg);
						return 1;
					}
				}
			}
		}
 *
 */
