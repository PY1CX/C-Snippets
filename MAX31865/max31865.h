/*
 *	max31865.h
 *
 *	MAX31865 Driver
 *	Author: Felipe Navarro
 *
 */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "spi.h"
#include "gpio.h"
#include "queue.h"

TaskHandle_t RTD_RX_Task;

/*  FAULT BITS ON FAULT STATUS REGISTER
 *  Table 7 from Datasheet:
 *  D7 - RTD High Threshold
 *  D6 - RTD Low  Threshold
 *  D5 - REFIN- > 0.85 * Vbias
 *  D4 - REFIN- < 0.85 * Vbias (FORCE- open)
 *  D3 - RTDIN- < 0.85 * Vbias (FORCE- open)
 *  D2 - Overvoltage/Undervoltage Fault
 *
 *  The only fault uncovered in the Fault Register is the RTDIN+ cable
 *  this one needs to be addressed when doing the conversion (with the
 *  10Meg resistor onboard, when we have a RTDIN+ cable fault the ADC
 *  conversion results will be near full scale. This needs to be
 *  addressed while reading the temperature.
 */
enum MAX31865_FAULT{D7 = 0, D6, D5, D4, D3, D2}MAX31865_FAULT;

//Config Register Bits
enum MAX31865_CONFIG_REG_BITS{
		LINEFILTER = 0,
		FAULT_STATUS_CLEAR,
		WIRE_NUMBER = 4,
		ONE_SHOT = 5,
		CONVERSION_MODE,
		VBIAS
} MAX31865_CONFIG_REG_BITS;

//MAX31865 Register Address
enum MAX31865_REGISTERS{

	//Registers Read Address
	CONFIG_R = 0x00,
	RTD_MSB_R,
	RTD_LSB_R,
	HIGH_FAULT_THRESHOLD_MSB_R,
	HIGH_FAULT_THRESHOLD_LSB_R,
	LOW_FAULT_THRESHOLD_MSB_R,
	LOW_FAULT_THRESHOLD_LSB_R,
	FAULT_STATUS_R,

	//Registers Write Address
	CONFIG_W = 0x80,
	HIGH_FAULT_THRESHOLD_MSB_W = 0x83,
	HIGH_FAULT_THRESHOLD_LSB_W,
	LOW_FAULT_THRESHOLD_MSB_W,
	LOW_FAULT_THRESHOLD_LSB_W
} MAX31865_REGISTERS;

//#define SET_BIT(x,bit) ((x) |= (1UL << (bit)))
//#define CLR_BIT(x,bit) ((x) &= ~(1UL << (bit)))

//Function Prototypes
int8_t begin_one_shot_cnv(SemaphoreHandle_t * Mutex_SPI);
int8_t MAX31865_config_hard_coded(SemaphoreHandle_t * Mutex_SPI);
void t_read_temp(void * pvParameters);
void t_rx_temp(void * pvParameters);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

