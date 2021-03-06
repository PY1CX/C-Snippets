/*
 *	ads1220.h
 *
 *	ADS1220 Driver
 *	Author: Felipe Navarro
 *
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "spi.h"
#include "gpio.h"
#include "queue.h"
#include "arm_math.h"

TaskHandle_t ADS1220_RX_Task;

/*
 *  Project related values
 */

#define VREF (2.048)

#define PERIOD_PH_MEASUREMENT (50)

#define FSR (((long int)1<<23)-1)

/*
 *  ADS1220 Registers and Command Values
 */

enum ADS1220_REGISTERS_AND_COMMANDS{

	//Write Single Register Command
	ADS1220_REG0_W = 0x40,
	ADS1220_REG1_W = 0x44,
	ADS1220_REG2_W = 0x48,
	ADS1220_REG3_W = 0x4C,

	//Read Single Register Command
	ADS1220_REG0_R = 0x20,
	ADS1220_REG1_R = 0x24,
	ADS1220_REG2_R = 0x28,
	ADS1220_REG3_R = 0x2C,

	//Read Data Command
	ADS1220_READ_DATA  = 0x10,
	//Start Sync Conversion
	ADS1220_START_SYNC = 0x08,
	//Reset
	ADS1220_RESET      = 0x06,
	//Power Down
	ADS1220_POWER_DOWN = 0x02
};

/*
 *  ADS1220_CONFIG is an enum for making it easier to enable/disable
 *  configs on the ADS1220.
 *
 *  EXAMPLE OF USAGE:
 *	Multiple bits Config:
 *  REG = ( REG & BITMASK ) | VALUE;
 *
 *  Single Bit Config:
 *  REG |= ( 1UL << BIT POSITION );
 */
enum ADS1220_CONFIG_VALUES{

	// Config Register 0
	ADS1220_GAIN_1 			  = 0x00,
	ADS1220_GAIN_2			  = (1<<1),
	ADS1220_GAIN_4			  = (1<<2),
	ADS1220_GAIN_8			  = (1<<2) | (1<<1),
	ADS1220_GAIN_16 		  = (1<<3),
	ADS1220_GAIN_32			  = (1<<3) | (1<<1),
	ADS1220_GAIN_64           = (1<<3) | (1<<3),
	ADS1220_GAIN_128		  = (1<<3) | (1<<2) | (1<<1),
	ADS1220_GAIN_BITMASK      = 0xF1,

	//ADS1220 MUX CONFIG
	//Named as: First AINp Last AINn

	ADS1220_MUX_AIN0_AIN1	  = 0x00,
	ADS1220_MUX_AIN0_AIN2	  = (1<<4),
	ADS1220_MUX_AIN0_AIN3	  = (1<<5),
	ADS1220_MUX_AIN1_AIN2     = (1<<5) | (1<<4),
	ADS1220_MUX_AIN1_AIN3     = (1<<6),
	ADS1220_MUX_AIN2_AIN3	  = (1<<6) | (1<<4),
	ADS1220_MUX_AIN1_AIN0     = (1<<6) | (1<<5),
	ADS1220_MUX_AIN3_AIN2     = (1<<6) | (1<<5) | (1<<4),
	ADS1220_MUX_AIN0_AVSS     = (1<<7),
	ADS1220_MUX_AIN1_AVSS     = (1<<7) | (1<<4),
	ADS1220_MUX_AIN2_AVSS     = (1<<7) | (1<<5),
	ADS1220_MUX_AIN3_AVSS     = (1<<7) | (1<<5) | (1<<4),
	//There is more FOUR cases not listed here
	//because they're not going to be used anyway.
	ADS1220_MUX_BITMASK		  = 0x0F,

	// Config Register 1

	ADS1220_DATA_RATE_0       = 0x00,
	ADS1220_DATA_RATE_1,
	ADS1220_DATA_RATE_2,
	ADS1220_DATA_RATE_3,
	ADS1220_DATA_RATE_4,
	ADS1220_DATA_RATE_5,
	ADS1220_DATA_RATE_6,
	ADS1220_DATA_RATE_7, //Reserved
	ADS1220_DATA_RATE_BITMASK = 0x1F,

	ADS1220_MODE_NORMAL       = 0x00,
	ADS1220_MODE_DUTY_CYCLE,
	ADS1220_MODE_TURBO,
	ADS1220_MODE_BITMASK      = 0xE7,

	// Config Register 2
	ADS1220_IDAC_OFF          = 0x00,
	ADS1220_IDAC_RES,
	ADS1220_IDAC_50ua,
	ADS1220_IDAC_100ua,
	ADS1220_IDAC_250ua,
	ADS1220_IDAC_500ua,
	ADS1220_IDAC_1000ua,
	ADS1220_IDAC_1500ua,
	ADS1220_IDAC_BITMASK      = 0xF8,

	ADS1220_VREF_INTERNAL     = 0x00,
	ADS1220_VREF_REFP0_REFN0  = (1<<6),
	ADS1220_VREF_AIN0_AIN3	  = (1<<7),
	ADS1220_VREF_ANALOG_SUPPLY= (1<<6)|(1<<7),
	ADS1220_VREF_BITMASK      = 0x3F,

	ADS1220_FIR_FILTER_NONE   = 0x00,
	ADS1220_FIR_FILTER_5060   = (1<<4),
	ADS1220_FIR_FILTER_50	  = (1<<5),
	ADS1220_FIR_FILTER_60     = (1<<5)|(1<<4),
	ADS1220_FIR_FILTER_BITMASK= 0xCF,

	// Config Register 3
	ADS1220_IDAC1_DISABLED    = 0x00,
	ADS1220_IDAC1_AIN0_REFP1,
	ADS1220_IDAC1_AIN1,
	ADS1220_IDAC1_AIN2,
	ADS1220_IDAC1_AIN3_REFN1,
	ADS1220_IDAC1_REFP0,
	ADS1220_IDAC1_REFN0,
	ADS1220_IDAC1_RESERVED,
	ADS1220_IDAC1_BITMASK     = 0x1F,

	ADS1220_IDAC2_DISABLED    = 0x00,
	ADS1220_IDAC2_AIN0_REFP1,
	ADS1220_IDAC2_AIN1,
	ADS1220_IDAC2_AIN2,
	ADS1220_IDAC2_AIN3_REFN1,
	ADS1220_IDAC2_REFP0,
	ADS1220_IDAC2_REFN0,
	ADS1220_IDAC2_RESERVED,
	ADS1220_IDAC2_BITMASK     = 0xE3,

}ADS1220_CONFIG_VALUES;

enum ADS1220_CONFIG_POS{

	// Config Register 0
	ADS1220_CONFIG_PGA_BYPASS = 0x0,
	ADS1220_CONFIG_GAIN 	  = 0x1,
	ADS1220_CONFIG_MUX        = 0x4,

	// Config Register 1
	ADS1220_CONFIG_BCS        = 0x0,
	ADS1220_CONFIG_TS         = 0x1,
	ADS1220_CONFIG_CONV_MODE  = 0x2,
	ADS1220_CONFIG_OP_MODE    = 0x3,
	ADS1220_CONFIG_DATA_RATE  = 0x5,

	// Config Register 2
	ADS1220_CONFIG_IDAC       = 0x0,
	ADS1220_CONFIG_PSW        = 0x3,
	ADS1220_CONFIG_FIR_FILTER = 0x4,
	ADS1220_CONFIG_VREF       = 0x6,

	// Config Register 3
	ADS1220_CONFIG_RESERVED   = 0x0, // Never change this register to 1
	ADS1220_CONFIG_DRDYM      = 0x1,
	ADS1220_CONFIG_I2MUX      = 0x2,
	ADS1220_CONFIG_I1MUX      = 0x5,

}ADS1220_CONFIG_POS;

enum ADS1220_FAULTS{
	ADS1220_SEMAPHORE_FAULT   = -3,
	ADS1220_SPI_FAULT 		  = -2,
	ADS1220_CONFIG_FAULT      = -1,
	ADS1220_SUCCESS           =  0
}ADS1220_FAULTS;

// Functions and Tasks Prototypes
int8_t ADS1220_start_conversion(SemaphoreHandle_t * Mutex_SPI);
int8_t WRITE_REG_ADS1220(uint8_t * _reg_addr, uint8_t * _reg_data);
int8_t SEND_CMD_ADS1220(uint8_t * _reg_addr);
void READ_CONFIG_REGISTERS(SemaphoreHandle_t Mutex_SPI);
int8_t ADS1220_config(SemaphoreHandle_t * Mutex_SPI);
void t_one_shot_ADS1220(void * pvParameters);
void t_RX_ADS1220(void * pvParameters);

