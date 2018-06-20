/*
 *	ads1220.h
 *
 *	AD9833 Driver
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

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "spi.h"
#include "gpio.h"
#include "queue.h"
#include "arm_math.h"

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
	ADS1220_GAIN_2,
	ADS1220_GAIN_4,
	ADS1220_GAIN_8,
	ADS1220_GAIN_16,
	ADS1220_GAIN_32,
	ADS1220_GAIN_64,
	ADS1220_GAIN_128,
	ADS1220_GAIN_BITMASK      = 0xF1,

	//ADS1220 MUX CONFIG
	//Named as: First AINp Last AINn

	ADS1220_MUX_AIN0_AIN1	  = 0x00,
	ADS1220_MUX_AIN0_AIN2,
	ADS1220_MUX_AIN0_AIN3,
	ADS1220_MUX_AIN1_AIN2,
	ADS1220_MUX_AIN1_AIN3,
	ADS1220_MUX_AIN2_AIN3,
	ADS1220_MUX_AIN1_AIN0,
	ADS1220_MUX_AIN3_AIN2,
	ADS1220_MUX_AIN0_AVSS,
	ADS1220_MUX_AIN1_AVSS,
	ADS1220_MUX_AIN2_AVSS,
	ADS1220_MUX_AIN3_AVSS,
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
	ADS1220_VREF_REFP0_REFN0,
	ADS1220_VREF_AIN0_AIN3,
	ADS1220_VREF_ANALOG_SUPPLY,
	ADS1220_VREF_BITMASK      = 0x3F,

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


