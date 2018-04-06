/*
 * leds.h
 *
 *  Created on: 5 de abr de 2018
 *      Author: Felipe
 */


#include "cmsis_os.h"
#include "gpio.h"


void t_LEDBLINK(void * pvParameters);
void t_LEDBLINK2(void * pvParameters);
void init_leds(void);
