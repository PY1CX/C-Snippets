/*
 * leds.c
 *
 *  Created on: 5 de abr de 2018
 *      Author: Felipe
 */

#include "leds.h"

/*
 * This two leds blinks in two tasks at the same priority and same delay
 * it was just to see the difference of time between them in the oscilloscope
 */

/* Blink a Led Task*/
void t_LEDBLINK(void * pvParameters){
	for(;;){
		printf("BLINK\n\r");
		HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
		//f_writeDEBUG("LED BLINK\r\n");
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/* Blink a Led Task*/
void t_LEDBLINK2(void * pvParameters){
	for(;;){
		HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void init_leds(void){
/* Create Task that Blinks the LED */
xTaskCreate(t_LEDBLINK,
			"Task Led",
			128,
			NULL,
			2,
			NULL);

xTaskCreate(t_LEDBLINK2,
			"Task Led 2",
			128,
			NULL,
			2,
			NULL);
}
