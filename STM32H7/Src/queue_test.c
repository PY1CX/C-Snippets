/*
 * queue_test.c
 *
 *  Created on: 5 de abr de 2018
 *      Author: Felipe
 */
#include "queue_test.h"

void t_Q_receive(void * pvParameters){

	QueueHandle_t Queue_data = (QueueHandle_t) pvParameters;
	char ch_t2 = 0;

	for(;;){
		if(xQueueReceive(Queue_data, &ch_t2 , portMAX_DELAY) != pdPASS){

		}else{
			printf("Teste %c \r\n", ch_t2);

		}
	}
}

void t_Q_generate (void * pvParameters){

	QueueHandle_t Queue_data = (QueueHandle_t) pvParameters;
	char ch_t = 'a';
	for(;;){
		xQueueSendToBack(Queue_data, &ch_t, 0);
		ch_t++;
		if(ch_t > 'z') { ch_t = 'a'; }
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void queue_test_init(void){

	  if(Queue_data == NULL){
		  Queue_data = xQueueCreate(2, sizeof(uint8_t));

		  /* Create the Si70xx task*/
		  xTaskCreate(t_Q_generate,
				  	  "Task Q Generate",
					  128,
					  (void*) Queue_data,
					  3,
					  NULL);

		  /* Create the receiver task*/
		  xTaskCreate(t_Q_receive,
				  	  "Task Q Receive",
					  128,
					  (void*) Queue_data,
					  3,
					  NULL);


		  vQueueAddToRegistry( Queue_data, (char*)"Queue_data" );

	  }
}



