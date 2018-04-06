/*
 * queue_test.h
 *
 *  Created on: 5 de abr de 2018
 *      Author: Felipe
 */
#include "cmsis_os.h"
#include "usart.h"

//Si70xx_data Queue Handle
QueueHandle_t Queue_data;


void t_Q_receive(void * pvParameters);
void t_Q_generate(void * pvParameters);
void queue_test_init(void);
