#include "aqc_adc.h"

#define SAMPLE_SIZE 4096
ALIGN_32BYTES (__SECTION_RAM_D2 uint16_t AdcValues_i16[SAMPLE_SIZE]);

void t_adc_aqc(void *pvParameters){
	//Init the ADC
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    printf("ERRO INICIALIZANDO ADC\r\n");
	  }

	for(;;){
		//Aqcuire
		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcValues_i16, sizeof(AdcValues_i16)) != HAL_OK){
			printf("ERRO INICIALIZANDO ADC DMA\r\n");
		}
		//Just a visual check
		if(AdcValues_i16[0] > 20000){
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc){
	kk_fl_compl_adc++;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(th_adc, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef * hadc){
	kk_fl_half_adc++;
}

void t_adc_rx(void * pvParameters){

	for(;;){
	    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
#ifdef PRINT_AQC
	    printf("__________START__________\r\n");
	    for(int index = 0; index < SAMPLE_SIZE ; index ++){
	    	printf("%i ", AdcValues_i16[index] );
	    }
	    printf("__________FINISH__________\r\n");
#endif
	}
}


void init_adc(void){

	xTaskCreate(t_adc_rx,
			  	  "Task ADC RX",
				  256,
				  NULL,
				  4,
				  &th_adc);

	xTaskCreate(t_adc_aqc,
			  	  "Task ADC",
				  256,
				  NULL,
				  2,
				  NULL);
}
