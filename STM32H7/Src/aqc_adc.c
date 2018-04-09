#include "aqc_adc.h"

#define __SECTION_AXIRAM __attribute__((section(".RAM_AXI"))) /* AXI SRAM (D1 domain): */

#define __SECTION_RAM_D2 __attribute__((section(".RAM_D2"))) /* AHB SRAM (D2 domain): */

#define __SECTION_RAM_D3 __attribute__((section(".RAM_D3"))) /* AHB SRAM (D3 domain): */

ALIGN_32BYTES (__SECTION_RAM_D2 uint16_t AdcValues_i16[4096]);


void t_adc_aqc(void *pvParameters){
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    printf("ERRO INICIALIZANDO ADC\r\n");
	  }
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcValues_i16, sizeof(AdcValues_i16)) != HAL_OK){
		printf("ERRO INICIALIZANDO ADC DMA\r\n");
	}
	for(;;){

	//HAL_ADC_Start(&hadc1);
		if(AdcValues_i16[0] > 20000){
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
		}

		//printf(HAL_ADC_GetState(&hadc1));
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc){
	kk_fl_compl_adc++;
	vTaskNotifyGiveFromISR(th_adc, pdFALSE);
	//printf("Conv Comp \r\n");

}

void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef * hadc){
	kk_fl_half_adc++;
	//printf("Conv Half Comp \r\n");
}

void t_adc_rx(void * pvParameters){
	for(;;){

		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);
			printf("K1\r\n");

	}
}


void init_adc(void){

	xTaskCreate(t_adc_aqc,
			  	  "Task ADC",
				  256,
				  NULL,
				  2,
				  NULL);

	th_adc = xTaskCreate(t_adc_rx,
			  	  "Task ADC RX",
				  256,
				  NULL,
				  2,
				  NULL);

}
