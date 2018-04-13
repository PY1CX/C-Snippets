#include "aqc_adc.h"

//#define PRINT_AQC
#define PRINT_FFT

#define SAMPLE_SIZE 2048
__SECTION_RAM_D2 float32_t AdcValues_i16[SAMPLE_SIZE];


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
	static float32_t testOutput[SAMPLE_SIZE/2];
	uint32_t fftSize = SAMPLE_SIZE/2;
	uint32_t ifftFlag = 0;
	uint32_t doBitReverse = 1;
	uint32_t maxValue, maxIndex;

	arm_cfft_instance_f32 S;
	for(;;){
	    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

	    arm_cfft_radix4_init_f32(&S, fftSize, ifftFlag, doBitReverse);
	    arm_cfft_radix4_f32(&S, AdcValues_i16);
	    arm_cmplx_mag_f32(AdcValues_i16, testOutput, fftSize);
	    arm_max_f32(testOutput, fftSize, &maxValue, &maxIndex);
	    printf("Max Value %lu Max Index %lu", maxValue, maxIndex);
#ifdef PRINT_FFT
	    printf("__________START__________\r\n");
	    for(int index = 0; index < fftSize ; index ++){
	    	printf("%f ", testOutput[index] );
	    }
	    printf("__________FINISH__________\r\n");
#endif

#ifdef PRINT_AQC
	    printf("__________START__________\r\n");
	    for(int index = 0; index < SAMPLE_SIZE ; index ++){
	    	printf("%.6f ", AdcValues_i16[index] );
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
				  2048,
				  NULL,
				  2,
				  NULL);
}
