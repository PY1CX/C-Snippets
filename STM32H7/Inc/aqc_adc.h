#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "stm32h7xx_hal_adc.h"

TaskHandle_t th_adc;

int kk_fl_half_adc;
int kk_fl_compl_adc;
void init_adc(void);
void t_adc_aqc(void *pvParameters);
void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef * hadc);
void HAL_ADC_ConvHalfCpltCallback (ADC_HandleTypeDef * hadc);

