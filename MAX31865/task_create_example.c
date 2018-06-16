/*

Example of dispatch of both tasks.

*/
  
  //Create Mutex for SPI1 and if success create SPI tasks
  	xSPI1_Semaphore = xSemaphoreCreateMutex();

  	if( xSPI1_Semaphore != NULL){
      //Task to be signaled for RX when DRDY goes low
      xTaskCreate(t_rx_temp,
        		  "Task RX TEMP",
        		  128,
      			  (void*) xSPI1_Semaphore,
        		  3,
        		  &RTD_RX_Task); //Task Handle for TaskNotify

  	  //Create MAX31865 task
  	  xTaskCreate(t_read_temp,
  			  	  "Task MAX31865",
  				  128,
				  (void*) xSPI1_Semaphore,
  				  2,
  				  NULL);



  	}
