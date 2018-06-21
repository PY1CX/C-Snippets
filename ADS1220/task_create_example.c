/*
Example of dispatch of both tasks.
*/
  
  //Create Mutex for SPI1 and if success create SPI tasks
  	xSPI1_Semaphore = xSemaphoreCreateMutex();
    
    //Check if Semaphore was created
  	if( xSPI1_Semaphore != NULL){
    
  	  /*
  	   *  ADS1220 Tasks
  	   */
      xTaskCreate(t_RX_ADS1220,
        		  "Task RX ADS1220",
        		  128,
      			  (void*) xSPI1_Semaphore,
        		  3,
        		  &ADS1220_RX_Task); //Task Handle for TaskNotify

  	  xTaskCreate(t_one_shot_ADS1220,
  			  	  "Task ADS1220",
  				  128,
				  (void*) xSPI1_Semaphore,
  				  2,
  				  NULL);
  	}
