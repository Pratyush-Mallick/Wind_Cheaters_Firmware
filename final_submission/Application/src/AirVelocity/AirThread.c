/*
 * AirThread.c
 *
 * Created: 12/14/2023 4:06:51 PM
 *  Author: praty
 */

#include "AirThread.h"

extern QueueHandle_t xQueueAirBuffer;

/**
 * function         vAirTask
 * @brief           Initialization code for all subsystems that require FreeRToS
 * @details			This function is called from the FreeRToS timer task. Any code
 *					here will be called before other tasks are initilized.
 * @param[in]       None
 * @return          None
 */
void vAirTask(void *pvParameters)
{
	float air_speed;
	
	while(1) {
		air_speed = FS3000_readMetersPerSecond();
		
		if (xQueueAirBuffer) {
			WifiAddAirDataToQueue(&air_speed);
		}
		
		vTaskDelay(1000);
	}
}