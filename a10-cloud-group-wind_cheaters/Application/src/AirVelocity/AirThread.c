/*
 * AirThread.c
 *
 * Created: 12/14/2023 4:06:51 PM
 *  Author: praty
 */

#include "AirThread.h"

void vAirTask(void *pvParameters)
{
	float air_speed;
	
	//while(1) {
		//air_speed = FS3000_readMetersPerSecond();
		//WifiAddAirDataToQueue(&air_speed);
	//}

	vTaskDelay(500);
}