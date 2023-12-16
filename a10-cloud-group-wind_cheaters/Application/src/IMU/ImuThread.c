/*
 * ImuThread.c
 *
 * Created: 12/14/2023 4:06:14 PM
 *  Author: praty
 */ 

#include "ImuThread.h"
extern QueueHandle_t xQueueImuBuffer;

void vImuTask(void *pvParameters)
{
	float air_speed;
	
	// Structure definition that holds IMU data
	struct ImuDataPacket_float imuData;
	int16_t data_raw_acceleration[3];
	
	stmdev_ctx_t *dev_ctx = GetImuStruct();
	
	while(1) {
		///* Read output only if new xl value is available */
		lsm6dso_acceleration_raw_get(dev_ctx, data_raw_acceleration);		
		imuData.xmg = lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
		imuData.ymg = lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
		imuData.zmg = lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);
		if (xQueueImuBuffer) {
			WifiAddImuDataToQueue(&imuData);
		}
		port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
		vTaskDelay(500);
		port_pin_set_output_level(LED_0_PIN, LED_0_INACTIVE);
		vTaskDelay(500);
	}
}