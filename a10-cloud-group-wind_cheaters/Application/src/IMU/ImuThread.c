/*
 * ImuThread.c
 *
 * Created: 12/14/2023 4:06:14 PM
 *  Author: praty
 */ 

#include "ImuThread.h"

void vImuTask(void *pvParameters)
{
	float air_speed;
	
	// Structure definition that holds IMU data
	struct ImuDataPacket imuData;
	int16_t data_raw_acceleration[3];
	int16_t acceleration_mg[3];
	
	stmdev_ctx_t *dev_ctx = GetImuStruct();
	
	//while(1) {
		///* Read output only if new xl value is available */
		////lsm6dso_xl_flag_data_ready_get(dev_ctx, &reg);
//
		//memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		//lsm6dso_acceleration_raw_get(dev_ctx, data_raw_acceleration);
		//acceleration_mg[0] =
		//lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
		//acceleration_mg[1] =
		//lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
		//acceleration_mg[2] =
		//lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);
		//
		//imuData.xmg = (int) acceleration_mg[0];
		//imuData.ymg = (int) acceleration_mg[1];
		//imuData.zmg = (int) acceleration_mg[2];
		//
		//WifiAddImuDataToQueue(&imuData);
		//
		//vTaskDelay(100);
	//}
}