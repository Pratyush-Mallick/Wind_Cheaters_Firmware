/*
 * Bme680Thread.c
 *
 * Created: 12/14/2023 4:05:28 PM
 *  Author: praty
 */ 

#include "Bme680Thread.h"

extern struct bme68x_dev bme;

void vBmeTask(void *pvParameters)
{
	int8_t rslt;
	uint8_t n_fields;
	uint8_t i = 0;
	struct bme68x_data data[BME68X_N_MEAS] = { { 0 } };
	
	//while(1) {
		////struct bme68x_dev t_dev;
		//rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme); /* Trigger a measurement */
//
		///* Wait for the measurement to complete */
		////t_dev.delay__us(BME68X_HEATR_DUR1_DELAY, t_dev.intf_ptr);
		//vTaskDelay(pdMS_TO_TICKS((uint32_t) 1000));
		//rslt = bme68x_get_data(BME68X_FORCED_MODE, &data[0], &n_fields, &bme);
		//
		//WifiAddBmeDataToQueue(&data);
	//}
}