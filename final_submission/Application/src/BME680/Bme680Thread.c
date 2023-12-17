/*
 * Bme680Thread.c
 *
 * Created: 12/14/2023 4:05:28 PM
 *  Author: Pratyush Mallick
 */ 

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "Bme680Thread.h"

/******************************************************************************
 * Extern Variables
 ******************************************************************************/
extern struct bme68x_dev bme;
extern QueueHandle_t xQueueBmeBuffer;
extern QueueHandle_t xQueueBmeCliBuffer;

/******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * function         vBmeTask
 * @brief           Collect data from the BME680 sensor continuously
 * @details			This task here collects bme680 sensor data periodically every second
                    and then add it to the queue ready to be publish or fetched over CLI.
 * @param[in]       pvParameters
 * @return          None
 */
void vBmeTask(void *pvParameters)
{
	int8_t rslt;
	uint8_t n_fields;
	uint8_t i = 0;
	
	struct bme68x_data data[BME68X_N_MEAS] = { { 0 } };
	
	while(1) {
		/* Trigger a measurement */
		rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        
		/* Wait for the measurement to be completed */
		vTaskDelay(pdMS_TO_TICKS((uint32_t) 1000));
		
		/* Fetch the data from the registers */
		rslt = bme68x_get_data(BME68X_FORCED_MODE, &data[0], &n_fields, &bme);
		
		/* If wifi queue initialized, add data to the queue */
		if (xQueueBmeBuffer) {
			WifiAddBmeDataToQueue(&data[0]);
		}
		
		/* If CLI queue initialized, add data to the queue */
		if (xQueueBmeCliBuffer) {
			CLIAddBmeDataToQueue(&data[0]);
		}
	}
}