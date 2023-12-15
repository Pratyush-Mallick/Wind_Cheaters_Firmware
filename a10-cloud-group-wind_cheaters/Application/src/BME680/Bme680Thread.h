/*
 * Bme680Thread.h
 *
 * Created: 12/14/2023 4:05:39 PM
 *  Author: praty
 */ 


#ifndef BME680THREAD_H_
#define BME680THREAD_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "main.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "BME680/bme68x.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
#define IMU_TASK_SIZE     200  //<Size of stack to assign to the UI thread. In words
#define AIR_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define AIR_VOLTAGE_LIMIT 13

/******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
void vBmeTask(void *pvParameters);

#endif /* BME680THREAD_H_ */