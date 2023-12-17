/*
 * Bme680Thread.h
 *
 * Created: 12/14/2023 4:05:39 PM
 *  Author: Pratyush Mallick
 */ 

#ifndef BME680THREAD_H_
#define BME680THREAD_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "main.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "CliThread/CliThread.h"
#include "BME680/bme68x.h"
#include "BME680/bme68x_defs.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
#define BME_TASK_SIZE     200  //<Size of stack to assign to the BME thread.
#define BME_TASK_PRIORITY (configMAX_PRIORITIES - 1)

/******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
void vBmeTask(void *pvParameters);

#endif /* BME680THREAD_H_ */