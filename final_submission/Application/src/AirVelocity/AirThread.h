/*
 * AirThread.h
 *
 * Created: 12/14/2023 4:07:05 PM
 *  Author: praty
 */

#ifndef AIRTHREAD_H_
#define AIRTHREAD_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "main.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "CliThread/CliThread.h"
#include "AirVelocity/FS_3000.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
#define AIR_TASK_SIZE     100  //<Size of stack to assign to the UI thread. In words
#define AIR_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define AIR_VOLTAGE_LIMIT 13

/******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
void vAirTask(void *pvParameters);

#endif /* AIRTHREAD_H_ */
