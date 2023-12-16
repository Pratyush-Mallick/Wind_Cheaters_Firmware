/*
 * ImuThread.h
 *
 * Created: 12/14/2023 4:06:28 PM
 *  Author: praty
 */ 


#ifndef IMUTHREAD_H_
#define IMUTHREAD_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include  "main.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "IMU/lsm6dso_reg.h"

/******************************************************************************
 * Defines
 ******************************************************************************/
#define IMU_TASK_SIZE     100  //<Size of stack to assign to the UI thread. In words
#define IMU_TASK_PRIORITY (configMAX_PRIORITIES - 1)

/******************************************************************************
 * Global Function Declaration
 ******************************************************************************/
void vImuTask(void *pvParameters);

#endif /* IMUTHREAD_H_ */