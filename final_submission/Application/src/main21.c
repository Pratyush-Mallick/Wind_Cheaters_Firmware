/**
 * @file      main.c
 * @brief     Main application entry point
 * @author    Eduardo Garcia and Pratyush
 * @date      2022-04-14
 * @copyright Copyright Bresslergroup\n
 *            This file is proprietary to Bresslergroup.
 *            All rights reserved. Reproduction or distribution, in whole
 *            or in part, is forbidden except by express written permission
 *            of Bresslergroup.
 ******************************************************************************/

/****
 * Includes
 ******************************************************************************/
#include <errno.h>

#include "CliThread/CliThread.h"
#include "FreeRTOS.h"
#include "IMU\lsm6dso_reg.h"
#include "SerialConsole.h"
#include "WifiHandlerThread/WifiHandler.h"
#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "I2cDriver\I2cDriver.h"
#include "CliThread/CliThread.h"
#include "BME680/Bme680Thread.h"
#include "IMU/ImuThread.h"
#include "AirVelocity/AirThread.h"
#include "CliThread/CliThread.h"
#include "main.h"
#include "stdio_serial.h"

#include "IMU\lsm6dso_reg.h"
#include "BME680\bme68x.h"
#include "SpiDriver\SpiDriver.h"
#include "AirVelocity\FS_3000.h"

/****
 * Defines and Types
 ******************************************************************************/
#define APP_TASK_ID 0 /**< @brief ID for the application task */
#define CLI_TASK_ID 1 /**< @brief ID for the command line interface task */

/****
 * Local Function Declaration
 ******************************************************************************/
void vApplicationIdleHook(void);
//!< Initial task used to initialize HW before other tasks are initialized
static void StartTasks(void);
void vApplicationDaemonTaskStartupHook(void);

void vApplicationStackOverflowHook(void);
void vApplicationMallocFailedHook(void);
void vApplicationTickHook(void);

/****
 * Variables
 ******************************************************************************/
static TaskHandle_t cliTaskHandle = NULL;      //!< CLI task handle
static TaskHandle_t daemonTaskHandle = NULL;   //!< Daemon task handle
static TaskHandle_t wifiTaskHandle = NULL;     //!< Wifi task handle

static TaskHandle_t accelTaskHandle = NULL;    //!< Accelerometer task handle
static TaskHandle_t airTaskHandle = NULL;      //!< Air Velocity task handle
static TaskHandle_t bmeTaskHandle = NULL;      //!< BME680 task handle

char bufferPrint[64];  ///< Buffer for daemon task

/* Bme680 specific device descriptor */
struct bme68x_dev bme;
int8_t rslt;

/* LSM6DSO SPI device descriptor */
stmdev_ctx_t *dev_ctx;

void app_gpio_config(void) 
{
	/* Get default config for led */
	struct port_config led_port_pin;
	port_get_config_defaults(&led_port_pin);
	
	led_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &led_port_pin);
	
	struct port_config dir_port_pin;
	port_get_config_defaults(&dir_port_pin);
	
	dir_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(DIRECTION, &dir_port_pin);
	
	struct port_config step_port_pin;
	port_get_config_defaults(&step_port_pin);
	
	step_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(STEP, &step_port_pin);
}

/**
 * @brief Main application function.
 * Application entry point.
 * @return int
 */
int main(void)
{
    /* Initialize the board. */
    system_init();

    /* Initialize the UART console. */
    InitializeSerialConsole();
	
	app_gpio_config();

    // Initialize trace capabilities
    vTraceEnable(TRC_START);
	
    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;  // Will not get here
}

/**
 * function          vApplicationDaemonTaskStartupHook
 * @brief            Initialization code for all subsystems that require FreeRToS
 * @details			This function is called from the FreeRToS timer task. Any code
 *					here will be called before other tasks are initialized.
 * @param[in]        None
 * @return           None
 */
void vApplicationDaemonTaskStartupHook(void)
{
	int result;
	
    SerialConsoleWriteString("\r\n\r\n----- Wind Cheaters: V1 -----\r\n");

    // Initialize HW that needs FreeRTOS Initialization
    SerialConsoleWriteString("\r\n\r\nInitialize HW...\r\n");
    if (I2cInitializeDriver() != STATUS_OK) {
        SerialConsoleWriteString("Error initializing I2C Driver!\r\n");
    } else {
        SerialConsoleWriteString("Initialized I2C Driver!\r\n");
    }
	
	/* To let the sensor power-on properly before communicating */
	vTaskDelay(2000);
	
	/* Initialize the BME680 Sensor
     * Interface preference is updated as a parameter
     * For I2C : BME68X_I2C_INTF
     */
    result = bme68x_interface_init(&bme, BME68X_I2C_INTF);
    result |= bme68x_init(&bme);
    result |= bme68x_default_config(&bme);

    if (result != BME68X_OK) {
	    SerialConsoleWriteString("BME680 Initial failed!\r\n");
	} else{
	    SerialConsoleWriteString("BME680 Initialed Success!\r\n");
    }
	
	/* Configure SPI for LSM6DSO */
	configure_spi_master();
	dev_ctx = GetImuStruct();
	
	/* Passing device specific handle. */
	dev_ctx->handle = &spi_master_instance;

    if (InitImu() == 0) {
	    SerialConsoleWriteString("IMU initialized!\r\n");
	} else {
	    SerialConsoleWriteString("Could not initialize IMU\r\n");
    }
	
	FS3000_begin();

    StartTasks();

    vTaskSuspend(daemonTaskHandle);
}

/**
 * function          StartTasks
 * @brief            Initialize application tasks
 * @details
 * @param[in]        None
 * @return           None
 */
static void StartTasks(void)
{
    snprintf(bufferPrint, 64, "Heap before starting tasks: %d\r\n", xPortGetFreeHeapSize());
    SerialConsoleWriteString(bufferPrint);

    // Initialize Tasks here
    if (xTaskCreate(vCommandConsoleTask, "CLI_TASK", CLI_TASK_SIZE, NULL, CLI_PRIORITY, &cliTaskHandle) != pdPASS) {
        SerialConsoleWriteString("ERR: CLI task could not be initialized!\r\n");
    }

    if (xTaskCreate(vWifiTask, "WIFI_TASK", WIFI_TASK_SIZE, NULL, WIFI_PRIORITY, &wifiTaskHandle) != pdPASS) {
        SerialConsoleWriteString("ERR: WIFI task could not be initialized!\r\n");
    }
	
	if (xTaskCreate(vImuTask, "ACCEL_TASK", IMU_TASK_SIZE, NULL, AIR_TASK_PRIORITY, &accelTaskHandle) != pdPASS) {
		SerialConsoleWriteString("ERR: IMU task could not be initialized!\r\n");
	}

	
	if (xTaskCreate(vBmeTask, "BME_TASK", BME_TASK_SIZE, NULL, BME_TASK_PRIORITY, &bmeTaskHandle) != pdPASS) {
		 SerialConsoleWriteString("ERR: BME task could not be initialized!\r\n");
	}
	

	snprintf(bufferPrint, 64, "Heap after all tasks %d\r\n", xPortGetFreeHeapSize());
	SerialConsoleWriteString(bufferPrint);
}

void vApplicationMallocFailedHook(void)
{
    SerialConsoleWriteString("Error on memory allocation on FREERTOS!\r\n");
    while (1);
}

void vApplicationStackOverflowHook(void)
{
    SerialConsoleWriteString("Error on stack overflow on FREERTOS!\r\n");
    while (1);
}

#include "MCHP_ATWx.h"
void vApplicationTickHook(void)
{
    SysTick_Handler_MQTT();
}
