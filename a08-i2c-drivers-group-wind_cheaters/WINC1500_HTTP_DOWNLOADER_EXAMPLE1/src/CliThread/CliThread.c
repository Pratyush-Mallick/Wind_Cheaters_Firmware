/**************************************************************************//**
* @file      CliThread.c
* @brief     File for the CLI Thread handler. Uses FREERTOS + CLI
* @author    Eduardo Garcia
* @date      2020-02-15

******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/
#include "CliThread.h"
#include "WifiHandlerThread\WifiHandler.h"
#include "IMU\lsm6dso_reg.h"
#include "SeesawDriver/Seesaw.h"
#include "BME680/bme68x.h"
#include "AirVelocity/FS_3000.h"

/******************************************************************************
* Defines
******************************************************************************/


/******************************************************************************
* Variables
******************************************************************************/
static const int8_t * const pcWelcomeMessage =
"FreeRTOS CLI.\r\nType Help to view a list of registered commands.\r\n";

extern struct bme68x_dev bme;

SemaphoreHandle_t cliCharReadySemaphore;		///<Semaphore to indicate that a character has been received


static const CLI_Command_Definition_t xTempGetCommand =
{
	"temp",
	"temp: Returns a value from the temperature sensor\r\n",
	CLI_GetTempData,
	0
};

static const CLI_Command_Definition_t xImuGetCommand =
{
	"imu",
	"imu: Returns a value from the IMU\r\n",
	CLI_GetImuData,
	0
};

static const CLI_Command_Definition_t xResetCommand =
{
	"reset",
	"reset: Resets the device\r\n",
	CLI_ResetDevice,
	0
};

static const CLI_Command_Definition_t xAirFlow = 
{
	"air",
	"air: Returns a value from FS-3000 airflow sensor\r\n", 
	(const pdCOMMAND_LINE_CALLBACK)CLI_AirFlow, 
	0
};

static const CLI_Command_Definition_t xOTAUCommand = 
{
	"fw", 
	"fw: Download a file and perform an FW update\r\n", 
	(const pdCOMMAND_LINE_CALLBACK)CLI_OTAU, 
	0
};


//Clear screen command
const CLI_Command_Definition_t xClearScreen =
{
	CLI_COMMAND_CLEAR_SCREEN,
	CLI_HELP_CLEAR_SCREEN,
	CLI_CALLBACK_CLEAR_SCREEN,
	CLI_PARAMS_CLEAR_SCREEN
};


/******************************************************************************
* Forward Declarations
******************************************************************************/
static void FreeRTOS_read(char* character);

/******************************************************************************
* Callback Functions
******************************************************************************/


/******************************************************************************
* CLI Thread
******************************************************************************/

void vCommandConsoleTask( void *pvParameters )
{
//REGISTER COMMANDS HERE

FreeRTOS_CLIRegisterCommand( &xTempGetCommand );
FreeRTOS_CLIRegisterCommand( &xImuGetCommand );
FreeRTOS_CLIRegisterCommand( &xClearScreen );
FreeRTOS_CLIRegisterCommand( &xResetCommand );
FreeRTOS_CLIRegisterCommand (&xAirFlow);
FreeRTOS_CLIRegisterCommand (&xOTAUCommand);

uint8_t cRxedChar[2], cInputIndex = 0;
BaseType_t xMoreDataToFollow;
/* The input and output buffers are declared static to keep them off the stack. */
static int8_t pcOutputString[ MAX_OUTPUT_LENGTH_CLI  ], pcInputString[ MAX_INPUT_LENGTH_CLI ];
static char pcLastCommand[ MAX_INPUT_LENGTH_CLI ];
static bool isEscapeCode = false;
static char pcEscapeCodes [4];
static uint8_t pcEscapeCodePos = 0;

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */

    /* Send a welcome message to the user knows they are connected. */
    SerialConsoleWriteString( pcWelcomeMessage);

	//Any semaphores/mutexes/etc you needed to be initialized, you can do them here
	cliCharReadySemaphore = xSemaphoreCreateBinary();
	if(cliCharReadySemaphore == NULL)
	{
		LogMessage(LOG_ERROR_LVL, "Could not allocate semaphore\r\n");
		vTaskSuspend( NULL );
	}


    for( ;; )
    {

	FreeRTOS_read(&cRxedChar[0]);

	if( cRxedChar[0] == '\n' || cRxedChar[0] == '\r'  )
        {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            SerialConsoleWriteString("\r\n");
			//Copy for last command
			isEscapeCode = false; pcEscapeCodePos = 0;
			strncpy(pcLastCommand, pcInputString, MAX_INPUT_LENGTH_CLI-1);
			pcLastCommand[MAX_INPUT_LENGTH_CLI-1] = 0;	//Ensure null termination

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            explanation of why this is. */
            do
            {
                /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand
                              (
                                  pcInputString,   /* The command string.*/
                                  pcOutputString,  /* The output buffer. */
                                  MAX_OUTPUT_LENGTH_CLI/* The size of the output buffer. */
                              );

                /* Write the output generated by the command interpreter to the
                console. */
				//Ensure it is null terminated
				pcOutputString[MAX_OUTPUT_LENGTH_CLI - 1] = 0;
                SerialConsoleWriteString(pcOutputString);

            } while( xMoreDataToFollow != pdFALSE );

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            cInputIndex = 0;
            memset( pcInputString, 0x00, MAX_INPUT_LENGTH_CLI );
			memset( pcOutputString, 0, MAX_OUTPUT_LENGTH_CLI);
        }
        else
        {
		            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */
		
			if (true == isEscapeCode) {

				if(pcEscapeCodePos < CLI_PC_ESCAPE_CODE_SIZE) {
					pcEscapeCodes[pcEscapeCodePos++] = cRxedChar[0];
				}
				else {
					isEscapeCode = false; pcEscapeCodePos = 0;
				}
			
				if (pcEscapeCodePos >= CLI_PC_MIN_ESCAPE_CODE_SIZE) {
				
					// UP ARROW SHOW LAST COMMAND
					if(strcasecmp(pcEscapeCodes, "oa"))	{
                            /// Delete current line and add prompt (">")
                            sprintf(pcInputString, "%c[2K\r>", 27);
				            SerialConsoleWriteString((char*)pcInputString);
                            /// Clear input buffer
                            cInputIndex = 0;
                            memset( pcInputString, 0x00, MAX_INPUT_LENGTH_CLI );
                        /// Send last command
						strncpy(pcInputString, pcLastCommand, MAX_INPUT_LENGTH_CLI - 1); 	
                        cInputIndex = (strlen(pcInputString) < MAX_INPUT_LENGTH_CLI - 1) ? strlen(pcLastCommand) : MAX_INPUT_LENGTH_CLI - 1;
						SerialConsoleWriteString(pcInputString);
					}
				
					isEscapeCode = false; pcEscapeCodePos = 0;
				}			
			}
            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */

            else if( cRxedChar[0] == '\r' )
            {
                /* Ignore carriage returns. */
            }
            else if( cRxedChar[0] == ASCII_BACKSPACE || cRxedChar[0] == ASCII_DELETE )
            {
				char erase[4] = {0x08, 0x20, 0x08, 0x00};
				SerialConsoleWriteString(erase);
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if( cInputIndex > 0 )
                {
                    cInputIndex--;
                    pcInputString[ cInputIndex ] = 0;
                }
            }
			// ESC
			else if( cRxedChar[0] == ASCII_ESC) {
				isEscapeCode = true; //Next characters will be code arguments
				pcEscapeCodePos = 0;
			}
            else
            {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a n is entered the complete
                string will be passed to the command interpreter. */
                if( cInputIndex < MAX_INPUT_LENGTH_CLI )
                {
                    pcInputString[ cInputIndex ] = cRxedChar[0];
                    cInputIndex++;
                }

					//Order Echo
					cRxedChar[1] = 0;
					SerialConsoleWriteString(&cRxedChar[0]);
            }
        }
    }
}



/**************************************************************************//**
* @fn			void FreeRTOS_read(char* character)
* @brief		This function block the thread unless we received a character
* @details		This function blocks until UartSemaphoreHandle is released to continue reading characters in CLI
* @note
*****************************************************************************/
static void FreeRTOS_read(char* character)
{


//We check if there are more characters in the buffer that arrived since the last time
//This function returns -1 if the buffer is empty, other value otherwise
int ret = SerialConsoleReadCharacter(character);


if(ret == -1)
{
	//there are no more characters - block the thread until we receive a semaphore indicating reception of at least 1 character
	xSemaphoreTake(cliCharReadySemaphore, portMAX_DELAY);

	//If we are here it means there are characters in the buffer - we re-read from the buffer to get the newly acquired character
	SerialConsoleReadCharacter(character);
}


}

/**************************************************************************//**
* @fn			void CliCharReadySemaphoreGiveFromISR(void)
* @brief		Give cliCharReadySemaphore binary semaphore from an ISR
* @details		
* @note
*****************************************************************************/
void CliCharReadySemaphoreGiveFromISR(void)
{
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( cliCharReadySemaphore, &xHigherPriorityTaskWoken );
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/******************************************************************************
* CLI Functions - Define here
******************************************************************************/
//Example CLI Command. Reads from the IMU and returns data.
BaseType_t CLI_GetTempData( int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
	int8_t rslt;
	uint8_t n_fields;
	uint8_t i = 0;
	struct bme68x_data data[BME68X_N_MEAS] = { { 0 } };
	//struct bme68x_dev t_dev;

	rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme); /* Trigger a measurement */

	/* Wait for the measurement to complete */
	//t_dev.delay__us(BME68X_HEATR_DUR1_DELAY, t_dev.intf_ptr);
	vTaskDelay(pdMS_TO_TICKS((uint32_t) 1000));
	rslt = bme68x_get_data(BME68X_FORCED_MODE, &data[0], &n_fields, &bme);
		    
	sprintf(pcWriteBuffer, "T: %d H: %d P: %d \n", data[0].temperature, data[0].humidity, data[0].pressure);
	
	return pdFALSE;
}

//Example CLI Command. Reads from the IMU and returns data.
BaseType_t CLI_GetImuData( int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
	static int16_t  data_raw_acceleration[3];
	static int16_t  data_raw_angular_rate;
	static float acceleration_mg[3];
	uint8_t reg  = 1;
	stmdev_ctx_t *dev_ctx = GetImuStruct();

	/* Read output only if new xl value is available */
	//lsm6dso_xl_flag_data_ready_get(dev_ctx, &reg);

	if (reg) {
		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(dev_ctx, data_raw_acceleration);
		acceleration_mg[0] =
		lsm6dso_from_fs2_to_mg(data_raw_acceleration[0]);
		acceleration_mg[1] =
		lsm6dso_from_fs2_to_mg(data_raw_acceleration[1]);
		acceleration_mg[2] =
		lsm6dso_from_fs2_to_mg(data_raw_acceleration[2]);

		snprintf(pcWriteBuffer, xWriteBufferLen, "Acceleration [mg]:X %d\tY %d\tZ %d\r\n",
		(int)acceleration_mg[0], (int)acceleration_mg[1], (int)acceleration_mg[2]);
	} else {
		snprintf(pcWriteBuffer,xWriteBufferLen, "No data ready! \r\n");
	}

	return pdFALSE;
}

//THIS COMMAND USES vt100 TERMINAL COMMANDS TO CLEAR THE SCREEN ON A TERMINAL PROGRAM LIKE TERA TERM
//SEE http://www.csie.ntu.edu.tw/~r92094/c++/VT100.html for more info
//CLI SPECIFIC COMMANDS
static char bufCli[CLI_MSG_LEN];
BaseType_t xCliClearTerminalScreen( char *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
	char clearScreen = ASCII_ESC;
	snprintf(bufCli, CLI_MSG_LEN - 1, "%c[2J", clearScreen);
	snprintf(pcWriteBuffer, xWriteBufferLen, bufCli);
	return pdFALSE;
}


//Example CLI Command. Resets system.
BaseType_t CLI_ResetDevice( int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString )
{
	system_reset();
	return pdFALSE;
}

/**************************************************************************/ /**
 * @brief    Air Flow Command
 * @param    p_cli 
 ******************************************************************************/
BaseType_t CLI_AirFlow(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	float air_speed = FS3000_readMetersPerSecond();
	
    sprintf(pcWriteBuffer, "Airflow: %0.2f m/s ", air_speed);
	
    //SerialConsoleWriteString(bufCli);
	return pdFALSE;
}

// Example CLI Command. Reads from the IMU and returns data.
BaseType_t CLI_OTAU(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString)
{
	char bootloader_flag[] = "0:boot_flag.txt";
	f_unlink(bootloader_flag);
	
	FIL file_object;
	
	bootloader_flag[0] = LUN_ID_SD_MMC_0_MEM + '0';
	FRESULT res = f_open(&file_object, (char const *)bootloader_flag, FA_CREATE_ALWAYS | FA_WRITE);

	if (res != FR_OK) {
		LogMessage(LOG_INFO_LVL, "[FAIL] res %d\r\n", res);
		} else {
		SerialConsoleWriteString("boot_flag.txt added!\r\n");
	}
	
	WifiHandlerSetState(WIFI_DOWNLOAD_INIT);

	vTaskDelay(1000);
	
	return pdFALSE;
}
