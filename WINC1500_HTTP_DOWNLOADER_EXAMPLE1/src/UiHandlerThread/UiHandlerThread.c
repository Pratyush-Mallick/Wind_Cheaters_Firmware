 /**************************************************************************//**
* @file      UiHandlerThread.c
* @brief     File that contains the task code and supporting code for the UI Thread for ESE516 Spring (Online) Edition
* @author    WindCheaters
* @date      2020-04-09 

******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/
#include <errno.h>
#include "asf.h"
#include "UiHandlerThread/UiHandlerThread.h"
#include "SeesawDriver/Seesaw.h"
#include "SerialConsole.h"
#include "main.h"

/******************************************************************************
* Defines
******************************************************************************/
#define EVENT_READ_ONE       1
#define NO_EVENT_STATE       0xFF
#define BUTTON_PRESSED_MASK  0x03
#define BUTTON_PRESSED_VAL   0x03

/******************************************************************************
* Variables
******************************************************************************/
uiStateMachine_state uiState;
/******************************************************************************
* Forward Declarations
******************************************************************************/

/******************************************************************************
* Callback Functions
******************************************************************************/


/******************************************************************************
* Task Function
******************************************************************************/

/**************************************************************************//**
* @fn		void vUiHandlerTask( void *pvParameters )
* @brief	Handles the key press light up tasks.
* @details 	Reads the event from the seesaw buffer and toggles the state of 
            light associated to the key
                				
* @param[in]	Parameters passed when task is initialized. In this case we can ignore them!
* @return		Should not return! This is a task defining function.
* @note         
*****************************************************************************/
void vUiHandlerTask( void *pvParameters )
{
//Do initialization code here
SerialConsoleWriteString("UI Task Started!");
uiState = UI_STATE_HANDLE_BUTTONS;

uint8_t buffer;
uint8_t key_index;
uint8_t led_state[16] = {0};

//Here we start the loop for the UI State Machine
while(1)
{
	switch(uiState)
	{
		case(UI_STATE_HANDLE_BUTTONS):
		{
		//Do the handle buttons code here!
		/* NOTE:
		Do not call SeesawReadKeypad(uint8_t *buffer, uint8_t count) with the argument count being zero. 
		There seems to be a bug with ASF when this is called (i2c_master_read_packet_job).
		The following guard can be used: */

		SeesawReadKeypad(&buffer, EVENT_READ_ONE);
		key_index = NEO_TRELLIS_SEESAW_KEY(buffer >> 2);
		
		if (buffer != NO_EVENT_STATE) {
			if ((buffer & BUTTON_PRESSED_MASK) == BUTTON_PRESSED_VAL) {
				led_state[key_index] = ~led_state[key_index];
			}
		}
				
		SeesawSetLed(key_index, led_state[key_index], led_state[key_index], led_state[key_index]);
		SeesawOrderLedUpdate();
		
		break;
		}

		case(UI_STATE_IGNORE_PRESSES):
		{
		//Ignore me for now
			break;
		}

		case(UI_STATE_SHOW_MOVES):
		{
		//Ignore me as well
			break;
		}

		default: //In case of unforseen error, it is always good to sent state machine to an initial state.
			uiState = UI_STATE_HANDLE_BUTTONS;
		break;
	}

	//After execution, you can put a thread to sleep for some time.
	vTaskDelay(50);
}



}

/******************************************************************************
* Functions
******************************************************************************/