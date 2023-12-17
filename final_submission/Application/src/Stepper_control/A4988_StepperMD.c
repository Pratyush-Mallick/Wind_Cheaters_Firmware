/**************************************************************************//**
* @file      A4988_StepperMD.c
* @brief     Source file for A4988 motor driver
* @author    Tarush Sharma
* @date      2023-12-15

******************************************************************************/


/******************************************************************************
* Includes
******************************************************************************/
#include "A4988_StepperMD.h"

/******************************************************************************
* Defines
******************************************************************************/

/******************************************************************************
* Variables
******************************************************************************/
static float AirVelocity;
float compare = -1;
int MaxSpeedDir = 0;
/******************************************************************************
* Forward Declarations
******************************************************************************/

/******************************************************************************
* Callback Functions
******************************************************************************/


/**************************************************************************//**
* @fn		int ExampleFuncionComment(int inputInt, void *pvParameters)
* @brief	An example function comment. Erase me!
* @details 	Write details of function here.
                				
* @param[in]	InputInt Use me to describe inputs to functions
* @param[out]	*pvParameters Use me to describe outputs of functions passed as arguments
* @return		Use me to explain the return of an argument.
* @note         
*****************************************************************************/
void AutomateTurbine(int degree)  {
    // get the number of steps 
    int steps = ((float) degree / 1.8);
    //set the direction and make sure the motor rotate in different directions each time  
    port_pin_set_output_level(DIRECTION, (bool)1);
    // rotate the motor 
    for(int i=0; i<steps;i++){
        // get data from air velocity sensor here and store in AirVelocity 

        AirVelocity = FS3000_readMetersPerSecond();

        if(AirVelocity > compare){
            compare = AirVelocity;
			MaxSpeedDir = i; 
        }

        // move the stepper motor the next angle 
        port_pin_set_output_level(STEP,(bool)1);
        delay_ms(50);
        port_pin_set_output_level(STEP,(bool)0);
        delay_ms(50);
    }
	
    int new_steps = steps - MaxSpeedDir;
    // position the turbine in the direction of maxi air velocity 
    //change the direction of the motor
    port_pin_set_output_level(DIRECTION,(bool)0); 
    for(int i=0; i < new_steps; i++){
        // move the stepper motor the next angle 
        port_pin_set_output_level(STEP,(bool)1);
        delay_ms(50);
        port_pin_set_output_level(STEP,(bool)0);
        delay_ms(50);
    }
}