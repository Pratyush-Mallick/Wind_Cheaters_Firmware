/**************************************************************************//**
* @file      A4988_StepperMD.h
* @brief     Header file for A4988 motor driver
* @author    Tarush Sharma
* @date      2023-12-15

******************************************************************************/

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
* Includes
******************************************************************************/
#include <asf.h>
#include "AirVelocity/FS_3000.h"

/******************************************************************************
* Defines
******************************************************************************/
#define DIRECTION PIN_PA03
#define STEP      PIN_PA02
#define CLOCK_WISE      1
#define ANTI_CLOCK_WISE 0
#define DEBUG_BUTTON PIN_PA10
/******************************************************************************
* Structures and Enumerations
******************************************************************************/

/******************************************************************************
* Global Function Declaration
******************************************************************************/
// this function is to be called whenever the user sends the automate command
void AutomateTurbine(int degree);

#ifdef __cplusplus
}
#endif