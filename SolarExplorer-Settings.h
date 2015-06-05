//----------------------------------------------------------------------------------
//	FILE:			{ProjectName}-Settings.h
//
//	Description:    This file contains the definitions for this project, and is 
//					linked to both {ProjectName}-Main.c and {ProjectName}-DPL-ISR.asm 
//					(where X is the project name).  
//
//	Type: 			Device Independent
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments © 2010
//----------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date	  | Description / Status
//----------------------------------------------------------------------------------
// 9 April 2010 - MB
//----------------------------------------------------------------------------------

#ifndef _PROJSETTINGS_H
#define _PROJSETTINGS_H

//**********************************************************************************
//  NOTE: WHEN CHANGING THIS FILE PLEASE REBUILD ALL
//**********************************************************************************

//==================================================================================
// Incremental Build options for System check-out
//==================================================================================
// BUILD 1 	 Closed Current Loop Inverter, internally generated sine
// BUILD 2   Closed Current Loop Inverter, internally generated sine with MPPT and DC bus Regulation 
// BUILD 3   Closed Current Loop Inverter, internally generated sine with MPPT and DC bus Regulation, PLL

#define INCR_BUILD 2
		
//==================================================================================
// System Settings
//----------------------------------------------------------------------------------
//Add any system specific setting below
#define HistorySize 8
#define PlotSize 64
//#define DLOG_SIZE   200

#define Sqrt2 _IQ24(1.4142136)
#define Isqrt2 _IQ24(0.70710678)	// 1/sqrt(2)
#define Gv_Inv _IQ15(1500)		// 3.3/0.083
#define KvInv _IQ15(39.759)				// 3.3/0.083

#define hallSenseScaling _IQ24(.185)
/**
 * HNIC: Set the voltage target for the Boost
 * For HIT Dev Board, we have a voltage divider with 2.7k/169k=0.01597633136
 * This gives us a maximum voltage of 206.55555556V
 * VdcRef is a ratio of this maximum.
 */

#define VdcRef	 _IQ24(0.363)	//will result in a voltage of about 75V


//#define GLOBAL_Q 22
#define GRID_FREQ	60

/**
 * Define values for global and forward controllers
 */
#define GLOBAL 2
#define FORWARD 1

#define TURNS_RATIO _IQ24(2.85)
#define AMP_VC	IQ24mpy(60.67)		//120VRMS(~169Vpk) will result from transformer with turns ratio of 2.85
#define AMP_IL  _IQ24(4.981)			//~5 Amps - determined form MATLAB for inductance of .05778 Henries, 173 microFarads

/**
 * Filter Values for HybridMode
 */
#define RES _IQ24(0.5) 			//In Ohms
#define IND _IQ24(0.5)			//In Henries, probably best to express in IQ30, 29, or 28?
#define CAP _IQ24(0.5)			//In Farads, perhaps also best expressed in same IQ as 'L'
#define FREQ 60
#define ERR _IQ24(0.001)		//Dimension-agnostic varibale used to determine equality of two floating point numbers. @TODO: Determine best Q format.
#define EPSILON _IQ24(0.001)	//Used to determine membership in M1 or M2 - used to avoid fast swithcing in certain regions of the Power Plane

/**
 * Tracking band constants - hard coded for now
 */
#define CMID _IQ24(0.001)
#define CIN _IQ24(0.001)
#define COUT _IQ24(0.001)

// inverter states
#define		INV_CAL					0
#define		INV_OFF					1
#define 	INV_SOFT_START			2
#define		INV_ON					3

// main states
#define		PWR_UP_INIT					0
#define		INV_EXEC					8

#define	MIN_MOD_INDEX	16000		// 0.488 -> 16000
#define	MAX_MOD_INDEX	50000		// 1.53 ->  50000


/**
 * @todo: update inverter set voltage
 */
// PWM1A & 1B related & PWM2A & 2B related - Inverter mode
#define	SINE_GEN_PRD			1500			// 60M / 20480 = 2929 in Asymmetric mode
#define	INVERTER_DEADBAND		20				//  2 uSec deadband
#define	MAX_INV_PWM_VALUE		SINE_GEN_PRD-100 // >60
#define	MIN_INV_PWM_VALUE		60				// > INVERTER_DEADBAND/2 AS SYMMETRIC MODE
#define	INV_SET_VOLTAGE				_IQ15(0.5)		// set inverter output voltage = 220.0V
#define	TEMP_LOOKUP_TBL_SIZE     	69
#define	SINE_TBL_SIZE 				512

//@todo: get rid og all references to the light sensor
#define LIGHTSENSOR_MAX_INV  _IQ13(0.41666)	// 1/LightSensorMaxOutput = 1/2.4

//==================================================================================
// Interrupt Framework options
//==================================================================================

#define EPWMn_DPL_ISR	1	// for EPWM triggered ISR set as 1
#define ADC_DPL_ISR	    0	// for ADC INT 1 triggered ISR set as 1 
#define CLAn_DPL_ISR	0	// for CLA Task n Triggered ISR set as 1

//----------------------------------------------------------------------------------
// If EPWMn_DPL_ISR = 1, then choose which module
//----------------------------------------------------------------------------------
#define EPWM1			0	// EPWM1 provides ISR trigger
#define EPWM2			0 	// EPWM2 provides ISR trigger
#define EPWM3			1	// EPWM3 provides ISR trigger
#define EPWM4			0	// EPWM4 provides ISR trigger
#define EPWM5			0	// EPWM5 provides ISR trigger
#define EPWM6			0	// EPWM6 provides ISR trigger

//----------------------------------------------------------------------------------
// If EPWM is triggering the CLAn_DPL__ISR = 1, then choose which Task is being triggered
// note the following code would clear the PWM interrupt,
// if an ADC interrupt is used to trigger the CLA leave the following all zeros
//----------------------------------------------------------------------------------
#define CLATASK1_EPWM1		0	// EPWM1 Interrupt provides CLA Task 1 trigger
#define CLATASK2_EPWM2		0	// EPWM2 Interrupt provides CLA Task 2 trigger
#define CLATASK3_EPWM3		1	// EPWM3 Interrupt provides CLA Task 3 trigger
#define CLATASK4_EPWM4		0	// EPWM4 Interrupt provides CLA Task 4 trigger
#define CLATASK5_EPWM5		0	// EPWM5 Interrupt provides CLA Task 5 trigger
#define CLATASK6_EPWM6		0	// EPWM6 Interrupt provides CLA Task 6 trigger
#define CLATASK7_EPWM7		0	// EPWM7 Interrupt provides CLA Task 7 trigger
// Task 8 is reserved for software based initialization when using DPlib

#endif //_PROJSETTINGS_H

