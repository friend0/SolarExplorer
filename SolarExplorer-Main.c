/* 
* @Author: ryanarodriguez
* @Date:   2015-01-24 18:29:34
* @Last Modified by:   Ryan A. Rodriguez
* @Last Modified time: 2015-01-24 19:49:14
* @File			SolarExplorer-Main.C
* @Brief:	DCDC MPPT + Inverter Project for Solar Explorer R5  
*                                         ____                                                         
*              |         | ``..     ..'' |    ~.    |`````````, | |``````.                             
*              |_________|     ``.''     |____.'_   |'''|'''''  | |       |                            
*              |         |       |       |       ~. |    `.     | |       |                            
*              |         |       |       |_______.' |      `.   | |......'                                                                                                                                   
*                                  ____                                 ____                           
* | |..          | `.           .' |            |`````````, `````|````` |            |`````````,        
* | |  ``..      |   `.       .'   |______      |'''|'''''       |      |______      |'''|'''''         
* | |      ``..  |     `.   .'     |            |    `.          |      |            |    `.            
* | |          ``|       `.'       |___________ |      `.        |      |___________ |      `.          
*                                                                                                                                                                                                       
*                                 ..'''' |``````.  |`````````,                                         
*                              .''       |       | |'''''''''                                          
*                           ..'          |       | |                                                   
*                     ....''             |......'  |  
*
* This project is the development implemntation of Dr. Ricardo Sanfelice and Jun Chai's hybrid control
* algorithm for steering the trajectory of a power inverter output to, and holding it within a desired 
* tracking band. 
*
* @TeamMembers: Ryan Rodriguez, Ben Chainey, Lucas Adams
* @Email: ryarodri@ucsc.edu, bchainey@ucsc.edu, lhadams@ucsc.edu
*/

#include "SolarExplorer-Includes.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Add protoypes of functions being used in the project here 


/**
 * Included From 'SolarExplorer-DevInit', which contains functions for initializing:
 * -Device: initially disable wDog, diable interrupts, clear interrupt flags, initialize PieCntl, PieVect
 * 		-PieCntl: Disable all interrupt enables, clear all interuupt flags
 * 		-PieVect: 
 * -WDog
 * -PLL
 * -ISR-illegal
 * -CLA_init
 * -MemCopy
 */
void DeviceInit(void);
#ifdef FLASH		
	void InitFlash();
#endif
void MemCopy();

#ifdef FLASH
#pragma CODE_SECTION(Inv_ISR,"ramfuncs");
#endif
interrupt void Inv_ISR(void);
interrupt void spiTxFifoIsr(void);
interrupt void spiRxFifoIsr(void);


void SPI_init();

//-------------------------------- DPLIB --------------------------------------------
void PWM_1ch_UpDwnCntCompl_CNF(int16 n, int16 period, int16 mode, int16 phase);
void ADC_SOC_CNF(int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode);


///////////////////////////////
//State Machine Framework    //
///////////////////////////////


//////////////////////////////////
//	  State Machine Framework //
//////////////////////////////////


//@TODO: Include State machine implementation here


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// MAIN CODE - starts here
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main(void)
{
//=================================================================================
//	INITIALISATION - General
//=================================================================================

	// The DeviceInit() configures the clocks and pin mux registers 
	// The function is declared in {ProjectName}-DevInit_F2803/2x.c,
	// Please ensure/edit that all the desired components pin muxes 
	// are configured properly that clocks for the peripherals used
	// are enabled, for example the individual PWM clock must be enabled 
	// along with the Time Base Clock 

	DeviceInit();	// Device Life support & GPIO
	
//=================================================================================
//	BACKGROUND (BG) LOOP
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
	for(;;)
	{
		// State machine entry & exit point

	}
} //END MAIN CODE



