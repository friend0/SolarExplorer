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

#include "Inverter.h"

/**
 * Local Function Prototypes
 */

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


/**********************************************************
 _       __    _   _          __  _____   __    ___  _____
| |\/|  / /\  | | | |\ |     ( (`  | |   / /\  | |_)  | |
|_|  | /_/--\ |_| |_| \|     _)_)  |_|  /_/--\ |_| \  |_|

 **********************************************************/

void main(void)
{
	
	/**
	 * Background (BG) Loop
	 */
	for(;;)
	{
		/**
		 * State machine entry & exit point
		 * @TODO: I'll be operating two state machines, one macro, one for the inverter.
		 * The 'macro' FSM will monitor the overall state of the PV Inverter system, i.e. 
		 * startup, producingPower, fault, etc...
		 *
		 * The ISR for the inverter will be fuckin' with the 'Inverter' FSM {Vdc, zeroVdc, negVdc}
		 *
		 * The Solar Explorer project points to an 'AlphaStatePtr', where each alpha state points to the next alphaState, A,B,C
		 * Each alpha state then makes calls to the substates that constitute the actual machine.
		 */
		runInverter();
	}
}
 
/************************************************
 _       __    _   _          ____  _      ___  
| |\/|  / /\  | | | |\ |     | |_  | |\ | | | \ 
|_|  | /_/--\ |_| |_| \|     |_|__ |_| \| |_|_/ 

 *************************************************/



