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

/**
 * SolarExplorer-Includes.h gives us:
 * IQMath.lib
 * PeripheralHeaderIncludes, which gives us access to all peripheral headers (ADC, PWM, i2C, etc...)
 * SolarExplorer-Settings - where we set incremental builds and choose which ePWM triggers ISR, deadband, sineGen and lighting settings
 * DPlib - 2p2z, 3p3z, and some ADCtrig defines
 * PWMDac
 *
 */
#include "SolarExplorer-Includes.h"

<<<<<<< HEAD
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
=======
 /*******************************
  * Function Prototypes
  *******************************/


>>>>>>> 9b437274f306d6a6669de3c4b121eabaa0babcb0
void DeviceInit(void);
#ifdef FLASH
	/**
 * Flash Control Register Intis
 */
	void InitFlash();
#endif
/**
 * Copy contents of memory, called iwth on arguments - why?
 */
void MemCopy();

#ifdef FLASH
#pragma CODE_SECTION(Inv_ISR,"ramfuncs");
#endif
interrupt void Inv_ISR(void);
interrupt void spiTxFifoIsr(void);
interrupt void spiRxFifoIsr(void);

/**
 * Hardware init of SPI function - works in tandem with Rx/Tx FIFO ISR declarations
 */
void SPI_init();


/**
 * DPLIB Configs
 */

/**
 * @brief PWM configuration
 *
 * Single (A Output) channel PWM configuration function, configures the PWM channel in
 * UP count mode.
 * 			
 * @param n      Target ePWM module, 1, 2, ... , 16
 * @param period PWM period in sysclks
 * @param mode   Master/Slave mode: 0 = Slave
 *               					1 = Master
 * @param phase  Phase offset from upstream master in sysclks, applicable only if more=0, i.e
 *               slave.
 */
void PWM_1ch_UpDwnCntCompl_CNF(int16 n, int16 period, int16 mode, int16 phase);

/**
 * @brief ADC 'Start of Conversion' config
 *
 * ADC configuration to support up to 16 conversions on 
 * Start of Conversion(SOC) based ADCs (type 3) found on F2802x and F3803x devices.  
 * Independent selection of Channel, Trigger and acquisition window using ChSel[],TrigSel[] and ACQPS[].
 * 	
 * @param ChSel    Channel Selection made via a channel # array passed as an argument
 * @param Trigsel  Source for triggering conversion of a channel, 
 *                 selction made via a trigger # array passed as an arguments
 * @param ACQPS    AcqWidth is the S/H aperture in #ADCCLKS, #array passed as arguments	
 * @param IntChSel Channel number that would trigger an ADC interrupt 1 on completion (EOC)
 * @param mode     Operating Mode:	0 = Start/Stop mode, needs a trigger event
 *                 					1 = Continuous Mode, no trigger needed
*                 				  	2 = CLA Mode, start stop mode with auto clr INT flag
 */
void ADC_SOC_CNF(int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode);

/**************************************************
 * State Machine Framework
 * @TODO: Include State Machine Implemention Here
 **************************************************/

<<<<<<< HEAD
/**
 * State Machine Framework
 * @TODO: Include State machine implementation here
 */


//FSM Here ->


/**
 * VARIABLE DECLARATIONS - GENERAL
 */


/**
 * Virtual Timers
 * @TODO figure out how these are being incremented
 */
int16	VTimer0[4];					// Virtual Timers slaved off CPU Timer 0
int16	VTimer1[4];					// Virtual Timers slaved off CPU Timer 1
int16	VTimer2[4];					// Virtual Timers slaved off CPU Timer 2


/**
 * Set up variables for specifying address locations to pass to MemCopy()
 */

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
// Used for copying CLA code from load location to RUN location 
extern Uint16 Cla1funcsLoadStart, Cla1funcsLoadEnd, Cla1funcsRunStart;


/**
 * ADC configuration vars
 */
int 	ChSel[16] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int		TrigSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int     ACQPS[16] =   {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};



/**********************************************************
 _       __    _   _          __  _____   __    ___  _____ 
| |\/|  / /\  | | | |\ |     ( (`  | |   / /\  | |_)  | |  
|_|  | /_/--\ |_| |_| \|     _)_)  |_|  /_/--\ |_| \  |_|         

 **********************************************************/

void main(void)
{

/**
 * Device and Variable Inits
 */
=======
 /*******************************
  * General Variable Declarations
  *******************************/

/************************
 * Main Code Starts Here
 *************************/

void main(void)
{

	/**
	 * Initializations
	 */
>>>>>>> 9b437274f306d6a6669de3c4b121eabaa0babcb0

	// The DeviceInit() configures the clocks and pin mux registers 
	// The function is declared in {ProjectName}-DevInit_F2803/2x.c,
	// Please ensure/edit that all the desired components pin muxes 
	// are configured properly that clocks for the peripherals used
	// are enabled, for example the individual PWM clock must be enabled 
	// along with the Time Base Clock 

	DeviceInit();	// Device Life support & GPIO


	/**
	 * Init State variables here
	 */
	
<<<<<<< HEAD


	/**
	 * Init Virtual Timers 0-2 here
	 */
	
/**
 * Background (BG) Loop
 */
=======
	/**
	 * Background Loop
	 */
>>>>>>> 9b437274f306d6a6669de3c4b121eabaa0babcb0

//--------------------------------- FRAMEWORK -------------------------------------
	for(;;)
	{
		// State machine entry & exit point

	}
}
 
/************************************************
 _       __    _   _          ____  _      ___  
| |\/|  / /\  | | | |\ |     | |_  | |\ | | | \ 
|_|  | /_/--\ |_| |_| \|     |_|__ |_| \| |_|_/ 

 *************************************************/


/**
 * @brief SPI init function
 *
 * Setting the registers to get SPI serial comm up
 * 
 * @TODO find out what SPI is being used for
 * @TODO consider the impact of moving this function to DevInit file
 */
void SPI_init()
{
	// Initialize SPI FIFO registers
   SpibRegs.SPICCR.bit.SPISWRESET=0; // Reset SPI
   SpibRegs.SPICCR.bit.SPILBK=0;	// No Loopback mode
   SpibRegs.SPICCR.bit.SPICHAR=0xF;	// 16 bit SPI word
	
   SpibRegs.SPICTL.bit.OVERRUNINTENA=0;
   SpibRegs.SPICTL.bit.CLK_PHASE=0;
   SpibRegs.SPICTL.bit.MASTER_SLAVE=1; // configure as a master
   SpibRegs.SPICTL.bit.SPIINTENA=0;
   SpibRegs.SPICTL.bit.TALK=1;
   
   SpibRegs.SPISTS.all=0x0000;
   
   SpibRegs.SPIBRR=0x0063;           // Baud rate
   
   SpibRegs.SPIRXBUF=0x0;
   SpibRegs.SPITXBUF=0x0;
   
   SpibRegs.SPIFFTX.all=0xC022;      // Enable FIFO's, set TX FIFO level to 4
   SpibRegs.SPIFFRX.all=0x0022;      // Set RX FIFO level to 4
   
   SpibRegs.SPIFFCT.all=0x00;
   SpibRegs.SPIPRI.all=0x0010;

   SpibRegs.SPICCR.bit.SPISWRESET=1;  // Enable SPI

   SpibRegs.SPIFFTX.bit.TXFIFO=1;
   SpibRegs.SPIFFRX.bit.RXFIFORESET=1;
   
   SpibRegs.SPIPRI.bit.FREE=1; 
}



