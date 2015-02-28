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
#include "fsm.h"
#include "inverterFSM.h"
#include "Structs.h"
#include "InverterVariables.h"
#include "ADC_Configs.h"
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


/****************
 * DPLIB Configs
 ****************/

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


/**
 * State Machine Framework
 * Declaration of states occurs in hBridgeFSM.h and InverterFSM.h
 *
 * @TODO: Refactor hBridge to HBridge
 * @TODO: Include State machine implementation here
 */

	//Declare the variable 'inverter' to be of the type 'Inverter', where 'Inverter' is the class
	//wrapping the FSM object. Note that the FSM object is a pointer to the current state. 
	Inverter inverter;
	hBridge hBridge;

	//Take the instance of the 'Inverter' class inverter, get the FSM it contains, and point it to an initialization state
	InverterCtor(&inverter);
	hBridgeCtor(&hBridge);

	//Derference the Fsm object pointed to by inverter, initialize it with an initial event
	//@TODO: need to figure the best initial event to send the machine
	FsmInit((Fsm *)&inverter, 0);
	FsmInit((Fsm *)&hBridge, 0);


	//Now the inverter is initialized, and pointing to an initial state function
	//From here, we can call the transition function to bring the FSM to life. 
	//@TODO write a wrapper for the trasition function implementing the jump set
	//Call the jump set only when we are in D, the jump set. 

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

/**
 * EPWM Struct and COMP_REGS struct declared in Structs.h
 */

/**
 * ADC and 2P2Z controller 'net macros' are declared in DPLib_Support.h
 * along with DPLib variables
 */

/**
 * Inverter Variables included in 'Invertervariables.h'
 */

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

	// The DeviceInit() configures the clocks and pin mux registers 
	// The function is declared in {ProjectName}-DevInit_F2803/2x.c,
	// Please ensure/edit that all the desired components pin muxes 
	// are configured properly that clocks for the peripherals used
	// are enabled, for example the individual PWM clock must be enabled 
	// along with the Time Base Clock 

	DeviceInit();	// Device Life support & GPIO


	// Only used if running from FLASH
	// Note that the variable FLASH is defined by the compiler with -d FLASH

	#ifdef FLASH		
	// Copy time critical code and Flash setup code to RAM
	// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files. 
		MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM
		InitFlash();	// Call the flash wrapper init function
	#endif //(FLASH)


	/**
	 * Timing sync for background loops
	 * Timer period definitions found in PeripheralHeaderIncludes.h
	 */
	CpuTimer0Regs.PRD.all =  mSec5;		// A tasks
	CpuTimer1Regs.PRD.all =  mSec50;	// B tasks
	CpuTimer2Regs.PRD.all =  mSec1000;	// C tasks

	/**
	 * Tasks State-machine init
	 * Need to get rid of these, or find out how to make only the DC-DC stage run with this
	 * @TODO reconcile this with new implementation
	 */
	Alpha_State_Ptr = &A0;
	A_Task_Ptr = &A1;
	B_Task_Ptr = &B1;
	C_Task_Ptr = &C1;

	VTimer0[0] = 0;	
	VTimer1[0] = 0;
	VTimer2[0] = 0;
	LedBlinkCnt = 5;
	LedBlinkCnt2 = 5;


	/**
	 * Init State variables here
	 */
	

	/**
	 * User Initialization
	 */
	userInitialization();
	/**
	 * PWM Inits
	 * @TODO: Go a bit deeper here and figure out what's actually being done:
	 * Calcs, Library calls, etc... Also, make sure to update the comments on everything.
	 */
	
	/**
	 * Configure PWM3 for 100Khz switching Frequency
	 * I think this is for the Boost Loop
	 */
	PWM_1ch_UpDwnCntCompl_CNF(3, 600,0,30); 

	/**
	 * Solar_PWM_Inv_1ph_unipolar_CNF(1, 1500, 20, 20);
	 * I think this is for the Inverter Loop
	 */
	PWM_1phInv_unipolar_CNF(1,1500,20,20);	

	/**
	 * ADC Varible Inits
	 */
	ADC_Init();

	/**
	 * Implements phase synchronization to avoid ADC and ISR conflicts
	 *
	 * @TODO: find out more about how this works, make a timing diagram
	 */
	pwmPhaseConfig();

	/**
	 * Digital Power CLA(DP) library initialization
	 */
	DPL_Init();

	/**
	 * Connect structures to variables used in code
	 */
	netConnections();

	/**
	 * Set the coefficients used in 2P2Z compensators
	 */
	controllerCoefficientInits();

	/**
	 * Net Variable Inits - set nets to known initial conditions
	 */
	netVariableInits();

	/**
	 * Initialize Signal Generator module for Sine wave generation
	 */
	sigGenInit();

	/**
	 * Initialize the DATALOG module
	 */
	dataLoggingInits();


	// Initialize PWMDAC module
	pwmdac1.PeriodMax = 500;   // 3000->10kHz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
	pwmdac1.PwmDacInPointer0 = &PwmDacCh1;
	pwmdac1.PwmDacInPointer1 = &PwmDacCh2;
	pwmdac1.PwmDacInPointer2 = &PwmDacCh3;
	pwmdac1.PwmDacInPointer3 = &PwmDacCh4;

	PWMDAC_INIT_MACRO(pwmdac1)

	/**
	 * Software PLL for grid tie 
	 * @TODO: figure out if I should comment this out, or if leaving it will have adverse effects
	 * since we're not planning on grid-tie for the moment.
	 */
	SPLL_1ph_init(60,_IQ21(0.00005),&spll1);

	PanelBoostConnect=0;
	MPPT_ENABLE = 0;
	Run_MPPT=0;
	MPPT_slew=0;
	VpvRef_MPPT=_IQ24(0.0);
	
	CloseVloopInv = 0;
 	CloseIloopInv = 0;
	ClearInvTrip=0;
	InvModIndex = 0;
	ScaleFactor = _IQ15(1.0);
	inv_Iset = _IQ24(0.0);
	InvSine=0;
	inv_meas_cur_diff_inst=0;
	VrmsReal=0;
	
	
	
	UpdateCoef=0;
	timer1=0;
	Offset_Volt=_IQ21(0.5); // the input sinusoid is offset with 1.65 V
	
	PVInverterState=0;
	Gui_MPPTEnable=0;
	Gui_InvStart=0;
	Gui_LightRatio_Avg=0;
	Gui_InvStop=0;
	Gui_PanelPower_Theoretical=_IQ9(0.0);
	
	// keep the PLL in reset by default
	ResetPLL=1;
	
	Gui_LightCommand=_IQ13(0.0);
	Gui_LightCommand_Prev=_IQ13(0.0);	
	/**
	 * Init Virtual Timers 0-2 here
	 */


	#ifdef FLASH
	// Initiate Commros 
		InitCommros();
	#endif 	

	/**
	 * Finally, initialize the ISRs, clear flags, setup PIE,
	 */
	ISR_Init();		


	/**
	 * Hardware init of SPI function - works in tandem with Rx/Tx FIFO ISR declarations
	 */
	SPI_init();
	
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

		#ifdef FLASH
				ServiceRoutine(&commros); 
				Datalogger(&commros.m_datalogger,0);
		#endif
	}
}
 
/************************************************
 _       __    _   _          ____  _      ___  
| |\/|  / /\  | | | |\ |     | |_  | |\ | | | \ 
|_|  | /_/--\ |_| |_| \|     |_|__ |_| \| |_|_/ 

 *************************************************/

void pwmPhaseConfig(void){

	EPwm1Regs.TBCTL.bit.PHSEN   = TB_DISABLE;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	
	EPwm2Regs.TBCTL.bit.PHSEN   = TB_ENABLE;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm2Regs.TBPHS.half.TBPHS = 2;
	
	EPwm3Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
	EPwm3Regs.TBCTL.bit.PHSEN=TB_ENABLE; 
	EPwm3Regs.TBCTL.bit.PHSDIR=TB_UP;
	
	EPwm3Regs.TBPHS.half.TBPHS=4;
}

void userInitialization(void){
	 /**
	  * User Initialization and Variable Definitions
	  * @TODO: Commment all of these variables with brief description of usages
	  */

	for(i=0;i<HistorySize;i++)
	{
		Hist_Vpv[i]=0;
		Hist_Ipv[i]=0;
		Hist_Light[i]=0;
		Hist_Vboost[i]=0;
	}

	HistPtr=0;

	K_Vpv=17050;							// Q15
	K_Ipv=17329;							// Q15
	K_Vboost=20463;							// Q15
	iK_Vboost=26236;						// Q15
	iK_Ipv=30981;	
	K_Light=27034;							//Q15

	Gui_Vpv=0;
	Gui_Ipv=0;
	Gui_Vboost=0;

	TransmitData=0;
	sdata[0]=0;     // Send data buffer
	sdata[1]=1;
}
/**
 * @brief Lib Module connection to nets
 * Connects the PWM, EMAVG and 2P2Z structures to variables used in code
 */
void netConnections(void){

	// Connect the PWM Driver input to an input variable, Open Loop System
	PWMDRV_1ch_UpDwnCntCompl_Duty3 = &Duty3A; //&Duty3A; //&Duty3A_fixed; 
	
	ADCDRV_1ch_Rlt1=&IboostswRead;
	ADCDRV_1ch_Rlt5=&VboostRead;
	ADCDRV_1ch_Rlt6=&IpvRead;
	ADCDRV_1ch_Rlt7=&VpvRead;
	
	// MATH_EMAVG1 block connections
	MATH_EMAVG_In1=&IpvRead;
	MATH_EMAVG_Out1=&IpvRead_EMAVG;
	MATH_EMAVG_Multiplier1=_IQ30(0.001); // a 1000 point moving average filter
	
	// MATH_EMAVG2 block connections
	MATH_EMAVG_In2=&VpvRead;
	MATH_EMAVG_Out2=&VpvRead_EMAVG;
	MATH_EMAVG_Multiplier2=_IQ30(0.001); // a 1000 point moving average filter
	
	
	//======================================================================================
	//connect the 2P2Z connections, for the inner current Loop
	CNTL_2P2Z_Ref2 = &IboostSwRef; 
	CNTL_2P2Z_Out2 = &Duty3A;
	CNTL_2P2Z_Fdbk2= &IboostswRead; 
	CNTL_2P2Z_Coef2 = &CNTL_2P2Z_CoefStruct2.b2;
	
	//connect the 2P2Z connections, for the outer Voltage Loop
	CNTL_2P2Z_Ref1 = &VpvRead;
	CNTL_2P2Z_Fdbk1 = &VpvRef;
	CNTL_2P2Z_Out1 = &IboostSwRef;  
	CNTL_2P2Z_Coef1 = &CNTL_2P2Z_CoefStruct1.b2;

}

void controllerCoefficientInits(void){

	//connect the 2P2Z connections, for the inner current Loop
	CNTL_2P2Z_Ref2 = &IboostSwRef; 
	CNTL_2P2Z_Out2 = &Duty3A;
	CNTL_2P2Z_Fdbk2= &IboostswRead; 
	CNTL_2P2Z_Coef2 = &CNTL_2P2Z_CoefStruct2.b2;
	
	//connect the 2P2Z connections, for the outer Voltage Loop
	CNTL_2P2Z_Ref1 = &VpvRead;
	CNTL_2P2Z_Fdbk1 = &VpvRef;
	CNTL_2P2Z_Out1 = &IboostSwRef;  
	CNTL_2P2Z_Coef1 = &CNTL_2P2Z_CoefStruct1.b2;
	
	// Coefficients for Outer Voltage Loop
	// PID coefficients & Clamping - Current loop (Q26)
	Dmax_V  = _IQ24(0.9);	
	Pgain_V = _IQ26(0.015);	  
	Igain_V = _IQ26(0.00005);  
	Dgain_V =_IQ26(0.0);  
				
	// Coefficient init	--- Coeeficient values in Q26
	CNTL_2P2Z_CoefStruct1.b2   =Dgain_V;                            // B2
    CNTL_2P2Z_CoefStruct1.b1   =(Igain_V-Pgain_V-Dgain_V-Dgain_V);  // B1
    CNTL_2P2Z_CoefStruct1.b0   =(Pgain_V + Igain_V + Dgain_V);      // B0
    CNTL_2P2Z_CoefStruct1.a2   =0.0;                              	// A2 = 0
    CNTL_2P2Z_CoefStruct1.a1   =_IQ26(1.0);                       	// A1 = 1 
    CNTL_2P2Z_CoefStruct1.max  =Dmax_V;					  		  	//Clamp Hi 
    CNTL_2P2Z_CoefStruct1.min  =_IQ24(0.0); 					  	//Clamp Min   
    
	// Coefficients for Inner Current Loop
	// PID coefficients & Clamping - Current loop (Q26)
	Dmax_I  = _IQ24(0.9);
	Pgain_I = _IQ26(0.015);	  
	Igain_I = _IQ26(0.00005);  
	Dgain_I =_IQ26(0.0); 
	
	// Coefficient init	--- Coeeficient values in Q26
	CNTL_2P2Z_CoefStruct2.b2   =Dgain_I;                            // B2
    CNTL_2P2Z_CoefStruct2.b1   =(Igain_I-Pgain_I-Dgain_I-Dgain_I);  // B1
    CNTL_2P2Z_CoefStruct2.b0   =(Pgain_I + Igain_I + Dgain_I);      // B0
    CNTL_2P2Z_CoefStruct2.a2   =0.0;                              	// A2 = 0
    CNTL_2P2Z_CoefStruct2.a1   =_IQ26(1.0);                       	// A1 = 1 
    CNTL_2P2Z_CoefStruct2.max  =Dmax_I;					  		  	//Clamp Hi 
    CNTL_2P2Z_CoefStruct2.min  =_IQ24(0.0); 					  	//Clamp Min   
}

void netVariableInits(){

	// Initialize the net variables
	Duty3A =_IQ24(0.0);
	VboostRead=_IQ24(0.0);
	IboostswRead=_IQ24(0.0);
	VpvRef=_IQ24(0.9);	// to increase current, we need to reduce VpvRef, thus initailize it with a high value. 
	IboostSwRef=_IQ24(0.0);
	Duty3A_fixed=_IQ24(0.2);	
	VpvRead_EMAVG=_IQ24(0.0);
	IpvRead_EMAVG=_IQ24(0.0); 
  
  	// MPPT testing related code 	
  	// mppt incc
	mppt_incc1.IpvH = _IQ(0.0001);
	mppt_incc1.IpvL = _IQ(-0.0001);
	mppt_incc1.VpvH = _IQ(0.0001);
	mppt_incc1.VpvL = _IQ(-0.0001);
	mppt_incc1.MaxVolt = _IQ(0.9);
	mppt_incc1.MinVolt = _IQ(0.0);
	mppt_incc1.Stepsize = _IQ(0.01);
	mppt_incc1.mppt_first=1;
	mppt_incc1.mppt_enable=0;
	
	//mppt pno
	mppt_pno1.DeltaPmin = _IQ(0.00001);
	mppt_pno1.MaxVolt = _IQ(0.9);
	mppt_pno1.MinVolt = _IQ(0.0);
	mppt_pno1.Stepsize = _IQ(0.01);

}

void sigGenInit(){

  	/* Signal Generator module initialisation           */ 
	sgen.offset=0;
    sgen.gain=0x7fff;       /* gain=1 in Q15                              */
	sgen.freq=0x14F8CF92; 		/* freq = (Required Freq/Max Freq)*2^31 */
								/* = (50/305.17)*2^31 = 0x14f8cf92 */
	sgen.step_max=0x3E7FB26;	/* Max Freq= (step_max * sampling freq)/2^32 */
								/* =(0x3E7FB26*20k)/2^32 = 305.17 */
	sgen.phase=0x80000000;      /* Phase= (required Phase)/180 in Q31 format  */    
	       		                /*      =  (+90/180) in Q31 = 8000h          */
      		
    //sine analyzer initialization
	sine_mainsV.Vin=0;
    sine_mainsV.SampleFreq=_IQ15(20000.0);
    sine_mainsV.Threshold=_IQ15(0.0);

}

void dataLoggingInits(void){
	
	// Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
    dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = _IQ15(0.08);
    dlog.size = 0x64;
    dlog.prescalar = 10;
    dlog.init(&dlog);	

}

/**
 * @brief INTERRUPTS & ISR INITIALIZATION (best to run this section after other initialization)
 *
 * @TODO: document this! look into how the pie operates in the C2000 family
 * @TODO: Why are EDIS and EALLOW being called so many times in between configs?
 */
void ISR_Init(void){



// Set up C28x Interrupt

//Also Set the appropriate # define's in the {ProjectName}-Settings.h 
//to enable interrupt management in the ISR
	EALLOW;
    PieVectTable.EPWM3_INT = &DPL_ISR;      	// DP Lib interrupt for boost closed loop control (100Khz) 
    PieVectTable.ADCINT1 = &Inv_ISR;		// Inverter Control Interrupt (20Khz)
     
   	PieCtrlRegs.PIEIER3.bit.INTx3 = 1;      	// PIE level enable, Grp3 / Int3
   	PieCtrlRegs.PIEIER1.bit.INTx1 	= 1;		// Enable ADCINT1 in PIE group 1
   	PieCtrlRegs.PIEIER6.bit.INTx1 	= 1;		// Enable INT1 in PIE group 6
   	PieCtrlRegs.PIEIER6.bit.INTx2 	= 1;		// Enable INT1 in PIE group 6
   	
   	EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;  	// INT on PRD event
   	EPwm3Regs.ETSEL.bit.INTEN = 1;              // Enable INT
    EPwm3Regs.ETPS.bit.INTPRD = ET_2ND;         // Generate INT on every event
    
    //ADC interrupt already enabled by ADC SOC Cnf

    IER |= M_INT3;                          	// Enable CPU INT3 connected to EPWM1-6 INTs:
    IER |= M_INT1;								// Enable CPU INT1 for ADCINT1,ADCINT2,ADCINT9,TripZone 
    
    EINT;                                   	// Enable Global interrupt INTM
    ERTM;                                   	// Enable Global realtime interrupt DBGM
	EDIS;      
	
// Protections

	EALLOW;
	
	Comp1Regs.COMPCTL.bit.COMPDACEN  =0x1;
	Comp1Regs.COMPCTL.bit.SYNCSEL    =0x0;	// asynchronous version of the COMP signal is passed to the EPWM/GPIO module
	Comp1Regs.COMPCTL.bit.CMPINV     =0x0;	// Output of the comparator is passed directly 
	Comp1Regs.COMPCTL.bit.COMPSOURCE =0x0;	// inverting input of the comparator is connected to the internal DAC
	Comp1Regs.DACVAL.bit.DACVAL		 =950;	// set DAC input to peak trip point   
	
	AdcRegs.COMPHYSTCTL.bit.COMP1_HYST_DISABLE = 0x1; 
	
// Cycle by cycle trip for overvoltage protection
	/**
	 * DCTRIPSEL Register: Digital Trip Compare Select
	 * 
	 * DCAHCOMPSEL: Digital Compare A High Input Select
	 * 		Defines the source for the DCAH input. The TZ signals, when used as trip signals, 
	 * 		are treated as normal inputs and can be defined as active high or active low.
	 * 		Here, DC_COMP1OUT = 0x8 which will use COMP1OUT as input
	 *
	 * TZDCSEL: Trip Zone Digital Compare Event Select Register
	 * 		DCAEVT2: Digital Compare Output A Event 2 Selection, 
	 * 		in this case is set to 'TZ_DCAH_HI' where TZ_DCAH_HI = 0x2.
	 * 		This option corresponds to DCAH = high, DCAL = don't care,
	 * 		where these are Digital Compare A high and Low resprectively. 
	 * 		These indicate when a trip event has occured.
	 */
	EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL=DC_COMP1OUT;
	EPwm3Regs.TZDCSEL.bit.DCAEVT2=TZ_DCAH_HI;
	
	EPwm3Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;
	EPwm3Regs.DCACTL.bit.EVT2FRCSYNCSEL=DC_EVT_ASYNC;
	 
	EPwm3Regs.TZSEL.bit.DCAEVT2=0x1;
	
	EDIS;


	EALLOW;
	
// Cycle by cycle interrupt for CPU halt trip

/**
 * TZSEL Register: Trip Zone Select:
 * 
 * TZCTL Register: Trip Zone Controller:
 *
 * TZCLR Register: Trip Zone Clear Register:
 *
 * TZFRC Register: Trip Zone Force Register:
 */
	EPwm3Regs.TZSEL.bit.CBC6=0x1;
	EPwm1Regs.TZSEL.bit.CBC6=0x1;
	EPwm2Regs.TZSEL.bit.CBC6=0x1;
	EPwm1Regs.TZSEL.bit.OSHT1=0x1;
	EPwm2Regs.TZSEL.bit.OSHT1=0x1;
	
// What do we want the OST/CBC events to do?
// TZA events can force EPWMxA
// TZB events can force EPWMxB
	EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
	EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
	
	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
	EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
	
	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO; // EPWMxA will go low 
	EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO; // EPWMxB will go low
	
	EDIS;
	
	//clear any spurious trips 
	EPwm3Regs.TZCLR.bit.OST=1;
	EPwm3Regs.TZCLR.bit.DCAEVT1=1;
	
	// software force the trip of inverter to disable the inverter completely
	EALLOW;
	EPwm1Regs.TZFRC.bit.OST=0x1;
	EPwm2Regs.TZFRC.bit.OST=0x1;
	EDIS;
}

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



