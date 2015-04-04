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
#include "stdbool.h"
#include "inverterVariables.h"

#include "fsm.h"
#include "panelFSM.h"
#include "mpptFSM.h"
#include "hBridgeFSM.h"
#include "inverterFSM.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Add protoypes of functions being used in the project here 

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

StateVariable state;

//-------------------------------- DPLIB --------------------------------------------
void PWM_1ch_UpDwnCntCompl_CNF(int16 n, int16 period, int16 mode, int16 phase);
void ADC_SOC_CNF(int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode);

// -------------------------------- FRAMEWORK --------------------------------------
// State Machine function prototypes
//----------------------------------------------------------------------------------

//----------------------------------------------------------------------------------

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// -------------------------------- FRAMEWORK --------------------------------------

int16	VTimer0[4];					// Virtual Timers slaved off CPU Timer 0
int16	VTimer1[4];					// Virtual Timers slaved off CPU Timer 1
int16	VTimer2[4];					// Virtual Timers slaved off CPU Timer 2

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
// Used for copying CLA code from load location to RUN location 
extern Uint16 Cla1funcsLoadStart, Cla1funcsLoadEnd, Cla1funcsRunStart;

// Used for ADC Configuration 
int 	ChSel[16] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int		TrigSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int     ACQPS[16] =   {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

//---------------------L------------------------------------------------------
// Used to indirectly access all EPWM modules

volatile struct EPWM_REGS *ePWM[] = 
 				  { &EPwm1Regs,			//intentional: (ePWM[0] not used)
				  	&EPwm1Regs,
					&EPwm2Regs,
					&EPwm3Regs,
					&EPwm4Regs,
					#if (!DSP2802x_DEVICE_H)
					&EPwm5Regs,
					&EPwm6Regs,
					#if (DSP2803x_DEVICE_H || DSP2804x_DEVICE_H)
					&EPwm7Regs,
					#if (DSP2804x_DEVICE_H)
					&EPwm8Regs
					#endif 
					#endif 
					#endif
				  };

// Used to indirectly access all Comparator modules
volatile struct COMP_REGS *Comp[] = 
 				  { &Comp1Regs,			//intentional: (Comp[0] not used)
					&Comp1Regs,				  
					&Comp2Regs,
					#if (DSP2803x_DEVICE_H)
					&Comp3Regs
					#endif 
				  };

// ---------------------------------- USER -----------------------------------------
// ---------------------------- DPLIB Net Pointers ---------------------------------
// Declare net pointers that are used to connect the DP Lib Macros  here 

// ADCDRV_1ch 
extern volatile long *ADCDRV_1ch_Rlt1;	//instance #1
extern volatile long *ADCDRV_1ch_Rlt5; 	// instance #5
extern volatile long *ADCDRV_1ch_Rlt6; 	// instance #6
extern volatile long *ADCDRV_1ch_Rlt7; 	// instance #7

// PWMDRV_1ch
//extern volatile long *PWMDRV_1ch_Duty3;	// instance #3, EPWM3
extern volatile long *PWMDRV_1ch_UpDwnCntCompl_Duty3; 
extern volatile long PWMDRV_1ch_UpDwnCntCompl_Period3; 

// CONTROL_2P2Z
extern volatile long *CNTL_2P2Z_Ref1;	// instance #1
extern volatile long *CNTL_2P2Z_Out1;	// instance #1
extern volatile long *CNTL_2P2Z_Fdbk1;	// instance #1
extern volatile long *CNTL_2P2Z_Coef1; 	// instance #1
extern volatile long CNTL_2P2Z_DBUFF1[5];

// CONTROL_2P2Z
extern volatile long *CNTL_2P2Z_Ref2;	// instance #1
extern volatile long *CNTL_2P2Z_Out2;	// instance #1
extern volatile long *CNTL_2P2Z_Fdbk2;	// instance #1
extern volatile long *CNTL_2P2Z_Coef2; 	// instance #1
extern volatile long CNTL_2P2Z_DBUFF2[5];

//MATH_EMAVG - instance #1
extern volatile long *MATH_EMAVG_In1;
extern volatile long *MATH_EMAVG_Out1;
extern volatile long MATH_EMAVG_Multiplier1;

//MATH_EMAVG - instance #2
extern volatile long *MATH_EMAVG_In2;
extern volatile long *MATH_EMAVG_Out2;
extern volatile long MATH_EMAVG_Multiplier2;

// ---------------------------- DPLIB Variables ---------------------------------
// Declare the net variables being used by the DP Lib Macro here 

volatile long Duty3A,Duty3A_fixed;
volatile long VboostRead;
volatile long VpvRef;
volatile long VpvRead, IpvRead;
volatile long VpvRead_EMAVG, IpvRead_EMAVG;
volatile long IboostswRead;
volatile long IboostSwRef;
volatile long VpvRef_MPPT;

#pragma DATA_SECTION(CNTL_2P2Z_CoefStruct1, "CNTL_2P2Z_Coef");  
struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct1;

#pragma DATA_SECTION(CNTL_2P2Z_CoefStruct2, "CNTL_2P2Z_Coef");  
struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct2;

long Pgain_V,Igain_V,Dgain_V,Dmax_V;
long Pgain_I,Igain_I,Dgain_I,Dmax_I;

int16 UpdateCoef;

// DC-DC MPPT Variables
int16 PanelBoostConnect;	// 0 when Panel is connected to battery input
							// 1 when panel is connected to boost input

//-------------------------- MPPT tracking PnO and Incc ---------------------------
mppt_incc mppt_incc1 = mppt_incc_DEFAULTS;
mppt_pno  mppt_pno1 = mppt_pno_DEFAULTS;

int16 MPPT_slew;
int16 Run_MPPT;
int16 MPPT_ENABLE;
//-----------------------------Inverter Variables ----------------------------------

//--------------------------------- SINE GEN LIB ------------------------------------
SGENHP_2 sgen = SGENHP_2_DEFAULTS;

//-------------------------------- PWM DAC driver -----------------------------------
int16 PwmDacCh1=0;
int16 PwmDacCh2=0;
int16 PwmDacCh3=0;
int16 PwmDacCh4=0;
 
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

//-------------- PID GRANDO INSTANCE Voltage Loop and Current Loop   -----------------

PID_GRANDO_CONTROLLER	pidGRANDO_Iinv = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER	pidGRANDO_Vinv = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};

// ------------- Solar Sine Analyzer Block to measure RMS, frequency and ZCD ---------
SineAnalyzer_diff sine_mainsV = SineAnalyzer_diff_DEFAULTS;

// ------------- Software PLL for Grid Tie Applications ------------------------------
SPLL_1ph spll1;

// ------------- Data Logger --------------------------------------------------------
// Datalogger options and instance creation
int16 DlogCh1 = 0;
int16 DlogCh2 = 0;
int16 DlogCh3 = 0;
int16 DlogCh4 = 0;

DLOG_4CH dlog = DLOG_4CH_DEFAULTS;

_iq24 inv_ref_vol_inst,inv_meas_vol_inst;
_iq24 inv_ref_cur_inst;
_iq24 inv_meas_cur_diff_inst;
_iq24 inv_meas_cur_lleg1_inst;
_iq24 inv_meas_cur_lleg2_inst;

int16 CloseVloopInv;
int16 CloseIloopInv;

// for open loop operation of the inverter
_iq24	InvModIndex;				// Q15 format - 0..1.99 ( > 1 is overmodulation)

_iq24 inv_Iset;

_iq15 InvSine;

int16 ClearInvTrip;

int16 ResetPLL;

int32 Grid_Freq=GRID_FREQ;
int32 Vac_in;
int32 Offset_Volt;

_iq15 VrmsReal, VavgReal, Temp2;
_iq15 ScaleFactor;

int16 PVInverterState;

// Stand Alone Flash Image Instrumentation, GPIO toggles for different states 
int16  LedBlinkCnt,LedBlinkCnt2;
int16 timer1;

int16 TransmitData;
Uint16 sdata[2];     // Send data buffer
// Display Values 

// Monitor ("Get")						// Display as:
_iq	Gui_Vpv;							// Q10
_iq Gui_Ipv;							// Q12
_iq	Gui_Vboost;							// Q9 
_iq	Gui_PanelPower;						// Q9
_iq Gui_PanelPower_Theoretical;         //
_iq Gui_MPPTTrackingEff;
_iq	Gui_Light;						// Q13
_iq	Gui_LightRatio;					// Q13
_iq	Gui_LightRatio_Avg;				// Q13

_iq  Gui_LightCommand;				//Q13
Uint16  Gui_MPPTEnable;
Uint16  Gui_InvStart;
Uint16  Gui_InvStop;




Uint16  Gui_LightCommand_Prev;

// History arrays are used for Running Average calculation (boxcar filter)
// Used for CCS display and GUI only, not part of control loop processing
int16	Hist_Vpv[HistorySize];
int16	Hist_Ipv[HistorySize];
int16	Hist_Light[HistorySize];
int16	Hist_Vboost[HistorySize];

//Scaling Constants (values found via spreadsheet; exact value calibrated per board)
int16	K_Vpv;							// Q15
int16	K_Ipv;							// Q15
int16	K_Vboost;						// Q15
int16	K_Light;						// Q15

int16	iK_Vboost;						// Q15
int16	iK_Ipv;						// Q15

// Variables for background support only (no need to access)
int16	i;								// common use incrementer
Uint32	HistPtr, temp_Scratch; 			// Temp here means Temporary

Uint16 VloopTicker=0;

Uint16 ZCDDetect=0;
int16 sine_prev=0;

Inverter inverter;
HBridge hBridge;
Mppt mppt;
Panel panel;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// MAIN CODE - starts here
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void main(void)
{
//=================================================================================
//	INITIALISATION - General
//=================================================================================
	/**
		 * State Machine Constructors
		 */
		InverterCtor(&inverter);
		HBridgeCtor(&hBridge);
		MpptCtor(&mppt);
		PanelCtor(&panel);

		/**
		 * Statem Machine Initializations
		 */
		FsmInit((Fsm *)&inverter, 0);
		FsmInit((Fsm *)&hBridge, 0);
		FsmInit((Fsm *)&mppt, 0);
		FsmInit((Fsm *)&panel, 0);


	// The DeviceInit() configures the clocks and pin mux registers 
	// The function is declared in {ProjectName}-DevInit_F2803/2x.c,
	// Please ensure/edit that all the desired components pin muxes 
	// are configured properly that clocks for the peripherals used
	// are enabled, for example the individual PWM clock must be enabled 
	// along with the Time Base Clock 

	DeviceInit();	// Device Life support & GPIO

//-------------------------------- FRAMEWORK --------------------------------------

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


// Timing sync for background loops
// Timer period definitions found in PeripheralHeaderIncludes.h
	CpuTimer0Regs.PRD.all =  mSec5;		// A tasks
	CpuTimer1Regs.PRD.all =  mSec50;	// B tasks
	CpuTimer2Regs.PRD.all =  mSec1000;	// C tasks

	VTimer0[0] = 0;	
	VTimer1[0] = 0;
	VTimer2[0] = 0;
	LedBlinkCnt = 5;
	LedBlinkCnt2 = 5;

// ---------------------------------- USER -----------------------------------------
//  put common initialization/variable definitions here

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
		
//==================================================================================
//	INCREMENTAL BUILD OPTIONS - NOTE: selected via {ProjectName-Settings.h
//==================================================================================
// ---------------------------------- USER -----------------------------------------

//----------------------------------------------------------------------
//#if (INCR_BUILD == 1) 	
//----------------------------------------------------------------------
	
	// Configure PWM3 for 100Khz switching Frequency
	PWM_1ch_UpDwnCntCompl_CNF(3, 600,0,30); 
	
	//Solar_PWM_Inv_1ph_unipolar_CNF(1, 1500, 20, 20);
	PWM_1phInv_unipolar_CNF(1,1500,20,20);		

//============== Inverter Driver initialization	==================
	/*invdrv.deadband = INVERTER_DEADBAND;
	invdrv.n1		= 1;			// EPWM1
	invdrv.n2		= 2;			// EPWM2
	invdrv.period	= SINE_GEN_PRD;
	Solar_PWMDRV_Inv_1ph_unipolINIT_MACRO(invdrv);
	*/				
//================================================================   	
   	// ADC Channel Selection for Configuring the ADC
	// The following configuration would configure the ADC for parameters needed for 
	

	
	//Map channel to ADC Pin
	// the dummy reads are to account for first sample issue in Rev 0 silicon
	// Please refer to the Errata and the datasheet, this would be tfixed in later versions of the silicon
    ChSel[0] = 14;						 // B6 - Iboostsw-FB, DC-DC Boost switch current, not routed on Rev 1, dummy read
    ChSel[1] = 14;						 // B6 - Iboostsw-FB, DC-DC Boost switch current, not routed on Rev 1  
    ChSel[2] = 4;						 // A4 - Ileg1,
    ChSel[3] = 4;						 // A4 - Ileg1,
    ChSel[4] = 6;						 // A6 - Ileg2,
    ChSel[5] = 2;                        // A2 - Vb_FB, DC DC Boost Output Voltage
    ChSel[6] = 0;						 // A0 - Ipv_FB, Panel input current  
    ChSel[7] = 1;						 // A1 - Vpv_FB, Panel input Voltage
    ChSel[8] = 7;						 // A7 - Vac-fb
    ChSel[9] = 5;						 // A5 - VN-fb  
    ChSel[10] = 9;						 // B1 - VL-fb
    ChSel[11] = 8;						 // B0 - Light-fb 
     
	// Select Trigger Event 
    TrigSel[0]= ADCTRIG_EPWM3_SOCA;
    TrigSel[1]= ADCTRIG_EPWM3_SOCA;
	TrigSel[2]= ADCTRIG_EPWM1_SOCA;
	TrigSel[3]= ADCTRIG_EPWM1_SOCA;
	TrigSel[4]= ADCTRIG_EPWM1_SOCA;
	TrigSel[5]= ADCTRIG_EPWM3_SOCA;
	TrigSel[6]= ADCTRIG_EPWM3_SOCA;
	TrigSel[7]= ADCTRIG_EPWM3_SOCA;
	TrigSel[8]= ADCTRIG_EPWM1_SOCA;
	TrigSel[9]= ADCTRIG_EPWM1_SOCA;
	TrigSel[10]= ADCTRIG_EPWM1_SOCA;
	TrigSel[11]= ADCTRIG_EPWM1_SOCA;
	
	
	ADC_SOC_CNF(ChSel,TrigSel,ACQPS,3,0); // use auto clr ADC int flag mode, end of conversion 2 trigger ADC INT 1

	
	// Configure the Start of Conversion for the ADC.
	
	// SOC for DCDC Boost MPPT
	EPwm3Regs.ETSEL.bit.SOCAEN 	= 1;
	EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;	// Use PRD event as trigger for ADC SOC 
    EPwm3Regs.ETPS.bit.SOCAPRD 	= ET_2ND;        // Generate pulse on 2nd event 
	
	//SOC for DCAC Inverter,SOCA is configured by the PWM CNF macro
	
	// Implement phase synchronization to avoid ADC and ISR conflicts
	EPwm1Regs.TBCTL.bit.PHSEN   = TB_DISABLE;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	
	EPwm2Regs.TBCTL.bit.PHSEN   = TB_ENABLE;
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
	EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm2Regs.TBPHS.half.TBPHS =2;
	
	EPwm3Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_IN;
	EPwm3Regs.TBCTL.bit.PHSEN=TB_ENABLE; 
	EPwm3Regs.TBCTL.bit.PHSDIR=TB_UP;
	
	EPwm3Regs.TBPHS.half.TBPHS=4;
	
	// Digital Power CLA(DP) library initialisation 
	DPL_Init();

//============================================================
	// Lib Module connection to "nets" 
	//----------------------------------------
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

	// Initialize the net variables
	Duty3A =_IQ24(0.0);
	VboostRead=_IQ24(0.0);
	IboostswRead=_IQ24(0.0);
	VpvRef=_IQ24(0.9);	// to increas current, we need to reduce VpvRef, thus initailize it with a high value. 
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
       		
	// Initialize DATALOG module
    dlog.iptr1 = &DlogCh1;
    dlog.iptr2 = &DlogCh2;
    dlog.iptr3 = &DlogCh3;
    dlog.iptr4 = &DlogCh4;
    dlog.trig_value = _IQ15(0.08);
    dlog.size = 0x64;
    dlog.prescalar = 10;
    dlog.init(&dlog);	  				
       					    	
//	pidGRANDO_Iinv.param.Kp=_IQ(1.2);
	pidGRANDO_Iinv.param.Kp=_IQ(0.8);
//	pidGRANDO_Iinv.param.Ki=_IQ(0.052);
	pidGRANDO_Iinv.param.Ki=_IQ(0.15);
	pidGRANDO_Iinv.param.Kd=_IQ(0.0);
	pidGRANDO_Iinv.param.Kr=_IQ(1.0);
	pidGRANDO_Iinv.param.Umax=_IQ(1.0);
	pidGRANDO_Iinv.param.Umin=_IQ(-1.0);
		    	     			
    // Initialize PWMDAC module
	pwmdac1.PeriodMax = 500;   // 3000->10kHz, 1500->20kHz, 1000-> 30kHz, 500->60kHz
    pwmdac1.PwmDacInPointer0 = &PwmDacCh1;
    pwmdac1.PwmDacInPointer1 = &PwmDacCh2;
    pwmdac1.PwmDacInPointer2 = &PwmDacCh3;
    pwmdac1.PwmDacInPointer3 = &PwmDacCh4;
    

	
	pidGRANDO_Vinv.param.Kp=_IQ(3.0);
	pidGRANDO_Vinv.param.Ki=_IQ(0.005);
	pidGRANDO_Vinv.param.Kd=_IQ(0.0);
	pidGRANDO_Vinv.param.Kr=_IQ(1.0);
	pidGRANDO_Vinv.param.Umax=_IQ(0.95);
	pidGRANDO_Vinv.param.Umin=_IQ(0.000);
		    		
	PWMDAC_INIT_MACRO(pwmdac1)
				
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
	
//#endif // (INCR_BUILD == 1)

#ifdef FLASH
// Initiate Commros 
	InitCommros();
#endif 

//====================================================================================
// INTERRUPTS & ISR INITIALIZATION (best to run this section after other initialization)
//====================================================================================

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
	EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL=DC_COMP1OUT;
	EPwm3Regs.TZDCSEL.bit.DCAEVT2=TZ_DCAH_HI;
	
	EPwm3Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;
	EPwm3Regs.DCACTL.bit.EVT2FRCSYNCSEL=DC_EVT_ASYNC;
	 
	EPwm3Regs.TZSEL.bit.DCAEVT2=0x1;
	
	EDIS;


	EALLOW;
	
// Cycle by cycle interrupt for CPU halt trip
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
	
	SPI_init();
//=================================================================================
//	BACKGROUND (BG) LOOP
//=================================================================================

//--------------------------------- FRAMEWORK -------------------------------------
	for(;;)  //infinite loop
	{
		// State machine entry & exit point
		//===========================================================
		//(*Alpha_State_Ptr)();	// jump to an Alpha state (A0,B0,...)
		//===========================================================

		static char returner = 0;
			// loop rate synchronizer for panel tasks
			if(CpuTimer0Regs.TCR.bit.TIF == 1)
			{
				CpuTimer0Regs.TCR.bit.TIF = 1;	// clear flag

				PanelEvent panelEvent;
				panelEvent.code = 'T';
				panelEvent.super_.transition = true;
				returner = PanelTransitionFunction(panel, &panelEvent);
				//if(returner == -1) return 0;
				FsmDispatch((Fsm *)&panel, (Event *)&panelEvent);  //dispatch

				VTimer0[0]++;			// virtual timer 0, instance 0 (spare)
			}

			// loop rate synchronizer for MPPT and 'Macro State Machine'
			if(CpuTimer1Regs.TCR.bit.TIF == 1)
			{
				CpuTimer1Regs.TCR.bit.TIF = 1;				// clear flag

				MpptEvent mpptEvent;
				mpptEvent.code = 'T';
				mpptEvent.super_.transition = true;
				returner = MpptTransitionFunction(mppt, &mpptEvent);
				//if(returner == -1) return 0;
				FsmDispatch((Fsm *)&mppt, (Event *)&mpptEvent);  //dispatch

				VTimer1[0]++;			// virtual timer 1, instance 0 (spare)
			}

			// loop rate synchronizer for coefficients tasks
			if(CpuTimer2Regs.TCR.bit.TIF == 1)
			{
				CpuTimer2Regs.TCR.bit.TIF = 1;				// clear flag
				static int count = 0;
				count++;

				if((count % 4 == 0) && (count != 0)){
					count = 0;
				/**
				 * This is the only thing that 'C' does, so I will just do this for now
				 */
					//Update Coefficients
					if(UpdateCoef==1)
					{
					CNTL_2P2Z_CoefStruct2.b2  = Dgain_I;                            // B2
					CNTL_2P2Z_CoefStruct2.b1  = (Igain_I-Pgain_I-Dgain_I-Dgain_I);  // B1
					CNTL_2P2Z_CoefStruct2.b0  = (Pgain_I + Igain_I + Dgain_I);      // B0
					UpdateCoef=0;
					}
				}
				VTimer2[0]++;			//virtual timer 2, instance 0 (spare)
			}

#ifdef FLASH
		ServiceRoutine(&commros); 
		Datalogger(&commros.m_datalogger,0);
#endif

	}
} //END MAIN CODE

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

static int i = 0;
int plotArray[PlotSize];

// ISR for inverter
interrupt void Inv_ISR()
{

#ifdef PWM_MODE

		EINT;
	//-------------------------------------------------------------------
	// Inverter State execution
	//-------------------------------------------------------------------

		VrmsReal = _IQ15mpy (KvInv, sine_mainsV.Vrms);

	//-----------------------------------------------------------------------------------------
	#if (INCR_BUILD == 1 ) // Current command is fixed
	//-----------------------------------------------------------------------------------------
		// frequency generation using Sine Gen function
		sgen.calc(&sgen);
		InvSine     = sgen.out1;

		if (ClearInvTrip==1)
		{
			EALLOW;
			EPwm1Regs.TZCLR.bit.OST=0x1;
			EPwm2Regs.TZCLR.bit.OST=0x1;
			EDIS;
			ClearInvTrip=0;
		}

	#endif // (INCR_BUILD == 1)

	//-----------------------------------------------------------------------------------------
	#if (INCR_BUILD == 2) 	// determine the current command
	//-----------------------------------------------------------------------------------------
		// frequency generation using Sine Gen function
		sgen.calc(&sgen);
		InvSine     = sgen.out1;

		//  Connect inputs of the PID_REG3 module and call the PID IQ controller

		//	Voltage loop

		pidGRANDO_Vinv.term.Ref = VboostRead; //Ref=VDC/sqrt(2) at full modulation index

		if((sine_prev<=0)&&(sgen.out1>0))
		{
			ZCDDetect=1;
		}
		if((sine_prev>=0)&&(sgen.out1<0))
		{
			ZCDDetect=1;
		}

		sine_prev=sgen.out1;

		if ( ClearInvTrip==1 && ZCDDetect==1 )
		{
			EALLOW;
			EPwm1Regs.TZCLR.bit.OST=0x1;
			EPwm2Regs.TZCLR.bit.OST=0x1;
			EDIS;
			ClearInvTrip=0;
		}

		if ((CloseVloopInv==1) && (ZCDDetect==1))
		{
			PID_GR_MACRO(pidGRANDO_Vinv);
			inv_Iset=pidGRANDO_Vinv.term.Out;
			VloopTicker++;
			ZCDDetect=0;
		}
	#endif // (INCR_BUILD == 2)

	//-----------------------------------------------------------------------------------------
	#if (INCR_BUILD == 3) 	// determine the current command
	//-----------------------------------------------------------------------------------------
		// PLL Start
		Vac_in=(long)((long)Vac_FB<<9)-Offset_Volt;	// shift to convert to Q21

		spll1.AC_input=Vac_in>>1;

		SPLL_1ph_MACRO(spll1);

		InvSine     = (long)(spll1.sin[0])>>6; // InvSine is in Q15

		//	Voltage loop
		pidGRANDO_Vinv.term.Fbk = VdcRef; // 30V/ 39.97
		pidGRANDO_Vinv.term.Ref = VboostRead; //Ref=VDC/sqrt(2) at full modulation index

		if (CloseVloopInv==1 && sine_mainsV.ZCD==1)
		{
			PID_GR_MACRO(pidGRANDO_Vinv);
			inv_Iset=pidGRANDO_Vinv.term.Out;
		}

		if (ResetPLL==1)
		{
			SPLL_1ph_init(60,_IQ21(0.00005),&spll1);	// Q20
			ResetPLL=0;
		}

		if (sine_mainsV.ZCD==1 && ClearInvTrip==1)
		{
			EALLOW;
			EPwm1Regs.TZCLR.bit.OST=0x1;
			EPwm2Regs.TZCLR.bit.OST=0x1;
			EDIS;
			ClearInvTrip=0;
		}
	#endif // (INCR_BUILD == 3)

		inv_ref_cur_inst = _IQ24mpy(inv_Iset, (((int32) (InvSine)) << 9)) ;

		inv_meas_cur_lleg1_inst=(((int32) Ileg1_fb) <<12)-_IQ24(0.5);
		inv_meas_cur_lleg2_inst=(((int32) Ileg2_fb) <<12)-_IQ24(0.5);

		inv_meas_cur_diff_inst = (inv_meas_cur_lleg1_inst - inv_meas_cur_lleg2_inst)<<1;

		inv_meas_vol_inst =((long)((long)Vac_FB<<12)-_IQ24(0.5))<<1;	// shift to convert to Q24

		pidGRANDO_Iinv.term.Fbk=inv_meas_cur_diff_inst;
		pidGRANDO_Iinv.term.Ref=inv_ref_cur_inst;

		if(CloseIloopInv==1)
		{
			DINT;
			PID_GR_MACRO(pidGRANDO_Iinv);
			EINT;
		}

		// Apply inverter o/p correction
		if (CloseIloopInv ==0)
		{
			PWMDRV_1phInv_unipolar(1,_IQ15(1500),_IQ24mpy((InvSine<<9),InvModIndex));
		}
		else
		{
			PWMDRV_1phInv_unipolar(1,_IQ15(1500),pidGRANDO_Iinv.term.Out);
		}

	// ------------------------------------------------------------------------------
	//    Connect inputs to the sine analyzer block , compute RMS, Freq, ZCD
	// ------------------------------------------------------------------------------
		sine_mainsV.Vin =(long)((long)Vac_FB<<3)-_IQ15(0.5);
		sine_mainsV.Vin = sine_mainsV.Vin <<1;
		SineAnalyzer_diff_MACRO (sine_mainsV);


	// ------------------------------------------------------------------------------
	//    Connect inputs of the Datalogger module
	// ------------------------------------------------------------------------------

	#if (INCR_BUILD == 2 || INCR_BUILD == 1)
	    DlogCh1 = (Uint16)sgen.out1;
	#elif (INCR_BUILD == 3)
		DlogCh1 = (Uint16)_IQtoIQ15(spll1.sin[0]<<3);  //PLL is in Q21, shift left by 3 to make it Q24
	#endif

		DlogCh2 = (int16)_IQtoIQ15(inv_meas_vol_inst);
		DlogCh3 = (int16)_IQtoIQ15(inv_meas_cur_diff_inst);
		DlogCh4 = (int16)_IQtoIQ15(inv_ref_cur_inst);

	    dlog.update(&dlog);

	// ------------------------------------------------------------------------------
	//    Connect inputs of the PWMDAC module
	// ------------------------------------------------------------------------------
	#if (INCR_BUILD == 2 || INCR_BUILD == 1)
	    PwmDacCh1 = (int16)(sgen.out1);
	#elif (INCR_BUILD == 3)
		PwmDacCh1 = (int16)_IQtoIQ15(spll1.sin[0]<<3);  //PLL is in Q21
	#endif
	    PwmDacCh2 = (int16)_IQtoIQ15(inv_meas_vol_inst);
	    PwmDacCh3 = (int16)_IQtoIQ15(inv_meas_cur_diff_inst);
	    PwmDacCh4 = (int16)_IQtoIQ15(inv_ref_cur_inst);

	// ------------------------------------------------------------------------------
	//    Call the PWMDAC update macro.
	// ------------------------------------------------------------------------------
		PWMDAC_MACRO(pwmdac1)

	#ifdef FLASH
		//update commros data logger probe
		Datalogger(&commros.m_datalogger,1);
	#endif

		//-------------------------------------------------------------------
		//			 Reinitialize for next ADC sequence
		//-------------------------------------------------------------------
		PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;		// Must acknowledge the PIE group
		AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		// Clear ADCINT1 flag
		//-------------------------------------------------------------------

		GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;	// Clear the pin

#elif HYBRID_MODE

	/**
	 * Things to determine about the state of the system:
	 * - Current through the inductor, voltage across the capacitor
	 * - Phase: determine switching regions based on phase, i.e. M1, M2, Si, So
	 */

	Vac_in=(long)((long)Vac_FB<<9)-Offset_Volt;	// shift to convert to Q21
	inv_ref_cur_inst = _IQ24mpy(inv_Iset, (((int32) (InvSine)) << 9)) ;

	inv_meas_cur_lleg1_inst=(((int32) Ileg1_fb) <<12)-_IQ24(0.5);
	inv_meas_cur_lleg2_inst=(((int32) Ileg2_fb) <<12)-_IQ24(0.5);

	inv_meas_cur_diff_inst = (inv_meas_cur_lleg1_inst - inv_meas_cur_lleg2_inst)<<1;

	inv_meas_vol_inst =((long)((long)Vac_FB<<12)-_IQ24(0.5))<<1;	// shift to convert to Q24

	//phase = atan(inv_meas_vol_inst, inv_meas_cur_diff_inst);
	//updateState(state, inv_meas_cur_diff_inst, inv_meas_vol_inst, phase);

	/**
	 * Based on current state, call transition function
	 * Do this by:
	 * a) creating an event when determination is made, calling transition function at end of decision logic
	 * b) call transition function with state variable as parameter
	 */

	hBridgeEvent hBridgeEvent;
	hBridgeEvent.code = 'T';
	panelEvent.super_.transition = true;
	returner = PanelTransitionFunction(panel, &panelEvent);
	FsmDispatch((Fsm *)&hBridge, (Event *)&hBridgeEvent);  //dispatch
	;
#endif

  	return;

}



