/*
 * inverterVariables.h
 *
 *  Created on: Mar 9, 2015
 *      Author: watchmen
 */

#ifndef INVERTERVARIABLES_H_
#define INVERTERVARIABLES_H_

#include "SolarExplorer-Settings.h"
#include "IQmathLib.h"
#include "PeripheralHeaderIncludes.h"
#include "mppt_incc.h"
#include "mppt_pno.h"
#include "sgen.h"
#include "PWM_dac.h"
#include "SineAnalyzer_diff.h"
#include "Util_DLOG4CH.h"
#include "pid_grando.h"

//#include "SPLL_1ph.h"

	// ADC Channel Selection for Configuring the ADC
	// The following configuration would configure the ADC for parameters needed for
#define Iboostsw_FB		AdcResult.ADCRESULT1
#define Ileg1_fb 		AdcResult.ADCRESULT3
#define Ileg2_fb		AdcResult.ADCRESULT4
#define Vboost_FB  		AdcResult.ADCRESULT5
#define Ipv_FB			AdcResult.ADCRESULT6
#define Vpv_FB			AdcResult.ADCRESULT7
#define Vac_FB			AdcResult.ADCRESULT8
#define VN_FB			AdcResult.ADCRESULT9
#define VL_FB			AdcResult.ADCRESULT10
#define LIGHT_FB		AdcResult.ADCRESULT11

extern int16	VTimer0[4];					// Virtual Timers slaved off CPU Timer 0
extern int16	VTimer1[4];					// Virtual Timers slaved off CPU Timer 1
extern int16	VTimer2[4];					// Virtual Timers slaved off CPU Timer 2

// Used for ADC Configuration
extern int 	ChSel[16];
extern int	TrigSel[16];
extern int  ACQPS[16];


/**
 * Struct for holding state of RLC filter
 */
typedef struct {
	/**
	 * Will need to be chosen for proper Q values in IQmath
	 **/
	long current, voltage, phase;
	char controller, bridgeState;
}StateVariable;

void updateState(StateVariable *s, long current, long voltage, long phase);

// Used to indirectly access all EPWM modules
extern volatile struct EPWM_REGS *ePWM[];

// Used to indirectly access all Comparator modules
extern volatile struct COMP_REGS *Comp[];
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

extern volatile long Duty3A,Duty3A_fixed;
extern volatile long VboostRead;
extern volatile long VpvRef;
extern volatile long VpvRead, IpvRead;
extern volatile long VpvRead_EMAVG, IpvRead_EMAVG;
extern volatile long IboostswRead;
extern volatile long IboostSwRef;
extern volatile long VpvRef_MPPT;

extern struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct1;

extern struct CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct2;

extern long Pgain_V,Igain_V,Dgain_V,Dmax_V;
extern long Pgain_I,Igain_I,Dgain_I,Dmax_I;

extern int16 UpdateCoef;

// DC-DC MPPT Variables
extern int16 PanelBoostConnect;	// 0 when Panel is connected to battery input
							// 1 when panel is connected to boost input

//-------------------------- MPPT tracking PnO and Incc ---------------------------
extern mppt_incc mppt_incc1;
extern mppt_pno  mppt_pno1;

extern int16 MPPT_slew;
extern int16 Run_MPPT;
extern int16 MPPT_ENABLE;
//-----------------------------Inverter Variables ----------------------------------

//--------------------------------- SINE GEN LIB ------------------------------------
extern SGENHP_2 sgen;

//-------------------------------- PWM DAC driver -----------------------------------
extern int16 PwmDacCh1;
extern int16 PwmDacCh2;
extern int16 PwmDacCh3;
extern int16 PwmDacCh4;

extern PWMDAC pwmdac1;

//-------------- PID GRANDO INSTANCE Voltage Loop and Current Loop   -----------------

extern PID_GRANDO_CONTROLLER	pidGRANDO_Iinv;
extern PID_GRANDO_CONTROLLER	pidGRANDO_Vinv;

// ------------- Solar Sine Analyzer Block to measure RMS, frequency and ZCD ---------
extern SineAnalyzer_diff sine_mainsV;

// ------------- Software PLL for Grid Tie Applications ------------------------------

//@TODO: will need to manage this struct some other way in external state machines
//extern SPLL_1ph spll1;

// ------------- Data Logger --------------------------------------------------------
// Datalogger options and instance creation
extern int16 DlogCh1;
extern int16 DlogCh2;
extern int16 DlogCh3;
extern int16 DlogCh4;


extern DLOG_4CH dlog;

extern _iq24 inv_ref_vol_inst,inv_meas_vol_inst;
extern _iq24 inv_ref_cur_inst;
extern _iq24 inv_meas_cur_diff_inst;
extern _iq24 inv_meas_cur_lleg1_inst;
extern _iq24 inv_meas_cur_lleg2_inst;

extern int16 CloseVloopInv;
extern int16 CloseIloopInv;

// for open loop operation of the inverter
extern _iq24	InvModIndex;				// Q15 format - 0..1.99 ( > 1 is overmodulation)extern

extern _iq24 inv_Iset;

extern _iq15 InvSine;

extern int16 ClearInvTrip;

extern int16 ResetPLL;

extern int32 Grid_Freq;
extern int32 Vac_in;
extern int32 Offset_Volt;

extern _iq15 VrmsReal, VavgReal, Temp2;
extern _iq15 ScaleFactor;

extern int16 PVInverterState;

// Stand Alone Flash Image Instrumentation, GPIO toggles for different states
extern int16  LedBlinkCnt,LedBlinkCnt2;
extern int16 timer1;

extern int16 TransmitData;
extern Uint16 sdata[2];     // Send data buffer
// Display Values

// Monitor ("Get")						// Display as:
extern _iq	Gui_Vpv;							// Q10
extern _iq Gui_Ipv;							// Q12
extern _iq	Gui_Vboost;							// Q9
extern _iq	Gui_PanelPower;						// Q9
extern _iq Gui_PanelPower_Theoretical;         //
extern _iq Gui_MPPTTrackingEff;
extern _iq	Gui_Light;						// Q13
extern _iq	Gui_LightRatio;					// Q13
extern _iq	Gui_LightRatio_Avg;				// Q13extern

extern _iq  Gui_LightCommand;				//Q13
extern Uint16  Gui_MPPTEnable;
extern Uint16  Gui_InvStart;
extern Uint16  Gui_InvStop;




extern Uint16  Gui_LightCommand_Prev;

// History arrays are used for Running Average calculation (boxcar filter)
// Used for CCS display and GUI only, not part of control loop processing
extern int16	Hist_Vpv[HistorySize];
extern int16	Hist_Ipv[HistorySize];
extern int16	Hist_Light[HistorySize];
extern int16	Hist_Vboost[HistorySize];

//Scaling Constants (values found via spreadsheet; exact value calibrated per board)
extern int16	K_Vpv;							// Q15
extern int16	K_Ipv;							// Q15
extern int16	K_Vboost;						// Q15
extern int16	K_Light;						// Q15

extern int16	iK_Vboost;						// Q15
extern int16	iK_Ipv;						// Q15

// Variables for background support only (no need to access)
extern int16	i;								// common use incrementer
extern Uint32	HistPtr, temp_Scratch; 			// Temp here means Temporaryextern

extern Uint16 VloopTicker;

extern Uint16 ZCDDetect;
extern int16 sine_prev;

#endif /* INVERTERVARIABLES_H_ */
