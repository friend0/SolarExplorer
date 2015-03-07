#ifndef INVERTER_VARIABLES_H
#define INVERTER_VARIABLES_H

/**
 * Not sure if I like this solution better than just including these as needed in C files, but it makes things cleaner for the time being
 */
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

//#include "SolarExplorer-Includes.h"
//#include "ADC_Configs.h"

// Used to indirectly access all EPWM modules
extern volatile struct EPWM_REGS *ePWM[];

// Used to indirectly access all Comparator modules
extern volatile struct COMP_REGS *Comp[];

extern int16 UpdateCoef;

// DC-DC MPPT Variables
extern int16 PanelBoostConnect;	// 0 when Panel is connected to battery input
// 1 when panel is connected to boost input

extern volatile long Duty3A,Duty3A_fixed;
extern volatile long VboostRead;
extern volatile long VpvRef;
extern volatile long VpvRead, IpvRead;
extern volatile long VpvRead_EMAVG, IpvRead_EMAVG;
extern volatile long IboostswRead;
extern volatile long IboostSwRef;
extern volatile long VpvRef_MPPT;



extern PID_GRANDO_CONTROLLER	pidGRANDO_Iinv;
extern PID_GRANDO_CONTROLLER	pidGRANDO_Vinv;

extern int16 MPPT_slew;
extern int16 Run_MPPT;
extern int16 MPPT_ENABLE;

/**
 * External structs
 * @TODO: Determine if this is th ebest way to do this
 */
extern mppt_incc mppt_incc1;
extern mppt_pno  mppt_pno1;

extern SGENHP_2 sgen;
extern PWMDAC pwmdac1;
extern SineAnalyzer_diff sine_mainsV;
extern DLOG_4CH dlog;
/**
 * These are type definitione in mppt_incc.h, causing problems being global, invoke them where they are needed
 * Maybe figure some way to pass the struct around if necessary. Other typedefinitions here are being ommited from global at this time as well.
 * extern mppt_incc mppt_incc1;
 * extern mppt_pno  mppt_pno1;
 * extern SGENHP_2 sgen;
 * PWMDAC pwmdac1;
 * SineAnalyzer_diff sine_mainsV;
 * extern DLOG_4CH dlog;
 *
 */

//-------------------------------- PWM DAC driver -----------------------------------
extern int16 PwmDacCh1;
extern int16 PwmDacCh2;
extern int16 PwmDacCh3;
extern int16 PwmDacCh4;



// ------------- Software PLL for Grid Tie Applications ------------------------------
//extern SPLL_1ph spll1;

// ------------- Data Logger --------------------------------------------------------
// Datalogger options and instance creation
extern int16 DlogCh1;
extern int16 DlogCh2;
extern int16 DlogCh3;
extern int16 DlogCh4;


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


extern _iq Gui_Vpv;							// Q10
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
extern int16	K_Light;						// Q15extern 

extern int16	iK_Vboost;						// Q15
extern int16	iK_Ipv;						// Q15

// Variables for background support only (no need to access)
extern int16	i;								// common use incrementer
extern Uint32	HistPtr, temp_Scratch; 			// Temp here means Temporary

extern Uint16 VloopTicker;

extern Uint16 ZCDDetect;
extern int16 sine_prev;

#endif
