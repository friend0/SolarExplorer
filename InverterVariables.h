#ifndef INVERTER_VARIABLES_H
#define INVERTER_VARIABLES_H

/-----------------------------Inverter Variables ----------------------------------

//--------------------------------- SINE GEN LIB ------------------------------------
SGENHP_2 sgen = SGENHP_2_DEFAULTS;

/**
 * PWM DAC Drivers
 *
 * @TODO: What are we using the DAC for?
 */
int16 PwmDacCh2=0;
int16 PwmDacCh3=0;
int16 PwmDacCh4=0;
 
PWMDAC pwmdac1 = PWMDAC_DEFAULTS;

/**
 * I won't be needing these for my implementation of the inverter algorithm!
 * @TODO: comment out/get rid of these
 */
//-------------- PID GRANDO INSTANCE Voltage Loop and Current Loop   -----------------

PID_GRANDO_CONTROLLER	pidGRANDO_Iinv = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};
PID_GRANDO_CONTROLLER	pidGRANDO_Vinv = {PID_TERM_DEFAULTS, PID_PARAM_DEFAULTS, PID_DATA_DEFAULTS};

/**
 * ------------- Solar Sine Analyzer Block to measure RMS, frequency and ZCD ---------
 */
SineAnalyzer_diff sine_mainsV = SineAnalyzer_diff_DEFAULTS;

/**
 * ------------- Software PLL for Grid Tie Applications ------------------------------
 */
SPLL_1ph spll1;

/** 
 * Datalogger options and instance creation
 */
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

/**
 * History arrays are used for Running Average calculation (boxcar filter)
 * Used for CCS display and GUI only, not part of control loop processing
 **/
 
int16	Hist_Vpv[HistorySize];
int16	Hist_Ipv[HistorySize];
int16	Hist_Light[HistorySize];
int16	Hist_Vboost[HistorySize];

/**
 * Scaling Constants (values found via spreadsheet; exact value calibrated per board)
 * @TODO: How are these values found? Link to the documentation here
 */
int16	K_Vpv;							// Q15
int16	K_Ipv;							// Q15
int16	K_Vboost;						// Q15
int16	K_Light;						// Q15

int16	iK_Vboost;						// Q15
int16	iK_Ipv;						// Q15

/**
 * Background support variables (no need to access)
 */
int16	i;								// common use incrementer
Uint32	HistPtr, temp_Scratch; 			// Temp here means Temporary

Uint16 VloopTicker=0;

Uint16 ZCDDetect=0;
int16 sine_prev=0;

#endif