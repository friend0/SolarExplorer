#ifndef DP_LIB_SUPPORT_H
#define DP_LIB_SUPPORT_H

// ---------------------------- DPLIB Net Pointers ---------------------------------
// Declare net pointers that are used to connect the DP Lib Macros  here 

/**
 * @TODO: Figure out exactly how these are all working, and how they tie in with the variables.
 */
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

#endif