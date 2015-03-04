#ifndef ADC_CONFIGS_H
#define ADC_CONFIGS_H

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

/**
 * ADC configuration vars
 */
int 	ChSel[16] =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int		TrigSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int     ACQPS[16] =   {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

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



void ADC_Init(void){

	/**
	 * Map channel to ADC Pin the dummy reads are to account for first sample issue in Rev 0 silicon
	 * Please refer to the Errata and the datasheet, this would be fixed in later versions of the silicon
	 */
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

	
	/**
	 * Configure the Start of Conversion for the ADC.
	 */
	
	// SOC for DCDC Boost MPPT
	EPwm3Regs.ETSEL.bit.SOCAEN 	= 1;
	EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;	// Use PRD event as trigger for ADC SOC 
    EPwm3Regs.ETPS.bit.SOCAPRD 	= ET_2ND;        // Generate pulse on 2nd event 
	
	//SOC for DCAC Inverter,SOCA is configured by the PWM CNF macro	
}

#endif
