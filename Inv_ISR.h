
#ifndef INV_ISR_H
#define INV_ISR_H

/**
 * @brief Inv_ISR function used in the feedback control of the inverter
 *
 * @TODO: Need to blank most of this and replace it with the hybrid inverter implementation when
 * it is appropriate
 */
interrupt void Inv_ISR()
{	
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

  	return;
  	
}

#endif
