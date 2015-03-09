/*
 * Inverter.h
 *
 *  Created on: Mar 6, 2015
 *      Author: watchmen
 */

#ifndef INVERTER_H_
#define INVERTER_H_


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


#ifdef FLASH
	void InitFlash();
#endif
void MemCopy();

#ifdef FLASH
#pragma CODE_SECTION(Inv_ISR,"ramfuncs");
#endif

void SPI_init();

void initializeInverter(void);

void runInverter(void);


#endif /* INVERTER_H_ */
