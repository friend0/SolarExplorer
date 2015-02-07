#ifndef STRUCTS_H
#define STRUCTS_H

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

#endif
