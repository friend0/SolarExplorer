#ifndef SolarExplorer_INCLUDES_H
#define SolarExplorer_INCLUDES_H
//======================================================================================
// ======================================================================================
// Include files
// ======================================================================================



// Include the IQmath Library First, define Global Q
#include "Solar_IQ.h"
//#include "IQmathLib.h"

#include "math.h"

// Include files for device support, F2803x in this case
#include "PeripheralHeaderIncludes.h"
#include "DSP2803x_EPWM_defines.h"	

// Kit specific header files, 
// define incremnetal build and other defines used by the project
#include "SolarExplorer-Settings.h"

// Library header files

// DPLib
#include "DPlib.h"	

//Sgen Lib
#include "sgen.h"

// Driver Lib
#include "DriverLib.h"
#include "PWM_1phInv_unipolar.h"
#include "PWM_dac.h"

// Solar Lib
#include "SineAnalyzer_diff.h"
#include "pid_grando.h"
#include "mppt_incc.h"
#include "mppt_pno.h"

#include "SPLL_1ph.h"

//DLOG Header file
#include "Util_DLOG4CH.h"			// Include header for the DLOG_4CH object\

#ifdef FLASH
//Commros header file 		
#include "Commros_user.h"
#endif

#endif
