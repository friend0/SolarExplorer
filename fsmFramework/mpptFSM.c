/**************************************************************************
*@file MpptFSM.C                                                          *
*@author Ryan Rodriguez                                                   *
*@date 12/27/14                                                           *
*@brief This module implements the Mppt state machine                     *
*This module is the software implementation of the Mppt state machine,    *
*and is intended for research purposes at this moment.                    *
**************************************************************************/

#include "fsm.h"
#include "mpptFSM.h"
//#include <stdio.h>
#include <ctype.h>
#include "stdbool.h"

/**
* Begin State Defintiions
*/

void MpptCtor(Mppt *self) {
    _FsmCtor_(&self->super_, &Mppt_initial);
}

//I have ommited the 'const' qualifier here because I would like to be bale to set the transition state here
//May also need to change others for setting exit transition? will this get handled in transition function?
void Mppt_initial(Mppt *self, Event *e) {
    /* ... initialization of Mppt attributes */

    //cannot call this here, the initial event passed is a zero...
    //e->transition = true;
    _FsmTran_((Fsm *) self, &Mppt_Execute);
}

/**
void Mppt_Execute(Mppt *self, Event *e) {

    if (e->transition == true) {
    	if(Run_MPPT==1)
    	{
    		// MPPT routine
    		mppt_incc1.Ipv = IpvRead_EMAVG; //IpvRead;
    		mppt_incc1.Vpv = VpvRead_EMAVG; //VpvRead;

    		mppt_incc_MACRO(mppt_incc1);

    		VpvRef_MPPT = mppt_incc1.VmppOut;

    		mppt_pno1.Ipv = IpvRead_EMAVG; //IpvRead;
    		mppt_pno1.Vpv = VpvRead_EMAVG; //VpvRead;

    		mppt_pno_MACRO(mppt_pno1);

    		//VpvRef_MPPT = mppt_pno1.VmppOut;

    		if(VpvRef_MPPT<_IQ24(0.0))
    		{
    			VpvRef_MPPT=_IQ24(0.0);
    		}
    		else if(VpvRef_MPPT>_IQ24(0.9))
    		{
    			VpvRef_MPPT=_IQ24(0.9);
    		}

    		VpvRef=VpvRef_MPPT;

    		Run_MPPT=0;
    	}

    //MPPT is a slow task, the following code enables to modulate the rate at which the MPPT is called

    	if(MPPT_slew==0)
    	{
    		if(MPPT_ENABLE==1)
    		{
    			Run_MPPT=1;
    			mppt_incc1.mppt_enable=1;
    		}
    		MPPT_slew=0;
    	}
    	else
    		MPPT_slew--;

    	// Toggle LD2 on the control card if MPPT enabled
    	if(MPPT_ENABLE==1)
    	{
    		if(LedBlinkCnt2==0)
    			{
    				GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;	//turn on/off LD2 on the controlCARD
    				LedBlinkCnt2=1;
    			}
    		else
    			LedBlinkCnt2--;
    	}
        e->transition = false;
    }

    switch (e->signal) {
        case TIMER_MPPT:
            _FsmTran_((Fsm *) self, &Mppt_Blink);
            break;

        case NO_EVENT:
            break;

        default:
            //_FsmTran_((Fsm *) self, &Mppt_Blink);
            break;
    }
}

void Mppt_Blink(Mppt *self, Event *e) {

    if (e->transition == true) {
    	// Toggle LD3 on control card to show execution of code
    	if(LedBlinkCnt==0)
    	{
    		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	//turn on/off LD3 on the controlCARD
    		LedBlinkCnt=4;
    	}
    	else
    		LedBlinkCnt--;
		e->transition = false;
    }

    switch (e->signal) {
        case TIMER_MPPT:
            _FsmTran_((Fsm *) self, &Mppt_StateMachine);
            break;
        default:
            //_FsmTran_((Fsm *) self, &Mppt_StateMachine);
            break;
    }
}

void Mppt_StateMachine(Mppt *self, Event *e) {

    if (e->transition == true) {
		#if (INCR_BUILD == 2)
			// Inverter State ==0 , wait for the command to start production of power
			// Inverter State ==1 , Check if panel voltage is available, i.e. Vpv > 5V
			//						if true enable MPPT
			// Inverter State ==2 , Check if Gui_Vboost>33V
			//						enable closed voltage loop regulation and current loop regulation
			// Inverter State ==3, wait for stop command, if stop, trip all PWM's, shut down MPPT and return to state 0, reset all values

			switch(PVInverterState)
			{
				case 0: // wait for the command to start the inverter
						if(Gui_InvStart==1)
						{
							PVInverterState=1;
							Gui_InvStart=0;
						}
						break;
				case 1: // check if PV is present
						if(Gui_Vpv>_IQ9(3.0))
						{
							PVInverterState=2;
							//Enable MPPT
							MPPT_ENABLE=1;
							ClearInvTrip=1;
							InvModIndex=_IQ24(0.0);
							pidGRANDO_Vinv.term.Fbk = VdcRef; // 30V/ 39.97
							mppt_incc1.Stepsize = _IQ(0.02);
						}

						if(Gui_InvStop==1)
						{
							PVInverterState=3;
						}
						break;
				case 2: // Check if DC Bus is greater than 33V , as currently the inverter is off this woudl happen quickly
						if(Gui_Vboost>_IQ9(31.0))
						{
							PVInverterState=3;
							CloseVloopInv=1;
							CloseIloopInv=1;
							mppt_incc1.Stepsize = _IQ(0.005);
						}
						if(Gui_InvStop==1)
						{
							PVInverterState=3;
						}
						break;
				case 3: // Wait for shut down sequence
						if(Gui_InvStop==1)
						{
							// switch off MPPT and also open the loop for current and voltage
							// the open loop index on the inverter is used to discharge the boost till it is close to the input panle voltage

							MPPT_ENABLE=0;
							mppt_incc1.mppt_first=1;
							mppt_pno1.mppt_first=1;

							VpvRef=_IQ24(0.9);

							// Run the reset sequence
							// Trip the PWM for the inverter
							// software force the trip of inverter to disable the inverter completely
							EALLOW;
							EPwm1Regs.TZFRC.bit.OST=0x1;
							EPwm2Regs.TZFRC.bit.OST=0x1;
							EDIS;

							// Wait for the command to be restarted
							PVInverterState=0;

							CloseVloopInv=0;
							CloseIloopInv=0;

							Gui_InvStop=0;

						}

						break;
				default:
						break;
			}

		#endif

		#if (INCR_BUILD == 3)

			// Inverter State ==1 , Check grid voltage, i.e. Vrms Real > _IQ15(12)
			// Inverter State ==2 , Check if panel voltage is available, i.e. Vpv > 3V
			//						if true enable MPPT
			// Inverter State ==3 , Check if Gui_Vboost>31V
			//						enable closed voltage loop regulation and current loop regulation
			// Inverter State == 4, Check if inv_Iset > 0.1 pu, Clear Inverter Trip
			// Inverter State == 5, wait for stop command, if stop, trip all PWM's, shut down MPPT and return to state 0, reset all values

			switch(PVInverterState)
			{
				case 0: // wait for the command to start the inverter
						if(Gui_InvStart==1)
						{
							PVInverterState=1;
							Gui_InvStart=0;
						}
						break;
				case 1: // once the inverter command is issued check if the grid is present
						if(VrmsReal>_IQ15(12.0))
						{
							PVInverterState=2;
							// take the PLL out of reset
						}
						break;
				case 2: // AC is present, check if PV is present
						if(Gui_Vpv>_IQ9(3.0))
						{
							PVInverterState=3;
							//Enable MPPT
							MPPT_ENABLE=1;
							mppt_incc1.Stepsize = _IQ(0.02);
						}
						break;
				case 3: // Check if DC Bus is greater than 31V , as currently the inverter is off this woudl happen quickly
						if(Gui_Vboost>_IQ9(31.0))
						{
							PVInverterState=4;
							CloseVloopInv=1;
							CloseIloopInv=1;
							mppt_incc1.Stepsize = _IQ(0.005);
						}
						break;
				case 4: // Check if there is enough current command then only connect the grid
						// this is for safety, as the board does not have reverse current sense,
						if(inv_Iset>_IQ24(0.1))
						{
							PVInverterState=5;
							ClearInvTrip=1;
						}
						break;
				case 5: // Wait for shut down sequence
						if(Gui_InvStop==1)
						{
							Gui_InvStop=0;
							// Run the reset sequence
							// Trip the PWM for the inverter
							// software force the trip of inverter to disable the inverter completely
							EALLOW;
							EPwm1Regs.TZFRC.bit.OST=0x1;
							EPwm2Regs.TZFRC.bit.OST=0x1;
							EDIS;

							MPPT_ENABLE=0;
							mppt_incc1.mppt_first=1;
							mppt_pno1.mppt_first=1;

							VpvRef=_IQ24(0.9);

							// Wait for the command to be restarted
							PVInverterState=0;

							CloseVloopInv=0;
							CloseIloopInv=0;

						}
						break;
				case 6: // wait for the power sequence
						break;
				default:
						break;
			}

			if(VrmsReal<_IQ15(10.0) && PVInverterState<3)
			{
				ResetPLL=1;
			}
			else
				ResetPLL=0;

		#endif

			if(timer1<20)
				timer1++;

			if(timer1==20)
			{
				PanelBoostConnect=1;
				Gui_LightCommand=_IQ13(0.2);
				timer1++;
			}

			//reset transition flag
			e->transition = false;
    }

    switch (e->signal) {
        case TIMER_MPPT:
            _FsmTran_((Fsm *) self, &Mppt_Execute);
            break;

        case NO_EVENT:
            break;

        default:;
            break;
    }
}

// @TODO: Figure out whta needs to be done with this. Dont think we need a seperate state for MPPT disable, this i smore on the macro level...
void Mppt_Disable(Mppt *self, Event *e) {

    if (e->transition == true) {
			e->transition = false;
    }

    switch (e->signal) {
        case EXECUTE:
            _FsmTran_((Fsm *) self, &Mppt_Execute);
            break;

        case NO_EVENT:
            break;

        default:;
            break;
    }
}
**/

/**
* @brief Used to determine the event that should be passed to the FsmDispatch function
*
* Using the MpptEvent class, we utilize the data variable 'code' to switch the signal
* of the Event super-class. Next, we take the updated Event signal and dispatch it to
* the current state function pointed to by Fsm of the class Mppt.
*
*
* @param self [description]
* @param e    [description]
*/

char MpptTransitionFunction(Mppt self, MpptEvent *e) {
    /**
     * Probably don't need state specific transitions since they're all getting the timer signal...
     * @TODO: remove duplicate code, dispatch signal to FSM regardless of state
     */
    void *funptr = self.super_.state__;
    //backtrace_symbols_fd(&funptr, 1, 1);

    if (funptr == &Mppt_Execute) {
        switch (e->code)                  //This switch uses the data attribute 'code' of the Mppt Event
        {
            case 'T' :
                e->super_.signal = TIMER_MPPT;
                e->super_.transition = true;
                break;

            case '.' :
                return -1;          // terminate the test

                //default : ke.super_.signal = ANY_KEY_SIG; break;
            default :
                e->super_.signal = 0;
                break;
        }
    }
    else if (funptr == &Mppt_Blink) {
        switch (e->code)                  //This switch uses the data attribute 'code' of the Mppt Event
        {
            case 'T' :
                e->super_.signal = TIMER_MPPT;
                e->super_.transition = true;
                break;
            case '.' :
                return -1;          // terminate the test

            default :
                e->super_.signal = 0;
                break;
        }
    }
    else if (funptr == &Mppt_Disable) {
        switch (e->code)                  //This switch uses the data attribute 'code' of the Mppt Event
        {
            case 'T' :
                e->super_.signal = TIMER_MPPT;
                e->super_.transition = true;
                break;
            case '.' :
                return -1;          // terminate the test

            default :
                e->super_.signal = 0;
                break;
        }
    }
    else;


    //FsmDispatch( (Fsm *)&self, (Event *)&e);  //dispatch
    return 0;
}


/**
* End State Implementations
*/



// int main()
// {

//     /**
//       * Define which state machine will be tested
//     */
//     #define MPPT 1

//     int returner = 0;
//     //Declare the variable k to be of the type 'inverter', where inverter is the class
//     //wrapping the FSM

//     //Take the class 'inverter', for example, and get the FSM it contains, then point it to an initialization state


//     Mppt k;
//     MpptCtor(&k);

//     FsmInit((Fsm *)&k, 0);

//     for (;;)
//     {
//         MpptEvent ke;

//         //printf("\nSignal<-");             //output the signal attribute of the event object

//         //ke.code should be the value sampled at ADC for actual implementation
//         ke.code = getc(stdin);            //obtain user input, use the data attribute 'code' to store it
//         getc(stdin);                      //discard newline '\n' //

//         void *funptr = k.super_.state__;
//         backtrace_symbols_fd(&funptr, 1, 1);


//         returner = MpptTransitionFunction(k, &ke);

//         if(returner == -1) return 0;
//         FsmDispatch((Fsm *)&k, (Event *)&ke);  //dispatch

//         funptr = k.super_.state__;
//         backtrace_symbols_fd(&funptr, 1, 1);


//     }
//     return 0;
// }
