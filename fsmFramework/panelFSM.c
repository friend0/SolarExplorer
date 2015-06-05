/**************************************************************************
*@file PanelFSM.C                                                          *
*@author Ryan Rodriguez                                                   *
*@date 12/27/14                                                           *
*@brief This module implements the Panel state machine                     *
*This module is the software implementation of the Panel state machine,    *
*and is intended for research purposes at this moment.                    *
**************************************************************************/

#include "fsm.h"
#include "PanelFSM.h"
#include "inverterVariables.h"
//#include <stdio.h>
#include <ctype.h>
#include "stdbool.h"

/**
* Begin State Defintiions
*/

void PanelCtor(Panel *self) {

    _FsmCtor_(&self->super_, &Panel_initial);
}


void Panel_initial(Panel *self, Event *e) {

    _FsmTran_((Fsm *) self, &Panel_Dashboard);
}


void Panel_Dashboard(Panel *self, Event *e) {

    if (e->transition == true) {
	// Dashboard measurement calculated by:
	//	Gui_Vbout = VboutAvg * K_Vbout, where VboutAvg = sum of 8 Vbout samples
	//	Gui_Iinb = IinbAvg * K_Iinb, where IinbAvg = sum of 8 Iinb samples

		HistPtr++;
		if (HistPtr >= HistorySize)	HistPtr = 0;

		// BoxCar Averages - Input Raw samples into BoxCar arrays
		//----------------------------------------------------------------
		Hist_Vpv[HistPtr]    = 	Vpv_FB;
		Hist_Ipv[HistPtr]    = 	Ipv_FB;
		Hist_Vboost[HistPtr] = 	Vboost_FB;

		temp_Scratch=0;
		for(i=0; i<8; i++)	temp_Scratch = temp_Scratch + Hist_Vpv[i];
		Gui_Vpv = ( (long) temp_Scratch * (long) K_Vpv ) >> 15;

		temp_Scratch=0;
		for(i=0; i<8; i++)	temp_Scratch = temp_Scratch + Hist_Ipv[i];
		Gui_Ipv = ( (long) temp_Scratch * (long) K_Ipv ) >> 15;

		temp_Scratch=0;
		for(i=0; i<8; i++)	temp_Scratch = temp_Scratch + Hist_Vboost[i];
		Gui_Vboost = ( (long) temp_Scratch * (long) K_Vboost ) >> 15;


		Gui_PanelPower=_IQ9mpy(((Gui_Vpv-_IQ9(0.20))>_IQ9(0.0)?(Gui_Vpv-_IQ9(0.20)):_IQ9(0.0)),(((long)(Gui_Ipv-_IQ12(0.03))>>3)>_IQ9(0.0)?((long)(Gui_Ipv-_IQ12(0.03))>>3):_IQ9(0.0)));

		if(Gui_PanelPower_Theoretical!=0)
			Gui_MPPTTrackingEff =_IQ9mpy(_IQ9div(Gui_PanelPower,Gui_PanelPower_Theoretical),_IQ9(100.0));
		else
			Gui_MPPTTrackingEff=0;

        e->transition = false;
    }

    switch (e->signal) {
        case TIMER_PANEL:
            _FsmTran_((Fsm *) self, &Panel_Dashboard);
            break;

        case NO_EVENT_PANEL:
            break;

        default:
            break;
    }
}


/**
* @brief Used to determine the event that should be passed to the FsmDispatch function
*
* Using the PanelEvent class, we utilize the data variable 'code' to switch the signal
* of the Event super-class. Next, we take the updated Event signal and dispatch it to
* the current state function pointed to by Fsm of the class Panel.
*
*
* @param self [description]
* @param e    [description]
*/

char PanelTransitionFunction(Panel self, PanelEvent *e) {
    //PanelEvent e is a wrapper structure containing members Char code, and Event super_.
    //  Code is just a data variable used to switch the signal attribute of the super_ Event.
    //  In our case, code should be a tuple of Il and Vc, used to switch the signal of the Event structure.
    //  The super_ Event is then passed to the dispatch function which calls the state function.
    //  The state function reads the signal and responds with the appropriate transition.

    //First, get the event pointed to by Panel event
    //Next, get the signal pointed to by the event in PanelEvent

    //printf("PanelState:%d\n", self.super_.state__);

    /**
    * Grab the address of the function currently being pointed to
    */
    void *funptr = self.super_.state__;
    //backtrace_symbols_fd(&funptr, 1, 1);

    if (funptr == &Panel_Dashboard) {
        switch (e->code)                  //This switch uses the data attribute 'code' of the Panel Event
        {
            case 'T' :
                e->super_.signal = TIMER_PANEL;
                e->super_.transition = true;
                break;
            case '.' :
                return -1;          // terminate the test

            default :
                e->super_.signal = 0;
                break;
        }
    }

    else;		//do nothing, error state


    //FsmDispatch( (Fsm *)&self, (Event *)&e);  //dispatch
    return 0;
}


/**
* End State Implementations
*/




