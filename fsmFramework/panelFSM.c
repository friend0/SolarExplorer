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
#include <stdio.h>
#include <ctype.h>
#include "stdbool.h"

/**
* Begin State Defintiions
*/

void PanelCtor(Panel *self) {

    _FsmCtor_(&self->super_, &Panel_initial);
}

//I have ommited the 'const' qualifier here because I would like to be bale to set the transition state here
//May also need to change others for setting exit transition? will this get handled in transition function?
void Panel_initial(Panel *self, Event *e) {
    /* ... initialization of Panel attributes */

    //cannot call this here, the initial event passed is a zero...
    //e->transition = true;
    _FsmTran_((Fsm *) self, &Panel_Dashboard);
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
        printf("\nPanel Power On!\n");
        switch (e->code)                  //This switch uses the data attribute 'code' of the Panel Event
        {
            case 'E' :
                e->super_.signal = TIMER;
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
    else if (funptr == &Panel_Connect) {
        printf("\nPanel Out of Parameters!\n");
        switch (e->code)                  //This switch uses the data attribute 'code' of the Panel Event
        {
            case 'T' :
                e->super_.signal = TIMER;
                e->super_.transition = true;
                break;
            case '.' :
                return -1;          // terminate the test

            default :
                e->super_.signal = 0;
                break;
        }
    }
    else if (funptr == &Panel_Emulator) {
        printf("\nPanel Within Parameters!\n");
        switch (e->code)                  //This switch uses the data attribute 'code' of the Panel Event
        {
            case 'T' :
                e->super_.signal = TIMER;
                e->super_.transition = true;
                break;
            case '.' :
                return -1;          // terminate the test

                //default : ke.super_.signal = ANY_KEY_SIG; break;
            default :
                e->super_.signal = NO_EVENT_PANEL;
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




