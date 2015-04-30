/**************************************************************************
*@File HBridgeFSM.C                                                      *
*@author Ryan Rodriguez                                                   *
*@date 12/27/14                                                           *
*@brief This module implements the HBridge state machine                 *
*This module is the software implementation of the HBridge state machine, *
*and is intended for research purposes at this moment.                    *
**************************************************************************/

#include "fsm.h"
#include "HBridgeFSM.h"
//#include <stdio.h>
#include <ctype.h>
#include "stdbool.h"
#include "inverterVariables.h"
///////////
//Events //
///////////

/* signals used by the HBridge FSM */
enum
{
    NEG_VDC,    //q = -1
    ZERO_VDC,   //q = 0
    VDC,        //q = 1
    NO_EVENT   //qDot = 0
};

/**
* Begin State Defintiions
*/

void HBridgeCtor(HBridge *self)
{
    _FsmCtor_(&self->super_, &HBridge_initial);
}

void HBridge_initial(HBridge *self, Event *e)
{
    /* ... initialization of HBridge attributes */
    _FsmTran_((Fsm *)self, &HBridge_default);
}

void HBridge_default(HBridge *self, Event *e)
{
    switch (e->signal)
    {

        case NEG_VDC:
            _FsmTran_((Fsm *)self, &HBridge_negVDC);
            break;

        case ZERO_VDC:
            _FsmTran_((Fsm *)self, &HBridge_Zero);
            break;

        case VDC:
            _FsmTran_((Fsm *)self, &HBridge_VDC);
            break;

        case NO_EVENT:
            _FsmTran_((Fsm *)self, &HBridge_Zero);
            break;
    }
}


void HBridge_VDC(HBridge *self, Event *e)
{
    switch (e->signal)
    {
        case NEG_VDC:
            _FsmTran_((Fsm *)self, &HBridge_negVDC);
            break;

        case ZERO_VDC:
            _FsmTran_((Fsm *)self, &HBridge_Zero);
            break;

        default:
            _FsmTran_((Fsm *)self, &HBridge_VDC);
            break;
    }
}

void HBridge_Zero(HBridge *self, Event *e)
{
    switch (e->signal)
    {
        case VDC:
            _FsmTran_((Fsm *)self, &HBridge_VDC);
            break;

        case NEG_VDC:
            _FsmTran_((Fsm *)self, &HBridge_negVDC);
            break;

        default:
            _FsmTran_((Fsm *)self, &HBridge_Zero);
            break;

    }
}

void HBridge_negVDC(HBridge *self, Event *e)
{
    switch (e->signal)
    {
        case VDC:
            _FsmTran_((Fsm *)self, &HBridge_VDC);
            break;

        case ZERO_VDC:
            _FsmTran_((Fsm *)self, &HBridge_Zero);
            break;

        default:
            _FsmTran_((Fsm *)self, &HBridge_negVDC);
            break;
    }
}
/**
* End State Definitions
*/

/**
* @brief Used to determine the event that should be passed to the FsmDispatch function
*
* Using the HBridgeEvent class, we utilize the data variable 'code' to switch the signal
* of the Event super-class. Next, we take the updated Event signal and dispatch it to
* the current state function pointed to by Fsm of the class HBridge.
*
*
* @param self [description]
* @param e    [description]
*/
/**
#define ampVc = 120/(2.85)*sqrt(2)
#define ampIl = 3*sqrt(2)
 C = ampIl/(ampVc*w)
 L = 1/(C*w^2)
a = ampVc
b = a/(C*w)
epsilon = .1 //choose some meaningful epsilon based on ADC resolution and sampling rate
err = 1e-4 //choose some meaningful error based on ADC resolution and sampling rate

cmid
cin
cout
Vz0 = (il/a)^2 + (vc/b)^2
**/

char HBridgeTransitionFunction(HBridge self, HBridgeEvent *e, StateVariable currentState)
{
    //After dereferencing, self is an HBridge object
    //
    //HBridgeEvent e is a wrapper structure containing members Char code, and Event super_.
    //  Code is just a data variable used to switch the signal attribute of the super_ Event.
    //  In our case, code should be a tuple of Il and Vc, used to switch the signal of the Event structure.
    //  The super_ Event is then passed to the dispatch function which calls the state function.
    //  The state function reads the signal and responds with the appropriate transition.

    //First, get the event pointed to by HBridge event
    //Next, get the signal pointed to by the event in HBridgeEvent

    void    *funptr = self.super_.state__;
    //void    *funptr = self->super_.state__;

    if(funptr == &HBridge_default){
        switch (e->code)                  //This switch uses the data attribute 'code' of the HBridge Event
        {
            case '.' : return -1;          // terminate the test
            case '+' : e->super_.signal = VDC; break;
            case '-' : e->super_.signal = NEG_VDC; break;
            case '0' : e->super_.signal = ZERO_VDC; break;

            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr == &HBridge_VDC){
        switch (e->code)                  //This switch uses the data attribute 'code' of the HBridge Event
        {
            case '.' : return -1;          // terminate the test
            case '+' : e->super_.signal = VDC; break;
            case '-' : e->super_.signal = NEG_VDC; break;
            case '0' : e->super_.signal = ZERO_VDC; break;

            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr == &HBridge_Zero){
        switch (e->code)                  //This switch uses the data attribute 'code' of the HBridge Event
        {
            case '.' : return -1;          // terminate the test
            case '+' : e->super_.signal = VDC; break;
            case '-' : e->super_.signal = NEG_VDC; break;
            case '0' : e->super_.signal = ZERO_VDC; break;

            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr == &HBridge_negVDC){
        switch (e->code)                  //This switch uses the data attribute 'code' of the HBridge Event
        {
            case '.' : return -1;          // terminate the test
            case '+' : e->super_.signal = VDC; break;
            case '-' : e->super_.signal = NEG_VDC; break;
            case '0' : e->super_.signal = ZERO_VDC; break;

            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else
        ;

    return 0;
}

