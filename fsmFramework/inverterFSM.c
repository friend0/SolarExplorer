/**************************************************************************
*@File inverterFSM.C                                                      *
*@author Ryan Rodriguez                                                   *
*@date 12/27/14                                                           *
*@brief This module implements the inverter state machine                 *
*This module is the software implementation of the inverter state machine,*
*and is intended for research purposes at this moment.                    *
**************************************************************************/

#include "fsm.h"
#include "inverterFSM.h"
//#include <stdio.h>
#include <ctype.h>
#include "stdbool.h"

//#include <execinfo.h>

/****************************************************************
Events
****************************************************************/
/* signals used by the Inverter FSM */
enum
{
    POWER_ON,
    OUTSIDE_PARAMETERS,
    INSIDE_PARAMETERS,
    ALMOST_OUT_OF_PARAMETERS,
    SHUT_DOWN,
    NO_EVENT
};

/**
* Begin State Defintiions
*/

void InverterCtor(Inverter *self)
{
    _FsmCtor_(&self->super_, &Inverter_initial);
}

void Inverter_initial(Inverter *self, Event *e)
{
    _FsmTran_((Fsm *)self, &Inverter_default);
}

void Inverter_default(Inverter *self, Event *e)
{
    switch (e->signal)
    {
        case POWER_ON:
            _FsmTran_((Fsm *)self, &Inverter_PowerOn);
            break;

        case OUTSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_OutOfParameters);
            break;

        case INSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_WithinParameters);
            break;

        case ALMOST_OUT_OF_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_AlmostOutOfParameters);
            break;

        case SHUT_DOWN:
            _FsmTran_((Fsm *)self, &Inverter_ShutDown);
            break;

        case NO_EVENT:
            break;

        default:
            ;
            break;
    }
}


void Inverter_PowerOn(Inverter *self, Event *e)
{
    switch (e->signal)
    {
        case POWER_ON:
            _FsmTran_((Fsm *)self, &Inverter_PowerOn);
            break;

        case OUTSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_OutOfParameters);
            break;

        case INSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_WithinParameters);
            break;

        case ALMOST_OUT_OF_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_AlmostOutOfParameters);
            break;

        case SHUT_DOWN:
            _FsmTran_((Fsm *)self, &Inverter_ShutDown);
            break;

        case NO_EVENT:
            break;

        default:
            ;
            break;
    }
}

void Inverter_OutOfParameters(Inverter *self, Event *e)
{
    switch (e->signal)
    {
        case POWER_ON:
            _FsmTran_((Fsm *)self, &Inverter_PowerOn);
            break;

        case OUTSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_OutOfParameters);
            break;

        case INSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_WithinParameters);
            break;

        case ALMOST_OUT_OF_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_AlmostOutOfParameters);
            break;

        case SHUT_DOWN:
            _FsmTran_((Fsm *)self, &Inverter_ShutDown);
            break;

        case NO_EVENT:
            break;

        default:
            ;
            break;

    }
}

void Inverter_WithinParameters(Inverter *self, Event *e)
{
    switch (e->signal)
    {
        case POWER_ON:
            _FsmTran_((Fsm *)self, &Inverter_PowerOn);
            break;

        case OUTSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_OutOfParameters);
            break;

        case INSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_WithinParameters);
            break;

        case ALMOST_OUT_OF_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_AlmostOutOfParameters);
            break;

        case SHUT_DOWN:
            _FsmTran_((Fsm *)self, &Inverter_ShutDown);
            break;

        case NO_EVENT:
            break;

        default:
            ;
            break;
    }
}

void Inverter_AlmostOutOfParameters(Inverter *self, Event *e)
{
    switch (e->signal)
    {
        case POWER_ON:
            _FsmTran_((Fsm *)self, &Inverter_PowerOn);
            break;

        case OUTSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_OutOfParameters);
            break;

        case INSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_WithinParameters);
            break;

        case ALMOST_OUT_OF_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_AlmostOutOfParameters);
            break;

        case SHUT_DOWN:
            _FsmTran_((Fsm *)self, &Inverter_ShutDown);
            break;

        case NO_EVENT:
            break;

        default:
            ;
            break;
    }
}

void Inverter_ShutDown(Inverter *self, Event *e)
{
    switch (e->signal)
    {
        case POWER_ON:
            _FsmTran_((Fsm *)self, &Inverter_PowerOn);
            break;

        case OUTSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_OutOfParameters);
            break;

        case INSIDE_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_WithinParameters);
            break;

        case ALMOST_OUT_OF_PARAMETERS:
            _FsmTran_((Fsm *)self, &Inverter_AlmostOutOfParameters);
            break;

        case SHUT_DOWN:
            _FsmTran_((Fsm *)self, &Inverter_ShutDown);
            break;

        case NO_EVENT:
            break;

        default:
            ;
            break;
    }
}

/**
* @brief Used to determine the event that should be passed to the FsmDispatch function
*
* In micro implmeentation, I think this mcould be replaced completely as most of these transitions should be occuring from
* ADC sample, no need to be passed signals, they can be determined on the fly inside the states. 
* Using the InverterEvent class, we utilize the data variable 'code' to switch the signal
* of the Event super-class. Next, we take the updated Event signal and dispatch it to
* the current state function pointed to by Fsm of the class Inverter.
*
*
* @param self [description]
* @param e    [description]
*/

char InverterTransitionFunction(Inverter self, InverterEvent *e)
{
    //InverterEvent e is a wrapper structure containing members Char code, and Event super_.
    //  Code is just a data variable used to switch the signal attribute of the super_ Event. 
    //  In our case, code should be a tuple of Il and Vc, used to switch the signal of the Event structure.
    //  The super_ Event is then passed to the dispatch function which calls the state function. 
    //  The state function reads the signal and responds with the appropriate transition.        

    //First, get the event pointed to by inverter event
    //Next, get the signal pointed to by the event in inverterEvent

    //printf("InverterState:%d\n", self.super_.state__);

    /**
     * Grab the address of the function currently being pointed to
     */
    void    *funptr = self.super_.state__;
    //backtrace_symbols_fd(&funptr, 1, 1);

    if(funptr == &Inverter_default){
        switch (e->code)                  //This switch uses the data attribute 'code' of the Inverter Event
        {

            case 'P' : e->super_.signal = POWER_ON; break;
            case 'S' : e->super_.signal = SHUT_DOWN; break;
            case 'W' : e->super_.signal = INSIDE_PARAMETERS; break;
            case 'O' : e->super_.signal = OUTSIDE_PARAMETERS; break;
            case 'A' : e->super_.signal = ALMOST_OUT_OF_PARAMETERS; break;
            case '.' : return -1;          // terminate the test

                //default : ke.super_.signal = ANY_KEY_SIG; break;
            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr  == &Inverter_PowerOn){
        switch (e->code)                  //This switch uses the data attribute 'code' of the Inverter Event
        {
            case 'P' : e->super_.signal = POWER_ON; break;
            case 'S' : e->super_.signal = SHUT_DOWN; break;
            case 'W' : e->super_.signal = INSIDE_PARAMETERS; break;
            case 'O' : e->super_.signal = OUTSIDE_PARAMETERS; break;
            case 'A' : e->super_.signal = ALMOST_OUT_OF_PARAMETERS; break;
            case '.' : return -1;          // terminate the test

                //default : ke.super_.signal = ANY_KEY_SIG; break;
            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr  == &Inverter_OutOfParameters){
        switch (e->code)                  //This switch uses the data attribute 'code' of the Inverter Event
        {
            case 'P' : e->super_.signal = POWER_ON; break;
            case 'S' : e->super_.signal = SHUT_DOWN; break;
            case 'W' : e->super_.signal = INSIDE_PARAMETERS; break;
            case 'O' : e->super_.signal = OUTSIDE_PARAMETERS; break;
            case 'A' : e->super_.signal = ALMOST_OUT_OF_PARAMETERS; break;
            case '.' : return -1;          // terminate the test

            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr  == &Inverter_WithinParameters){
        switch (e->code)                  //This switch uses the data attribute 'code' of the Inverter Event
        {
            case 'P' : e->super_.signal = POWER_ON; break;
            case 'S' : e->super_.signal = SHUT_DOWN; break;
            case 'W' : e->super_.signal = INSIDE_PARAMETERS; break;
            case 'O' : e->super_.signal = OUTSIDE_PARAMETERS; break;
            case 'A' : e->super_.signal = ALMOST_OUT_OF_PARAMETERS; break;
            case '.' : return -1;          // terminate the test

                //default : ke.super_.signal = ANY_KEY_SIG; break;
            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr  == &Inverter_AlmostOutOfParameters){
        switch (e->code)                  //This switch uses the data attribute 'code' of the Inverter Event
        {
            case 'P' : e->super_.signal = POWER_ON; break;
            case 'S' : e->super_.signal = SHUT_DOWN; break;
            case 'W' : e->super_.signal = INSIDE_PARAMETERS; break;
            case 'O' : e->super_.signal = OUTSIDE_PARAMETERS; break;
            case 'A' : e->super_.signal = ALMOST_OUT_OF_PARAMETERS; break;
            case '.' : return -1;          // terminate the test

                //default : ke.super_.signal = ANY_KEY_SIG; break;
            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else if(funptr  == &Inverter_ShutDown){
        switch (e->code)                  //This switch uses the data attribute 'code' of the Inverter Event
        {
            case 'P' : e->super_.signal = POWER_ON; break;
            case 'S' : e->super_.signal = SHUT_DOWN; break;
            case 'W' : e->super_.signal = INSIDE_PARAMETERS; break;
            case 'O' : e->super_.signal = OUTSIDE_PARAMETERS; break;
            case 'A' : e->super_.signal = ALMOST_OUT_OF_PARAMETERS; break;
            case '.' : return -1;          // terminate the test

                //default : ke.super_.signal = ANY_KEY_SIG; break;
            default : e->super_.signal = NO_EVENT; break;
        }
    }
    else ;


    //FsmDispatch( (Fsm *)&self, (Event *)&e);  //dispatch
    return 0;
}


/**
* End State Implementations
*/
