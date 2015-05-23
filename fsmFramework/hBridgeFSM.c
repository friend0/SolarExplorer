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

long Vz0;
bool inC, inD, M1, M2;

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

char HBridgeTransitionFunction(HBridge self, HBridgeEvent *e, StateVariable state)
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
    
    /**
     * Determine which set the state variable belongs to: C or D
     */
    
    //Reset set membership status
    inC = false;   
    inD = false;

    /**
     * C:
     * Determining Flow Set Membership
     * Don't think this is necessary in hardware implementation
     */
    Vz0 = (state.current/ALPHA)^2 + (state.voltage/BETA)^2;


    /**
     * Supervisory Controller 
     * Determine if we need to swtich controllers, depending on where the state variable is
     */
    if(state.controller == GLOBAL){
        if((Vz0 >= CIN) && (Vz0 <= COUT)){      // are we between the two tracking bands? -> select forward controller
            state -> controller = FORWARD;
            //inD = true;
        }
    }
    else if(state.controller == FORWARD){
        if((Vz0 >= COUT) || (Vz0 <= CIN)){      // are we inside both, or outside both tracking bands? -> select global controller
            state -> controller = GLOBAL;
        }
    }

    /**
     * I don't think we really need to know when we're in the flow set since we will always be flowing, not simulating
     */

    /** Forward Controller Check */
    if(state.controller == FORWARD)           
    {      
        if((Vz0 >= CIN) && (Vz0 <= COUT)){
            inC = true;
        }
        else{
            inC = false;            
        }

    }
    /** Global Controller Check */
    else if(state.controller == GLOBAL)   
    {     
        if((Vz0 <= CIN) && (Vz0 >= COUT)){
            inC = true;
        }
        else{
            inC = false;            
        }
    }

    /**
     * D:
     * Determining Jump Set membership
     */
    
    /**
     * Determine if we are in 'fast-swithcing regions' M1 or M2 so that we may respond accordingly
     */
    //M1 = ((_IQabs(Vz0-cout) < ERROR) && ((state.current >= 0) && (state.current <= EPSILON)) && (state.voltage <= 0)) ? true:false;
    //M2 = ((_IQabs(Vz0 - COUT) < ERROR) && ((state.current >= -EPSILON) && (state.current <= 0)) && (state.voltage >= 0)) ? true:false;
    if((_IQabs(Vz0-cout) < ERROR) && ((state.current >= 0) && (state.current <= EPSILON)) && (state.voltage <= 0)) M1 = true;
    else M1 = false;
    
    if((_IQabs(Vz0 - COUT) < ERROR) && ((state.current >= -EPSILON) && (state.current <= 0)) && (state.voltage >= 0)) M2 = true;
    else M2 = false;

    /** Forward Controller Check */
    if(state.controller == FORWARD)           
    {      
        if(q != 0){

           if( (abs(Vz0-cin) <= err) && (il*q <= 0)){
               inD = 1;
           }
           else if( (abs(Vz0-cout) <= err) && (il*q >= 0)){
               inD = 1;
           }
       }
        else if (q == 0){
            if( (abs(Vz0-cin) <= err) && (q == 0)){
                inD = 1;
            }
        }
    }

    /** Global Controller CHeck */
    if(state.controller == GLOBAL){
        if((Vz0 >= cin) && (Vz0 <= cout)){
            inD = 1;
        }
    }

    /**
     * If we're in the jump set, determine which state transition to make
     * Else, dont waste clock cycles!
     */
    if(inD){

        /**
         * For the Hfw Controller
         */
        if(State.controller == FORWARD){    
            if(State.bridgeState != NEG_VDC){
                if( ((abs(Vz0-cout) <= err) && (il >= 0) && (~M1)) || (((abs(Vz0-cin) <= err)) && (il <= 0)) ){
                qplus = NEG_VDC;
                }
            }
            else if ( ((M1) && (abs(il - mEpsilon) >= err) && (q == 1)) || ((M2) && (abs(il + mEpsilon) >= err) && (q == -1)) ){
                    qplus = ZERO_VDC;
                }
            else if(State.bridgeState != VDC){
                if( ((abs(Vz0 - cout) <= err) && (il <= 0) && (~M2)) || (((abs(Vz0 - cin) <= err)) && (il >= 0)) ){
                    qplus = VDC;
                }
            }
            else{
                qplus = NO_EVENT;                
            }
        }

        /**
         * For the Hg Controller
         */
        else if(State.controller == GLOBAL){
            if(Vz0 <= CIN){
                qplus = VDC;
            }
            else if(Vz0 >= COUT){
                qplus = ZERO_VDC;
            }
            else{
                qplus = NO_EVENT;
            }
        }
    }

    if(pplus != NO_EVENT){
        e->super_.signal = qplus;
    }

    return 0;
}

