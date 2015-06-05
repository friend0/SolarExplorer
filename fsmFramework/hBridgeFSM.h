#ifndef H_BRIDGE_FSM_H
#define H_BRIDGE_FSM_H

#include "inverterVariables.h"

/////////////////////////////////////////
//Declaration of FSM and Event objects //
/////////////////////////////////////////

typedef struct HBridge HBridge;
typedef struct HBridgeEvent HBridgeEvent;

struct HBridge
{
    Fsm super_; /* extend the Fsm class */
    //Attributes
};

struct HBridgeEvent
{
    Event super_; /* extend the Event class */
    //Attributes
    char code;
};

//Making the enum of state public so that PWM driver function can access on return of transition function


////////////////////////////////////
// Declaration of state functions //
////////////////////////////////////

/**
* @brief Constructor Function
* This function is to be used for 'instantiating' state machines
* @code(.c)
*     HBridge HBridge;
*     HBridgeCtor(&HBridge);
* @endcode
* HBridgeCtor is a wrapper function for a call to:
* @code
*     _FsmCtor_(&self->super_, &HBridge_initial);
* @endcode
* which uses the 'super class'
*
* @param self
*/
void HBridgeCtor(HBridge *self);
/**
* @brief Entry state to be used for initialization and setup of the state machine.
* 
* Implements the initial transition of the HBridge FSM. To be used for initializations
* and setup of the machine. Can also serve no function but to transition to the default or
* zero states. 
*
* @param self self reference to HBridgeFSM
* @param e    event
*/
void HBridge_initial(HBridge *self, Event *e);

/**
* Implements the default transition
* @param self self reference to HBridgeFSM
* @param e    event
*/
void HBridge_default(HBridge *self, Event *e);

/**
* Implements the state handler for the case that the H-Bridge
* is supplying +Vdc to the input of the RLC filter.
* @param self self reference to HBridgeFSM
* @param e    event
*/
void HBridge_VDC(HBridge *self, Event *e);

/**
* Implements the state handler for the case that the H-Bridge
* is supplying zero volts DC to the input of the RLC filter.
* @param self self reference to HBridgeFSM
* @param e    event
*/
void HBridge_Zero(HBridge *self, Event *e);

/**
* Implements the state handler for the case that the H-Bridge
* is supplying -Vdc to the input of the RLC filter.
* @param self self reference to HBridgeFSM
* @param e    event
*/
void HBridge_negVDC(HBridge *self, Event *e);

/**
 * Implements the transition logic for a particular state machine. Uses a switch to
 * respond to various inputs depending on the current state of the machine.
 *
 * @param self self reference to HBridgeFSM
 * @param e    event
 */

char HBridgeTransitionFunction(HBridge self, HBridgeEvent *e, StateVariable currentState);

#endif

