#ifndef H_BRIDGE_FSM_H
#define H_BRIDGE_FSM_H

/////////////////////////////////////////
//Declaration of FSM and Event objects //
/////////////////////////////////////////

typedef struct hBridge hBridge;
typedef struct hBridgeEvent hBridgeEvent;

struct hBridge
{
    Fsm super_; /* extend the Fsm class */
    //Attributes
};

struct hBridgeEvent
{
    Event super_; /* extend the Event class */
    //Attributes
    char code;
};


////////////////////////////////////
// Declaration of state functions //
////////////////////////////////////

/**
* @brief Constructor Function
* This function is to be used for 'instantiating' state machines
* @code(.c)
*     hBridge hBridge;
*     hBridgeCtor(&hBridge);
* @endcode
* hBridgeCtor is a wrapper function for a call to:
* @code
*     _FsmCtor_(&self->super_, &hBridge_initial);
* @endcode
* which uses the 'super class'
*
* @param self
*/
void hBridgeCtor(hBridge *self);
/**
* @brief Entry state to be used for initialization and setup of the state machine.
* 
* Implements the initial transition of the hBridge FSM. To be used for initializations
* and setup of the machine. Can also serve no function but to transition to the default or
* zero states. 
*
* @param self self reference to hBridgeFSM
* @param e    event
*/
void hBridge_initial(hBridge *self, Event *e);

/**
* Implements the default transition
* @param self self reference to hBridgeFSM
* @param e    event
*/
void hBridge_default(hBridge *self, Event *e);

/**
* Implements the state handler for the case that the H-Bridge
* is supplying +Vdc to the input of the RLC filter.
* @param self self reference to hBridgeFSM
* @param e    event
*/
void hBridge_VDC(hBridge *self, Event *e);

/**
* Implements the state handler for the case that the H-Bridge
* is supplying zero volts DC to the input of the RLC filter.
* @param self self reference to hBridgeFSM
* @param e    event
*/
void hBridge_Zero(hBridge *self, Event *e);

/**
* Implements the state handler for the case that the H-Bridge
* is supplying -Vdc to the input of the RLC filter.
* @param self self reference to hBridgeFSM
* @param e    event
*/
void hBridge_negVDC(hBridge *self, Event *e);

/**
 * Implements the transition logic for a particular state machine. Uses a switch to
 * respond to various inputs depending on the current state of the machine.
 *
 * @param self self reference to hBridgeFSM
 * @param e    event
 */

char hBridgeTransitionFunction(hBridge self, hBridgeEvent *e);

#endif

