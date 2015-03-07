/*
 * panelFSM.h
 *
 *  Created on: Mar 5, 2015
 *      Author: watchmen
 */

#ifndef PANELFSM_H_
#define PANELFSM_H_


//////////////////////////////////////////
// Declaration of FSM and Event objects //
//////////////////////////////////////////

typedef struct Panel Panel;
typedef struct PanelEvent PanelEvent;

struct Panel {
    Fsm super_; /* extend the Fsm class */
    //Attributes
};

struct PanelEvent {
    Event super_; /* extend the Event class */
    //Attributes
    char code;

};

///////////
//Events //
///////////

/* signals used by the Panel FSM - Algorithm either runs, or it does not. This is a submachine of the inverterFSM.*/
enum {
    TIMER_PANEL,
    NO_EVENT_PANEL
};

////////////////////////////////////
// Declaration of state functions //
////////////////////////////////////

/**
* @brief Constructor Function
* This function is to be used for 'instantiating' state machines
* @code(.c)
*     Panel Panel;
*     PanelCtor(&Panel);
* @endcode
* PanelCtor is a wrapper function for a call to:
* @code
*     _FsmCtor_(&self->super_, &Panel_initial);
* @endcode
* which uses the 'super class'
*
* @param self
*/
void PanelCtor(Panel *self);
/**
* @brief Entry state to be used for initialization and setup of the state machine.
*
* Implements the initial transition of the Panel FSM. To be used for initializations
* and setup of the machine. Can also serve no function but to transition to the default or
* zero states.
*
* @param self self reference to PanelFSM
* @param e    event
*/
void Panel_initial(Panel *self, Event *e);

/**
* Implements the default transition
* @param self self reference to PanelFSM
* @param e    event
*/
void Panel_Dashboard(Panel *self, Event *e);

/**
* Implements the state handler for the case that the Panel
* algorithm is no longer running.
*
* @param self self reference to PanelFSM
* @param e    event
*/
void Panel_Connect(Panel *self, Event *e);


/**
* Implements the state handler for the LED blink routine; provides
* user feedback, shows that Panel is running.
*
* @param self self reference to PanelFSM
* @param e    event
*/
void Panel_Emulator(Panel *self, Event *e);

char PanelTransitionFunction(Panel self, PanelEvent *e);

#endif /* PANELFSM_H_ */
