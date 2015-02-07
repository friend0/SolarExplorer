/**
 * @file fsm.h
 * @author Ryan A. Rodriguez
 * @brief Class and Function defintions required to define FSMs using this framework
 *
 * @TODO put extended description here
 *
 * @TODO put code example here
 * instantiate
 * initialize
 * state definitions
 */

#ifndef FSM_H
#define FSM_H

/**
 * Event Class Definition
 */
typedef struct Event Event;

/**
 * FSM class definition
 */
typedef struct Fsm Fsm;

/**
 * @TODO - put a good description of the signal variable here
 */
typedef short Signal;

/**
 * @TODO need to comb through the C magic going on here and
 * 		 thoroughly document what is going on.
 * The following is a pointer to a state handler function. The state handler is 
 * mereley a function that uses a switch to respond to signals. 
 */
typedef void (*State)(Fsm *, Event const *);

/** 
* @brief Event base class
* @type Event
* @member signal
* 
* This struct is the base class for events. It has a single attribute
* called signal.
*
* We should note that the signal 's' in set K is an invariant, for an
* input voltage Vin.
* \f$
* s \in K, \\
* K = [0,Vin]
* \f$
* More accurately, Vin will be scaled corresponding to the reference voltage
* at the ADC.
*/
struct Event
{
Signal signal;
};

/** 
* @brief FSM base class
* @type FSM
* @member state__
* 
* This struct is the base class for Finite State Machines. It has a single attribute
* called state__. State__, ostensibly, should never be accessed by the client directly.
*
*/
/* Finite State Machine base class */
struct Fsm
{
State state__; /* the current state */
};


/////////////////////////////////////
// 'Inlined' methods of Fsm class  //
/////////////////////////////////////

/**
* @brief Constructor selfthod, initializes state
* 
* Get the state pointed to by the Fsm contained by 'self_' and point it to 
* desired initial state.
*
* @param self_ reference to 'self' object, specifies the object being operated on
* @param init_ Desired Initial state
*/
#define _FsmCtor_(self_, init_) ((self_)->state__ = (State)(init_))

/**
* @brief Initializes state machine
*
* Dereference the object wrapping the Fsm, get the current state.
* The current state is a function to which we can pass arguments.
*
* @param self_ reference to 'self' object, specifies the object being operated on
* @param e_	Initial Event to be passed to the current state function pointed to by Fsm
*           object being pointed to by 'self_'
*/
#define FsmInit(self_, e_) (*(self_)->state__)((self_), (e_))

/**
* @brief Dispatches events to the state machine
*
* Fsm Dispatch should be called after the Event signal is determined. Dispatch function 
* gets the Fsm object reference by 'self' and passes a new event to the current state. 
* Note that the current state is actually a function that takes in a reference to self, and a new event. 
* This state function responds appropriately to new events, and performs transition based on the signal. 
*
* EX: (*(self_)->state__) <- is actually a function! A state function to be precise...
* So, (*(self_)->state__)(args1,..., argsN) calls the function with the given arguments.
* 
* @param self_ reference to 'self' object, specifies the object being operated on 
* @param e_	The Event to be passed to the current state function pointed to by Fsm 
*           contained by 'self_'
*/
#define FsmDispatch(self_, e_) (*(self_)->state__)((self_), (e_))

/**
 * @brief function used to perform a state transition
 * 
* This makes a state transition, i.e. updates the state to q+. 
* This function should be called within each individual state base upon 
* input event, and current state. 
*
* State transitions are accomplished by retrieving the current sate pointed to by 'self_', 
* then setting it equal to the desired nextState targ_.
*
* @param self eference to 'self' object, specifies the object being operated on 
* @param targ_ The target state to be transitioned to
*/
#define _FsmTran_(self_, targ_) ((self_)->state__ = (State)(targ_))

#endif