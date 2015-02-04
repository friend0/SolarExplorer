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
 *
 * I think this is just a way to make state into a type definition
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


/**
* 'inlined' selfthods of Fsm class
* */

/**
* @brief Constructor selfthod, initializes state
* Get the initial state and point it to initial state
*/
#define _FsmCtor_(self_, init_) ((self_)->state__ = (State)(init_))

/**
* @brief Initializes state machine
*
* @param self_ reference to 'self' object, specifies the object being operated on
* @param e_	Initial Event
*/
#define FsmInit(self_, e_) (*(self_)->state__)((self_), (e_))

/**
* @brief Dispatches events to the state machine
*
* @param self_ reference to 'self' object, specifies the object being operated on 
* @param e_	The Event being passed to the FSM contained by 'self_'
*/
#define FsmDispatch(self_, e_) (*(self_)->state__)((self_), (e_))

/**
* This takes a state transition, i.e. updates the state
* q+
*/
#define _FsmTran_(self_, targ_) ((self_)->state__ = (State)(targ_))

#endif