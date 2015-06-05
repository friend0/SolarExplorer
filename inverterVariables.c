/*
 * inverterVariables.c
 *
 *  Created on: Mar 26, 2015
 *      Author: watchmen
 */

#include "inverterVariables.h"


/**
 * The initState function is to be used during setup to init the StateVariable struct and all of its constituent members.
 * 
 * @param s       The StateVariable to be set
 * @param current Current reading
 * @param voltage Voltage reading
 * @param phase   Phase reading
 * @param controller  Global or Forward Controller - typically initialized to Global controller as it is the safer choice
 * @param bridgeState the current state at the hBridge - initialize to zero is the safest choice
 */
void initState(StateVariable *s){
  s->current = 0;
  s->voltage = 0;
  s->phase = 0;
  s->controller = GLOBAL;
  s->bridgeState = ZERO_VDC;
}

/**
 * The updateState() function is used to quickly set the state variable
 * with data collected in the inverter interrupt. Is used to set a variable of type StateVariable.
 *
 * @todo fiure out the best type for the readings. I.e pass in IQ format or as long. 
 * 
 * @param s       The StateVariable to be set
 * @param current Current reading
 * @param voltage Voltage reading
 * @param phase   Phase reading
 */
void updateState(StateVariable *s, long current, long voltage, long phase){
  s->current = current;
  s->voltage = voltage;
  s->phase = phase;
}
