/*
 * am_driver_safe_states.h
 *
 *  Created on: March 17, 2017
 *      Author: Kent Askenmalm
 */

#ifndef AUTOMOWER_SAFE_STATES_H_
#define AUTOMOWER_SAFE_STATES_H_



#include <hq_decision_making/hq_FSM.h>
#include <hq_decision_making/hq_ROSTask.h>
#include <hq_decision_making/hq_DecisionMaking.h>

#include <am_driver_safe/automower_safe.h>

namespace Husqvarna { 

extern void ConnectDriverAndStates(AutomowerSafePtr driver);

FSM_HEADER(AutoMowerSafeStates);

}
#endif /* AUTOMOWER_SAFE_STATES_H_ */
