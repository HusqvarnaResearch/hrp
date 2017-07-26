 /*************************************************************
 *
 *   hq_DecisionMaking.h
 *
 *   Adapted for Husqvarna Research Platform
 * 
 *          March 23, 2017
 *     by:  Kent Askenmalm, Husqvarna Group
 *  
 *
 * Code origin: https://github.com/cogniteam/decision_making
 *
 * original header below
 ************************************************************/
/*
 * DecisionMaking.h
 *
 *  Created on: Nov 20, 2013
 *      Author: dan
 */

#ifndef HQ_DECISIONMAKING_H_
#define HQ_DECISIONMAKING_H_

#ifndef DECISION_MAKING_EVENTS_MACROSES
#define DECISION_MAKING_EVENTS_MACROSES
#warning Decision Making: On decisions states changes reports are disabled.

#define DM_SYSTEM_STOP

#define ON_FSM_START(NAME, CALLS, EVENTS)
#define ON_FSM_END(NAME, CALLS, EVENTS, RESULT)

#define ON_FSM_STATE_START(NAME, CALLS, EVENTS)
#define ON_FSM_STATE_END(NAME, CALLS, EVENTS)

#define ON_BT_NODE_START(NAME, TYPE, CALLS, EVENTS)
#define ON_BT_NODE_END(NAME, CALLS, EVENTS, RESULT)

#define ON_TAO_START(NAME, CALLS, EVENTS)
#define ON_TAO_END(NAME, CALLS, EVENTS, RESULT)

#define ON_TAO_STATE_START(NAME, CALLS, EVENTS)
#define ON_TAO_STATE_END(NAME, CALLS, EVENTS)

#endif

#ifndef CALL_REMOTE
#error Decision Making: CALL_REMOTE macros does not defined
#endif


#endif /* HQ_DECISIONMAKING_H_ */
