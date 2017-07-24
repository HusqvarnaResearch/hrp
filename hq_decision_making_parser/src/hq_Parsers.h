/*************************************************************
 *
 *   hq_Parsers.h
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
 * Parsers.h
 *
 *  Created on: Nov 27, 2013
 *      Author: dan
 */

#ifndef HQ_PARSERS_H_
#define HQ_PARSERS_H_


#include "hq_FSMConstructor.h"

class FSMParser;

FSMParser* createFSM(std::string filename);

void del(FSMParser*);

fsm_constructor::FSMConstructor& parseFSM(FSMParser*);


#endif /* HQ_PARSERS_H_ */
