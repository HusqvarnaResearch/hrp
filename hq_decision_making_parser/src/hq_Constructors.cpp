 /*************************************************************
 *
 *   hq_Constructors.cpp
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
 * Constructors.cpp
 *
 *  Created on: Nov 28, 2013
 *      Author: dan
 */


#include "hq_FSMConstructor.h"


namespace fsm_constructor{

std::string FSMConstructor::copy(std::string name)const{
	std::stringstream s; s<<fsms.at(name); return s.str();
}

void FSMConstructor::saveXml(std::ostream& out, std::string tab, std::string name, std::string id)const{
	const Fsm& fsm = fsms.at(name);
	fsm.lib = this;
	fsm.tab = tab;
	fsm.id = id;
	fsm_constructor::saveXml(out, fsm);
}

void FSMConstructor::saveDot(std::ostream& out, std::string tab, std::string name, std::string id)const{
	const Fsm& fsm = fsms.at(name);
	fsm.lib = this;
	fsm.tab = tab;
	fsm.id = id;
	fsm_constructor::saveDot(out, fsm);
}

void FSMConstructor::map_ids(std::string name, std::string id)const{
	const Fsm& fsm = fsms.at(name);
	fsm.lib = this;
	fsm.id = id;
	fsm_constructor::map_ids(fsm);
}

}


