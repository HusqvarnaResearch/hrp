/*************************************************************
 *
 *   hq_Container.h
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
 * Container.h
 *
 *  Created on: Nov 27, 2013
 *      Author: dan
 */

#ifndef HQ_CONTAINER_H_
#define HQ_CONTAINER_H_

#include <string>
#include <iostream>
#include <map>
#include <string>
#include <sstream>

#define DOT_FILE_EXT ".xot"

class Container{
	static int& id_counter(){static int c; return c; }
public:
	static void reset_id_counter(){ id_counter()=0; }
	static std::string get_id_counter(){ std::stringstream s; s<<id_counter()++; return s.str();}
	static std::map<std::string, std::string>& map_id_to_number(){ static std::map<std::string, std::string> m; return m; }
	static std::stringstream& postData(){ static std::stringstream s; return s; }
	static void clear_postData(){
		postData().str(""); }

	virtual ~Container(){}

	virtual bool contains(std::string name)const=0;

	virtual std::string copy(std::string name)const=0;

	virtual void saveXml(std::ostream& out, std::string tab, std::string name, std::string id)const=0;

	virtual void saveDot(std::ostream& out, std::string tab, std::string name, std::string id)const=0;

	virtual void map_ids(std::string name, std::string id)const=0;
};


#endif /* HQ_CONTAINER_H_ */
