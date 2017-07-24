/*************************************************************
 *
 *   hq_FSMConstructor.h
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
 * FSMConstructor.h
 *
 *  Created on: Nov 27, 2013
 *      Author: dan
 */

#ifndef HQ_FSMCONSTRUCTOR_H_
#define HQ_FSMCONSTRUCTOR_H_

#include <deque>
#include <map>
#include <sstream>
#include <vector>
#include <iostream>
#include "hq_Container.h"
#include <deque>

namespace fsm_constructor{
using namespace std;

class FSMConstructor;
class Element{
public:
	mutable FSMConstructor const* lib;
	mutable string tab;
	Element():lib(0){}
};

class Raise:public Element{
public:
	std::string text;
};

class EventAction:public Element{
public:
	std::string type;
	std::string text;
	~EventAction(){}
};

class OnEvent:public Element{
public:
	mutable std::string id;
	std::string type;
	std::string text;
	std::vector<EventAction> actions;
};

class Call:public Element{
public:
	mutable std::string id;
	std::string type;
	std::string text;
	int line, pos;
	std::string file;

	string getId()const{ return id+"/"+text; }
};

class States:public Element{
public:
	mutable std::string id;
	std::string name;
	std::vector<Call> calls;

	std::vector<Raise> raises;

	std::vector<OnEvent> events;

	std::deque<OnEvent> stack_events;

	void create_event(){ stack_events.push_back(OnEvent()); }
	void add_event(){ events.push_back(event()); drop_event(); }
	void drop_event(){ stack_events.pop_back(); }
	OnEvent& event(){ return stack_events.back(); }

	std::string getId()const{ return id+"/"+name; }
};

class Fsm:public Element{
public:
	mutable std::string id;
	std::string name;
	std::string start;
	std::vector<States> states;
	std::deque<States> stack;

	void create(){
		States st;
		if(stack.empty()){
			st.id = getId();
		}else{
			st.id = stack.back().getId();
		}
		stack.push_back(st);
	}
	void add(){ states.push_back(state()); drop(); }
	void drop(){ stack.pop_back(); }
	States& state(){ return stack.back(); }

	std::string getId()const{ return id+"/"+name; }
};

class FSMConstructor : public Element, public Container {
public:
	std::map<std::string, Fsm> fsms;
	std::deque<Fsm> stack;
	Container* trees;
	std::stringstream& errors;
	std::string filename;
	//mutable std::map< std::string, std::deque<std::string> > name_to_id;

	FSMConstructor(std::stringstream& errors, std::string filename):trees(0), errors(errors), filename(filename){}

	void create(){
		stack.push_back(Fsm());
	}
	void drop(){
		stack.pop_back();
	}
	void add(){
		fsms[fsm().name] = fsm();
		drop();
	}
	Fsm& fsm(){ return stack.back(); }

	bool contains_fsm(string name)const{
		return contains(name);
	}
	std::string copy_fsm(std::string name)const{
		return copy(name);
	}

	bool contains_tree(string name)const{
		return trees->contains(name);
	}

	std::string copy_tree(std::string name)const{
		return trees->copy(name);
	}

	void saveXml_fsm(std::ostream& out, std::string tab, std::string name, std::string id)const{
		saveXml(out, tab, name, id);
	}
	void saveXml_tree(std::ostream& out, std::string tab, std::string name, std::string id)const{
		trees->saveXml(out, tab, name, id);
	}


	bool contains(std::string name)const{ return fsms.find(name)!=fsms.end(); }
	std::string copy(std::string name)const;
	void saveXml(std::ostream& out, std::string tab, std::string name, std::string id)const;



	void saveDot_fsm(std::ostream& out, std::string tab, std::string name, std::string id)const{
		saveDot(out, tab, name, id);
	}
	void saveDot_tree(std::ostream& out, std::string tab, std::string name, std::string id)const{
		trees->saveDot(out, tab, name, id);
	}

	void saveDot(std::ostream& out, std::string tab, std::string name, std::string id)const;

	void map_ids_tree(std::string name, std::string id)const{trees->map_ids(name, id);}
	void map_ids_fsm(std::string name, std::string id)const{map_ids(name, id);}
	void map_ids(std::string name, std::string id)const;
};

//==========================================================================

std::ostream& operator<<(std::ostream& out, const Raise& o);

std::ostream& operator<<(std::ostream& out, const EventAction& o);

std::ostream& operator<<(std::ostream& out, const OnEvent& o);

std::ostream& operator<<(std::ostream& out, const Call& o);
std::ostream& operator<<(std::ostream& out, const States& o);

std::ostream& operator<<(std::ostream& out, const Fsm& o);

std::ostream& operator<<(std::ostream& out, const FSMConstructor& o);

std::ostream& operator<<(std::ostream& out, const Call& o);

//======================================================================

std::ostream& saveXml(std::ostream& out, const Raise& o);

std::ostream& saveXml(std::ostream& out, const EventAction& o);

std::ostream& saveXml(std::ostream& out, const OnEvent& o);

std::ostream& saveXml(std::ostream& out, const States& o);

std::ostream& saveXml(std::ostream& out, const Fsm& o);

std::ostream& saveXml(std::ostream& out, const Call& o);

std::ostream& saveXml(std::ostream& out, const FSMConstructor& o);

void saveXml(std::string prefix, const FSMConstructor& o);


//======================================================================

std::ostream& saveDot(std::ostream& out, const Raise& o);

std::ostream& saveDot(std::ostream& out, const EventAction& o);

std::ostream& saveDot(std::ostream& out, const OnEvent& o, const States& state, const Fsm& fsm);

std::ostream& saveDot(std::ostream& out, const States& o, const Fsm& fsm);

std::ostream& saveDot(std::ostream& out, const Fsm& o);

std::ostream& saveDot(std::ostream& out, const Call& o);

std::ostream& saveDot(std::ostream& out, const FSMConstructor& o);

void saveDot(std::string prefix, const FSMConstructor& o);


void map_ids(const Call& o);
void map_ids(const States& o);
void map_ids(const Fsm& o);
void map_ids(const FSMConstructor& o);

std::string searchSimpleState(const States& state);
std::string searchSimpleState(const Container* cnt_fsm, std::string name);

}

void xml_version(std::ostream& out, std::string tab);

#endif /* HQ_FSMCONSTRUCTOR_H_ */
