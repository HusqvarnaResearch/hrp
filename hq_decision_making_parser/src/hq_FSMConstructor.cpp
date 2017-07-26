/*************************************************************
 *
 *   hq_FSMConstructor.cpp
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
 * FSMConstructor.cpp
 *
 *  Created on: Nov 29, 2013
 *      Author: dan
 */

#include "hq_FSMConstructor.h"
#include "hq_ParserExceptions.h"

#include <fstream>

#define XML( X ) out<<o.tab<<#X<<endl
#define CML( X ) out<<o.tab<<X<<endl
#define S( X ) <<#X


void xml_version(std::ostream& out, std::string tab){
	out<<tab<<"<?xml version=\"1.0\"?>";
}

namespace fsm_constructor{
using namespace std;

void xml_scxml_bgn(std::ostream& out, std::string tab, std::string name, std::string start){
	string _name="";
	string _start="";
	if(name.size()>0) _name = " id=\""+name+"\"";
	if(start.size()>0)_start= " initialstate=\""+start+"\"";
	out<<tab<<"<scxml"<<_name<<_start<<">";
}
void xml_scxml_end(std::ostream& out, std::string tab){
	out<<tab<<"</scxml>";
}
void xml_state_bgn(std::ostream& out, std::string tab, std::string name, std::string start, std::string id){
	string _name="";
	string _start="";
	string _id="";
	if(name.size()>0) _name = " name=\""+name+"\"";
	if(start.size()>0)_start= " initialstate=\""+start+"\"";
	if(id.size()>0)_id= " id=\""+id+"\"";
	out<<tab<<"<state"<<_name<<_start<<_id<<">";
}
void xml_state_end(std::ostream& out, std::string tab){
	out<<tab<<"</state>";
}
void xml_transition_bgn(std::ostream& out, std::string tab, std::string event, std::string target){
	string _event="";
	string _target="";
	if(event.size()>0) _event = " event=\""+event+"\"";
	if(target.size()>0)_target= " target=\""+target+"\"";
	out<<tab<<"<transition"<<_event<<_target<<">";
}
void xml_transition_end(std::ostream& out, std::string tab){
	out<<tab<<"</transition>";
}
void xml_parallel_bgn(std::ostream& out, std::string tab){
	out<<tab<<"<parallel>";
}
void xml_parallel_end(std::ostream& out, std::string tab){
	out<<tab<<"</parallel>";
}
void xml_onentry_bgn(std::ostream& out, std::string tab){
	out<<tab<<"<onentry>";
}
void xml_onentry_end(std::ostream& out, std::string tab){
	out<<tab<<"</onentry>";
}
void xml_send(std::ostream& out, std::string tab, string event){
	out<<tab<<"<send event=\""<<  event <<"\" />";
}
void xml_call_task(std::ostream& out, std::string tab, std::string name, std::string id){
	string _name="";
	string _id="";
	if(id.size()>0) _id = " id=\""+id+"\"";
	if(name.size()>0) _name = " name=\"TASK["+name+"]\"";
	out<<tab<<"<invoke"<<_name<<_id<<" />";
}


std::ostream& saveXml(std::ostream& out, const Raise& o){
	xml_send(out, o.tab, o.text);
	return out;
}

std::ostream& saveXml(std::ostream& out, const EventAction& o){
	if(o.type=="raise"){
		xml_send(out, o.tab, o.text);
	}
	return out;
}

std::ostream& saveXml(std::ostream& out, const OnEvent& o){

	string target="";
	int next_counter=0;
	for(std::vector<EventAction>::const_iterator i=o.actions.begin();i!=o.actions.end();i++){
		if(i->type=="next"){
			target=o.id + "/" + i->text;
			next_counter++;
		}
	}
	string tab="";
	xml_transition_bgn(out, o.tab, o.text, target);
	if(next_counter<(int)o.actions.size()){ out<<endl; tab=o.tab; }
	for(std::vector<EventAction>::const_iterator i=o.actions.begin();i!=o.actions.end();i++){
		i->lib=o.lib;
		i->tab=o.tab+"   ";
		if(i->type!="next")
			saveXml(out, *i)<<endl;
	}
	xml_transition_end(out, tab);
	return out;
}

std::ostream& saveXml(std::ostream& out, const States& o){
	//o.lib->name_to_id[o.name].push_back(o.getId());
	xml_state_bgn(out, o.tab, o.name, "", o.getId()); out<<endl;

	if(not o.raises.empty()){
		string tab = o.tab+"   ";
		xml_onentry_bgn(out, tab); out<<endl;
		for(std::vector<Raise>::const_iterator i=o.raises.begin();i!=o.raises.end();i++){
			i->lib=o.lib;
			i->tab=tab+"   ";
			saveXml(out, *i)<<endl;
		}
		xml_onentry_end(out, tab); out<<endl;
	}

	if(not o.calls.empty()){
		string tab = o.tab+"   ";
		if(o.calls.size()>1){ xml_parallel_bgn(out, tab); tab+="    ";  out<<endl; }
		for(std::vector<Call>::const_iterator i=o.calls.begin();i!=o.calls.end();i++){
			i->lib=o.lib;
			i->tab=tab;
			i->id = o.getId();
			saveXml(out, *i)<<endl;
		}
		if(o.calls.size()>1){ xml_parallel_end(out, o.tab+"   "); out<<endl; }
	}

	if(not o.events.empty()){
		for(std::vector<OnEvent>::const_iterator i=o.events.begin();i!=o.events.end();i++){
			i->lib=o.lib;
			i->tab=o.tab+"   ";
			i->id = o.id;
			saveXml(out, *i)<<endl;
		}
	}

	xml_state_end(out, o.tab);
	//o.lib->name_to_id[o.name].pop_back();
	return out;
}

std::ostream& saveXml(std::ostream& out, const Call& o){
	string resolved = "";
	if(o.type == "fsm"){
		bool isResolved = o.lib->contains_fsm(o.text);
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   FSM("<<o.text<<") definition does not found." <<endl;
			out<<o.tab<<"<error>"<<endl;
			out<<o.tab<<"   "<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			out<<o.tab<<"   "<<"   FSM("<<o.text<<") definition does not found." <<endl;
			out<<o.tab<<"</error>";
		}else{
			o.lib->saveXml_fsm(out, o.tab, o.text, o.id);
		}
		return out;
	}
	if(o.type == "bt"){
		bool isResolved = o.lib->contains_tree(o.text);
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   BT("<<o.text<<") definition does not found." <<endl;
			out<<o.tab<<"<error>"<<endl;
			out<<o.tab<<"   "<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			out<<o.tab<<"   "<<"   BT("<<o.text<<") definition does not found." <<endl;
			out<<o.tab<<"</error>";
		}else{
			o.lib->saveXml_tree(out, o.tab, o.text, o.id);
		}
		return out;
	}
	if(o.type == "task"){
		string tab=o.tab;
		xml_call_task(out, tab, o.text, o.getId());
	}
	return out;
}

std::ostream& saveXml(std::ostream& out, const Fsm& o){
	//o.lib->name_to_id[o.name].push_back(o.getId());
	xml_state_bgn(out, o.tab, o.name, o.start, o.getId()); out<<endl;
	for(std::vector<States>::const_iterator i=o.states.begin();i!=o.states.end();i++){
		i->lib=o.lib;
		i->tab=o.tab+"   ";
		i->id=o.getId();
		saveXml(out, *i)<<endl;
	}
	xml_state_end(out, o.tab);
	//o.lib->name_to_id[o.name].pop_back();
	return out;
}

std::ostream& saveXml(std::ostream& out, const FSMConstructor& o){
//	xml_version(out, o.tab); out<<endl;
	xml_scxml_bgn(out, o.tab, "", ""); out<<endl;
	for(std::map<std::string, Fsm>::const_iterator i=o.fsms.begin();i!=o.fsms.end();i++){
		i->second.lib = &o;
		i->second.tab=o.tab+"   ";
		saveXml(out, i->second)<<endl;
	}
	xml_scxml_end(out, o.tab);
	return out;
}

void saveXml(std::string path_prefix, const FSMConstructor& o){
	for(std::map<std::string, Fsm>::const_iterator i=o.fsms.begin();i!=o.fsms.end();i++){
		std::stringstream filename; filename << path_prefix << i->second.name <<".scxml";
		std::ofstream out(filename.str().c_str());
		if(out.is_open()){
			xml_version(out, o.tab); out<<endl;
			out<<"<!-- from file: "<<o.filename<<" -->"<<endl;
			xml_scxml_bgn(out, o.tab, "", ""); out<<endl;
			i->second.lib = &o;
			i->second.tab=o.tab+"   ";
			saveXml(out, i->second)<<endl;
			xml_scxml_end(out, o.tab);
			out.close();
		}else{
			throw PEFileNotCreated(filename.str());
		}
	}
}

//==========================================================================================



std::ostream& operator<<(std::ostream& out, const Raise& o){
	return
	out<<"            raise "<<" : "<<o.text;
}

std::ostream& operator<<(std::ostream& out, const EventAction& o){
	return
	out<<"               action "<<o.type<<" : "<<o.text;
}

std::ostream& operator<<(std::ostream& out, const OnEvent& o){
	out<<"            Event{ "<<endl;
	out<<"               type="<<o.type<<endl;
	out<<"               event="<<o.text<<endl;
	for(std::vector<EventAction>::const_iterator i=o.actions.begin();i!=o.actions.end();i++){
		i->lib=o.lib;
		out<<(*i)<<endl;
	}
	out<<"            }";
	return out;
}

std::ostream& operator<<(std::ostream& out, const Call& o);
std::ostream& operator<<(std::ostream& out, const States& o){
	out<<"      State{ "<<endl;
	out<<"         name="<<o.name<<endl;
	out<<"         calls:"<<endl;
	for(std::vector<Call>::const_iterator i=o.calls.begin();i!=o.calls.end();i++){
		i->lib=o.lib;
		out<<(*i)<<endl;
	}
	out<<"         raises:"<<endl;

	for(std::vector<Raise>::const_iterator i=o.raises.begin();i!=o.raises.end();i++){

		i->lib=o.lib;
		out<<(*i)<<endl;
	}
	out<<"         events:"<<endl;
	for(std::vector<OnEvent>::const_iterator i=o.events.begin();i!=o.events.end();i++){
		i->lib=o.lib;
		out<<(*i)<<endl;
	}
	out<<"      }";
	return out;
}

std::ostream& operator<<(std::ostream& out, const Fsm& o){
	out<<"FSM{ "<<endl;
	out<<"   name="<<o.name<<endl;
	out<<"   start="<<o.start<<endl;
	out<<"   States"<<endl;
	for(std::vector<States>::const_iterator i=o.states.begin();i!=o.states.end();i++){
		i->lib=o.lib;
		out<<(*i)<<endl;
	}
	out<<"}";
	return out;
}

std::ostream& operator<<(std::ostream& out, const FSMConstructor& o){
	for(std::map<std::string, Fsm>::const_iterator i=o.fsms.begin();i!=o.fsms.end();i++){
		i->second.lib = &o;
		out<<i->second<<endl;
	}
	return out;
}

std::ostream& operator<<(std::ostream& out, const Call& o){
	string resolved = "";
	if(o.type == "fsm"){
		bool isResolved = o.lib->contains_fsm(o.text);
		resolved = isResolved?"resolved":"not found";
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   FSM("<<o.text<<") definition does not found." <<endl;
		}
		return
			out<<"            call "<<o.type<<" : "<<o.text << " : " << resolved;
	}
	if(o.type == "bt"){
		bool isResolved = o.lib->contains_tree(o.text);
		resolved = isResolved?"resolved":"not found";
		if(not isResolved){
			o.lib->errors<<"in "<<o.file<<":"<<o.line<<":"<<o.pos<<endl;
			o.lib->errors<<"   BT("<<o.text<<") definition does not found." <<endl;
		}
		return
			out<<"            call "<<o.type<<" : "<<o.text << " : " << resolved;
	}
	if(o.type == "task"){
		return
			out<<"            call "<<o.type<<" : "<<o.text;
	}
	return out;
}



}


