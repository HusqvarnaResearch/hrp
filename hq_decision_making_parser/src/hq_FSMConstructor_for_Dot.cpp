/*************************************************************
 *
 *   hq_FSMConstructor_for_Dot.cpp
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


namespace fsm_constructor{
using namespace std;

std::ostream& saveDot(std::ostream& out, const Raise& o){
	return out;
}

std::ostream& saveDot(std::ostream& out, const EventAction& o){
	return out;
}



std::ostream& saveDot(std::ostream& out, const OnEvent& o, const States& state, const Fsm& fsm){

	string target="";
	int next_counter=0;
	for(std::vector<EventAction>::const_iterator i=o.actions.begin();i!=o.actions.end();i++){
		if(i->type=="next"){
			target=o.id + "/" + i->text;
			next_counter++;
		}
	}
	if(next_counter>0){
		string src="";
		string ltail="";
		string dst="";
		string lhead="";
		src = Container::map_id_to_number()[searchSimpleState(state)];
		if(state.calls.empty()){
			ltail= src;
		}else{
			ltail="cluster_"+Container::map_id_to_number()[state.getId()];
		}
		for(std::vector<States>::const_iterator i=fsm.states.begin();i!=fsm.states.end();i++){
			if(i->getId()!=target) continue;
			dst = Container::map_id_to_number()[searchSimpleState(*i)];
			if(i->calls.empty()){
				lhead= dst;
			}else{
				lhead="cluster_"+Container::map_id_to_number()[i->getId()];
			}
			break;
		}
		if(dst.size()>0){
			Container::postData()<<o.tab
					<< src
					<<" -> "
					<< dst
					<< "[ label=\"" << o.text <<"\" ltail=\""<< ltail <<"\" lhead=\""<< lhead << "\", fontsize=8 ]" <<endl;
			;
//			cout<<"// Edge " <<o.tab
//					<< src
//					<<" -> "
//					<< dst
//					<< "[ label=\"" << o.text <<"\" ltail=\""<< ltail <<"\" lhead=\""<< lhead << "\" ]" <<endl;

		}else{
			out<<o.tab<<"// target for edge : "<<target<< " not found"<<endl;
		}
	}

	return out;
}

std::ostream& saveDot(std::ostream& out, const States& o, const Fsm& fsm){
	if(o.calls.empty()){

		out<<o.tab<<Container::map_id_to_number()[o.getId()] << "[ label=\"" << o.name <<"\" ]" <<" //state: "<<o.getId()<<endl;
		if(not o.events.empty()){
			for(std::vector<OnEvent>::const_iterator i=o.events.begin();i!=o.events.end();i++){
				i->lib=o.lib;
				i->tab=o.tab;
				i->id = o.id;
				saveDot(out, *i, o, fsm)<<endl;
			}
		}
	}else{
		out<<o.tab<<"subgraph cluster_"<<Container::map_id_to_number()[o.getId()]<<"{ //state: "<<o.getId()<<endl;
		out<<o.tab<<"   label=\""<<o.name<<"\""<<endl;
		if(not o.calls.empty()){
			string tab = o.tab+"   ";
			for(std::vector<Call>::const_iterator i=o.calls.begin();i!=o.calls.end();i++){
				i->lib=o.lib;
				i->tab=tab;
				i->id = o.getId();
				saveDot(out, *i)<<endl;
			}
		}

		if(not o.events.empty()){
			for(std::vector<OnEvent>::const_iterator i=o.events.begin();i!=o.events.end();i++){
				i->lib=o.lib;
				i->tab=o.tab+"   ";
				i->id = o.id;
				saveDot(out, *i, o, fsm)<<endl;
			}
		}

		out<<o.tab<<"}";
	}
	return out;
}

std::ostream& saveDot(std::ostream& out, const Call& o){
	string resolved = "";
	if(o.type == "fsm"){
		bool isResolved = o.lib->contains_fsm(o.text);
		if(isResolved){
			o.lib->saveDot_fsm(out, o.tab, o.text, o.id);
		}
		return out;
	}
	if(o.type == "bt"){
		bool isResolved = o.lib->contains_tree(o.text);
		if(isResolved){
			out<<o.tab<<"subgraph cluster_"<<Container::map_id_to_number()[o.getId()]<<"{ //call bt : "<<o.getId()<<endl;
			out<<o.tab<<"   label=\"BT["<<o.text<<"]\""<<endl;
			o.lib->saveDot_tree(out, o.tab+"    ", o.text, o.id);
			out<<o.tab<<"}";
		}
		return out;
	}
	if(o.type == "task"){
		string tab=o.tab;
		out<<o.tab<<Container::map_id_to_number()[o.getId()]<<" [label=\""<<o.text<<"\"]  //call: "<<o.getId();
	}
	return out;
}

std::ostream& saveDot(std::ostream& out, const Fsm& o){
	string start_index = "str"+Container::map_id_to_number()[o.getId()];
	out<<o.tab<<"subgraph cluster_"<<Container::map_id_to_number()[o.getId()]<<"{ //fsm: "<<o.getId()<<endl;
	out<<o.tab<<"   label=\""<<o.name<<"\""<<endl;
	out<<o.tab<<"   "<<start_index<<" [shape=point]"<<endl;
	for(std::vector<States>::const_iterator i=o.states.begin();i!=o.states.end();i++){
		i->lib=o.lib;
		i->tab=o.tab+"   ";
		i->id=o.getId();
		saveDot(out, *i, o)<<endl;
	}
	{
		string src=start_index;
		string ltail=src;
		string dst="";
		string lhead="";
		for(std::vector<States>::const_iterator i=o.states.begin();i!=o.states.end();i++){
			if(i->getId()!=o.getId()+"/"+o.start) continue;
			dst = Container::map_id_to_number()[searchSimpleState(*i)];
			if(i->calls.empty()){
				lhead= dst;
			}else{
				lhead="cluster_"+Container::map_id_to_number()[i->getId()];
			}
			break;
		}
		if(dst.size()>0){
			Container::postData()<<o.tab
					<< src
					<<" -> "
					<< dst
					<< "[ lhead=\""<< lhead << "\", fontsize=8 ] //start" <<endl;
			;
		}
	}
	out<<o.tab<<"}";
	return out;
}


void printIds(){
//	std::cout<<"\nPrint all ids map (FSM): {"<<endl;
//	for(std::map<std::string, std::string>::const_iterator i=Container::map_id_to_number().begin();i!=Container::map_id_to_number().end();i++){
//		std::cout<<"    "<< i->first <<" :=  "<< i->second <<std::endl;
//	}
//	std::cout<<"}"<<std::endl;
}


std::ostream& saveDot(std::ostream& out, const FSMConstructor& o){
	for(std::map<std::string, Fsm>::const_iterator i=o.fsms.begin();i!=o.fsms.end();i++){
		i->second.lib = &o;
		i->second.tab=o.tab+"   ";
		map_ids(i->second); printIds();
		Container::clear_postData();
		out<<o.tab<<"digraph "<<i->second.name<<" { //fsm"<<endl;
		out<<o.tab<<"   compound=\"true\""<<endl;
		saveDot(out, i->second)<<endl;
		out<<o.tab<<Container::postData().str()<<endl;
		out<<o.tab<<"}";
	}
	return out;
}

void saveDot(std::string path_prefix, const FSMConstructor& o){
	for(std::map<std::string, Fsm>::const_iterator i=o.fsms.begin();i!=o.fsms.end();i++){
		std::stringstream filename; filename << path_prefix << i->second.name <<DOT_FILE_EXT;
		std::ofstream out(filename.str().c_str());
		if(out.is_open()){
			i->second.lib = &o;
			i->second.tab=o.tab+"   ";
			map_ids(i->second); printIds();
			Container::clear_postData();
			out<<"//from file: "<<o.filename<<" "<<endl;
			out<<o.tab<<"digraph "<<i->second.name<<" { //fsm"<<endl;
			out<<o.tab<<"   compound=\"true\""<<endl;
			saveDot(out, i->second)<<endl;
			out<<o.tab<<Container::postData().str()<<endl;
			out<<o.tab<<"}"<<endl;
			out.close();
		}else{
			throw PEFileNotCreated(filename.str());
		}
	}
}

//----------------------------------

void map_ids(const Call& o){
	Container::map_id_to_number()[o.getId()] = Container::get_id_counter();
	if(o.type == "fsm"){
		bool isResolved = o.lib->contains_fsm(o.text);
		if(isResolved) o.lib->map_ids_fsm(o.text, o.id);
	}
	if(o.type == "bt"){
		bool isResolved = o.lib->contains_tree(o.text);
		if(isResolved) o.lib->map_ids_tree(o.text, o.id);
	}
}


void map_ids(const States& o){
	Container::map_id_to_number()[o.getId()] = Container::get_id_counter();
	if(not o.calls.empty()){
		for(std::vector<Call>::const_iterator i=o.calls.begin();i!=o.calls.end();i++){
			i->lib=o.lib;
			i->id = o.getId();
			map_ids(*i);
		}
	}
}

void map_ids(const Fsm& o){
	Container::map_id_to_number()[o.getId()] = Container::get_id_counter();
	for(std::vector<States>::const_iterator i=o.states.begin();i!=o.states.end();i++){
		i->lib=o.lib;
		i->id = o.getId();
		map_ids(*i);
	}
}

void map_ids(const FSMConstructor& o){
	Container::reset_id_counter();
	Container::map_id_to_number().clear();
	for(std::map<std::string, Fsm>::const_iterator i=o.fsms.begin();i!=o.fsms.end();i++){
		i->second.lib = &o;
		map_ids(i->second);
	}
}


std::string searchSimpleState(const States& o){
	if(not o.calls.empty()){
		for(std::vector<Call>::const_iterator i=o.calls.begin();i!=o.calls.end();i++){
			if(i->type=="task"){
				return i->getId();
			}
			if(i->type=="bt"){
				return i->getId()+"/"+i->text;
			}
			if(i->type=="fsm"){
				bool isResolved = o.lib->contains_fsm(i->text);
				if(isResolved){
					const Fsm& fsm = o.lib->fsms.at(i->text);
					return searchSimpleState(fsm.states.front());
				}
			}
		}
	}else{
		return o.getId();
	}
	return "NOT-FOUND";
}
std::string searchSimpleState(Container const* cnt_fsm, std::string name){
	FSMConstructor const* con = (FSMConstructor const*) cnt_fsm;
	const Fsm& fsm = con->fsms.at(name);
	return searchSimpleState(fsm.states.front());
}


}


