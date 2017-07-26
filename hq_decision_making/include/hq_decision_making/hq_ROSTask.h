 /*************************************************************
 *
 *   hq_ROSTask.h
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
 * ROSTask.h
 *
 *  Created on: Nov 5, 2013
 *      Author: dan
 */

#ifndef HQ_ROSTASK_H_
#define HQ_ROSTASK_H_

#include "hq_EventSystem.h"
#include "hq_TaskResult.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <map>
#include <boost/function.hpp>

namespace decision_making{

class LocalTasks{
	typedef decision_making::TaskResult (* callTask)(std::string, const decision_making::CallContext&, decision_making::EventQueue&);
	typedef std::map<std::string, callTask> callbacks;
	typedef boost::function<decision_making::TaskResult (std::string, const decision_making::CallContext&, decision_making::EventQueue&) > callTask_fun;
	typedef std::map<std::string, callTask_fun> callbacks_fun;
	static callbacks& get(){ static callbacks t; return t; }
	static callbacks_fun& get_fun(){ static callbacks_fun t; return t; }
public:
	typedef callTask_fun Function;
	static void registrate(std::string task_name, callTask cb){
		get()[task_name]=cb;
	}
	static void registrate(std::string task_name, Function cb){
		get_fun()[task_name]=cb;
	}
	static bool registrated(std::string task_name){
		return get().find(task_name)!=get().end() or get_fun().find(task_name)!=get_fun().end();
	}
	static void unregistrated(std::string task_name){
		if( get().find(task_name)!=get().end() )
			get().erase(get().find(task_name));
		if( get_fun().find(task_name)!=get_fun().end() )
			get_fun().erase(get_fun().find(task_name));
	}
	static decision_making::TaskResult call(
			std::string task_name,
			std::string task_address,
			const decision_making::CallContext& call_ctx,
			decision_making::EventQueue& events
	){
		if(get().find(task_name)!=get().end())
			return get()[task_name](task_address, call_ctx, events);
		if(get_fun().find(task_name)!=get_fun().end())
			return get_fun()[task_name](task_address, call_ctx, events);
		return decision_making::TaskResult::FAIL(1000,"Task Not Registrated As Local Task");
	}
};


decision_making::TaskResult callTask(std::string task_address, const decision_making::CallContext& call_ctx, decision_making::EventQueue& events);
#define CALL_REMOTE(NAME, CALLS, EVENTS) boost::bind(&decision_making::callTask, #NAME, CALLS, EVENTS)

#define DECISION_MAKING_EVENTS_MACROSES

#define DM_SYSTEM_STOP and ros::ok()

#define ON_FUNCTION(FNAME) void FNAME(std::string name, std::string type, const decision_making::CallContext& call_ctx, decision_making::EventQueue& events, decision_making::TaskResult result)
#define NO_RESULT decision_making::TaskResult::UNDEF()

ON_FUNCTION(on_fsm_start);
#define ON_FSM_START(NAME, CALLS, EVENTS) on_fsm_start(NAME, "FSM", CALLS, EVENTS, NO_RESULT)

ON_FUNCTION(on_fsm_end);
#define ON_FSM_END(NAME, CALLS, EVENTS, RESULT) on_fsm_end(NAME, "FSM", CALLS, EVENTS, RESULT)


ON_FUNCTION(on_fsm_state_start);
#define ON_FSM_STATE_START(NAME, CALLS, EVENTS) on_fsm_state_start(NAME, "FSM_STATE", decision_making::CallContext(CALLS,NAME), EVENTS, NO_RESULT)
ON_FUNCTION(on_fsm_state_end);
#define ON_FSM_STATE_END(NAME, CALLS, EVENTS) on_fsm_state_end(NAME, "FSM_STATE", decision_making::CallContext(CALLS,NAME), EVENTS, NO_RESULT)


ON_FUNCTION(on_bt_node_start);
#define ON_BT_NODE_START(NAME, TYPE, CALLS, EVENTS) on_bt_node_start(NAME, std::string("BT_")+TYPE, CALLS, EVENTS, NO_RESULT)
ON_FUNCTION(on_bt_node_end);
#define ON_BT_NODE_END(NAME, CALLS, EVENTS, RESULT) on_bt_node_end(NAME, "BT_NODE", CALLS, EVENTS, RESULT)


ON_FUNCTION(on_tao_tree_start);
#define ON_TAO_START(NAME, CALLS, EVENTS) on_tao_tree_start(NAME, "TAO", CALLS, EVENTS, NO_RESULT)
ON_FUNCTION(on_tao_tree_end);
#define ON_TAO_END(NAME, CALLS, EVENTS, RESULT) on_tao_tree_end(NAME, "TAO", CALLS, EVENTS, RESULT)

ON_FUNCTION(on_tao_plan_start);
#define ON_TAO_STATE_START(NAME, CALLS, EVENTS) on_tao_plan_start(NAME, "TAO_PLAN", decision_making::CallContext(CALLS,NAME), EVENTS, NO_RESULT)
ON_FUNCTION(on_tao_plan_end);
#define ON_TAO_STATE_END(NAME, CALLS, EVENTS) on_tao_plan_end(NAME, "TAO_PLAN", decision_making::CallContext(CALLS,NAME), EVENTS, NO_RESULT)


class RosConstraints{
public:

	static ros::Publisher& getAdder(){
		static ros::Publisher p = ros::NodeHandle().advertise<std_msgs::String>("/scriptable_monitor/add_script", 100);
		return p;
	}
	static ros::Publisher& getRemover(){
		static ros::Publisher p = ros::NodeHandle().advertise<std_msgs::String>("/scriptable_monitor/remove_script", 100);
		return p;
	}


	RosConstraints(string name, string script)
	: script_name(name), script(script)
	{
		std::stringstream st; st<<"#! name "<<name<<"\n#! type predicate\n"<<script;
		std_msgs::String msg; msg.data = preproc(st.str());
		getAdder().publish(msg);
	}
	~RosConstraints(){
		std_msgs::String msg; msg.data = script_name;
		getRemover().publish(msg);
	}


	std::string preproc(std::string txt)const;

private:
	std::string script_name;
	std::string script;
};

#define CONSTRAINTS(NAME, SCRIPT) \
		RosConstraints ros_constraints_##NAME(call_ctx.str()+"/"#NAME, #SCRIPT);


class RosEventQueue:public decision_making::EventQueue{
	ros::Publisher publisher;
	ros::Subscriber subscriber;
	bool do_not_publish_spin;
public:
	RosEventQueue();
	RosEventQueue(EventQueue* parent);
	RosEventQueue(EventQueue* parent, bool isTransit);

	void onNewEvent(const std_msgs::String::ConstPtr& msg);
	virtual void raiseEvent(const decision_making::Event& e);
	virtual bool check_external_ok(){return ros::ok();}

	void publish_spin_event(){ do_not_publish_spin = false; }

    virtual Event waitEvent(){
        boost::mutex::scoped_lock l(events_mutex);
        while(events_system_stop==false && events.empty() DM_SYSTEM_STOP)   on_new_event.timed_wait(l, boost::get_system_time()+ boost::posix_time::milliseconds(1000));
        events_system_stop = not (not events_system_stop DM_SYSTEM_STOP);
        if(events_system_stop)
            return Event();
        Event e = events.front();
        events.pop_front();
        return e;
    }
};

}

void ros_decision_making_init(int &argc, char **argv);

#endif /* HQ_ROSTASK_H_ */
