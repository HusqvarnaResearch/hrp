 /*************************************************************
 *
 *   hq_ROSTask.cpp
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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <hq_robot_task/hq_RobotTask.h>
#include <hq_robot_task/hq_RobotTaskAction.h>

#include <boost/filesystem.hpp>

#include "hq_ROSTask.h"

#ifndef DMDEBUG
#define DMDEBUG(...)
#endif


using namespace std;
using namespace hq_robot_task;

#include <fstream>

namespace {

	ofstream f1("/tmp/f1");
	ofstream f2("/tmp/f2");
	ofstream f3("/tmp/f3");
	long f1counter=0;
	long f2counter=0;
	long f3counter=0;
}


namespace decision_making{

typedef actionlib::SimpleActionClient<hq_RobotTaskAction> Client;

const double WAIT_SERVER_DURATION = 1.0;
const double WAIT_RESULT_DURATION = 1.0;

#define CHECK_FOR_TERMINATION \
		if( events.isTerminated() or not ros::ok() ){\
			DMDEBUG( cout<<" TASK("<<task_address<<":TERMINATED) " ; )\
			RosDiagnostic::get().publish(call_ctx.str(), "TASK", "stopped", str(TaskResult::TERMINATED()));\
			return TaskResult::TERMINATED();\
		}


TaskResult callTask(std::string task_address, const CallContext& call_ctx, EventQueue& events);

class RosNodeInformation{
	RosNodeInformation():run_id(uid()){}
	static char hex(int i){ if(i<10) return '0'+i; return 'A'+(i-10); }
	static string uid(){
		srand(time(NULL));
		stringstream s;
		for(int i=0;i<4;i++){ for(int i=0;i<5;i++) s<<hex(rand()%16); s<<'-';} for(int i=0;i<5;i++) s<<hex(rand()%10);
		return s.str();
	}
public:
	static RosNodeInformation& get(){ static RosNodeInformation node; return node; }
	std::string executable_path;
	std::string executable_dir;
	std::string node_name;
	std::string node_namespace;
	std::string run_id;
};

class RosDiagnostic{
public:
	static RosDiagnostic& get(){static RosDiagnostic d; return d;}
	typedef void (*update_t)();
	struct DiagnosticMessage{
		string name;
		string type;
		string status;
		string info;
		DiagnosticMessage(){}
		DiagnosticMessage(
			string name,
			string type,
			string status,
			string info
		):
				name(name),
				type(type),
				status(status),
				info(info)
		{}
		string str()const{
			stringstream s;
			s<<"D["<<name<<":"<<type<<":"<<status<<":"<<info<<"]";
			return s.str();
		}

		diagnostic_msgs::DiagnosticStatus::Ptr getDiagnosticStatus()const{
			diagnostic_updater::DiagnosticStatusWrapper* ret_ptr = new diagnostic_updater::DiagnosticStatusWrapper();
			diagnostic_msgs::DiagnosticStatus::Ptr ret( ret_ptr );
			diagnostic_updater::DiagnosticStatusWrapper& stat = * ret_ptr;
			const DiagnosticMessage& msg = *this;
			RosNodeInformation& info = RosNodeInformation::get();

			stat.hardware_id = "none";
			stat.name = info.node_name.substr(1)+": "+"decision_making";

			stringstream summery;
			summery<<"["<<f3counter<<"] "<<msg.name<<" "<<msg.type<<" is "<<msg.status<<". "<<msg.info;
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, summery.str().c_str());

			stat.add("name", msg.name);
			stat.add("type", msg.type);
			stat.add("status", msg.status);
			stat.add("result", msg.info);
			stat.add("node_name", info.node_name);
			stat.add("node_exe_file", info.executable_path);
			stat.add("node_exe_dir", info.executable_dir);
			stat.add("node_run_id", info.run_id);

			f3<<"#"<<f3counter<<": "<<msg.name<<" "<<msg.type<<" is "<<msg.status<<". "<<msg.info<<endl;
			f3counter++;
			return ret;
		}
	};
	void publish(
			string name,
			string type,
			string status,
			string info
	){
		boost::mutex::scoped_lock l(mtx);
		{
			boost::mutex::scoped_lock lq(mtx_queue);
			DiagnosticMessage msg ( name, type, status, info );
			//cout<<msg.str();
			f1<<"#"<<f1counter<<": "<<msg.name<<" "<<msg.type<<" is "<<msg.status<<". "<<msg.info<<endl;
			f1counter++;
			queue.push_back(msg);

			diagnostic_msgs::DiagnosticStatus::Ptr dg_msg = msg.getDiagnosticStatus();
			diagnostic_msgs::DiagnosticArray dga_msg;
			dga_msg.status.push_back(*(dg_msg.get()));
			diagnostic_publisher.publish(dga_msg);

		}
		update();
		on_new_message.notify_one();
	}
	DiagnosticMessage tryGet(bool& success){
		//boost::mutex::scoped_lock l(mtx);
		boost::mutex::scoped_lock lq(mtx_queue);
		success = false;
		if(queue.empty()) return DiagnosticMessage();
		DiagnosticMessage msg = queue.front();
		queue.pop_front();
		success=true;
		return msg;
	}
	void update(){ ros_diagnostic_updater.force_update(); }
	class DiagnosticTask{
	public:
		void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat)
		{
			bool ok;
			DiagnosticMessage msg = get().tryGet(ok);
			RosNodeInformation& info = RosNodeInformation::get();
			if(ok){
				stringstream summery;
				summery<<"["<<f2counter<<"] "<<msg.name<<" "<<msg.type<<" is "<<msg.status<<". "<<msg.info;
				stat.summary(diagnostic_msgs::DiagnosticStatus::OK, summery.str().c_str());

				stat.add("name", msg.name);
				stat.add("type", msg.type);
				stat.add("status", msg.status);
				stat.add("result", msg.info);
				stat.add("node_name", info.node_name);
				stat.add("node_exe_file", info.executable_path);
				stat.add("node_exe_dir", info.executable_dir);
				stat.add("node_run_id", info.run_id);

				f2<<"#"<<f2counter<<": "<<msg.name<<" "<<msg.type<<" is "<<msg.status<<". "<<msg.info<<endl;
				f2counter++;
			}
		}
	};
private:
	boost::mutex mtx;
	boost::mutex mtx_queue;
	boost::condition_variable on_new_message;
	std::deque<DiagnosticMessage> queue;
	diagnostic_updater::Updater ros_diagnostic_updater;
	DiagnosticTask diagnostik_task;
	ros::Publisher diagnostic_publisher;

	RosDiagnostic(){
		ros_diagnostic_updater.setHardwareID("none");
		ros_diagnostic_updater.add("decision_making", &diagnostik_task, &RosDiagnostic::DiagnosticTask::produce_diagnostics);

		ros::NodeHandle node;
		//diagnostic_publisher = node.advertise<diagnostic_msgs::DiagnosticStatus>("/decision_making/monitoring", 100);
		diagnostic_publisher = node.advertise<diagnostic_msgs::DiagnosticArray>("/decision_making/monitoring", 100);
	}
};

#define IMPL_ON_FUNCTION(NAME, TEXT) \
	ON_FUNCTION(NAME){\
		/*cout<<"[on_fsm_start]"<<call_ctx.str()<<endl;*/\
		RosDiagnostic::get().publish(call_ctx.str(), type, #TEXT, str(result));\
	}

IMPL_ON_FUNCTION(on_fsm_start, started)
IMPL_ON_FUNCTION(on_fsm_end, stopped)
IMPL_ON_FUNCTION(on_fsm_state_start, started)
IMPL_ON_FUNCTION(on_fsm_state_end, stopped)
IMPL_ON_FUNCTION(on_bt_node_start, started)
IMPL_ON_FUNCTION(on_bt_node_end, stopped)
IMPL_ON_FUNCTION(on_tao_tree_start, started)
IMPL_ON_FUNCTION(on_tao_tree_end, stopped)
IMPL_ON_FUNCTION(on_tao_plan_start, started)
IMPL_ON_FUNCTION(on_tao_plan_end, stopped)

std::string RosConstraints::preproc(std::string txt)const{
	std::stringstream source(txt);
	std::stringstream result;
	enum states_t{ nline, normal, replace } state=nline;
	for(size_t i=0;i<txt.size();i++){
		char c = txt[i];
		switch(state){
		case nline:
			switch(c){
			case '$': result<<"#!"; state=normal; break;
			case ';': state=replace; break;
			case '\n': result<<c; state=nline; break;
			default: result<<c; state=normal; break;
			}
			break;
		case normal:
			switch(c){
			case ';': state=replace; break;
			case '\n': result<<c; state=nline; break;
			default: result<<c; state=normal; break;
			}
			break;
		case replace:
			switch(c){
			case ' ': result<<'\n'; state=nline; break;
			case '\n': result<<';'<<c; state=nline; break;
			default: result<<';'<<c; state=normal; break;
			}
			break;
		}
	}
	return result.str();
}

TaskResult callTask(std::string task_address, const CallContext& gl_call_ctx, EventQueue& events){
	DMDEBUG( cout<<" TASK("<<task_address<<":CALL) " ;)
	//CallContext call_ctx(gl_call_ctx, task_address);
	const CallContext& call_ctx = gl_call_ctx;
	RosDiagnostic::get().publish(call_ctx.str(), "TASK", "started", str(decision_making::TaskResult::UNDEF()));

	string task_name, task_params;
	size_t i=task_address.find('(');
	if(i==string::npos) task_name = task_address;
	else{ task_name=task_address.substr(0,i); task_params=task_address.substr(i+1,task_address.size()-1); }
	string full_task_name = task_name;

	if(LocalTasks::registrated(task_name)){
		TaskResult res = LocalTasks::call(task_name, task_address, call_ctx, events);
		RosDiagnostic::get().publish(call_ctx.str(), "TASK", "stopped", str(res));
		Event new_event ( Event(""+ MapResultEvent::map(task_name, res.error()), call_ctx) );
		DMDEBUG( cout<<" TASK("<<task_address<<":"<<new_event<<") "; )
		events.raiseEvent(new_event);
		return res;
	}

	while(true)
	{
		// create the action client
		// true causes the client to spin its own thread
		Client ac(full_task_name.c_str(), true);

		DMDEBUG( cout<<" Task["<<full_task_name<<":Waiting for action server to start task] "; )
		// wait for the action server to start
		//ac.waitForServer(); //will wait for infinite time

		bool serverFound = false;
		while( not serverFound  and ros::ok() and not events.isTerminated() ){
			ac.waitForServer(ros::Duration(WAIT_SERVER_DURATION));
			if(ac.isServerConnected()){
				DMDEBUG( cout<<" Task["<<full_task_name<<": is found.] "; )
				serverFound = true;
			}else{
				DMDEBUG( cout<<" Task["<<full_task_name<<": not found, retry] "; )
			}
		}

		CHECK_FOR_TERMINATION

		// send a goal to the action
		hq_RobotTaskGoal goal;
			goal.name = task_name;
			stringstream suid; suid<<"task_id_tested_"<<::time(NULL);
			goal.uid = suid.str();
			goal.parameters=task_params;

		DMDEBUG( cout<<" Task["<<full_task_name<<":goal is nm="<< goal.name <<", id="<< goal.uid <<", pr="<< goal.parameters<<"] "; )

		ac.sendGoal(goal);
		DMDEBUG( cout<<" Task["<<full_task_name<<":goal sent. wait for result"<<"] "; )

		bool finished = false;
		while(not events.isTerminated() and ros::ok()){

			DMDEBUG( cout<<" Task["<<full_task_name<<":Wait for "<<WAIT_RESULT_DURATION<<" sec] "; )

			//wait for the action to return
			bool finished_before_timeout = ac.waitForResult(ros::Duration(WAIT_RESULT_DURATION));

			if (finished_before_timeout)
			{
				stringstream res_str;
				actionlib::SimpleClientGoalState state = ac.getState();
				res_str<< "finished:st="<<state.toString();
				finished = true;
				if(
					state==actionlib::SimpleClientGoalState::SUCCEEDED ||
					state==actionlib::SimpleClientGoalState::PREEMPTED ||
					state==actionlib::SimpleClientGoalState::ABORTED
				){
					res_str<<",rs="<<ac.getResult()->success;
					res_str<<",ds="<<ac.getResult()->description;
					res_str<<",pl="<<ac.getResult()->plan;
				}
				DMDEBUG( cout<<" Task["<<full_task_name<<":"<< res_str.str() <<"] "; )
				break;
			}
		}

		if(!finished){
			DMDEBUG( cout<<" Task["<<full_task_name<<":"<< "Abort task goal" <<"] "; )
			ac.cancelGoal();
		}

		CHECK_FOR_TERMINATION

		DMDEBUG( cout<<" Task["<<full_task_name<<":"<< "Result is gotten" <<"] "; )

		Event new_event ( Event(""+ MapResultEvent::map(task_name, ac.getResult()->success), call_ctx) );
		DMDEBUG( cout<<" TASK("<<task_address<<":"<<new_event<<") "; )
		events.raiseEvent(new_event);

		TaskResult res;
		if(ac.getResult()->success==0){
			res = TaskResult::SUCCESS();
		}else{
			res = TaskResult::FAIL(TaskResult::rerangeErrorCode(ac.getResult()->success), ac.getResult()->description);
		}
		RosDiagnostic::get().publish(call_ctx.str(), "TASK", "stopped", str(res));
		return res;

	}
	RosDiagnostic::get().publish(call_ctx.str(), "TASK", "stopped", str(TaskResult::FAIL()));
	return TaskResult::FAIL();
}

RosEventQueue::RosEventQueue():decision_making::EventQueue(), do_not_publish_spin(true){
	std::string node_name = ros::this_node::getName();
	std::string topic_name = "/decision_making/"+node_name+"/events";
	publisher = ros::NodeHandle().advertise<std_msgs::String>(topic_name, 100);
	{boost::this_thread::sleep(boost::posix_time::seconds(1.0));}
	subscriber= ros::NodeHandle().subscribe<std_msgs::String>(topic_name, 100, &RosEventQueue::onNewEvent, this);
	{boost::this_thread::sleep(boost::posix_time::seconds(1.0));}
}
RosEventQueue::RosEventQueue(EventQueue* parent):decision_making::EventQueue(parent), do_not_publish_spin(true){
}
RosEventQueue::RosEventQueue(EventQueue* parent, bool isTransit):decision_making::EventQueue(parent,isTransit), do_not_publish_spin(true){
}

void RosEventQueue::onNewEvent(const std_msgs::String::ConstPtr& msg){
	//cout<<"RosEventQueue:onNewEvent: "<<msg->data<<endl;
	decision_making::Event e(msg->data);
	decision_making::EventQueue::raiseEvent(e);
}
void RosEventQueue::raiseEvent(const decision_making::Event& e){
	if( do_not_publish_spin and e.equals(Event::SPIN_EVENT()) ){
		decision_making::EventQueue::raiseEvent(e);
		return;
	}
	std_msgs::String::Ptr msg(new std_msgs::String());
	msg->data = e._name;
	publisher.publish(msg);
}


}


void ros_decision_making_init(int &argc, char **argv){
	using namespace decision_making;
	RosNodeInformation& info = RosNodeInformation::get();
	info.node_name = ros::this_node::getName();
	info.node_namespace = ros::this_node::getNamespace();
	info.executable_path = boost::filesystem::path(argv[0]).string();
	info.executable_dir = boost::filesystem::current_path().string();
	RosDiagnostic::get();
	RosConstraints::getAdder();
	RosConstraints::getRemover();
	boost::this_thread::sleep(boost::posix_time::seconds(1.0));
}



