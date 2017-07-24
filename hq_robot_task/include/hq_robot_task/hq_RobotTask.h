 /*************************************************************
 *
 *   hq_RobotTask.h
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
 * 
 * 
 * RobotTask.h
 *
 *  Created on: Sep 27, 2012
 *      Author: Dan Erusalimchik
 */

#ifndef _HQ_DEF_ROBOT_TASK
#define _HQ_DEF_ROBOT_TASK

//#define SINGLE_GOAL
#ifndef SINGLE_GOAL
	#define MULTI_GOAL
#endif

namespace robot_task{
//	static const int32_t FAIL = 1;
//	static const int32_t SUCCESS = 0;
//	static const int32_t PLAN=-1;
}

#ifdef SINGLE_GOAL
	#include <actionlib/server/simple_action_server.h>
#endif
#ifdef MULTI_GOAL
	#include <hq_robot_task/hq_MultiGoalActionServer.h>
	#include <map>
	#include <boost/thread.hpp>
#endif

#include <hq_robot_task/hq_RobotTaskAction.h>
#include <hq_robot_task/hq_StringOperations.h>
#include <sstream>
#include <iostream>

namespace robot_task{

	using namespace std;
	using namespace hq_robot_task;
	using namespace robot_task_strings;

	class RobotTask{
	public:
		static const int32_t FAIL = 1;
		static const int32_t SUCCESS = 0;
		static const int32_t PLAN=-1;

	protected:
	    typedef hq_RobotTaskFeedback FEEDBACK;
	    typedef hq_RobotTaskResult RESULT;
#ifdef SINGLE_GOAL
	    typedef RobotTaskGoalConstPtr GOAL;
	    typedef actionlib::SimpleActionServer<RobotTaskAction> Server;
#endif
#ifdef MULTI_GOAL
	    typedef actionlib::MGActionServer<hq_RobotTaskAction>::GoalHandle GOAL;
	    typedef actionlib::MGActionServer<hq_RobotTaskAction> Server;
#endif

	protected:
	    ros::NodeHandle _node;
	    Server _server;
	    string _name;
	    FEEDBACK _feedback;
	    RESULT _result;

	    RobotTask(string name):
			_node(),
			_server(_node, name, boost::bind(&RobotTask::abstract_task, this, _1), false),
			_name(name)
		{
			_server.start();
			ROS_INFO("instance of %s started.",_name.c_str());
		}

	    virtual ~RobotTask(){}

#ifdef SINGLE_GOAL
		void finish(const int32_t& success, const std::string& description, const string& plan, bool AbortForUnsuccess = true){
		#define GOAL_SUCCESS _server.setSucceeded(_result);
		#define GOAL_ABORT _server.setAborted(_result);
		#define GOAL_PREEMPTED _server.setPreempted();
#endif
#ifdef MULTI_GOAL
		void finish(GOAL goalH, const int32_t& success, const std::string& description, const string& plan, bool AbortForUnsuccess = true){
		#define GOAL_SUCCESS _server.setSucceeded(goalH, _result);
		#define GOAL_ABORT _server.setAborted(goalH, _result);
		#define GOAL_PREEMPTED _server.setPreempted(goalH);
#endif
			_result.success = success;
			_result.description = description;
			if(success <= 0)
			{
				ROS_INFO("%s: Succeeded. %s", _name.c_str(), description.c_str());
				if(success == PLAN){
					ROS_INFO("%s: New plan. %s", _name.c_str(), description.c_str());
					_result.plan = plan;
				}
				GOAL_SUCCESS
			}else{
				if(AbortForUnsuccess){
					ROS_INFO("%s: Aborted. %s", _name.c_str(), description.c_str());
					GOAL_ABORT
				}else{
					ROS_INFO("%s: Preempted", _name.c_str());
					GOAL_PREEMPTED
				}
			}
		}

		class TaskResult{
			friend class RobotTask;
		protected:
			int32_t success;
			string plan;
			string description;
			bool preempted;

			TaskResult(bool preempted)
			:success(RobotTask::FAIL), plan(""), description("Preempted"), preempted(preempted){}

		public:
			TaskResult(const string& plan, const std::string& description )
			:success(RobotTask::PLAN), plan(plan), description(description), preempted(false){}

			TaskResult(const int32_t success, const std::string& description)
			:success(success), plan(""), description(description), preempted(false){}


			TaskResult(const TaskResult& o)
			:success(o.success), plan(o.plan), description(o.description), preempted(o.preempted){ }

			static TaskResult PREEMPTED(){ return TaskResult(true); }
			static TaskResult FAIL(){ return TaskResult(RobotTask::FAIL,"Unexpected exception"); }
			static TaskResult SUCCESS(){ return TaskResult(RobotTask::SUCCESS,"success"); }
		};

		virtual RobotTask::TaskResult task(const string& name, const string& uid, Arguments& args)=0;

#ifdef MULTI_GOAL
		std::map<boost::thread::id,GOAL> goalsByThread;
#endif
		bool isPreempt(){
#ifdef SINGLE_GOAL
			return (_server.isPreemptRequested() || !ros::ok());
#endif
#ifdef MULTI_GOAL
			GOAL goal = goalsByThread[boost::this_thread::get_id()];
			return (_server.isPreemptRequested(goal) || !ros::ok());
#endif
		}

		void sleep(long millisec){
			boost::this_thread::sleep(boost::posix_time::millisec(millisec));
		}

#ifdef SINGLE_GOAL
		void abstract_task(const GOAL &goal){
#endif
#ifdef MULTI_GOAL
		void abstract_task(GOAL goal_handle){
			goalsByThread[boost::this_thread::get_id()]=goal_handle;
			const hq_RobotTaskGoalConstPtr& goal = goal_handle.getGoal();
#endif
	        /* GET TASK PARAMETERS */
	        ROS_INFO("%s: Start: task name = %s", _name.c_str(), goal->name.c_str());
	        ROS_INFO("%s: Start: task id = %s", _name.c_str(), goal->uid.c_str());
	        ROS_INFO("%s: Start: task params = %s", _name.c_str(), goal->parameters.c_str());

	        /* HERE PROCESS TASK PARAMETERS */
	        Arguments args;
	        try{
	        	args = parseArguments(goal->parameters);
	        }catch(...){
	        	ROS_INFO("%s: WARNING: argument parser report about parameters problem.", _name.c_str());
	        }

	        RobotTask::TaskResult res = task(goal->name, goal->uid, args);

#ifdef SINGLE_GOAL
	        finish( res.success, res.description, res.plan, !res.preempted);
#endif
#ifdef MULTI_GOAL
	        finish( goal_handle, res.success, res.description, res.plan, !res.preempted);
			goalsByThread.erase(boost::this_thread::get_id());
#endif
	    }
	};


}

class SSTREAM{
	typedef const char* c_str_t;
	boost::shared_ptr<std::stringstream> stream;
public:
	operator c_str_t()const{ return c_str(); }
	c_str_t c_str()const{ if(stream.get()) return stream->str().c_str(); return 0; }
	operator std::string()const{ return str(); }
	std::string str()const{ if(stream.get()) return stream->str(); return ""; }
	SSTREAM(const SSTREAM& str):stream(str.stream){}
	SSTREAM():stream(new std::stringstream()){}
	template <typename T>
	SSTREAM& operator<<(const T& t){ if(stream.get()) *(stream.get()) << t; return *this; }
};
#define STR(X) ((SSTREAM()<<X).c_str())

#endif // _HQ_DEF_ROBOT_TASK
