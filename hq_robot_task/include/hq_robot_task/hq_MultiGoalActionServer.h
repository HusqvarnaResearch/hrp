 /*************************************************************
 *
 *   hq_MultiGoalActionServer.h
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
 
 /*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, CogniTeam, Ltd.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of CogniTeam, Ltd. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Dan Erusalimchik
* 
* Based on: code of Willow Garage, Inc. simple_action_server.h
* See Software License Agreement for it.
* Copyright (c) 2008, Willow Garage, Inc.
*********************************************************************/

#ifndef HQ_ACTIONLIB_MULTIGOAL_ACTION_SERVER_H_
#define HQ_ACTIONLIB_MULTIGOAL_ACTION_SERVER_H_

#include <boost/thread/condition.hpp>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/action_definition.h>
#include <deque>

//#define ROS_DEBUG_NAMED(T,X) std::cout<<'['<<T<<"] "<<X<<std::endl;

namespace actionlib {
  /** @class MGActionServer @brief The MGActionServer
   * implements a multi goal policy on top of the ActionServer class. The
   * specification of the policy is as follows: number of goals can have an
   * active status at a time.
   */
	template <class ActionSpec>
	class MGActionServer {
	public:
		//generates typedefs that we'll use to make our lives easier
		ACTION_DEFINITION(ActionSpec);

		typedef typename ActionServer<ActionSpec>::GoalHandle GoalHandle;
		typedef boost::function<void (GoalHandle)> ExecuteCallback;


		class GoalQueueItem{
		public:
			GoalHandle goal;
			bool isPreemptReq;
			boost::thread* thread;
			GoalQueueItem(GoalHandle g):goal(g),isPreemptReq(false),thread(0){}
			//GoalQueueItem():isPreemptReq(false),thread(0){}
			~GoalQueueItem(){
				//ROS_DEBUG_NAMED("daner", "start remove GoalQueueItem");
				isPreemptReq = true;
				if(thread){
					thread=NULL;
				}
				//ROS_DEBUG_NAMED("daner", "end remove GoalQueueItem");
			}
		};
		class GoalQueue{
		public:
			typedef typename std::deque<typename MGActionServer<ActionSpec>::GoalQueueItem>::iterator iterator;
			std::deque<GoalQueueItem> queue;

			void add(GoalHandle goal){ 
				queue.push_back(GoalQueueItem(goal)); 
			}
			iterator find(GoalHandle goal){
				for(iterator i=queue.begin();i!=queue.end();i++)
					if(goal==i->goal) return i;
				return queue.end();
			}
			iterator begin(){ return queue.begin(); }
			iterator end(){ return queue.end(); }
			size_t size()const{ return queue.size(); }
			void remove(GoalHandle goal){
				iterator i = find(goal);
				if(i!=end()){ queue.erase(i); }
			}
			void remove(iterator i){
				if(i!=end()){ queue.erase(i); }
			}
		};
	  

      /**
       * @brief  Constructor for a MGActionServer
       * @param n A NodeHandle to create a namespace under
       * @param name A name for the action server
       * @param execute_cb Optional callback that gets called in a separate thread whenever
       *                   a new goal is received, allowing users to have blocking callbacks.
       *                   Adding an execute callback also deactivates the goalCallback.
       * @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.
       */
      MGActionServer(ros::NodeHandle n, std::string name, ExecuteCallback execute_cb, bool auto_start);

      ~MGActionServer();

      /**
       * @brief  Allows  polling implementations to query about preempt requests
       * @return True if a preempt is requested, false otherwise
       */
      bool isPreemptRequested(GoalHandle goal);

      /**
       * @brief  Allows  polling implementations to query about the status of the current goal
       * @return True if a goal is active, false otherwise
       */
      bool isActive(GoalHandle goal);

      /**
       * @brief  Sets the status of the active goal to succeeded
       * @param  result An optional result to send back to any clients of the goal
       * @param  result An optional text message to send back to any clients of the goal
       */
      void setSucceeded(GoalHandle gh, const Result& result = Result(), const std::string& text = std::string(""));

      /**
       * @brief  Sets the status of the active goal to aborted
       * @param  result An optional result to send back to any clients of the goal
       * @param  result An optional text message to send back to any clients of the goal
       */
      void setAborted(GoalHandle gh, const Result& result = Result(), const std::string& text = std::string(""));

      /**
       * @brief  Sets the status of the active goal to preempted
       * @param  result An optional result to send back to any clients of the goal
       * @param  result An optional text message to send back to any clients of the goal
       */
      void setPreempted(GoalHandle gh, const Result& result = Result(), const std::string& text = std::string(""));

      void requestPreemption(GoalHandle gh);

	  /**
       * @brief  Explicitly start the action server, used it auto_start is set to false
       */
      void start();

      /**
       * @brief  Explicitly shutdown the action server
       */
      void shutdown();
	  
	  
      void removePreemptedGoals(std::vector<boost::thread*>& thread_for_delete); 

    private:
      /**
       * @brief  Callback for when the ActionServer receives a new goal and passes it on
       */
      void goalCallback(GoalHandle goal);

      /**
       * @brief  Callback for when the ActionServer receives a new preempt and passes it on
       */
      void preemptCallback(GoalHandle preempt);

      /**
       * @brief  Called from a separate thread to call blocking execute calls
       */
      void executeLoop();

      ros::NodeHandle n_;

      boost::shared_ptr<ActionServer<ActionSpec> > as_;

      //GoalHandle current_goal_, next_goal_;
	  GoalQueue active_goals, new_goals;

      //bool new_goal_, preempt_request_, new_goal_preempt_request_;

      boost::recursive_mutex lock_;

      ExecuteCallback execute_callback_;

      boost::condition execute_condition_;
      boost::thread* execute_thread_;

      boost::mutex terminate_mutex_;
      bool need_to_terminate_;
  };
};

/*********************************************************************
*********************************************************************/

namespace actionlib {

  template <class ActionSpec>
  MGActionServer<ActionSpec>::MGActionServer(ros::NodeHandle n, std::string name, ExecuteCallback execute_callback, bool auto_start)
    : n_(n), execute_callback_(execute_callback), need_to_terminate_(false) {

    //create the action server
    as_ = 
		boost::shared_ptr<ActionServer<ActionSpec> >(
			new ActionServer<ActionSpec>(
				n, name,
				boost::bind(&MGActionServer::goalCallback, this, _1),
				boost::bind(&MGActionServer::preemptCallback, this, _1),
				auto_start
			)
		);

    if (execute_callback_ != NULL)
    {
      execute_thread_ = new boost::thread(boost::bind(&MGActionServer::executeLoop, this));
    }
  }



  template <class ActionSpec>
  MGActionServer<ActionSpec>::~MGActionServer()
  {
    if(execute_thread_)
      shutdown();
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::shutdown()
  {
    if (execute_callback_)
    {
      {
        boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
        need_to_terminate_ = true;
      }

      assert(execute_thread_);
      execute_thread_->join();
      delete execute_thread_;
      execute_thread_ = NULL;
    }
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::requestPreemption(GoalHandle gh){
		typename GoalQueue::iterator g = active_goals.find(gh);
		if(g!=active_goals.end()){
			g->isPreemptReq=true;
			execute_condition_.notify_all();
		}
  }

  template <class ActionSpec>
  bool MGActionServer<ActionSpec>::isPreemptRequested(GoalHandle gh){
    boost::recursive_mutex::scoped_lock lock(lock_);
		typename GoalQueue::iterator g = active_goals.find(gh);
		if(g!=active_goals.end()) return g->isPreemptReq;
		return true;
  }

  template <class ActionSpec>
  bool MGActionServer<ActionSpec>::isActive(GoalHandle gh){
    typename GoalQueue::iterator g = active_goals.find(gh);
	if(g==active_goals.end()) return false;
    if(!g->goal.getGoal())
      return false;
    unsigned int status = g->goal.getGoalStatus().status;
    return status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING;
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::setSucceeded(GoalHandle gh, const Result& result, const std::string& text){
    boost::recursive_mutex::scoped_lock lock(lock_);
		ROS_DEBUG_NAMED("actionlib", "Setting the current goal as succeeded");
		gh.setSucceeded(result, text);
		requestPreemption(gh);
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::setAborted(GoalHandle gh, const Result& result, const std::string& text){
    boost::recursive_mutex::scoped_lock lock(lock_);
		ROS_DEBUG_NAMED("actionlib", "Setting the current goal as aborted");
		gh.setAborted(result, text);
		requestPreemption(gh);
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::setPreempted(GoalHandle gh, const Result& result, const std::string& text){
    boost::recursive_mutex::scoped_lock lock(lock_);
		ROS_DEBUG_NAMED("actionlib", "Setting the current goal as canceled");
		gh.setCanceled(result, text);
		requestPreemption(gh);
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::goalCallback(GoalHandle goal){
    boost::recursive_mutex::scoped_lock lock(lock_);
		ROS_DEBUG_NAMED("actionlib", "A new goal has been recieved by the multi goal action server");
		ROS_INFO("actionlib: " "A new goal has been recieved by the multi goal action server");
		new_goals.add(goal);
		execute_condition_.notify_all();
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::preemptCallback(GoalHandle preempt){
	boost::recursive_mutex::scoped_lock lock(lock_);
		ROS_DEBUG_NAMED("actionlib", "A preempt has been received by the MGActionServer");
		ROS_INFO("actionlib: " "A preempt has been received by the MGActionServer");

		//std::cout<<"[dan]   GoalQueue::iterator i = new_goals.find(preempt)"<<std::endl;
		typename GoalQueue::iterator i = new_goals.find(preempt);
		if(i!=new_goals.end()){
			i->goal.setCanceled(Result(), "Cancled before activation");
			new_goals.remove(i);
			std::cout<<"TaskServer: goal was preempted without activation"<<std::endl;
			return;
		}
		
		//std::cout<<"[dan]   requestPreemption(preempt)"<<std::endl;
		requestPreemption(preempt);
		
		
		//std::cout<<"[dan] end of preemptCallback"<<std::endl;
  }
  template <class ActionSpec>
  void MGActionServer<ActionSpec>::removePreemptedGoals(std::vector<boost::thread*>& thread_for_delete){
	if( active_goals.size()>0 ){
		bool found= true;
		//std::cout<<"[dan] start search preempted goals for remove"<<std::endl;
		while(found){
			found = false;
			for(typename GoalQueue::iterator i=active_goals.begin(); i!=active_goals.end(); i++){
				if(i->isPreemptReq){ 
					 
					//std::cout<<"[dan]       preempted goal found."<<std::endl;
					boost::thread* th = i->thread;
					active_goals.remove(i);
					thread_for_delete.push_back(th);
					//std::cout<<"[dan]       preempted goal removed."<<std::endl;
					
					found = true;
					break;
				}
			}
		}
		//ROS_DEBUG_NAMED("daner", "no goals found for remove.");
	}
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::executeLoop(){

		ros::Duration loop_duration = ros::Duration().fromSec(.1);
		std::vector<boost::thread*> thread_for_delete;

		while (n_.ok())
		{
			while(thread_for_delete.size()>0){
				thread_for_delete[0]->join();
				delete thread_for_delete[0];
				thread_for_delete.erase(thread_for_delete.begin());
			}
			//std::cout<<"[dan] -------"<<std::endl;
			{
				boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
				if (need_to_terminate_)
					break;
			}

			boost::recursive_mutex::scoped_lock lock(lock_);
			
				
				//std::cout<<"[dan] check preempted goals"<<std::endl;
				// clean preempted goals
				bool reportSize=false;
				size_t agc = active_goals.size();
				size_t stopped_tasks = 0;
				removePreemptedGoals(thread_for_delete);
				if( agc!=active_goals.size() ){
					stopped_tasks = agc-active_goals.size();
					reportSize=true;
				}
			
			
				//std::cout<<"[dan] check new goals"<<std::endl;
				// activate new goals
				size_t started_tasks=0;
				while(new_goals.size()>0){
					GoalHandle goal = new_goals.begin()->goal;
					new_goals.remove(new_goals.begin());
					active_goals.add(goal);
					typename GoalQueue::iterator igoal = active_goals.find(goal);
					goal.setAccepted("Goal activated by MGServer");
					igoal->thread = new boost::thread(boost::bind(execute_callback_, goal));
					started_tasks++;
				}
				if(started_tasks){
					reportSize=true;
				}
				
				if(reportSize){
					ROS_INFO("TaskServer: stopped %i, started %i, active %i",
							 (int)stopped_tasks, (int)started_tasks, (int)active_goals.size());
				}
				
				execute_condition_.timed_wait(lock, boost::posix_time::milliseconds(loop_duration.toSec() * 1000.0f));
		}
    
		while(new_goals.size()>0) new_goals.remove(new_goals.begin());
		while(active_goals.size()>0){
			thread_for_delete.push_back(active_goals.begin()->thread);
			active_goals.remove(active_goals.begin());
		}
		while(thread_for_delete.size()>0){
			thread_for_delete[0]->join();
			delete thread_for_delete[0];
			thread_for_delete.erase(thread_for_delete.begin());
		}
  }

  template <class ActionSpec>
  void MGActionServer<ActionSpec>::start(){
    as_->start();
  }

};


#endif
