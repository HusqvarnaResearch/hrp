# /*************************************************************
# *
# *   hq_multi_goal_action_server.py
# *
# *   Adapted for Husqvarna Research Platform
# * 
# *          March 23, 2017
# *     by:  Kent Askenmalm, Husqvarna Group
# *  
# *
# * Code origin: https://github.com/cogniteam/decision_making
# *
# * original header below
# ************************************************************/

#! /usr/bin/env python
# Copyright (c) 2013, CogniTeam, Ltd.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the CogniTeam, Ltd. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dan Erisalimchik.
# Based on ROS version of simple_action_server.py by Alexander Sorokin.


import roslib; roslib.load_manifest('robot_task')
import rospy

import threading
import traceback

from actionlib_msgs.msg import *

from actionlib import ActionServer
from actionlib.server_goal_handle import ServerGoalHandle;

def nop_cb(goal_handle):
	pass


#def createEmptyGoal():
	#return ServerGoalHandle();


class GoalQueueItem:
	def __init__(self, goal):
		if not isinstance(goal, GoalQueueItem):
			Exception("GoalQueueItem has to handle only objects of type ServerGoalHandle, but object type is "+str(goal.__class__))
		self.goal=goal#ServerGoalHandle();
		self.isPreempted=False
		self.thread=None
	def __eq__(self, other):
		return (isinstance(other, self.__class__)
			and self.goal == other.goal)
	def __ne__(self, other):
		return not self.__eq__(other)
	def __str__(self):
		s_isp = ""
		if self.isPreempted: s_isp = ",Preempted"
		return "MQI("+str(self.goal)+s_isp+")"


class MQIterator:
	def __init__(self, mq, idex):
		self.mq = mq;
		self.idx = idex
	def get(self):
		return self.mq[self.idx]
	def __eq__(self, other):
		return (isinstance(other, self.__class__)
			and self.mq == other.mq
			and self.idx == other.idx)
	def __ne__(self, other):
		return not self.__eq__(other)
	def inc(self, n=1):
		self.idx+=n
	def dec(self, n=1):
		self.idx-=n
	def next(self):
		return MQIterator(self.mq,self.idx+1)
	def prev(self):
		return MQIterator(self.mq,self.idx-1)

class GoalQueue:
	def __init__(self):
		self.queue = []
	def __len__(self):
		return len(self.queue)
	def __getitem__(self, i):
		return self.queue[i]
	def isEmpty(self):
		return len(self.queue)==0
	def add(self, obj):
		if isinstance(obj,ServerGoalHandle):
			obj = GoalQueueItem(obj)
		if not isinstance(obj, GoalQueueItem):
			Exception("Type of added object is not GoalQueueItem. object type is "+str(obj.__class__))
		self.queue.append(obj)
	def find(self, obj):
		if isinstance(obj,ServerGoalHandle):
			obj = GoalQueueItem(obj)
		try:
			#rospy.loginfo("[dan] Start search Goal : %s in %s", str(obj.goal.get_goal_id().id), str([x.goal.get_goal_id().id for x in self.queue]));
			i=[x.goal.get_goal_id().id for x in self.queue].index(obj.goal.get_goal_id().id)
		except ValueError, e:
			#rospy.loginfo("[dan] ... not found")
			return MQIterator(self,len(self.queue))
		#rospy.loginfo("[dan] ... found i=%s", str(i))
		return MQIterator(self,i)
	def begin(self):
		return MQIterator(self,0)
	def end(self):
		return MQIterator(self,len(self.queue))
	def remove(self, obj):
		if isinstance(obj, MQIterator):
			if obj!=self.end():
				self.queue[obj.idx].isPreempted=True
				del self.queue[obj.idx]
		else:
			self.remove(self.find(obj))
	def __str__(self):
		s=[str(x) for x in self.queue]
		return "MQ:["+",".join(s)+"]"
		
	def moveTo(self, other):
		if len(self)>0:
			other.add(self.begin().get())
			self.remove(self.begin())
			return other.end().prev();
		return other.end()
	
if __name__ == '__main__':
	pass

class RunExecuteCallback(threading.Thread):
	def __init__(self, goal, server):
		threading.Thread.__init__(self)
		self.goal = goal
		self.server = server
	def run(self):
		#rospy.loginfo("[dan] Run thread with goal: %s", str( self.goal.get_goal_id().id ));
		try:
			self.server.execute_callback(self.goal)
			if self.server.is_active(self.goal):
				rospy.logwarn("Your executeCallback did not set the goal to a terminal status.  " +
							"This is a bug in your ActionServer implementation. Fix your code!  "+
							"For now, the ActionServer will set this goal to aborted");
				self.server.set_aborted(self.goal, None, "No terminal state was set.");
		except Exception, ex:
			rospy.logerr("Exception in your execute callback: %s\n%s", str(ex), traceback.format_exc())
			self.server.set_aborted(self.goal, None, "Exception in execute callback: %s" % str(ex))


## @class MGActionServer
## @brief The MGActionServer
## implements a multi goal policy on top of the ActionServer class. 
class MGActionServer:
	## @brief Constructor for a MGActionServer
	## @param name A name for the action server
	## @param execute_cb Optional callback that gets called in a separate thread whenever
	## a new goal is received, allowing users to have blocking callbacks.
	## Adding an execute callback also deactivates the goalCallback.
	## @param  auto_start A boolean value that tells the ActionServer wheteher or not to start publishing as soon as it comes up. THIS SHOULD ALWAYS BE SET TO FALSE TO AVOID RACE CONDITIONS and start() should be called after construction of the server.
	def __init__(self, name, ActionSpec, execute_cb = None, auto_start = True):

		self.active_goals=GoalQueue()
		self.new_goals=GoalQueue()

		self.execute_callback = execute_cb;

		self.need_to_terminate = False
		self.terminate_mutex = threading.RLock();
		self.lock = threading.RLock();

		self.execute_condition = threading.Condition(self.lock);

		if self.execute_callback:
			self.execute_thread = threading.Thread(None, self.executeLoop, name="ExecuterLoop");
			self.execute_thread.start();
		else:
			self.execute_thread = None

		#create the action server
		self.action_server = ActionServer(name, ActionSpec, self.internal_goal_callback,self.internal_preempt_callback,auto_start);


	def __del__(self):
		if hasattr(self, 'execute_callback') and self.execute_callback:
			with self.terminate_mutex:
				self.need_to_terminate = True;

			assert(self.execute_thread);
			self.execute_thread.join();


	def accept_new_goal(self):
		if self.new_goals.isEmpty():
			return None
		g = self.new_goals.moveTo(self.active_goals)
		g.get().isPreempted = False
		g.get().goal.set_accepted("This goal has been accepted by the multi goal action server");
		return g;


	## @brief Allows  polling implementations to query about the availability of a new goal
	## @return True if a new goal is available, false otherwise
	def is_new_goal_available(self):
		return self.new_goals.isEmpty()==False;


	## @brief Allows  polling implementations to query about preempt requests
	## @return True if a preempt is requested, false otherwise
	def is_preempt_requested(self, goal):
		with self.lock:
			g = self.active_goals.find(goal)
			if g!=self.active_goals.end():
				return g.get().isPreempted;
			return True

	## @brief Allows  polling implementations to query about the status of the current goal
	## @return True if a goal is active, false otherwise
	def is_active(self, goal):
		g = self.active_goals.find(goal)
		if g==self.active_goals.end(): return False
		status = g.get().goal.get_goal_status().status;
		return status == actionlib_msgs.msg.GoalStatus.ACTIVE or status == actionlib_msgs.msg.GoalStatus.PREEMPTING;

	def requestPreemption(self, goal):
		g = self.active_goals.find(goal)
		if g!=self.active_goals.end():
			g.get().isPreempted = True
			self.execute_condition.notify();
		
	## @brief Sets the status of the active goal to succeeded
	## @param  result An optional result to send back to any clients of the goal
	def set_succeeded(self, goal, result=None, text=""):
		self.execute_condition.acquire();
		try:
			if not result:
				result=self.get_default_result();
			rospy.loginfo("Setting the current goal as successed");
			goal.set_succeeded(result, text);
			self.requestPreemption(goal)
			self.execute_condition.release();
		except Exception, e:
			rospy.logerr("MGActionServer.set_succeeded - exception %s",str(e))
			self.execute_condition.release();

	## @brief Sets the status of the active goal to aborted
	## @param  result An optional result to send back to any clients of the goal
	def set_aborted(self, goal, result = None, text=""):
		self.execute_condition.acquire();
		try:
			if not result:
				result=self.get_default_result();
			rospy.loginfo("Setting the current goal as aborted");
			goal.set_aborted(result, text);
			self.requestPreemption(goal)
			self.execute_condition.release();
		except Exception, e:
			rospy.logerr("MGActionServer.set_aborted - exception %s",str(e))
			self.execute_condition.release();

	def get_default_result(self):
		return self.action_server.ActionResultType();

	## @brief Sets the status of the active goal to preempted
	## @param  result An optional result to send back to any clients of the goal
	def set_preempted(self, goal,result=None, text=""):
		self.execute_condition.acquire();
		try:
			if not result:
				result=self.get_default_result();
			rospy.loginfo("Setting the current goal as canceled");
			goal.set_canceled(result, text);
			self.requestPreemption(goal)
			self.execute_condition.release();
		except Exception, e:
			rospy.logerr("MGActionServer.set_preempted - exception %s",str(e))
			self.execute_condition.release();


	## @brief Explicitly start the action server, used it auto_start is set to false
	def start(self):
		self.action_server.start();


	## @brief Callback for when the ActionServer receives a new goal and passes it on
	def internal_goal_callback(self, goal):
		self.execute_condition.acquire();
		try:
			rospy.loginfo("A new goal %s has been recieved by the MGActionServer",goal.get_goal_id().id);
			
			self.new_goals.add(goal)

			self.execute_condition.notify();
			self.execute_condition.release();
			
		except Exception, e:
			rospy.logerr("MGActionServer.internal_goal_callback - exception %s",str(e))
			self.excondition.release();

	## @brief Callback for when the ActionServer receives a new preempt and passes it on
	def internal_preempt_callback(self,goal):
		self.execute_condition.acquire();
		try:
			rospy.loginfo("A preempt for goal %s has been received by the MGActionServer",goal.get_goal_id().id);
			
			inq = self.new_goals.find(goal)
			if inq!=self.new_goals.end():
				self.new_goals.remove(inq)
				self.execute_condition.release();
				return
			
			self.requestPreemption(goal)
			self.execute_condition.release();
		except Exception, e:
			rospy.logerr("MGActionServer.internal_preempt_callback - exception %s",str(e))
			self.execute_condition.release();
			
	def removePreemptedGoals(self, threads_for_delete):
		found = True
		while found:
			found = False
			for x in self.active_goals:
				if x.isPreempted:
					self.active_goals.remove(x)
					threads_for_delete.append(x.thread)
					found=True
					break
					
			
	## @brief Called from a separate thread to call blocking execute calls
	def executeLoop(self):
		loop_duration = rospy.Duration.from_sec(.1);
		#loop_duration = rospy.Duration.from_sec(1.0);
		threads_for_delete = []

		while (not rospy.is_shutdown()):
			#rospy.loginfo("[dan] MGAS: execute");
			print_sum = False
			
			#rospy.loginfo("[dan] start remove deleted threads")
			c_deleted = 0
			while(len(threads_for_delete)>0):
				threads_for_delete[0].join()
				del threads_for_delete[0]
				c_deleted+=1
			#rospy.loginfo("[dan] finished remove deleted threads: deleted %s",str(c_deleted))
			if c_deleted>0 : print_sum = True

			with self.terminate_mutex:
				if (self.need_to_terminate):
					break;

			with self.execute_condition:
				
				#remove preempted goals
				#rospy.loginfo("[dan] start stopping preempted goals")
				self.removePreemptedGoals(threads_for_delete)
				#rospy.loginfo("[dan] finished stopping preempted goals: stopped %s",str(len(threads_for_delete)))
				if len(threads_for_delete)>0: print_sum=True
				
				
				#run new goals
				#rospy.loginfo("[dan] start running new goals")
				c_newtasks = 0
				while(len(self.new_goals)>0):
					goal = self.accept_new_goal()
					goal.get().thread = RunExecuteCallback(goal.get().goal, self);
					goal.get().thread.start()
					c_newtasks += 1
				#rospy.loginfo("[dan] finished running new goals: started %s",str(c_newtasks))
				if c_newtasks>0: print_sum = True
			
				#rospy.loginfo("[dan] there are %s active goals",str(len(self.active_goals)))
				if print_sum:
					rospy.loginfo("[ActionServer] summery: %s stopped threads, %s removed preempted goals, %s activated goals, %s active tasks",
					               str(c_deleted), str(len(threads_for_delete)), str(c_newtasks), str(len(self.active_goals)))
				
				self.execute_condition.wait(loop_duration.to_sec());
				
		#rospy.loginfo("[dan] start remove deleted threads")
		while(len(threads_for_delete)>0):
			threads_for_delete[0].join()
			del threads_for_delete[0]
		#rospy.loginfo("[dan] finished remove deleted threads")

		#rospy.loginfo("[dan] start remove not activated threads")
		while(len(self.new_goals)>0):
			del self.new_goals[0]
		#rospy.loginfo("[dan] finished remove not activated threads")

		#rospy.loginfo("[dan] start remove not activated threads")
		while(len(self.active_goals)>0):
			self.active_goals.isPreempted
			self.active_goals[0].thread.join()
			del self.active_goals[0]
		#rospy.loginfo("[dan] finished remove not activated threads")
			
		#rospy.loginfo("[dan] MG Action Server DONE")

