#! /usr/bin/env python


import roslib; roslib.load_manifest('robot_task')
import rospy

import actionlib
import robot_task.msg
from multi_goal_action_server import *
import thread

RobotTask_SUCCESS = 0
RobotTask_PLAN = -1
RobotTask_FAIL = 1;

class RTResult(object):
	def __init__(self, SUCC_val, PLAN_val, DES_val, PREEPTED_val):
		self.success = SUCC_val
		self.plan = PLAN_val
		self.description = DES_val
		self.isPreepted = PREEPTED_val
		
def RTResult_PREEPTED():
	return RTResult(RobotTask_FAIL, "", "Preempted", True)
def RTResult_SUCCESSED(desc):
	return RTResult(RobotTask_SUCCESS, "", desc, False)
def RTResult_PLAN(plan,desc):
	return RTResult(RobotTask_PLAN, "", desc, False)
def RTResult_ABORT(err,desc):
	if err<=RobotTask_SUCCESS:
		err = RobotTask_FAIL
	return RTResult(err, "", desc, False)
	
def parseParameters(params):
	def findBlock(s, to, tc, si):
		stri=-1
		endi=-1
		counter=0
		#print '>>>',s,':',to,':',tc,':',si
		for i in xrange(si, len(s)):
			c = s[i]
			#print "    ",c,"-",i,'[',(counter==0),(c==to),(c==tc),']',
			if c==to:
				if counter==0:
					stri=i
				counter+=1
			if c==tc:
				counter-=1
				if counter==0:
					endi=i
			#print '-',counter,'-',stri,'-',endi
			if endi>-1:
				return (stri, endi)
		return (stri, endi)
	def searchArgIndex(s, blocks):
		def checkInBlock(i, blocks):
			for b in blocks:
				if b[0]<=i and i<=b[1]: return True
			return False
		commas=[]
		i = s.find(',',0)
		while i>=0:
			if not checkInBlock(i, blocks):
				commas.append(i)
			i = s.find(',',i+1)
		return commas
	def findAllBlocks(s, to, tc):
		blocks=[]
		bs,be = findBlock(params, to, tc,0)
		while bs>=0 and be>0:
			blocks.append( (bs, be) )
			bs,be = findBlock(params, to, tc,be+1)
		return blocks
	blocks=[]
	blocks = blocks + findAllBlocks(params, '(', ')')
	blocks = blocks + findAllBlocks(params, '[', ']')
	blocks = blocks + findAllBlocks(params, '{', '}')
	blocks = blocks + findAllBlocks(params, '<', '>')
	#print blocks
	commas = searchArgIndex(params, blocks)
	args=[]
	if len(commas)==0 : args.append(params)
	else :
		com_s=0
		for com_e in commas:
			arg = params[com_s:com_e]
			args.append(arg.strip())
			com_s = com_e+1
		arg = params[com_s:len(params)]
		args.append(arg.strip())	
	vmap={}
	for a in args:
		if len(a)==0: continue
		if a.find('=')>=0:
			k,v = a[:a.find('=')],a[a.find('=')+1:]
			vmap[k.strip()] = v.strip()
		else:
			vmap[a.strip()]=""
	
	return vmap


class RobotTask(object):
	_result   = robot_task.msg.RobotTaskResult()

	def __init__(self, name):
		self._action_name = name
		self._goals = {}
		self._as = MGActionServer(self._action_name, robot_task.msg.RobotTaskAction, self.abstract_task, False)
		rospy.loginfo('%s: Start create task object' % self._action_name)
		self._as.start()
		rospy.loginfo('%s: Task object created' % self._action_name)

	def currentGoal(self):
		return self._goals[thread.get_ident()]
		
	def finish(self, res):
		#print "[dan] finish task"
		if res.isPreepted:
			rospy.loginfo('%s: Task Preempted' % self._action_name)
			self._as.set_preempted(self.currentGoal())
			return
		if res.success<=RobotTask_SUCCESS:
			self._result.success = res.success
			if res.success == RobotTask_PLAN:
				self._result.plan = res.plan
			self._result.description = res.description
			rospy.loginfo('%s: Task Succeeded' % self._action_name)
			self._as.set_succeeded(self.currentGoal(), self._result)
			return
		self._result.success = res.success
		self._result.description = res.description
		rospy.loginfo('%s: Task Aborted' % self._action_name)
		self._as.set_aborted(self.currentGoal(), self._result)
		
	def isPreepted(self):
		return rospy.is_shutdown() or self._as.is_preempt_requested(self.currentGoal())

	def abstract_task(self, goal):
		#print "[dan] start abstract task"
		rospy.loginfo('%s: Task Started' % self._action_name)
		self._goals[thread.get_ident()]=goal
		arguments = parseParameters(goal.get_goal().parameters)
		rospy.loginfo('%s: ... name=%s, id=%s, args=%s' , self._action_name , goal.get_goal().name , goal.get_goal().uid , str(arguments))
		res = self.task(goal.get_goal().name, goal.get_goal().uid, arguments)
		if rospy.is_shutdown(): return
		self.finish(res)
		del self._goals[thread.get_ident()]
	
	def task(self, name, uid, parameters):
		#print "[dan] Empty Task"
		return RTResult_ABORT(1,"Empty Task")


      
if __name__ == '__main__':
	pass
