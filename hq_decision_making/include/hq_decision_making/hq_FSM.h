 /*************************************************************
 *
 *   hq_FSM.h
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
 * FSM.h
 *
 *  Created on: Nov 14, 2013
 *      Author: dan
 */

#ifndef HQ_FSM_H_
#define HQ_FSM_H_


#include "hq_EventSystem.h"
#include "hq_TaskResult.h"

#ifndef DMDEBUG
#define DMDEBUG(...)
#endif


namespace decision_making{

class ___ABS__ScoppedThreadsOnExit{
public:
    virtual ~___ABS__ScoppedThreadsOnExit(){}
    virtual void exit()=0;
    virtual boost::thread_group& getThreads()=0;
};
struct ScoppedThreads{
	typedef boost::shared_ptr<EventQueue> EventQueuePtr;
	typedef boost::shared_ptr<CallContext> CallContextPtr;
	typedef boost::shared_ptr<___ABS__ScoppedThreadsOnExit> ScoppedThreadsOnExitPtr;
	boost::thread_group threads;
	vector<EventQueuePtr> events;
	vector<CallContextPtr> contexts;
	void add(boost::thread* thread){threads.add_thread(thread);};
	void add(EventQueuePtr event){ events.push_back(event); }
	void add(CallContextPtr event){ contexts.push_back(event); }

	void stopEvents(){
		BOOST_FOREACH(EventQueuePtr e, events){
			e->close();
		}
	}
	struct Cleaner{
		ScoppedThreads& target;
		Cleaner(ScoppedThreads& target):target(target){}
		~Cleaner(){
			target.runOnExit();
			target.stopEvents();
			target.threads.join_all();
		}
	};

	vector<ScoppedThreadsOnExitPtr> on_exits;
	void add(ScoppedThreadsOnExitPtr exit){ on_exits.push_back(exit); }
	//void runOnExit();
	void runOnExit(){
	    BOOST_FOREACH(ScoppedThreadsOnExitPtr e, on_exits){
	        e->exit();
	        e->getThreads().join_all();
	    }
	}
};
class ScoppedThreadsOnExit:public ___ABS__ScoppedThreadsOnExit{
public:
	EventQueue* events_queue;
	CallContext& state_call_ctx;
	ScoppedThreads SUBMACHINESTHREADS;
	ScoppedThreadsOnExit(CallContext& call_ctx, EventQueue* events_queue):
		events_queue(events_queue), state_call_ctx(call_ctx)
	{}
	virtual ~ScoppedThreadsOnExit(){}
	//virtual void exit()=0;
	virtual boost::thread_group& getThreads(){ return SUBMACHINESTHREADS.threads; }
};


#define FSM_HEADER(NAME) \
	decision_making::TaskResult Fsm##NAME(const decision_making::CallContext*, decision_making::EventQueue*, std::string);\
	decision_making::TaskResult Fsm##NAME(const decision_making::CallContext* p, decision_making::EventQueue* q);

#define FSM(NAME) \
	FSM_HEADER(NAME)\
	decision_making::TaskResult Fsm##NAME(const decision_making::CallContext* p, decision_making::EventQueue* q){return Fsm##NAME(p,q,#NAME);}\
	decision_making::TaskResult Fsm##NAME(const decision_making::CallContext* parent_call_ctx, decision_making::EventQueue* parent_event_queue, std::string fsm_name)

#define FSM_STATES enum STATES

#define __DEFCALLCONTEXT decision_making::CallContext call_ctx(parent_call_ctx?decision_making::CallContext(*parent_call_ctx, fsm_name):decision_making::CallContext(fsm_name));
#define __DEFEVENTQUEUE decision_making::EventQueue* events_queue(parent_event_queue);
#define FSM_START(STATE) \
		state ( STATE ); \
		decision_making::TaskResult fsm_result = decision_making::TaskResult::TERMINATED();\
		__DEFCALLCONTEXT __DEFEVENTQUEUE  \
		DMDEBUG( cout<<" FSM("<<fsm_name<<":START) "; )\
		ON_FSM_START(fsm_name, call_ctx, *events_queue);

#define FSM_BGN \
		bool fsm_stop = false; \
		while(not fsm_stop and not events_queue->isTerminated() DM_SYSTEM_STOP){ \
			switch(state){ { {

#define FSM_END \
			}}}} \
			DMDEBUG( cout<<" FSM("<<fsm_name<<":FINISH) "; ) \
			ON_FSM_END(fsm_name, call_ctx, *events_queue, fsm_result);\
			return fsm_result;

#define __STARTOFSTATE(X) \
		DMDEBUG( string outname("STT("+fsm_name+":"+call_ctx.str()+"/"+#X+")");cout<<outname<<"{ "; )\
		std::string state_name(#X);\
		decision_making::CallContext state_call_ctx(call_ctx, state_name);\
		ON_FSM_STATE_START(state_name, call_ctx, *events_queue);

#define __ENDOFSTATE \
		DMDEBUG( struct _STATE_FINISHER_PRINT{std::string n;_STATE_FINISHER_PRINT( std::string n): n(n){}~_STATE_FINISHER_PRINT(){DMDEBUG( cout<<"}"<<n<<" "; )} void r(){}}_ep(outname);_ep.r(); )\
		struct _STATE_FINISHER{\
			std::string state_name; decision_making::CallContext& ctx; decision_making::EventQueue& queue;\
			_STATE_FINISHER(std::string state_name, decision_making::CallContext& ctx, decision_making::EventQueue& queue): state_name(state_name),ctx(ctx),queue(queue){}\
			~_STATE_FINISHER(){\
				ON_FSM_STATE_END(state_name, ctx, queue);\
			} void r(){}\
		}_e(state_name, call_ctx, *events_queue);_e.r();

#define FSM_CONTEXT state_call_ctx

#define FSM_STATE(X)  \
			}}}break; \
			case X: { \
				decision_making::ScoppedThreads SUBMACHINESTHREADS; \
				__STARTOFSTATE(X)  __ENDOFSTATE

#define FSM_NEXT(STATE) \
				state = STATE; \
				break;
#define FSM_ON_EVENT(EVENT, DO) \
			if(event==decision_making::Event(EVENT,state_call_ctx)){ \
				DMDEBUG( cout<<" GOTO("<<fsm_name<<":"<<decision_making::Event(EVENT,call_ctx)<< "->" #DO ") "; ) \
				DO;\
			}

#define FSM_EVENT(EVENT) decision_making::Event(#EVENT,state_call_ctx))

#define FSM_ON_CONDITION(COND, DO) \
			if(COND){ \
				DMDEBUG( cout<<" GOTO("<<fsm_name<<":"<<decision_making::Event(#COND,state_call_ctx)<< "->" #DO ") "; ) \
				DO;\
			}

#define FSM_RAISE(EVENT) \
			DMDEBUG( cout<<" RAISE("<<fsm_name<<":"<<decision_making::Event(EVENT, state_call_ctx)<<") "; ) \
			events_queue->raiseEvent(decision_making::Event(EVENT, state_call_ctx));

//Deprecated
#define FSM_RISE(EVENT) FSM_RAISE(EVENT)

#define FSM_EVENTS_DROP events_queue->drop_all();

#define __DEFSUBEVENTQUEUE(TASK) decision_making::ScoppedThreads::EventQueuePtr events_queu##TASK( new decision_making::EventQueue(events_queue) );
#define __DEFSUBCTEXT(TASK) decision_making::ScoppedThreads::CallContextPtr call_ctx##TASK( new decision_making::CallContext(state_call_ctx, #TASK) );
#define __SHR_TO_REF(X) (*(X.get()))
#define FSM_CALL_TASK(TASK) \
			__DEFSUBEVENTQUEUE(TASK) __DEFSUBCTEXT(TASK) \
			SUBMACHINESTHREADS.add(events_queu##TASK); \
			SUBMACHINESTHREADS.add(call_ctx##TASK); \
			SUBMACHINESTHREADS.add(\
				new boost::thread(  CALL_REMOTE(TASK, boost::ref(__SHR_TO_REF(call_ctx##TASK)), boost::ref(__SHR_TO_REF(events_queu##TASK)))  ));

#define FSM_CALL_FSM(NAME) \
			__DEFSUBEVENTQUEUE(NAME) \
			SUBMACHINESTHREADS.add(events_queu##NAME); \
			SUBMACHINESTHREADS.add(\
					new boost::thread(boost::bind(&Fsm##NAME, &state_call_ctx, events_queu##NAME.get())  ));


#define FSM_CALL_BT(NAME) \
			__DEFSUBEVENTQUEUE(NAME) __DEFSUBCTEXT(NAME) \
			SUBMACHINESTHREADS.add(events_queu##NAME); \
			SUBMACHINESTHREADS.add(call_ctx##NAME); \
			decision_making::EventQueue& __t_events_queu##NAME = __SHR_TO_REF(events_queu##NAME);\
			decision_making::CallContext& __t_call_ctx##NAME = __SHR_TO_REF(call_ctx##NAME);\
			__BT_CREATE_BT_CALL_FUNCTION(NAME, __t_call_ctx##NAME, __t_events_queu##NAME)\
			SUBMACHINESTHREADS.add(\
				__CALL_BT_FUNCTION(NAME, boost::ref(__t_call_ctx##NAME), boost::ref(__t_events_queu##NAME))  \
			);

#define FSM_ON_STATE_EXIT_BGN \
			class __ON_STATE_EXIT_STRUCT:public decision_making::ScoppedThreadsOnExit{public:\
				__ON_STATE_EXIT_STRUCT(CallContext& state_call_ctx, EventQueue* events_queue):decision_making::ScoppedThreadsOnExit(state_call_ctx, events_queue){}\
				virtual void exit(){

#define FSM_ON_STATE_EXIT_END \
				}\
			};\
			decision_making::ScoppedThreadsOnExit* __tmp___ON_STATE_EXIT_STRUCT = new __ON_STATE_EXIT_STRUCT(state_call_ctx, events_queue);\
			SUBMACHINESTHREADS.add(decision_making::ScoppedThreads::ScoppedThreadsOnExitPtr(__tmp___ON_STATE_EXIT_STRUCT));

#define FSM_ON_STATE_EXIT(...) } __VA_ARGS__ {

#define FSM_STOP(EVENT, RESULT) \
			fsm_stop=true; \
			FSM_RAISE(EVENT); \
			fsm_result = RESULT; \
			break;

#define __CLEAN_THREAD_AND_EVENTS decision_making::ScoppedThreads::Cleaner SUBMACHINESTHREADSCLEANER(SUBMACHINESTHREADS);
#define FSM_TRANSITIONS  __CLEAN_THREAD_AND_EVENTS {Event event; while((event=events_queue->waitEvent())==true){

#define FSM_DROP_EVENTS events_queue->drop_all();

#define FSM_PRINT_EVENT if(not event.equals(decision_making::Event::SPIN_EVENT())){ cout<<" READ("<<fsm_name<<":"<<event<<") "; }

}


#endif /* HQ_FSM_H_ */
