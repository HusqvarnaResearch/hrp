 /*************************************************************
 *
 *   hq_EventSystem.h
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
 * EventSystem.h
 *
 *  Created on: Nov 14, 2013
 *      Author: dan
 */

#ifndef HQ_EVENTSYSTEM_H_
#define HQ_EVENTSYSTEM_H_


#include <boost/thread.hpp>
#include <deque>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/foreach.hpp>
#include <boost/regex.hpp>
#include <boost/bind.hpp>

namespace decision_making{
	using namespace std;

class CallContextParameters{
public:
		virtual ~CallContextParameters(){}
		typedef boost::shared_ptr<CallContextParameters> Ptr;
		virtual std::string str()const=0;

};
struct CallContext{
private:
	CallContextParameters::Ptr _parameters;
public:
	vector<string> stack;

	CallContext(const CallContext& ctx, string name){
		if(ctx.stack.size()>0)
			stack = ctx.stack;
		push(name);
		_parameters = ctx._parameters;
		//cout<<"[ ctx created : "<<str()<<" ]";
	}
	CallContext(string name){
		push(name);
		//cout<<"[ ctx created : "<<str()<<" ]";
	}
	CallContext(){
		//cout<<"[ ctx created : "<<str()<<" ]";
	}
	~CallContext(){
		//cout<<"[ ctx deleted : "<<str()<<" ]";
	}
	string str()const{
		if(stack.size()==0) return "/";
		stringstream path;
		BOOST_FOREACH(string s, stack){
			path<<"/"<<s;
		}
		return path.str();
	}
	void push(string name){ stack.push_back(name); }
	void pop(){ stack.pop_back(); }

	template<class A>
	void createParameters(A* a= new A()){
		_parameters = CallContextParameters::Ptr(a);
	}
	bool isParametersDefined()const{ return _parameters.get()!=NULL; }
	struct ExceptionParametersUndefined{};
	template<class A>
	A& parameters()const{
		if(isParametersDefined()==false) throw ExceptionParametersUndefined();
		return *(boost::static_pointer_cast<A>(_parameters).get());
	}
	template<class A>
	A& parameters(){
		if(isParametersDefined()==false) createParameters<A>();
		return *(boost::static_pointer_cast<A>(_parameters).get());
	}
};
typedef CallContext FSMCallContext;

struct Event{
	string _name;
	Event(string lname, const CallContext& ctx){
		if(lname.size()==0){ _name=lname; return; }
		if(lname[0]=='/'){ _name=lname; return; }
		if(lname[0]=='@' and lname.size()<3){ _name=ctx.str(); return; }
		if(lname[0]=='@'){
			if(lname[1]=='/') _name = lname;
			else _name = '@'+ctx.str()+"/"+lname.substr(1);
			return;
		}
		_name = ctx.str()+"/"+lname;
	}
	Event(string lname = ""){
		if(lname.size()==0){ _name=lname; return; }
		if(lname[0]=='/'){ _name = lname; return; }
		_name = "/"+lname;
	}
	Event(const char _lname[]){
		string lname(_lname);
		if(lname.size()==0){ _name=lname; return; }
		if(lname[0]=='/'){ _name = lname; return; }
		_name = "/"+lname;
	}
	string name()const{ return _name; }
	string event_name()const{
		size_t i=0,l=i;
		while(i!=string::npos){
			l=i; i=_name.find('/',l+1);
		}
		return _name.substr(l+1,_name.size());
	}
	bool isUndefined()const{ return _name.size()==0; }
	bool isDefined()const{ return not isUndefined(); }
	bool isRegEx()const{ return _name.size()>0 and _name[0]=='@'; }
	bool equals(const Event& e)const{
		if(e.isRegEx() and !isRegEx()){ return e.regex(_name); }
		if(isRegEx() and !e.isRegEx()){ return regex(e._name); }
		return _name==e._name;
	}
	bool operator==(const Event& e)const{
		return equals(e);
	}
	bool operator!=(const Event& e)const{
		return not equals(e);
	}
	operator bool()const{ return isDefined(); }

	bool regex(std::string text)const{
		if(isRegEx()==false) return _name==text;
		boost::regex e(_name.substr(1));
		return regex_match(text, e);
	}
	static Event SPIN_EVENT(){ return Event("/SPIN"); }
};
inline std::ostream& operator<<(std::ostream& o, Event t){
	return o<<"E["<<t.name()<<']';
}

class EventQueue{
protected:
	const bool isTransit;
	mutable boost::mutex events_mutex;
	boost::condition_variable on_new_event;
	std::deque<Event> events;
	bool events_system_stop;
	std::deque<EventQueue*> subs;
	EventQueue* parent;
	int max_unreaded_events_number;
	static const int max_unreaded_events_number_dif=1000;
#	define MUEN max_unreaded_events_number(max_unreaded_events_number_dif)
public:
	EventQueue(EventQueue* parent):isTransit(false),events_system_stop(false),parent(parent),MUEN{
		if(parent){
			max_unreaded_events_number = parent->max_unreaded_events_number;
			parent->subscribe(this);
		}
	}
	EventQueue(EventQueue* parent, bool isTransit):isTransit(isTransit), events_system_stop(false),parent(parent),MUEN{
		if(parent){
			max_unreaded_events_number = parent->max_unreaded_events_number;
			parent->subscribe(this);
		}
	}
	EventQueue(int muen = max_unreaded_events_number_dif):isTransit(false),events_system_stop(false),parent(NULL),MUEN{
		max_unreaded_events_number = (muen);
	}
#	undef MUEN
	virtual ~EventQueue(){
		if(parent)
			parent->remove(this);
		{
			boost::mutex::scoped_lock l(events_mutex);
			events_system_stop = true;
			on_new_event.notify_all();
			BOOST_FOREACH(EventQueue* sub, subs)
				sub->close();
		}
	}

	virtual void raiseEvent(const Event& e){
		if(parent) parent->raiseEvent(e);
		else addEvent(e);
	}

	//Deprecated
	virtual void riseEvent(const Event& e){ raiseEvent(e); }

private:
	void addEvent(Event e){
		boost::mutex::scoped_lock l(events_mutex);
		bool notify = true;
		if(not isTransit){
			if(events.empty()){
				events.push_back(e);
			}else{
				if(e==Event::SPIN_EVENT()){
				}else{
					if(events.size() > max_unreaded_events_number){
						events.pop_front();
					}
					events.push_back(e);
				}
			}
		}
		BOOST_FOREACH(EventQueue* sub, subs) sub->addEvent(e);
		if(notify) on_new_event.notify_one();
	}
public:
	virtual Event waitEvent(){
		boost::mutex::scoped_lock l(events_mutex);
		while(events_system_stop==false && events.empty())	on_new_event.wait(l);
		if(events_system_stop)
			return Event();
		Event e = events.front();
		events.pop_front();
		return e;
	}
	Event tryGetEvent(bool& success){
		boost::mutex::scoped_lock l(events_mutex);
		success = false;
		if(events_system_stop or events.empty())
			return Event();
		Event e = events.front();
		events.pop_front();
		success = true;
		return e;
	}
	void drop_all(){
		boost::mutex::scoped_lock l(events_mutex);
		events.clear();
	}
public:
	void close(){
		boost::mutex::scoped_lock l(events_mutex);
		events_system_stop = true;
		on_new_event.notify_all();
		BOOST_FOREACH(EventQueue* sub, subs)
			sub->close();
	}
public:
	void subscribe(EventQueue* sub){
		boost::mutex::scoped_lock l(events_mutex);
		if(events_system_stop) return;
		subs.push_back(sub);
	}
private:
	void remove(EventQueue* sub){
		boost::mutex::scoped_lock l(events_mutex);
		for(deque<EventQueue*>::iterator i=subs.begin();i!=subs.end();i++){
			if((*i)==sub){
				subs.erase(i);
				break;
			}
		}
	}
public:
	bool isTerminated()const{
		boost::mutex::scoped_lock l(events_mutex);
		return events_system_stop;
	}
	typedef boost::shared_ptr<EventQueue> Ptr;
#define EQ_SPINNER_DIF_RATE 10
	class Spinner{
		friend class EventQueue;
		EventQueue& events;
		Spinner(EventQueue& events):events(events){}
		boost::thread_group threads;
		void spinOne(){
			events.riseEvent(Event::SPIN_EVENT());
			//cout<<"[spin]"<<endl;
		}
		void spin(double rate = EQ_SPINNER_DIF_RATE){
			while(events.check_external_ok() and not events.isTerminated()){
				spinOne();
				boost::this_thread::sleep(boost::posix_time::milliseconds(1000/rate));
			}
			if(not events.check_external_ok() and not events.isTerminated()){
				events.close();
			}
		}
		void start(double rate = EQ_SPINNER_DIF_RATE){
			threads.add_thread(new boost::thread(boost::bind(&EventQueue::Spinner::spin, this, rate)));
		}
	public:
		~Spinner(){ if(events.check_external_ok()) threads.join_all(); }
	};

	virtual bool check_external_ok(){return true;}
	void spinOne(){ Spinner s(*this); s.spinOne(); }
	void spin(double rate=EQ_SPINNER_DIF_RATE){ Spinner s(*this); s.spin(rate); }
	void async_spin(double rate=EQ_SPINNER_DIF_RATE, double start_delay=0.0){
		boost::this_thread::sleep(boost::posix_time::seconds(start_delay));
		_spinner = boost::shared_ptr<Spinner>(new Spinner (*this));
		_spinner->start(rate);
	}
private:
	boost::shared_ptr<Spinner> _spinner;
};


class MapResultEvent{
	typedef std::map<int,string> mapper_item_type;
	typedef std::map<string, mapper_item_type> mapper_type;
public:
	static mapper_type& get(){ static mapper_type mapper; return mapper; }

	static string map(string task, int result){
		std::stringstream res; res<<result;
		mapper_type& mapper = get();
		if(mapper.find(task)==mapper.end()) return res.str();
		mapper_item_type& item = mapper[task];
		if(item.find(result)==item.end()) return res.str();
		return item[result];
	}
	static void map(string task, int result, string event){
		mapper_type& mapper = get();
		mapper[task][result]=event;
	}
};

}

#endif /* HQ_EVENTSYSTEM_H_ */
