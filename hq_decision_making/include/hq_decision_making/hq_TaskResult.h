 /*************************************************************
 *
 *   hq_TaskResult.h
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
 * TaskResult.h
 *
 *  Created on: Nov 14, 2013
 *      Author: dan
 */

#ifndef HQ_TASKRESULT_H_
#define HQ_TASKRESULT_H_

#include <boost/thread.hpp>
#include <deque>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>

namespace decision_making{

struct TaskResult{
private:
	int error_;
	std::string description_;
	TaskResult(int er, std::string desc):error_(er),description_(desc){}
	static const int undefied_code = INT_MAX;
	static const int error_start_code = 2;
	static const int preempted_code = 1;
public:
	TaskResult():error_(undefied_code),description_("UNDEF"){}
	static TaskResult SUCCESS(){ return TaskResult(0,"OK"); }
	static TaskResult FAIL(int er=error_start_code, std::string what="FIAL"){ return TaskResult(er, what); }
	static TaskResult FAIL(std::string what){ return TaskResult(error_start_code, what); }
	static TaskResult TERMINATED(){ return TaskResult(preempted_code, "TERMINATED"); }
	static TaskResult UNDEF(){ return TaskResult(undefied_code, "UNDEF"); }
	bool isFail()const{ return error_>=error_start_code and error_!=undefied_code; }
	bool isTermianted()const{ return error_==preempted_code; }
	bool isSuccess()const{ return error_<1; }
	bool isDefined()const{ return error_!=undefied_code; }
	bool isUndefined()const{ return not isDefined(); }
	int error()const{return error_;}
	std::string what()const{ return description_;}

	static int rerangeErrorCode(int code){ return error_start_code-1+code; }

	const bool operator==(const TaskResult& o)const{ return error_ == o.error_; }
	const bool operator!=(const TaskResult& o)const{ return error_ != o.error_; }
};
static std::ostream& operator<<(std::ostream& out, const TaskResult& res){
	out<<"Result{";
	if(res.isSuccess()) return out<<"SUCCESS}";
	if(res.isTermianted()) return out<<"TERMINATED}";
	if(res.isFail()) return out<<"Fail,"<<res.error()<<","<<res.what()<<"}";
	if(res.isUndefined()) return out<<"UNDEF}";
	return out<<"UNKNOWN,"<<res.error()<<","<<res.what()<<"}";
}

static string str(const TaskResult& res){
	stringstream s; s<<res; return s.str();
}

}


#endif /* HQ_TASKRESULT_H_ */
