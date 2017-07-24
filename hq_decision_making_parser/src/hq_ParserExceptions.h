/*************************************************************
 *
 *   hq_ParserExceptions.h
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
 * ParserExceptions.h
 *
 *  Created on: Dec 1, 2013
 *      Author: dan
 */

#ifndef HQ_PARSEREXCEPTIONS_H_
#define HQ_PARSEREXCEPTIONS_H_

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <errno.h>

using namespace std;

class ParserException{
protected:
    int _line;
    int _position;

	ParserException(std::string desc="", int line = 0, int position = 0)
        : desc(desc), _line(line), _position(position) { };
	virtual ~ParserException(){};
	std::string desc;
public:
	std::string what() const {
	    stringstream ss;
	    ss << desc;

	    if (_line > 0 && _position > 0)
	        ss << endl << "Line: " << _line << ", position: " << _position;

	    return ss.str();
	}

	template <class A>
	ParserException& operator<<(const A& a){
		std::stringstream s;
		s<<a;
		desc += s.str();
		return *this;
	}
};

class UnexpectedEndOfFile : public ParserException {
public:
    UnexpectedEndOfFile(string expectedString = "", int line = 0, int position = 0)
        : ParserException("Expected string '" + expectedString + "' not found", line, position) { }
};

class ClosingBracketNotFound : public ParserException {
public:
    ClosingBracketNotFound() : ParserException("Closing bracket not found") { }
};

class UnexpectedToken: public ParserException {
public:
    UnexpectedToken(string character, string expectedToken = "", int line = 0, int position = 0)
        : ParserException("Unexpected character: '" + character + "', expected: '" + expectedToken + "'", line, position) { }
};

class PEFileNotFound:public ParserException{
public:
	PEFileNotFound(std::string filename):ParserException("File "+filename+" does not found. ")
	{
		(*this)<<"\n\tIOError: "<<strerror( errno );
	}
};

class PEFileNotCreated:public ParserException{
public:
	PEFileNotCreated(std::string filename):ParserException("File "+filename+" cann't be open. ")
	{
		(*this)<<"\n\tIOError: "<<strerror( errno );
	}
};

#endif /* HQ_PARSEREXCEPTIONS_H_ */
