/*************************************************************
 *
 *   hq_decision_making_parser.cpp
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
 * main.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: dan
 */



#include "hq_Parsers.h"
#include "hq_ParserExceptions.h"
#include <sstream>

using namespace std;

#define CREATE_PARSERS \
	FSMParser* fsmparser = 0;\
	struct GC{\
		FSMParser*& fsmparser;\
		std::string file;\
		GC(FSMParser*& fsmparser, std::string file):\
			fsmparser(fsmparser), file(file)\
		{\
			fsmparser = createFSM(file);\
		}\
		~GC(){\
			del(fsmparser);\
		}\
	} gc(fsmparser, file);

bool parseToXml(std::ostream& parsing_result, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);

		//set cross links from different types of parses

		xml_version(parsing_result, ""); parsing_result<<std::endl;
		fsm_constructor::saveXml(parsing_result, fsm)<<std::endl;

		errors << fsm.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}
bool parseToXml(std::string result_prefix, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);

		//set cross links from different types of parses

		fsm_constructor::saveXml(result_prefix, fsm);

		errors << fsm.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}

bool parseToDot(std::ostream& parsing_result, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);

		//set cross links from different types of parses

		fsm_constructor::saveDot(parsing_result, fsm)<<std::endl;

		errors << fsm.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}
bool parseToDot(std::string result_prefix, std::ostream& errors, std::string file){

	CREATE_PARSERS

	//Parse files and get references to internal structures
	try{

		fsm_constructor::FSMConstructor& fsm = parseFSM(fsmparser);

		//set cross links from different types of parses

		fsm_constructor::saveDot(result_prefix, fsm);

		errors << fsm.errors.str();

	}catch(const PEFileNotFound& err){
		errors << err.what() << std::endl;
		return false;
	}

	return true;
}


vector<string> split(string text, char del=':'){
	stringstream in(text);
	vector<string> res;
	char ch;
	while(in.eof()==false){
		in >> ch;
		stringstream word;
		while(in.eof()==false and ch!=del){
			word << ch;
			in >> ch;
		}
		if(word.str().size()>0)
			res.push_back(word.str());
	}
	return res;
}

#define SearchFlag(NAME) searchFlag(argc, argv, NAME)
int searchFlag(int argc, char** argv, string name){
	for(int i=0;i<argc;i++){
		string v(argv[i]);
		if( v == name ) return i;
	}
	return -1;
}
#define SearchValue(NAME) searchValue(argc, argv, NAME)
int searchValue(int argc, char** argv, string name){
	int i = searchFlag(argc, argv, name);
	if(i<0 or i==argc-1) return -1;
	return i+1;
}

int main(int argc, char** argv){

	cout<<"-- Start decision making parsing"<<endl;
	string exe ( argv[0] );
	string print_mode = "PRINT_ERRORS";
	if(SearchFlag("-pe")>0) print_mode = "PRINT_ERRORS";
	if(SearchFlag("-pa")>0) print_mode = "PRINT_ALL";
	bool isParseDots = false;
	bool isParseXmls = false;
	if(SearchFlag("-dot")>0) isParseDots=true;
	if(SearchFlag("-xml")>0) isParseXmls=true;
	string project="";
	if(SearchValue("-src")>0){
		project = string( argv[SearchValue("-src")] );
	}
	string shared_folder="";
	if(SearchValue("-dst")>0){
		shared_folder = string( argv[SearchValue("-dst")] );
		if(shared_folder[shared_folder.size()-1]!='/'){
			shared_folder+="/";
		}
	}else{
		std::cerr << "Error: destination folder does not found in parameters. (use -dst flag)" << endl;
		return 1;
	}
	vector<string> files;
	if(SearchValue("-f")>0){
		files = split(string(argv[SearchValue("-f")]));
	}else{
		std::cerr << "Error: list of files does not found in parameters. (use -f flag and : as delimiter)" << endl;
		return 2;
	}


	cout<<"   -- parser path: "<<exe<<endl;
	cout<<"   -- project folder: "<<project<<endl;
	cout<<"   -- share folder: "<<shared_folder<<endl;
	if(print_mode == "PRINT_ALL")
		cout<<"files"<<endl;
	if(isParseXmls)
		cout<<"    -- parse XMLs"<<endl;
	if(isParseDots)
		cout<<"    -- parse DOTs"<<endl;

	for(size_t i=0;i<files.size();i++){
		std::string input_file = files[i];
		if(print_mode == "PRINT_ALL"){
			cout<<"    "<<i<<") parse "<<input_file<<endl;
			cout<<"       save to "<<shared_folder<<endl;
		}
		if(isParseXmls){
			//cout<<"    -- parse XMLs"<<endl;
			std::stringstream errors;
			parseToXml(shared_folder, errors, input_file);
			if(errors.str().size()>0){
				std::cerr << errors.str() <<std::endl;
			}
		}
		if(isParseDots){
			//cout<<"    -- parse DOTs"<<endl;
			std::stringstream errors;
			parseToDot(shared_folder, errors, input_file);
			if(not isParseXmls){
				if(errors.str().size()>0){
					std::cerr << errors.str() <<std::endl;
				}
			}
		}
	}

	cout<<"-- End decision making parsing"<<endl;
	return 0;
}
