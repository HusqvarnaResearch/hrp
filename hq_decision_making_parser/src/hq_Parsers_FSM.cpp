/*************************************************************
 *
 *   hq_Parsers_FSM.cpp
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
 // http://www.compileonline.com/compile_cpp0x_online.php
#include <iostream>
#include <fstream>

#include "hq_Tokenizer.h"
#include "hq_FSMConstructor.h"
#include "hq_ParserExceptions.h"

using fsm_constructor::FSMConstructor;

#define PRINT(X) //cout<<"parser: "<<X<<endl

class FSMParser {
public:
	enum TokenType{
		tkn_null=0,
		tkn_text=1,
		tkn_fsm,
		tkn_fsm_start,
		tkn_fsm_bgn,
		tkn_fsm_end,
		tkn_state,
		tkn_call_task,
		tkn_call_fsm,
		tkn_call_bt,
		tkn_transitions,
		tkn_on_event,
		tkn_on_condition,
		tkn_raise,
		tkn_states,
		tkn_next,

		tkn_bopen,
		tkn_bclose,
		tkn_fopen,
		tkn_fclose,
		tkn_semicol,
		tkn_col,
		tkn_slesh,
	};



	//============= TOKENAZER ===============================

	struct TokenizerInit{
		void init(Parser::TokenizerData<TokenType>& tkn){
			tkn.string_token["FSM"]=tkn_fsm;
			tkn.string_token["FSM_START"]=tkn_fsm_start;
			tkn.string_token["FSM_BGN"]=tkn_fsm_bgn;
			tkn.string_token["FSM_END"]=tkn_fsm_end;
			tkn.string_token["FSM_STATE"]=tkn_state;
			tkn.string_token["FSM_NEXT"]=tkn_next;
			tkn.string_token["FSM_CALL_FSM"]=tkn_call_fsm;
			tkn.string_token["FSM_CALL_TASK"]=tkn_call_task;
			tkn.string_token["FSM_CALL_BT"]=tkn_call_bt;
			tkn.string_token["FSM_TRANSITIONS"]=tkn_transitions;
			tkn.string_token["FSM_ON_EVENT"]=tkn_on_event;
			tkn.string_token["FSM_ON_CONDITION"]=tkn_on_condition;
			tkn.string_token["FSM_RAISE"]=tkn_raise;
			tkn.string_token["FSM_RISE"]=tkn_raise;
			tkn.string_token["FSM_STATES"]=tkn_states;

			tkn.spec_token['(']=tkn_bopen;
			tkn.spec_token[')']=tkn_bclose;
			tkn.spec_token['{']=tkn_fopen;
			tkn.spec_token['}']=tkn_fclose;
			tkn.spec_token[',']=tkn_col;
			tkn.spec_token[';']=tkn_semicol;
			tkn.spec_token['/']=tkn_slesh;
		}
		bool isTokenSumbol(char c){
			return
				('0'<=c and c<='9') or
				('a'<=c and c<='z') or
				('A'<=c and c<='Z') or
				(c=='_')
			;
		}
		bool isDelimiter(char c){
			return not isTokenSumbol(c);
		}
		bool isFlowControl(char c){
			return c==10 or c==13;
		}
	};

	//============= LEXEM PARSER =================================

	typedef Parser::TokenStream<TokenType> tstream;
	typedef Parser::Token<TokenType> Token;

	bool fsm(tstream& stream, Token& tkn){
		PRINT("start search from "<<stream.first());
		while(not stream.eof()){
			stream >> tkn;
			if( tkn.type == tkn_fsm ){
				PRINT("fsm found");
				return true;
			}
		}
		return false;
	}

	bool name(tstream& stream, string txt, string& output, int& line, int& pos){
		Token tkn;
		TKN_NEXT( tkn_bopen )
		TKN_NEXT( tkn_text )
		PRINT(txt<<" name is "<<tkn.text);
		output = tkn.text;
		searchLineInfo(tkn.start, line, pos);
		TKN_NEXT( tkn_bclose )
		return true;
	}
	bool name(tstream& stream, string txt, string& output){
		int line, pos;
		return name(stream,txt, output, line, pos);
	}
	bool name(tstream& stream, string txt){ string t; return name(stream,txt,t); }


	bool fsm_start(tstream& stream){
		Token tkn;
		while(not stream.eof()){
			stream >> tkn;
			if( tkn.type == tkn_fsm_start ){
				string start_state;
				TKN_SEARCH( name(stream, "fsm start", start_state) )
				constructor.fsm().start = start_state;
				return true;
			}
		}
		ADD_ERROR(_curr_fsm, "Cann't find FSM_START for FSM("+_curr_fsm_name+")")
		return false;
	}
	bool fsm_bgn(tstream& stream){
		Token tkn;
		while(not stream.eof()){
			stream >> tkn;
			if( tkn.type == tkn_fsm_bgn ){
				PRINT("fsm_bgn found");
				return true;
			}
		}
		ADD_ERROR(_curr_fsm, "Cann't find FSM_BGN for FSM("+_curr_fsm_name+")")
		return false;
	}

	bool transition_name(tstream& stream){
		Token tkn;
		TKN_NEXT( tkn_bopen )
		TKN_NEXT( tkn_text )
		PRINT("on event "<<tkn.text;
		TKN_NEXT( tkn_col )
		TKN_NEXT( tkn_text )
		cout<<" go to "<<tkn.text);
		TKN_NEXT( tkn_bclose )
		return true;
	}
	bool textUpTo(tstream& stream, TokenType endTkn, string& output){
		Token tkn;
		bool r=false;
		int s(-1),e(-1);
		while(not stream.eof() and tkn.type!=endTkn){
			stream >> tkn;
			r=true;
			if( tkn.type==endTkn ) continue;
			if(s<0)s=tkn.start;
			e=tkn.end;
		}
		if(s>=0 and e>s) output = fullText.substr(s, e-s);
		return r;
	}

	bool search_calls_in_state(tstream& stream, Token& tkn){
		if( tkn.type == tkn_call_fsm ){
			PRINT("call fsm");
			fsm_constructor::Call call; call.type = "fsm"; call.file=filename; call.id=constructor.fsm().state().getId();
			TKN_SEARCH( name(stream, "called fsm", call.text, call.line, call.pos) )
			constructor.fsm().state().calls.push_back(call);
		}
		if( tkn.type == tkn_call_bt ){
			PRINT("call bt");
			fsm_constructor::Call call; call.type = "bt"; call.file=filename; call.id=constructor.fsm().state().getId();
			TKN_SEARCH( name(stream, "called bt", call.text, call.line, call.pos) )
			constructor.fsm().state().calls.push_back(call);
		}
		if( tkn.type == tkn_call_task ){
			PRINT("call task");
			fsm_constructor::Call call; call.type = "task"; call.file=filename; call.id=constructor.fsm().state().getId();
			TKN_SEARCH( name(stream, "called task", call.text, call.line, call.pos) )
			constructor.fsm().state().calls.push_back(call);
		}
		return true;
	}
	bool search_raises_in_state(tstream& stream, Token& tkn){
		if( tkn.type == tkn_raise ){
			TKN_NEXT(tkn_bopen)
			string text="";
			TKN_SEARCH( textUpTo(stream, tkn_bclose, text));
			PRINT("raise event: "<<text);
			fsm_constructor::Raise raise; raise.text=text;
			constructor.fsm().state().raises.push_back(raise);
		}
		return true;
	}
	bool search_next_in_events(tstream& stream, Token& tkn, bool& found){
		if( tkn.type == tkn_next ){
			string text;
			TKN_SEARCH( name(stream, "transition", text) )
			found=true;
			fsm_constructor::EventAction ac; ac.type="next";
			ac.text=text;
			constructor.fsm().state().event().actions.push_back(ac);
		}
		return true;
	}
	bool search_raise_in_events(tstream& stream, Token& tkn, bool& found){
		if( tkn.type == tkn_raise ){
			TKN_NEXT(tkn_bopen)
			string text="";
			TKN_SEARCH( textUpTo(stream, tkn_bclose, text));
			PRINT("raise event: "<<text);
			found = true;
			fsm_constructor::EventAction ac; ac.type="raise";
			ac.text=text;
			constructor.fsm().state().event().actions.push_back(ac);
		}
		return true;
	}
	bool search_on_events_in_transition(tstream& stream, Token& tkn){
		if( tkn.type == tkn_on_event ){
			TKN_NEXT( tkn_bopen)
			string text="";
			TKN_SEARCH( textUpTo(stream, tkn_col, text));
			PRINT("on event : "<<text);
			constructor.fsm().state().create_event();
			constructor.fsm().state().event().type="event";
			constructor.fsm().state().event().text=text;
			bool found_next = false, found_raise = false;
			stream >> tkn;
			TKN_SEARCH( search_next_in_events(stream, tkn, found_next) )
			TKN_SEARCH( search_raise_in_events(stream, tkn, found_raise) )
			if(found_next==false and found_raise==false){
				constructor.fsm().state().drop_event();
				return false;
			}
			TKN_NEXT( tkn_bclose)
			constructor.fsm().state().add_event();
		}
		return true;
	}
	string encode(const std::string& data) {
	    std::string buffer;
	    buffer.reserve(data.size()*1.1);
	    for(size_t pos = 0; pos != data.size(); ++pos) {
	        switch(data[pos]) {
	            case '&':  buffer.append("&amp;");       break;
	            case '\"': buffer.append("&quot;");      break;
	            case '\'': buffer.append("&apos;");      break;
	            case '<':  buffer.append("&lt;");        break;
	            case '>':  buffer.append("&gt;");        break;
	            default:   buffer.append(&data[pos], 1); break;
	        }
	    }
	    //data.swap(buffer);
	    return buffer;
	}
	bool search_on_conditions_in_transition(tstream& stream, Token& tkn){
		if( tkn.type == tkn_on_condition ){
			TKN_NEXT( tkn_bopen)
			string text="";
			TKN_SEARCH( textUpTo(stream, tkn_col, text));
			PRINT("on condition : "<<text);
			constructor.fsm().state().create_event();
			constructor.fsm().state().event().type="condition";
			constructor.fsm().state().event().text=encode(text);
			bool found_next = false, found_raise = false;
			stream >> tkn;
			TKN_SEARCH( search_next_in_events(stream, tkn, found_next) )
			TKN_SEARCH( search_raise_in_events(stream, tkn, found_raise) )
			if(found_next==false and found_raise==false){
				constructor.fsm().state().drop_event();
				return false;
			}
			TKN_NEXT( tkn_bclose)
			constructor.fsm().state().add_event();
		}
		return true;
	}

	bool transitions(tstream& stream){
		Token tkn;
		size_t current_end = stream.getEnd();
		bool transitions=false;
		while(not stream.eof()){
			stream >> tkn;
			TKN_SEARCH( search_calls_in_state(stream , tkn) )
			TKN_SEARCH( search_raises_in_state(stream , tkn) )
			if( tkn.type == tkn_transitions ){
				PRINT("transition");
				TKN_NEXT_OPTIONAL(tkn_fopen, return true)
				size_t end_of_tran = TKN_SEARCH_CLOSE_PARENT(end_of_tran, stream, tkn_fopen, tkn_fclose, "FSM Transitions definition")
				stream.setEnd(end_of_tran);
				transitions = true;
			}
			if(transitions){
				TKN_SEARCH_OPTIONAL( search_on_events_in_transition(stream , tkn) , {})
				TKN_SEARCH_OPTIONAL( search_on_conditions_in_transition(stream , tkn) , {})
			}
		}
		stream.setEnd(current_end);
		return true;
	}

	bool fsm_state_body(tstream& stream, size_t end_of_fsm){
		Token tkn;
		TKN_NEXT( tkn_fopen )
		size_t end_of_state = TKN_SEARCH_CLOSE_PARENT(end_of_state, stream, tkn_fopen, tkn_fclose, "FSM State definition")
		stream.setEnd(end_of_state);
		TKN_SEARCH( transitions(stream) )
		stream.setEnd(end_of_fsm);
		return true;
	}

	bool fsm_states(tstream& stream, size_t end_of_fsm){
		Token tkn;
		while(not stream.eof()){
			stream >> tkn;
			if( tkn.type == tkn_fsm_end ){
				PRINT("fsm_end found");
				return true;
			}
			if( tkn.type == tkn_state ){
				PRINT("state found");
				_curr_state = tkn;
				string state_name;
				TKN_SEARCH( name(stream, "state", state_name) )
				_curr_state_name = state_name;
				constructor.fsm().create();
				constructor.fsm().state().name=state_name;
				if( fsm_state_body(stream, end_of_fsm) ){
					constructor.fsm().add();
				}else{
					constructor.fsm().drop();
					ADD_ERROR(_curr_state, "Error in state "+_curr_state_name+" definition")
					return false;
				}
			}
		}
		ADD_ERROR(_curr_fsm, "Cann't find FSM_END for FSM("+_curr_fsm_name+")")
		return false;
	}

	bool parse_fsm_body(tstream& stream){
		Token tkn;
		TKN_NEXT( tkn_fopen )
		size_t end_of_fsm = TKN_SEARCH_CLOSE_PARENT(end_of_fsm, stream, tkn_fopen, tkn_fclose, "FSM definition")
		stream.setEnd(end_of_fsm);
		TKN_SEARCH( fsm_start(stream) )
		TKN_SEARCH( fsm_bgn(stream) )
		TKN_SEARCH( fsm_states(stream, end_of_fsm) )
		stream.setEnd();
		return true;
	}
	bool parse_fsm(tstream& stream){
		Token tkn;
		TKN_SEARCH( fsm(stream, tkn) )
		_curr_fsm = tkn;
		string fsm_name;
		TKN_SEARCH( name(stream, "fsm", fsm_name) )
		_curr_fsm_name = fsm_name;
		constructor.create();
		constructor.fsm().name = fsm_name;
		if( parse_fsm_body(stream) ){
			constructor.add();
		}else{
			constructor.drop();
			return false;
		}
		return true;
	}

	struct TokenizerContext{
		Parser::Tokenizer<TokenType, TokenizerInit> tokenizer;
		std::stringstream ss_fullText;
		stringstream buffer;
		size_t start_index, index;
		ifstream& file;
		char c;
		TokenizerContext(ifstream& file):
		start_index(0),index(0),
		file(file), c(0)
		{}
	};

	void searchLineInfo(int i, int& line, int& pos){
		line=0;
		for(size_t n=0;n<lines.size();n++){
			if(i<lines[n]){
				pos = i-lines[line-2];
				return;
			}
			line = n+2;
		}
	}

	void saveReadedChar(TokenizerContext& ctx , char c){
		ctx.ss_fullText<<c;
		if(c=='\n') lines.push_back(ctx.index);
	}

	bool skipComments( TokenizerContext& ctx )
	{
		if(ctx.c=='/' and ctx.file.eof()==false){
			char cc;
			ctx.file.read(&cc, 1); ctx.ss_fullText<<cc;
			if(cc=='/'){
				ctx.index++;
				while(cc!='\n' and ctx.file.eof()==false){
					ctx.file.read(&cc, 1);  saveReadedChar(ctx, cc); ctx.index++;
				}
				ctx.index++;
				return true;
			}
			else{
				ctx.tokenizer.searchToken(ctx.index++, ctx.buffer, ctx.start_index, ctx.c, tokens);
				ctx.c=cc;
			}
		}
		return false;
	}

	FSMParser(string file):filename(file), constructor(errors, file){}

	string filename;
	std::string fullText;
	tstream tokens;
	FSMConstructor constructor;
	std::stringstream errors;
	std::vector<int> lines;
	Token _curr_fsm;
	std::string _curr_fsm_name;
	Token _curr_state;
	std::string _curr_state_name;

	FSMConstructor&  main()
	{
		PRINT(__TIME__);
		if(true){
			PRINT("Read input file");
			ifstream file(filename.c_str());
			TokenizerContext ctx(file);
			if(ctx.file.is_open()){
				PRINT("File ("<<filename<<") is open");
				while(ctx.file.eof()==false){
					ctx.file.read(&ctx.c, 1); saveReadedChar(ctx, ctx.c);
					if(skipComments(ctx)) continue;
					ctx.tokenizer.searchToken(ctx.index++, ctx.buffer, ctx.start_index, ctx.c, tokens);
				}
				PRINT("End of file");
				fullText = ctx.ss_fullText.str();
				file.close();
			}else{
				throw PEFileNotFound(filename);
			}
		}
		if(false){
			PRINT("Result: ");
			while(not tokens.eof()){
				Token t; tokens>>t;
				PRINT(t);
			}
		}
		if(true){
			while(not tokens.eof()){
				parse_fsm(tokens);
			}

			//PRINT(constructor);
		}
		return constructor;
	}
};

ostream& operator<<(ostream& out, FSMParser::TokenType t){
	#define PRINTTKN(x) case FSMParser::tkn_##x: return out<<#x;
	switch(t){
		PRINTTKN(null)
		PRINTTKN(fsm)
		PRINTTKN(fsm_start)
		PRINTTKN(fsm_bgn)
		PRINTTKN(fsm_end)
		PRINTTKN(state)
		PRINTTKN(call_task)
		PRINTTKN(call_fsm)
		PRINTTKN(next)
		PRINTTKN(call_bt)
		PRINTTKN(transitions)
		PRINTTKN(on_event)
		PRINTTKN(on_condition)
		PRINTTKN(raise)
		PRINTTKN(states)

		PRINTTKN(bopen)
		PRINTTKN(bclose)
		PRINTTKN(fopen)
		PRINTTKN(fclose)
		PRINTTKN(semicol)
		PRINTTKN(col)
		PRINTTKN(text)
		PRINTTKN(slesh)
	}
	return out<<"???";
	#undef PRINTTKN
}

FSMConstructor& parseFSM(FSMParser* p){
	return p->main();
}

FSMParser* createFSM(std::string filename){
	return new FSMParser(filename);
}

void del(FSMParser* p){ delete p; }

#undef PRINT
