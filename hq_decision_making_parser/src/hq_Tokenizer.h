/*************************************************************
 *
 *   hq_Tokenizer.h
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
 * Parser.h
 *
 *  Created on: Nov 11, 2013
 *      Author: dan
 */

#ifndef HQ_TOKENIZER_PARSER_H_
#define HQ_TOKENIZER_PARSER_H_


#include <string>
#include <sstream>
#include <map>
#include <vector>

using namespace std;

namespace Parser{

inline string str(char c){
    stringstream s;
    if(c<20) s<<(int)c;
    else s<<c;
    return s.str();
}

template<class TokenType>
struct Token{
    TokenType type;
    string text;
    size_t start, end;
    Token(TokenType type=TokenType(0), string text=""):type(type),text(text),start(0),end(0){}
    size_t size()const{ if(end<start) return 0; return end-start; }
};
template<class TokenType>
static ostream& operator<<(ostream& out, const Token<TokenType>& t){
    out<<t.type;
    if(t.type==1) return out<<"{"<<t.start<<":"<<t.text<<":"<<t.end<<"}";
    return out;
}

struct tstream_state{
	size_t i;
	tstream_state(size_t i):i(i){}
};
template<class TokenType>
struct TokenStream{
	vector< Token<TokenType> > st;
	size_t i;
	size_t end;
	TokenStream():i(0),end((size_t)-1){}
	void clear(){ i=0;st.clear(); }
	void reset(){ i=0; }
	TokenStream<TokenType>& operator<<(Token<TokenType> tkn){st.push_back(tkn);return *this;}
	TokenStream<TokenType>& operator>>(Token<TokenType>& tkn){if(eof()) return *this; tkn=st[i++]; return *this;}
	TokenStream<TokenType>& operator<<(TokenType tkn){st.push_back(Token<TokenType>(tkn,""));return *this;}
	TokenStream<TokenType>& operator>>(TokenType& tkn){if(eof()) return *this; tkn=st[i++].type; return *this;}
	TokenStream<TokenType>& operator<<(string tkn){st.push_back(Token<TokenType>(1,tkn));return *this;}
	TokenStream<TokenType>& operator>>(string& tkn){if(eof()) return *this; tkn=st[i++].text; return *this;}
	const Token<TokenType>& first()const{ return st[i]; }
	const Token<TokenType>& last()const{ return st[st.size()-1]; }
	size_t count()const{ return st.size()-i; }
	bool eof()const{ if(end==(size_t)-1) return i>=st.size(); else return i>=end; }
	void setEnd(size_t e = (size_t)-1){ if(e<=st.size() or e==size_t(-1)) end=e; }
	size_t getEnd()const{return end;}
	tstream_state state()const{ return tstream_state(i); }
	void state(tstream_state t){ i=t.i; }
};

template<class TokenType>
class TokenizerData{
public:
	map<string,TokenType> string_token;
	map<char,TokenType> spec_token;
};
template<class TokenType, class Init>
class Tokenizer:public TokenizerData<TokenType>{
public:
	Init p;
	bool in_string;
	bool prev_slash;

	template<class T,class B>
	bool contains(const map<T,B>& m, const T& t){ return m.find(t)!=m.end(); }
	Tokenizer():in_string(false), prev_slash(false){
		p.init(*this);
	}
	void searchToken(size_t index, stringstream& buf, size_t& start_index, char c, TokenStream<TokenType>& tkn_stream){
	    const bool verb = false;
	    if(verb) cout<<"Proc ["<<str(c)<<"]"<<endl;
	    Token<TokenType> tkn;
	    if( (!in_string and p.isDelimiter(c)) or (in_string and c=='\"' and !prev_slash) ){
	        if(verb) cout<<"... is delimiter"<<endl;
	        if(contains(this->string_token, buf.str())){
	            if(verb) cout<<"... ... token found"<<endl;
	            tkn = this->string_token[buf.str()];
	        }else{
	            if(verb) cout<<"... ... token is not found. select as text"<<endl;
	            tkn = Token<TokenType>((TokenType)1, buf.str());
	        }
	        tkn.start = start_index;
	        tkn.end = index;
	        if(tkn.size()>0){
	            if(verb){ cout<<"... ... add token "<<tkn; cout <<std::endl; }
	            tkn_stream<<tkn;
	        }else{
	            if(verb) cout<<"... ... ignore token"<<endl;
	        }
	        buf.str("");
	        start_index = index;
	        if(contains(this->spec_token, c)){
	            if(verb) cout<<"... ...  is special token"<<endl;
	            tkn = this->spec_token[c];
	            stringstream tmp; tmp<<c;
	            tkn.text = tmp.str();
	            tkn.start=index;
	            tkn.end = index+1;
	            tkn_stream<<tkn;
	        }
            if( c == '\"' ){
                in_string = !in_string;
            }

	        start_index+=1;
	    }else{
	        if(verb) cout<<"... is not delimiter"<<endl;
	        buf<<c;
	    }
	    if(verb) cout<<"... current buffer is ["<<buf.str()<<"]"<<endl;
        prev_slash = c == '\\';
	}
};

#define ADD_ERROR( tkn, X )\
		{int l,p; searchLineInfo(tkn.end, l,p);\
		errors<<"In "<<filename<<":"<<l<<":"<<p<<" "<<tkn<<endl<<"    "<<X<<endl;}

#define TKN_SEARCH_CLOSE_PARENT(END, stream, topen, tclose, X) tkn_search_close_parent(stream, topen, tclose);\
    if(END==size_t(-1)){\
    	PRINT("Unexpected end of file during " X ".");\
    	int l,p; searchLineInfo(tkn.end, l,p);\
        errors<<"In "<<filename<<":"<<l<<":"<<p<<" "<<tkn<<endl<<"    Unexpected end of structure during " X " parsing."<<endl;\
        return false;\
    }

#define TKN_SEARCH_OPTIONAL(TKN, RET) if(not (TKN)) RET;
#define TKN_NEXT_SEARCH_OPTIONAL(TKN, RET) stream >> tkn; if(not (TKN)) RET;
#define TKN_NEXT_OPTIONAL(TKN, RET) TKN_NEXT_SEARCH_OPTIONAL(tkn.type==TKN, RET)

#define TKN_NEXT(TKN) TKN_NEXT_OPTIONAL(TKN, return false)
#define TKN_SEARCH(TKN) TKN_SEARCH_OPTIONAL(TKN, return false)
#define TKN_NEXT_SEARCH(TKN) TKN_NEXT_SEARCH_OPTIONAL(TKN, return false)


template<class TokenType>
size_t tkn_search_close_parent(TokenStream<TokenType>& stream, TokenType topen, TokenType tclose){
    tstream_state state = stream.state();
    int c = 0;
    Token<TokenType> tkn;
    while(c>=0 and not stream.eof()){
        stream >> tkn;
        if(tkn.type == topen) c++;
        if(tkn.type == tclose)c--;
    }
    size_t res = size_t(-1);
    if(c<0) res = stream.i;
    stream.state(state);
    return res;
}

}

#endif /* HQ_TOKENIZER_PARSER_H_ */
