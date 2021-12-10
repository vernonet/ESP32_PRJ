#pragma once

#include "AudioTools/Vector.h"
#include "AudioHttp/StrExt.h"
#include "AudioHttp/HttpLineReader.h" 
#include "AudioHttp/Url.h"
#include "AudioHttp/HttpTypes.h" 

namespace audio_tools {

// Class Configuration
const int MaxHeaderLineLength = 200;

// Define relevant header content
const char* CONTENT_TYPE = "Content-Type";
const char* CONTENT_LENGTH = "Content-Length";
const char* CONNECTION = "Connection";
const char* CON_CLOSE = "close";
const char* CON_KEEP_ALIVE = "keep-alive";
const char* TRANSFER_ENCODING = "Transfer-Encoding";
const char* CHUNKED = "chunked";
const char* ACCEPT = "Accept";
const char* ACCEPT_ALL = "*/*";
const char* SUCCESS = "Success";
const char* USER_AGENT = "User-Agent";
const char* DEFAULT_AGENT = "Mozilla/5.0 (compatible; Googlebot/2.1; +http://www.google.com/bot.html)";
const char* HOST_C = "Host";
const char* ACCEPT_ENCODING = "Accept-Encoding";
const char* IDENTITY = "identity";
const char* LOCATION = "Location";


// Http methods
const char* methods[] = {"?","GET","HEAD","POST","PUT","DELETE","TRACE","OPTIONS","CONNECT","PATCH",nullptr};

/**
 * @brief A individual key - value header line 
 * 
 */
struct HttpHeaderLine {
    StrExt key;
    StrExt value;
    bool active;
};

/**
 * @brief In a http request and reply we need to process header information. With this API
 * we can define and query the header information. The individual header lines are stored
 * in a vector. This is the common functionality for the HttpRequest and HttpReplyHeader
 * subclasses
 * @author Phil Schatzmann
 * @copyright GPLv3
 * 
 */
class HttpHeader {
    public:
        HttpHeader(){
            LOGI("HttpHeader");
           // set default values
            protocol_str = "HTTP/1.1";
            url_path = "/";
            status_msg = "";
        }
        ~HttpHeader(){
            LOGI("~HttpHeader");
            clear(true);
        }

        /// clears the data - usually we do not delete but we just set the active flag
        HttpHeader& clear(bool activeFlag=true) {
            is_written = false;
            is_chunked = false;
            url_path = "/";
            for (auto it = lines.begin() ; it != lines.end(); ++it){
                if (activeFlag){
                    (*it)->active = false;
                } else {
                    delete *it;
                }
            }
            if (!activeFlag){
                lines.clear();
            }
            return *this;
        }

        HttpHeader& put(const char* key, const char* value){
            if (value!=nullptr && strlen(value)>0){
                HttpHeaderLine *hl = headerLine(key);
                if (hl==nullptr){
                    LOGE("HttpHeader::put - did not add HttpHeaderLine for %s", key);
                    return *this;
                }

                // log entry
                LOGI("HttpHeader::put -> '%s' : '%s'", key, value);

                hl->value = value;
                hl->active = true;

                if (Str(key) == TRANSFER_ENCODING &&  Str(value) == CHUNKED){
                    LOGD("HttpHeader::put -> is_chunked!!!");
                    this->is_chunked = true;
                }
            } else {
                LOGD("HttpHeader::put - value ignored because it is null for %s", key);
            }
            return *this;
        }

        /// adds a new line to the header - e.g. for content size
        HttpHeader& put(const char* key, int value){
            LOGD("HttpHeader::put %s %d", key, value);
            HttpHeaderLine *hl = headerLine(key);

            if (value>1000){
                LOGW("value is > 1000");
            }

            // add value
            hl->value = value;
            hl->active = true;
            LOGI("%s %s", key, hl->value.c_str());
            return *this;
        }

        /// adds a  received new line to the header
        HttpHeader& put(const char* line){
            LOGD("HttpHeader::put -> %s", (const char*) line);
            Str keyStr(line);
            int pos = keyStr.indexOf(":");
            char *key = (char*)line;
            key[pos] = 0;

            const char *value = line+pos+2;
            return put((const char*)key,value);
        }

        // determines a header value with the key
        const char* get(const char* key){
            for (auto it = lines.begin() ; it != lines.end(); ++it){
                HttpHeaderLine *line = *it;
                line->key.trim();
                if (Str(line->key).equalsIgnoreCase(key)){
                    const char* result = line->value.c_str();
                    return line->active ? result : nullptr;
                }
            }
            return nullptr;
        }

        // reads a single header line 
        void readLine(Client &in, char* str, int len){
            reader.readlnInternal(in, (uint8_t*) str, len, false);
            LOGI("HttpHeader::readLine -> %s",str);
        }

        // writes a lingle header line
        void writeHeaderLine(Client &out,HttpHeaderLine *header ){
            if (header==nullptr){
                LOGI("HttpHeader::writeHeaderLine: the value must not be null");
                return;
            }
            LOGD("HttpHeader::writeHeaderLine: %s",header->key.c_str());
            if (!header->active){
                LOGI("HttpHeader::writeHeaderLine - not active");
                return;
            }
            if (header->value.c_str() == nullptr){
                LOGI("HttpHeader::writeHeaderLine - ignored because value is null");
                return;
            }

            char msg[200];
            Str msg_str(msg,200);
            msg_str = header->key.c_str();
            msg_str += ": ";
            msg_str += header->value.c_str();
            msg_str += CRLF;
            out.print(msg);

            // remove crlf from log
            int len = strlen(msg);
            msg[len-2] = 0;
            LOGI(" -> %s ", msg);

            // marke as processed
            header->active = false;
        }
        

        const char* urlPath() {
            return url_path.c_str(); 
        }

        const char* protocol() {
            return protocol_str.c_str(); 
        }

        MethodID method(){
            return method_id;
        }

        int statusCode() {
            return status_code;
        }

        const char* statusMessage() {
            return status_msg.c_str();
        }

        bool isChunked(){
            // the value is automatically set from the reply
            return is_chunked;
        }

        // reads the full header from the request (stream)
        void read(Client &in) {
            LOGI("HttpHeader::read");
            // remove all existing value
            clear();

            char line[MaxHeaderLineLength];   
            if (in.connected()){
                if (in.available()==0) {
                    LOGW("Waiting for data...");
                    while(in.available()==0){
                        delay(500);
                    }
                }
                readLine(in, line, MaxHeaderLineLength);
                parse1stLine(line);
                while (in.available()){
                    readLine(in, line, MaxHeaderLineLength);
                    if (isValidStatus()){
                        Str lineStr(line);
                        lineStr.ltrim();
                        if (lineStr.isEmpty()){
                            break;
                        }
                        put(line); 
                    }               
                }
            }
        }

        // writes the full header to the indicated HttpStreamedMultiOutput stream
        void write(Client &out){
            LOGI("HttpHeader::write");
            write1stLine(out);
            for (auto it = lines.begin() ; it != lines.end(); ++it){
                writeHeaderLine(out, *it);
            }
            // print empty line
            crlf(out);
            out.flush();
            is_written = true;
        }

        // automatically create new lines
        void setAutoCreateLines(bool is_auto_line){
            create_new_lines = is_auto_line;
        }

        bool isValidStatus() {
            return status_code >= 200 && status_code < 300;
        }


    protected:
        int status_code = UNDEFINED;
        bool is_written = false;
        bool is_chunked = false;
        bool create_new_lines = true;
        MethodID method_id;
        // we store the values on the heap. this is acceptable because we just have one instance for the
        // requests and one for the replys: which needs about 2*100 bytes 
        StrExt protocol_str = StrExt(10);
        StrExt url_path = StrExt(70);
        StrExt status_msg = StrExt(20);
        Vector<HttpHeaderLine*> lines;
        HttpLineReader reader;
        const char* CRLF = "\r\n";

        // the headers need to delimited with CR LF
        void crlf(Client &out) {
            out.print(CRLF);
            LOGI(" -> %s ", "<CR LF>");
        }

        // gets or creates a header line by key
        HttpHeaderLine *headerLine(const char* key) {
            if (key!=nullptr){
                for (auto it = lines.begin() ; it != lines.end(); ++it){
                    HttpHeaderLine *pt = (*it);
                    if (pt!=nullptr && pt->key.c_str()!=nullptr){
                        if (strcmp(pt->key.c_str(),key)==0){
                            pt->active = true;
                            return pt;
                        }
                    }
                }
                if (create_new_lines){
                    HttpHeaderLine *newLine = new HttpHeaderLine();
                    LOGD("HttpHeader::headerLine - new line created for %s", key);
                    newLine->active = true;
                    newLine->key = key;
                    lines.push_back(newLine);
                    return newLine;
                }
            } else {
                LOGI("HttpHeader::headerLine %s", "The key must not be null");
            }
            return nullptr;            
        }

        MethodID getMethod(const char* line){
            // set action
            for (int j=0; methods[j]!=nullptr;j++){
                const char *action = methods[j];
                int len = strlen(action);
                if (strncmp(action,line,len)==0){
                    return (MethodID) j;
                }
            }
            return (MethodID)0;
        }


        virtual void write1stLine(Client &out) = 0;
        virtual void parse1stLine(const char *line) = 0;

     
};

/**
 * @brief Reading and writing of Http Requests
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class HttpRequestHeader : public HttpHeader {
    public:
        // Defines the action id, url path and http version for an request
        HttpHeader& setValues(MethodID id, const char* urlPath, const char* protocol=nullptr){
            this->method_id = id;
            this->url_path = urlPath;
            
            LOGI("HttpRequestHeader::setValues - path: %s",this->url_path.c_str());
            if (protocol!=nullptr){
                this->protocol_str = protocol;
            }
            return *this;
        }

        // action path protocol
        void write1stLine(Client &out){
            LOGI("HttpRequestHeader::write1stLine");
            char msg[200];
            Str msg_str(msg,200);

            const char* method_str = methods[this->method_id];
            msg_str = method_str;
            msg_str += " ";
            msg_str += this->url_path.c_str();
            msg_str += " ";
            msg_str += this->protocol_str.c_str();
            msg_str += CRLF;
            out.print(msg);

            int len = strlen(msg);
            msg[len-2]=0;
            LOGI("-> %s", msg);
        }

        // parses the requestline 
        // Request-Line = Method SP Request-URI SP HTTP-Version CRLF
        void parse1stLine(const char *line){
            LOGI("HttpRequestHeader::parse1stLine %s", line);
            Str line_str(line);
            int space1 = line_str.indexOf(" ");
            int space2 = line_str.indexOf(" ", space1+1);

            this->method_id = getMethod(line);
            this->protocol_str.substring(line_str, space2+1, line_str.length());
            this->url_path.substring(line_str,space1+1,space2);
            this->url_path.trim();
  
            LOGI("->method %s", methods[this->method_id]);
            LOGI("->protocol %s", protocol_str.c_str());
            LOGI("->url_path %s", url_path.c_str());
        }

};

/**
 * @brief Reading and Writing of Http Replys
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class HttpReplyHeader : public HttpHeader  {
    public:
        // defines the values for the rely
        void setValues(int statusCode, const char* msg="", const char* protocol=nullptr){
            LOGI("HttpReplyHeader::setValues");
            status_msg = msg;
            status_code = statusCode;
            if (protocol!=nullptr){
                this->protocol_str = protocol;
            }
        }

        // reads the final chunked reply headers 
        void readExt(Client &in) {
            LOGI("HttpReplyHeader::readExt");
            char line[MaxHeaderLineLength];   
            readLine(in, line, MaxHeaderLineLength);
            while(strlen(line)!=0){
                put(line);                
                readLine(in, line, MaxHeaderLineLength);
            }
        }

        // HTTP-Version SP Status-Code SP Reason-Phrase CRLF
        void write1stLine(Client &out){
            LOGI("HttpReplyHeader::write1stLine");
            char msg[200];
            Str msg_str(msg,200);
            msg_str = this->protocol_str.c_str();
            msg_str += " ";
            msg_str += this->status_code;
            msg_str += " ";
            msg_str += this->status_msg.c_str();
            LOGI("-> %s", msg);
            out.print(msg);
            crlf(out);
        }


        // HTTP-Version SP Status-Code SP Reason-Phrase CRLF
        // we just update the pointers to point to the correct position in the
        // http_status_line
        void parse1stLine(const char *line){
            LOGI("HttpReplyHeader::parse1stLine: %s",line);
            Str line_str(line);
            int space1 = line_str.indexOf(' ',0);
            int space2 = line_str.indexOf(' ',space1+1);

            // save http version 
            protocol_str.substring(line_str,0,space1);

            // find response status code after the first space
            char status_c[6];
            Str status(status_c,6);
            status.substring(line_str, space1+1, space2);
            status_code = atoi(status_c);

            // get reason-phrase after last SP
            status_msg.substring(line_str, space2+1, line_str.length());

        }


};

}
