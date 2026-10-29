#pragma once
#include "aai16_protocol.h"
#include "include/model_upload.h"
#include <cstring>

namespace upload {
struct Frame { uint16_t length; bool control; uint8_t bytes[AAI16_FRAME_MAX]; };
inline bool frame_accepts(const Frame &frame,const aai16_Envelope &request) {
    return frame.control ? (request.which_body==aai16_Envelope_model_control_tag || request.which_body==aai16_Envelope_model_selection_tag) :
        request.which_body==aai16_Envelope_model_upload_chunk_tag;
}
// One MQTT connection emits ordered fragments. This object never allocates,
// decodes protobuf, touches storage or changes a transfer session.
class Ingress {
    Frame frame_={};
    int total_=0,id_=0;
    bool active_=false;
    static bool topic_is(const char *p,size_t n,const char *name) {
        return p&&n==std::strlen(name)&&!std::memcmp(p,name,n);
    }
public:
    void reset() { active_=false; }
    bool feed(const char *topic,size_t topic_length,const void *data,int length,int offset,int total,int id,bool retained,Frame &completed) {
        if(offset==0) {
            active_=false;
            bool control=topic_is(topic,topic_length,MODEL_CONTROL_TOPIC);
            if(!control&&!topic_is(topic,topic_length,MODEL_CHUNK_TOPIC)) return false;
            if(!retained&&total>0&&total<=AAI16_FRAME_MAX) {
                active_=true; frame_.control=control; frame_.length=0; total_=total; id_=id;
            }
        }
        if(!active_||retained||!data||length<=0||offset!=frame_.length||total!=total_||id!=id_||length>total-offset||
           (topic&&topic_length&&!topic_is(topic,topic_length,frame_.control?MODEL_CONTROL_TOPIC:MODEL_CHUNK_TOPIC))) {
            active_=false; return false;
        }
        std::memcpy(frame_.bytes+offset,data,length); frame_.length+=length;
        if(frame_.length!=total_) return false;
        completed=frame_; active_=false; return true;
    }
};
}
