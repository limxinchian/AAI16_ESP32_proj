#include "motion_candidate.h"
#include "motion_engine.h"
#include "candidate_provider.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include <cmath>
#include <cstring>
#include <cstdio>

namespace {
uint32_t u32(const uint8_t *p) { return uint32_t(p[0]) | uint32_t(p[1])<<8 | uint32_t(p[2])<<16 | uint32_t(p[3])<<24; }
float f32(const uint8_t *p) { uint32_t bits=u32(p); float f; std::memcpy(&f,&bits,4); return f; }
bool fail(char *detail, const char *why) { std::snprintf(detail,64,"%s",why); return false; }
bool zeros(const uint8_t *p, size_t n) { for(size_t i=0;i<n;++i) if(p[i]) return false; return true; }
bool version_valid(const char *s) {
    size_t n=0;
    for(;n<40 && s[n];++n) if(!((s[n]>='a'&&s[n]<='z')||(s[n]>='A'&&s[n]<='Z')||
        (s[n]>='0'&&s[n]<='9')||s[n]=='-'||s[n]=='_'||s[n]=='.')) return false;
    return n && n<40 && zeros(reinterpret_cast<const uint8_t*>(s+n),40-n);
}
bool shape(const tflite::Tensor *t, tflite::TensorType type, int a, int b=0) {
    if(!t || t->type()!=type || t->is_variable() || t->sparsity() || !t->shape() ||
       t->shape()->size()!=unsigned(b?2:1) || t->shape()->Get(0)!=a || (b && t->shape()->Get(1)!=b)) return false;
    if(t->shape_signature() && t->shape_signature()->size()) {
        if(t->shape_signature()->size()!=t->shape()->size()) return false;
        for(unsigned i=0;i<t->shape()->size();++i) if(t->shape_signature()->Get(i)!=t->shape()->Get(i)) return false;
    }
    return true;
}
bool quant(const tflite::Tensor *t, unsigned channels, bool symmetric) {
    const auto *q=t->quantization();
    if(!q || !q->scale() || q->scale()->size()!=channels || !q->zero_point() ||
       q->zero_point()->size()!=channels || q->quantized_dimension()!=0 ||
       q->details_type()!=tflite::QuantizationDetails_NONE) return false;
    for(float s:*q->scale()) if(!std::isfinite(s)||s<=0||s>1000||s<1e-12f) return false;
    for(int64_t z:*q->zero_point()) if(symmetric ? z!=0 : (z< -128||z>127)) return false;
    return true;
}
// Strict semantic admission BEFORE passing an externally supplied FlatBuffer to
// the interpreter. Structural FlatBuffer validity alone doesn't bound shapes,
// tensor indices, constant buffers or quantization used by kernel allocation.
bool graph_contract(const uint8_t *bytes, size_t size, unsigned hidden) {
    flatbuffers::Verifier verifier(bytes,size);
    if(!tflite::VerifyModelBuffer(verifier)) return false;
    const auto *m=tflite::GetModel(bytes);
    if(m->version()!=TFLITE_SCHEMA_VERSION || !m->subgraphs() || m->subgraphs()->size()!=1 ||
       !m->buffers() || !m->operator_codes() || m->operator_codes()->size()!=1) return false;
    const auto *code=m->operator_codes()->Get(0);
    if(code->builtin_code()!=tflite::BuiltinOperator_FULLY_CONNECTED || code->version()!=4 || code->custom_code()) return false;
    const auto *g=m->subgraphs()->Get(0);
    if(!g->inputs()||g->inputs()->size()!=1||!g->outputs()||g->outputs()->size()!=1||
       !g->operators()||g->operators()->size()!=2||!g->tensors()||g->tensors()->size()!=7) return false;
    unsigned used=0;
    auto tensor=[&](int index)->const tflite::Tensor* {
        if(index<0||index>=7) return nullptr;
        used |= 1U<<index;
        const auto *t=g->tensors()->Get(index);
        return t->buffer()<m->buffers()->size() ? t : nullptr;
    };
    auto buffer_size=[&](const tflite::Tensor *t)->size_t {
        const auto *b=m->buffers()->Get(t->buffer());
        if(b->offset() || b->size()) return SIZE_MAX; // external buffer storage not supported
        return b->data()?b->data()->size():0;
    };
    int previous=g->inputs()->Get(0);
    for(unsigned layer=0;layer<2;++layer) {
        const auto *op=g->operators()->Get(layer);
        const auto *opt=op->builtin_options_as_FullyConnectedOptions();
        if(op->opcode_index()!=0||!op->inputs()||op->inputs()->size()!=3||!op->outputs()||op->outputs()->size()!=1||
           op->inputs()->Get(0)!=previous||!opt||opt->keep_num_dims()||opt->asymmetric_quantize_inputs()||
           opt->weights_format()!=tflite::FullyConnectedOptionsWeightsFormat_DEFAULT||
           opt->fused_activation_function()!=(layer?tflite::ActivationFunctionType_NONE:tflite::ActivationFunctionType_RELU)||
           (op->intermediates()&&op->intermediates()->size()) || (op->custom_options()&&op->custom_options()->size())) return false;
        int input_width=layer?hidden:60, output_width=layer?2:hidden;
        auto *in=tensor(op->inputs()->Get(0)), *w=tensor(op->inputs()->Get(1)), *b=tensor(op->inputs()->Get(2)), *out=tensor(op->outputs()->Get(0));
        if(!shape(in,tflite::TensorType_INT8,1,input_width)||!shape(w,tflite::TensorType_INT8,output_width,input_width)||
           !shape(b,tflite::TensorType_INT32,output_width)||!shape(out,tflite::TensorType_INT8,1,output_width)||
           !quant(in,1,false)||!quant(out,1,false)||!quant(w,output_width,true)||!quant(b,output_width,true)||
           buffer_size(in)!=0||buffer_size(out)!=0||buffer_size(w)!=unsigned(input_width*output_width)||buffer_size(b)!=unsigned(output_width*4)) return false;
        for(int i=0;i<output_width;++i) {
            float expected=in->quantization()->scale()->Get(0)*w->quantization()->scale()->Get(i);
            if(std::fabs(b->quantization()->scale()->Get(i)-expected)>expected*1e-4f) return false;
            double multiplier=double(expected)/out->quantization()->scale()->Get(0);
            // No positive left shift of the INT32 accumulator is admitted.
            if(!std::isfinite(multiplier)||multiplier<=0||multiplier>=1) return false;
            auto raw=m->buffers()->Get(b->buffer())->data()->data();
            int32_t bias=static_cast<int32_t>(u32(raw+4*i));
            if(bias < -2000000000 || bias > 2000000000) return false;
        }
        previous=op->outputs()->Get(0);
    }
    return previous==g->outputs()->Get(0) && used==127;
}
struct RuntimeGuard { RuntimeGuard(){motion_runtime_enter();} ~RuntimeGuard(){motion_runtime_leave();} };
}

bool motion::CandidateProvider::parse(const uint8_t *p,size_t size,char detail[64]) {
    auto *info=&this->info;
    *info={};
    if(!p||size<256||size>73728||reinterpret_cast<uintptr_t>(p)%16||std::memcmp(p,"AAI16MP1",8)||
       u32(p+8)!=1||u32(p+12)!=256) return fail(detail,"package_header");
    uint32_t family=u32(p+16),profile=u32(p+20),length=u32(p+24),count=u32(p+28),offset=u32(p+32),gold=u32(p+36);
    if((family!=1&&family!=2)||profile!=1||!length||length>65536||count!=4||offset!=256||
       gold!=((256+length+15)&~15U)||size!=gold+count*320||u32(p+40)!=size||!zeros(p+60,4)||!zeros(p+168,88))
        return fail(detail,"package_contract");
    static const uint8_t contract_sha[]={0xe0,0x36,0xfd,0xc6,0xe8,0x32,0x33,0x34,0x05,0x0a,0x7a,0x29,0xda,0xad,0x62,0x68,0x7d,0xe9,0xa8,0x91,0x19,0x3f,0x26,0xfd,0xe1,0x95,0x3f,0xbf,0xbc,0xcc,0xc2,0x94};
    if(std::memcmp(p+136,contract_sha,32)||!zeros(p+offset+length,gold-offset-length)) return fail(detail,"preprocessing_contract");
    info->family=family; info->profile=profile; info->model_size=length;
    std::memcpy(info->version,p+64,40); std::memcpy(info->model_sha256,p+104,32);
    info->input_scale=f32(p+44); info->input_zero_point=static_cast<int32_t>(u32(p+48));
    info->output_scale=f32(p+52); info->output_zero_point=static_cast<int32_t>(u32(p+56));
    if(!version_valid(info->version)||!std::isfinite(info->input_scale)||info->input_scale<=0||
       !std::isfinite(info->output_scale)||info->output_scale<=0||info->input_zero_point< -128||info->input_zero_point>127||
       info->output_zero_point< -128||info->output_zero_point>127) return fail(detail,"package_metadata");
    if(!graph_contract(p+offset,length,family==1?16:8)) return fail(detail,"int8_graph_contract");
    for(unsigned i=0;i<4;++i) {
        const uint8_t *g=p+gold+320*i;
        if(!zeros(g+302,2)||!zeros(g+308,12)||u32(g+304)>1) return fail(detail,"golden_format");
        for(unsigned j=0;j<60;++j) {
            golden[i].raw[j/3][j%3]=f32(g+4*j);
            if(!std::isfinite(golden[i].raw[j/3][j%3])) return fail(detail,"golden_nonfinite");
        }
        std::memcpy(golden[i].input,g+240,60); std::memcpy(golden[i].output,g+300,2); golden[i].prediction=u32(g+304);
    }
    descriptor={family==1?motion::ModelFamily::A:motion::ModelFamily::B,profile,
        {p+offset,length,info->model_sha256,info->version,info->input_scale,info->output_scale,info->input_zero_point,info->output_zero_point},golden,4};
    return true;
}
extern "C" bool motion_candidate_validate(const uint8_t *p,size_t size,uint8_t *arena,size_t arena_size,
                                           motion_candidate_info_t *info,char detail[64]) {
    motion::CandidateProvider provider;
    if(!provider.parse(p,size,detail)) { *info={}; return false; }
    *info=provider.info;
    // No flash operations while holding this lock; the candidate has its own arena.
    RuntimeGuard lock;
    motion::Engine engine;
    if(!engine.init(provider.descriptor.view,arena,arena_size)) return fail(detail,engine.error());
    size_t passed=0;
    if(!motion::model_self_test(engine,provider.descriptor,passed)) return fail(detail,"candidate_golden_mismatch");
    std::snprintf(detail,64,"validated_4_of_4"); return true;
}
