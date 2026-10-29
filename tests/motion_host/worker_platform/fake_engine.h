#pragma once
// Fault injection only. Real TFLM/model execution is tested in test_engine.cc.
#include <cstdint>
#include <cstddef>
#include "model_provider.h"
extern bool init_fail, golden_fail, invoke_fail, config_during_invoke;
extern unsigned invoke_calls;
extern bool trial_fail;
void change_config_in_invoke();
void on_engine_init(const motion::ModelView &);
namespace motion {
class Engine {
    ModelView view_{};
    bool trial_=false, fresh_=false;
public:
    void reset() {}
    void trial() { if(fresh_) { trial_=true; fresh_=false; } }
    bool init(const ModelView &view,uint8_t *,size_t) { view_ = view; fresh_=true; on_engine_init(view); return !init_fail; }
    bool invoke(const int8_t *,int8_t out[2]) {
        if(trial_) { trial_=false; out[0]=1; out[1]=0; return !trial_fail; }
        ++invoke_calls;
        if(config_during_invoke) { config_during_invoke=false; change_config_in_invoke(); }
        out[0]=1; out[1]=0; bool ok=!invoke_fail; invoke_fail=false; return ok;
    }
    const char *error() const { return "injected_failure"; }
    size_t arena_used() const { return 2048; }
    float input_scale() const { return 0.0031422548927366734f; }
    int input_zero_point() const { return 12; }
    float output_scale() const { return view_.output_scale; }
    int output_zero_point() const { return view_.output_zero_point; }
};
inline const ModelDescriptor *bundled_model(ModelFamily family) {
    static const uint8_t sha[32]={1};
    static const GoldenVector golden[4]={{{},{},{1,0},0},{{},{},{1,0},0},{{},{},{1,0},0},{{},{},{1,0},0}};
    static const ModelDescriptor a = {ModelFamily::A, 1,
        {nullptr, 3104, sha, "aai16-motion-model-a-0.1.0", 0.0031422548927366734f, 0.046926043927669525f, 12, -12}, golden, 4};
    static const ModelDescriptor b = {ModelFamily::B, 1,
        {nullptr, 2384, sha, "aai16-motion-model-b-0.1.0", 0.0031422548927366734f, 0.0462011881172657f, 12, -29}, golden, 4};
    return family==ModelFamily::A?&a:&b;
}
inline const ModelDescriptor &startup_model() { return *bundled_model(ModelFamily::A); }
inline const char *model_family_name(ModelFamily family) { return family == ModelFamily::A ? "A" : "B"; }
inline bool model_self_test(Engine &engine, const ModelDescriptor &, size_t &passed) { engine.trial(); passed = golden_fail ? 0 : 4; return !golden_fail; }
}
