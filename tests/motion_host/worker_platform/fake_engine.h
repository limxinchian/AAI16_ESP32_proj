#pragma once
// Fault injection only. Real TFLM/model execution is tested in test_engine.cc.
#include <cstdint>
#include <cstddef>
extern bool init_fail, golden_fail, invoke_fail, config_during_invoke;
extern unsigned invoke_calls;
void change_config_in_invoke();
namespace motion {
struct ModelView { const char *version; };
class Engine {
public:
    bool init(const ModelView &,uint8_t *,size_t) { return !init_fail; }
    bool invoke(const int8_t *,int8_t out[2]) {
        ++invoke_calls;
        if(config_during_invoke) { config_during_invoke=false; change_config_in_invoke(); }
        out[0]=1; out[1]=0; return !invoke_fail;
    }
    const char *error() const { return "injected_failure"; }
    size_t arena_used() const { return 2048; }
    float input_scale() const { return 0.0031422548927366734f; }
    int input_zero_point() const { return 12; }
    float output_scale() const { return 0.046926043927669525f; }
    int output_zero_point() const { return -12; }
};
inline ModelView bundled_model() { return {"aai16-motion-model-a-0.1.0"}; }
inline bool bundled_self_test(Engine &) { return !golden_fail; }
}
