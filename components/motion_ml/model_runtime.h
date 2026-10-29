#pragma once
#include "motion_engine.h"
#include "candidate_provider.h"
#include "motion_model_control.h"
namespace motion {
// Only the ML worker calls this object, between live Invoke calls.
class ModelRuntime {
public:
    using Allocate = void *(*)(size_t);
    using Release = void (*)(void *);
    ModelRuntime(Allocate allocate, Release release): allocate_(allocate), release_(release) {}
    ~ModelRuntime();
    bool start();
    motion_model_reply_t change(const motion_model_command_t &command);
    bool recover_invoke();
    const motion_model_state_t &state() const { return state_; }
    Engine &engine() { return slots_[active_].engine; }
    const ModelDescriptor &model() const { return slots_[active_].model?*slots_[active_].model:*bundled_model(ModelFamily::A); }
private:
    struct Slot {
        Engine engine;
        CandidateProvider provider;
        const ModelDescriptor *model=nullptr;
        uint8_t *arena=nullptr, *package=nullptr;
        motion_model_identity_t identity{};
        bool healthy=false;
    } slots_[2];
    unsigned active_=0;
    Allocate allocate_;
    Release release_;
    motion_model_state_t state_{};
    void clear(unsigned slot);
    bool prepare(unsigned slot, const motion_model_command_t &, char detail[64]);
    void refresh();
};
bool same_identity(const motion_model_identity_t &, const motion_model_identity_t &);
motion_model_identity_t bundled_identity(const ModelDescriptor &);
}
