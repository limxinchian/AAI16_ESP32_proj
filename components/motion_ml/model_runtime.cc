#include "model_runtime.h"
#include <cstring>
#include <cstdio>
extern "C" void motion_model_bundled_a_identity(motion_model_identity_t *identity) {
    *identity=motion::bundled_identity(*motion::bundled_model(motion::ModelFamily::A));
}
namespace motion {
namespace {
struct Guard { Guard(){motion_runtime_enter();} ~Guard(){motion_runtime_leave();} };
bool fail(char *detail,const char *reason) { std::snprintf(detail,64,"%s",reason); return false; }
}
motion_model_identity_t bundled_identity(const ModelDescriptor &model) {
    motion_model_identity_t id{};
    id.family=model.family==ModelFamily::A?1:2; id.source=1;
    std::snprintf(id.version,sizeof(id.version),"%s",model.view.version);
    std::memcpy(id.sha256,model.view.sha256,32);
    return id;
}
bool same_identity(const motion_model_identity_t &a,const motion_model_identity_t &b) {
    return a.family==b.family && a.source==b.source && (a.source!=2||a.bank==b.bank) &&
        !std::strcmp(a.version,b.version) && !std::memcmp(a.sha256,b.sha256,32);
}
void ModelRuntime::clear(unsigned index) {
    auto &s=slots_[index];
    s.engine.reset(); // Destroy tensors/interpreter BEFORE releasing model bytes.
    if(s.package) release_(s.package);
    s.package=nullptr; s.healthy=false; s.model=nullptr;
}
ModelRuntime::~ModelRuntime() {
    Guard lock;
    for(unsigned i=0;i<2;++i) { clear(i); if(slots_[i].arena) release_(slots_[i].arena); }
}
void ModelRuntime::refresh() {
    state_.active=slots_[active_].identity;
    state_.ready=slots_[active_].healthy;
    state_.has_previous=slots_[1-active_].healthy;
    state_.previous=state_.has_previous?slots_[1-active_].identity:motion_model_identity_t{};
}
bool ModelRuntime::prepare(unsigned index,const motion_model_command_t &command,char detail[64]) {
    auto &s=slots_[index];
    clear(index);
    s.identity=command.target;
    if(!s.arena) s.arena=static_cast<uint8_t*>(allocate_(MOTION_CANDIDATE_ARENA));
    if(!s.arena) return fail(detail,"arena_allocation_active_retained");
    if(command.target.source==1) {
        // Only A is a remotely selectable bundled fallback; B must be stored.
        s.model=bundled_model(ModelFamily::A);
        if(!same_identity(command.target,bundled_identity(*s.model))) return fail(detail,"bundled_identity_mismatch");
    } else if(command.target.source==2 && command.package && command.package_size<=73728 && command.package_size>=256) {
        s.package=static_cast<uint8_t*>(allocate_(command.package_size));
        if(!s.package) return fail(detail,"package_allocation_active_retained");
        std::memcpy(s.package,command.package,command.package_size);
        if(!s.provider.parse(s.package,command.package_size,detail)) return false;
        s.model=&s.provider.descriptor;
        auto id=bundled_identity(*s.model); id.source=2; id.bank=command.target.bank;
        if(!same_identity(command.target,id)) return fail(detail,"stored_identity_mismatch");
    } else return fail(detail,"missing_stored_package");
    if(!s.engine.init(s.model->view,s.arena,MOTION_CANDIDATE_ARENA)) return fail(detail,s.engine.error());
    size_t passed=0;
    if(!model_self_test(s.engine,*s.model,passed)) return fail(detail,"activation_golden_mismatch");
    // Additional trial after the four exact end-to-end goldens. Both classes
    // occur in the package goldens; no live sensor availability is required.
    int8_t output[2];
    if(!s.engine.invoke(s.model->golden[0].input,output) ||
        std::memcmp(output,s.model->golden[0].output,2)) return fail(detail,"trial_inference_failed");
    s.healthy=true;
    return true;
}
bool ModelRuntime::start() {
    Guard lock;
    motion_model_command_t command{};
    command.target=bundled_identity(*bundled_model(ModelFamily::A));
    prepare(0,command,state_.detail);
    refresh();
    if(state_.ready) std::strcpy(state_.detail,"boot_bundled_a_volatile_activation");
    return state_.ready;
}
motion_model_reply_t ModelRuntime::change(const motion_model_command_t &command) {
    Guard lock;
    motion_model_reply_t reply{};
    if(command.expected_epoch!=state_.epoch || state_.epoch==UINT32_MAX) {
        reply.result=MOTION_CHANGE_STALE; std::strcpy(reply.detail,"model_epoch_changed_get_status");
    } else if(!command.rollback && state_.ready && same_identity(command.target,state_.active)) {
        std::strcpy(reply.detail,"already_active");
    } else {
        unsigned next=1-active_;
        bool reuse=slots_[next].healthy && (command.rollback || same_identity(command.target,slots_[next].identity));
        bool ok=false;
        if(reuse) {
            // A rollback rechecks the retained interpreter without rebuilding.
            size_t passed=0;
            ok=model_self_test(slots_[next].engine,*slots_[next].model,passed);
            if(!ok) { clear(next); std::strcpy(reply.detail,"previous_golden_failed_active_retained"); }
        } else {
            auto selected=command;
            if(command.rollback) {
                selected.target=bundled_identity(*bundled_model(ModelFamily::A));
                selected.package=nullptr; selected.package_size=0;
            }
            ok=prepare(next,selected,reply.detail);
            if(!ok) clear(next);
        }
        if(ok) {
            active_=next; ++state_.epoch; state_.rolled_back=command.rollback;
            std::strcpy(reply.detail,command.rollback?"rollback_ok_volatile":"activated_golden_4_of_4_trial_ok_volatile");
        } else {
            reply.result=MOTION_CHANGE_FAILED;
            state_.rolled_back=state_.ready;
        }
        refresh();
        std::snprintf(state_.detail,64,"%s",reply.detail);
    }
    reply.state=state_;
    return reply;
}
bool ModelRuntime::recover_invoke() {
    Guard lock;
    slots_[active_].healthy=false;
    auto next=1-active_;
    size_t passed=0;
    bool ok=slots_[next].healthy && model_self_test(slots_[next].engine,*slots_[next].model,passed);
    if(!ok) {
        motion_model_command_t command{};
        command.target=bundled_identity(*bundled_model(ModelFamily::A));
        ok=prepare(next,command,state_.detail);
    }
    if(ok && state_.epoch!=UINT32_MAX) {
        const auto failed=active_;
        active_=next; ++state_.epoch; clear(failed);
        state_.rolled_back=true;
        std::strcpy(state_.detail,"invoke_failed_automatic_rollback");
    } else {
        clear(next);
        std::strcpy(state_.detail,"invoke_failed_fallback_unavailable");
    }
    refresh(); return state_.ready;
}
}
