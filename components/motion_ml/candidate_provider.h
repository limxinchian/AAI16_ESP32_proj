#pragma once
#include "model_provider.h"
#include "motion_candidate.h"
namespace motion {
// Noncopyable because descriptor references this object's info/golden storage.
struct CandidateProvider {
    motion_candidate_info_t info{};
    GoldenVector golden[4]{};
    ModelDescriptor descriptor{};
    CandidateProvider() = default;
    CandidateProvider(const CandidateProvider &) = delete;
    CandidateProvider &operator=(const CandidateProvider &) = delete;
    bool parse(const uint8_t *package, size_t size, char detail[64]);
};
}
