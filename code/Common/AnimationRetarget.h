/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2026, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

/** @file AnimationRetarget.h
 *  @brief Declares a utility to retarget skeletal animations between
 *         different rigs (bone mapping, rest-pose and scale normalization).
 */
#ifndef AI_ANIMATION_RETARGET_H_INC
#define AI_ANIMATION_RETARGET_H_INC

#include <assimp/anim.h>
#include <assimp/defs.h>
#include <assimp/quaternion.h>
#include <assimp/vector3.h>

#include <string>
#include <unordered_map>
#include <vector>

struct aiNode;

namespace Assimp {

// ---------------------------------------------------------------------------
/** Options controlling how an animation is retargeted onto another rig. */
struct ASSIMP_API RetargetOptions {
    /** Explicit bone mapping: source bone name -> target bone name.
     *  If empty, bones are matched automatically by identical node names. */
    std::unordered_map<std::string, std::string> mBoneMapping;

    /** Name of the source bone that carries root motion (usually the hips).
     *  If empty, the mapped bone closest to the source skeleton root is used. */
    std::string mRootBoneName;

    /** Retarget the translation of the root bone (root motion). If false,
     *  the root bone keeps its rest position (in-place animation). */
    bool mEnableRootMotion = true;

    /** If the source animation loops (first key == last key), force the
     *  retargeted animation to close the loop exactly. */
    bool mEnforceLoopClosure = true;

    /** Retarget scaling keys relative to the rest scale. */
    bool mRetargetScaleKeys = true;

    /** Ticks per second of the output animation. 0 keeps the source rate.
     *  If it differs from the source rate, channels are resampled uniformly. */
    double mTargetTicksPerSecond = 0.0;

    /** Minimum fraction of mapped source channels [0..1]. If the coverage of
     *  a retargeted animation falls below this value, the source animation
     *  is returned unchanged (global fallback) instead of a distorted one. */
    double mMinCoverage = 0.0;

    /** Upper bound for the position scale normalization factor. Protects
     *  against degenerate rest poses with near-zero bone lengths. */
    ai_real mMaxPositionScale = (ai_real)100.0;

    /** Epsilon used for zero tests and loop detection. */
    ai_real mEpsilon = (ai_real)1e-6;
};

// ---------------------------------------------------------------------------
/** Statistics collected while building the retarget plan and while
 *  retargeting animations. */
struct ASSIMP_API RetargetMetrics {
    /** Number of node channels in the last retargeted source animation. */
    unsigned int mNumSourceChannels = 0;

    /** Number of source channels that could be mapped to the target rig. */
    unsigned int mNumMappedChannels = 0;

    /** Number of source channels without a counterpart on the target rig. */
    unsigned int mNumUnmappedChannels = 0;

    /** Number of channels that fell back to (renamed) source data because
     *  their retarget produced invalid values. */
    unsigned int mNumFallbackChannels = 0;

    /** Number of animations returned as unmodified copies of the source. */
    unsigned int mNumGlobalFallbacks = 0;

    /** Number of channels whose loop closure was enforced. */
    unsigned int mNumLoopClosures = 0;

    /** Fraction of source channels mapped to the target rig (0..1). */
    double mCoverage = 0.0;

    /** Mean relative deviation of per-bone size ratios from the global
     *  size ratio. Non-zero values indicate non-uniform proportion
     *  differences between the rigs. */
    double mMeanScaleError = 0.0;

    /** Maximum relative deviation of a per-bone size ratio. */
    double mMaxScaleError = 0.0;

    /** True if the last Retarget() call returned the source animation. */
    bool mLastRetargetUsedGlobalFallback = false;
};

// ---------------------------------------------------------------------------
/** Retargets skeletal animations from one rig onto another.
 *
 *  The retarget plan is built once from the rest poses of both skeletons
 *  (Build) and cached; its memory footprint is O(number of mapped bones).
 *  Retarget() then converts any number of animations in a single pass over
 *  the key frames - it never rebuilds the plan per frame and never touches
 *  the scene graphs the rest poses were read from.
 *
 *  Rotations are retargeted in world space relative to the rest pose:
 *  the target bone's world rotation differs from its rest orientation by
 *  the same delta as the source bone's. Parent orientations are taken from
 *  the rest pose (the standard approximation for humanoid retargeting).
 *  Quaternion keys are kept in a single hemisphere to avoid sign flips.
 *
 *  On failure (insufficient bone coverage, degenerate rest data) the
 *  source animation is returned unchanged instead of a distorted result.
 */
class ASSIMP_API AnimationRetarget {
public:
    AnimationRetarget();
    ~AnimationRetarget();

    AnimationRetarget(const AnimationRetarget &) = delete;
    AnimationRetarget &operator=(const AnimationRetarget &) = delete;

    /// @brief Builds the retarget plan from the rest poses of two skeletons.
    /// @param sourceRoot  Root node of the skeleton the animation comes from.
    /// @param targetRoot  Root node of the skeleton to retarget onto.
    /// @param options     Mapping and normalization options.
    /// @return true if a usable plan could be built. If false is returned,
    ///         Retarget() will fall back to returning unmodified copies.
    bool Build(const aiNode *sourceRoot, const aiNode *targetRoot, const RetargetOptions &options);

    /// @brief Retargets one animation onto the target rig.
    /// The input animation and the scene graphs are left unmodified.
    /// @param source  The animation to retarget (from the source rig).
    /// @return A newly allocated animation (caller owns it). On global
    ///         fallback this is an unmodified deep copy of the source.
    aiAnimation *Retarget(const aiAnimation *source);

    /// @brief Returns true if a valid plan was built.
    bool IsReady() const;

    /// @brief Returns the options the plan was built with.
    const RetargetOptions &GetOptions() const;

    /// @brief Returns the collected metrics.
    const RetargetMetrics &GetMetrics() const;

    /// @brief Returns the global size ratio target/source (bone lengths).
    double GetSizeRatio() const;

private:
    struct Impl;
    Impl *mImpl;
};

} // namespace Assimp

#endif // AI_ANIMATION_RETARGET_H_INC
