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

/** @file AnimationRetarget.cpp
 *  @brief Implementation of the skeletal animation retargeting utility.
 */

#include "AnimationRetarget.h"

#include <assimp/ai_assert.h>
#include <assimp/matrix4x4.h>
#include <assimp/scene.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace Assimp {

namespace {

// ---------------------------------------------------------------------------
// Small vector helpers (component-wise operations not provided by aiVector3D).
inline aiVector3D MulComponents(const aiVector3D &a, const aiVector3D &b) {
    return aiVector3D(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline aiVector3D DivComponentsGuarded(const aiVector3D &a, const aiVector3D &b, ai_real eps) {
    return aiVector3D(std::abs(b.x) > eps ? a.x / b.x : (ai_real)0.0,
            std::abs(b.y) > eps ? a.y / b.y : (ai_real)0.0,
            std::abs(b.z) > eps ? a.z / b.z : (ai_real)0.0);
}

inline bool IsFinite(const aiVector3D &v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool IsFinite(const aiQuaternion &q) {
    return std::isfinite(q.w) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z);
}

inline ai_real Dot(const aiQuaternion &a, const aiQuaternion &b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

inline aiQuaternion Inverted(const aiQuaternion &q) {
    aiQuaternion inv(q.w, -q.x, -q.y, -q.z);
    const ai_real lenSq = Dot(q, q);
    if (lenSq > (ai_real)1e-12) {
        const ai_real invLen = (ai_real)1.0 / std::sqrt(lenSq);
        inv.w *= invLen;
        inv.x *= invLen;
        inv.y *= invLen;
        inv.z *= invLen;
    }
    return inv;
}

// ---------------------------------------------------------------------------
// Rest-pose description of one skeleton node.
struct NodeRest {
    std::string name;
    int parentIndex = -1;
    aiVector3D localPos;
    aiQuaternion localRot;
    aiVector3D localScale;
    aiVector3D globalPos;
    aiQuaternion globalRot;
    aiVector3D globalScale;
    unsigned int depth = 0;
};

// Collects the rest pose of a whole skeleton. The transforms are read from
// the node graph, the graph itself is never modified.
void CollectRestPose(const aiNode *root, std::vector<NodeRest> &out,
        std::unordered_map<std::string, size_t> &indexByName) {
    out.clear();
    indexByName.clear();
    if (root == nullptr) {
        return;
    }

    struct StackEntry {
        const aiNode *node;
        int parentIndex;
        aiMatrix4x4 parentGlobal;
        unsigned int depth;
    };

    std::vector<StackEntry> stack;
    stack.push_back({ root, -1, aiMatrix4x4(), 0 });

    while (!stack.empty()) {
        const StackEntry entry = stack.back();
        stack.pop_back();

        NodeRest rest;
        rest.name = entry.node->mName.C_Str();
        rest.parentIndex = entry.parentIndex;
        rest.depth = entry.depth;

        entry.node->mTransformation.Decompose(rest.localScale, rest.localRot, rest.localPos);
        rest.localRot.Normalize();

        const aiMatrix4x4 global = entry.parentGlobal * entry.node->mTransformation;
        global.Decompose(rest.globalScale, rest.globalRot, rest.globalPos);
        rest.globalRot.Normalize();

        const size_t index = out.size();
        // The first occurrence of a name wins, matching aiNode::FindNode.
        if (indexByName.find(rest.name) == indexByName.end()) {
            indexByName[rest.name] = index;
        }
        out.push_back(rest);

        for (unsigned int i = 0; i < entry.node->mNumChildren; ++i) {
            stack.push_back({ entry.node->mChildren[i], (int)index, global, entry.depth + 1 });
        }
    }
}

// ---------------------------------------------------------------------------
// Samples a key frame track at an arbitrary time.
template <typename TKey>
void SampleTrack(const TKey *keys, unsigned int numKeys, double time, typename TKey::elem_type &out) {
    ai_assert(keys != nullptr);
    ai_assert(numKeys > 0);

    if (numKeys == 1 || time <= keys[0].mTime) {
        out = keys[0].mValue;
        return;
    }
    if (time >= keys[numKeys - 1].mTime) {
        out = keys[numKeys - 1].mValue;
        return;
    }

    // Binary search for the key pair surrounding 'time'.
    unsigned int lo = 0;
    unsigned int hi = numKeys - 1;
    while (hi - lo > 1) {
        const unsigned int mid = (lo + hi) / 2;
        if (keys[mid].mTime <= time) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    const double span = keys[hi].mTime - keys[lo].mTime;
    const ai_real factor = span > 0.0 ? (ai_real)((time - keys[lo].mTime) / span) : (ai_real)0.0;
    const Interpolator<TKey> interpolate;
    interpolate(out, keys[lo], keys[hi], factor);
}

// ---------------------------------------------------------------------------
aiNodeAnim *CopyChannel(const aiNodeAnim &channel) {
    std::unique_ptr<aiNodeAnim> copy(new aiNodeAnim);
    copy->mNodeName = channel.mNodeName;
    copy->mPreState = channel.mPreState;
    copy->mPostState = channel.mPostState;

    copy->mNumPositionKeys = channel.mNumPositionKeys;
    if (channel.mNumPositionKeys > 0) {
        copy->mPositionKeys = new aiVectorKey[channel.mNumPositionKeys];
        std::copy(channel.mPositionKeys, channel.mPositionKeys + channel.mNumPositionKeys, copy->mPositionKeys);
    }
    copy->mNumRotationKeys = channel.mNumRotationKeys;
    if (channel.mNumRotationKeys > 0) {
        copy->mRotationKeys = new aiQuatKey[channel.mNumRotationKeys];
        std::copy(channel.mRotationKeys, channel.mRotationKeys + channel.mNumRotationKeys, copy->mRotationKeys);
    }
    copy->mNumScalingKeys = channel.mNumScalingKeys;
    if (channel.mNumScalingKeys > 0) {
        copy->mScalingKeys = new aiVectorKey[channel.mNumScalingKeys];
        std::copy(channel.mScalingKeys, channel.mScalingKeys + channel.mNumScalingKeys, copy->mScalingKeys);
    }
    return copy.release();
}

aiAnimation *CopyAnimation(const aiAnimation &source) {
    std::unique_ptr<aiAnimation> copy(new aiAnimation);
    copy->mName = source.mName;
    copy->mDuration = source.mDuration;
    copy->mTicksPerSecond = source.mTicksPerSecond;

    copy->mNumChannels = source.mNumChannels;
    if (source.mNumChannels > 0) {
        copy->mChannels = new aiNodeAnim *[source.mNumChannels];
        for (unsigned int i = 0; i < source.mNumChannels; ++i) {
            copy->mChannels[i] = CopyChannel(*source.mChannels[i]);
        }
    }

    copy->mNumMeshChannels = source.mNumMeshChannels;
    if (source.mNumMeshChannels > 0) {
        copy->mMeshChannels = new aiMeshAnim *[source.mNumMeshChannels];
        for (unsigned int i = 0; i < source.mNumMeshChannels; ++i) {
            aiMeshAnim *meshAnim = new aiMeshAnim;
            meshAnim->mName = source.mMeshChannels[i]->mName;
            meshAnim->mNumKeys = source.mMeshChannels[i]->mNumKeys;
            if (meshAnim->mNumKeys > 0) {
                meshAnim->mKeys = new aiMeshKey[meshAnim->mNumKeys];
                std::copy(source.mMeshChannels[i]->mKeys, source.mMeshChannels[i]->mKeys + meshAnim->mNumKeys, meshAnim->mKeys);
            }
            copy->mMeshChannels[i] = meshAnim;
        }
    }

    copy->mNumMorphMeshChannels = source.mNumMorphMeshChannels;
    if (source.mNumMorphMeshChannels > 0) {
        copy->mMorphMeshChannels = new aiMeshMorphAnim *[source.mNumMorphMeshChannels];
        for (unsigned int i = 0; i < source.mNumMorphMeshChannels; ++i) {
            const aiMeshMorphAnim *src = source.mMorphMeshChannels[i];
            aiMeshMorphAnim *morphAnim = new aiMeshMorphAnim;
            morphAnim->mName = src->mName;
            morphAnim->mNumKeys = src->mNumKeys;
            if (morphAnim->mNumKeys > 0) {
                morphAnim->mKeys = new aiMeshMorphKey[morphAnim->mNumKeys];
                for (unsigned int k = 0; k < morphAnim->mNumKeys; ++k) {
                    morphAnim->mKeys[k].mTime = src->mKeys[k].mTime;
                    morphAnim->mKeys[k].mNumValuesAndWeights = src->mKeys[k].mNumValuesAndWeights;
                    const unsigned int n = src->mKeys[k].mNumValuesAndWeights;
                    if (n > 0) {
                        morphAnim->mKeys[k].mValues = new unsigned int[n];
                        morphAnim->mKeys[k].mWeights = new double[n];
                        std::copy(src->mKeys[k].mValues, src->mKeys[k].mValues + n, morphAnim->mKeys[k].mValues);
                        std::copy(src->mKeys[k].mWeights, src->mKeys[k].mWeights + n, morphAnim->mKeys[k].mWeights);
                    }
                }
            }
            copy->mMorphMeshChannels[i] = morphAnim;
        }
    }
    return copy.release();
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Cached per-bone retarget data. Everything in here is O(1) per bone and
// computed exactly once in Build(); Retarget() only reads it.
struct BonePlan {
    std::string sourceName;
    std::string targetName;

    // Rotation retarget: qTarget = rotPre * qSource * rotPost.
    aiQuaternion rotPre;
    aiQuaternion rotPost;

    // Rest-pose data of the target bone.
    aiQuaternion targetRestLocalRot;
    aiVector3D targetRestLocalPos;
    aiVector3D targetRestLocalScale;

    // Rest-pose data of the source bone.
    aiVector3D sourceRestLocalPos;
    aiVector3D sourceRestLocalScale;

    // Root-motion transform of the translation delta.
    aiQuaternion sourceParentGlobalRot;
    aiVector3D sourceParentGlobalScale;
    aiQuaternion invTargetParentGlobalRot;
    aiVector3D invTargetParentGlobalScale;
    ai_real positionScale = (ai_real)1.0;

    bool isRootBone = false;
    bool valid = false;
};

// ---------------------------------------------------------------------------
struct AnimationRetarget::Impl {
    RetargetOptions options;
    RetargetMetrics metrics;
    std::vector<BonePlan> bones;
    std::unordered_map<std::string, size_t> planBySourceName;
    double sizeRatio = 1.0;
    bool ready = false;

    aiAnimation *GlobalFallback(const aiAnimation &source) {
        metrics.mLastRetargetUsedGlobalFallback = true;
        ++metrics.mNumGlobalFallbacks;
        return CopyAnimation(source);
    }
};

// ---------------------------------------------------------------------------
AnimationRetarget::AnimationRetarget() :
        mImpl(new Impl) {
    // empty
}

AnimationRetarget::~AnimationRetarget() {
    delete mImpl;
}

bool AnimationRetarget::IsReady() const {
    return mImpl->ready;
}

const RetargetOptions &AnimationRetarget::GetOptions() const {
    return mImpl->options;
}

const RetargetMetrics &AnimationRetarget::GetMetrics() const {
    return mImpl->metrics;
}

double AnimationRetarget::GetSizeRatio() const {
    return mImpl->sizeRatio;
}

// ---------------------------------------------------------------------------
bool AnimationRetarget::Build(const aiNode *sourceRoot, const aiNode *targetRoot, const RetargetOptions &options) {
    mImpl->options = options;
    mImpl->ready = false;
    mImpl->bones.clear();
    mImpl->planBySourceName.clear();
    mImpl->sizeRatio = 1.0;
    mImpl->metrics = RetargetMetrics();

    if (sourceRoot == nullptr || targetRoot == nullptr) {
        return false;
    }

    std::vector<NodeRest> sourceRest;
    std::vector<NodeRest> targetRest;
    std::unordered_map<std::string, size_t> sourceIndex;
    std::unordered_map<std::string, size_t> targetIndex;
    CollectRestPose(sourceRoot, sourceRest, sourceIndex);
    CollectRestPose(targetRoot, targetRest, targetIndex);

    // Resolve the bone mapping: explicit mapping or automatic by name.
    std::vector<std::pair<size_t, size_t>> mapped; // (source idx, target idx)
    if (!options.mBoneMapping.empty()) {
        for (const auto &entry : options.mBoneMapping) {
            const auto srcIt = sourceIndex.find(entry.first);
            const auto tgtIt = targetIndex.find(entry.second);
            if (srcIt != sourceIndex.end() && tgtIt != targetIndex.end()) {
                mapped.emplace_back(srcIt->second, tgtIt->second);
            }
        }
    } else {
        for (size_t i = 0; i < sourceRest.size(); ++i) {
            const auto tgtIt = targetIndex.find(sourceRest[i].name);
            if (tgtIt != targetIndex.end()) {
                mapped.emplace_back(i, tgtIt->second);
            }
        }
    }

    if (mapped.empty()) {
        return false;
    }

    // Global size ratio from the mapped bone lengths (mean of per-bone
    // ratios). Also feeds the scale-error metrics.
    double ratioSum = 0.0;
    unsigned int ratioCount = 0;
    std::vector<double> boneRatios(mapped.size(), 0.0);
    for (size_t i = 0; i < mapped.size(); ++i) {
        const NodeRest &src = sourceRest[mapped[i].first];
        const NodeRest &tgt = targetRest[mapped[i].second];
        if (src.parentIndex < 0 || tgt.parentIndex < 0) {
            continue;
        }
        const ai_real srcLen = (src.globalPos - sourceRest[src.parentIndex].globalPos).Length();
        const ai_real tgtLen = (tgt.globalPos - targetRest[tgt.parentIndex].globalPos).Length();
        if (srcLen > options.mEpsilon && tgtLen > options.mEpsilon) {
            boneRatios[i] = (double)(tgtLen / srcLen);
            ratioSum += boneRatios[i];
            ++ratioCount;
        }
    }
    if (ratioCount > 0) {
        mImpl->sizeRatio = ratioSum / ratioCount;
    }

    // Scale-error metrics: relative deviation of the per-bone ratios from
    // the global ratio. Zero for rigs that differ only by uniform scale.
    double errSum = 0.0;
    double errMax = 0.0;
    for (size_t i = 0; i < mapped.size(); ++i) {
        if (boneRatios[i] <= 0.0 || mImpl->sizeRatio <= 0.0) {
            continue;
        }
        const double err = std::abs(boneRatios[i] - mImpl->sizeRatio) / mImpl->sizeRatio;
        errSum += err;
        errMax = std::max(errMax, err);
    }
    if (ratioCount > 0) {
        mImpl->metrics.mMeanScaleError = errSum / ratioCount;
        mImpl->metrics.mMaxScaleError = errMax;
    }

    // Root bone: explicit option or the mapped bone closest to the source root.
    size_t rootMapping = std::numeric_limits<size_t>::max();
    if (!options.mRootBoneName.empty()) {
        const auto it = sourceIndex.find(options.mRootBoneName);
        if (it != sourceIndex.end()) {
            for (size_t i = 0; i < mapped.size(); ++i) {
                if (mapped[i].first == it->second) {
                    rootMapping = i;
                    break;
                }
            }
        }
    }
    if (rootMapping == std::numeric_limits<size_t>::max()) {
        unsigned int bestDepth = std::numeric_limits<unsigned int>::max();
        for (size_t i = 0; i < mapped.size(); ++i) {
            if (sourceRest[mapped[i].first].depth < bestDepth) {
                bestDepth = sourceRest[mapped[i].first].depth;
                rootMapping = i;
            }
        }
    }

    // Build the cached per-bone plan.
    const ai_real eps = options.mEpsilon;
    const ai_real maxPosScale = std::max(options.mMaxPositionScale, (ai_real)1.0);
    mImpl->bones.reserve(mapped.size());
    for (size_t i = 0; i < mapped.size(); ++i) {
        const NodeRest &src = sourceRest[mapped[i].first];
        const NodeRest &tgt = targetRest[mapped[i].second];

        BonePlan plan;
        plan.sourceName = src.name;
        plan.targetName = tgt.name;
        plan.targetRestLocalRot = tgt.localRot;
        plan.targetRestLocalPos = tgt.localPos;
        plan.targetRestLocalScale = tgt.localScale;
        plan.sourceRestLocalPos = src.localPos;
        plan.sourceRestLocalScale = src.localScale;
        plan.isRootBone = (i == rootMapping);

        const NodeRest &srcParent = src.parentIndex >= 0 ? sourceRest[src.parentIndex] : src;
        const NodeRest &tgtParent = tgt.parentIndex >= 0 ? targetRest[tgt.parentIndex] : tgt;

        // qTargetLocal = inv(tgtParentGlobal) * srcParentGlobal * qSourceLocal
        //                * inv(srcGlobal) * tgtGlobal
        plan.rotPre = Inverted(tgtParent.globalRot) * srcParent.globalRot;
        plan.rotPost = Inverted(src.globalRot) * tgt.globalRot;

        // Root-motion delta transform (world-space delta, scaled, then
        // transformed back into the target parent frame).
        plan.sourceParentGlobalRot = srcParent.globalRot;
        plan.sourceParentGlobalScale = srcParent.globalScale;
        plan.invTargetParentGlobalRot = Inverted(tgtParent.globalRot);
        plan.invTargetParentGlobalScale = DivComponentsGuarded(aiVector3D(1, 1, 1), tgtParent.globalScale, eps);

        ai_real posScale = (ai_real)mImpl->sizeRatio;
        posScale = std::min(std::max(posScale, (ai_real)1.0 / maxPosScale), maxPosScale);
        plan.positionScale = posScale;

        plan.valid = IsFinite(plan.rotPre) && IsFinite(plan.rotPost) &&
                IsFinite(plan.targetRestLocalPos) && IsFinite(plan.targetRestLocalScale) &&
                IsFinite(plan.sourceRestLocalPos) && IsFinite(plan.sourceRestLocalScale);

        mImpl->planBySourceName[plan.sourceName] = mImpl->bones.size();
        mImpl->bones.push_back(plan);
    }

    mImpl->ready = true;
    return true;
}

// ---------------------------------------------------------------------------
aiAnimation *AnimationRetarget::Retarget(const aiAnimation *source) {
    mImpl->metrics.mNumSourceChannels = 0;
    mImpl->metrics.mNumMappedChannels = 0;
    mImpl->metrics.mNumUnmappedChannels = 0;
    mImpl->metrics.mNumFallbackChannels = 0;
    mImpl->metrics.mNumLoopClosures = 0;
    mImpl->metrics.mCoverage = 0.0;
    mImpl->metrics.mLastRetargetUsedGlobalFallback = false;

    if (source == nullptr) {
        return nullptr;
    }
    if (!mImpl->ready) {
        return mImpl->GlobalFallback(*source);
    }

    const RetargetOptions &options = mImpl->options;
    const ai_real eps = options.mEpsilon;

    // Channel coverage of this animation.
    unsigned int mappedChannels = 0;
    for (unsigned int i = 0; i < source->mNumChannels; ++i) {
        if (mImpl->planBySourceName.find(source->mChannels[i]->mNodeName.C_Str()) != mImpl->planBySourceName.end()) {
            ++mappedChannels;
        }
    }
    mImpl->metrics.mNumSourceChannels = source->mNumChannels;
    mImpl->metrics.mNumMappedChannels = mappedChannels;
    mImpl->metrics.mNumUnmappedChannels = source->mNumChannels - mappedChannels;
    mImpl->metrics.mCoverage = source->mNumChannels > 0 ? (double)mappedChannels / source->mNumChannels : 0.0;

    if (mImpl->metrics.mCoverage < options.mMinCoverage) {
        return mImpl->GlobalFallback(*source);
    }

    // Frame-rate conversion: uniform resampling grid in output ticks.
    const double sourceTps = source->mTicksPerSecond > 0.0 ? source->mTicksPerSecond : 1.0;
    const double targetTps = options.mTargetTicksPerSecond > 0.0 ? options.mTargetTicksPerSecond : sourceTps;
    const bool resample = targetTps != sourceTps;
    const double outDuration = source->mDuration * (targetTps / sourceTps);

    std::unique_ptr<aiAnimation> out(new aiAnimation);
    out->mName = source->mName;
    out->mDuration = outDuration;
    out->mTicksPerSecond = targetTps;
    out->mChannels = new aiNodeAnim *[mappedChannels];
    out->mNumChannels = mappedChannels;

    unsigned int outChannelIndex = 0;
    for (unsigned int c = 0; c < source->mNumChannels; ++c) {
        const aiNodeAnim &channel = *source->mChannels[c];
        const auto planIt = mImpl->planBySourceName.find(channel.mNodeName.C_Str());
        if (planIt == mImpl->planBySourceName.end()) {
            continue; // Unmapped bone: the target rig keeps its rest pose.
        }
        const BonePlan &plan = mImpl->bones[planIt->second];

        std::unique_ptr<aiNodeAnim> outChannel(new aiNodeAnim);
        outChannel->mNodeName = aiString(plan.targetName);
        outChannel->mPreState = channel.mPreState;
        outChannel->mPostState = channel.mPostState;

        bool channelOk = plan.valid;

        // Output sample times for this channel.
        std::vector<double> times;
        if (resample) {
            const unsigned int numSamples = (unsigned int)std::max(1.0, std::ceil(outDuration)) + 1;
            times.reserve(numSamples);
            for (unsigned int s = 0; s < numSamples; ++s) {
                times.push_back(std::min((double)s, outDuration));
            }
            if (times.back() < outDuration) {
                times.push_back(outDuration);
            }
        } else {
            const unsigned int maxKeys = std::max(channel.mNumPositionKeys,
                    std::max(channel.mNumRotationKeys, channel.mNumScalingKeys));
            times.reserve(std::max(1u, maxKeys));
            for (unsigned int k = 0; k < maxKeys; ++k) {
                double t = 0.0;
                if (channel.mNumRotationKeys > 0) {
                    t = channel.mRotationKeys[std::min(k, channel.mNumRotationKeys - 1)].mTime;
                } else if (channel.mNumPositionKeys > 0) {
                    t = channel.mPositionKeys[std::min(k, channel.mNumPositionKeys - 1)].mTime;
                } else if (channel.mNumScalingKeys > 0) {
                    t = channel.mScalingKeys[std::min(k, channel.mNumScalingKeys - 1)].mTime;
                }
                if (times.empty() || t > times.back()) {
                    times.push_back(t);
                }
            }
            if (times.empty()) {
                times.push_back(0.0);
            }
        }
        const double timeToSource = sourceTps / targetTps;

        // --- Rotations -------------------------------------------------
        const unsigned int numRotKeys = channel.mNumRotationKeys > 0 ? (unsigned int)times.size() : 1;
        outChannel->mNumRotationKeys = numRotKeys;
        outChannel->mRotationKeys = new aiQuatKey[numRotKeys];
        if (channel.mNumRotationKeys > 0) {
            aiQuaternion previous;
            for (unsigned int k = 0; k < numRotKeys && channelOk; ++k) {
                const double srcTime = resample ? times[k] * timeToSource : times[k];
                aiQuaternion srcRot;
                SampleTrack(channel.mRotationKeys, channel.mNumRotationKeys, srcTime, srcRot);
                srcRot.Normalize();

                aiQuaternion tgtRot = plan.rotPre * srcRot * plan.rotPost;
                tgtRot.Normalize();

                // Keep the track in one hemisphere: flip the sign if this
                // key jumps to the antipodal representation.
                if (k > 0 && Dot(previous, tgtRot) < (ai_real)0.0) {
                    tgtRot.w = -tgtRot.w;
                    tgtRot.x = -tgtRot.x;
                    tgtRot.y = -tgtRot.y;
                    tgtRot.z = -tgtRot.z;
                }
                channelOk = IsFinite(tgtRot);
                previous = tgtRot;

                outChannel->mRotationKeys[k].mTime = times[k];
                outChannel->mRotationKeys[k].mValue = tgtRot;
            }
        } else {
            // Missing rotation channel: the bone keeps its rest rotation.
            outChannel->mRotationKeys[0].mTime = 0.0;
            outChannel->mRotationKeys[0].mValue = plan.targetRestLocalRot;
        }

        // --- Positions (root motion on the root bone only) --------------
        const bool translateRoot = plan.isRootBone && options.mEnableRootMotion && channel.mNumPositionKeys > 0;
        const unsigned int numPosKeys = translateRoot ? (unsigned int)times.size() : 1;
        outChannel->mNumPositionKeys = numPosKeys;
        outChannel->mPositionKeys = new aiVectorKey[numPosKeys];
        if (translateRoot) {
            for (unsigned int k = 0; k < numPosKeys && channelOk; ++k) {
                const double srcTime = resample ? times[k] * timeToSource : times[k];
                aiVector3D srcPos;
                SampleTrack(channel.mPositionKeys, channel.mNumPositionKeys, srcTime, srcPos);

                const aiVector3D delta = srcPos - plan.sourceRestLocalPos;
                const aiVector3D worldDelta = plan.sourceParentGlobalRot.Rotate(
                        MulComponents(delta, plan.sourceParentGlobalScale));
                const aiVector3D scaledDelta = worldDelta * plan.positionScale;
                aiVector3D localDelta = plan.invTargetParentGlobalRot.Rotate(scaledDelta);
                localDelta = MulComponents(localDelta, plan.invTargetParentGlobalScale);

                const aiVector3D tgtPos = plan.targetRestLocalPos + localDelta;
                channelOk = IsFinite(tgtPos);

                outChannel->mPositionKeys[k].mTime = times[k];
                outChannel->mPositionKeys[k].mValue = tgtPos;
            }
        } else {
            // Non-root bones keep their rest translation: bone lengths of
            // the two rigs differ and must not be animated by the source.
            outChannel->mPositionKeys[0].mTime = 0.0;
            outChannel->mPositionKeys[0].mValue = plan.targetRestLocalPos;
        }

        // --- Scaling -----------------------------------------------------
        const bool retargetScale = options.mRetargetScaleKeys && channel.mNumScalingKeys > 0;
        const unsigned int numScaleKeys = retargetScale ? (unsigned int)times.size() : 1;
        outChannel->mNumScalingKeys = numScaleKeys;
        outChannel->mScalingKeys = new aiVectorKey[numScaleKeys];
        if (retargetScale) {
            for (unsigned int k = 0; k < numScaleKeys && channelOk; ++k) {
                const double srcTime = resample ? times[k] * timeToSource : times[k];
                aiVector3D srcScale;
                SampleTrack(channel.mScalingKeys, channel.mNumScalingKeys, srcTime, srcScale);

                // Scale relative to the rest scale, then re-applied to the
                // target rest scale (handles non-uniform rest scaling).
                const aiVector3D relative = DivComponentsGuarded(srcScale, plan.sourceRestLocalScale, eps);
                const aiVector3D tgtScale = MulComponents(relative, plan.targetRestLocalScale);
                channelOk = IsFinite(tgtScale);

                outChannel->mScalingKeys[k].mTime = times[k];
                outChannel->mScalingKeys[k].mValue = tgtScale;
            }
        } else {
            outChannel->mScalingKeys[0].mTime = 0.0;
            outChannel->mScalingKeys[0].mValue = plan.targetRestLocalScale;
        }

        // --- Loop closure ------------------------------------------------
        if (channelOk && options.mEnforceLoopClosure && numRotKeys > 1 && channel.mNumRotationKeys > 1) {
            const aiQuaternion &first = channel.mRotationKeys[0].mValue;
            const aiQuaternion &last = channel.mRotationKeys[channel.mNumRotationKeys - 1].mValue;
            const ai_real loopEps = (ai_real)1e-4;
            if (std::abs(std::abs(Dot(first, last)) - (ai_real)1.0) < loopEps) {
                // The source loops: snap the last output key to the first so
                // retarget drift cannot break the loop.
                outChannel->mRotationKeys[numRotKeys - 1].mValue = outChannel->mRotationKeys[0].mValue;
                if (translateRoot && channel.mNumPositionKeys > 1) {
                    const aiVector3D &firstPos = channel.mPositionKeys[0].mValue;
                    const aiVector3D &lastPos = channel.mPositionKeys[channel.mNumPositionKeys - 1].mValue;
                    if ((firstPos - lastPos).SquareLength() < loopEps * loopEps) {
                        outChannel->mPositionKeys[numPosKeys - 1].mValue = outChannel->mPositionKeys[0].mValue;
                    }
                }
                ++mImpl->metrics.mNumLoopClosures;
            }
        }

        if (!channelOk) {
            // Per-channel fallback: keep the original track data (renamed to
            // the target bone) instead of emitting distorted values.
            std::unique_ptr<aiNodeAnim> fallback(CopyChannel(channel));
            fallback->mNodeName = aiString(plan.targetName);
            outChannel = std::move(fallback);
            ++mImpl->metrics.mNumFallbackChannels;
        }

        out->mChannels[outChannelIndex++] = outChannel.release();
    }

    // Mesh and morph channels are not bone-bound: copy them unchanged.
    out->mNumMeshChannels = source->mNumMeshChannels;
    if (source->mNumMeshChannels > 0) {
        out->mMeshChannels = new aiMeshAnim *[source->mNumMeshChannels];
        for (unsigned int i = 0; i < source->mNumMeshChannels; ++i) {
            aiMeshAnim *meshAnim = new aiMeshAnim;
            meshAnim->mName = source->mMeshChannels[i]->mName;
            meshAnim->mNumKeys = source->mMeshChannels[i]->mNumKeys;
            if (meshAnim->mNumKeys > 0) {
                meshAnim->mKeys = new aiMeshKey[meshAnim->mNumKeys];
                std::copy(source->mMeshChannels[i]->mKeys, source->mMeshChannels[i]->mKeys + meshAnim->mNumKeys, meshAnim->mKeys);
            }
            out->mMeshChannels[i] = meshAnim;
        }
    }
    out->mNumMorphMeshChannels = source->mNumMorphMeshChannels;
    if (source->mNumMorphMeshChannels > 0) {
        out->mMorphMeshChannels = new aiMeshMorphAnim *[source->mNumMorphMeshChannels];
        for (unsigned int i = 0; i < source->mNumMorphMeshChannels; ++i) {
            const aiMeshMorphAnim *src = source->mMorphMeshChannels[i];
            aiMeshMorphAnim *morphAnim = new aiMeshMorphAnim;
            morphAnim->mName = src->mName;
            morphAnim->mNumKeys = src->mNumKeys;
            if (morphAnim->mNumKeys > 0) {
                morphAnim->mKeys = new aiMeshMorphKey[morphAnim->mNumKeys];
                for (unsigned int k = 0; k < morphAnim->mNumKeys; ++k) {
                    morphAnim->mKeys[k].mTime = src->mKeys[k].mTime;
                    morphAnim->mKeys[k].mNumValuesAndWeights = src->mKeys[k].mNumValuesAndWeights;
                    const unsigned int n = src->mKeys[k].mNumValuesAndWeights;
                    if (n > 0) {
                        morphAnim->mKeys[k].mValues = new unsigned int[n];
                        morphAnim->mKeys[k].mWeights = new double[n];
                        std::copy(src->mKeys[k].mValues, src->mKeys[k].mValues + n, morphAnim->mKeys[k].mValues);
                        std::copy(src->mKeys[k].mWeights, src->mKeys[k].mWeights + n, morphAnim->mKeys[k].mWeights);
                    }
                }
            }
            out->mMorphMeshChannels[i] = morphAnim;
        }
    }

    return out.release();
}

} // namespace Assimp
