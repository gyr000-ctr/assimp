/*
---------------------------------------------------------------------------
Open Asset Import Library (assimp)
---------------------------------------------------------------------------

Copyright (c) 2006-2026, assimp team

All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the following
conditions are met:

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
---------------------------------------------------------------------------
*/

#include "UnitTestPCH.h"

#include "Common/AnimationRetarget.h"

#include <assimp/matrix4x4.h>
#include <assimp/scene.h>

#include <memory>

using namespace Assimp;

namespace {

// ---------------------------------------------------------------------------
// Test helpers: synthetic skeletons and animations.

aiNode *CreateNode(const std::string &name, const aiVector3D &localPos) {
    aiNode *node = new aiNode;
    node->mName = aiString(name);
    aiMatrix4x4::Translation(localPos, node->mTransformation);
    return node;
}

void AttachChild(aiNode *parent, aiNode *child) {
    aiNode **children = new aiNode *[parent->mNumChildren + 1];
    for (unsigned int i = 0; i < parent->mNumChildren; ++i) {
        children[i] = parent->mChildren[i];
    }
    children[parent->mNumChildren] = child;
    delete[] parent->mChildren;
    parent->mChildren = children;
    ++parent->mNumChildren;
    child->mParent = parent;
}

struct Skeleton {
    std::unique_ptr<aiNode> root;
    aiNode *hips = nullptr;
    aiNode *spine = nullptr;
    aiNode *arm = nullptr;
};

// A small humanoid-ish rig: Root -> Hips -> Spine -> Arm.
// scale uniformly grows the bone offsets, armFactor makes the
// proportions non-uniform when != scale.
Skeleton CreateSkeleton(ai_real scale, ai_real armFactor, bool withArm) {
    Skeleton skel;
    skel.root.reset(CreateNode("Root", aiVector3D(0, 0, 0)));
    skel.hips = CreateNode("Hips", aiVector3D(0, 1, 0) * scale);
    skel.spine = CreateNode("Spine", aiVector3D(0, (ai_real)0.5, 0) * scale);
    AttachChild(skel.root.get(), skel.hips);
    AttachChild(skel.hips, skel.spine);
    if (withArm) {
        skel.arm = CreateNode("Arm", aiVector3D((ai_real)0.5, (ai_real)0.25, 0) * armFactor);
        AttachChild(skel.spine, skel.arm);
    }
    return skel;
}

struct AnimationDeleter {
    void operator()(aiAnimation *anim) const {
        delete anim;
    }
};
using AnimationPtr = std::unique_ptr<aiAnimation, AnimationDeleter>;

aiNodeAnim *AddChannel(aiAnimation *anim, const std::string &nodeName) {
    aiNodeAnim **channels = new aiNodeAnim *[anim->mNumChannels + 1];
    for (unsigned int i = 0; i < anim->mNumChannels; ++i) {
        channels[i] = anim->mChannels[i];
    }
    aiNodeAnim *channel = new aiNodeAnim;
    channel->mNodeName = aiString(nodeName);
    channels[anim->mNumChannels] = channel;
    delete[] anim->mChannels;
    anim->mChannels = channels;
    ++anim->mNumChannels;
    return channel;
}

void SetRotationKeys(aiNodeAnim *channel, const std::vector<aiQuatKey> &keys) {
    delete[] channel->mRotationKeys;
    channel->mNumRotationKeys = (unsigned int)keys.size();
    channel->mRotationKeys = new aiQuatKey[keys.size()];
    std::copy(keys.begin(), keys.end(), channel->mRotationKeys);
}

void SetPositionKeys(aiNodeAnim *channel, const std::vector<aiVectorKey> &keys) {
    delete[] channel->mPositionKeys;
    channel->mNumPositionKeys = (unsigned int)keys.size();
    channel->mPositionKeys = new aiVectorKey[keys.size()];
    std::copy(keys.begin(), keys.end(), channel->mPositionKeys);
}

void SetScalingKeys(aiNodeAnim *channel, const std::vector<aiVectorKey> &keys) {
    delete[] channel->mScalingKeys;
    channel->mNumScalingKeys = (unsigned int)keys.size();
    channel->mScalingKeys = new aiVectorKey[keys.size()];
    std::copy(keys.begin(), keys.end(), channel->mScalingKeys);
}

const aiNodeAnim *FindChannel(const aiAnimation *anim, const std::string &name) {
    for (unsigned int i = 0; i < anim->mNumChannels; ++i) {
        if (name == anim->mChannels[i]->mNodeName.C_Str()) {
            return anim->mChannels[i];
        }
    }
    return nullptr;
}

aiQuaternion RotationZ(ai_real angle) {
    return aiQuaternion(aiVector3D(0, 0, 1), angle);
}

void ExpectVecNear(const aiVector3D &actual, const aiVector3D &expected, ai_real eps) {
    EXPECT_NEAR(actual.x, expected.x, eps);
    EXPECT_NEAR(actual.y, expected.y, eps);
    EXPECT_NEAR(actual.z, expected.z, eps);
}

void ExpectQuatNear(const aiQuaternion &actual, const aiQuaternion &expected, ai_real eps) {
    // Quaternions are double-cover: compare up to sign.
    const ai_real dot = actual.w * expected.w + actual.x * expected.x + actual.y * expected.y + actual.z * expected.z;
    const ai_real sign = dot < 0 ? (ai_real)-1.0 : (ai_real)1.0;
    EXPECT_NEAR(actual.w, sign * expected.w, eps);
    EXPECT_NEAR(actual.x, sign * expected.x, eps);
    EXPECT_NEAR(actual.y, sign * expected.y, eps);
    EXPECT_NEAR(actual.z, sign * expected.z, eps);
}

} // anonymous namespace

// ---------------------------------------------------------------------------
class utAnimationRetarget : public ::testing::Test {
protected:
    RetargetOptions DefaultOptions() {
        RetargetOptions options;
        options.mRootBoneName = "Hips";
        return options;
    }
};

// Same skeleton on both sides: the retargeted animation must reproduce the
// source tracks (identity retarget).
TEST_F(utAnimationRetarget, sameSkeleton_identityRetarget) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(1, 1, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 2.0;
    anim->mTicksPerSecond = 30.0;

    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.3)), aiQuatKey(2.0, RotationZ((ai_real)0.6)) });
    SetPositionKeys(hips, { aiVectorKey(0.0, aiVector3D(0, 1, 0)), aiVectorKey(2.0, aiVector3D(0, 1, 0)) });
    SetScalingKeys(hips, { aiVectorKey(0.0, aiVector3D(1, 1, 1)), aiVectorKey(2.0, aiVector3D(1, 1, 1)) });

    aiNodeAnim *spine = AddChannel(anim.get(), "Spine");
    SetRotationKeys(spine, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(2.0, RotationZ((ai_real)-0.4)) });

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));
    EXPECT_DOUBLE_EQ(1.0, retarget.GetSizeRatio());

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    ASSERT_EQ(2u, out->mNumChannels);

    const RetargetMetrics &metrics = retarget.GetMetrics();
    EXPECT_EQ(2u, metrics.mNumSourceChannels);
    EXPECT_EQ(2u, metrics.mNumMappedChannels);
    EXPECT_EQ(0u, metrics.mNumUnmappedChannels);
    EXPECT_EQ(0u, metrics.mNumFallbackChannels);
    EXPECT_DOUBLE_EQ(1.0, metrics.mCoverage);
    EXPECT_NEAR(0.0, metrics.mMaxScaleError, 1e-6);
    EXPECT_FALSE(metrics.mLastRetargetUsedGlobalFallback);

    const aiNodeAnim *outHips = FindChannel(out.get(), "Hips");
    ASSERT_NE(outHips, nullptr);
    ASSERT_EQ(3u, outHips->mNumRotationKeys);
    ExpectQuatNear(outHips->mRotationKeys[1].mValue, RotationZ((ai_real)0.3), (ai_real)1e-5);
    ExpectQuatNear(outHips->mRotationKeys[2].mValue, RotationZ((ai_real)0.6), (ai_real)1e-5);

    const aiNodeAnim *outSpine = FindChannel(out.get(), "Spine");
    ASSERT_NE(outSpine, nullptr);
    ExpectQuatNear(outSpine->mRotationKeys[1].mValue, RotationZ((ai_real)-0.4), (ai_real)1e-5);
}

// Target rig with uniformly doubled proportions: rotations are preserved,
// root motion is scaled by the size ratio.
TEST_F(utAnimationRetarget, differentProportions_scalesRootMotion) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(2, 2, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 2.0;
    anim->mTicksPerSecond = 30.0;

    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(2.0, RotationZ((ai_real)0.5)) });
    SetPositionKeys(hips, { aiVectorKey(0.0, aiVector3D(0, 1, 0)), aiVectorKey(2.0, aiVector3D(1, 1, 0)) });

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));
    EXPECT_NEAR(2.0, retarget.GetSizeRatio(), 1e-5);

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);

    const aiNodeAnim *outHips = FindChannel(out.get(), "Hips");
    ASSERT_NE(outHips, nullptr);
    ASSERT_EQ(2u, outHips->mNumPositionKeys);
    // Rest position of the target hips plus the doubled motion delta.
    ExpectVecNear(outHips->mPositionKeys[0].mValue, aiVector3D(0, 2, 0), (ai_real)1e-5);
    ExpectVecNear(outHips->mPositionKeys[1].mValue, aiVector3D(2, 2, 0), (ai_real)1e-5);
    ExpectQuatNear(outHips->mRotationKeys[1].mValue, RotationZ((ai_real)0.5), (ai_real)1e-5);

    // Non-root bones keep the target rest translation (bone lengths differ).
    const aiNodeAnim *spine = FindChannel(out.get(), "Spine");
    ASSERT_EQ(spine, nullptr); // no Spine channel in the source animation
}

// Non-uniform proportions: the per-bone ratios differ, which must show up
// in the scale-error metrics.
TEST_F(utAnimationRetarget, nonUniformProportions_reportsScaleError) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(2, 3, true); // arm grows 3x, rest 2x

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));

    const RetargetMetrics &metrics = retarget.GetMetrics();
    EXPECT_GT(metrics.mMaxScaleError, 0.1);
    EXPECT_GT(metrics.mMeanScaleError, 0.0);
    EXPECT_GE(metrics.mMaxScaleError, metrics.mMeanScaleError);
}

// Bones missing on the target rig are skipped and counted; the remaining
// channels are still retargeted.
TEST_F(utAnimationRetarget, missingBones_skipsUnmappedChannels) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(1, 1, false); // no "Arm" on the target

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 1.0;
    anim->mTicksPerSecond = 30.0;

    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.2)) });
    aiNodeAnim *arm = AddChannel(anim.get(), "Arm");
    SetRotationKeys(arm, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.7)) });

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(1u, out->mNumChannels);
    EXPECT_NE(FindChannel(out.get(), "Hips"), nullptr);
    EXPECT_EQ(FindChannel(out.get(), "Arm"), nullptr);

    const RetargetMetrics &metrics = retarget.GetMetrics();
    EXPECT_EQ(2u, metrics.mNumSourceChannels);
    EXPECT_EQ(1u, metrics.mNumMappedChannels);
    EXPECT_EQ(1u, metrics.mNumUnmappedChannels);
    EXPECT_DOUBLE_EQ(0.5, metrics.mCoverage);
    EXPECT_FALSE(metrics.mLastRetargetUsedGlobalFallback);
}

// If the coverage drops below the configured minimum, the source animation
// must be returned unchanged instead of a distorted retarget.
TEST_F(utAnimationRetarget, lowCoverage_fallsBackToSourceAnimation) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(1, 1, false); // covers only Hips/Spine/Root

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 1.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.2)) });
    aiNodeAnim *arm = AddChannel(anim.get(), "Arm");
    SetRotationKeys(arm, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.7)) });

    RetargetOptions options = DefaultOptions();
    options.mMinCoverage = 0.9; // coverage is 0.5 -> global fallback

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), options));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    // Unmodified copy: same channels, same target names, same keys.
    ASSERT_EQ(2u, out->mNumChannels);
    EXPECT_NE(FindChannel(out.get(), "Arm"), nullptr);
    const aiNodeAnim *outArm = FindChannel(out.get(), "Arm");
    ASSERT_EQ(2u, outArm->mNumRotationKeys);
    ExpectQuatNear(outArm->mRotationKeys[1].mValue, RotationZ((ai_real)0.7), (ai_real)1e-6);

    const RetargetMetrics &metrics = retarget.GetMetrics();
    EXPECT_TRUE(metrics.mLastRetargetUsedGlobalFallback);
    EXPECT_EQ(1u, metrics.mNumGlobalFallbacks);
}

// Root motion can be disabled to produce an in-place animation.
TEST_F(utAnimationRetarget, rootMotion_canBeDisabled) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(2, 2, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 2.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(2.0, RotationZ((ai_real)0.5)) });
    SetPositionKeys(hips, { aiVectorKey(0.0, aiVector3D(0, 1, 0)), aiVectorKey(2.0, aiVector3D(3, 1, 0)) });

    RetargetOptions options = DefaultOptions();
    options.mEnableRootMotion = false;

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), options));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    const aiNodeAnim *outHips = FindChannel(out.get(), "Hips");
    ASSERT_NE(outHips, nullptr);
    // No root motion: a single rest-position key.
    ASSERT_EQ(1u, outHips->mNumPositionKeys);
    ExpectVecNear(outHips->mPositionKeys[0].mValue, aiVector3D(0, 2, 0), (ai_real)1e-5);
    // Rotations are still retargeted.
    ExpectQuatNear(outHips->mRotationKeys[1].mValue, RotationZ((ai_real)0.5), (ai_real)1e-5);
}

// Quaternion keys must stay in one hemisphere even if the source track
// flips the sign of consecutive keys.
TEST_F(utAnimationRetarget, quaternionSign_isKeptContinuous) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(1, 1, true);

    const aiQuaternion q = RotationZ((ai_real)0.4);
    const aiQuaternion qFlipped(-q.w, -q.x, -q.y, -q.z); // same rotation

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 2.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, q), aiQuatKey(1.0, qFlipped), aiQuatKey(2.0, q) });

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    const aiNodeAnim *outHips = FindChannel(out.get(), "Hips");
    ASSERT_NE(outHips, nullptr);
    ASSERT_EQ(3u, outHips->mNumRotationKeys);
    for (unsigned int k = 1; k < outHips->mNumRotationKeys; ++k) {
        const aiQuaternion &a = outHips->mRotationKeys[k - 1].mValue;
        const aiQuaternion &b = outHips->mRotationKeys[k].mValue;
        const ai_real dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
        EXPECT_GE(dot, (ai_real)0.0);
    }
}

// A different output frame rate resamples the channels uniformly.
TEST_F(utAnimationRetarget, differentFrameRate_resamplesChannels) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(1, 1, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 30.0; // 1 second at 30 tps
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(30.0, RotationZ((ai_real)0.6)) });

    RetargetOptions options = DefaultOptions();
    options.mTargetTicksPerSecond = 60.0;

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), options));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    EXPECT_DOUBLE_EQ(60.0, out->mTicksPerSecond);
    EXPECT_DOUBLE_EQ(60.0, out->mDuration);

    const aiNodeAnim *outHips = FindChannel(out.get(), "Hips");
    ASSERT_NE(outHips, nullptr);
    EXPECT_EQ(61u, outHips->mNumRotationKeys); // uniform grid, 1 tick spacing
    EXPECT_DOUBLE_EQ(0.0, outHips->mRotationKeys[0].mTime);
    EXPECT_DOUBLE_EQ(60.0, outHips->mRotationKeys[60].mTime);
    // Mid-point of the slerp between identity and 0.6 rad is 0.3 rad.
    ExpectQuatNear(outHips->mRotationKeys[30].mValue, RotationZ((ai_real)0.3), (ai_real)1e-4);
}

// A looping source animation must still loop exactly after retargeting.
TEST_F(utAnimationRetarget, loopClosure_isEnforced) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(2, 2, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 2.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.5)), aiQuatKey(2.0, RotationZ(0)) });
    SetPositionKeys(hips, { aiVectorKey(0.0, aiVector3D(0, 1, 0)), aiVectorKey(1.0, aiVector3D(1, 1, 0)), aiVectorKey(2.0, aiVector3D(0, 1, 0)) });

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    const aiNodeAnim *outHips = FindChannel(out.get(), "Hips");
    ASSERT_NE(outHips, nullptr);
    ASSERT_EQ(3u, outHips->mNumRotationKeys);

    const aiQuaternion &first = outHips->mRotationKeys[0].mValue;
    const aiQuaternion &last = outHips->mRotationKeys[2].mValue;
    EXPECT_EQ(first.w, last.w);
    EXPECT_EQ(first.x, last.x);
    EXPECT_EQ(first.y, last.y);
    EXPECT_EQ(first.z, last.z);
    EXPECT_EQ(outHips->mPositionKeys[0].mValue, outHips->mPositionKeys[2].mValue);
    EXPECT_EQ(1u, retarget.GetMetrics().mNumLoopClosures);
}

// Missing channels in the source are filled from the target rest pose.
TEST_F(utAnimationRetarget, missingChannels_filledFromRestPose) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(2, 2, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 1.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    // Rotation keys only: no position, no scaling track.
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.2)) });

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    const aiNodeAnim *outHips = FindChannel(out.get(), "Hips");
    ASSERT_NE(outHips, nullptr);
    ASSERT_EQ(1u, outHips->mNumPositionKeys);
    ExpectVecNear(outHips->mPositionKeys[0].mValue, aiVector3D(0, 2, 0), (ai_real)1e-5);
    ASSERT_EQ(1u, outHips->mNumScalingKeys);
    ExpectVecNear(outHips->mScalingKeys[0].mValue, aiVector3D(1, 1, 1), (ai_real)1e-5);
}

// Explicit bone mapping retargets across differently named rigs.
TEST_F(utAnimationRetarget, explicitBoneMapping_mapsRenamedBones) {
    Skeleton source = CreateSkeleton(1, 1, true);

    // Target rig with different bone names.
    Skeleton target;
    target.root.reset(CreateNode("Armature", aiVector3D(0, 0, 0)));
    target.hips = CreateNode("Pelvis", aiVector3D(0, 2, 0));
    target.spine = CreateNode("Spine01", aiVector3D(0, 1, 0));
    AttachChild(target.root.get(), target.hips);
    AttachChild(target.hips, target.spine);

    RetargetOptions options;
    options.mBoneMapping["Hips"] = "Pelvis";
    options.mBoneMapping["Spine"] = "Spine01";
    options.mRootBoneName = "Hips";

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 1.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.3)) });
    SetPositionKeys(hips, { aiVectorKey(0.0, aiVector3D(0, 1, 0)), aiVectorKey(1.0, aiVector3D(1, 1, 0)) });

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), options));

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    const aiNodeAnim *pelvis = FindChannel(out.get(), "Pelvis");
    ASSERT_NE(pelvis, nullptr);
    EXPECT_EQ(FindChannel(out.get(), "Hips"), nullptr);
    ExpectQuatNear(pelvis->mRotationKeys[1].mValue, RotationZ((ai_real)0.3), (ai_real)1e-5);
    ASSERT_EQ(2u, pelvis->mNumPositionKeys);
    ExpectVecNear(pelvis->mPositionKeys[1].mValue, aiVector3D(2, 2, 0), (ai_real)1e-5);
    EXPECT_DOUBLE_EQ(1.0, retarget.GetMetrics().mCoverage);
}

// Building with invalid input fails cleanly and Retarget falls back.
TEST_F(utAnimationRetarget, invalidBuild_fallsBackToSource) {
    Skeleton source = CreateSkeleton(1, 1, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 1.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)) });

    AnimationRetarget retarget;
    EXPECT_FALSE(retarget.Build(source.root.get(), nullptr, DefaultOptions()));
    EXPECT_FALSE(retarget.IsReady());

    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);
    EXPECT_EQ(1u, out->mNumChannels);
    EXPECT_TRUE(retarget.GetMetrics().mLastRetargetUsedGlobalFallback);
    EXPECT_EQ(1u, retarget.GetMetrics().mNumGlobalFallbacks);
}

// The input animation and the scene graphs must not be modified.
TEST_F(utAnimationRetarget, inputsAreNotModified) {
    Skeleton source = CreateSkeleton(1, 1, true);
    Skeleton target = CreateSkeleton(2, 2, true);

    AnimationPtr anim(new aiAnimation);
    anim->mDuration = 1.0;
    anim->mTicksPerSecond = 30.0;
    aiNodeAnim *hips = AddChannel(anim.get(), "Hips");
    SetRotationKeys(hips, { aiQuatKey(0.0, RotationZ(0)), aiQuatKey(1.0, RotationZ((ai_real)0.3)) });
    SetPositionKeys(hips, { aiVectorKey(0.0, aiVector3D(0, 1, 0)), aiVectorKey(1.0, aiVector3D(1, 1, 0)) });

    const aiMatrix4x4 sourceTransform = source.hips->mTransformation;
    const aiMatrix4x4 targetTransform = target.hips->mTransformation;

    AnimationRetarget retarget;
    ASSERT_TRUE(retarget.Build(source.root.get(), target.root.get(), DefaultOptions()));
    AnimationPtr out(retarget.Retarget(anim.get()));
    ASSERT_NE(out, nullptr);

    EXPECT_EQ(sourceTransform, source.hips->mTransformation);
    EXPECT_EQ(targetTransform, target.hips->mTransformation);
    ASSERT_EQ(2u, anim->mChannels[0]->mNumRotationKeys);
    ExpectQuatNear(anim->mChannels[0]->mRotationKeys[1].mValue, RotationZ((ai_real)0.3), (ai_real)1e-6);
    ExpectVecNear(anim->mChannels[0]->mPositionKeys[1].mValue, aiVector3D(1, 1, 0), (ai_real)1e-6);
}
