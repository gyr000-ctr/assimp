# Animation Retargeting

Assimp imports animations as plain tracks of key frames (`aiAnimation` with
`aiNodeAnim` channels). It does **not** change this: imported animations are
kept exactly as the source file describes them. Retargeting an animation from
one rig onto another (different proportions, hierarchy or bone names) is an
explicit, opt-in step provided by `Assimp::AnimationRetarget`
(`code/Common/AnimationRetarget.h`).

## Overview

`AnimationRetarget` maps key frames from a source skeleton onto a target
skeleton:

- **Bone mapping** — explicit (`RetargetOptions::mBoneMapping`,
  source name -> target name) or automatic by identical node names.
- **Rest-pose normalization** — rotations are retargeted in world space
  relative to the rest pose: the target bone's world orientation differs from
  its rest orientation by the same delta as the source bone's. This makes the
  retarget independent of different rest orientations and hierarchy depths.
- **Scale normalization** — a global size ratio is derived from the mapped
  bone lengths and applied to root motion; non-uniform rest scaling is
  handled per component. Scale keys are retargeted relative to the rest
  scale.
- **Root motion** — the translation of the root bone (explicit via
  `mRootBoneName` or auto-detected as the mapped bone closest to the
  skeleton root) is retargeted and scaled; it can be disabled
  (`mEnableRootMotion = false`) for in-place animations. All other bones
  keep their rest translation, because bone lengths differ between rigs.
- **Missing bones** — source channels without a mapped target bone are
  skipped (the target bone keeps its rest pose) and counted in the metrics.
- **Missing channels** — components without keys (position/rotation/scale)
  are filled from the target rest pose.
- **Quaternion sign** — rotation tracks are kept in one hemisphere so
  consecutive keys never jump to the antipodal representation.
- **Frame rates** — set `mTargetTicksPerSecond` to resample all channels
  uniformly onto a different tick rate; otherwise the source key times are
  kept.
- **Loop closure** — if the source animation loops (first key == last key),
  the retargeted animation is snapped so the loop closes exactly
  (`mEnforceLoopClosure`).

## Usage

```cpp
#include "Common/AnimationRetarget.h"

Assimp::RetargetOptions options;
options.mRootBoneName = "Hips";            // optional, auto-detected otherwise
options.mBoneMapping["Hips"] = "Pelvis";   // optional, auto-by-name otherwise
options.mTargetTicksPerSecond = 60.0;      // optional frame-rate conversion

Assimp::AnimationRetarget retarget;
if (retarget.Build(sourceScene->mRootNode, targetScene->mRootNode, options)) {
    aiAnimation *retargeted = retarget.Retarget(sourceAnimation); // you own it
    const Assimp::RetargetMetrics &metrics = retarget.GetMetrics();
    // ... use retargeted ...
    delete retargeted;
}
```

`Build` reads the rest poses from both node graphs and caches a plan whose
size is O(number of mapped bones). `Retarget` then converts any number of
animations in a single pass over the key frames — the plan is never rebuilt
per frame, and neither the input animation nor the scene graphs are
modified.

## Failure handling and fallback

Retargeting never returns a distorted animation when the input is unusable:

- If `Build` fails (null skeletons, no mapped bones), `Retarget` returns an
  unmodified deep copy of the source animation.
- If the bone coverage of an animation drops below
  `RetargetOptions::mMinCoverage`, the source animation is returned
  unchanged (global fallback).
- If a single channel produces non-finite values (degenerate rest data),
  that channel falls back to the original track data renamed to the target
  bone.

## Metrics

`RetargetMetrics` (via `GetMetrics()`) reports:

| Member | Meaning |
|---|---|
| `mCoverage` | Fraction of source channels mapped to the target rig |
| `mNumMappedChannels` / `mNumUnmappedChannels` | Channel counts |
| `mNumFallbackChannels` | Channels that fell back to source data |
| `mNumGlobalFallbacks` | Animations returned as unmodified copies |
| `mNumLoopClosures` | Channels whose loop closure was enforced |
| `mMeanScaleError` / `mMaxScaleError` | Relative deviation of per-bone size ratios from the global ratio (non-zero for non-uniform proportion differences) |

## Limitations

- Parent orientations are taken from the rest pose (the standard
  approximation for humanoid retargeting), not from the animated pose.
- Translation is retargeted for the root bone only; other bones keep their
  rest translation so the target rig's bone lengths are preserved.
- Mesh and morph animation channels are copied through unchanged.

## Tests

See `test/unit/utAnimationRetarget.cpp` for coverage of: identical
skeletons, different proportions, non-uniform scale, missing bones, root
motion on/off, quaternion sign continuity, frame-rate conversion, loop
closure, missing channels, explicit bone mapping and the fallback paths.
