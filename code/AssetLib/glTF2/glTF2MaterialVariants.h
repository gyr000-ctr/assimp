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
following disclaimer in the documentation and/or other
materials provided with the distribution.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the
following disclaimer in the documentation and/or other
materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
contributors may be used to endorse or promote products
derived from this software without specific prior
written permission.

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

/** @file glTF2MaterialVariants.h
 *  Additional, opt-in API for the KHR_materials_variants glTF 2.0 extension.
 *
 *  The glTF2 importer keeps the default material of every primitive in
 *  aiMesh::mMaterialIndex, so existing callers see no behavior change. The
 *  variant data is exposed as aiScene::mMetaData entries:
 *
 *  - "KHR_materials_variants:count"                  (int32) number of declared variants
 *  - "KHR_materials_variants:name_<i>"               (aiString) name of variant i
 *  - "KHR_materials_variants:mesh_<m>:default"       (int32) default material index of mesh m
 *  - "KHR_materials_variants:mesh_<m>:material_<v>"  (int32) material index of mesh m for variant v
 *
 *  Meshes without a mapping for a variant simply have no such key; callers
 *  must keep the default material then. The glTF2 exporter reads the same
 *  keys back and restores the extension, so import -> export -> import
 *  round-trips preserve the variant mappings.
 *
 *  The inline helpers below implement enumeration and switching by name or
 *  index on top of these keys.
 */
#ifndef GLTF2_MATERIAL_VARIANTS_H_INC
#define GLTF2_MATERIAL_VARIANTS_H_INC

#include <assimp/metadata.h>
#include <assimp/scene.h>

#include <string>

namespace Assimp {
namespace glTF2Variants {

//! Metadata key prefix for all KHR_materials_variants entries.
inline const char *const MetadataPrefix = "KHR_materials_variants";

//! Metadata key (int32) holding the number of declared material variants.
inline std::string VariantCountKey() {
    return std::string(MetadataPrefix) + ":count";
}

//! Metadata key (aiString) holding the name of variant @p variantIndex.
inline std::string VariantNameKey(unsigned int variantIndex) {
    return std::string(MetadataPrefix) + ":name_" + std::to_string(variantIndex);
}

//! Metadata key (int32) holding the default material index of mesh @p meshIndex.
inline std::string MeshDefaultMaterialKey(unsigned int meshIndex) {
    return std::string(MetadataPrefix) + ":mesh_" + std::to_string(meshIndex) + ":default";
}

//! Metadata key (int32) holding the material index of mesh @p meshIndex for variant @p variantIndex.
inline std::string MeshVariantMaterialKey(unsigned int meshIndex, unsigned int variantIndex) {
    return std::string(MetadataPrefix) + ":mesh_" + std::to_string(meshIndex) + ":material_" + std::to_string(variantIndex);
}

//! Returns the number of material variants declared in the scene, 0 when the
//! extension is not present.
inline unsigned int GetMaterialVariantCount(const aiScene *scene) {
    if (nullptr == scene || nullptr == scene->mMetaData) {
        return 0;
    }
    int32_t count = 0;
    if (!scene->mMetaData->Get(VariantCountKey(), count) || count < 0) {
        return 0;
    }
    return static_cast<unsigned int>(count);
}

//! Returns the name of variant @p variantIndex, or false when there is no such variant.
inline bool GetMaterialVariantName(const aiScene *scene, unsigned int variantIndex, aiString &name) {
    if (variantIndex >= GetMaterialVariantCount(scene)) {
        return false;
    }
    return scene->mMetaData->Get(VariantNameKey(variantIndex), name);
}

//! Returns the index of the variant called @p name, or -1 when there is no such
//! variant. Duplicate variant names resolve to the first occurrence.
inline int FindMaterialVariant(const aiScene *scene, const char *name) {
    if (nullptr == scene || nullptr == name) {
        return -1;
    }
    const unsigned int count = GetMaterialVariantCount(scene);
    for (unsigned int i = 0; i < count; ++i) {
        aiString variantName;
        if (GetMaterialVariantName(scene, i, variantName) && variantName == aiString(name)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

//! Returns the material index that mesh @p meshIndex uses when variant
//! @p variantIndex is active, or false when the mesh has no mapping for this
//! variant (its default material applies then).
inline bool GetMeshVariantMaterialIndex(const aiScene *scene, unsigned int meshIndex, unsigned int variantIndex, unsigned int &materialIndex) {
    if (nullptr == scene || nullptr == scene->mMetaData || meshIndex >= scene->mNumMeshes) {
        return false;
    }
    int32_t index = -1;
    if (!scene->mMetaData->Get(MeshVariantMaterialKey(meshIndex, variantIndex), index) || index < 0) {
        return false;
    }
    materialIndex = static_cast<unsigned int>(index);
    return true;
}

//! Switches every mesh of the scene to the material of variant @p variantIndex.
//! Meshes without a mapping for this variant fall back to their default material.
//! The default material of a mesh is restored on later switches, so repeated
//! calls with different variants behave correctly.
//! @return false when the scene has no such variant; the scene is left unchanged then.
inline bool ApplyMaterialVariant(aiScene *scene, unsigned int variantIndex) {
    if (nullptr == scene || variantIndex >= GetMaterialVariantCount(scene)) {
        return false;
    }
    for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
        unsigned int materialIndex = 0;
        if (GetMeshVariantMaterialIndex(scene, m, variantIndex, materialIndex)) {
            if (materialIndex < scene->mNumMaterials) {
                scene->mMeshes[m]->mMaterialIndex = materialIndex;
            }
            continue;
        }
        // No mapping for this variant: restore the default material.
        int32_t defaultMaterial = -1;
        if (scene->mMetaData->Get(MeshDefaultMaterialKey(m), defaultMaterial) && defaultMaterial >= 0 &&
                static_cast<unsigned int>(defaultMaterial) < scene->mNumMaterials) {
            scene->mMeshes[m]->mMaterialIndex = static_cast<unsigned int>(defaultMaterial);
        }
    }
    return true;
}

//! Switches every mesh of the scene to the material of the variant called @p name.
//! @return false when there is no variant with this name; the scene is left unchanged then.
inline bool ApplyMaterialVariant(aiScene *scene, const char *name) {
    const int variantIndex = FindMaterialVariant(scene, name);
    if (variantIndex < 0) {
        return false;
    }
    return ApplyMaterialVariant(scene, static_cast<unsigned int>(variantIndex));
}

} // namespace glTF2Variants
} // namespace Assimp

#endif // GLTF2_MATERIAL_VARIANTS_H_INC
