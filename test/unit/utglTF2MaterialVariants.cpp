/*
---------------------------------------------------------------------------
Open Asset Import Library (assimp)
---------------------------------------------------------------------------

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

/** @file utglTF2MaterialVariants.cpp
 *  @brief Tests for KHR_materials_variants import, variant switching and
 *         import -> export -> import round-trips.
 */

#include "UnitTestPCH.h"

#include "AssetLib/glTF2/glTF2MaterialVariants.h"

#include <assimp/Exporter.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <fstream>
#include <sstream>
#include <string>

using namespace Assimp;

namespace {

const char *const kModelDir = ASSIMP_TEST_MODELS_DIR "/glTF2/KHR_materials_variants";

void expectVariantNames(const aiScene *scene, unsigned int count, const char *name0, const char *name1) {
    ASSERT_NE(nullptr, scene);
    ASSERT_EQ(count, glTF2Variants::GetMaterialVariantCount(scene));
    aiString name;
    ASSERT_TRUE(glTF2Variants::GetMaterialVariantName(scene, 0, name));
    EXPECT_STREQ(name0, name.C_Str());
    if (count > 1) {
        ASSERT_TRUE(glTF2Variants::GetMaterialVariantName(scene, 1, name));
        EXPECT_STREQ(name1, name.C_Str());
    }
}

void expectVariantMaterial(const aiScene *scene, unsigned int mesh, unsigned int variant, unsigned int expected) {
    unsigned int materialIndex = 0;
    ASSERT_TRUE(glTF2Variants::GetMeshVariantMaterialIndex(scene, mesh, variant, materialIndex))
            << "mesh " << mesh << " variant " << variant;
    EXPECT_EQ(expected, materialIndex);
}

} // namespace

class utglTF2MaterialVariants : public ::testing::Test {};

TEST_F(utglTF2MaterialVariants, importSinglePrimitive) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(std::string(kModelDir) + "/VariantsSinglePrimitive.gltf", aiProcess_ValidateDataStructure);
    ASSERT_NE(nullptr, scene);
    ASSERT_EQ(1u, scene->mNumMeshes);
    // 3 materials from the file plus the default material the importer appends
    ASSERT_EQ(4u, scene->mNumMaterials);

    // Variant enumeration
    expectVariantNames(scene, 2, "red", "green");
    EXPECT_EQ(0, glTF2Variants::FindMaterialVariant(scene, "red"));
    EXPECT_EQ(1, glTF2Variants::FindMaterialVariant(scene, "green"));
    EXPECT_EQ(-1, glTF2Variants::FindMaterialVariant(scene, "blue"));

    // The default material stays in the mesh material index
    EXPECT_EQ(0u, scene->mMeshes[0]->mMaterialIndex);

    // Variant -> material mapping
    expectVariantMaterial(scene, 0, 0, 1);
    expectVariantMaterial(scene, 0, 1, 2);
    unsigned int dummy = 0;
    EXPECT_FALSE(glTF2Variants::GetMeshVariantMaterialIndex(scene, 0, 2, dummy));
}

TEST_F(utglTF2MaterialVariants, switchVariantsByNameAndIndex) {
    Assimp::Importer importer;
    aiScene *scene = const_cast<aiScene *>(importer.ReadFile(std::string(kModelDir) + "/VariantsSinglePrimitive.gltf", aiProcess_ValidateDataStructure));
    ASSERT_NE(nullptr, scene);

    // Switch by name
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, "green"));
    EXPECT_EQ(2u, scene->mMeshes[0]->mMaterialIndex);

    // Switch by index
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, 0u));
    EXPECT_EQ(1u, scene->mMeshes[0]->mMaterialIndex);

    // Unknown variants must not touch the scene
    EXPECT_FALSE(glTF2Variants::ApplyMaterialVariant(scene, "does_not_exist"));
    EXPECT_EQ(1u, scene->mMeshes[0]->mMaterialIndex);
    EXPECT_FALSE(glTF2Variants::ApplyMaterialVariant(scene, 42u));
    EXPECT_EQ(1u, scene->mMeshes[0]->mMaterialIndex);
}

TEST_F(utglTF2MaterialVariants, importMultiPrimitiveSharedMaterials) {
    Assimp::Importer importer;
    aiScene *scene = const_cast<aiScene *>(importer.ReadFile(std::string(kModelDir) + "/VariantsMultiPrimitive.gltf", aiProcess_ValidateDataStructure));
    ASSERT_NE(nullptr, scene);
    ASSERT_EQ(2u, scene->mNumMeshes);
    // 3 materials from the file plus the default material the importer appends
    ASSERT_EQ(4u, scene->mNumMaterials);

    expectVariantNames(scene, 2, "chrome", "matte");

    // Default materials of both primitives
    EXPECT_EQ(0u, scene->mMeshes[0]->mMaterialIndex);
    EXPECT_EQ(1u, scene->mMeshes[1]->mMaterialIndex);

    // Materials are shared between variants and defaults, not duplicated
    expectVariantMaterial(scene, 0, 0, 1);
    expectVariantMaterial(scene, 0, 1, 2);
    expectVariantMaterial(scene, 1, 0, 0);
    unsigned int dummy = 0;
    EXPECT_FALSE(glTF2Variants::GetMeshVariantMaterialIndex(scene, 1, 1, dummy));

    // Switching to "chrome" remaps both meshes
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, "chrome"));
    EXPECT_EQ(1u, scene->mMeshes[0]->mMaterialIndex);
    EXPECT_EQ(0u, scene->mMeshes[1]->mMaterialIndex);

    // "matte" has no mapping for mesh 1: it falls back to its default material
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, "matte"));
    EXPECT_EQ(2u, scene->mMeshes[0]->mMaterialIndex);
    EXPECT_EQ(1u, scene->mMeshes[1]->mMaterialIndex);

    // Switching back restores the mapped materials
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, 0u));
    EXPECT_EQ(1u, scene->mMeshes[0]->mMaterialIndex);
    EXPECT_EQ(0u, scene->mMeshes[1]->mMaterialIndex);
}

TEST_F(utglTF2MaterialVariants, importMissingDefaultMaterial) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(std::string(kModelDir) + "/VariantsNoDefaultMaterial.gltf", aiProcess_ValidateDataStructure);
    ASSERT_NE(nullptr, scene);
    ASSERT_EQ(1u, scene->mNumMeshes);
    // 1 material from the file plus the default material the importer appends
    ASSERT_EQ(2u, scene->mNumMaterials);

    ASSERT_EQ(1u, glTF2Variants::GetMaterialVariantCount(scene));
    // Safe fallback: the appended default material is used as the default
    EXPECT_EQ(1u, scene->mMeshes[0]->mMaterialIndex);
    expectVariantMaterial(scene, 0, 0, 0);
}

TEST_F(utglTF2MaterialVariants, importOutOfRangeMappingEntries) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(std::string(kModelDir) + "/VariantsOutOfRange.gltf", aiProcess_ValidateDataStructure);
    // The invalid entries must not fail the import
    ASSERT_NE(nullptr, scene);
    ASSERT_EQ(1u, scene->mNumMeshes);
    // 2 materials from the file plus the default material the importer appends
    ASSERT_EQ(3u, scene->mNumMaterials);

    expectVariantNames(scene, 2, "ok", "broken");

    // The valid mapping survived
    expectVariantMaterial(scene, 0, 0, 1);

    // Material index 7 (out of range) and variant index 9 (out of range) were dropped
    unsigned int dummy = 0;
    EXPECT_FALSE(glTF2Variants::GetMeshVariantMaterialIndex(scene, 0, 1, dummy));

    // Switching to the unmapped variant falls back to the default material
    aiScene *mutableScene = const_cast<aiScene *>(scene);
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(mutableScene, 1u));
    EXPECT_EQ(0u, mutableScene->mMeshes[0]->mMaterialIndex);
}

TEST_F(utglTF2MaterialVariants, importDuplicateVariantNames) {
    Assimp::Importer importer;
    aiScene *scene = const_cast<aiScene *>(importer.ReadFile(std::string(kModelDir) + "/VariantsDuplicateNames.gltf", aiProcess_ValidateDataStructure));
    ASSERT_NE(nullptr, scene);

    // All declared variants are kept so that variant indices stay stable
    ASSERT_EQ(3u, glTF2Variants::GetMaterialVariantCount(scene));
    aiString name;
    ASSERT_TRUE(glTF2Variants::GetMaterialVariantName(scene, 0, name));
    EXPECT_STREQ("twin", name.C_Str());
    ASSERT_TRUE(glTF2Variants::GetMaterialVariantName(scene, 1, name));
    EXPECT_STREQ("twin", name.C_Str());
    ASSERT_TRUE(glTF2Variants::GetMaterialVariantName(scene, 2, name));
    EXPECT_STREQ("other", name.C_Str());

    // Name lookups resolve to the first occurrence
    EXPECT_EQ(0, glTF2Variants::FindMaterialVariant(scene, "twin"));
    EXPECT_EQ(2, glTF2Variants::FindMaterialVariant(scene, "other"));

    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, "twin"));
    EXPECT_EQ(1u, scene->mMeshes[0]->mMaterialIndex);
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, 1u));
    EXPECT_EQ(2u, scene->mMeshes[0]->mMaterialIndex);
}

TEST_F(utglTF2MaterialVariants, importUndeclaredExtension) {
    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(std::string(kModelDir) + "/VariantsUndeclared.gltf", aiProcess_ValidateDataStructure);
    // Safe fallback: the extension data is kept even though it is not declared
    ASSERT_NE(nullptr, scene);
    ASSERT_EQ(1u, glTF2Variants::GetMaterialVariantCount(scene));
    aiString name;
    ASSERT_TRUE(glTF2Variants::GetMaterialVariantName(scene, 0, name));
    EXPECT_STREQ("undeclared_variant", name.C_Str());
    expectVariantMaterial(scene, 0, 0, 1);
}

#ifndef ASSIMP_BUILD_NO_EXPORT

TEST_F(utglTF2MaterialVariants, roundTripSinglePrimitive) {
    const std::string outFile = std::string(kModelDir) + "/VariantsSinglePrimitive_out.gltf";

    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(std::string(kModelDir) + "/VariantsSinglePrimitive.gltf", aiProcess_ValidateDataStructure);
    ASSERT_NE(nullptr, scene);

    Assimp::Exporter exporter;
    ASSERT_EQ(aiReturn_SUCCESS, exporter.Export(scene, "gltf2", outFile)) << exporter.GetErrorString();

    // The exported file declares the extension and carries the variant names
    std::ifstream in(outFile);
    ASSERT_TRUE(in.good());
    std::stringstream buffer;
    buffer << in.rdbuf();
    const std::string json = buffer.str();
    EXPECT_NE(std::string::npos, json.find("\"KHR_materials_variants\""));
    EXPECT_NE(std::string::npos, json.find("\"extensionsUsed\""));
    EXPECT_NE(std::string::npos, json.find("\"mappings\""));
    EXPECT_NE(std::string::npos, json.find("red"));
    EXPECT_NE(std::string::npos, json.find("green"));

    // Re-import and verify the mapping survived
    Assimp::Importer reImporter;
    const aiScene *roundTrip = reImporter.ReadFile(outFile, aiProcess_ValidateDataStructure);
    ASSERT_NE(nullptr, roundTrip) << reImporter.GetErrorString();
    ASSERT_EQ(1u, roundTrip->mNumMeshes);
    // The exporter must not duplicate materials shared by variants: every
    // referenced input material appears exactly once in the exported file.
    ASSERT_EQ(scene->mNumMaterials, roundTrip->mNumMaterials);

    expectVariantNames(roundTrip, 2, "red", "green");
    EXPECT_EQ(0u, roundTrip->mMeshes[0]->mMaterialIndex);
    expectVariantMaterial(roundTrip, 0, 0, 1);
    expectVariantMaterial(roundTrip, 0, 1, 2);
}

TEST_F(utglTF2MaterialVariants, roundTripMultiPrimitiveSharedMaterials) {
    const std::string outFile = std::string(kModelDir) + "/VariantsMultiPrimitive_out.glb";

    Assimp::Importer importer;
    const aiScene *scene = importer.ReadFile(std::string(kModelDir) + "/VariantsMultiPrimitive.gltf", aiProcess_ValidateDataStructure);
    ASSERT_NE(nullptr, scene);
    const unsigned int numMaterials = scene->mNumMaterials;

    Assimp::Exporter exporter;
    ASSERT_EQ(aiReturn_SUCCESS, exporter.Export(scene, "glb2", outFile)) << exporter.GetErrorString();

    Assimp::Importer reImporter;
    const aiScene *roundTrip = reImporter.ReadFile(outFile, aiProcess_ValidateDataStructure);
    ASSERT_NE(nullptr, roundTrip) << reImporter.GetErrorString();
    ASSERT_EQ(2u, roundTrip->mNumMeshes);

    // Materials shared by several variants must not be duplicated: every
    // referenced input material appears exactly once in the exported file.
    EXPECT_EQ(numMaterials, roundTrip->mNumMaterials);

    expectVariantNames(roundTrip, 2, "chrome", "matte");
    EXPECT_EQ(0u, roundTrip->mMeshes[0]->mMaterialIndex);
    EXPECT_EQ(1u, roundTrip->mMeshes[1]->mMaterialIndex);
    expectVariantMaterial(roundTrip, 0, 0, 1);
    expectVariantMaterial(roundTrip, 0, 1, 2);
    expectVariantMaterial(roundTrip, 1, 0, 0);
    unsigned int dummy = 0;
    EXPECT_FALSE(glTF2Variants::GetMeshVariantMaterialIndex(roundTrip, 1, 1, dummy));
}

TEST_F(utglTF2MaterialVariants, roundTripAfterVariantSwitch) {
    const std::string outFile = std::string(kModelDir) + "/VariantsSwitched_out.glb";

    Assimp::Importer importer;
    aiScene *scene = const_cast<aiScene *>(importer.ReadFile(std::string(kModelDir) + "/VariantsSinglePrimitive.gltf", aiProcess_ValidateDataStructure));
    ASSERT_NE(nullptr, scene);

    // Switch to a variant before exporting: the variant metadata must survive
    // and the active material becomes the new default.
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(scene, "green"));
    ASSERT_EQ(2u, scene->mMeshes[0]->mMaterialIndex);

    Assimp::Exporter exporter;
    ASSERT_EQ(aiReturn_SUCCESS, exporter.Export(scene, "glb2", outFile)) << exporter.GetErrorString();

    Assimp::Importer reImporter;
    aiScene *roundTrip = const_cast<aiScene *>(reImporter.ReadFile(outFile, aiProcess_ValidateDataStructure));
    ASSERT_NE(nullptr, roundTrip) << reImporter.GetErrorString();
    expectVariantNames(roundTrip, 2, "red", "green");

    // The re-import may re-index materials (only referenced materials are kept,
    // in reference order), so verify the mapping by material identity.
    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(roundTrip, "red"));
    aiString matName;
    ASSERT_LT(roundTrip->mMeshes[0]->mMaterialIndex, roundTrip->mNumMaterials);
    ASSERT_EQ(AI_SUCCESS, roundTrip->mMaterials[roundTrip->mMeshes[0]->mMaterialIndex]->Get(AI_MATKEY_NAME, matName));
    EXPECT_STREQ("red", matName.C_Str());

    ASSERT_TRUE(glTF2Variants::ApplyMaterialVariant(roundTrip, "green"));
    ASSERT_LT(roundTrip->mMeshes[0]->mMaterialIndex, roundTrip->mNumMaterials);
    ASSERT_EQ(AI_SUCCESS, roundTrip->mMaterials[roundTrip->mMeshes[0]->mMaterialIndex]->Get(AI_MATKEY_NAME, matName));
    EXPECT_STREQ("green", matName.C_Str());
}

#endif // ASSIMP_BUILD_NO_EXPORT
