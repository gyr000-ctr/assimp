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

#include <assimp/scene.h>

#include "PostProcessing/TriangulateProcess.h"

#include <cmath>
#include <set>
#include <tuple>
#include <vector>

using namespace std;
using namespace Assimp;

class TriangulateProcessTest : public ::testing::Test {
public:
    virtual void SetUp();
    virtual void TearDown();

protected:
    aiMesh *pcMesh;
    TriangulateProcess *piProcess;
};

void TriangulateProcessTest::SetUp() {
    piProcess = new TriangulateProcess();
    pcMesh = new aiMesh();

    pcMesh->mNumFaces = 1000;
    pcMesh->mFaces = new aiFace[1000];
    pcMesh->mVertices = new aiVector3D[10000];

    pcMesh->mPrimitiveTypes = aiPrimitiveType_POINT | aiPrimitiveType_LINE | aiPrimitiveType_POLYGON;

    for (unsigned int m = 0, t = 0, q = 4; m < 1000; ++m) {
        ++t;
        aiFace &face = pcMesh->mFaces[m];
        face.mNumIndices = t;
        if (4 == t) {
            face.mNumIndices = q++;
            t = 0;

            if (10 == q) q = 4;
        }
        face.mIndices = new unsigned int[face.mNumIndices];
        for (unsigned int p = 0; p < face.mNumIndices; ++p) {
            face.mIndices[p] = pcMesh->mNumVertices;

            // construct fully convex input data in ccw winding, xy plane
            aiVector3D &v = pcMesh->mVertices[pcMesh->mNumVertices++];
            v.z = 0.f;
            v.x = cos(p * (float)(AI_MATH_TWO_PI) / face.mNumIndices);
            v.y = sin(p * (float)(AI_MATH_TWO_PI) / face.mNumIndices);
        }
    }
}

void TriangulateProcessTest::TearDown() {
    delete piProcess;
    delete pcMesh;
}

TEST_F(TriangulateProcessTest, testTriangulation) {
    piProcess->TriangulateMesh(pcMesh);

    for (unsigned int m = 0, t = 0, q = 4, max = 1000, idx = 0; m < max; ++m) {
        ++t;
        aiFace &face = pcMesh->mFaces[m];
        if (4 == t) {
            t = 0;
            max += q - 3;

            std::vector<bool> ait(q, false);

            for (unsigned int i = 0, tt = q - 2; i < tt; ++i, ++m) {
                aiFace &curFace = pcMesh->mFaces[m];
                EXPECT_EQ(3U, curFace.mNumIndices);

                for (unsigned int qqq = 0; qqq < curFace.mNumIndices; ++qqq) {
                    ait[curFace.mIndices[qqq] - idx] = true;
                }
            }
            for (std::vector<bool>::const_iterator it = ait.begin(); it != ait.end(); ++it) {
                EXPECT_TRUE(*it);
            }
            --m;
            idx += q;
            if (++q == 10) {
                q = 4;
            }
        } else {
            EXPECT_EQ(t, face.mNumIndices);

            for (unsigned int i = 0; i < face.mNumIndices; ++i, ++idx) {
                EXPECT_EQ(idx, face.mIndices[i]);
            }
        }
    }

    // we should have no valid normal vectors now because we aren't a pure polygon mesh
    EXPECT_TRUE(pcMesh->mNormals == nullptr);
}

// ------------------------------------------------------------------------------------------------
// Tests for the shared polygon triangulation helper used by both
// TriangulateProcess and the glTF 2.0 exporter.
#include "Common/PolygonTriangulate.h"

namespace {

class PolygonTriangulationHelperTest : public ::testing::Test {
protected:
    std::vector<aiVector3D> scratch3D;
    std::vector<std::vector<aiVector2D>> scratch2D;
    std::vector<unsigned int> triangles;

    // Sum of unsigned triangle areas in the xy plane, used as a coverage
    // check against the analytic polygon area.
    double TriangleAreaSum(const std::vector<aiVector3D> &verts,
            const unsigned int *face, const std::vector<unsigned int> &tris) const {
        double sum = 0.0;
        for (size_t i = 0; i + 2 < tris.size(); i += 3) {
            const aiVector3D &a = verts[face[tris[i]]];
            const aiVector3D &b = verts[face[tris[i + 1]]];
            const aiVector3D &c = verts[face[tris[i + 2]]];
            sum += std::abs(0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)));
        }
        return sum;
    }
};

// Concave L-shaped polygon: ear cutting must succeed and cover the full
// polygon area.
TEST_F(PolygonTriangulationHelperTest, concavePolygon) {
    const std::vector<aiVector3D> verts = {
            {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {2.0f, 1.0f, 0.0f},
            {1.0f, 1.0f, 0.0f}, {1.0f, 2.0f, 0.0f}, {0.0f, 2.0f, 0.0f}};
    const unsigned int face[] = {0, 1, 2, 3, 4, 5};

    const auto result = PolygonTriangulation::TriangulateFace(
            verts.data(), face, 6, triangles, scratch3D, scratch2D);
    ASSERT_EQ(PolygonTriangulation::Result::Success, result);
    ASSERT_EQ(12u, triangles.size()); // 6 unique corners -> 4 triangles

    std::vector<bool> used(6, false);
    for (unsigned int idx : triangles) {
        ASSERT_LT(idx, 6u);
        used[idx] = true;
    }
    for (bool u : used) {
        EXPECT_TRUE(u);
    }
    // L-shape area = 3.
    EXPECT_NEAR(3.0, TriangleAreaSum(verts, face, triangles), 1e-9);
}

// Same polygon, reversed winding: same coverage, opposite orientation.
TEST_F(PolygonTriangulationHelperTest, windingOrderIsPreserved) {
    const std::vector<aiVector3D> verts = {
            {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {2.0f, 1.0f, 0.0f},
            {1.0f, 1.0f, 0.0f}, {1.0f, 2.0f, 0.0f}, {0.0f, 2.0f, 0.0f}};
    const unsigned int ccw[] = {0, 1, 2, 3, 4, 5};
    const unsigned int cw[] = {5, 4, 3, 2, 1, 0};

    std::vector<unsigned int> ccwTris, cwTris;
    ASSERT_EQ(PolygonTriangulation::Result::Success,
            PolygonTriangulation::TriangulateFace(verts.data(), ccw, 6, ccwTris, scratch3D, scratch2D));
    ASSERT_EQ(PolygonTriangulation::Result::Success,
            PolygonTriangulation::TriangulateFace(verts.data(), cw, 6, cwTris, scratch3D, scratch2D));

    ASSERT_EQ(ccwTris.size(), cwTris.size());
    EXPECT_NEAR(TriangleAreaSum(verts, ccw, ccwTris), TriangleAreaSum(verts, cw, cwTris), 1e-9);

    auto firstSignedArea = [&](const unsigned int *face, const std::vector<unsigned int> &tris) {
        const aiVector3D &a = verts[face[tris[0]]];
        const aiVector3D &b = verts[face[tris[1]]];
        const aiVector3D &c = verts[face[tris[2]]];
        return 0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
    };
    EXPECT_GT(firstSignedArea(ccw, ccwTris), 0.0);
    EXPECT_LT(firstSignedArea(cw, cwTris), 0.0);
}

// Repeated first/last vertex and consecutive duplicates are removed.
TEST_F(PolygonTriangulationHelperTest, repeatedEndpoints) {
    const std::vector<aiVector3D> verts = {
            {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    const unsigned int face[] = {0, 0, 1, 2, 3, 4};

    const auto result = PolygonTriangulation::TriangulateFace(
            verts.data(), face, 6, triangles, scratch3D, scratch2D);
    ASSERT_EQ(PolygonTriangulation::Result::Success, result);
    EXPECT_EQ(6u, triangles.size()); // quad -> 2 triangles
    EXPECT_NEAR(1.0, TriangleAreaSum(verts, face, triangles), 1e-9);
}

// Collinear points do not create zero-area triangles.
TEST_F(PolygonTriangulationHelperTest, collinearPoints) {
    const std::vector<aiVector3D> verts = {
            {0.0f, 0.0f, 0.0f}, {0.5f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
            {1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    const unsigned int face[] = {0, 1, 2, 3, 4};

    const auto result = PolygonTriangulation::TriangulateFace(
            verts.data(), face, 5, triangles, scratch3D, scratch2D);
    ASSERT_EQ(PolygonTriangulation::Result::Success, result);
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        const aiVector3D &a = verts[face[triangles[i]]];
        const aiVector3D &b = verts[face[triangles[i + 1]]];
        const aiVector3D &c = verts[face[triangles[i + 2]]];
        const double area = std::abs(0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)));
        EXPECT_GT(area, 1e-10);
    }
    EXPECT_NEAR(1.0, TriangleAreaSum(verts, face, triangles), 1e-9);
}

// Fully collinear and duplicate-only input is rejected, not triangulated.
TEST_F(PolygonTriangulationHelperTest, degenerateInputIsRejected) {
    {
        const std::vector<aiVector3D> collinear = {
                {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {3.0f, 0.0f, 0.0f}};
        const unsigned int face[] = {0, 1, 2, 3};
        EXPECT_EQ(PolygonTriangulation::Result::Degenerate,
                PolygonTriangulation::TriangulateFace(collinear.data(), face, 4, triangles, scratch3D, scratch2D));
        EXPECT_TRUE(triangles.empty());
    }
    {
        const std::vector<aiVector3D> duplicates = {
                {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}, {1.0f, 1.0f, 1.0f}};
        const unsigned int face[] = {0, 1, 2};
        EXPECT_EQ(PolygonTriangulation::Result::Degenerate,
                PolygonTriangulation::TriangulateFace(duplicates.data(), face, 3, triangles, scratch3D, scratch2D));
        EXPECT_TRUE(triangles.empty());
    }
}

// Running TriangulateProcess on a concave mesh yields exactly the same
// triangle topology as the exporter's triangulation of the same face.
TEST_F(PolygonTriangulationHelperTest, matchesTriangulateProcess) {
    const std::vector<aiVector3D> verts = {
            {0.0f, 0.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {2.0f, 1.0f, 0.0f},
            {1.0f, 1.0f, 0.0f}, {1.0f, 2.0f, 0.0f}, {0.0f, 2.0f, 0.0f}};

    // Reference: direct helper call.
    const unsigned int faceIndices[] = {0, 1, 2, 3, 4, 5};
    const auto direct = PolygonTriangulation::TriangulateFace(
            verts.data(), faceIndices, 6, triangles, scratch3D, scratch2D);
    ASSERT_EQ(PolygonTriangulation::Result::Success, direct);
    const std::vector<unsigned int> reference = triangles;

    // Same input through the post-processing step.
    aiMesh mesh;
    mesh.mNumVertices = static_cast<unsigned int>(verts.size());
    mesh.mVertices = new aiVector3D[verts.size()];
    for (size_t i = 0; i < verts.size(); ++i) {
        mesh.mVertices[i] = verts[i];
    }
    mesh.mNumFaces = 1;
    mesh.mFaces = new aiFace[1];
    mesh.mFaces[0].mNumIndices = 6;
    mesh.mFaces[0].mIndices = new unsigned int[6]{0, 1, 2, 3, 4, 5};
    mesh.mPrimitiveTypes = aiPrimitiveType_POLYGON;

    TriangulateProcess process;
    ASSERT_TRUE(process.TriangulateMesh(&mesh));
    ASSERT_EQ(reference.size() / 3, static_cast<size_t>(mesh.mNumFaces));

    // NGON encoding may cyclically rotate individual triangles, so
    // compare unordered triangle sets normalized by cyclic rotation.
    auto rotateMin = [](unsigned int a, unsigned int b, unsigned int c) {
        std::tuple<unsigned int, unsigned int, unsigned int> t0 = {a, b, c};
        std::tuple<unsigned int, unsigned int, unsigned int> t1 = {b, c, a};
        std::tuple<unsigned int, unsigned int, unsigned int> t2 = {c, a, b};
        return std::min(t0, std::min(t1, t2));
    };
    std::set<std::tuple<unsigned int, unsigned int, unsigned int>> expected, actual;
    for (size_t f = 0; f < reference.size(); f += 3) {
        expected.insert(rotateMin(faceIndices[reference[f]], faceIndices[reference[f + 1]],
                faceIndices[reference[f + 2]]));
    }
    for (unsigned int f = 0; f < mesh.mNumFaces; ++f) {
        ASSERT_EQ(3u, mesh.mFaces[f].mNumIndices);
        actual.insert(rotateMin(mesh.mFaces[f].mIndices[0], mesh.mFaces[f].mIndices[1],
                mesh.mFaces[f].mIndices[2]));
    }
    EXPECT_EQ(expected, actual);
}

} // namespace

// Slightly non-planar polygon: triangulation must still succeed and
// every produced triangle must have genuine 3D area.
TEST_F(PolygonTriangulationHelperTest, nonPlanarPolygon) {
    const std::vector<aiVector3D> verts = {
            {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {1.0f, 1.0f, 0.1f},
            {0.5f, 1.2f, -0.1f}, {0.0f, 1.0f, 0.0f}};
    const unsigned int face[] = {0, 1, 2, 3, 4};

    const auto result = PolygonTriangulation::TriangulateFace(
            verts.data(), face, 5, triangles, scratch3D, scratch2D);
    ASSERT_EQ(PolygonTriangulation::Result::Success, result);
    ASSERT_FALSE(triangles.empty());
    for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
        const aiVector3D &a = verts[face[triangles[i]]];
        const aiVector3D &b = verts[face[triangles[i + 1]]];
        const aiVector3D &c = verts[face[triangles[i + 2]]];
        const double area3D = 0.5 * ((b - a) ^ (c - a)).Length();
        EXPECT_GT(area3D, 1e-10);
    }
}

// A self-intersecting (bowtie) polygon must not crash and must never
// produce zero-area output; it is either triangulated into valid
// triangles or rejected outright.
TEST_F(PolygonTriangulationHelperTest, selfIntersectingPolygon) {
    const std::vector<aiVector3D> verts = {
            {0.0f, 0.0f, 0.0f}, {2.0f, 2.0f, 0.0f}, {2.0f, 0.0f, 0.0f}, {0.0f, 2.0f, 0.0f}};
    const unsigned int face[] = {0, 1, 2, 3};

    const auto result = PolygonTriangulation::TriangulateFace(
            verts.data(), face, 4, triangles, scratch3D, scratch2D);
    if (result == PolygonTriangulation::Result::Success) {
        for (size_t i = 0; i + 2 < triangles.size(); i += 3) {
            const aiVector3D &a = verts[face[triangles[i]]];
            const aiVector3D &b = verts[face[triangles[i + 1]]];
            const aiVector3D &c = verts[face[triangles[i + 2]]];
            const double area3D = 0.5 * ((b - a) ^ (c - a)).Length();
            EXPECT_GT(area3D, 1e-10);
        }
    } else {
        EXPECT_TRUE(triangles.empty());
    }
}
