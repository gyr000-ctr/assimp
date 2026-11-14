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

/** @file  TriangulateProcess.cpp
 *  @brief Implementation of the post processing step to split up
 *    all faces with more than three indices into triangles.
 *
 *
 *  The triangulation algorithm will handle concave or convex polygons.
 *  Self-intersecting or non-planar polygons are not rejected, but
 *  they're probably not triangulated correctly.
 *
 * DEBUG SWITCHES - do not enable any of them in release builds:
 *
 * AI_BUILD_TRIANGULATE_COLOR_FACE_WINDING
 *   - generates vertex colors to represent the face winding order.
 *     the first vertex of a polygon becomes red, the last blue.
 * AI_BUILD_TRIANGULATE_DEBUG_POLYS
 *   - dump all polygons and their triangulation sequences to
 *     a file
 */
#ifndef ASSIMP_BUILD_NO_TRIANGULATE_PROCESS

#include "PostProcessing/TriangulateProcess.h"
#include "PostProcessing/ProcessHelper.h"
#include "Common/PolyTools.h"
#include "Common/PolygonTriangulate.h"

#include <memory>
#include <cstdint>

//#define AI_BUILD_TRIANGULATE_COLOR_FACE_WINDING
//#define AI_BUILD_TRIANGULATE_DEBUG_POLYS

#define POLY_OUTPUT_FILE "assimp_polygons_debug.txt"

using namespace Assimp;

namespace {

    /**
     * @brief Helper struct used to simplify NGON encoding functions.
     */
    struct NGONEncoder {
        NGONEncoder() : mLastNGONFirstIndex((unsigned int)-1) {}

        /**
         * @brief Encode the current triangle, and make sure it is recognized as a triangle.
         *
         * This method will rotate indices in tri if needed in order to avoid tri to be considered
         * part of the previous ngon. This method is to be used whenever you want to emit a real triangle,
         * and make sure it is seen as a triangle.
         *
         * @param tri Triangle to encode.
         */
        void ngonEncodeTriangle(aiFace * tri) {
            ai_assert(tri->mNumIndices == 3);

            // Rotate indices in new triangle to avoid ngon encoding false ngons
            // Otherwise, the new triangle would be considered part of the previous NGON.
            if (isConsideredSameAsLastNgon(tri)) {
                std::swap(tri->mIndices[0], tri->mIndices[2]);
                std::swap(tri->mIndices[1], tri->mIndices[2]);
            }

            mLastNGONFirstIndex = tri->mIndices[0];
        }

        /**
         * @brief Encode a quad (2 triangles) in ngon encoding, and make sure they are seen as a single ngon.
         *
         * @param tri1 First quad triangle
         * @param tri2 Second quad triangle
         *
         * @pre Triangles must be properly fanned from the most appropriate vertex.
         */
        void ngonEncodeQuad(aiFace *tri1, aiFace *tri2) {
            ai_assert(tri1->mNumIndices == 3);
            ai_assert(tri2->mNumIndices == 3);
            ai_assert(tri1->mIndices[0] == tri2->mIndices[0]);

            // If the selected fanning vertex is the same as the previously
            // emitted ngon, we use the opposite vertex which also happens to work
            // for tri-fanning a concave quad.
            // ref: https://github.com/assimp/assimp/pull/3695#issuecomment-805999760
            if (isConsideredSameAsLastNgon(tri1)) {
                // Right-rotate indices for tri1 (index 2 becomes the new fanning vertex)
                std::swap(tri1->mIndices[0], tri1->mIndices[2]);
                std::swap(tri1->mIndices[1], tri1->mIndices[2]);

                // Left-rotate indices for tri2 (index 2 becomes the new fanning vertex)
                std::swap(tri2->mIndices[1], tri2->mIndices[2]);
                std::swap(tri2->mIndices[0], tri2->mIndices[2]);

                ai_assert(tri1->mIndices[0] == tri2->mIndices[0]);
            }

            mLastNGONFirstIndex = tri1->mIndices[0];
        }

        /**
         * @brief Check whether this triangle would be considered part of the lastly emitted ngon or not.
         *
         * @param tri Current triangle.
         * @return true If used as is, this triangle will be part of last ngon.
         * @return false If used as is, this triangle is not considered part of the last ngon.
         */
        bool isConsideredSameAsLastNgon(const aiFace * tri) const {
            ai_assert(tri->mNumIndices == 3);
            return tri->mIndices[0] == mLastNGONFirstIndex;
        }

    private:
        unsigned int mLastNGONFirstIndex;
    };

}

// ------------------------------------------------------------------------------------------------
// Returns whether the processing step is present in the given flag field.
bool TriangulateProcess::IsActive( unsigned int pFlags) const {
    return (pFlags & aiProcess_Triangulate) != 0;
}

// ------------------------------------------------------------------------------------------------
// Executes the post processing step on the given imported data.
void TriangulateProcess::Execute( aiScene* pScene) {
    ASSIMP_LOG_DEBUG("TriangulateProcess begin");

    bool bHas = false;
    for( unsigned int a = 0; a < pScene->mNumMeshes; a++)
    {
        if (pScene->mMeshes[ a ]) {
            if ( TriangulateMesh( pScene->mMeshes[ a ] ) ) {
                bHas = true;
            }
        }
    }
    if ( bHas ) {
        ASSIMP_LOG_INFO( "TriangulateProcess finished. All polygons have been triangulated." );
    } else {
        ASSIMP_LOG_DEBUG( "TriangulateProcess finished. There was nothing to be done." );
    }
}

// ------------------------------------------------------------------------------------------------
// Triangulates the given mesh.
bool TriangulateProcess::TriangulateMesh( aiMesh* pMesh) {
    // Now we have aiMesh::mPrimitiveTypes, so this is only here for test cases
    if (!pMesh->mPrimitiveTypes)    {
        bool bNeed = false;

        for( unsigned int a = 0; a < pMesh->mNumFaces; a++) {
            const aiFace& face = pMesh->mFaces[a];
            if( face.mNumIndices != 3)  {
                bNeed = true;
            }
        }
        if (!bNeed) {
            return false;
        }
    }
    else if (!(pMesh->mPrimitiveTypes & aiPrimitiveType_POLYGON)) {
        return false;
    }

    // Upper bound for the number of output faces. Degenerate polygon
    // faces may reduce the actual count, so the output array is sized for
    // the worst case.
    uint32_t numOut = 0;
    for( unsigned int a = 0; a < pMesh->mNumFaces; a++) {
        aiFace& face = pMesh->mFaces[a];
        if( face.mNumIndices <= 3) {
            ++numOut;
        } else {
            numOut += face.mNumIndices-2;
        }
    }

    // Just another check whether aiMesh::mPrimitiveTypes is correct:
    // if every face is already a point, line or triangle there is no
    // polygon work to do here.
    if (numOut == pMesh->mNumFaces) {
        ASSIMP_LOG_ERROR( "Invalidation detected in the number of indices: does not fit to the primitive type." );
        return false;
    }

    // the output mesh will contain triangles, but no polys anymore
    pMesh->mPrimitiveTypes |= aiPrimitiveType_TRIANGLE;
    pMesh->mPrimitiveTypes &= ~aiPrimitiveType_POLYGON;

    // The mesh becomes NGON encoded now, during the triangulation process.
    pMesh->mPrimitiveTypes |= aiPrimitiveType_NGONEncodingFlag;

    aiFace* out = new aiFace[numOut](), *curOut = out;
    std::vector<aiVector3D> temp_verts3d; /* temporary storage for the shared triangulator */
    std::vector<std::vector<aiVector2D>> temp_poly; /* temporary storage for earcut.hpp */
    std::vector<unsigned int> triIndices; /* triangulator output, indices into the face */

    NGONEncoder ngonEncoder;

    // Apply vertex colors to represent the face winding?
#ifdef AI_BUILD_TRIANGULATE_COLOR_FACE_WINDING
    if (!pMesh->mColors[0])
        pMesh->mColors[0] = new aiColor4D[pMesh->mNumVertices];
    else
        new(pMesh->mColors[0]) aiColor4D[pMesh->mNumVertices];

    aiColor4D* clr = pMesh->mColors[0];
#endif

#ifdef AI_BUILD_TRIANGULATE_DEBUG_POLYS
    FILE* fout = fopen(POLY_OUTPUT_FILE,"a");
#endif

    const aiVector3D* verts = pMesh->mVertices;

    for( unsigned int a = 0; a < pMesh->mNumFaces; a++) {
        aiFace& face = pMesh->mFaces[a];

        unsigned int* idx = face.mIndices;
        unsigned int num = face.mNumIndices;

        // Apply vertex colors to represent the face winding?
#ifdef AI_BUILD_TRIANGULATE_COLOR_FACE_WINDING
        for (unsigned int i = 0; i < face.mNumIndices; ++i) {
            aiColor4D& c = clr[idx[i]];
            c.r = (i+1) / static_cast<float>(face.mNumIndices);
            c.b = 1.f - c.r;
        }
#endif

        aiFace* const last_face = curOut;

        // if it's a simple point,line or triangle: just copy it
        if( face.mNumIndices <= 3)
        {
            aiFace& nface = *curOut++;
            nface.mNumIndices = face.mNumIndices;
            nface.mIndices    = face.mIndices;
            face.mIndices = nullptr;

            // points and lines don't require ngon encoding (and are not supported either!)
            if (nface.mNumIndices == 3) ngonEncoder.ngonEncodeTriangle(&nface);

            continue;
        }
        // A polygon with at least four vertices can be either concave or
        // convex, clockwise or counter-clockwise. Both quads and larger
        // n-gons go through the shared ear-cutting triangulator so that
        // the export path makes identical decisions (see
        // Common/PolygonTriangulate.h).
        const PolygonTriangulation::Result triResult = PolygonTriangulation::TriangulateFace(
                verts, idx, num, triIndices, temp_verts3d, temp_poly);
        if (triResult != PolygonTriangulation::Result::Success) {
            ASSIMP_LOG_WARN("TriangulateProcess: skipped a degenerate or invalid polygon face");
            delete[] face.mIndices;
            face.mIndices = nullptr;
            continue;
        }

        // Emit the triangles. The indices returned by the triangulator
        // refer to positions inside the face's own index sequence, so map
        // them back to mesh vertex indices below.
        for (size_t t = 0; t + 2 < triIndices.size(); t += 3) {
            aiFace &nface = *curOut++;
            nface.mIndices = new unsigned int[3];
            nface.mNumIndices = 3;
            nface.mIndices[0] = idx[triIndices[t]];
            nface.mIndices[1] = idx[triIndices[t + 1]];
            nface.mIndices[2] = idx[triIndices[t + 2]];
        }

        // prevent double deletion of the indices field
        face.mIndices = nullptr;

#ifdef AI_BUILD_TRIANGULATE_DEBUG_POLYS
        fprintf(fout, "triangulation sequence:");
        for (aiFace *f = last_face; f != curOut; ++f) {
            unsigned int *i = f->mIndices;
            fprintf(fout, " (%i %i %i)", i[0], i[1], i[2]);
        }
        fprintf(fout, "\n*********************************************************************\n");
        fflush(fout);
#endif

        // Encode the produced triangles with the ngon convention so that
        // consecutive faces can be reconstructed as polygons.
        for (aiFace *f = last_face; f != curOut; ++f) {
            ngonEncoder.ngonEncodeTriangle(f);
        }

        delete[] face.mIndices;
        face.mIndices = nullptr;
    }

#ifdef AI_BUILD_TRIANGULATE_DEBUG_POLYS
    fclose(fout);
#endif

    // kill the old faces
    delete [] pMesh->mFaces;

    // ... and store the new ones
    pMesh->mFaces    = out;
    pMesh->mNumFaces = (unsigned int)(curOut-out); /* not necessarily equal to numOut */
    return true;
}

#endif // !! ASSIMP_BUILD_NO_TRIANGULATE_PROCESS
