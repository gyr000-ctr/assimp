/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2026, assimp team

All rights reserved.

Redistribution and use of this source code in source and binary forms,
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

/** @file  PolygonTriangulate.h
 *  @brief Shared, robust triangulation of arbitrary (possibly concave)
 *    polygons. Used by both the TriangulateProcess post-processing step
 *    and by exporters (e.g. glTF 2.0) so that the import and export
 *    stages make identical decisions for concave polygons and
 *    degenerate input.
 *
 *  The implementation projects the polygon onto the best-fit plane
 *  derived from its Newell normal and runs the mapbox earcut algorithm,
 *  which handles convex and concave polygons as well as clockwise and
 *  counter-clockwise winding. The following deterministic policies are
 *  applied:
 *
 *  - Repeated first/last vertices (closed loops) and consecutive
 *    duplicate vertices are removed before triangulation.
 *  - Collinear vertices are tolerated (handled by earcut); no
 *    zero-area triangle is ever emitted.
 *  - Fully degenerate faces (fewer than three non-collinear vertices
 *    after cleanup, or a Newell normal of zero length) are rejected:
 *    the caller skips the face and logs a warning. No geometry is
 *    produced for such faces.
 *  - Self-intersecting or non-planar input cannot be triangulated
 *    unambiguously. earcut still returns a triangle covering, but every
 *    emitted triangle is validated against the polygon's plane and a
 *    positive-area threshold; invalid triangles are dropped and the
 *    rejection is logged. If no valid triangle remains the face is
 *    treated as degenerate.
 *
 *  The helper never reorders or duplicates vertices: every emitted
 *  index refers to a position in the caller's original face index
 *  sequence, preserving vertex attributes, normals and material
 *  assignments.
 */
#pragma once
#ifndef AI_POLYGON_TRIANGULATE_H_INCLUDED
#define AI_POLYGON_TRIANGULATE_H_INCLUDED

#include "Common/PolyTools.h"
#include "contrib/earcut-hpp/earcut.hpp"

#include <assimp/defs.h>
#include <assimp/mesh.h>
#include <assimp/vector2.h>
#include <assimp/vector3.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace mapbox::util {

template <>
struct nth<0, aiVector2D> {
    inline static auto get(const aiVector2D &t) {
        return t.x;
    }
};
template <>
struct nth<1, aiVector2D> {
    inline static auto get(const aiVector2D &t) {
        return t.y;
    }
};

} // namespace mapbox::util

namespace Assimp {
namespace PolygonTriangulation {

/// Outcome of a single-face triangulation attempt.
enum class Result {
    /// The face was triangulated successfully.
    Success,
    /// The face is degenerate (not enough non-collinear vertices,
    /// zero-area normal) and produced no triangles.
    Degenerate,
    /// The input could not be triangulated (e.g. earcut returned an
    /// inconsistent result for a self-intersecting polygon).
    Invalid
};

/// @brief Remove consecutive duplicate positions (including a repeated
///   first/last vertex) from a face index sequence.
/// @param vertices   Mesh vertex positions.
/// @param faceIndices Original face indices, length @p numIndices.
/// @param numIndices Number of entries in @p faceIndices.
/// @param outIndices Receives the cleaned index sequence (positions into
///   the original face sequence, i.e. values in [0, numIndices)).
/// @param epsilon    Distance below which two positions are considered
///   coincident.
inline void CleanupPolygonRing(const aiVector3D *vertices, const unsigned int *faceIndices,
        unsigned int numIndices, std::vector<unsigned int> &outIndices, ai_real tolerance) {
    outIndices.clear();
    outIndices.reserve(numIndices);

    const ai_real tolSquared = tolerance * tolerance;
    for (unsigned int i = 0; i < numIndices; ++i) {
        if (outIndices.empty()) {
            outIndices.push_back(i);
            continue;
        }
        const aiVector3D &a = vertices[faceIndices[outIndices.back()]];
        const aiVector3D &b = vertices[faceIndices[i]];
        if ((a - b).SquareLength() > tolSquared) {
            outIndices.push_back(i);
        }
    }

    // Drop a repeated first vertex at the end of the ring.
    while (outIndices.size() > 1) {
        const aiVector3D &a = vertices[faceIndices[outIndices.front()]];
        const aiVector3D &b = vertices[faceIndices[outIndices.back()]];
        if ((a - b).SquareLength() > tolSquared) {
            break;
        }
        outIndices.pop_back();
    }
}

/// @brief Triangulate a single polygon face.
///
/// The call is allocation-light apart from the output vectors: temporary
/// storage is bounded by a constant multiple of the face's index count,
/// and the same scratch containers can be reused across faces.
///
/// @param vertices      Mesh vertex positions (mVertices of the mesh).
/// @param faceIndices   Indices of the face to triangulate.
/// @param numIndices    Number of entries in @p faceIndices (>= 3).
/// @param outTriangles  Receives triples of indices. Each value is an
///   index into @p faceIndices, i.e. an index in [0, numIndices); the
///   caller maps it back to a mesh vertex through faceIndices. The
///   winding order matches the input polygon.
/// @param scratch3D     Scratch storage, reused across calls.
/// @param scratch2D     Scratch storage, reused across calls.
/// @return Result code as documented on #Result. On anything other than
///   #Success, @p outTriangles is left empty.
inline Result TriangulateFace(const aiVector3D *vertices, const unsigned int *faceIndices,
        unsigned int numIndices, std::vector<unsigned int> &outTriangles,
        std::vector<aiVector3D> &scratch3D, std::vector<std::vector<aiVector2D>> &scratch2D) {
    outTriangles.clear();

    if (numIndices < 3) {
        return Result::Degenerate;
    }

    // Determine a scale-relative coincidence tolerance from the face's
    // position bounds so the behavior is independent of the model's units.
    aiVector3D faceMin(vertices[faceIndices[0]]);
    aiVector3D faceMax(vertices[faceIndices[0]]);
    for (unsigned int i = 1; i < numIndices; ++i) {
        const aiVector3D &v = vertices[faceIndices[i]];
        faceMin.x = std::min(faceMin.x, v.x);
        faceMin.y = std::min(faceMin.y, v.y);
        faceMin.z = std::min(faceMin.z, v.z);
        faceMax.x = std::max(faceMax.x, v.x);
        faceMax.y = std::max(faceMax.y, v.y);
        faceMax.z = std::max(faceMax.z, v.z);
    }
    const ai_real faceScale = (faceMax - faceMin).Length();
    const ai_real tolerance = std::max<ai_real>(ai_epsilon, faceScale * ai_epsilon);

    std::vector<unsigned int> ring;
    CleanupPolygonRing(vertices, faceIndices, numIndices, ring, tolerance);
    const unsigned int num = static_cast<unsigned int>(ring.size());
    if (num < 3) {
        return Result::Degenerate;
    }

    // Collect the cleaned polygon positions. NewellNormal duplicates the
    // first two entries at the end, so reserve two extra slots.
    scratch3D.resize(static_cast<size_t>(num) + 2);
    for (unsigned int i = 0; i < num; ++i) {
        scratch3D[i] = vertices[faceIndices[ring[i]]];
    }

    aiVector3D normal;
    NewellNormal<3, 3, 3>(normal, static_cast<int>(num), &scratch3D.front().x, &scratch3D.front().y,
            &scratch3D.front().z);

    // A zero-length Newell normal means all points are collinear (or the
    // face has zero area): there is no plane to triangulate on.
    if (normal.SquareLength() <= tolerance * tolerance) {
        return Result::Degenerate;
    }

    // Select the largest normal component to ignore for the 2D projection.
    const ai_real ax = normal.x > 0 ? normal.x : -normal.x;
    const ai_real ay = normal.y > 0 ? normal.y : -normal.y;
    const ai_real az = normal.z > 0 ? normal.z : -normal.z;

    unsigned int ac = 0, bc = 1; // drop z: project onto xy
    ai_real inv = normal.z;
    if (ax > ay) {
        if (ax > az) { // drop x: project onto yz
            ac = 1;
            bc = 2;
            inv = normal.x;
        }
    } else if (ay > az) { // drop y: project onto zx
        ac = 2;
        bc = 0;
        inv = normal.y;
    }

    // Swap the projection axes for a negated projection vector so that
    // the 2D winding keeps the orientation of the 3D polygon.
    if (inv < 0.0f) {
        std::swap(ac, bc);
    }

    if (scratch2D.empty()) {
        scratch2D.resize(1);
    }
    std::vector<aiVector2D> &projected = scratch2D[0];
    projected.resize(num);
    ai_real minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < num; ++i) {
        projected[i].x = scratch3D[i][ac];
        projected[i].y = scratch3D[i][bc];
        if (i == 0) {
            minX = maxX = projected[i].x;
            minY = maxY = projected[i].y;
        } else {
            minX = std::min(minX, projected[i].x);
            maxX = std::max(maxX, projected[i].x);
            minY = std::min(minY, projected[i].y);
            maxY = std::max(maxY, projected[i].y);
        }
    }

    const std::vector<uint32_t> earIndices = mapbox::earcut<uint32_t>(scratch2D);
    if (earIndices.size() < 3 || earIndices.size() % 3 != 0) {
        return Result::Invalid;
    }

    // Validate every produced triangle. A positive 2D area proves both
    // that the triangle is non-degenerate on the polygon plane and that
    // its winding is consistent with the input ring. The threshold is
    // relative to the projected polygon's size so it is unit-independent.
    const ai_real projectedScaleSquared =
            (maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY);
    const double areaThreshold = static_cast<double>(ai_epsilon) *
            static_cast<double>(std::max<ai_real>(ai_epsilon, std::sqrt(std::max<ai_real>(projectedScaleSquared, ai_epsilon))));
    outTriangles.reserve(earIndices.size());
    for (size_t i = 0; i + 2 < earIndices.size(); i += 3) {
        const uint32_t i0 = earIndices[i];
        const uint32_t i1 = earIndices[i + 1];
        const uint32_t i2 = earIndices[i + 2];
        if (i0 >= num || i1 >= num || i2 >= num) {
            outTriangles.clear();
            return Result::Invalid;
        }

        const double area = GetArea2D(projected[i0], projected[i1], projected[i2]);
        if (std::abs(area) <= areaThreshold) {
            // Zero-area triangle resulting from collinear points or a
            // self-intersecting polygon: skip it rather than emitting
            // broken geometry.
            continue;
        }

        outTriangles.push_back(ring[i0]);
        outTriangles.push_back(ring[i1]);
        outTriangles.push_back(ring[i2]);
    }

    if (outTriangles.empty()) {
        return Result::Degenerate;
    }
    return Result::Success;
}

} // namespace PolygonTriangulation
} // namespace Assimp

#endif // AI_POLYGON_TRIANGULATE_H_INCLUDED
