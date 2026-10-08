#pragma once

#include "Core.h"
#include "Math.hpp"
#include "World.h"
#include <cstdio>
#include <cstring>
namespace ce {

static inline Vec3 TransformXByRotation(Quat q) {
    Quat res = q * Quat(q.x, q.w, -q.y, q.z);
    return { res.x, res.y, res.z };
}

static inline Vec3 TransformYByRotation(Quat q) {
    Quat res = q * Quat(q.y, -q.z, q.w, q.x);
    return { res.x, res.y, res.z };
}

static inline Vec3 TransformZByRotation(Quat q) {
    Quat res = q * Quat(q.z, q.y, -q.x, q.w);
    return { res.x, res.y, res.z };
}

/**
 * @todo Optimize this to use an adjacency list implementation. Should test if
 * that would make things faster, I imagine for the average case (convex hulls <
 * 50 vertices) we would not see a speed up compared to a SIMD implementation of
 * this traversal.
 */
inline Vec3 SupportPoint(ConvexHull *hull, Vec3 normal, u8 *outIdx) {
    u8 maxIdx = -1;
    Real maxVal = MIN_REAL;

    for (u32 i = 0; i < hull->NumVertices; ++i) {
        Real val = dot(hull->Vertices[i], normal);
        if (val > maxVal) {
            maxVal = val;
            maxIdx = i;
        }
    }
    *outIdx = maxIdx;
    return hull->Vertices[maxIdx];
}

inline Vec3 MinkowskiDiffSupportPoint(ConvexHull *hull1, ConvexHull *hull2, Transform relativeTransform, Vec3 normal,
                                      u16 *outIdx) {
    u8 tmpIdx;
    Vec3 v1 = SupportPoint(hull1, normal, &tmpIdx);
    *outIdx = tmpIdx;
    Vec3 v2 = SupportPoint(hull2, rotate(-normal, relativeTransform.Orientation), &tmpIdx);
    *outIdx |= static_cast<u16>(tmpIdx) << 8;
    return v1 - v2;
}

struct GJKData {
    Vec3 Simplex[4]{};
    u16 SimplexIds[4]{};
    u16 OldSimplexIds[4]{}; // Used for having a well defined termination condition
    Vec3 SearchDir;
    u8 NumOldSimplexPoints;
    u8 NumSimplexPoints;
    bool ShouldTerminate = false;
};

#define SET_SIMPLEX_1(point, data, idx)                                                                                \
    data->SearchDir = -point;                                                                                          \
    data->Simplex[0] = point;                                                                                          \
    data->SimplexIds[0] = data->OldSimplexIds[idx];                                                                       \
    data->NumSimplexPoints = 1;

#define SET_SIMPLEX_2(point1, point2, data, dir, p1Idx, p2Idx)                                                         \
    data->SearchDir = dir;                                                                                             \
    data->Simplex[0] = point1;                                                                                         \
    data->Simplex[1] = point2;                                                                                         \
    data->SimplexIds[0] = data->OldSimplexIds[p1Idx];                                                                     \
    data->SimplexIds[1] = data->OldSimplexIds[p2Idx];                                                                     \
    data->NumSimplexPoints = 2;

#define SET_SIMPLEX_3(point1, point2, point3, data, dir, p1Idx, p2Idx, p3Idx)                                          \
    data->SearchDir = dir;                                                                                             \
    data->Simplex[0] = point1;                                                                                         \
    data->Simplex[1] = point2;                                                                                         \
    data->Simplex[2] = point3;                                                                                         \
    data->SimplexIds[0] = data->OldSimplexIds[p1Idx];                                                                  \
    data->SimplexIds[1] = data->OldSimplexIds[p2Idx];                                                                  \
    data->SimplexIds[2] = data->OldSimplexIds[p3Idx];                                                                  \
    data->NumSimplexPoints = 3;

static void GJKGetCloestPointOnLine(Vec3 inA, Vec3 inB, GJKData *data) {
    // Recall; second point will always be the newly added point.
    Vec3 ab = inB - inA;
    Real abSqMag = dot(ab, ab);

    Real t = dot(ab, -inA);
    if (t <= 0) {
        // this should never occur!
        SET_SIMPLEX_1(inA, data, 0);
    } else if (t > abSqMag) {
        SET_SIMPLEX_1(inB, data, 1);
    } else {
        SET_SIMPLEX_2(inA, inB, data, cross(cross(ab, -inA), ab), 0, 1);
    }
}

// Should give a consistent orientation to what was provided.
static Vec3 GetTriangleNormal(Vec3 inA, Vec3 inB, Vec3 inC) {
    Vec3 ac = inA - inC;
    Vec3 bc = inB - inC;

    Real acSqMag = dot(ac, ac);
    Real bcSqMag = dot(bc, bc);

    if (acSqMag > bcSqMag) {
        return cross(bc, inB - inA);
    } else {
        return cross(ac, inB - inA);
    }
}

#define LINE_TO_ORIGIN(p, line) cross(cross(line, -p), line)

static void GJKGetClosestPointOnTriangle(Vec3 inA, Vec3 inB, Vec3 inC, GJKData *data) {

    Vec3 AC = inA - inC;
    Vec3 BC = inB - inC;
    Vec3 BA = inB - inA;
    Vec3 normal;
    // Use the shortest side, the normal should be in a consistent orientation
    if (dot(BC, BC) < dot(BA, BA)) {
        normal = cross(AC, BA);
    } else {
        normal = cross(AC, BC);
    }

    // Vertex C
    bool AC_C = dot(inC, AC) <= 0;
    bool BC_C = dot(inC, BC) <= 0;
    if (!AC_C && !BC_C) {
        SET_SIMPLEX_1(inC, data, 2);
        return;
    }

    // Vertex A
    bool AC_A = dot(inA, AC) >= 0;
    bool BA_A = dot(inA, BA) <= 0;
    if (!AC_A && !BA_A) {
        SET_SIMPLEX_1(inA, data, 0);
        return;
    }

    // Vertex B
    bool BA_B = dot(inB, BA) >= 0;
    bool BC_B = dot(inB, BC) >= 0;
    if (!BA_B && !BC_B) {
        SET_SIMPLEX_1(inB, data, 0);
        return;
    }

    // Edge CA
    Vec3 nAC = cross(AC, normal); // pointing out
    if (AC_C && AC_A && dot(inC, nAC) <= 0) {
        SET_SIMPLEX_2(inA, inC, data, LINE_TO_ORIGIN(inA, AC), 0, 2);
        return;
    }

    // Edge BC
    Vec3 nBC = cross(normal, BC);
    if (BC_B && BC_C && dot(inC, nBC) <= 0) {
        SET_SIMPLEX_2(inB, inC, data, LINE_TO_ORIGIN(inB, BC), 1, 2);
        return;
    }

    // Edge BA
    Vec3 nBA = cross(BA, normal);
    if (BA_A && BA_B && dot(inA, nBA) <= 0) {
        SET_SIMPLEX_2(inA, inB, data, LINE_TO_ORIGIN(inB, BA), 0, 1);
        return;
    }

    // Must be in the triangle face
    if (dot(inC, normal) >= 0) {
        normal = -normal;
    }
    SET_SIMPLEX_3(inA, inB, inC, data, normal, 0, 1, 2);
}

static void GJKGetClosestPointOnTetrahedron(Vec3 inA, Vec3 inB, Vec3 inC, Vec3 inD, GJKData *data) {

    // fix any winding issues
    Vec3 nABC = GetTriangleNormal(inA, inB, inC);
    if (dot(inD - inA, nABC) < 0) {
        // counter clockwise from D perspective, clockwise from outside, reverse
        std::swap(inB, inC);
        std::swap(data->OldSimplexIds[1], data->OldSimplexIds[2]);
        nABC = -nABC;
    }

    Vec3 DA = inA - inD;
    Vec3 DB = inB - inD;
    Vec3 DC = inC - inD;
    Vec3 AB = inB - inA;
    Vec3 AC = inC - inA;
    Vec3 BC = inC - inB;

    Vec3 nDBA = GetTriangleNormal(inD, inB, inA);
    Vec3 nDAC = GetTriangleNormal(inD, inA, inC);
    Vec3 nDCB = GetTriangleNormal(inD, inC, inB);

    // Avoid using lagrange rule, which can reduce the total number of dot
    // products to ~10 due to potential stability issues;
    // @todo Evaluate if stability issues are a problem with lagrange rule.
    // All values indicate whether it's within the vornoi region
    bool DA_D = dot(DA, inD) <= ZERO;
    bool DA_A = dot(DA, inA) >= ZERO;
    bool DB_D = dot(DB, inD) <= ZERO;
    bool DB_B = dot(DB, inB) >= ZERO;
    bool DC_D = dot(DC, inD) <= ZERO;
    bool DC_C = dot(DC, inC) >= ZERO;

    bool AB_A = dot(AB, inA) <= ZERO;
    bool AB_B = dot(AB, inB) >= ZERO;
    bool AC_A = dot(AC, inA) <= ZERO;
    bool AC_C = dot(AC, inC) >= ZERO;

    bool BC_B = dot(BC, inB) <= ZERO;
    bool BC_C = dot(BC, inC) >= ZERO;

    // Can optimize ifs by nesting, too lazy to do it
    if (!DA_D && !DB_D && !DC_D) {
        SET_SIMPLEX_1(inD, data, 3);
        return;
    }

    if (!DA_A && !AB_A && !AC_A) {
        SET_SIMPLEX_1(inA, data, 0);
        printf("DEBUG: inA selected as symplex \n");
        return;
    }

    if (!DB_B && !AB_B && !BC_B) {
        SET_SIMPLEX_1(inB, data, 1);
        printf("DEBUG: inB selected as symplex \n");
        return;
    }

    if (!DC_C && !AC_C && !BC_C) {
        SET_SIMPLEX_1(inC, data, 2);
        printf("DEBUG: inC selected as symplex \n");
        return;
    }

    // Edge DA
    bool DBA_DA = dot(cross(nDBA, DA), inD) >= ZERO;
    bool DAC_DA = dot(cross(nDAC, DA), inD) <= ZERO;
    if (DA_A && DA_D && !DBA_DA && !DAC_DA) {
        SET_SIMPLEX_2(inD, inA, data, LINE_TO_ORIGIN(inD, DA), 3, 0);
        return;
    }

    // Edge DB
    bool DBA_DB = dot(cross(nDBA, DB), inD) <= ZERO;
    bool DCB_DB = dot(cross(nDCB, DB), inD) >= ZERO;
    if (DB_B && DB_D && !DBA_DB && !DCB_DB) {
        SET_SIMPLEX_2(inD, inB, data, LINE_TO_ORIGIN(inD, DB), 3, 1);
        return;
    }

    // Edge DC
    bool DAC_DC = dot(cross(nDAC, DC), inD) >= ZERO;
    bool DCB_DC = dot(cross(nDCB, DC), inD) <= ZERO;
    if (DC_C && DC_D && !DAC_DC && !DCB_DC) {
        SET_SIMPLEX_2(inD, inC, data, LINE_TO_ORIGIN(inD, DC), 3, 2);
        return;
    }

    // Edge AB
    bool DBA_AB = dot(cross(nDBA, AB), inA) >= ZERO;
    bool ABC_AB = dot(cross(nABC, AB), inA) <= ZERO;
    if (AB_A && AB_B && !DBA_AB && !ABC_AB) {
        SET_SIMPLEX_2(inA, inB, data, LINE_TO_ORIGIN(inA, AB), 0, 1);
        return;
    }

    // Edge AC
    bool DAC_AC = dot(cross(nDAC, AC), inA) <= ZERO;
    bool ABC_AC = dot(cross(nABC, AC), inA) >= ZERO;
    if (AC_A && AC_C && !DAC_AC && !ABC_AC) {
        SET_SIMPLEX_2(inA, inC, data, LINE_TO_ORIGIN(inA, AC), 0, 2);
        return;
    }

    // Edge BC
    bool DCB_BC = dot(cross(nDCB, BC), inB) >= ZERO;
    bool ABC_BC = dot(cross(nABC, BC), inB) <= ZERO;
    if (BC_B && BC_C && !DCB_BC && !ABC_BC) {
        SET_SIMPLEX_2(inB, inC, data, LINE_TO_ORIGIN(inB, BC), 1, 2);
        return;
    }

    if (ABC_AB && ABC_AC && ABC_BC && dot(nABC, inA) >= ZERO) {
        SET_SIMPLEX_3(inA, inB, inC, data, -nABC, 0, 1, 2);
        return;
    }

    // Face DBA
    if (DBA_DA && DBA_AB && DBA_DB && dot(nDBA, inD) >= ZERO) {
        SET_SIMPLEX_3(inD, inB, inA, data, -nDBA, 3, 1, 0);
        return;
    }

    if (DAC_DA && DAC_AC && DAC_DC && dot(nDAC, inD) >= ZERO) {
        SET_SIMPLEX_3(inD, inA, inC, data, -nDAC, 3, 0, 2);
        return;
    }

    if (DCB_BC && DCB_DB && DCB_DC && dot(nDCB, inD) >= ZERO) {
        SET_SIMPLEX_3(inD, inC, inB, data, -nDCB, 3, 2, 1);
        return;
    }

    // No Face, must be inside the tetrahedron
    data->SearchDir = Vec3(ZERO);
    data->NumSimplexPoints = 4;
}

/**
 * One interation of the GJK loop. Given the current support point, the
 * iteration will update/simplify the simplex and compute the cloest point to
 * the origin. The caller is expected to call this function in a loop with their
 * own terminating condition and use the direction to get the new supporting
 * point each iteration.
 *
 *
 */
static void GJKIteration(GJKData *data) {
    // Used for termination condition before simplexes are simplified
    memcpy(data->OldSimplexIds, data->SimplexIds, sizeof(u16) * data->NumSimplexPoints);
    data->NumOldSimplexPoints = data->NumSimplexPoints;
    switch (data->NumSimplexPoints) {
    case 2:
        GJKGetCloestPointOnLine(data->Simplex[0], data->Simplex[1], data);
        break;
    case 3:
        GJKGetClosestPointOnTriangle(data->Simplex[0], data->Simplex[1], data->Simplex[2], data);
        break;
    case 4:
        GJKGetClosestPointOnTetrahedron(data->Simplex[0], data->Simplex[1], data->Simplex[2], data->Simplex[3], data);
        break;
    }
}

static Vec3 ClosestPointOnLine(Vec3 inA, Vec3 inB, Vec3 inP) {
    Vec3 BA = inB - inA;
    Real t = dot(inP - inA, BA) / dot(BA, BA);
    return inA + (BA * t);
}

static void GetOriginLineBarycentricCoordinates(Vec3 inA, Vec3 inB, Real *s, Real *t) {
    // Use barycentric coordinates
    Vec3 BA = inB - inA;
    Real aCoord = dot(-inA, BA);
    Real bCoord = dot(inB, BA);
    *s = bCoord / (aCoord + bCoord);
    *t = aCoord / (aCoord + bCoord);
}

static Vec3 ClosestPointOnPlane(Vec3 normal, Vec3 pointOnPlane, Vec3 inP) {
    return inP - (normal * dot(normal, inP - pointOnPlane));
}

static void GetOriginTriangleBarycentricCoordinates(Vec3 normal, Vec3 inA, Vec3 inB, Vec3 inC, Real *u, Real *v,
                                                    Real *w) {
    Real vc = dot(normal, cross(inA, inB));
    Real vb = dot(normal, cross(inC, inA));
    Real va = dot(normal, cross(inB, inC));
    Real s = va + vb + vc;
    *u = va / s;
    *v = vb / s;
    *w = 1 - *u - *v;
}

static inline void GJKAddSupportPoint(GJKData *data, Vec3 supportPoint, u16 supportPointId) {
    data->Simplex[data->NumSimplexPoints] = supportPoint;
    data->SimplexIds[data->NumSimplexPoints] = supportPointId;
    ++data->NumSimplexPoints;
}

static inline bool GJKIndexInOldSimplexIds(u16 newSupportPointIdx, GJKData *data) {
    for (u32 i = 0; i < data->NumOldSimplexPoints; ++i) {
        if (data->SimplexIds[i] == newSupportPointIdx) {
            return true;
        }
    }

    return false;
}

} // namespace ce
