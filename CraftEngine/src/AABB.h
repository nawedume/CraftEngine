#include "Geometry.h"
#include "Math.hpp"
#include "World.h"
#include "utils.hpp"

namespace ce {

    // results is 6 array, and the result will be returned in the format {+x, -x, +y, -y, +z, -z}
    static inline void GetMinMaxExtents(ConvexHull* hull, Real* results) {
        Vec3 v = hull->Vertices[0];
        results[0] = v.x;
        results[1] = v.x;
        results[2] = v.y;
        results[3] = v.y;
        results[4] = v.z;
        results[5] = v.z;

        for (u32 i = 1; i < hull->NumVertices; ++i) {
            v = hull->Vertices[i];

            if (v.x > results[0]) {
                results[0] = v.x;
            } else if (v.x < results[1]) {
                results[1] = v.x;
            }

            if (v.y > results[2]) {
                results[2] = v.y;
            } else if (v.y < results[3]) {
                results[3] = v.y;
            }

            if (v.z > results[4]) {
                results[4] = v.z;
            } else if (v.z < results[5]) {
                results[5] = v.z;
            }
        }
    }

    static inline AABB CalculateTightFittingAABBForSphere(BSphere* sphere, Transform* t) {
        Real radius = sphere->Radius;
        return { t->Pos, Vec3(radius, radius, radius)};
    }

    static inline AABB CalculateTightFittingAABBForCapsule(BCapsule* cap, Transform* t) {
        Vec3 v = abs(TransformYByRotation(t->Orientation) * cap->HalfLength);
        return { t->Pos, v + cap->Radius };
    }

    static inline AABB CalculateTightFittingAABBForHullInLocalSpace(ConvexHull* hull) {
        Real extents[6];
        GetMinMaxExtents(hull, extents);

        Real centerX = (extents[0] + extents[1]) / 2.0;
        Real lenX = extents[0] - centerX;

        Real centerY = (extents[2] + extents[3]) / 2.0;
        Real lenY = extents[2] - centerY;

        Real centerZ = (extents[4] + extents[5]) / 2.0;
        Real lenZ = extents[4] - centerZ;

        return { Vec3(centerX, centerY, centerZ), Vec3(lenX, lenY, lenZ)};
    }

    static inline AABB CalculateAABBFromRotatedAABB(AABB* aabb, Transform* t) {
        // Might be better to cache this and pass it in
        Mat3 m = qToMat(t->Orientation);
        AABB outBox { t->Pos, Vec3(0.0) };

        for (u32 i = 0; i < 3; ++i) {
            for (u32 j = 0; j < 3; ++j) {
                outBox.Center[i] += m[j][i] * aabb->Center[j];
                outBox.HalfEdge[i] += abs(m[j][i]) * aabb->HalfEdge[j];
            }
        }

        return outBox;
    }

    static inline bool AABBOverlapTest(AABB* b1, AABB* b2) {
        return abs(b1->Center.x - b2->Center.x) <= (b1->HalfEdge.x + b2->HalfEdge.x) &&
            abs(b1->Center.y - b2->Center.y) <= (b1->HalfEdge.y + b2->HalfEdge.y) &&
            abs(b1->Center.z - b2->Center.z) <= (b1->HalfEdge.z + b2->HalfEdge.z);
    }
}
