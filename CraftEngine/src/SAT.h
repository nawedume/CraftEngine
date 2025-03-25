#pragma once

#include "World.h"

namespace ce {

    union SatId {
        struct {
            u32 RefFaceId : 7;
            u32 IncFaceId : 7;
            u32 PointId : 2;
            u32 ClipEdgeId : 14;
        } FaceInfo;

        struct {
            u32 EdgeId1: 7;
            u32 EdgeId2: 7;
        } EdgeInfo;
        u32 Id;
    };

    struct SatResult {
        Vec3 Axis;
        u8 ManifoldSize; // Limit to 4
        SatId Ids[4];
        Real Penetrations[4];
        Vec3 Points[4];
        bool Body1IsRef;
    };

    bool ConvexSat(ConvexHull g1, ConvexHull g2, Transform* relativeTransform, SatResult* result);
}
