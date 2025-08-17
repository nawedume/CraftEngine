#include "Geometry.h"
#include "Math.hpp"
#include "ContactSphereVsHull.h"

namespace ce {
extern void HandleSphereAndHullDeepIntersection(BodyId body1, BodyId body2, Transform *transform1, Transform *transform2,
                                                ConvexHull *hull, BSphere *sphere, Vec3 localSphereCenterPoint,
                                                WorldContactSet *contactSet);

void CreateSphereAndHullManifold(u32 body1, u32 body2, Transform *transform1, Transform *transform2, ConvexHull *hull,
                                 BSphere *sphere, Vec3 localContactPoint, Vec3 localSphereCenterPoint,
                                 WorldContactSet *contactSet) {
    Vec3 d = localContactPoint - localSphereCenterPoint;
    Real distance = length(d);
    Real penetration = sphere->Radius - distance;
    if (penetration < 0.0f) {
        return;
    } else if (isZero(distance)) {
        HandleSphereAndHullDeepIntersection(body1, body2, transform1, transform2, hull, sphere, localSphereCenterPoint,
                                            contactSet);
        return;
    }

    ManifoldId outManifoldId;
    Manifold outManifold;

    outManifoldId.Body1 = body1;
    outManifoldId.Body2 = body2;

    // Put into world coordinates
    outManifold.Normal = rotate(d / distance, transform2->Orientation);
    outManifold.NumPoints = 1;
    outManifold.PointIds[0] = ContactPointId{.Id = 0};

    ContactPoint point{};
    point.RelContactPoint1 = rotate(d, transform2->Orientation);
    point.RelContactPoint2 = rotate(localContactPoint, transform2->Orientation);
    point.Penetration = penetration;
    outManifold.Points[0] = point;

    contactSet->ManifoldIds.push_back(outManifoldId);
    contactSet->Manifolds.push_back(outManifold);
}

void HandleSphereAndHullDeepIntersection(u32 body1, u32 body2, Transform *transform1, Transform *transform2,
                                         ConvexHull *hull, BSphere *sphere, Vec3 localSphereCenterPoint,
                                         WorldContactSet *contactSet) {

    Real minimumFaceDistance = 0.0f;
    u32 minimumFaceIdx = -1;
    for (u32 faceIdx = 0; faceIdx < hull->NumFaces; ++faceIdx) {
        Face *face = &hull->Faces[faceIdx];
        Real distance = dot(localSphereCenterPoint, face->Normal) - face->D;
        if (distance < minimumFaceDistance) {
            minimumFaceDistance = distance;
            minimumFaceIdx = faceIdx;
        }
    }

    ManifoldId outManifoldId;
    Manifold outManifold;

    outManifoldId.Body1 = body1;
    outManifoldId.Body2 = body2;

    outManifold.Normal = hull->Faces[minimumFaceIdx].Normal;
    outManifold.NumPoints = 1;
    outManifold.PointIds[0] = ContactPointId{.Id = 0};

    ContactPoint point{};
    point.RelContactPoint1 = Vec3(0.0f);
    point.RelContactPoint2 = rotate(localSphereCenterPoint, transform2->Orientation);
    point.Penetration =
        sphere->Radius - minimumFaceDistance; // minimumFaceDistance should be negative, the penetration depth is the
                                              // sphere radius + the distance the center is from the face

    outManifold.Points[0] = point;

    contactSet->ManifoldIds.push_back(outManifoldId);
    contactSet->Manifolds.push_back(outManifold);
}

void SphereAndHullTest(u32 body1, u32 body2, Transform *transform1, Transform *transform2, BSphere *sphere,
                       BHull *bhull, WorldContactSet *contactSet) {

    // First run GJK to find the feature with the least penetration on the
    // center of the sphere
    ConvexHull *hull = bhull->Hull;

    // Will be in the local coordinate frame of the hull
    Vec3 centerPosition = transform2->Inverse().Apply(transform1->Pos);
    Vec3 firstVertex = hull->Vertices[0] - centerPosition;
    GJKData data = {.Simplex{firstVertex}, .SearchDir{-firstVertex}, .NumSimplexPoints = 1, .ShouldTerminate = false};

    u8 supportPointIdx;
    do {
        Vec3 supportPointHull = SupportPoint(hull, data.SearchDir, &supportPointIdx);
        Vec3 supportPoint = supportPointHull - centerPosition;

        if (GJKIndexInOldSimplexIds(supportPointIdx, &data)) {
            switch (data.NumSimplexPoints) {
            case 1: {
                Vec3 contactPoint = hull->Vertices[data.SimplexIds[0]];
                CreateSphereAndHullManifold(body1, body2, transform1, transform2, hull, sphere, contactPoint,
                                            centerPosition, contactSet);
            } break;
            case 2: {
                // It's fine to find the cloest point to the line instead of a segment since we know that
                // if the parameter t was out of bounds, GJK would've returned a point instead of a line
                Vec3 contactPoint = ClosestPointOnLine(hull->Vertices[data.SimplexIds[0]],
                                                       hull->Vertices[data.SimplexIds[1]], centerPosition);
                CreateSphereAndHullManifold(body1, body2, transform1, transform2, hull, sphere, contactPoint,
                                            centerPosition, contactSet);
            } break;
            case 3: {
                Vec3 v1 = hull->Vertices[data.SimplexIds[0]];
                Vec3 v2 = hull->Vertices[data.SimplexIds[1]];
                Vec3 v3 = hull->Vertices[data.SimplexIds[2]];
                Vec3 contactPoint = ClosestPointOnPlane(normalize(GetTriangleNormal(v1, v2, v3)), v1, centerPosition);
                CreateSphereAndHullManifold(body1, body2, transform1, transform2, hull, sphere, contactPoint,
                                            centerPosition, contactSet);
            } break;
            case 4:
                assert(false);
            }

            return;
        }

        GJKAddSupportPoint(&data, supportPoint, supportPointIdx);
        GJKIteration(&data);
    } while (data.NumSimplexPoints != 4);

    // v is considered 0, we need to fall back in SAT
    HandleSphereAndHullDeepIntersection(body1, body2, transform1, transform2, hull, sphere, centerPosition, contactSet);
}

} // namespace ce
