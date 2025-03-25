#include "ContactSphereVsCapsule.h"
#include "Math.hpp"

namespace ce {
void SphereAndCapsuleTest(u32 body1, u32 body2, Transform *transform1, Transform *transform2, BSphere *sphere,
                          BCapsule *cap, WorldContactSet *contactSet) {

    // find the closest point in the capsule inner segment to the sphere center
    Vec3 cDir = rotate(Vec3(0.0, cap->HalfLength, 0.0), transform2->Orientation);
    Vec3 cN = normalize(cDir);
    Vec3 cP = transform2->Pos - cN;

    Real sphereProj = dot(transform1->Pos - cP, cN);
    Vec3 capsulePoint;
    if (sphereProj >= 2.0 * cap->HalfLength) {
        capsulePoint = (cP + 2.0f * cDir);
    } else if (sphereProj <= 0.0) {
        capsulePoint = cP;
    } else {
        capsulePoint = cP + sphereProj * cN;
    }

    Vec3 d = capsulePoint - transform1->Pos;
    Real distance = length(d);
    Real penetration = sphere->Radius + cap->Radius - distance;
    if (penetration < 0.0) {
        return;
    }

    ManifoldId manifoldId;
    manifoldId.Body1 = body1;
    manifoldId.Body2 = body2;
    contactSet->ManifoldIds.push_back(manifoldId);

    Manifold manifold;
    manifold.Normal = d / distance;
    manifold.NumPoints = 1;
    manifold.PointIds[0] = ContactPointId{.Id = 0};

    ContactPoint point{};
    point.RelContactPoint1 = manifold.Normal * (sphere->Radius - 0.5f * penetration);
    point.RelContactPoint2 = capsulePoint + (-manifold.Normal * (cap->Radius - 0.5f * penetration)) - transform2->Pos;
    point.Penetration = penetration;

    manifold.Points[0] = point;
    contactSet->Manifolds.push_back(manifold);
}
} // namespace ce
