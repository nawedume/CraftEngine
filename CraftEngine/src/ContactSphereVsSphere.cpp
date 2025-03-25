
#include "ContactSphereVsSphere.h"

namespace ce {
void SphereAndSphereTest(u32 body1, u32 body2, Transform *transform1, Transform *transform2, BSphere *sphere1,
                         BSphere *sphere2, WorldContactSet *contactSet) {
    Vec3 d = transform2->Pos - transform1->Pos;
    Real dist = length(d);
    Real sumOfRadius = sphere1->Radius + sphere2->Radius;
    Real pen = sumOfRadius - dist;

    if (pen > 0.0f) {
        ManifoldId manifoldId;
        manifoldId.Body1 = body1;
        manifoldId.Body2 = body2;
        contactSet->ManifoldIds.push_back(manifoldId);

        Manifold manifold;
        manifold.Normal = d / dist;
        manifold.NumPoints = 1;
        manifold.PointIds[0] = ContactPointId{.Id = 0};

        ContactPoint point{};
        point.RelContactPoint1 = manifold.Normal * (sphere1->Radius - 0.5f * pen);
        point.RelContactPoint2 = -manifold.Normal * (sphere2->Radius - 0.5f * pen);
        point.Penetration = pen;

        manifold.Points[0] = point;
        contactSet->Manifolds.push_back(manifold);
    }
}
} // namespace ce
