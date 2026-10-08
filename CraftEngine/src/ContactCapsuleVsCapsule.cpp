#include "Math.hpp"
#include "ContactCapsuleVsCapsule.h"

namespace ce {
void CapsuleAndCapsuleTest(u32 body1, u32 body2, Transform *transform1, Transform *transform2, BCapsule *cap1,
                           BCapsule *cap2, WorldContactSet *contactSet) {

    Vec3 c1N = rotate(Vec3(0.0, cap1->HalfLength, 0.0), transform1->Orientation);
    Vec3 p1 = transform1->Pos + c1N;
    Vec3 p2 = transform1->Pos - c1N;

    Vec3 c2N = rotate(Vec3(0.0, cap2->HalfLength, 0.0), transform2->Orientation);
    Vec3 p3 = transform2->Pos + c2N;
    Vec3 p4 = transform2->Pos - c2N;

    Vec3 d1 = p2 - p1;
    Vec3 d2 = p4 - p3;

    Vec3 d0 = p1 - p3;
    Real d1d1 = dot(d1, d1);
    Real d1d2 = dot(d1, d2);
    Real d1d0 = dot(d1, d0);
    Real d2d2 = dot(d2, d2);
    Real d2d0 = dot(d2, d0);

    Real denom = d1d1*d2d2 - d1d2*d1d2;

    Real s;
    if (denom > 1e-7) {
        s = clamp((-d1d2*d2d0 + d1d0*d2d2) / denom, 0.0, 1.0);
    } else {
        // Parallel, set s to 0, and we'll clamp later on. Perhaps it's better to change to be closest to center?
        s = 0.0;
    }
    Real t = clamp((d1d2*s + d2d0) / d2d2, 0.0, 1.0);
    s =  clamp((t*d1d2 - d1d0) / d1d1, 0.0, 1.0);

    Vec3 pA = p1 + s*d1;
    Vec3 pB = p3 + t*d2;
    Vec3 d = pB - pA;
    Real distance = length(d);
    Real penetration = cap1->Radius + cap2->Radius - distance;
    if (penetration < 0.0f) {
        // no contact
        return;
    }

    // Create contact
    // @todo; should detect parallel capsules to allow for stacking with 2 contact points
    ManifoldId manifoldId;
    manifoldId.Body1 = body1;
    manifoldId.Body2 = body2;
    contactSet->ManifoldIds.push_back(manifoldId);

    Manifold manifold;
    manifold.Normal = d / distance;
    manifold.NumPoints = 1;
    manifold.PointIds[0] = ContactPointId{.Id = 0};

    ContactPoint point{};
    point.RelContactPoint1 = pA + manifold.Normal * (cap1->Radius - 0.5f * penetration) - transform1->Pos;
    point.RelContactPoint2 = pB - manifold.Normal * (cap2->Radius - 0.5f * penetration) - transform2->Pos;
    point.Penetration = penetration;

    manifold.Points[0] = point;
    contactSet->Manifolds.push_back(manifold);
}
} // namespace ce
