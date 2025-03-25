#include "Math.hpp"
#include "ContactCapsuleVsCapsule.h"

namespace ce {
void CapsuleAndCapsuleTest(u32 body1, u32 body2, Transform *transform1, Transform *transform2, BCapsule *cap1,
                           BCapsule *cap2, WorldContactSet *contactSet) {

    // find distance between the two lines and clip, taken from https://paulbourke.net/geometry/pointlineplane/
    Vec3 c1N = rotate(Vec3(0.0, cap1->HalfLength, 0.0), transform1->Orientation);
    Vec3 p1 = transform1->Pos + c1N;
    Vec3 p2 = transform1->Pos - c1N;

    Vec3 c2N = rotate(Vec3(0.0, cap2->HalfLength, 0.0), transform2->Orientation);
    Vec3 p3 = transform2->Pos + c2N;
    Vec3 p4 = transform2->Pos - c2N;

    Real d_1343 = dot(p1 - p3, p4 - p3);
    Real d_4321 = dot(p4 - p3, p2 - p1);
    Real d_1321 = dot(p1 - p3, p2 - p1);
    Real d_4343 = dot(p4 - p3, p4 - p3);
    Real d_2121 = dot(p2 - p1, p2 - p1);

    Real denom = (d_2121 * d_4343 - d_4321 * d_4321);

    Vec3 pA;
    Vec3 pB;
    if (abs(denom) < 1e-7) {
        Vec3 normalA = normalize(p2 - p1);
        if (dot(p4 - p3, normalA) < 0.0f) {
            std::swap(p3, p4);
        }

        Real p4_p1_dot = dot(p4 - p1, normalA);
        Real p3_p1_dot = dot(p3 - p1, normalA);
        Real p4_p2_dot = dot(p4 - p2, normalA);
        Real p3_p2_dot = dot(p3 - p2, normalA);
        if (p4_p1_dot <= 0.0f) {
            // p4
            pA = p1;
            pB = p4;
        } else if (p3_p2_dot >= 0.0) {
            // p3
            pA = p2;
            pB = p3;
        } else {
            if (p3_p1_dot >= 0.0) {
                // p3 is under p1, use p3
                pA = p1 + normalA * p3_p1_dot;
                pB = p3;
            } else if (p4_p2_dot <= 0.0) {
                // p4 is above p2, use p4
                pA = p2 + normalA * p4_p2_dot;
                pB = p4;
            } else {
                // p3 is above p1, and p4 is under p2, use p1
                pA = p1;
                pB = p3 + normalA * p3_p1_dot;
            }
        }

    } else {
        Real mua = (d_1343 * d_4321 - d_1321 * d_4343) / denom;
        Real mub = (d_1343 + mua * d_4321) / d_4343;

        if (mua <= 0.0f) {
            pA = p1;
        } else if (mua >= 1.0f) {
            pA = p2;
        } else {
            pA = p1 + mua * (p2 - p1);
        }

        if (mub <= 0.0f) {
            pB = p3;
        } else if (mua >= 1.0f) {
            pB = p4;
        } else {
            pB = p3 + mub * (p4 - p3);
        }
    }

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
