#include "Math.hpp"
#include "SAT.h"
#include "ContactHullVsHull.h"


namespace ce {
void HullAndHullTest(u32 body1, u32 body2, Transform *transform1, Transform *transform2, BHull *hull1, BHull *hull2,
                     WorldContactSet *contactSet) {
    // @todo: Optimize by caching
    ConvexHull h1 = *hull1->Hull;
    ConvexHull h2 = *hull2->Hull;

    Transform relativeTransform;
    relativeTransform.Orientation = conj(transform1->Orientation) * transform2->Orientation;
    relativeTransform.Pos = rotateInv(transform2->Pos - transform1->Pos, transform1->Orientation);

    SatResult result;
    if (ConvexSat(h1, h2, &relativeTransform, &result)) {

        ManifoldId manifoldId;

        manifoldId.Body1 = body1;
        manifoldId.Body2 = body2;
        contactSet->ManifoldIds.push_back(manifoldId);

        Manifold manifold;
        manifold.NumPoints = result.ManifoldSize;

        // Transform the axis to world coordinates from body 1s coordinates
        result.Axis = rotate(result.Axis, transform1->Orientation);

        // SAT will return the axis of the penetration of the reference face for
        // face contacts. If the reference face is on body 2, then it will point
        // from body 2 to body 1, so we reverse the normal, which will now make
        // it point from body 1 to body 2. the manifold point of view, the
        // normal always points away from manifoldId.Body1. For edge contacts,
        // this is never needed.
        if (result.Body1IsRef) {
            manifold.Normal = result.Axis;
        } else {
            manifold.Normal = -result.Axis;
        }

        // @todo SAT currently uses global contact points, should fix so that
        // it's better for floating point comparisons
        for (u8 i = 0; i < result.ManifoldSize; ++i) {
            manifold.PointIds[i] = ContactPointId{.Id = result.Ids[i].Id};
            manifold.Points[i] = ContactPoint{
                .Penetration = result.Penetrations[i],
                .RelContactPoint1 = rotate(result.Points[i], transform1->Orientation),
                .RelContactPoint2 = rotate(result.Points[i] - relativeTransform.Pos, transform1->Orientation),
            };
        }
        manifold.NumPoints = result.ManifoldSize;
        contactSet->Manifolds.push_back(manifold);
    }
}
} // namespace ce
