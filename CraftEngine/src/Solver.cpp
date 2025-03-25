#include "Solver.h"
#include "Debug.h"
#include "Math.hpp"
#include "World.h"

namespace ce {

Vec3 GetRelativeVelocity(RigidBody *body1, RigidBody *body2, ContactPoint *point) {
    return body2->LinearVelocity + cross(body2->AngularVelocity, point->RelContactPoint2) - body1->LinearVelocity -
           cross(body1->AngularVelocity, point->RelContactPoint1);
}

void PrepManifolds(WorldContactSet *contactSet, RigidBody *bodies, Material *materials, ImpulseStore *impulseStore,
                   Real inverseDt, WorldSettings settings) {
    for (u32 manifoldIdx = 0; manifoldIdx < contactSet->Size(); ++manifoldIdx) {
        Manifold *manifold = &contactSet->Manifolds[manifoldIdx];
        createOrthonormalBasis(manifold->Normal, manifold->Tangents, manifold->Tangents + 1);

        ManifoldId *mid = &contactSet->ManifoldIds[manifoldIdx];
        Material *material1 = &materials[mid->Body1];
        Material *material2 = &materials[mid->Body2];

        manifold->Restitution = sqrt(material1->Restitution * material2->Restitution);
        manifold->FrictionCoef = sqrt(material1->Friction * material2->Friction);

        Real inverseMass = bodies[mid->Body1].InverseMass + bodies[mid->Body2].InverseMass;
        Mat3 inverseInertia1 = bodies[mid->Body1].InverseInertia;
        Mat3 inverseInertia2 = bodies[mid->Body2].InverseInertia;

        for (u32 i = 0; i < manifold->NumPoints; ++i) {
            ContactPoint *point = &manifold->Points[i];

            point->R1xN = cross(point->RelContactPoint1, manifold->Normal);
            point->R2xN = cross(point->RelContactPoint2, manifold->Normal);

            point->R1xT[0] = cross(point->RelContactPoint1, manifold->Tangents[0]);
            point->R2xT[0] = cross(point->RelContactPoint2, manifold->Tangents[0]);

            point->R1xT[1] = cross(point->RelContactPoint1, manifold->Tangents[1]);
            point->R2xT[1] = cross(point->RelContactPoint2, manifold->Tangents[1]);

            point->NInverseEffectiveMass = inverseMass + dot(point->R1xN, inverseInertia1 * point->R1xN) +
                                           dot(point->R2xN, inverseInertia2 * point->R2xN);
            point->TInverseEffectiveMass[0] = inverseMass + dot(point->R1xT[0], inverseInertia1 * point->R1xT[0]) +
                                              dot(point->R2xT[0], inverseInertia2 * point->R2xT[0]);
            point->TInverseEffectiveMass[1] = inverseMass + dot(point->R1xT[1], inverseInertia1 * point->R1xT[1]) +
                                              dot(point->R2xT[1], inverseInertia2 * point->R2xT[1]);

            RigidBody *body1 = &bodies[mid->Body1];
            RigidBody *body2 = &bodies[mid->Body2];

            point->Bias =
                (max(0.0f, point->Penetration - settings.PenetrationSlop) * settings.StabilizationTerm * inverseDt) +
                (manifold->Restitution *
                 glm::max(dot(GetRelativeVelocity(body1, body2, point), manifold->Normal) - settings.RestitutionSlop,
                          0.0f));

            // Find and apply cached impulses
            FullContactId fid;
            fid.MID = contactSet->ManifoldIds[manifoldIdx];
            fid.PointId = manifold->PointIds[i];

            ContactPoint cachedPoint;
            if (impulseStore->Get(fid, &cachedPoint)) {
                // Found impulses from last frame
                point->NImpulse = cachedPoint.NImpulse;
                point->TImpulse[0] = cachedPoint.TImpulse[0];
                point->TImpulse[1] = cachedPoint.TImpulse[1];

                Vec3 deltaNormal = manifold->Normal * point->NImpulse;
                Vec3 P = (point->NImpulse * manifold->Normal) + (point->TImpulse[0] * manifold->Tangents[0]) +
                         (point->TImpulse[1] * manifold->Tangents[1]);

                body1->LinearVelocity -= body1->InverseMass * P;
                body1->AngularVelocity -= body1->InverseInertia * cross(point->RelContactPoint1, P);

                body2->LinearVelocity += body2->InverseMass * P;
                body2->AngularVelocity += body2->InverseInertia * cross(point->RelContactPoint2, P);
            } else {
                point->NImpulse = 0.0;
                point->TImpulse[0] = 0.0;
                point->TImpulse[1] = 0.0;
            }
        }
    }
}

void SolveManifolds(RigidBody *bodies, WorldContactSet *contactSet, Real inverseDt) {
    for (u32 manifoldIdx = 0; manifoldIdx < contactSet->Size(); ++manifoldIdx) {
        ManifoldId manifoldId = contactSet->ManifoldIds[manifoldIdx];
        Manifold *manifold = &contactSet->Manifolds[manifoldIdx];

        RigidBody *body1 = &bodies[manifoldId.Body1];
        RigidBody *body2 = &bodies[manifoldId.Body2];

        for (u32 pointIdx = 0; pointIdx < manifold->NumPoints; ++pointIdx) {
            ContactPoint *point = &manifold->Points[pointIdx];

            // tangents
            for (int i = 0; i < 2; ++i) {
                Real tangent1Speed = dot(GetRelativeVelocity(body1, body2, point), manifold->Tangents[i]);
                Real tDeltaLambda = -tangent1Speed / point->TInverseEffectiveMass[i];
                Real tLambda = point->TImpulse[i];
                point->TImpulse[i] = clamp(tLambda + tDeltaLambda, -manifold->FrictionCoef, manifold->FrictionCoef);
                tDeltaLambda = point->TImpulse[i] - tLambda;

                Vec3 deltaT = tDeltaLambda * manifold->Tangents[i];

                body1->LinearVelocity -= body1->InverseMass * deltaT;
                body1->AngularVelocity -= body1->InverseInertia * (point->R1xT[i] * tDeltaLambda);

                body2->LinearVelocity += body2->InverseMass * deltaT;
                body2->AngularVelocity += body2->InverseInertia * (point->R2xT[i] * tDeltaLambda);
            }

            // normal
            Real separatingSpeed = dot(GetRelativeVelocity(body1, body2, point), manifold->Normal);

            Real constraint = -separatingSpeed + point->Bias;
            Real deltaLambda = constraint / point->NInverseEffectiveMass;
            Real lambda = point->NImpulse;
            point->NImpulse = max(point->NImpulse + deltaLambda, 0.0f);
            deltaLambda = point->NImpulse - lambda;
            Vec3 deltaNormal = manifold->Normal * deltaLambda;

            body1->LinearVelocity -= body1->InverseMass * deltaNormal;
            body1->AngularVelocity -= body1->InverseInertia * (point->R1xN * deltaLambda);

            body2->LinearVelocity += body2->InverseMass * deltaNormal;
            body2->AngularVelocity += body2->InverseInertia * (point->R2xN * deltaLambda);
        }
    }
}
} // namespace ce
