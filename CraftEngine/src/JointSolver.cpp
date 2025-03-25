#include "JointSolver.h"
#include "Math.hpp"
#include "World.h"

namespace ce {

// Only handles RevoluteJoint right now, TODO: Add more later
void PrepJoints(World *world, Real inverseDt) {
    for (u32 i = 0; i < world->AnchorJoints.size(); ++i) {
        RevoluteJoint *joint = &world->AnchorJoints[i];
        RigidBody *body1 = &world->RigidBodies[joint->Bodies.Body1];
        RigidBody *body2 = &world->RigidBodies[joint->Bodies.Body2];

        joint->Anchors[0] = rotate(joint->LocalAnchors[0], world->Transforms[joint->Bodies.Body1].Orientation);
        joint->Anchors[1] = rotate(joint->LocalAnchors[1], world->Transforms[joint->Bodies.Body2].Orientation);

        Real inverseMass = body1->InverseMass + body2->InverseMass;
        Mat3 M1{inverseMass, 0.0f, 0.0f, 0.0f, inverseMass, 0.0f, 0.0f, 0.0f, inverseMass};
        Mat3 crossMat1 = matrixCross3(joint->Anchors[0]);
        Mat3 M2 = crossMat1 * body1->InverseInertia * crossMat1;

        Mat3 crossMat2 = matrixCross3(joint->Anchors[1]);
        Mat3 M3 = crossMat2 * body2->InverseInertia * crossMat2;

        joint->EffectiveMass = inverse(M1 - M2 - M3);

        Vec3 dist = (world->Transforms[joint->Bodies.Body2].Pos + joint->Anchors[1]) -
                    (world->Transforms[joint->Bodies.Body1].Pos + joint->Anchors[0]);
        joint->Bias = -0.2f * dist * inverseDt;

        // warm starting
        body1->LinearVelocity -= body1->InverseMass * joint->Impulse;
        body1->AngularVelocity -= body1->InverseInertia * cross(joint->Anchors[0], joint->Impulse);

        body2->LinearVelocity += body2->InverseMass * joint->Impulse;
        body2->AngularVelocity += body2->InverseInertia * cross(joint->Anchors[1], joint->Impulse);
    }
}

void SolveJoints(World *world, Real inverseDt) {
    for (u32 i = 0; i < world->AnchorJoints.size(); ++i) {
        RevoluteJoint *joint = &world->AnchorJoints[i];
        RigidBody *body1 = &world->RigidBodies[joint->Bodies.Body1];
        RigidBody *body2 = &world->RigidBodies[joint->Bodies.Body2];
        Vec3 relativeVelocity = body2->LinearVelocity + cross(body2->AngularVelocity, joint->Anchors[1]) -
                                body1->LinearVelocity - cross(body1->AngularVelocity, joint->Anchors[0]);
        Vec3 deltaImpulse = joint->EffectiveMass * (joint->Bias - relativeVelocity);

        body1->LinearVelocity -= body1->InverseMass * deltaImpulse;
        body1->AngularVelocity -= body1->InverseInertia * cross(joint->Anchors[0], deltaImpulse);

        body2->LinearVelocity += body2->InverseMass * deltaImpulse;
        body2->AngularVelocity += body2->InverseInertia * cross(joint->Anchors[1], deltaImpulse);

        joint->Impulse += deltaImpulse;
    }
}
} // namespace ce
