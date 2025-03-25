#include "World.h"
#include "BroadPhase.h"
#include "Math.hpp"
#include "Contact.h"
#include "Solver.h"
#include "AABB.h"
#include "JointSolver.h"

namespace ce {
    Mat4 Transform::Matrix() {
        Mat4 m = qToMat4(Orientation);
        m[3][0] = Pos.x;
        m[3][1] = Pos.y;
        m[3][2] = Pos.z;
        m[3][3] = 1.0f;
        return m;
    }

    Vec3 Transform::Apply(Vec3 vertex) {
        return rotate(vertex, Orientation) + Pos;
    }

    Vec3 Transform::ApplyInverse(Vec3 vertex) {
        return rotateInv(vertex - Pos, Orientation);
    }

    Transform Transform::Inverse() {
        return {
            .Pos = -Pos,
            .Orientation = conj(Orientation)
        };
    }

    Transform Transform::Compose(Transform const& other) {
        return {
            .Pos = rotate(this->Pos + other.Pos, this->Orientation),
            .Orientation = this->Orientation * other.Orientation
        };
    }

    Face Face::Apply(Transform* t) {
        Face f;
        f.Normal = rotate(Normal, t->Orientation);
        f.D = D + dot(t->Pos, f.Normal);
        return f;
    }

    Vec3 ConvexHull::GetSupport(Vec3 dir) {
        Real maxVal = -MAX_REAL;
        Vec3 maxVertex;
        for (u32 i = 0; i < NumVertices; ++i) {
            Real val = dot(Vertices[i], dir);
            if (val > maxVal) {
                maxVal = val;
                maxVertex = Vertices[i];
            }
        }
        return maxVertex;
    }

    bool ImpulseStore::Get(FullContactId id, ContactPoint* res) {
        auto found = Store.find(id);
        if (found != Store.end()) {
            *res = found->second;
            return true;
        }
        return false;
    }

    void ImpulseStore::Set(FullContactId id, ContactPoint* p) {
        Store[id] = *p;
    }

    static Real LINEAR_DAMPING_COEFF = 0.995;
    static Real ANGULAR_DAMPING_COEFF = 0.995;

    static u32 WorldCount = 0;

    World* NewWorld() {
        World* world = new World();
        world->WorldId = WorldCount++;
        world->ContactSet = new WorldContactSet();
        return world;
    }

    void IntegrateVelocities(World* world, Real deltaTime) {
        Real const linearDampingCoef = powf(LINEAR_DAMPING_COEFF, deltaTime);
        Real const angularDampingCoef = powf(ANGULAR_DAMPING_COEFF, deltaTime);

        for (u32 i = 0; i < world->RigidBodies.size(); ++i) {
            RigidBody* body = &world->RigidBodies[i];
            if (body->InverseMass == 0.0f) {
                continue;
            }

            ForceAccumulator* force = &world->ForceAccumulators[i];

            body->LinearVelocity +=  (force->LinearForce * body->InverseMass + world->GravityAcc) * deltaTime * linearDampingCoef;
            body->AngularVelocity += (force->AngularForce * body->InverseInertia) * deltaTime * angularDampingCoef;

            force->LinearForce = Vec3(0.0f);
            force->AngularForce = Vec3(0.0f);
        }
    }

    void IntegratePositions(World* world, Real deltaTime) {
        for (u32 i = 0; i < world->NumBodies(); ++i) {
            Transform* transform = &world->Transforms[i];
            RigidBody* body = &world->RigidBodies[i];

            transform->Pos += body->LinearVelocity * deltaTime;
            transform->Orientation += 0.5f * deltaTime * vec3ToQuat(body->AngularVelocity) * transform->Orientation;
            transform->Orientation = normalize(transform->Orientation);
        }
    }

    void PostPoseUpdate(World* world) {
        for (int i = 0; i < world->NumBodies(); ++i) {
            Collider* collider = &world->Colliders[i];
            switch (collider->Id) {
                case ColliderId::SPHERE:
                    world->AABBs[i] = CalculateTightFittingAABBForSphere(&collider->Sphere, &world->Transforms[i]);
                    break;
                case ColliderId::CAPSULE:
                    world->AABBs[i] = CalculateTightFittingAABBForCapsule(&collider->Capsule, &world->Transforms[i]);
                    break;
                case ColliderId::HULL:
                    world->AABBs[i] = CalculateAABBFromRotatedAABB(&collider->Hull.Hull->Bounds, &world->Transforms[i]);
                    break;
                default:
                    assert(false && "Collider not implemented for AABB compute yet.");
            }
        }
    }

    void Step(World* world, Real deltaTime) {
        Real inverseDeltaTime = 1.0f / deltaTime;

        for (u32 i = 0; i < world->NumBodies(); ++i) {
            // Is used for contact detection to determine the objects axes
            // might not be necessary, maybe should remove and make it lazy
            Transform& transform = world->Transforms[i];
            RigidBody& body = world->RigidBodies[i];
            Mat3 localInverseInertia = world->RigidBodiesBase[i].InverseInertiaLocal;

            // Update the global inertia vector using the current orientation
            Mat3 rotation = qToMat(transform.Orientation);
            Mat3 globalInverseInertia = rotation * localInverseInertia * transpose(rotation);
            body.InverseInertia = globalInverseInertia;
        }

        BruteForceBroadPhaseSIMD(world);

        IntegrateVelocities(world, deltaTime);

        world->ContactSet->Clear();
        DetectContacts(world, world->BroadPhaseBodies);

        // Solve contacts
        PrepManifolds(world->ContactSet, world->RigidBodies.data(), world->Materials.data(), &world->StoredImpulses, inverseDeltaTime, world->Settings);
        PrepJoints(world, inverseDeltaTime);

        for (u32 iteration = 0; iteration < world->Settings.NumOfSolverIterations; ++iteration) {
            SolveManifolds(world->RigidBodies.data(), world->ContactSet, inverseDeltaTime);
            SolveJoints(world, inverseDeltaTime);
        }

        IntegratePositions(world, deltaTime);
        PostPoseUpdate(world);

        for (u32 i = 0; i < world->ContactSet->Size(); ++i) {
            Manifold* m = &world->ContactSet->Manifolds[i];
            for (u32 j = 0; j < m->NumPoints; ++j) {
                m->Points[j].Bias = 0.0f;
            }
        }

        // relaxation
        for (u32 iteration = 0; iteration < world->Settings.NumOfRelaxationIterations; ++iteration) {
            SolveManifolds(world->RigidBodies.data(), world->ContactSet, inverseDeltaTime);
        }

        // Store impulses from this frame
        for (u32 i = 0; i < world->ContactSet->Size(); ++i) {
            Manifold* manifold = &world->ContactSet->Manifolds[i];
            ManifoldId* manifoldId = &world->ContactSet->ManifoldIds[i];
            FullContactId id;
            id.MID = *manifoldId;
            for (u32 j = 0; j < manifold->NumPoints; ++j) {
                ContactPointId* pointId = &manifold->PointIds[j];
                ContactPoint point = manifold->Points[j];

                FullContactId finalId = id;
                finalId.PointId = *pointId;
                world->StoredImpulses.Set(finalId, &point);
            }
        }
    }

    BodyId AddBody(World* world, Transform transform, RigidBodyBase bodyBase, RigidBody body, Collider collider, AABB aabb) {
        u32 bodyId = world->NumBodies();

        world->Transforms.push_back(transform);

        Mat3 rotation = qToMat(transform.Orientation);
        Mat3 globalInverseInertia = rotation * bodyBase.InverseInertiaLocal * transpose(rotation);
        body.InverseInertia = globalInverseInertia;
        world->RigidBodies.push_back(body);

        world->RigidBodiesBase.push_back(bodyBase);
        world->ForceAccumulators.push_back(ForceAccumulator {});
        world->Colliders.push_back(collider);
        world->Materials.push_back(Material {});
        world->AABBs.push_back(aabb);

        return bodyId;
    }

    BodyId AddSphere(World *world, Transform &transform, SphereDef &def) {
        Real inverseMass = (1.0f / def.Mass);
        Real r = (5.0f / 2.0f) * inverseMass  * 1.0f / (def.Radius * def.Radius);
        Mat3 inverseInertia = Mat3 {
            r, 0.0f, 0.0f,
            0.0f, r, 0.0f,
            0.0f, 0.0f, r
        };
        RigidBody body {
            .LinearVelocity = Vec3(0.0f, 0.0f, 0.0f),
            .AngularVelocity = Vec3(0.0f, 0.0f, 0.0f),
            .InverseMass = inverseMass
        };

        Collider collider { .Id = ColliderId::SPHERE, .Sphere = { .Radius = def.Radius } };
        // @todo, replace when adding separate transform for shape offset
        Transform t {};
        return AddBody(world, transform, { .InverseInertiaLocal = inverseInertia }, body, collider, CalculateTightFittingAABBForSphere(&collider.Sphere, &t));
    }

    BodyId AddHollowSphere(World *world, Transform &transform, SphereDef &def) {
        Real inverseMass = (1.0f / def.Mass);
        Real r = (3.0f / 2.0f) * inverseMass  * 1.0f / (def.Radius * def.Radius);
        Mat3 inverseInertia = Mat3 {
            r, 0.0f, 0.0f,
            0.0f, r, 0.0f,
            0.0f, 0.0f, r
        };
        RigidBody body {
            .LinearVelocity = Vec3(0.0f, 0.0f, 0.0f),
            .AngularVelocity = Vec3(0.0f, 0.0f, 0.0f),
            .InverseMass = inverseMass,
        };

        Collider collider { .Id = ColliderId::SPHERE, .Sphere = { .Radius = def.Radius } };
        Transform t {};
        return AddBody(world, transform, { .InverseInertiaLocal = inverseInertia }, body, collider, CalculateTightFittingAABBForSphere(&collider.Sphere, &t));
    }

    BodyId AddCapsule(World* world, Transform& transform, CapsuleDef& def) {
        // @todo: Implement with the real moment of inertia of a capsule. This is a cylinder.
        Real inverseMass = (1.0f / def.Mass);
        Real height = def.HalfLength * 2.0f;
        Real ixz = (1.0f / 12.0f) * def.Mass * (3.f * def.Radius * def.Radius + height * height);
        Real iy = 0.5f * def.Mass * def.Radius * def.Radius;

        Mat3 inverseInertia = {
            ixz, 0.0f, 0.0f,
            0.0f, iy, 0.0f,
            0.0f, 0.0f, ixz
        };

        RigidBody body {
            .LinearVelocity = Vec3(0.0f),
            .AngularVelocity = Vec3(0.0f),
            .InverseMass = inverseMass
        };
        Collider collider { .Id = ColliderId::CAPSULE, .Capsule = { .Radius = def.Radius, .HalfLength = def.HalfLength }};
        Transform t {};
        return AddBody(world, transform, { .InverseInertiaLocal = inverseInertia }, body, collider, CalculateTightFittingAABBForCapsule(&collider.Capsule, &t));
    }

    void ApplyLinearForce(World* world, u32 idx, Vec3 force) {
        world->ForceAccumulators[idx].LinearForce += force;
    }

    void ApplyAngularForce(World* world, u32 idx, Vec3 force) {
        world->ForceAccumulators[idx].AngularForce += force;
    }

    BodyId AddConvexHull(World* world, Transform& transform, ConvexHullDef& def) {
        RigidBody body {
            .LinearVelocity = Vec3(0.0f),
            .AngularVelocity  = Vec3(0.0f),
            .InverseMass = (1.0f / def.Mass),
        };

        def.Hull->CalculateBounds();
        Collider collider { .Id = ColliderId::HULL, .Hull = BHull { .Hull = def.Hull } };
        // todo cache the inverse calculation so we don't need to do it for each instance
        return AddBody(world, transform, { .InverseInertiaLocal = inverse(def.Inertia) }, body, collider, CalculateAABBFromRotatedAABB(&def.Hull->Bounds, &transform));
    }

    BodyId AddStaticConvexHull(World* world, Transform& transform, ConvexHullDef& def) {
        // @todo update implementation to convert the Convexhull vertices and normals to use the transform, that way we don't need to constantly apply the transform
        RigidBody body {
            .LinearVelocity = Vec3(0.0f),
            .AngularVelocity  = Vec3(0.0f),
            .InverseMass = 0.0f
        };
        def.Hull->CalculateBounds();
        Collider collider { .Id = ColliderId::HULL, .Hull = BHull { .Hull = def.Hull } };
        return AddBody(world, transform, { .InverseInertiaLocal = Mat4(0.0f) }, body, collider, CalculateAABBFromRotatedAABB(&def.Hull->Bounds, &transform));
    }

    void ConvexHull::CalculateBounds() {
        Bounds = CalculateTightFittingAABBForHullInLocalSpace(this);
    }

    JointId AddRevoluteJoint(World* world, BodyId b1, BodyId b2, Vec3 globalAnchor) {
        RevoluteJoint outJoint;
        outJoint.Bodies.Body1 = b1;
        outJoint.Bodies.Body2 = b2;

        Transform* t1 = &world->Transforms[b1];
        Transform* t2 = &world->Transforms[b2];
        outJoint.LocalAnchors[0] = rotateInv(globalAnchor - t1->Pos, t1->Orientation);
        outJoint.LocalAnchors[1] = rotateInv(globalAnchor - t2->Pos, t2->Orientation);

        JointId id = world->AnchorJoints.size();
        world->AnchorJoints.push_back(outJoint);
        return id;
    }
}
