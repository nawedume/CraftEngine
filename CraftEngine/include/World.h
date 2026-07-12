#pragma once

#include "Core.h"
#include <unordered_map>
#include <vector>

namespace ce {
    Vec3 const X_AXIS = Vec3(1.0f, 0.0f, 0.0f);
    Vec3 const Y_AXIS = Vec3(0.0f, 1.0f, 0.0f);
    Vec3 const Z_AXIS = Vec3(0.0f, 0.0f, 1.0f);
    Vec3 const UP_AXIS = Vec3(0.0f, 1.0f, 0.0f);
    typedef u32 BodyId;

    struct Transform {
        Vec3 Pos { 0.0f, 0.0f, 0.0f };
        Quat Orientation { 1.0f, 0.0f, 0.0f, 0.0f };

        Mat4 Matrix();

        Vec3 Apply(Vec3 vertex);

        Vec3 ApplyInverse(Vec3 vertex);

        Transform Inverse();

        Transform Compose(Transform const& other);
    };

    struct RigidBody {
        Vec3 LinearVelocity;
        Vec3 AngularVelocity;

        Real InverseMass;
        Mat3 InverseInertia;
    };

    struct RigidBodyBase {
        Mat3 InverseInertiaLocal;
    };

    struct ForceAccumulator {
        Vec3 LinearForce;
        Vec3 AngularForce;
    };

    // Half edge representation makes it easier to transform when objects rotate
    struct AABB {
        Vec3 Center;
        Vec3 HalfEdge;
    };

    enum ColliderId {
        SPHERE,
        CAPSULE,
        HULL
    };

    struct BSphere {
        Real Radius;
    };

    struct ConvexHull;
    struct BHull {
        ConvexHull* Hull;
    };

    struct BCapsule {
        Real Radius;
        Real HalfLength;
    };

    struct Collider {
        ColliderId Id;

        union {
            BSphere Sphere;
            BCapsule Capsule;
            BHull Hull;
        };
    };

    struct Material {
        Real Restitution { 0.0f };
        Real Friction { 0.5f };
    };

    struct WorldContactSet;
    struct ContactPoint;

    union ManifoldId {
        struct {
            u32 Body1;
            u32 Body2;
        };
        u64 Id;
    };

    union ContactPointId {
        u32 Id;
    };

    struct FullContactId {
        ManifoldId MID;
        ContactPointId PointId;

        bool operator==(const ce::FullContactId& other) const {
            return (MID.Id == other.MID.Id) && (PointId.Id == other.PointId.Id);
        }
    };
};

namespace std {
    template <> struct hash<ce::FullContactId> {
        size_t operator()(const ce::FullContactId& k) const {
            return ((hash<size_t>()(k.MID.Id) ^ (hash<size_t>()(k.PointId.Id) << 1)) >> 1);
        }
    };
};

namespace ce {
    struct ImpulseStore {
        std::unordered_map<FullContactId, ContactPoint> Store;
        bool Get(FullContactId id, ContactPoint* res);
        void Set(FullContactId id, ContactPoint* p);

        void Clear() {
            Store.clear();
        }
    };

    struct WorldSettings {
        Real PenetrationSlop = 0.05;
        Real StabilizationTerm = 0.1;
        Real RestitutionSlop = 0.05;
        Real NumOfSolverIterations = 5;
        Real NumOfRelaxationIterations = 5;
    };

    struct BodyPair {
        BodyId Body1;
        BodyId Body2;
    };

    struct RevoluteJoint {
        BodyPair Bodies;
        Vec3 LocalAnchors[2];
        Vec3 Anchors[2];
        Mat3 EffectiveMass;
        Vec3 Bias { 0.0, 0.0, 0.0 };
        Vec3 Impulse { 0.0, 0.0, 0.0 };
    };

    struct World {
        u8 WorldId;

        WorldSettings Settings {};

        // Rigid Body transforms
        std::vector<Transform> Transforms;
        std::vector<RigidBody> RigidBodies;
        std::vector<RigidBodyBase> RigidBodiesBase;
        std::vector<ForceAccumulator> ForceAccumulators;

        std::vector<Collider> Colliders;
        std::vector<Material> Materials;
        std::vector<AABB> AABBs; // 1 per collider
        std::vector<BodyPair> BroadPhaseBodies;

        // Joints
        std::vector<RevoluteJoint> AnchorJoints;

        WorldContactSet* ContactSet;

        ImpulseStore StoredImpulses;

        Vec3 GravityAcc { 0.0f, -9.81f, 0.0f };

        u32 NumBodies() {
            return Transforms.size();
        }
    };

    extern World* NewWorld();

    extern void Step(World* world, Real deltaTime);

    extern void CreateContactSet(World* world);

    extern BodyId AddBody(World* world, Transform transform, RigidBodyBase bodyBase, RigidBody body, Collider collider, AABB aabb);

    extern void ApplyLinearForce(World* world, u32 idx, Vec3 force);

    extern void ApplyAngularForce(World* world, u32 idx, Vec3 force);

    struct BoxDef {
        Real Mass;
        Vec3 HalfEdge;
    };
    extern BodyId AddBox(World* world, Transform& transform, BoxDef& def);

    struct SphereDef {
        Real Mass;
        Real Radius;
    };
    extern BodyId AddSphere(World* world, Transform& transform, SphereDef& def);

    extern BodyId AddHollowSphere(World* world, Transform& transform, SphereDef& def);

    struct CapsuleDef {
        Real Mass;
        Real Radius;
        Real HalfLength;
    };
    extern BodyId AddCapsule(World* world, Transform& transform, CapsuleDef& def);

    struct Face {
        Vec3 Normal;
        Real D;

        Face Apply(Transform* t);
    };

    struct HalfEdge {
        u8 Vertex1;
        u8 Vertex2;
        u8 FaceId;
        u8 NextEdge;
    };

    // @todo split this up for each feature, find what's a reasonable limit
    int const MAX_NUM_FEATURES_CONVEX_HULL = 127;

    /**
     * A simple data structure that makes it easy to represent convex shapes.
     * It's up to the caller to ensure the shapes are convex, and there is no checks to ensure
     * that the shape is of correct form.
     *
     * Maybe require some optimizing and cleaning up later, but fine for now. Immutable.
     */
    struct ConvexHull {
        Vec3* Vertices;
        Face* Faces;
        HalfEdge* HalfEdges;
        u8* FirstEdgeIndex; // Same size as Faces

        u8 NumFaces;
        u8 NumHalfEdges;
        u8 NumVertices;

        AABB Bounds;

        ConvexHull(u8 numFaces, u8 numHalfEdges, u8 numVertices): NumFaces(numFaces), NumHalfEdges(numHalfEdges), NumVertices(numVertices) {
            assert(numFaces <= MAX_NUM_FEATURES_CONVEX_HULL);
            assert(numHalfEdges <= MAX_NUM_FEATURES_CONVEX_HULL);
            assert(numVertices <= MAX_NUM_FEATURES_CONVEX_HULL);

            Faces = new Face[NumFaces];
            Vertices = new Vec3[NumVertices];
            HalfEdges = new HalfEdge[NumHalfEdges];
            FirstEdgeIndex = new u8[numFaces];
        }

        Vec3 GetDirection(u8 edgeId) {
            HalfEdge* edge = &HalfEdges[edgeId];
            return Vertices[edge->Vertex2] - Vertices[edge->Vertex1];
        }

        Vec3 GetSupport(Vec3 dir);

        Face* GetAdjFace(u8 edgeIdx) {
            if (edgeIdx & 1) {
                return &Faces[HalfEdges[edgeIdx - 1].FaceId];
            } else {
                return &Faces[HalfEdges[edgeIdx + 1].FaceId];
            }
        }

        void CalculateBounds();
    };

    struct ConvexHullDef {
        Real Mass;
        Mat3 Inertia;
        ConvexHull* Hull;
    };

    extern BodyId AddConvexHull(World* world, Transform& transform, ConvexHullDef& def);

    extern BodyId AddStaticConvexHull(World* world, Transform& transform, ConvexHullDef& def);

    struct ContactPoint {
        Real Penetration;

        Vec3 RelContactPoint1;
        Vec3 RelContactPoint2;

        // @derrived
        Vec3 R1xN;
        Vec3 R2xN;

        Vec3 R1xT[2];
        Vec3 R2xT[2];

        // Normal Impulse
        Real NImpulse;

        // Tangent Impulse
        Real TImpulse[2];

        Real NEffectiveMass;
        Real TEffectiveMass[2];

        Real Bias { 0.0f };
    };

    struct Manifold {
        Vec3 Normal;
        Vec3 Tangents[2];

        Real Restitution;
        Real FrictionCoef;

        ContactPointId PointIds[4];
        ContactPoint Points[4];
        u8 NumPoints;
    };

    struct WorldContactSet {
        std::vector<ManifoldId> ManifoldIds;
        std::vector<Manifold> Manifolds;

        u32 Size() {
            return ManifoldIds.size();
        }

        void Clear() {
            ManifoldIds.clear();
            Manifolds.clear();
        }
    };

    typedef u32 JointId;

    JointId AddRevoluteJoint(World* world, BodyId b1, BodyId b2, Vec3 globalAnchor);
};
