#include "ContactCapsuleVsHull.h"
#include "Debug.h"
#include "Geometry.h"
#include "Math.hpp"
#include "World.h"

namespace ce {

void CreateCapsuleAndHullManifold(BodyId body1, BodyId body2, Transform *transform1, Transform *transform2,
                                  BCapsule *cap, ConvexHull *hull, Vec3 *capsulePoints, Face *face,
                                  WorldContactSet *contactSet) {

    // mostly dupe of the deep intersection tests, could probably simplify it by combining the two but going to keep
    // separate in case things change
    Manifold manifold;
    Vec3 relativePos = transform2->Pos - transform1->Pos;

    manifold.NumPoints = 0;
    for (u32 i = 0; i < 2; ++i) {
        Real distance = dot(capsulePoints[i], face->Normal) - face->D;
        Real penetration = cap->Radius - distance;

        if (penetration > 0.0) {
            ContactPoint point{};
            Vec3 localPoint = capsulePoints[i] - face->Normal * distance;
            point.RelContactPoint2 = rotate(localPoint, transform2->Orientation);
            point.RelContactPoint1 = relativePos + point.RelContactPoint2;
            point.Penetration = penetration;
            manifold.Points[manifold.NumPoints] = point;
            manifold.PointIds[manifold.NumPoints] = ContactPointId{i};
            ++manifold.NumPoints;
        }
    }

    if (manifold.NumPoints > 0) {
        ManifoldId manifoldId;
        manifoldId.Body1 = body1;
        manifoldId.Body2 = body2;

        manifold.Normal = -rotate(face->Normal, transform2->Orientation);
        contactSet->ManifoldIds.push_back(manifoldId);
        contactSet->Manifolds.push_back(manifold);
    }
}

void ClipCapsuleLine(Vec3 *capsulePoints, Vec3 capsuleDir, u32 faceIdx, ConvexHull *hull) {

    u8 firstEdgeIdx = hull->FirstEdgeIndex[faceIdx];
    u8 currentEdgeIdx = firstEdgeIdx;
    do {
        HalfEdge *edge = &hull->HalfEdges[currentEdgeIdx];
        Face *face = hull->GetAdjFace(currentEdgeIdx);
        Vec3 vertexOnPlane = hull->Vertices[edge->Vertex1];
        Real faceDotCapsuleDir = dot(face->Normal, capsuleDir);
        if (!isZero(faceDotCapsuleDir)) {
            // Need to see which vertex is on the outside, i.e. which one should we clip
            Real t = dot(face->Normal, vertexOnPlane - capsulePoints[0]) / faceDotCapsuleDir;

            if (t > 0.0 && t < 1.0) {
                if (faceDotCapsuleDir < 0.0f) {
                    // Clip the first vertex
                    capsulePoints[0] = capsulePoints[0] + t * capsuleDir;
                } else {
                    // The second point is going towads the normal
                    capsulePoints[1] = capsulePoints[0] + t * capsuleDir;
                }
            }
        }

        currentEdgeIdx = edge->NextEdge;
    } while (currentEdgeIdx != firstEdgeIdx);
}

void HandleCapsuleAndHullDeepIntersection(BodyId body1, BodyId body2, Transform *t1, Transform *t2,
                                          Vec3 capsulePoints[2], ConvexHull *hull, Real radius, WorldContactSet *contactSet) {

    Real minimumFaceDistance = MIN_REAL;
    u32 minimumFaceIdx = -1;
    for (u32 faceIdx = 0; faceIdx < hull->NumFaces; ++faceIdx) {
        Face *face = &hull->Faces[faceIdx];
        Real distance = min(dot(capsulePoints[0], face->Normal), dot(capsulePoints[1], face->Normal)) - face->D;
        if (distance >= minimumFaceDistance) {
            minimumFaceDistance = distance;
            minimumFaceIdx = faceIdx;
        }
    }

    Face *separatingFace = &hull->Faces[minimumFaceIdx];

    // Need to check edges as well
    Vec3 capsuleDir = capsulePoints[1] - capsulePoints[0];
    Real minimumEdgeDistance = 0.0f;
    u32 miniumEdgeIdx = -1;
    Vec3 minimumEdgeNormal;
    for (u32 edgeIdx = 0; edgeIdx < hull->NumHalfEdges; edgeIdx += 2) {
        Vec3 edgeNormal = normalize(cross(hull->GetDirection(edgeIdx), capsuleDir));
        HalfEdge *edge = &hull->HalfEdges[edgeIdx];
        if (dot(edgeNormal, hull->Vertices[edge->Vertex1]) < 0.0) {
            edgeNormal = -edgeNormal;
        }
        Real distance = min(dot(capsulePoints[0], edgeNormal), dot(capsulePoints[1], edgeNormal)) -
                        dot(hull->Vertices[edge->Vertex1], edgeNormal);
        if (distance < minimumEdgeDistance) {
            minimumEdgeDistance = distance;
            miniumEdgeIdx = edgeIdx;
            minimumEdgeNormal = edgeNormal;
        }
    }

    if (minimumFaceDistance >= minimumEdgeDistance) {
        assert(minimumFaceDistance <= 0.0f); // Double check that there is an actua penetration, if this was positive it
                                             // would be a separating face
        ClipCapsuleLine(capsulePoints, capsuleDir, minimumFaceIdx, hull);

        // project the capsule points onto the reference face
        ManifoldId manifoldId;
        Manifold manifold;

        manifoldId.Body1 = body1;
        manifoldId.Body2 = body2;
        manifold.NumPoints = 0;

        Face *face = &hull->Faces[minimumFaceIdx];
        manifold.Normal = -face->Normal;

        Vec3 relativePos = t2->Pos - t1->Pos;
        Real distance0 = dot(capsulePoints[0], face->Normal) - face->D;
        if (distance0 <= 0) {
            manifold.PointIds[0] = ContactPointId{0};
            Vec3 localPosition = capsulePoints[0] + manifold.Normal * distance0;
            ContactPoint point;
            point.RelContactPoint2 = rotate(localPosition, t2->Orientation);
            point.RelContactPoint1 = point.RelContactPoint2 + relativePos;
            point.Penetration = -distance0 + radius;
            manifold.Points[0] = point;

            manifold.NumPoints++;
        }

        Real distance1 = dot(capsulePoints[1], face->Normal) - face->D;
        if (distance1 <= 0) {
            manifold.PointIds[manifold.NumPoints] = ContactPointId{0};
            Vec3 localPosition = capsulePoints[1] + manifold.Normal * distance1;
            ContactPoint point;
            point.RelContactPoint2 = rotate(localPosition, t2->Orientation);
            point.RelContactPoint1 = point.RelContactPoint2 + relativePos;
            point.Penetration = -distance1 + radius;
            manifold.Points[manifold.NumPoints] = point;

            manifold.NumPoints++;
        }

        assert(manifold.NumPoints > 0);
        contactSet->ManifoldIds.push_back(manifoldId);
        contactSet->Manifolds.push_back(manifold);
    } else {
        // Edge contact, only a single manifold point is needed
        ManifoldId manifoldId;
        Manifold manifold;

        manifoldId.Body1 = body1;
        manifoldId.Body2 = body2;
        manifold.NumPoints = 1;

        HalfEdge *edge = &hull->HalfEdges[miniumEdgeIdx];

        Vec3 v1 = hull->Vertices[edge->Vertex1];
        Vec3 v2 = hull->Vertices[edge->Vertex2];

        // project lines onto the plane
        // derrivation from https://paulbourke.net/geometry/pointlineplane/
        Vec3 p13 = v1 - capsulePoints[0];
        Vec3 p43 = capsulePoints[1] - capsulePoints[0];
        Vec3 p21 = v2 - v1;

        Real d1343 = dot(p13, p43);
        Real d4321 = dot(p43, p21);
        Real d1321 = dot(p13, p21);
        Real d4343 = dot(p43, p43);
        Real d2121 = dot(p21, p21);

        // Query Distance is negative since there is a penetration, query normal points from 1 to 2, but the edge should
        // be passed the current edge, thus we need to move back. The two negatives cancel out.
        Real t = (d1343 * d4321 - d1321 * d4343) / (d2121 * d4343 - d4321 * d4321);
        Vec3 p = v1 + (t * p21) - (0.5f * minimumEdgeNormal * minimumEdgeDistance);

        manifold.PointIds[0] = ContactPointId{0};
        manifold.Normal = -minimumEdgeNormal;
        ContactPoint point;
        point.RelContactPoint2 = rotate(p, t2->Orientation);
        point.RelContactPoint1 = t2->Pos + point.RelContactPoint2 - t1->Pos;
        point.Penetration = minimumEdgeDistance;
        manifold.Points[0] = point;

        contactSet->ManifoldIds.push_back(manifoldId);
        contactSet->Manifolds.push_back(manifold);
    }
}

void CreateCapsuleAndHullManifold(BodyId body1, BodyId body2, Transform *transform1, Transform *transform2,
                                  BCapsule *cap, ConvexHull *hull, Vec3 cp1, Vec3 cp2, WorldContactSet *contactSet) {

    Vec3 d = cp2 - cp1;
    Real distance = length(d);
    Real penetration = cap->Radius - distance;
    if (penetration < 0.0) {
        return;
    }

    ManifoldId outManifoldId;
    Manifold outManifold;

    outManifoldId.Body1 = body1;
    outManifoldId.Body2 = body2;

    outManifold.Normal = rotate(d / distance, transform2->Orientation);
    outManifold.NumPoints = 1;
    outManifold.PointIds[0] = ContactPointId{0};

    ContactPoint point{};
    // the hull contact point is the feature being used in the relative global space.
    // the capsule contact point is from the center of the capsule to that position
    point.RelContactPoint2 = rotate(cp2, transform2->Orientation);
    point.RelContactPoint1 = transform2->Pos + point.RelContactPoint2 - transform1->Pos;
    point.Penetration = penetration;

    outManifold.Points[0] = point;
    contactSet->ManifoldIds.push_back(outManifoldId);
    contactSet->Manifolds.push_back(outManifold);
}

Vec3 CapsuleSupportPoint(ConvexHull *hull, Vec3 searchDir, Vec3 capsulePoints[2], u8 *capsuleIdx, u8 *hullIdx) {
    Vec3 hullPoint = SupportPoint(hull, searchDir, hullIdx);

    Real cp0 = dot(capsulePoints[0], -searchDir);
    Real cp1 = dot(capsulePoints[1], -searchDir);
    if (cp0 >= cp1) {
        *capsuleIdx = 0;
        return hullPoint - capsulePoints[0];
    } else {
        *capsuleIdx = 1;
        return hullPoint - capsulePoints[1];
    }
}

void CapsuleAndHullTest(u32 body1, u32 body2, Transform *transform1, Transform *transform2, BCapsule *cap, BHull *bhull,
                        WorldContactSet *contactSet) {
    // First run GJK to find the feature with the least penetration on the
    // interior of the capsule
    ConvexHull *hull = bhull->Hull;

    Vec3 cN = rotate(Vec3(0.0, cap->HalfLength, 0.0), transform1->Orientation);
    Vec3 capsulePoints[2]{transform2->ApplyInverse(transform1->Pos + cN),
                          transform2->ApplyInverse(transform1->Pos - cN)};
    Vec3 firstVertex = hull->Vertices[0] - capsulePoints[0];

    GJKData data{.Simplex{firstVertex}, .SearchDir{-firstVertex}, .NumSimplexPoints = 1, .ShouldTerminate = false};
    u8 capsulePointIdx;
    u8 hullPointIdx;
    u32 stepIdx = 0;
    u32 const MAX_STEPS = 256;
    do {
        Vec3 supportPoint = CapsuleSupportPoint(hull, data.SearchDir, capsulePoints, &capsulePointIdx, &hullPointIdx);
        u16 pid = capsulePointIdx;
        pid |= static_cast<u16>(hullPointIdx) << 8;

        if (GJKIndexInOldSimplexIds(pid, &data)) {
            switch (data.NumSimplexPoints) {
            case 1: {
                // Will just be 1 point of the capsule and 1 point of the hull
                Vec3 cp1 = capsulePoints[data.SimplexIds[0] & 0x00ff];
                Vec3 cp2 = hull->Vertices[(data.SimplexIds[0] & 0xff00) >> 8];
                CreateCapsuleAndHullManifold(body1, body2, transform1, transform2, cap, hull, cp1, cp2, contactSet);
            }
                return;
            case 2: {
                // Use bary-centric coordinates and use those to interpolate the contact points
                Real params[2];
                GetOriginLineBarycentricCoordinates(data.Simplex[0], data.Simplex[1], &params[0], &params[1]);
                Vec3 cp1{0};
                Vec3 cp2{0};
                for (int i = 0; i < data.NumSimplexPoints; ++i) {
                    cp1 += params[i] * capsulePoints[data.SimplexIds[i] & 0x00ff];
                    cp2 += params[i] * hull->Vertices[(data.SimplexIds[i] & 0xff00) >> 8];
                }

                CreateCapsuleAndHullManifold(body1, body2, transform1, transform2, cap, hull, cp1, cp2, contactSet);
            }
                return;
            case 3: {
                // Use bary-centric coordinates and interpolate those points
                u8 hullIds[3]{};
                u8 capsuleIds[3]{};
                for (int i = 0; i < 3; ++i) {
                    capsuleIds[i] = data.SimplexIds[i] & 0x00ff;
                    hullIds[i] = (data.SimplexIds[i] & 0xff00) >> 8;
                }

                // 3 seperate hull vertices and 2 differnet capsuleIds
                if (hullIds[0] != hullIds[1] && hullIds[0] != hullIds[2] && hullIds[1] != hullIds[2]) {
                    // find face
                    u8 faceVertex[MAX_NUM_FEATURES_CONVEX_HULL]{};
                    for (u32 edgeIdx = 0; edgeIdx < hull->NumHalfEdges; ++edgeIdx) {
                        HalfEdge *edge = &hull->HalfEdges[edgeIdx];
                        u8 vertex = edge->Vertex1;
                        if (vertex == hullIds[0] || vertex == hullIds[1] || vertex == hullIds[2]) {
                            faceVertex[edge->FaceId]++;
                        }
                    }

                    u8 faceIdx;
                    for (faceIdx = 0; faceIdx < hull->NumFaces; ++faceIdx) {
                        if (faceVertex[faceIdx] == 3) {
                            break;
                        }
                    }
                    assert(faceIdx < hull->NumFaces);

                    ClipCapsuleLine(capsulePoints, capsulePoints[1] - capsulePoints[0], faceIdx, hull);
                    CreateCapsuleAndHullManifold(body1, body2, transform1, transform2, cap, hull, capsulePoints,
                                                 &hull->Faces[faceIdx], contactSet);

                } else {

                    Vec3 normal = GetTriangleNormal(data.Simplex[0], data.Simplex[1], data.Simplex[2]);
                    Real params[3];
                    GetOriginTriangleBarycentricCoordinates(normal, data.Simplex[0], data.Simplex[1], data.Simplex[2],
                                                            &params[0], &params[1], &params[2]);
                    Vec3 cp1{0};
                    Vec3 cp2{0};
                    for (int i = 0; i < data.NumSimplexPoints; ++i) {
                        cp1 += params[i] * capsulePoints[capsuleIds[i]];
                        cp2 += params[i] * hull->Vertices[hullIds[i]];
                    }

                    CreateCapsuleAndHullManifold(body1, body2, transform1, transform2, cap, hull, cp1, cp2, contactSet);
                }
            }
                return;
            case 4:
                assert(false);
            }
        }

        GJKAddSupportPoint(&data, supportPoint, pid);
        GJKIteration(&data);
        ++stepIdx;

        assert(stepIdx < MAX_STEPS); //Shouldn't happen, we want to know when this may occur
    } while (data.NumSimplexPoints != 4 && stepIdx < MAX_STEPS);

    // Deep intersection, fall back to SAT
    HandleCapsuleAndHullDeepIntersection(body1, body2, transform1, transform2, capsulePoints, hull, cap->Radius, contactSet);
}
} // namespace ce
