#include "SAT.h"
#include "World.h"
#include "Math.hpp"
#include <cstring>

namespace ce {

    struct FaceQuery {
        Real Distance;
        u8 FaceIndex;
    };

    FaceQuery QueryFaceDirections(ConvexHull* g1, ConvexHull* g2) {
        FaceQuery q { .Distance = -MAX_REAL };

        for (u32 faceIdx = 0; faceIdx < g1->NumFaces; ++faceIdx) {
            Face plane = g1->Faces[faceIdx];

            Vec3 support = g2->GetSupport(-plane.Normal);

            Real distance = dot(plane.Normal, support) - plane.D;

            if (distance > q.Distance) {
                q.Distance = distance;
                q.FaceIndex = faceIdx;
            }
        }

        return q;
    }

    struct EdgeQuery {
        Vec3 Normal;
        Real Distance;
        u8 EdgeIndex1;
        u8 EdgeIndex2;
    };

    EdgeQuery QueryEdgeDirections(ConvexHull* g1, ConvexHull* g2) {
        EdgeQuery q { .Distance = -MAX_REAL };
        for (u32 edgeIdx1 = 0; edgeIdx1 < g1->NumHalfEdges; edgeIdx1 += 2) {
            for (u32 edgeIdx2 = 0; edgeIdx2 < g2->NumHalfEdges; edgeIdx2 += 2) {

                Vec3 edge1Dir = g1->GetDirection(edgeIdx1);
                Vec3 edge2Dir = g2->GetDirection(edgeIdx2);

                Vec3 axis = cross(edge1Dir, edge2Dir);
                Real smag = dot(axis, axis);
                if (smag < 1e-6) {
                    // parallel edges, skip
                    continue;
                }

                HalfEdge* edge1 = &g1->HalfEdges[edgeIdx1];
                Vec3 edge1Face1 = g1->Faces[edge1->FaceId].Normal;
                Vec3 edge1Face2 = g1->GetAdjFace(edgeIdx1)->Normal;

                HalfEdge* edge2 = &g2->HalfEdges[edgeIdx2];
                Vec3 edge2Face1 = -g2->Faces[edge2->FaceId].Normal;
                Vec3 edge2Face2 = -g2->GetAdjFace(edgeIdx2)->Normal;

                Vec3 hemispherePlane = cross(edge1Face2, edge2Face1);
                // @todo re-evaliate whether this logic is fine , I added fuzziness to the hemisphere test because there seems to
                // be an issue when dealing with two edges where the face normals are almost the same. this causes the
                // hemisphere test to be almost 0, slightly positive. I believe this is because of floating point error. At the moment,
                // assuming if it's almost zero, then it's good enough
                bool isMinkowskiFace = ((dot(edge1Dir, edge2Face1) * dot(edge1Dir, edge2Face2)) < 0) &&
                        ((dot(edge2Dir, edge1Face1) * dot(edge2Dir, edge1Face2)) < 0) &&
                        ((dot(hemispherePlane, edge1Face1) * dot(hemispherePlane, edge2Face2)) > 1e-6);
                if (isMinkowskiFace) {
                    if (dot(axis, g1->Vertices[edge1->Vertex1]) < 0.0f) {
                        axis = -axis;
                    }

                    // Need to normalize the axis to calculate the proper distance, uptil this point we only cared about the sign
                    axis = axis / sqrt(smag);
                    Real distance = dot(axis, g2->Vertices[edge2->Vertex1] - g1->Vertices[edge1->Vertex1]);
                    if (distance > q.Distance) {
                        q.Distance = distance;
                        q.EdgeIndex1 = edgeIdx1;
                        q.EdgeIndex2 = edgeIdx2;
                        q.Normal = axis;
                    }
                }
            }
        }

        return q;
    }

    struct SatPoint {
        Vec3 Point {};
        Real Distance { 0.0f };
        u16 ClipEdges {};
        u8 SourcePointIdx { 0 };
        int totalEdges = 0;

        void AddClipEdge(u8 edge) {
            ClipEdges = (ClipEdges << 7) | edge;
            ++totalEdges;
        }
    };

#define ARR_PTR(type, size, name, name2) type name2[size]; \
    type * name = name2;

    void ClipAndFilterIncidentPoints(SatPoint* incPoints, FaceQuery* query, ConvexHull* g1, ConvexHull* g2, u8& numIncPoints, Face* refFace) {


        // For each reference plane, clip the edges
        u8 edgeIdx = g1->FirstEdgeIndex[query->FaceIndex];
        u8 currentEdgeIdx = edgeIdx;
        ARR_PTR(SatPoint, 254, incPointsTmp, ip);
        u8 numIncPointsTmp = 0;
        do {
            // @important this might not be correct, we shouldn't clip based on adj faces, but the side planes of the reference face,
            // we can get this normal via cross product with the edge and the ref face normal
            Face* plane = g1->GetAdjFace(currentEdgeIdx);
            for (u8 pointIdx = 0; pointIdx < numIncPoints; ++pointIdx) {
                incPoints[pointIdx].Distance = dot(plane->Normal, incPoints[pointIdx].Point) - plane->D;
            }

            for (u8 pointIdx = 0; pointIdx < numIncPoints; ++pointIdx) {
                u8 pointIdx2 = (pointIdx + 1) % numIncPoints;
                // @todo: Speed up by removing last check outside loop
                bool p1 = incPoints[pointIdx].Distance <= 0.0f;
                bool p2 = incPoints[pointIdx2].Distance <= 0.0f;

                // @todo: Check to see if this can be sped up by using a swtich statement
                // Note: We do nothing if both points are outside
                // Probably can be sped up by just doing 2 checks
                if (p1 && p2) {
                    incPointsTmp[numIncPointsTmp++] = incPoints[pointIdx];
                } else if (p1) {
                    incPointsTmp[numIncPointsTmp++] = incPoints[pointIdx];

                    // clip second point
                    Real a1 = abs(incPoints[pointIdx].Distance);
                    Real a2 = abs(incPoints[pointIdx2].Distance);
                    Real a = a1 / (a1 + a2);
                    incPointsTmp[numIncPointsTmp].Point = incPoints[pointIdx].Point + (a * (incPoints[pointIdx2].Point - incPoints[pointIdx].Point));
                    incPointsTmp[numIncPointsTmp].AddClipEdge(currentEdgeIdx);
                    ++numIncPointsTmp;
                } else if (p2) {
                    // clip First point, no need to add second point, that will be added when it's processed
                    Real a1 = abs(incPoints[pointIdx].Distance);
                    Real a2 = abs(incPoints[pointIdx2].Distance);
                    Real a = a1 / (a1 + a2);
                    incPointsTmp[numIncPointsTmp].Point = incPoints[pointIdx].Point + (a * (incPoints[pointIdx2].Point - incPoints[pointIdx].Point));
                    incPointsTmp[numIncPointsTmp].AddClipEdge(currentEdgeIdx);
                    ++numIncPointsTmp;
                }
            }

            numIncPoints = numIncPointsTmp;
            numIncPointsTmp = 0;
            std::swap(incPoints, incPointsTmp);

            currentEdgeIdx = g1->HalfEdges[currentEdgeIdx].NextEdge;
        } while (currentEdgeIdx != edgeIdx);

        // For each clipped point; only take the points which lie under the reference face
        for (u32 pointIdx = 0; pointIdx < numIncPoints; ++pointIdx) {
            Real d = dot(refFace->Normal, incPoints[pointIdx].Point) - refFace->D;
            if (d <= 0.0f) {
                incPointsTmp[numIncPointsTmp] = incPoints[pointIdx];
                incPointsTmp[numIncPointsTmp].Distance = d;
                ++numIncPointsTmp;
            }
        }

        numIncPoints = numIncPointsTmp;
        // If the final points is in the temporary array, then copy it over to the incPoints array that the caller owns
        if (incPointsTmp == ip) {
            std::memcpy(incPoints, incPointsTmp, sizeof(SatPoint) * 254);
        }
    }

    void ReduceManifold(SatPoint* incPoints, u8 numIncPoints, Vec3 faceNormal, SatPoint* outPoints) {
        outPoints[0] = incPoints[0];

        // second point, furthest away from the first point
        Real maxDistance = 0.0f;
        for (u32 i = 1; i < numIncPoints; ++i) {
            Vec3 e = incPoints[i].Point - incPoints[0].Point;
            Real d = dot(e, e);
            if (d > maxDistance) {
                outPoints[1] = incPoints[i];
                maxDistance = d;
            }
        }

        // third point, max area of the triangle made by the 3 points. Uses normal so we don't need to square root and we get the signed distance
        Real maxArea = 0.0f;
        for (u32 i = 0; i < numIncPoints; ++i) {
            Vec3 e1 = incPoints[i].Point - outPoints[0].Point;
            Vec3 e2 = incPoints[i].Point - outPoints[1].Point;
            Real area = dot(cross(e1, e2), faceNormal);
            if (area > maxArea) {
                outPoints[2] = incPoints[i];
                maxArea = area;
            }
        }

        // fourth point
        Real minArea = 0.0f;
        for (u32 i = 0; i < numIncPoints; ++i) {
            Vec3& p = incPoints[i].Point;
            Vec3 e1 = p - outPoints[0].Point;
            Vec3 e2 = p - outPoints[1].Point;
            Real area = dot(cross(e1, e2), faceNormal);
            if (area < minArea) {
                outPoints[3] = incPoints[i];
                minArea = area;
            }

            e1 = p - outPoints[1].Point;
            e2 = p - outPoints[2].Point;
            area = dot(cross(e1, e2), faceNormal);
            if (area < minArea) {
                outPoints[3] = incPoints[i];
                minArea = area;
            }

            e1 = p - outPoints[2].Point;
            e2 = p - outPoints[0].Point;
            area = dot(cross(e1, e2), faceNormal);
            if (area < minArea) {
                outPoints[3] = incPoints[i];
                minArea = area;
            }
        }
    }

    SatId CreateFaceContactId(u8 refFaceId, u8 incFaceId, u8 pointId, u8 clipId) {
        SatId id;
        id.FaceInfo.RefFaceId = refFaceId;
        id.FaceInfo.IncFaceId = incFaceId;
        id.FaceInfo.PointId = pointId;
        id.FaceInfo.ClipEdgeId = clipId;
        return id;
    }

    SatId CreateEdgeContactId(u8 edgeId1, u8 edgeId2) {
        SatId id;
        id.EdgeInfo.EdgeId1 = edgeId1;
        id.EdgeInfo.EdgeId2 = edgeId2;
        return id;
    }

    /**
     * The first polyhedra has the reference face, while the second has the incident face.
     */
    SatResult FaceContactGeneration(FaceQuery* query, ConvexHull* g1, ConvexHull* g2) {
        // Get the reference face
        Face refFace = g1->Faces[query->FaceIndex];
        Face incFace;
        u8 incFaceId;
        Real antiParallelCoef = MAX_REAL;
        for (u32 incFaceIdx = 0; incFaceIdx < g2->NumFaces; ++incFaceIdx) {
            Face face = g2->Faces[incFaceIdx];
            Real coef = dot(refFace.Normal, face.Normal);
            if (coef < antiParallelCoef) {
                incFace = face;
                incFaceId = incFaceIdx;
                antiParallelCoef = coef;
            }
        }

        // Get the incident points
        // There is a maximum of 127 points around a polygon, each point should maximally generate
        // two other points (is this correct?), so 256 sized working set is sufficient.
        SatPoint incPoints[MAX_NUM_FEATURES_CONVEX_HULL * 2] {};

        u8 numIncPoints = 0;
        u8 edgeIdx = g2->FirstEdgeIndex[incFaceId];
        u8 currentEdgeIdx = edgeIdx;
        do {
            u8 sourceVertex = g2->HalfEdges[currentEdgeIdx].Vertex1;
            incPoints[numIncPoints].Point = g2->Vertices[sourceVertex];
            incPoints[numIncPoints].SourcePointIdx = sourceVertex;
            currentEdgeIdx = g2->HalfEdges[currentEdgeIdx].NextEdge;
            ++numIncPoints;
        } while (currentEdgeIdx != edgeIdx);

        ClipAndFilterIncidentPoints(incPoints, query, g1, g2, numIncPoints, &refFace);

        SatResult result { .Axis = refFace.Normal, .Ids = { 0, 0, 0, 0 } };
        if (numIncPoints < 4) {
            result.ManifoldSize = numIncPoints;

            for (u8 i = 0; i < numIncPoints; ++i) {
                result.Ids[i] = CreateFaceContactId(query->FaceIndex, incFaceId, i, incPoints[i].ClipEdges);
                result.Points[i] = incPoints[i].Point;
                result.Penetrations[i] = -incPoints[i].Distance;
            }

            return result;
        }

        // Reduce the number of contact points to 4;
        SatPoint resultPoints[4];
        ReduceManifold(incPoints, numIncPoints, refFace.Normal, resultPoints);

        result.ManifoldSize = 4;
        for (u8 i = 0; i < 4; ++i) {
            assert(resultPoints[i].totalEdges <= 2);

            result.Ids[i] = CreateFaceContactId(query->FaceIndex, incFaceId, i, resultPoints[i].ClipEdges);
            result.Points[i] = resultPoints[i].Point;
            result.Penetrations[i] = -resultPoints[i].Distance;
        }

        //assert
        for (u8 i = 0; i < 4; ++i) {
            for (u8 j = i + 1; j < 4; ++j) {
                //assert(resultPoints[i].ClipEdges != resultPoints[j].ClipEdges);
            }
        }

        return result;
    }

    SatResult EdgeContactGeneration(ConvexHull* g1, ConvexHull* g2, EdgeQuery* query) {
        // Find the cloest point between the two edges
        HalfEdge* edge1 = &g1->HalfEdges[query->EdgeIndex1];
        HalfEdge* edge2 = &g2->HalfEdges[query->EdgeIndex2];

        Vec3 v1 = g1->Vertices[edge1->Vertex1];
        Vec3 v2 = g1->Vertices[edge1->Vertex2];
        Vec3 v3 = g2->Vertices[edge2->Vertex1];
        Vec3 v4 = g2->Vertices[edge2->Vertex2];

        // project lines onto the plane
        // derrivation from https://paulbourke.net/geometry/pointlineplane/
        Vec3 p13 = v1 - v3;
        Vec3 p43 = v4 - v3;
        Vec3 p21 = v2 - v1;

        Real d1343 = dot(p13, p43);
        Real d4321 = dot(p43, p21);
        Real d1321 = dot(p13, p21);
        Real d4343 = dot(p43, p43);
        Real d2121 = dot(p21, p21);

        // Query Distance is negative since there is a penetration, query normal points from 1 to 2, but the edge should be passed the current edge,
        // thus we need to move back. The two negatives cancel out.
        Real t = (d1343*d4321 - d1321*d4343) / (d2121*d4343 - d4321*d4321);
        Vec3 p = v1 + (t * p21) + (0.5f * query->Normal * query->Distance);

        return SatResult {
            .Axis = query->Normal,
            .ManifoldSize = 1,
            .Ids = { CreateEdgeContactId(query->EdgeIndex1, query->EdgeIndex2) },
            .Penetrations = { -query->Distance },
            .Points = { p },
            .Body1IsRef = true,
        };
    }

    // Assuming here everything is global coordinates
    bool ConvexSat(ConvexHull g1, ConvexHull g2, Transform* relativeTransform, SatResult* result) {
        // Convert the vertices of the second body to be in the first bodies coordinate space
        Vec3 outVertices2[127];
        for (u32 i = 0; i < g2.NumVertices; ++i) {
            outVertices2[i] = relativeTransform->Apply(g2.Vertices[i]);
        }
        g2.Vertices = outVertices2;
        FaceQuery faceQuery1 = QueryFaceDirections(&g1, &g2);
        if (faceQuery1.Distance > 0.0f) {
            return false;
        }

        // Convert the faces of the second body to be in the first bodies coordinate space
        Face outFaces2[127];
        for (u32 i = 0; i < g2.NumFaces; ++i) {
            outFaces2[i] = g2.Faces[i].Apply(relativeTransform);
        }
        g2.Faces = outFaces2;
        FaceQuery faceQuery2 = QueryFaceDirections(&g2, &g1);
        if (faceQuery2.Distance > 0.0f) {
            return false;
        }

        EdgeQuery edgeQuery = QueryEdgeDirections(&g1, &g2);
        if (edgeQuery.Distance > 0.0f) {
            return false;
        }

        // bias face contacts for temporal coherence
        static Real const TOL_ABS = 0.01f;
        static Real const TOL_REL = 1.05f;

        // Distance should be negative, so multiplying by a value greater than 1 will bias it negatively toward a deeper penetration
        // while adding a positive value, will bias something towards a shallow penetration
#define LOWER_PEN(q1, q2) (TOL_ABS + q1.Distance) > (TOL_REL * q2.Distance)

        // find the minimum penetration/greatest distance, prioritize Face A > Face B > Edge
        if (LOWER_PEN(faceQuery2, edgeQuery)) {
            if (LOWER_PEN(faceQuery1, faceQuery2)) {
                // FaceQuery1
                *result = FaceContactGeneration(&faceQuery1, &g1, &g2);
                result->Body1IsRef = true;
            } else {
                // FaceQuery 2
                *result = FaceContactGeneration(&faceQuery2, &g2, &g1);
                result->Body1IsRef = false;
            }
        } else {
            if (LOWER_PEN(faceQuery1, edgeQuery)) {
                // FaceQuery1
                *result = FaceContactGeneration(&faceQuery1, &g1, &g2);
                result->Body1IsRef = true;
            } else {
                // EdgeQuery
                *result = EdgeContactGeneration(&g1, &g2, &edgeQuery);
                result->Body1IsRef = true;
            }
        }

        return true;
    }
}
