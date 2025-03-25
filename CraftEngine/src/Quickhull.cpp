#include "Quickhull.h"
#include <assert.h>
#include <stdlib.h>
#include "Math.hpp"
#include <string>

namespace ce {
typedef struct {
    uint8_t IsMarked[(2 * MAX_NUM_FACES) / 8];
} qhMarkedBitSet;

#define QH_GET_MARKED(bitset, x) (bitset->IsMarked[x >> 2] >> ((x << 1) & 7)) & 3
#define QH_SET_MARKED(bitset, x, v) bitset->IsMarked[x >> 2] = bitset->IsMarked[x >> 2] | (v << ((x << 1) & 7))

typedef struct {
    uint8_t IsVisited[(2 * MAX_NUM_EDGES) / 8];
} qhVisitedEdgeBitSet;

#define QH_IS_EDGE_VISITED(bitset, x) (bitset.IsVisited[x >> 3] & (1 << (x & 7)))
#define QH_SET_EDGE_VISITED(bitset, x) bitset.IsVisited[x >> 3] = bitset.IsVisited[x >> 3] | (1 << (x & 7))

void qhDeleteFace(qhFacesList *facesList, qhIndex faceIdx) {
    qhFace *face = facesList->Faces + faceIdx;
    if (faceIdx == facesList->Head) {
        facesList->Head = face->Next;
    } else if (faceIdx == facesList->Tail) {
        facesList->Tail = face->Prev;
    } else {
        facesList->Faces[face->Prev].Next = face->Next;
        facesList->Faces[face->Next].Prev = face->Prev;
    }
}

// Depth-first search of faces that are "visible" from vertex v.
// Removes visible faces from the faces list, building up a list of "horizon edges."
// Orphaned vertices (their faces were removed) are gathered into orphanList.
void qhDFS(qhHalfEdgeIndex *edgeStack, qhMarkedBitSet *faceMarkSet, qhHalfEdge *edges, qhHalfEdgeIndex *horizonEdges,
           int *numHorizonEdges, qhFacesList *facesList, qhFace *startingFace, Vec3 v, qhConflictList *orphanList,
           qhConflictListItem *clist) {
    qhFace *faces = facesList->Faces;
    qhHalfEdgeIndex startingEdgeIdx = startingFace->HalfEdge;
    qhHalfEdgeIndex currentEdgeIdx = startingEdgeIdx;
    edgeStack[0] = startingEdgeIdx;
    int edgeStackSize = 1;

    QH_SET_MARKED(faceMarkSet, edges[startingEdgeIdx].Face, 1);
    qhDeleteFace(facesList, edges[startingEdgeIdx].Face);
    *orphanList = startingFace->CList;

    while (edgeStackSize > 0) {
        qhHalfEdge *edge = edges + currentEdgeIdx;
        qhHalfEdge *twinHalfEdge = edges + edge->Twin;

        switch (QH_GET_MARKED(faceMarkSet, twinHalfEdge->Face)) {
        case 0: {
            qhFace *face = faces + twinHalfEdge->Face;
            if (dot(face->Normal, (v - face->CenterPoint)) <= 0.0f) {
                // This face is on the horizon
                QH_SET_MARKED(faceMarkSet, twinHalfEdge->Face, 3);
                horizonEdges[(*numHorizonEdges)++] = currentEdgeIdx;
                currentEdgeIdx = edge->Next;
            } else {
                // Visible face, delete it
                QH_SET_MARKED(faceMarkSet, twinHalfEdge->Face, 1);
                edgeStack[edgeStackSize++] = edge->Twin;
                currentEdgeIdx = twinHalfEdge->Next;
                qhDeleteFace(facesList, twinHalfEdge->Face);

                // Merge orphaned vertices
                qhFace *f = faces + twinHalfEdge->Face;
                if (orphanList->Size > 0) {
                    if (f->CList.Size > 0) {
                        clist[orphanList->Tail].Next = f->CList.Head;
                        orphanList->Tail = f->CList.Tail;
                        orphanList->Size += f->CList.Size;
                    }
                } else {
                    *orphanList = f->CList;
                }
            }
        } break;
        case 1: {
            // Already marked visible
            currentEdgeIdx = edge->Next;
        } break;
        case 2:
            // Should never happen
            exit(10);
        case 3: {
            // Already marked as horizon
            horizonEdges[(*numHorizonEdges)++] = currentEdgeIdx;
            currentEdgeIdx = edge->Next;
        } break;
        }

        // If we reach an edge that is the same as top-of-stack, pop stack
        while (edgeStackSize > 0 && edgeStack[edgeStackSize - 1] == currentEdgeIdx) {
            --edgeStackSize;
            currentEdgeIdx = edges[edges[currentEdgeIdx].Twin].Next;
        }
    }
}

// Decide if two faces should merge
int qhShouldMergeFaces(qhFace *f1, qhFace *f2) {
    float ep = 0.0;
    float i, j;

    Vec3 v1 = f1->CenterPoint;
    Vec3 v2 = f2->CenterPoint;

#define MAX_ABS(a, b)  \
    i = fabs(a);       \
    j = fabs(b);       \
    i > j ? i : j;

    ep += MAX_ABS(v1.x, v2.x);
    ep += MAX_ABS(v1.y, v2.y);
    ep += MAX_ABS(v1.z, v2.z);
    ep = 3 * FLT_EPSILON * ep;

    Vec3 d = v1 - v2;
    return fabs(dot(d, f1->Normal)) < ep && fabs(dot(d, f2->Normal)) < ep;
}

// Try to fix topology in edge merges when a vertex might only have two faces
void FixTopology(qhFacesList *facesList, qhHalfEdgeIndex mainFirstEdgeIdx, qhHalfEdgeIndex mainSecondEdgeIdx,
                 qhIndex faceIdx, qhHalfEdge *edges) {
    qhHalfEdge *mainFirstEdge = edges + mainFirstEdgeIdx;
    qhHalfEdge *mainSecondEdge = edges + mainSecondEdgeIdx;

    qhHalfEdgeIndex otherFirstEdgeIdx = mainFirstEdge->Twin;
    qhHalfEdge *otherFirstEdge = edges + otherFirstEdgeIdx;

    qhHalfEdgeIndex otherSecondEdgeIdx = mainSecondEdge->Twin;
    qhHalfEdge *otherSecondEdge = edges + otherSecondEdgeIdx;

    if (otherFirstEdge->Face == otherSecondEdge->Face) {
        // If it's a triangle face, we merge edges accordingly
        qhHalfEdgeIndex otherThirdEdgeIdx = otherFirstEdge->Next;
        qhHalfEdge *otherThirdEdge = edges + otherThirdEdgeIdx;
        qhHalfEdgeIndex otherFourthEdgeIdx = otherThirdEdge->Next;

        if (otherFourthEdgeIdx == otherSecondEdgeIdx) {
            // Triangle case
            qhIndex otherFace = otherSecondEdge->Face;
            qhHalfEdgeIndex sourceIdx = edges[mainFirstEdgeIdx].Prev;
            qhHalfEdgeIndex sinkIdx = edges[mainSecondEdgeIdx].Next;

            otherThirdEdge->Prev = sourceIdx;
            otherThirdEdge->Next = sinkIdx;
            otherThirdEdge->Face = faceIdx;

            qhHalfEdge *source = edges + sourceIdx;
            qhHalfEdge *sink = edges + sinkIdx;

            source->Next = otherThirdEdgeIdx;
            sink->Prev = otherThirdEdgeIdx;

            qhDeleteFace(facesList, otherFace);
        } else {
            // General case
            otherSecondEdge->Next = otherThirdEdgeIdx;
            otherThirdEdge->Prev = otherSecondEdgeIdx;
            otherSecondEdge->Twin = mainFirstEdgeIdx;
            facesList->Faces[otherSecondEdge->Face].HalfEdge = otherSecondEdgeIdx;

            mainFirstEdge->Twin = otherSecondEdgeIdx;
            mainFirstEdge->Next = mainSecondEdge->Next;
            edges[mainSecondEdge->Next].Prev = mainFirstEdgeIdx;
            facesList->Faces[mainFirstEdge->Face].HalfEdge = mainFirstEdgeIdx;
        }
    }
}

// Merge the second face into the first face
qhHalfEdgeIndex qhMergeFaces(qhIndex face1Idx, qhIndex face2Idx, qhFace *f1, qhFace *f2, qhHalfEdge *edge,
                             qhHalfEdge *twinEdge, qhFacesList *facesList, Vec3 *vertices, qhHalfEdge *edges,
                             qhConflictListItem *clist) {
    qhHalfEdgeIndex prevEdgeIdx = edge->Prev;
    qhHalfEdgeIndex nextEdgeIdx = edge->Next;
    qhHalfEdgeIndex twinPrevIdx = twinEdge->Prev;
    qhHalfEdgeIndex twinNextIdx = twinEdge->Next;

    f1->HalfEdge = prevEdgeIdx;

    // Update all edges of f2 to belong to f1
    qhHalfEdgeIndex startingEdge = f2->HalfEdge;
    qhHalfEdgeIndex currentEdge = startingEdge;
    do {
        qhHalfEdge *he = edges + currentEdge;
        he->Face = face1Idx;
        currentEdge = he->Next;
    } while (currentEdge != startingEdge);

    edges[prevEdgeIdx].Next = twinNextIdx;
    edges[nextEdgeIdx].Prev = twinPrevIdx;
    edges[twinPrevIdx].Next = nextEdgeIdx;
    edges[twinNextIdx].Prev = prevEdgeIdx;

    // Recompute center, normal for merged face
    Vec3 n, p;
    n.x = n.y = n.z = 0.f;
    p.x = p.y = p.z = 0.f;

    startingEdge = f1->HalfEdge;
    qhHalfEdgeIndex e1 = startingEdge;
    qhHalfEdgeIndex e2 = edges[e1].Next;
    int vCount = 0;

    do {
        Vec3 v1 = vertices[edges[e1].Tail];
        Vec3 v2 = vertices[edges[e2].Tail];

        // Newell's method summation for normal
        n.x += (v1.y - v2.y) * (v1.z + v2.z);
        n.y += (v1.z - v2.z) * (v1.x + v2.x);
        n.z += (v1.x - v2.x) * (v1.y + v2.y);

        p = p + v1;
        vCount++;
        e1 = e2;
        e2 = edges[e2].Next;
    } while (e1 != startingEdge);

    p = p * (1.0f / vCount);
    f1->CenterPoint = p;
    f1->Normal = normalize(n);

    // Merge the conflict lists
    qhConflictList *f1List = &f1->CList;
    qhConflictList *f2List = &f2->CList;
    if (f1List->Size > 0) {
        if (f2List->Size > 0) {
            clist[f1List->Tail].Next = f2List->Head;
            f1List->Tail = f2List->Tail;
            f1List->Size += f2List->Size;
            if (f1List->MaxDistance < f2List->MaxDistance) {
                f1List->MaxDistance = f2List->MaxDistance;
                f1List->MaxVertex = f2List->MaxVertex;
            }
        }
    } else {
        *f1List = *f2List;
    }

    qhDeleteFace(facesList, face2Idx);

    // Fix any topological errors for the two vertices connecting the faces
    FixTopology(facesList, prevEdgeIdx, twinNextIdx, face1Idx, edges);
    FixTopology(facesList, twinPrevIdx, nextEdgeIdx, face1Idx, edges);

    return twinNextIdx;
}

qhFacesList qhCreateHull(int numVertices, Vec3 *vertices, qhFace *faces, qhHalfEdge *edges, qhHalfEdgeIndex *edgeStack,
                         qhConflictListItem *clist, qhHalfEdgeIndex *horizonEdges, IterationCallback callback) {
    float maxValue = vertices[0].x;
    float minValue = vertices[0].x;
    qhIndex v1Idx = 0;
    qhIndex v2Idx = 0;

    // Find max and min along x-axis
    for (int i = 1; i < numVertices; ++i) {
        float d = vertices[i].x;
        if (d > maxValue) {
            maxValue = d;
            v1Idx = i;
        } else if (d < minValue) {
            minValue = d;
            v2Idx = i;
        }
    }

    Vec3 v1 = vertices[v1Idx];
    Vec3 v2 = vertices[v2Idx];

    // Find a point v3 farthest from the line v1->v2
    Vec3 midpoint = (v1 + v2) * 0.5f;
    float maxDistance = -FLT_MAX;
    qhIndex v3Idx = 0;
    Vec3 dir = v2 - midpoint;
    float denom = dot(dir, dir);
    for (int i = 0; i < numVertices; ++i) {
        if (i == v1Idx || i == v2Idx)
            continue;
        Vec3 v = vertices[i];
        Vec3 v_midpoint = v - midpoint;
        float u = dot(v_midpoint, dir) / denom;
        Vec3 w = v1 + (dir * u);
        Vec3 dvec = v - w;
        float d = dot(dvec, dvec);
        if (d >= maxDistance) {
            maxDistance = d;
            v3Idx = i;
        }
    }
    Vec3 v3 = vertices[v3Idx];

    // Find a point v4 farthest from the plane formed by v1, v2, v3
    Vec3 dir2 = v3 - v1;
    Vec3 normal = cross(dir, dir2);

    midpoint = (v1 + v2 + v3) * (1.f / 3.f);
    maxDistance = FLT_MIN;
    float maxSignedDistance = 0.f;
    qhIndex v4Idx = 0;
    for (int i = 0; i < numVertices; ++i) {
        if (i == v1Idx || i == v2Idx || i == v3Idx)
            continue;
        float ds = dot(normal, (vertices[i] - midpoint));
        float d = fabs(ds);
        if (d > maxDistance) {
            maxDistance = d;
            maxSignedDistance = ds;
            v4Idx = i;
        }
    }

    Vec3 v4 = vertices[v4Idx];

    // Ensure the winding is correct (if maxSignedDistance > 0, swap v2 and v3)
    if (maxSignedDistance > 0.0f) {
        Vec3 tmp = v2;
        v2 = v3;
        v3 = tmp;
        qhIndex tmpIdx = v2Idx;
        v2Idx = v3Idx;
        v3Idx = tmpIdx;
        normal = -normal;
    }

    // Build initial tetrahedron (4 faces, 12 half-edges)
    edges[0] = (qhHalfEdge){v1Idx, 2, 4, 1, 0};   // v1->v2
    edges[1] = (qhHalfEdge){v2Idx, 6, 8, 0, 1};   // v2->v1
    edges[2] = (qhHalfEdge){v2Idx, 4, 0, 3, 0};   // v2->v3
    edges[3] = (qhHalfEdge){v3Idx, 9, 10, 2, 2};  // v3->v2
    edges[4] = (qhHalfEdge){v3Idx, 0, 2, 5, 0};   // v3->v1
    edges[5] = (qhHalfEdge){v1Idx, 11, 7, 4, 3};  // v1->v3
    edges[6] = (qhHalfEdge){v1Idx, 8, 1, 7, 1};   // v1->v4
    edges[7] = (qhHalfEdge){v4Idx, 5, 11, 6, 3};  // v4->v1
    edges[8] = (qhHalfEdge){v4Idx, 1, 6, 9, 1};   // v4->v2
    edges[9] = (qhHalfEdge){v2Idx, 10, 3, 8, 2};  // v2->v4
    edges[10] = (qhHalfEdge){v4Idx, 3, 9, 11, 2}; // v4->v3
    edges[11] = (qhHalfEdge){v3Idx, 7, 5, 10, 3}; // v3->v4

    faces[0] = (qhFace){ (v1 + v2 + v3) * (1.f/3.f), normalize(normal), 0, 0, 1};
    faces[1] = (qhFace){ (v1 + v2 + v4) * (1.f/3.f), normalize(cross((v4 - v1), (v2 - v1))), 1, 0, 2};
    faces[2] = (qhFace){ (v2 + v3 + v4) * (1.f/3.f), normalize(cross((v4 - v2), (v3 - v2))), 3, 1, 3};
    faces[3] = (qhFace){ (v1 + v3 + v4) * (1.f/3.f), normalize(cross((v3 - v1), (v4 - v1))), 5, 2, 3};

    // Segment the remaining vertices among the 4 initial faces
    int cListSize = 0;
    for (int i = 0; i < 4; ++i) {
        faces[i].CList.Head = -1;
        faces[i].CList.Tail = -1;
        faces[i].CList.Size = 0;
        faces[i].CList.MaxDistance = 0.f;
        faces[i].CList.MaxVertex = -1;
    }

    for (int i = 0; i < numVertices; ++i) {
        if (i == v1Idx || i == v2Idx || i == v3Idx || i == v4Idx)
            continue;
        Vec3 v = vertices[i];
        float bestD = FLT_MIN;
        qhIndex bestFace = -1;

        for (int faceIdx = 0; faceIdx < 4; ++faceIdx) {
            qhFace *face = faces + faceIdx;
            float d = dot(face->Normal, (v - face->CenterPoint));
            if (d > 0.0f && d > bestD) {
                bestD = d;
                bestFace = faceIdx;
            }
        }

        if (bestFace != (qhIndex)-1) {
            qhConflictListItem li = {v, bestD};
            qhIndex idx = i;
            li.Next = faces[bestFace].CList.Head;
            faces[bestFace].CList.Head = idx;
            if (faces[bestFace].CList.Size == 0) {
                faces[bestFace].CList.Tail = idx;
            }
            faces[bestFace].CList.Size++;
            if (faces[bestFace].CList.MaxDistance < bestD) {
                faces[bestFace].CList.MaxDistance = bestD;
                faces[bestFace].CList.MaxVertex = idx;
            }
            clist[idx] = li;
        }
    }

    int numFaces = 4;
    int numEdges = 12;
    qhFacesList facesList;
    facesList.Faces = faces;
    facesList.Head = 0;
    facesList.Tail = 3;

    // Optional debug check/callback
    if (callback)
        callback(&facesList, vertices, edges);

    // Main loop: pick the face with the farthest conflict vertex and grow hull
    while (1) {
        float maxDist = 0.f;
        int sFace = -1;

        facesList.Faces[facesList.Tail].Next = -1;
        qhIndex faceIdx = facesList.Head;
        while (faceIdx != (qhIndex)-1) {
            qhFace *face = faces + faceIdx;
            if (face->CList.Size > 0 && face->CList.MaxDistance > maxDist) {
                sFace = faceIdx;
                maxDist = face->CList.MaxDistance;
            }
            faceIdx = face->Next;
        }

        if (sFace == -1) {
            // No more external points to add
            break;
        }

        qhFace *mainFace = faces + sFace;
        qhIndex maxVertexIdx = mainFace->CList.MaxVertex;
        qhConflictListItem clItem = clist[maxVertexIdx];
        Vec3 v = clItem.Vertex;

        qhMarkedBitSet faceMarkSet;
        memset(&faceMarkSet, 0, sizeof(faceMarkSet));
        int numHorizonEdges = 0;
        qhConflictList orphanList;
        orphanList.Size = 0;
        orphanList.Head = -1;
        orphanList.Tail = -1;
        orphanList.MaxDistance = 0.f;
        orphanList.MaxVertex = -1;

        qhDFS(edgeStack, &faceMarkSet, edges, horizonEdges, &numHorizonEdges, &facesList, mainFace, v, &orphanList,
              clist);

        // Create new faces for each horizon edge
        qhHalfEdgeIndex startEdgeIdx = numEdges;
        qhIndex startFaceIdx = numFaces;
        Vec3 vThird = v * (1.f / 3.f);

        for (int i = 0; i < numHorizonEdges; ++i) {
            qhHalfEdgeIndex e1Idx = numEdges++;
            qhHalfEdgeIndex e2Idx = numEdges++;

            qhHalfEdge *e1 = edges + e1Idx;
            qhHalfEdge *e2 = edges + e2Idx;

            qhHalfEdgeIndex eIdx = horizonEdges[i];
            qhHalfEdge *e = edges + eIdx;

            e2->Next = eIdx;
            e2->Prev = e1Idx;
            e2->Tail = maxVertexIdx;
            e2->Twin = e1Idx - 2;

            e1->Next = e2Idx;
            e1->Prev = eIdx;
            e1->Tail = edges[e->Next].Tail;
            e1->Twin = e2Idx + 2;

            e->Next = e1Idx;
            e->Prev = e2Idx;

            qhIndex faceI = numFaces;
            qhFace *face = faces + faceI;
            face->Next = faceI + 1;
            face->Prev = faceI - 1;
            numFaces++;

            e1->Face = faceI;
            e2->Face = faceI;
            e->Face = faceI;

            Vec3 v1 = vertices[e->Tail];
            Vec3 v2 = vertices[e1->Tail];
            face->Normal = normalize(cross((v2 - v1), (v - v1)));
            face->CenterPoint = (v1 * (1.f/3.f)) + (v2 * (1.f/3.f)) + vThird;
            face->HalfEdge = eIdx;

            face->CList.Size = 0;
            face->CList.Head = -1;
            face->CList.Tail = -1;
            face->CList.MaxDistance = 0.f;
            face->CList.MaxVertex = -1;
        }

        // Close the horizon loop
        edges[startEdgeIdx + 1].Twin = numEdges - 2;
        edges[numEdges - 2].Twin = startEdgeIdx + 1;

        faces[facesList.Tail].Next = startFaceIdx;
        faces[startFaceIdx].Prev = facesList.Tail;
        facesList.Tail = numFaces - 1;

        // Assign orphaned vertices to new faces
        qhIndex liIdx = orphanList.Head;
        for (int i = 0; i < orphanList.Size; ++i) {
            qhConflictListItem *li = clist + liIdx;
            qhIndex tmpNext = li->Next;

            if (liIdx != maxVertexIdx) {
                float minD = FLT_MAX;
                int minFace = -1;
                Vec3 vv = li->Vertex;

                // Find best new face for this orphaned vertex
                for (int faceI = startFaceIdx; faceI < numFaces; ++faceI) {
                    qhFace *face = faces + faceI;
                    float d = dot(face->Normal, (vv - face->CenterPoint));
                    if (d >= 0.f && d < minD) {
                        minD = d;
                        minFace = faceI;
                    }
                }

                if (minFace != -1) {
                    qhFace *face = faces + minFace;
                    li->Distance = minD;
                    li->Next = face->CList.Head;
                    face->CList.Head = liIdx;
                    if (face->CList.Size == 0) {
                        face->CList.Tail = liIdx;
                    }
                    face->CList.Size++;
                    if (face->CList.MaxDistance < minD) {
                        face->CList.MaxDistance = minD;
                        face->CList.MaxVertex = liIdx;
                    }
                }
            }
            liIdx = tmpNext;
        }

        // Merge newly created faces if needed
        qhVisitedEdgeBitSet edgeBitSet;
        memset(&edgeBitSet, 0, sizeof(edgeBitSet));

        qhIndex currentFaceIdx = faces[startFaceIdx].Prev;
        while (currentFaceIdx < facesList.Tail) {
            currentFaceIdx = faces[currentFaceIdx].Next;
            qhFace *currentFace = faces + currentFaceIdx;

            qhHalfEdgeIndex eStart = currentFace->HalfEdge;
            qhHalfEdgeIndex eCur = eStart;
            while (!QH_IS_EDGE_VISITED(edgeBitSet, eCur)) {
                QH_SET_EDGE_VISITED(edgeBitSet, eCur);
                qhHalfEdge *cEdge = edges + eCur;
                qhHalfEdge *tEdge = edges + cEdge->Twin;
                qhIndex tfIdx = tEdge->Face;

                qhHalfEdgeIndex nextEdge = cEdge->Next;
                if (qhShouldMergeFaces(currentFace, faces + tfIdx)) {
                    nextEdge = qhMergeFaces(currentFaceIdx, tfIdx, currentFace, faces + tfIdx, cEdge, tEdge, &facesList,
                                            vertices, edges, clist);
                }
                eCur = nextEdge;
            }
        }

        if (callback)
            callback(&facesList, vertices, edges);
    }

    return facesList;
}
} // namespace ce
