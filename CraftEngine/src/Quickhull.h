#pragma once

#include "Core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define MAX_NUM_VERTICES 200000
#define MAX_NUM_EDGES (MAX_NUM_VERTICES * 3) - 6
#define MAX_NUM_HALF_EDGES 2 * MAX_NUM_EDGES
#define MAX_NUM_FACES (MAX_NUM_VERTICES * 2) - 4

#define MAX_EDGE_BUFFER_SIZE MAX_NUM_HALF_EDGES * 2
#define MAX_FACE_BUFFER_SIZE MAX_NUM_FACES * 2

typedef uint32_t qhHalfEdgeIndex;
typedef uint32_t qhIndex;

namespace ce {

// todo Optimize structures to work better with cache, especially Face
// Should be a conflict list for each face
typedef struct qhConflictListItem {
    Vec3 Vertex;
    float Distance;
    qhIndex Next;
} qhConflictListItem;

typedef struct qhConflictList {
    float MaxDistance;
    int Size;
    qhIndex MaxVertex;
    qhIndex Head;
    qhIndex Tail; // @todo not implemented in code yet
} qhConflictList;

typedef struct {
    Vec3 CenterPoint;
    Vec3 Normal;
    qhHalfEdgeIndex HalfEdge;
    qhIndex Prev;
    qhIndex Next;
    qhConflictList CList;
} qhFace;

typedef struct {
    qhFace *Faces;
    qhIndex Head;
    qhIndex Tail;
} qhFacesList;

typedef struct {
    qhIndex Tail;
    qhHalfEdgeIndex Next;
    qhHalfEdgeIndex Prev;
    qhHalfEdgeIndex Twin;
    qhIndex Face;

} qhHalfEdge;

typedef void (*IterationCallback)(qhFacesList *faceList, Vec3 *vertices, qhHalfEdge *edges);

extern qhFacesList qhCreateHull(int numVertices, Vec3 *vertices, qhFace *faces, qhHalfEdge *edges,
                                qhHalfEdgeIndex *edgeStack, qhConflictListItem *clist, qhHalfEdgeIndex *horizonEdges,
                                IterationCallback callback);

} // namespace ce
