#include "Quickhull.h"
#include "World.h"
#include <cstring>

namespace ce {

// The conversion function:
static ConvexHull ConvertQuickhullResult(const qhFacesList* facesList, const Vec3 *vertices, int numVertices, const qhHalfEdge *qhHalfEdges) {
    // TODO: Implement converstion function for quickhull
    // The quick hull algoirthm is complete, and has been tested extensively outside the engine, the integration between the two still needs to be thought out
    // due to the different data formats
}

} // namespace ce
