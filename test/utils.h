#include <string>
#include "World.h"

inline void const PrintVec(std::string const& prefix, ce::Vec3 v) {
    printf("%s: (%f, %f, %f)\n", prefix.c_str(), v.x, v.y, v.z);
}

static void const PrintVec3(std::string const& prefix, float* v) {
    printf("%s: (%f, %f, %f)\n", prefix.c_str(), v[0], v[1], v[2]);
}

inline void const PrintQuat(std::string const& prefix, ce::Quat v) {
    printf("%s: (%f, %f, %f, %f)\n", prefix.c_str(), v.x, v.y, v.z, v.w);
}

static ce::ConvexHull* CreateBoxConvexHull(ce::Transform& t, ce::Vec3 halfEdge) {
    ce::ConvexHull* hull = new ce::ConvexHull(6, 24, 8);

    ce::Real xs = halfEdge.x;
    ce::Real ys = halfEdge.y;
    ce::Real zs = halfEdge.z;

    hull->Vertices[0] = ce::Vec3(-xs, -ys, -zs);
    hull->Vertices[1] = ce::Vec3(-xs, -ys, zs);
    hull->Vertices[2] = ce::Vec3(-xs, ys, -zs);
    hull->Vertices[3] = ce::Vec3(-xs, ys, zs);
    hull->Vertices[4] = ce::Vec3(xs, -ys, -zs);
    hull->Vertices[5] = ce::Vec3(xs, -ys, zs);
    hull->Vertices[6] = ce::Vec3(xs, ys, -zs);
    hull->Vertices[7] = ce::Vec3(xs, ys, zs);

    hull->HalfEdges[0] = ce::HalfEdge { 0, 4, 0, 2 };
    hull->HalfEdges[1] = ce::HalfEdge { 4, 0, 5, 15 };

    hull->HalfEdges[2] = ce::HalfEdge { 4, 5, 0, 4 };
    hull->HalfEdges[3] = ce::HalfEdge { 5, 4, 1, 8 };

    hull->HalfEdges[4] = ce::HalfEdge { 5, 1, 0, 6 };
    hull->HalfEdges[5] = ce::HalfEdge { 1, 5, 4, 11 };

    hull->HalfEdges[6] = ce::HalfEdge { 1, 0, 0, 0 };
    hull->HalfEdges[7] = ce::HalfEdge { 0, 1, 3, 18 };

    hull->HalfEdges[8] = ce::HalfEdge { 4, 6, 1, 23 };
    hull->HalfEdges[9] = ce::HalfEdge { 6, 4, 5, 1 };

    hull->HalfEdges[10] = ce::HalfEdge { 7, 5, 1, 3 };
    hull->HalfEdges[11] = ce::HalfEdge { 5, 7, 4, 21 };

    hull->HalfEdges[12] = ce::HalfEdge { 6, 2, 2, 17 };
    hull->HalfEdges[13] = ce::HalfEdge { 2, 6, 5, 9 };

    hull->HalfEdges[14] = ce::HalfEdge { 2, 0, 3, 7 };
    hull->HalfEdges[15] = ce::HalfEdge { 0, 2, 5, 13 };

    hull->HalfEdges[16] = ce::HalfEdge { 3, 2, 3, 14 };
    hull->HalfEdges[17] = ce::HalfEdge { 2, 3, 2, 20 };

    hull->HalfEdges[18] = ce::HalfEdge { 1, 3, 3, 16 };
    hull->HalfEdges[19] = ce::HalfEdge { 3, 1, 4, 5 };

    hull->HalfEdges[20] = ce::HalfEdge { 3, 7, 2, 22 };
    hull->HalfEdges[21] = ce::HalfEdge { 7, 3, 4, 19 };

    hull->HalfEdges[22] = ce::HalfEdge { 7, 6, 2, 12 };
    hull->HalfEdges[23] = ce::HalfEdge { 6, 7, 1, 10 };

    hull->Faces[0] = { .Normal = { 0.0f, -1.0f,  0.0f}, .D = ys };
    hull->Faces[1] = { .Normal = { 1.0f,  0.0f,  0.0f}, .D = xs };
    hull->Faces[2] = { .Normal = { 0.0f,  1.0f,  0.0f}, .D = ys };
    hull->Faces[3] = { .Normal = {-1.0f,  0.0f,  0.0f}, .D = xs };
    hull->Faces[4] = { .Normal = { 0.0f,  0.0f,  1.0f}, .D = zs };
    hull->Faces[5] = { .Normal = { 0.0f,  0.0f, -1.0f}, .D = zs };

    hull->FirstEdgeIndex[0] = 0;
    hull->FirstEdgeIndex[1] = 3;
    hull->FirstEdgeIndex[2] = 12;
    hull->FirstEdgeIndex[3] = 7;
    hull->FirstEdgeIndex[4] = 5;
    hull->FirstEdgeIndex[5] = 1;

    return hull;
}

static ce::BodyId CreateBox(ce::World* world, ce::Transform t, ce::ConvexHullDef* def, ce::Vec3 halfEdge, ce::Vec3 color, bool isStatic) {
    def->Hull = CreateBoxConvexHull(t, halfEdge);

    ce::BodyId bid;
    if (!isStatic) {
        ce::Mat3 inertia = ce::Mat3 {
            (1.0f / 12.0f) * (def->Mass * (halfEdge.y*halfEdge.y + halfEdge.z*halfEdge.z)), 0.0f, 0.0f,
            0.0f, (1.0f / 12.0f) * (def->Mass * (halfEdge.x*halfEdge.x + halfEdge.z*halfEdge.z)), 0.0f,
            0.0f, 0.0f, (1.0f / 12.0f) * (def->Mass * (halfEdge.y*halfEdge.y + halfEdge.x*halfEdge.x))
        };
        def->Inertia = inertia;
        bid = ce::AddConvexHull(world, t, *def);
    } else {
        bid = ce::AddStaticConvexHull(world, t, *def);
    }
    return bid;
}
