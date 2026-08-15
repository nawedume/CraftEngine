#ifndef MESH_H
#define MESH_H

#include <malloc/_malloc_type.h>
#include <stdio.h>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <string.h>

const float BoxMesh[]{
    // positions // normals // texture coords
    -1.0f, -1.0f, -1.0f, 0.0f,  0.0f,  -1.0f, 0.0f, 0.0f, 1.0f,  -1.0f, -1.0f, 0.0f,  0.0f,  -1.0f, 1.0f, 0.0f,
    1.0f,  1.0f,  -1.0f, 0.0f,  0.0f,  -1.0f, 1.0f, 1.0f, 1.0f,  1.0f,  -1.0f, 0.0f,  0.0f,  -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,  -1.0f, 0.0f,  0.0f,  -1.0f, 0.0f, 1.0f, -1.0f, -1.0f, -1.0f, 0.0f,  0.0f,  -1.0f, 0.0f, 0.0f,
    -1.0f, -1.0f, 1.0f,  0.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f,  -1.0f, 1.0f,  0.0f,  0.0f,  1.0f,  1.0f, 0.0f,
    1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f, 1.0f, 1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f, 1.0f,
    -1.0f, 1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f, 1.0f, -1.0f, -1.0f, 1.0f,  0.0f,  0.0f,  1.0f,  0.0f, 0.0f,
    -1.0f, 1.0f,  1.0f,  -1.0f, 0.0f,  0.0f,  1.0f, 0.0f, -1.0f, 1.0f,  -1.0f, -1.0f, 0.0f,  0.0f,  1.0f, 1.0f,
    -1.0f, -1.0f, -1.0f, -1.0f, 0.0f,  0.0f,  0.0f, 1.0f, -1.0f, -1.0f, -1.0f, -1.0f, 0.0f,  0.0f,  0.0f, 1.0f,
    -1.0f, -1.0f, 1.0f,  -1.0f, 0.0f,  0.0f,  0.0f, 0.0f, -1.0f, 1.0f,  1.0f,  -1.0f, 0.0f,  0.0f,  1.0f, 0.0f,

    1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f,  1.0f,  -1.0f, 1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
    1.0f,  -1.0f, -1.0f, 1.0f,  0.0f,  0.0f,  0.0f, 1.0f, 1.0f,  -1.0f, -1.0f, 1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
    1.0f,  -1.0f, 1.0f,  1.0f,  0.0f,  0.0f,  0.0f, 0.0f, 1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
    -1.0f, -1.0f, -1.0f, 0.0f,  -1.0f, 0.0f,  0.0f, 1.0f, 1.0f,  -1.0f, -1.0f, 0.0f,  -1.0f, 0.0f,  1.0f, 1.0f,
    1.0f,  -1.0f, 1.0f,  0.0f,  -1.0f, 0.0f,  1.0f, 0.0f, 1.0f,  -1.0f, 1.0f,  0.0f,  -1.0f, 0.0f,  1.0f, 0.0f,
    -1.0f, -1.0f, 1.0f,  0.0f,  -1.0f, 0.0f,  0.0f, 0.0f, -1.0f, -1.0f, -1.0f, 0.0f,  -1.0f, 0.0f,  0.0f, 1.0f,
    -1.0f, 1.0f,  -1.0f, 0.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f,  1.0f,  -1.0f, 0.0f,  1.0f,  0.0f,  1.0f, 1.0f,
    1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  1.0f, 0.0f, 1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  1.0f, 0.0f,
    -1.0f, 1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  0.0f, 0.0f, -1.0f, 1.0f,  -1.0f, 0.0f,  1.0f,  0.0f,  0.0f, 1.0f};

struct GraphicBuffers {
    float *VertexBuffer;
    unsigned int NumVertices;
    unsigned int *IndexBuffer;
    unsigned int NumIndices;
};

inline GraphicBuffers createSphereMesh(float radius, unsigned int numLat, unsigned int numLong) {
    float latAngleStep = M_PI / (numLat + 1);
    float longAngleStep = (2.0f * M_PI) / numLong;

    float currentLat = latAngleStep;

    float *vertexBuffer = new float[((numLat * numLong) + 2) * 8];
    unsigned int numVertices = 0;
    for (int latIdx = 0; latIdx < numLat; ++latIdx) {
        float currentLong = 0.0f;
        for (int longIdx = 0; longIdx < numLong; ++longIdx) {
            float *v = &vertexBuffer[((latIdx * numLong) + longIdx) * 8];

            v[3] = sin(currentLat) * cos(currentLong);
            v[4] = cos(currentLat);
            v[5] = sin(currentLat) * sin(currentLong);

            v[0] = radius * v[3];
            v[1] = radius * v[4];
            v[2] = radius * v[5];

            // @TODO: Need to fill in the proper texture coordinates
            // @alert: Read this before you use textures for spheres!
            v[6] = 0.0f;
            v[7] = 0.0f;

            ++numVertices;
            currentLong += longAngleStep;
        }
        currentLat += latAngleStep;
    }

    // Add the poles of the sphere
    int topVertex = (numLat * numLong) * 8;
    float *v = &vertexBuffer[topVertex];
    v[0] = 0.0f;
    v[1] = radius;
    v[2] = 0.0f;
    v[3] = 0.0f;
    v[4] = 1.0f;
    v[5] = 0.0f;

    // @TODO: Fill out these texture coordinates
    v[6] = 0.0f;
    v[7] = 0.0f;

    int bottomVertex = topVertex + 8;
    v = &vertexBuffer[bottomVertex];
    v[0] = 0.0f;
    v[1] = -radius;
    v[2] = 0.0f;
    v[3] = 0.0f;
    v[4] = -1.0f;
    v[5] = 0.0f;

    // @TODO: Fill out these texture coordinates
    v[6] = 0.0f;
    v[7] = 0.0f;

    numVertices += 2;

    // Create faces
    unsigned int *indexBuffer = new unsigned int[((numLat - 1) * numLong * 2 * 3) + (2 * numLong * 3)];
    unsigned int numFaceIndices = 0;
    for (int latIdx = 0; latIdx < (numLat - 1); ++latIdx) {
        int rowIdx = (latIdx * numLong);
        int rowIdx2 = rowIdx + numLong;
        for (int longIdx = 0; longIdx < numLong - 1; ++longIdx) {
            int idx1 = rowIdx + longIdx;
            int idx2 = idx1 + 1;
            int idx3 = rowIdx2 + longIdx;
            int idx4 = idx3 + 1;

            indexBuffer[numFaceIndices++] = idx1;
            indexBuffer[numFaceIndices++] = idx4;
            indexBuffer[numFaceIndices++] = idx3;

            indexBuffer[numFaceIndices++] = idx1;
            indexBuffer[numFaceIndices++] = idx2;
            indexBuffer[numFaceIndices++] = idx4;
        }

        int idx1 = rowIdx + (numLong - 1);
        int idx2 = rowIdx;
        int idx3 = rowIdx2 + (numLong - 1);
        int idx4 = rowIdx2;

        indexBuffer[numFaceIndices++] = idx1;
        indexBuffer[numFaceIndices++] = idx4;
        indexBuffer[numFaceIndices++] = idx3;

        indexBuffer[numFaceIndices++] = idx1;
        indexBuffer[numFaceIndices++] = idx2;
        indexBuffer[numFaceIndices++] = idx4;
    }

    int topIdx = (numLat * numLong);
    int bottomIdx = topIdx + 1;
    int lastRow = (numLat - 1) * numLong;
    for (int longIdx = 0; longIdx < numLong; ++longIdx) {
        indexBuffer[numFaceIndices++] = topIdx;
        indexBuffer[numFaceIndices++] = longIdx;
        indexBuffer[numFaceIndices++] = (longIdx + 1) % numLong;

        indexBuffer[numFaceIndices++] = bottomIdx;
        indexBuffer[numFaceIndices++] = lastRow + longIdx;
        indexBuffer[numFaceIndices++] = lastRow + ((longIdx + 1) % numLong);
    }

    return {.VertexBuffer = vertexBuffer,
            .NumVertices = numVertices,
            .IndexBuffer = indexBuffer,
            .NumIndices = numFaceIndices};
}

inline GraphicBuffers createCapsuleMesh(float radius, float halfLength, unsigned int numLat, unsigned int numLong) {
    using namespace glm;
    int vSize = numLat * numLong * 8 * 2;
    float *vertices = new float[vSize];

    float thetaStep = half_pi<float>() / (numLat - 1);
    float phiStep = 2.0f * pi<float>() / numLong;
    int i = 0;
    float start = 0.0;
    float multiplier = 1.0f;
    for (int j = 0; j < 2; ++j) {
        for (int latIdx = 0; latIdx < numLat; ++latIdx) {
            float theta = start + thetaStep * latIdx;
            float y = cos(theta) * radius + multiplier * halfLength;
            float sinTheta = sin(theta) * radius;

            for (int longIdx = 0; longIdx < numLong; ++longIdx) {
                float phi = phiStep * longIdx;
                float x = sinTheta * cos(phi);
                float z = sinTheta * sin(phi);

                vertices[i++] = x;
                vertices[i++] =  y;
                vertices[i++] = z;
                vertices[i++] = x;
                vertices[i++] =  y;
                vertices[i++] = z;
                // uv
                vertices[i++] = 0.0f;
                vertices[i++] = 0.0f;

            }
        }
        start = half_pi<float>();
        multiplier = -1.0f;

    }

    unsigned int numIndices = 3 * 2 * numLat * 2 * (numLong );
    unsigned int *indices = new unsigned int[numIndices];
    i = 0;
    for (int latIdx = 0; latIdx < (numLat * 2) - 1; ++latIdx) {
        for (int longIdx = 0; longIdx < numLong; ++longIdx) {
            int first = (latIdx * numLat) + longIdx;
            int second = first + numLat;
            int third = (latIdx * numLat) + ((longIdx + 1) % numLong);
            int fourth = third + numLat;

            indices[i++] = first;
            indices[i++] = second;
            indices[i++] = fourth;

            indices[i++] = first;
            indices[i++] = fourth;
            indices[i++] = third;
        }
    }

    return {.VertexBuffer = vertices,
            .NumVertices = numLat * numLong * 2,
            .IndexBuffer = indices,
            .NumIndices = static_cast<unsigned int>(i)};
}

inline GraphicBuffers createArrowMesh(float headWidth, float lineWidth, float length, float tailLength) {
    assert(lineWidth < headWidth);
    assert(tailLength < length);

    GraphicBuffers g;

    float* vertices = new float[13 * 8];
    int vi = 0;
    #define ADD_VERTEX(x, y, z, nx, ny, nz, tx, ty) vertices[vi++] = x; vertices[vi++] = y; vertices[vi++] = z; vertices[vi++] = nx; vertices[vi++] = ny; vertices[vi++] = nz; vertices[vi++] = tx; vertices[vi++] = ty;
    ADD_VERTEX(0.0, length, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
    ADD_VERTEX(headWidth, tailLength, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
    ADD_VERTEX(0.0, tailLength, -headWidth, 0.0, 0.0, -1.0, 0.0, 0.0);
    ADD_VERTEX(-headWidth, tailLength, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0);
    ADD_VERTEX(0.0, tailLength, headWidth, 0.0, 0.0, 1.0, 0.0, 0.0);

    ADD_VERTEX(lineWidth, tailLength, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
    ADD_VERTEX(0.0, tailLength, -lineWidth, 0.0, 0.0, -1.0, 0.0, 0.0);
    ADD_VERTEX(-lineWidth, tailLength, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0);
    ADD_VERTEX(0.0, tailLength, lineWidth, 0.0, 0.0, 1.0, 0.0, 0.0);

    ADD_VERTEX(lineWidth, .0, .0, 1., .0, .0, .0, .0)
    ADD_VERTEX(.0, .0, -lineWidth, 0., .0, -1., .0, .0)
    ADD_VERTEX(-lineWidth, .0, .0, -1., .0, .0, .0, .0)
    ADD_VERTEX(.0, .0, lineWidth, 0., .0, 1., .0, .0)

    unsigned int* indices = new unsigned int[22 * 3];
    #define ADD_FACE(a, b, c, i) indices[3*i] = a; indices[3*i + 1] = b; indices[3*i + 2] = c;
    ADD_FACE(0, 1, 2, 0)
    ADD_FACE(0, 2, 3, 1)

    ADD_FACE(0, 3, 4, 2)
    ADD_FACE(0, 4, 1, 3)

    ADD_FACE(1, 2, 5, 4)
    ADD_FACE(2, 6, 5, 5)

    ADD_FACE(2, 3, 6, 6)
    ADD_FACE(3, 7, 6, 7)

    ADD_FACE(3, 4, 7, 8)
    ADD_FACE(4, 8, 7, 9)

    ADD_FACE(4, 1, 8, 10)
    ADD_FACE(1, 5, 8, 11)

    ADD_FACE(5, 9, 10, 12)
    ADD_FACE(5, 10, 6, 13)

    ADD_FACE(6, 10, 11, 14)
    ADD_FACE(6, 11, 7, 15)

    ADD_FACE(7, 11, 12, 16)
    ADD_FACE(7, 12, 8, 17)

    ADD_FACE(8, 12, 9, 18)
    ADD_FACE(8, 9, 5, 19)

    ADD_FACE(12, 10, 9, 20)
    ADD_FACE(12, 11, 10, 21)

    return {.VertexBuffer = vertices,
            .NumVertices = 104,
            .IndexBuffer = indices,
            .NumIndices = 66 };
}

inline GraphicBuffers createBoxMesh(float w, float l, float h) {
    float* boxMesh =  new float[36 * 8];
    memcpy(boxMesh, BoxMesh, sizeof(float) * 36 * 8);
    for (int i = 0; i < 36 * 8; i += 8) {
        boxMesh[i] = BoxMesh[i] * w;
        boxMesh[i + 1] = BoxMesh[i + 1] * l;
        boxMesh[i + 2] = BoxMesh[i + 2] * h;
    }
    return {
        .VertexBuffer = boxMesh,
        .NumVertices = 36,
        .IndexBuffer = nullptr,
        .NumIndices = 0
    };
}

#endif
