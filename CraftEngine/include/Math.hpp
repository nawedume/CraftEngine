#pragma once

#include "Core.h"
#include "glm/detail/func_trigonometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/matrix_cross_product.hpp>
#include <limits>

namespace ce {

Real const ZERO = Real(0.0);
Real const MAX_REAL = std::numeric_limits<Real>::max();
Real const MIN_REAL = std::numeric_limits<Real>::lowest();

inline Vec3 cross(Vec3 a, Vec3 b) { return glm::cross(a, b); }

inline Real dot(Vec3 a, Vec3 b) { return glm::dot(a, b); }

inline Quat vec3ToQuat(Vec3 w) { return Quat(0.0f, w); }

inline Real radians(Real angleInDeg) { return glm::radians(angleInDeg); }

inline Quat aaToQuat(Vec3 axis, Real angle) {
    angle = angle * 0.5f;
    Quat q;
    q.w = cos(angle);
    Real s = sin(angle);
    q.x = s * axis.x;
    q.y = s * axis.y;
    q.z = s * axis.z;
    return q;
}

inline Mat3 qToMat(Quat quat) { return glm::toMat3(quat); }

static inline Vec3 row(Mat3 m) { return Vec3(m[0][0], m[1][0], m[2][0]); }

inline Mat4 qToMat4(Quat quat) { return glm::toMat4(quat); }

inline Quat normalize(Quat q) { return glm::normalize(q); }

inline Vec3 normalize(Vec3 v) { return glm::normalize(v); }

inline float mag(Vec3 v) { return glm::length(v); }

inline Quat conj(Quat q) { return conjugate(q); }

inline Vec3 rotate(Vec3 v, Quat q) {
    Quat vq;
    vq.w = 0.0f;
    vq.x = v.x;
    vq.y = v.y;
    vq.z = v.z;
    Quat resq = q * vq * conj(q);

    return {resq.x, resq.y, resq.z};
}

inline Vec3 rotateInv(Vec3 v, Quat q) { return rotate(v, conj(q)); }

inline bool isZero(Real r) { return abs(r) < 1e-9; }

inline bool isOne(Real r) { return abs(r - 1.0f) < 1e-6; }

inline Vec3 wrap(Vec3 v) { return {v.z, v.x, v.y}; }

// Taken from: https://box2d.org/posts/2014/02/computing-a-basis/
inline void createOrthonormalBasis(Vec3 const axis, Vec3 *axis2, Vec3 *axis3) {
    // Suppose vector a has all equal components and is a unit vector:
    // a = (s, s, s)
    // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at
    // least one component of a unit vector must be greater or equal
    // to 0.57735.

    Vec3 a2;
    if (abs(axis.x) >= 0.57735f)
        a2 = Vec3(axis.y, -axis.x, 0.0f);
    else
        a2 = Vec3(0.0f, axis.z, -axis.y);

    *axis2 = normalize(a2);
    *axis3 = -cross(axis, *axis2);
}

inline Mat3 transpose(Mat3 mat) { return glm::transpose(mat); }

static inline Real max(Real a, Real b) { return glm::max(a, b); }

static inline Real min(Real a, Real b) { return glm::min(a, b); }

inline Real clamp(Real val, Real min, Real max) { return glm::clamp(val, min, max); }

inline Mat3 inverse(Mat3 mat) { return glm::inverse(mat); }

inline bool isUnitLength(Vec3 v) { return isOne(mag(v)); }

inline Real sqrt(Real a) { return glm::sqrt(a); }

inline Vec3 abs(Vec3 v) { return glm::abs(v); }

inline Real abs(Real a) { return glm::abs(a); }

inline Real cosh(Real a) {
    return glm::cosh(a);
}

inline Mat3 matrixCross3(Vec3 v) {
    return glm::matrixCross3(v);
}
}; // namespace ce
