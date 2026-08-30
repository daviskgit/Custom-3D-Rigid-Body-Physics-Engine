// math.h - minimal linear algebra for the physics engine and renderer.
// Hand-rolled on purpose: vec3 / 3x3 matrix / quaternion for simulation,
// plus a 4x4 matrix used only to feed the GL pipeline.
#pragma once
#include <cmath>
#include <cstdint>

namespace pe {

using Real = float;
constexpr Real PI = Real(3.14159265358979323846);

inline Real clampf(Real v, Real lo, Real hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vec3 {
    Real x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(Real s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(Real s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& b) { x += b.x; y += b.y; z += b.z; return *this; }
    Vec3& operator-=(const Vec3& b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
    Vec3& operator*=(Real s) { x *= s; y *= s; z *= s; return *this; }
    Real operator[](int i) const { return (&x)[i]; }
    Real& operator[](int i) { return (&x)[i]; }
};

inline Vec3 operator*(Real s, const Vec3& v) { return {v.x * s, v.y * s, v.z * s}; }
inline Real dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline Real lengthSq(const Vec3& v) { return dot(v, v); }
inline Real length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) {
    Real l = length(v);
    return l > Real(1e-12) ? v / l : Vec3{0, 0, 0};
}

// Column-major 3x3 matrix, columns c0..c2. Used for rotation and inertia tensors.
struct Mat3 {
    Vec3 c0{1, 0, 0}, c1{0, 1, 0}, c2{0, 0, 1};
    Mat3() = default;
    Mat3(const Vec3& a, const Vec3& b, const Vec3& c) : c0(a), c1(b), c2(c) {}

    static Mat3 diagonal(Real a, Real b, Real c) {
        return {{a, 0, 0}, {0, b, 0}, {0, 0, c}};
    }
    static Mat3 zero() { return {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; }

    Vec3 operator*(const Vec3& v) const { return c0 * v.x + c1 * v.y + c2 * v.z; }
    Mat3 operator*(const Mat3& m) const { return {(*this) * m.c0, (*this) * m.c1, (*this) * m.c2}; }
    Mat3 operator+(const Mat3& m) const { return {c0 + m.c0, c1 + m.c1, c2 + m.c2}; }
    Mat3 operator*(Real s) const { return {c0 * s, c1 * s, c2 * s}; }

    Mat3 transposed() const {
        return {{c0.x, c1.x, c2.x}, {c0.y, c1.y, c2.y}, {c0.z, c1.z, c2.z}};
    }
    Mat3 inverse() const {
        // Cofactor / determinant inversion; inertia tensors are always invertible.
        Vec3 r0{c0.x, c1.x, c2.x}, r1{c0.y, c1.y, c2.y}, r2{c0.z, c1.z, c2.z};
        Vec3 m0 = cross(r1, r2), m1 = cross(r2, r0), m2 = cross(r0, r1);
        Real det = dot(r0, m0);
        Real inv = det != 0 ? Real(1) / det : Real(0);
        // rows of the inverse are m0,m1,m2 * inv -> store back as columns
        return {{m0.x * inv, m1.x * inv, m2.x * inv},
                {m0.y * inv, m1.y * inv, m2.y * inv},
                {m0.z * inv, m1.z * inv, m2.z * inv}};
    }
};

// Unit quaternion for body orientation. Stored as (w, x, y, z).
struct Quat {
    Real w = 1, x = 0, y = 0, z = 0;
    Quat() = default;
    Quat(Real w_, Real x_, Real y_, Real z_) : w(w_), x(x_), y(y_), z(z_) {}

    static Quat fromAxisAngle(const Vec3& axis, Real angle) {
        Vec3 a = normalize(axis);
        Real h = angle * Real(0.5);
        Real s = std::sin(h);
        return {std::cos(h), a.x * s, a.y * s, a.z * s};
    }

    Quat operator*(const Quat& b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z,
                w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w};
    }
    Quat operator+(const Quat& b) const { return {w + b.w, x + b.x, y + b.y, z + b.z}; }
    Quat operator*(Real s) const { return {w * s, x * s, y * s, z * s}; }

    Quat normalized() const {
        Real n = std::sqrt(w * w + x * x + y * y + z * z);
        if (n < Real(1e-12)) return {1, 0, 0, 0};
        Real inv = Real(1) / n;
        return {w * inv, x * inv, y * inv, z * inv};
    }

    Mat3 toMat3() const {
        Real xx = x * x, yy = y * y, zz = z * z;
        Real xy = x * y, xz = x * z, yz = y * z;
        Real wx = w * x, wy = w * y, wz = w * z;
        return {{1 - 2 * (yy + zz), 2 * (xy + wz), 2 * (xz - wy)},
                {2 * (xy - wz), 1 - 2 * (xx + zz), 2 * (yz + wx)},
                {2 * (xz + wy), 2 * (yz - wx), 1 - 2 * (xx + yy)}};
    }
};

// Integrate an angular velocity (world frame) into an orientation quaternion.
inline Quat integrateOrientation(const Quat& q, const Vec3& omega, Real dt) {
    Quat wq{0, omega.x, omega.y, omega.z};
    Quat dq = wq * q * (Real(0.5) * dt);
    return (q + dq).normalized();
}

// ---- 4x4 matrix: rendering only -------------------------------------------------
struct Mat4 {
    Real m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; // column-major

    static Mat4 identity() { return {}; }

    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for (int col = 0; col < 4; ++col)
            for (int row = 0; row < 4; ++row) {
                Real s = 0;
                for (int k = 0; k < 4; ++k) s += m[k * 4 + row] * b.m[col * 4 + k];
                r.m[col * 4 + row] = s;
            }
        return r;
    }

    static Mat4 translation(const Vec3& t) {
        Mat4 r;
        r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
        return r;
    }
    static Mat4 fromRotationScale(const Mat3& rot, const Vec3& scale) {
        Mat4 r;
        r.m[0] = rot.c0.x * scale.x; r.m[1] = rot.c0.y * scale.x; r.m[2] = rot.c0.z * scale.x; r.m[3] = 0;
        r.m[4] = rot.c1.x * scale.y; r.m[5] = rot.c1.y * scale.y; r.m[6] = rot.c1.z * scale.y; r.m[7] = 0;
        r.m[8] = rot.c2.x * scale.z; r.m[9] = rot.c2.y * scale.z; r.m[10] = rot.c2.z * scale.z; r.m[11] = 0;
        r.m[12] = r.m[13] = r.m[14] = 0; r.m[15] = 1;
        return r;
    }
    static Mat4 perspective(Real fovyRad, Real aspect, Real zn, Real zf) {
        Real f = Real(1) / std::tan(fovyRad * Real(0.5));
        Mat4 r{};
        for (int i = 0; i < 16; ++i) r.m[i] = 0;
        r.m[0] = f / aspect;
        r.m[5] = f;
        r.m[10] = (zf + zn) / (zn - zf);
        r.m[11] = -1;
        r.m[14] = (2 * zf * zn) / (zn - zf);
        return r;
    }
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        Vec3 f = normalize(center - eye);
        Vec3 s = normalize(cross(f, up));
        Vec3 u = cross(s, f);
        Mat4 r{};
        for (int i = 0; i < 16; ++i) r.m[i] = 0;
        r.m[0] = s.x; r.m[4] = s.y; r.m[8] = s.z;
        r.m[1] = u.x; r.m[5] = u.y; r.m[9] = u.z;
        r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
        r.m[12] = -dot(s, eye); r.m[13] = -dot(u, eye); r.m[14] = dot(f, eye);
        r.m[15] = 1;
        return r;
    }
};

} // namespace pe
