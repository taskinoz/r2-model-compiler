// Vector, quaternion and matrix helpers using Source engine conventions.
//
// RadianEuler is (x = roll, y = pitch, z = yaw) in radians, as stored in SMD files
// and in mstudiobone_t::rot. matrix3x4 is row-major with the translation in column 3.
#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>

namespace r2c {

constexpr float kPi = 3.14159265358979323846f;

struct Vec2 {
    float x = 0, y = 0;
    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
};

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    bool operator==(const Vec3& o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator!=(const Vec3& o) const { return !(*this == o); }
};

inline float Dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float Length(const Vec3& v) { return std::sqrt(Dot(v, v)); }
inline Vec3 Normalize(const Vec3& v) {
    float l = Length(v);
    return l > 1e-20f ? v / l : Vec3(0, 0, 0);
}
inline Vec3 Min(const Vec3& a, const Vec3& b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 Max(const Vec3& a, const Vec3& b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }

struct Vec4 {
    float x = 0, y = 0, z = 0, w = 0;
};

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
    Quat() = default;
    Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

// Roll (x), pitch (y), yaw (z) in radians.
struct RadianEuler {
    float x = 0, y = 0, z = 0;
    RadianEuler() = default;
    RadianEuler(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    float& operator[](int i) { return (&x)[i]; }
    float operator[](int i) const { return (&x)[i]; }
};

struct Mat3x4 {
    float m[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
    Vec3 Origin() const { return {m[0][3], m[1][3], m[2][3]}; }
    void SetOrigin(const Vec3& v) { m[0][3] = v.x; m[1][3] = v.y; m[2][3] = v.z; }
    Vec3 Column(int c) const { return {m[0][c], m[1][c], m[2][c]}; }
};

inline Mat3x4 Identity() { return Mat3x4(); }

inline Mat3x4 Concat(const Mat3x4& a, const Mat3x4& b) {
    Mat3x4 o;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            o.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
        o.m[i][3] += a.m[i][3];
    }
    return o;
}

inline Vec3 Transform(const Mat3x4& m, const Vec3& v) {
    return {m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z + m.m[0][3],
            m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z + m.m[1][3],
            m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z + m.m[2][3]};
}
inline Vec3 Rotate(const Mat3x4& m, const Vec3& v) {
    return {m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
            m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
            m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z};
}
// Inverse rotation (transpose), ignoring translation.
inline Vec3 IRotate(const Mat3x4& m, const Vec3& v) {
    return {m.m[0][0] * v.x + m.m[1][0] * v.y + m.m[2][0] * v.z,
            m.m[0][1] * v.x + m.m[1][1] * v.y + m.m[2][1] * v.z,
            m.m[0][2] * v.x + m.m[1][2] * v.y + m.m[2][2] * v.z};
}
// Inverse of a rigid transform.
inline Mat3x4 InvertRigid(const Mat3x4& m) {
    Mat3x4 o;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) o.m[i][j] = m.m[j][i];
    Vec3 t = m.Origin();
    o.m[0][3] = -(t.x * o.m[0][0] + t.y * o.m[0][1] + t.z * o.m[0][2]);
    o.m[1][3] = -(t.x * o.m[1][0] + t.y * o.m[1][1] + t.z * o.m[1][2]);
    o.m[2][3] = -(t.x * o.m[2][0] + t.y * o.m[2][1] + t.z * o.m[2][2]);
    return o;
}
// General affine inverse (handles scale).
Mat3x4 Invert(const Mat3x4& m);

Mat3x4 AngleMatrix(const RadianEuler& a);
Mat3x4 AngleMatrix(const RadianEuler& a, const Vec3& pos);
Mat3x4 AngleMatrix(const RadianEuler& a, const Vec3& pos, const Vec3& scale);
RadianEuler MatrixAngles(const Mat3x4& m);
Quat AngleQuaternion(const RadianEuler& a);
RadianEuler QuaternionAngles(const Quat& q);
Mat3x4 QuaternionMatrix(const Quat& q);
Mat3x4 QuaternionMatrix(const Quat& q, const Vec3& pos);
Quat MatrixQuaternion(const Mat3x4& m);
Quat QuaternionNormalize(const Quat& q);
Quat QuaternionAlign(const Quat& p, const Quat& q);  // q flipped to the hemisphere of p
Quat QuaternionMult(const Quat& p, const Quat& q);
Quat QuaternionConjugate(const Quat& q);
Quat QuaternionSlerp(const Quat& p, const Quat& q, float t);
float QuaternionDot(const Quat& a, const Quat& b);
// Matrix scale helpers
Vec3 MatrixScale(const Mat3x4& m);

inline float DegToRad(float d) { return d * (kPi / 180.f); }
inline float RadToDeg(float r) { return r * (180.f / kPi); }

// Compressed formats used in animation data.
uint64_t PackQuaternion64(const Quat& q);
Quat UnpackQuaternion64(uint64_t v);
uint16_t FloatToHalf(float f);
float HalfToFloat(uint16_t h);

}  // namespace r2c
