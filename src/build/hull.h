// 3D convex hull (incremental, double precision) and mass properties.
#pragma once

#include <array>
#include <vector>

#include "core/math.h"

namespace r2c {

struct DVec3 {
    double x = 0, y = 0, z = 0;
    DVec3() = default;
    DVec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    explicit DVec3(const Vec3& v) : x(v.x), y(v.y), z(v.z) {}
    DVec3 operator+(const DVec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    DVec3 operator-(const DVec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    DVec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
    Vec3 F() const { return Vec3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)); }
};
inline double DDot(const DVec3& a, const DVec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline DVec3 DCross(const DVec3& a, const DVec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double DLen(const DVec3& a) { return std::sqrt(DDot(a, a)); }

struct Hull {
    std::vector<DVec3> verts;                  // only points on the hull
    std::vector<std::array<int, 3>> tris;      // CCW seen from outside
    bool valid = false;
};

// Builds the convex hull of the points. Returns an invalid hull for fewer than
// four non-coplanar points.
Hull ConvexHull(const std::vector<DVec3>& pts);

struct MassProps {
    double volume = 0;
    DVec3 center;
    double inertia[3][3] = {};  // about the centre of mass (not divided by volume)
    double area = 0;
};
MassProps ComputeMassProps(const Hull& h);

}  // namespace r2c
