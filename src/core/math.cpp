#include "core/math.h"

#include <cstring>

namespace r2c {

Mat3x4 Invert(const Mat3x4& in) {
    // 3x3 inverse via cofactors, then translation.
    const float(&m)[3][4] = in.m;
    float a = m[0][0], b = m[0][1], c = m[0][2];
    float d = m[1][0], e = m[1][1], f = m[1][2];
    float g = m[2][0], h = m[2][1], i = m[2][2];
    float A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
    float det = a * A + b * B + c * C;
    Mat3x4 o;
    if (std::fabs(det) < 1e-30f) return Identity();
    float inv = 1.f / det;
    o.m[0][0] = A * inv;
    o.m[0][1] = -(b * i - c * h) * inv;
    o.m[0][2] = (b * f - c * e) * inv;
    o.m[1][0] = B * inv;
    o.m[1][1] = (a * i - c * g) * inv;
    o.m[1][2] = -(a * f - c * d) * inv;
    o.m[2][0] = C * inv;
    o.m[2][1] = -(a * h - b * g) * inv;
    o.m[2][2] = (a * e - b * d) * inv;
    Vec3 t = in.Origin();
    for (int r = 0; r < 3; r++) o.m[r][3] = -(o.m[r][0] * t.x + o.m[r][1] * t.y + o.m[r][2] * t.z);
    return o;
}

Mat3x4 AngleMatrix(const RadianEuler& a) {
    // Source: AngleMatrix(QAngle(pitch = a.y, yaw = a.z, roll = a.x))
    float sy = std::sin(a.z), cy = std::cos(a.z);
    float sp = std::sin(a.y), cp = std::cos(a.y);
    float sr = std::sin(a.x), cr = std::cos(a.x);
    Mat3x4 m;
    m.m[0][0] = cp * cy;
    m.m[1][0] = cp * sy;
    m.m[2][0] = -sp;
    float crcy = cr * cy, crsy = cr * sy, srcy = sr * cy, srsy = sr * sy;
    m.m[0][1] = sp * srcy - crsy;
    m.m[1][1] = sp * srsy + crcy;
    m.m[2][1] = sr * cp;
    m.m[0][2] = sp * crcy + srsy;
    m.m[1][2] = sp * crsy - srcy;
    m.m[2][2] = cr * cp;
    m.m[0][3] = m.m[1][3] = m.m[2][3] = 0;
    return m;
}

Mat3x4 AngleMatrix(const RadianEuler& a, const Vec3& pos) {
    Mat3x4 m = AngleMatrix(a);
    m.SetOrigin(pos);
    return m;
}

RadianEuler MatrixAngles(const Mat3x4& m) {
    float forward[3] = {m.m[0][0], m.m[1][0], m.m[2][0]};
    float left[3] = {m.m[0][1], m.m[1][1], m.m[2][1]};
    float up2 = m.m[2][2];
    float xyDist = std::sqrt(forward[0] * forward[0] + forward[1] * forward[1]);
    RadianEuler a;
    if (xyDist > 0.001f) {
        a.z = std::atan2(forward[1], forward[0]);
        a.y = std::atan2(-forward[2], xyDist);
        a.x = std::atan2(left[2], up2);
    } else {
        a.z = std::atan2(-left[0], left[1]);
        a.y = std::atan2(-forward[2], xyDist);
        a.x = 0;
    }
    return a;
}

Quat AngleQuaternion(const RadianEuler& a) {
    float sr = std::sin(a.x * 0.5f), cr = std::cos(a.x * 0.5f);
    float sp = std::sin(a.y * 0.5f), cp = std::cos(a.y * 0.5f);
    float sy = std::sin(a.z * 0.5f), cy = std::cos(a.z * 0.5f);
    float srXcp = sr * cp, crXsp = cr * sp;
    float crXcp = cr * cp, srXsp = sr * sp;
    Quat q;
    q.x = srXcp * cy - crXsp * sy;
    q.y = crXsp * cy + srXcp * sy;
    q.z = crXcp * sy - srXsp * cy;
    q.w = crXcp * cy + srXsp * sy;
    return q;
}

Mat3x4 QuaternionMatrix(const Quat& q) {
    Mat3x4 m;
    m.m[0][0] = 1.0f - 2.0f * q.y * q.y - 2.0f * q.z * q.z;
    m.m[1][0] = 2.0f * q.x * q.y + 2.0f * q.w * q.z;
    m.m[2][0] = 2.0f * q.x * q.z - 2.0f * q.w * q.y;
    m.m[0][1] = 2.0f * q.x * q.y - 2.0f * q.w * q.z;
    m.m[1][1] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.z * q.z;
    m.m[2][1] = 2.0f * q.y * q.z + 2.0f * q.w * q.x;
    m.m[0][2] = 2.0f * q.x * q.z + 2.0f * q.w * q.y;
    m.m[1][2] = 2.0f * q.y * q.z - 2.0f * q.w * q.x;
    m.m[2][2] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y;
    m.m[0][3] = m.m[1][3] = m.m[2][3] = 0;
    return m;
}

Mat3x4 QuaternionMatrix(const Quat& q, const Vec3& pos) {
    Mat3x4 m = QuaternionMatrix(q);
    m.SetOrigin(pos);
    return m;
}

RadianEuler QuaternionAngles(const Quat& q) { return MatrixAngles(QuaternionMatrix(q)); }

Quat MatrixQuaternion(const Mat3x4& mat) {
    // Normalise columns first so scaled matrices still give a unit quaternion.
    Mat3x4 m = mat;
    for (int c = 0; c < 3; c++) {
        Vec3 col = Normalize(m.Column(c));
        m.m[0][c] = col.x; m.m[1][c] = col.y; m.m[2][c] = col.z;
    }
    Quat q;
    float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
    if (trace > 0) {
        float s = std::sqrt(trace + 1.0f) * 2.f;
        q.w = 0.25f * s;
        q.x = (m.m[2][1] - m.m[1][2]) / s;
        q.y = (m.m[0][2] - m.m[2][0]) / s;
        q.z = (m.m[1][0] - m.m[0][1]) / s;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        float s = std::sqrt(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.f;
        q.w = (m.m[2][1] - m.m[1][2]) / s;
        q.x = 0.25f * s;
        q.y = (m.m[0][1] + m.m[1][0]) / s;
        q.z = (m.m[0][2] + m.m[2][0]) / s;
    } else if (m.m[1][1] > m.m[2][2]) {
        float s = std::sqrt(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.f;
        q.w = (m.m[0][2] - m.m[2][0]) / s;
        q.x = (m.m[0][1] + m.m[1][0]) / s;
        q.y = 0.25f * s;
        q.z = (m.m[1][2] + m.m[2][1]) / s;
    } else {
        float s = std::sqrt(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.f;
        q.w = (m.m[1][0] - m.m[0][1]) / s;
        q.x = (m.m[0][2] + m.m[2][0]) / s;
        q.y = (m.m[1][2] + m.m[2][1]) / s;
        q.z = 0.25f * s;
    }
    return QuaternionNormalize(q);
}

float QuaternionDot(const Quat& a, const Quat& b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

Quat QuaternionNormalize(const Quat& q) {
    float l = std::sqrt(QuaternionDot(q, q));
    if (l < 1e-20f) return Quat();
    return {q.x / l, q.y / l, q.z / l, q.w / l};
}

Quat QuaternionAlign(const Quat& p, const Quat& q) {
    // Pick whichever of q / -q is closer to p.
    float a = 0, b = 0;
    for (int i = 0; i < 4; i++) {
        a += (p[i] - q[i]) * (p[i] - q[i]);
        b += (p[i] + q[i]) * (p[i] + q[i]);
    }
    if (a > b) return {-q.x, -q.y, -q.z, -q.w};
    return q;
}

Quat QuaternionMult(const Quat& p, const Quat& q) {
    Quat qt = QuaternionAlign(p, q);
    Quat o;
    o.x = p.x * qt.w + p.y * qt.z - p.z * qt.y + p.w * qt.x;
    o.y = -p.x * qt.z + p.y * qt.w + p.z * qt.x + p.w * qt.y;
    o.z = p.x * qt.y - p.y * qt.x + p.z * qt.w + p.w * qt.z;
    o.w = -p.x * qt.x - p.y * qt.y - p.z * qt.z + p.w * qt.w;
    return o;
}

Quat QuaternionConjugate(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }

Quat QuaternionSlerp(const Quat& p, const Quat& q0, float t) {
    Quat q = QuaternionAlign(p, q0);
    float cosom = QuaternionDot(p, q);
    float sclp, sclq;
    if ((1.0f + cosom) > 0.000001f) {
        if ((1.0f - cosom) > 0.000001f) {
            float omega = std::acos(cosom);
            float sinom = std::sin(omega);
            sclp = std::sin((1.0f - t) * omega) / sinom;
            sclq = std::sin(t * omega) / sinom;
        } else {
            sclp = 1.0f - t;
            sclq = t;
        }
        return QuaternionNormalize({sclp * p.x + sclq * q.x, sclp * p.y + sclq * q.y, sclp * p.z + sclq * q.z,
                                    sclp * p.w + sclq * q.w});
    }
    Quat qt(-q.y, q.x, -q.w, q.z);
    sclp = std::sin((1.0f - t) * (0.5f * kPi));
    sclq = std::sin(t * (0.5f * kPi));
    return {sclp * p.x + sclq * qt.x, sclp * p.y + sclq * qt.y, sclp * p.z + sclq * qt.z, qt.w};
}

Vec3 MatrixScale(const Mat3x4& m) { return {Length(m.Column(0)), Length(m.Column(1)), Length(m.Column(2))}; }

uint64_t PackQuaternion64(const Quat& q) {
    auto enc = [](float v) -> uint64_t {
        int i = static_cast<int>(v * 1048576.f) + 1048576;
        return static_cast<uint64_t>(std::clamp(i, 0, 2097151));
    };
    uint64_t x = enc(q.x), y = enc(q.y), z = enc(q.z), wneg = q.w < 0 ? 1 : 0;
    return x | (y << 21) | (z << 42) | (wneg << 63);
}

Quat UnpackQuaternion64(uint64_t v) {
    Quat q;
    q.x = (static_cast<int>(v & 0x1FFFFF) - 1048576) * (1 / 1048576.5f);
    q.y = (static_cast<int>((v >> 21) & 0x1FFFFF) - 1048576) * (1 / 1048576.5f);
    q.z = (static_cast<int>((v >> 42) & 0x1FFFFF) - 1048576) * (1 / 1048576.5f);
    float d = 1.0f - (q.x * q.x + q.y * q.y + q.z * q.z);
    q.w = d > 0 ? std::sqrt(d) : 0;
    if (v >> 63) q.w = -q.w;
    return q;
}

uint16_t FloatToHalf(float f) {
    // IEEE 754 half, clamped to the largest finite value (Source float16 has no inf).
    uint32_t bits;
    std::memcpy(&bits, &f, 4);
    uint32_t sign = (bits >> 16) & 0x8000;
    float af = std::fabs(f);
    if (!(af == af)) return static_cast<uint16_t>(sign);  // NaN -> 0
    if (af >= 65504.f) return static_cast<uint16_t>(sign | 0x7BFF);
    if (af < 5.96046448e-08f) return static_cast<uint16_t>(sign);
    int exp;
    float mant = std::frexp(af, &exp);  // af = mant * 2^exp, mant in [0.5,1)
    int e = exp - 1 + 15;
    if (e <= 0) {
        // subnormal
        int m = static_cast<int>(std::lround(af / 5.96046448e-08f));
        return static_cast<uint16_t>(sign | m);
    }
    int m = static_cast<int>(std::lround((mant * 2.f - 1.f) * 1024.f));
    if (m == 1024) {
        m = 0;
        e++;
        if (e >= 31) return static_cast<uint16_t>(sign | 0x7BFF);
    }
    return static_cast<uint16_t>(sign | (e << 10) | m);
}

float HalfToFloat(uint16_t h) {
    int sign = (h >> 15) & 1, e = (h >> 10) & 0x1F, m = h & 0x3FF;
    float v;
    if (e == 0)
        v = m * 5.96046448e-08f;
    else
        v = std::ldexp(1.f + m / 1024.f, e - 15);
    return sign ? -v : v;
}

}  // namespace r2c
