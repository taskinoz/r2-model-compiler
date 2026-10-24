#include "build/hull.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>

namespace r2c {

namespace {

struct Face {
    int v[3];
    DVec3 n;
    bool alive = true;
    std::vector<int> outside;
    int farthest = -1;
    double distance = 0;
};

uint64_t EdgeKey(int a, int b) { return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b); }

}  // namespace

Hull ConvexHull(const std::vector<DVec3>& input) {
    Hull out;
    if (input.size() < 4) return out;
    DVec3 mn = input[0], mx = input[0];
    for (auto& p : input) {
        mn = DVec3(std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z));
        mx = DVec3(std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z));
    }
    double size = DLen(mx - mn);
    if (size <= 0) return out;
    const double eps = size * 1e-7;

    // Weld within the plane tolerance. Exact equality leaves microscopic edges
    // after float transforms of repeated SMD vertices. Search neighbouring cells
    // as well so points on opposite sides of a cell boundary still weld.
    std::vector<DVec3> pts;
    std::map<std::tuple<int64_t, int64_t, int64_t>, std::vector<int>> cells;
    for (auto& p : input) {
        int64_t x = static_cast<int64_t>(std::floor((p.x - mn.x) / eps));
        int64_t y = static_cast<int64_t>(std::floor((p.y - mn.y) / eps));
        int64_t z = static_cast<int64_t>(std::floor((p.z - mn.z) / eps));
        bool duplicate = false;
        for (int dx = -1; dx <= 1 && !duplicate; ++dx)
            for (int dy = -1; dy <= 1 && !duplicate; ++dy)
                for (int dz = -1; dz <= 1 && !duplicate; ++dz) {
                    auto it = cells.find({x + dx, y + dy, z + dz});
                    if (it == cells.end()) continue;
                    for (int i : it->second)
                        if (DLen(p - pts[i]) <= eps) { duplicate = true; break; }
                }
        if (!duplicate) {
            cells[{x, y, z}].push_back(static_cast<int>(pts.size()));
            pts.push_back(p);
        }
    }
    if (pts.size() < 4) return out;

    // initial simplex: extreme points
    int i0 = 0, i1 = -1, i2 = -1, i3 = -1;
    for (size_t i = 0; i < pts.size(); i++)
        if (pts[i].x < pts[i0].x) i0 = static_cast<int>(i);
    double best = -1;
    for (size_t i = 0; i < pts.size(); i++) {
        double d = DLen(pts[i] - pts[i0]);
        if (d > best) {
            best = d;
            i1 = static_cast<int>(i);
        }
    }
    best = -1;
    for (size_t i = 0; i < pts.size(); i++) {
        double d = DLen(DCross(pts[i1] - pts[i0], pts[i] - pts[i0]));
        if (d > best) {
            best = d;
            i2 = static_cast<int>(i);
        }
    }
    if (best <= eps * size) return out;
    DVec3 n = DCross(pts[i1] - pts[i0], pts[i2] - pts[i0]);
    best = -1;
    for (size_t i = 0; i < pts.size(); i++) {
        double d = std::fabs(DDot(n, pts[i] - pts[i0])) / DLen(n);
        if (d > best) {
            best = d;
            i3 = static_cast<int>(i);
        }
    }
    if (best <= eps * 10) return out;

    std::vector<Face> faces;
    std::unordered_map<uint64_t, int> edgeFace;  // directed edge -> face
    auto addFace = [&](int a, int b, int c) {
        Face f;
        f.v[0] = a;
        f.v[1] = b;
        f.v[2] = c;
        DVec3 nn = DCross(pts[b] - pts[a], pts[c] - pts[a]);
        double l = DLen(nn);
        f.n = l > 0 ? nn * (1.0 / l) : nn;
        faces.push_back(f);
        int id = static_cast<int>(faces.size() - 1);
        edgeFace[EdgeKey(a, b)] = id;
        edgeFace[EdgeKey(b, c)] = id;
        edgeFace[EdgeKey(c, a)] = id;
    };
    // orient the tetrahedron outward
    if (DDot(n, pts[i3] - pts[i0]) > 0) std::swap(i1, i2);
    addFace(i0, i1, i2);
    addFace(i0, i3, i1);
    addFace(i1, i3, i2);
    addFace(i2, i3, i0);

    auto distance = [&](int f, const DVec3& p) {
        return DDot(faces[f].n, p - pts[faces[f].v[0]]);
    };
    // Each outside point belongs to one face. Only points on removed faces need
    // reassignment, avoiding a full point-cloud scan on every hull expansion.
    auto assign = [&](int pi, size_t firstFace) {
        int owner = -1;
        double bestDistance = eps;
        for (size_t f = firstFace; f < faces.size(); ++f) {
            if (!faces[f].alive) continue;
            double d = distance(static_cast<int>(f), pts[pi]);
            if (d > bestDistance) { bestDistance = d; owner = static_cast<int>(f); }
        }
        if (owner < 0) return;
        Face& f = faces[owner];
        f.outside.push_back(pi);
        if (bestDistance > f.distance) { f.distance = bestDistance; f.farthest = pi; }
    };
    for (size_t pi = 0; pi < pts.size(); ++pi)
        if (pi != static_cast<size_t>(i0) && pi != static_cast<size_t>(i1) &&
            pi != static_cast<size_t>(i2) && pi != static_cast<size_t>(i3)) assign(static_cast<int>(pi), 0);
    for (;;) {
        int pi = -1;
        double bestDistance = eps;
        for (auto& f : faces)
            if (f.alive && f.distance > bestDistance) { bestDistance = f.distance; pi = f.farthest; }
        if (pi < 0) break;
        const DVec3& p = pts[pi];
        std::vector<int> visible;
        for (size_t f = 0; f < faces.size(); f++)
            // The outside-point tolerance must not clip the visible region:
            // leaving a slightly visible neighbour creates overlapping faces
            // along the horizon on nearly coplanar float-transformed meshes.
            if (faces[f].alive && distance(static_cast<int>(f), p) > size * 1e-12)
                visible.push_back(static_cast<int>(f));
        std::vector<char> isVis(faces.size(), 0);
        for (int f : visible) isVis[f] = 1;
        // horizon: edges of visible faces whose twin face is not visible
        std::vector<std::pair<int, int>> horizon;
        for (int f : visible) {
            for (int e = 0; e < 3; e++) {
                int a = faces[f].v[e], b = faces[f].v[(e + 1) % 3];
                auto it = edgeFace.find(EdgeKey(b, a));
                if (it == edgeFace.end() || !isVis[it->second]) horizon.push_back({a, b});
            }
        }
        std::vector<int> orphaned;
        for (int f : visible) {
            faces[f].alive = false;
            for (int point : faces[f].outside)
                if (point != pi) orphaned.push_back(point);
            faces[f].outside.clear();
            for (int e = 0; e < 3; e++) {
                int a = faces[f].v[e], b = faces[f].v[(e + 1) % 3];
                auto it = edgeFace.find(EdgeKey(a, b));
                if (it != edgeFace.end() && it->second == f) edgeFace.erase(it);
            }
        }
        size_t firstNewFace = faces.size();
        for (auto& h : horizon) addFace(h.first, h.second, pi);
        for (int point : orphaned) assign(point, firstNewFace);
    }
    // compact output
    std::vector<int> remap(pts.size(), -1);
    for (auto& f : faces) {
        if (!f.alive) continue;
        std::array<int, 3> t;
        for (int k = 0; k < 3; k++) {
            int v = f.v[k];
            if (remap[v] < 0) {
                remap[v] = static_cast<int>(out.verts.size());
                out.verts.push_back(pts[v]);
            }
            t[k] = remap[v];
        }
        out.tris.push_back(t);
    }
    out.valid = out.tris.size() >= 4;
    return out;
}

MassProps ComputeMassProps(const Hull& h) {
    MassProps m;
    double V = 0;
    DVec3 C;
    double s[3][3] = {};
    for (auto& t : h.tris) {
        const DVec3 &a = h.verts[t[0]], &b = h.verts[t[1]], &c = h.verts[t[2]];
        double v = DDot(a, DCross(b, c)) / 6.0;
        V += v;
        C = C + (a + b + c) * (v / 4.0);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                s[i][j] += v / 20.0 * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + (a[i] + b[i] + c[i]) * (a[j] + b[j] + c[j]));
        m.area += DLen(DCross(b - a, c - a)) * 0.5;
    }
    m.volume = V;
    if (V <= 0) return m;
    m.center = C * (1.0 / V);
    // second moments about the centre
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) s[i][j] -= V * m.center[i] * m.center[j];
    m.inertia[0][0] = s[1][1] + s[2][2];
    m.inertia[1][1] = s[0][0] + s[2][2];
    m.inertia[2][2] = s[0][0] + s[1][1];
    m.inertia[0][1] = m.inertia[1][0] = -s[0][1];
    m.inertia[0][2] = m.inertia[2][0] = -s[0][2];
    m.inertia[1][2] = m.inertia[2][1] = -s[1][2];
    return m;
}

}  // namespace r2c
