// Collision models: $collisionmodel (one rigid solid, optionally $concave) and
// $collisionjoints (one solid per bone, ragdoll constraints) -> PHY (IVP compact
// surfaces plus keyvalue text), and Respawn's per-piece "map collision" block.
//
// Behaviour follows studiomdl's collisionmodel.cpp; the IVP layout matches retail
// Titanfall 2 files: piece ledges, then a root convex-hull ledge, one shared point
// array, then the ledge tree (pre-order, left child adjacent).
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <numeric>
#include <map>
#include <set>
#include <unordered_map>

#include "build/context.h"
#include "build/hull.h"
#include "build/surfaceprops.h"
#include "core/buffer.h"
#include "core/log.h"
#include "core/strings.h"

namespace r2c {

namespace {

constexpr double kInch = 0.0254;
constexpr double kCubicMetersPerCubicInch = kInch * kInch * kInch;

float DensityFor(const std::string& sp) {
    int i = SurfacePropIndex(sp);
    if (i >= 0) return kSurfaceProps[i].density;
    Warn("unknown surfaceprop \"%s\", using default density", sp.c_str());
    return 2000;
}

// HL (inches, Z up) -> IVP (metres, Y down): (x, -z, y) * 0.0254
DVec3 ToIVP(const DVec3& v) { return DVec3(v.x * kInch, -v.z * kInch, v.y * kInch); }

struct Solid {
    std::string name, parent;
    int bone = 0;           // global bone the solid follows
    int clientData = 0;     // ledge client data (bone + 1 for jointed solids)
    std::vector<Hull> pieces;  // HL units, solid space
    double volume = 0;      // in^3
    double area = 0;
    float damping = 0, rotdamping = 0, inertia = 1, massBias = 1, drag = -1;
    float mass = 1;
    std::string surfaceProp;
};

// ---------------------------------------------------------------------------
// IVP compact surface
// ---------------------------------------------------------------------------

struct IvpPiece {
    std::vector<int> pointIdx;               // hull vertex -> global point index
    std::vector<std::array<int, 3>> tris;    // hull triangles (hull vertex indices)
    DVec3 center;                            // AABB centre of the piece (IVP)
};

struct TreeNode {
    std::vector<int> pieces;
    int left = -1, right = -1;
};

std::vector<uint8_t> BuildLedge(const std::vector<DVec3>& points, const std::vector<int>& pidx,
                                const std::vector<std::array<int, 3>>& tris, int clientOrNode, bool hasChildren,
                                int pointOffsetFromLedge) {
    Buffer b;
    int ntri = static_cast<int>(tris.size());
    b.I32(pointOffsetFromLedge);
    b.I32(clientOrNode);
    uint32_t size16 = static_cast<uint32_t>((16 + 16 * ntri) / 16);
    uint32_t bits = (hasChildren ? 1u : 0u) | (1u << 2) | (size16 << 8);
    b.U32(bits);
    b.I16(static_cast<int16_t>(ntri));
    b.I16(0);
    // edge twins
    std::map<std::pair<int, int>, int> edgeSlot;
    for (int t = 0; t < ntri; t++)
        for (int e = 0; e < 3; e++) edgeSlot[{tris[t][e], tris[t][(e + 1) % 3]}] = t * 4 + 1 + e;
    // planes for pierce indices
    std::vector<DVec3> n(ntri), c(ntri);
    std::vector<double> d(ntri);
    for (int t = 0; t < ntri; t++) {
        const DVec3 &a = points[pidx[tris[t][0]]], &bb = points[pidx[tris[t][1]]], &cc = points[pidx[tris[t][2]]];
        DVec3 nn = DCross(bb - a, cc - a);
        double l = DLen(nn);
        n[t] = l > 0 ? nn * (1 / l) : nn;
        d[t] = DDot(n[t], a);
        c[t] = (a + bb + cc) * (1.0 / 3.0);
    }
    for (int t = 0; t < ntri; t++) {
        // pierce: the face the inward normal ray from this face's centroid exits through
        int pierce = t;
        double bestT = 1e30;
        DVec3 dir = n[t] * -1.0;
        for (int u = 0; u < ntri; u++) {
            if (u == t) continue;
            double den = DDot(n[u], dir);
            if (den <= 1e-9) continue;
            double tt = (d[u] - DDot(n[u], c[t])) / den;
            if (tt <= 1e-9) continue;
            DVec3 hit = c[t] + dir * tt;
            // inside triangle u?
            const DVec3 &a = points[pidx[tris[u][0]]], &bb = points[pidx[tris[u][1]]], &cc = points[pidx[tris[u][2]]];
            bool inside = DDot(DCross(bb - a, hit - a), n[u]) >= -1e-9 && DDot(DCross(cc - bb, hit - bb), n[u]) >= -1e-9 &&
                          DDot(DCross(a - cc, hit - cc), n[u]) >= -1e-9;
            double score = inside ? tt : tt + 1e6;
            if (score < bestT) {
                bestT = score;
                pierce = u;
            }
        }
        uint32_t tb = (static_cast<uint32_t>(t) & 0xFFF) | ((static_cast<uint32_t>(pierce) & 0xFFF) << 12);
        b.U32(tb);
        for (int e = 0; e < 3; e++) {
            int a = tris[t][e], bb = tris[t][(e + 1) % 3];
            int self = t * 4 + 1 + e;
            auto it = edgeSlot.find({bb, a});
            int opp = it == edgeSlot.end() ? 0 : it->second - self;
            uint32_t eb = (static_cast<uint32_t>(pidx[a]) & 0xFFFF) | ((static_cast<uint32_t>(opp) & 0x7FFF) << 16);
            b.U32(eb);
        }
    }
    return b.Data();
}

// Builds the IVP compact surface for one solid. Returns bytes starting at the legacy
// surface header. Also outputs the drag fractions.
std::vector<uint8_t> BuildCompactSurface(const Solid& s, DVec3& dragOut) {
    // global point array (IVP space) and pieces
    std::vector<DVec3> points;
    std::vector<IvpPiece> pieces;
    std::vector<Hull> ivpHulls;
    for (auto& h : s.pieces) {
        IvpPiece p;
        Hull ih;
        ih.valid = true;
        ih.tris = h.tris;
        for (auto& v : h.verts) {
            DVec3 iv = ToIVP(v);
            p.pointIdx.push_back(static_cast<int>(points.size()));
            points.push_back(iv);
            ih.verts.push_back(iv);
        }
        // IVP point indices are 16-bit
        p.tris = h.tris;
        DVec3 mn = ih.verts[0], mx = ih.verts[0];
        for (auto& v : ih.verts) {
            mn = DVec3(std::min(mn.x, v.x), std::min(mn.y, v.y), std::min(mn.z, v.z));
            mx = DVec3(std::max(mx.x, v.x), std::max(mx.y, v.y), std::max(mx.z, v.z));
        }
        p.center = (mn + mx) * 0.5;
        pieces.push_back(p);
        ivpHulls.push_back(ih);
    }
    if (points.size() > 65535) Fatal("collision model %s has too many points", s.name.c_str());

    // mass properties
    double V = 0;
    DVec3 C;
    std::vector<MassProps> mp;
    for (auto& h : ivpHulls) {
        mp.push_back(ComputeMassProps(h));
        V += mp.back().volume;
        C = C + mp.back().center * mp.back().volume;
    }
    if (V > 0) C = C * (1.0 / V);
    double I[3] = {0, 0, 0};
    for (size_t i = 0; i < mp.size(); i++) {
        DVec3 dd = mp[i].center - C;
        double dsq[3] = {dd.y * dd.y + dd.z * dd.z, dd.x * dd.x + dd.z * dd.z, dd.x * dd.x + dd.y * dd.y};
        for (int k = 0; k < 3; k++) I[k] += mp[i].inertia[k][k] + mp[i].volume * dsq[k];
    }
    double ulr = 0;
    for (auto& p : points) ulr = std::max(ulr, DLen(p - C));

    // drag fractions: share of each axis-aligned bounding face covered by the shape
    {
        DVec3 mn = points[0], mx = points[0];
        for (auto& p : points) {
            mn = DVec3(std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z));
            mx = DVec3(std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z));
        }
        double frac[3];
        const int N = 24;
        for (int axis = 0; axis < 3; axis++) {
            int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
            int hit = 0;
            for (int i = 0; i < N; i++)
                for (int j = 0; j < N; j++) {
                    double u = mn[a1] + (mx[a1] - mn[a1]) * (i + 0.5) / N;
                    double v = mn[a2] + (mx[a2] - mn[a2]) * (j + 0.5) / N;
                    bool any = false;
                    for (auto& h : ivpHulls) {
                        // line along axis through (u,v): inside the hull's projection?
                        double tmin = -1e30, tmax = 1e30;
                        bool ok = true;
                        for (auto& t : h.tris) {
                            DVec3 nn = DCross(h.verts[t[1]] - h.verts[t[0]], h.verts[t[2]] - h.verts[t[0]]);
                            double dd = DDot(nn, h.verts[t[0]]);
                            DVec3 o;
                            double oc[3];
                            oc[axis] = 0;
                            oc[a1] = u;
                            oc[a2] = v;
                            o = DVec3(oc[0], oc[1], oc[2]);
                            double den = nn[axis];
                            double num = dd - DDot(nn, o);
                            if (std::fabs(den) < 1e-12) {
                                if (num < 0) {
                                    ok = false;
                                    break;
                                }
                            } else if (den > 0)
                                tmax = std::min(tmax, num / den);
                            else
                                tmin = std::max(tmin, num / den);
                        }
                        if (ok && tmin <= tmax) {
                            any = true;
                            break;
                        }
                    }
                    if (any) hit++;
                }
            frac[axis] = static_cast<double>(hit) / (N * N);
        }
        dragOut = DVec3(frac[0], frac[1], frac[2]);
    }

    // ledge tree over pieces
    std::vector<TreeNode> nodes;
    std::function<int(std::vector<int>)> build = [&](std::vector<int> ps) -> int {
        int id = static_cast<int>(nodes.size());
        nodes.push_back(TreeNode());
        nodes[id].pieces = ps;
        if (ps.size() > 1) {
            DVec3 mn = pieces[ps[0]].center, mx = mn;
            for (int p : ps) {
                const DVec3& c = pieces[p].center;
                mn = DVec3(std::min(mn.x, c.x), std::min(mn.y, c.y), std::min(mn.z, c.z));
                mx = DVec3(std::max(mx.x, c.x), std::max(mx.y, c.y), std::max(mx.z, c.z));
            }
            DVec3 ext = mx - mn;
            int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : (ext.y >= ext.z ? 1 : 2);
            std::stable_sort(ps.begin(), ps.end(), [&](int a, int b) { return pieces[a].center[axis] < pieces[b].center[axis]; });
            size_t half = ps.size() / 2;
            std::vector<int> l(ps.begin(), ps.begin() + half), r(ps.begin() + half, ps.end());
            int li = build(l);
            int ri = build(r);
            nodes[id].left = li;
            nodes[id].right = ri;
        }
        return id;
    };
    std::vector<int> all;
    for (size_t i = 0; i < pieces.size(); i++) all.push_back(static_cast<int>(i));
    build(all);
    // pre-order layout (build() already creates nodes in pre-order)

    // root hull for multi-piece solids
    bool rootHull = pieces.size() > 1;
    std::vector<int> hullIdx;
    std::vector<std::array<int, 3>> hullTris;
    if (rootHull) {
        Hull h = ConvexHull(points);
        if (!h.valid)
            rootHull = false;
        else {
            for (auto& v : h.verts) {
                int found = -1;
                for (size_t i = 0; i < points.size(); i++)
                    if (points[i].x == v.x && points[i].y == v.y && points[i].z == v.z) {
                        found = static_cast<int>(i);
                        break;
                    }
                hullIdx.push_back(found);
            }
            hullTris = h.tris;
        }
    }

    // layout: header(48), piece ledges, root hull ledge, points, nodes
    size_t off = 48;
    std::vector<size_t> ledgeOff;
    for (auto& p : pieces) {
        ledgeOff.push_back(off);
        off += 16 + 16 * p.tris.size();
    }
    size_t hullOff = off;
    if (rootHull) off += 16 + 16 * hullTris.size();
    size_t pointsOff = off;
    off += 16 * points.size();
    size_t nodesOff = off;
    off += 28 * nodes.size();
    size_t total = off;

    Buffer b;
    b.Reserve(48);
    b.PatchAt<float>(0, static_cast<float>(C.x));
    b.PatchAt<float>(4, static_cast<float>(C.y));
    b.PatchAt<float>(8, static_cast<float>(C.z));
    for (int k = 0; k < 3; k++) b.PatchAt<float>(12 + k * 4, static_cast<float>(V > 0 ? I[k] / V : 0));
    b.PatchAt<float>(24, static_cast<float>(ulr));
    b.PatchAt<uint32_t>(28, 245u | (static_cast<uint32_t>(total) << 8));
    b.PatchI32(32, static_cast<int32_t>(nodesOff));
    b.PatchI32(44, 0x53505649);  // "IVPS"
    for (size_t i = 0; i < pieces.size(); i++) {
        auto led = BuildLedge(points, pieces[i].pointIdx, pieces[i].tris, s.clientData, false,
                              static_cast<int>(pointsOff - ledgeOff[i]));
        b.Append(led);
    }
    if (rootHull) {
        auto led = BuildLedge(points, hullIdx, hullTris, static_cast<int>(nodesOff - hullOff), true,
                              static_cast<int>(pointsOff - hullOff));
        b.Append(led);
    }
    for (auto& p : points) {
        b.F32(static_cast<float>(p.x));
        b.F32(static_cast<float>(p.y));
        b.F32(static_cast<float>(p.z));
        b.F32(0);
    }
    // nodes
    int leafCounter = 0;
    for (size_t i = 0; i < nodes.size(); i++) {
        const TreeNode& nd = nodes[i];
        size_t at = nodesOff + i * 28;
        std::vector<DVec3> pts;
        for (int p : nd.pieces)
            for (int idx : pieces[p].pointIdx) pts.push_back(points[idx]);
        DVec3 mn = pts[0], mx = pts[0];
        for (auto& p : pts) {
            mn = DVec3(std::min(mn.x, p.x), std::min(mn.y, p.y), std::min(mn.z, p.z));
            mx = DVec3(std::max(mx.x, p.x), std::max(mx.y, p.y), std::max(mx.z, p.z));
        }
        DVec3 center = (mn + mx) * 0.5;
        double radius = 0;
        for (auto& p : pts) radius = std::max(radius, DLen(p - center));
        int right = 0, ledge = 0;
        uint8_t freeByte = 0;
        if (nd.left >= 0) {
            right = static_cast<int>((nodesOff + nd.right * 28) - at);
            if (i == 0 && rootHull) ledge = static_cast<int>(hullOff) - static_cast<int>(at);
        } else {
            ledge = static_cast<int>(ledgeOff[nd.pieces[0]]) - static_cast<int>(at);
            freeByte = static_cast<uint8_t>(leafCounter++);
        }
        b.I32(right);
        b.I32(ledge);
        b.F32(static_cast<float>(center.x));
        b.F32(static_cast<float>(center.y));
        b.F32(static_cast<float>(center.z));
        b.F32(static_cast<float>(radius));
        for (int k = 0; k < 3; k++) {
            double e = mx[k] - mn[k];
            double v = radius > 0 ? std::ceil(e / radius * 125.0) : 0;
            b.U8(static_cast<uint8_t>(std::clamp(v, 0.0, 255.0)));
        }
        b.U8(freeByte);
        (void)at;
    }
    if (b.Size() != total) Fatal("internal: IVP surface size mismatch (%d vs %d)", static_cast<int>(b.Size()), static_cast<int>(total));
    return b.Data();
}

// ---------------------------------------------------------------------------
// Respawn map-collision block (one brush per convex piece, HL units)
// ---------------------------------------------------------------------------

// Retail rule (measured over mp_common): an edge gets a bevel plane only when its two
// faces' normals differ by more than ~75 degrees (dot < 0.26). Edges index their side
// planes and vertices with single bytes, so a piece may have at most 255 planes (faces
// plus bevels) and 255 vertices; a model with a larger piece carries no block. Retail
// pieces reach 82 faces / 99 planes (single-player rocks).
constexpr double kBevelDot = 0.26;
constexpr int kMaxBrushIndex = 255;

std::vector<uint8_t> BuildBrushPiece(const Hull& h, int& faces, int& bevels, int& edges) {
    const auto& tris = h.tris;
    const auto& triPolygon = h.triPolygon;
    struct Face {
        DVec3 n;
        double d;
        std::vector<int> tris;
        int polygon = -1;
        DVec3 areaNormal;  // sum of the member triangles' cross products
    };
    std::vector<Face> fs;
    std::vector<int> triFace(tris.size());
    double scale = 0;
    for (auto& v : h.verts) scale = std::max(scale, DLen(v));
    double dEps = std::max(1e-3, scale * 1e-5);
    std::map<int, int> polygonFace;
    for (size_t t = 0; t < tris.size(); t++) {
        const DVec3 &a = h.verts[tris[t][0]], &b = h.verts[tris[t][1]], &c = h.verts[tris[t][2]];
        DVec3 cross = DCross(b - a, c - a);
        DVec3 n = cross;
        double l = DLen(n);
        if (l > 0) n = n * (1 / l);
        double d = DDot(n, a);
        int polygon = t < triPolygon.size() ? triPolygon[t] : -1;
        int f = -1;
        if (polygon >= 0) {
            // one face per source polygon, even when slightly bent or coplanar with
            // a neighbouring polygon
            auto it = polygonFace.find(polygon);
            if (it != polygonFace.end()) f = it->second;
        } else {
            for (size_t k = 0; k < fs.size(); k++)
                if (fs[k].polygon < 0 && DDot(fs[k].n, n) > 0.99999 && std::fabs(fs[k].d - d) < dEps) {
                    f = static_cast<int>(k);
                    break;
                }
        }
        if (f < 0) {
            fs.push_back({n, d, {}, polygon, DVec3()});
            f = static_cast<int>(fs.size() - 1);
            if (polygon >= 0) polygonFace[polygon] = f;
        }
        fs[f].tris.push_back(static_cast<int>(t));
        fs[f].areaNormal = fs[f].areaNormal + cross;
        triFace[t] = f;
    }
    // a polygon face's plane: its area-weighted normal, pushed out to its outermost vertex
    for (auto& f : fs) {
        if (f.polygon < 0 || f.tris.size() < 2) continue;
        double l = DLen(f.areaNormal);
        if (l <= 0) continue;
        f.n = f.areaNormal * (1 / l);
        f.d = -1e300;
        for (int t : f.tris)
            for (int c = 0; c < 3; c++) f.d = std::max(f.d, DDot(f.n, h.verts[tris[t][c]]));
    }
    // directed edge -> face, boundary edges per face
    std::map<std::pair<int, int>, int> dirFace;
    for (size_t t = 0; t < tris.size(); t++)
        for (int e = 0; e < 3; e++) dirFace[{tris[t][e], tris[t][(e + 1) % 3]}] = triFace[t];
    struct Edge {
        int v0, v1, s0, s1;
    };
    std::vector<Edge> es;
    std::set<std::pair<int, int>> done;
    for (size_t f = 0; f < fs.size(); f++)
        for (int t : fs[f].tris)
            for (int e = 0; e < 3; e++) {
                int a = tris[t][e], b = tris[t][(e + 1) % 3];
                int other = dirFace[{b, a}];
                if (other == static_cast<int>(f)) continue;  // interior edge of a merged face
                auto key = std::make_pair(std::min(a, b), std::max(a, b));
                if (done.count(key)) continue;
                done.insert(key);
                es.push_back({a, b, static_cast<int>(f), other});
            }
    if (!h.brushLoops.empty()) {
        // faces are the source polygons: each loop's plane, and the loop edges shared
        // with the face whose loop runs the other way (a polygon's collinear points
        // stay on its edges, as retail's do)
        fs.clear();
        es.clear();
        done.clear();
        std::map<std::pair<int, int>, int> loopFace;
        for (size_t f = 0; f < h.brushLoops.size(); f++) {
            const auto& loop = h.brushLoops[f];
            DVec3 n;
            for (size_t k = 0; k < loop.size(); k++) {
                const DVec3 &p = h.verts[loop[k]], &q = h.verts[loop[(k + 1) % loop.size()]];
                n = n + DVec3((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
            }
            double l = DLen(n);
            if (l > 0) n = n * (1 / l);
            double d = -1e300;
            for (int v : loop) d = std::max(d, DDot(n, h.verts[v]));
            fs.push_back({n, d, {}, static_cast<int>(f), DVec3()});
            for (size_t k = 0; k < loop.size(); k++) loopFace[{loop[k], loop[(k + 1) % loop.size()]}] = static_cast<int>(f);
        }
        for (size_t f = 0; f < h.brushLoops.size(); f++) {
            const auto& loop = h.brushLoops[f];
            for (size_t k = 0; k < loop.size(); k++) {
                int a = loop[k], b = loop[(k + 1) % loop.size()];
                auto key = std::make_pair(std::min(a, b), std::max(a, b));
                if (done.count(key)) continue;
                done.insert(key);
                auto other = loopFace.find({b, a});
                int side = other != loopFace.end() ? other->second : -1;
                if (side < 0)  // no opposite edge: the other face that has both corners
                    for (size_t g = 0; g < h.brushLoops.size() && side < 0; g++)
                        if (g != f && std::count(h.brushLoops[g].begin(), h.brushLoops[g].end(), a) &&
                            std::count(h.brushLoops[g].begin(), h.brushLoops[g].end(), b))
                            side = static_cast<int>(g);
                es.push_back({a, b, static_cast<int>(f), side >= 0 ? side : static_cast<int>(f)});
            }
        }
    }
    faces = static_cast<int>(fs.size());
    edges = static_cast<int>(es.size());
    bevels = 0;
    Buffer out;
    for (auto& f : fs) {
        out.F32(static_cast<float>(f.n.x));
        out.F32(static_cast<float>(f.n.y));
        out.F32(static_cast<float>(f.n.z));
        out.F32(static_cast<float>(f.d));
    }
    for (auto& e : es) {
        if (DDot(fs[e.s0].n, fs[e.s1].n) >= kBevelDot) continue;
        bevels++;
        DVec3 n = fs[e.s0].n + fs[e.s1].n;
        double l = DLen(n);
        if (l > 0) n = n * (1 / l);
        out.F32(static_cast<float>(n.x));
        out.F32(static_cast<float>(n.y));
        out.F32(static_cast<float>(n.z));
        out.F32(static_cast<float>(DDot(n, h.verts[e.v0])));
    }
    for (auto& e : es) {
        DVec3 o = h.verts[e.v0], dd = h.verts[e.v1] - h.verts[e.v0];
        out.V3(o.F());
        out.V3(dd.F());
        out.U8(static_cast<uint8_t>(e.s0));
        out.U8(static_cast<uint8_t>(e.s1));
        out.U8(static_cast<uint8_t>(e.v0));
        out.U8(static_cast<uint8_t>(e.v1));
    }
    for (auto& v : h.verts) out.V3(v.F());
    return out.Data();
}

void KeyInt(std::string& s, const char* k, int v) { s += Format("\"%s\" \"%d\"\n", k, v); }
void KeyStr(std::string& s, const char* k, const std::string& v) { s += Format("\"%s\" \"%s\"\n", k, v.c_str()); }
void KeyFloat(std::string& s, const char* k, float v) { s += Format("\"%s\" \"%f\"\n", k, v); }

// Like studiomdl's BuildVertWeldTable: smoothing groups separate convex pieces
// even when they touch at identical positions.
std::vector<std::vector<int>> ConnectedComponents(const std::vector<Vec3>& pos, const std::vector<Vec3>& normals,
                                                 const std::vector<std::array<int, 3>>& faces, float normalBlend) {
    std::vector<int> parent(pos.size());
    for (size_t i = 0; i < parent.size(); i++) parent[i] = static_cast<int>(i);
    std::function<int(int)> find = [&](int x) { return parent[x] == x ? x : parent[x] = find(parent[x]); };
    auto unite = [&](int a, int b) { parent[find(a)] = find(b); };
    std::map<std::tuple<float, float, float>, std::vector<int>> weld;
    for (size_t i = 0; i < pos.size(); i++) {
        auto key = std::make_tuple(pos[i].x, pos[i].y, pos[i].z);
        auto& candidates = weld[key];
        for (int j : candidates)
            if (Dot(normals[i], normals[j]) > normalBlend) {
                unite(static_cast<int>(i), j);
                break;
            }
        candidates.push_back(static_cast<int>(i));
    }
    for (auto& f : faces) {
        unite(f[0], f[1]);
        unite(f[1], f[2]);
    }
    std::map<int, std::vector<int>> groups;
    for (size_t i = 0; i < pos.size(); i++) groups[find(static_cast<int>(i))].push_back(static_cast<int>(i));
    std::vector<std::vector<int>> out;
    // keep source order (by lowest vertex index)
    std::vector<std::pair<int, std::vector<int>>> ordered;
    for (auto& g : groups) ordered.push_back({g.second.front(), g.second});
    std::sort(ordered.begin(), ordered.end(), [](auto& a, auto& b) { return a.first < b.first; });
    for (auto& o : ordered) out.push_back(o.second);
    return out;
}

bool MakeHull(const std::vector<Vec3>& pts, Hull& out) {
    std::vector<DVec3> d;
    for (auto& p : pts) d.push_back(DVec3(p));
    out = ConvexHull(d);
    return out.valid;
}

// Points that lie on a hull's surface without being corners (a cylinder cap's centre,
// a point on an edge) stay as hull vertices, splitting the triangle (or the two
// triangles sharing the edge) they lie on. Retail PHY triangulates around such points
// and lists them in the map-collision piece. Returns how many were inserted.
int InsertSurfacePoints(Hull& h, const std::vector<Vec3>& pts) {
    double scale = 1;
    for (auto& v : h.verts) scale = std::max(scale, DLen(v));
    const double eps = std::max(1e-3, scale * 1e-6);
    int inserted = 0;
    for (auto& pf : pts) {
        DVec3 p(pf);
        bool corner = false;
        for (auto& v : h.verts)
            if (DLen(v - p) < eps) {
                corner = true;
                break;
            }
        if (corner) continue;
        for (size_t t = 0; t < h.tris.size(); t++) {
            int ia = h.tris[t][0], ib = h.tris[t][1], ic = h.tris[t][2];
            const DVec3 a = h.verts[ia], b = h.verts[ib], c = h.verts[ic];
            DVec3 n = DCross(b - a, c - a);
            double area = DLen(n);
            if (area <= 0) continue;
            n = n * (1 / area);
            if (std::fabs(DDot(n, p - a)) > eps) continue;
            // signed distance of p inside each edge (positive = inside)
            const DVec3* ends[3][2] = {{&a, &b}, {&b, &c}, {&c, &a}};
            double inside[3];
            for (int e = 0; e < 3; e++) {
                DVec3 edge = *ends[e][1] - *ends[e][0];
                double len = DLen(edge);
                inside[e] = len > 0 ? DDot(DCross(edge, p - *ends[e][0]), n) / len : -1;
            }
            if (inside[0] < -eps || inside[1] < -eps || inside[2] < -eps) continue;
            int ip = static_cast<int>(h.verts.size());
            h.verts.push_back(p);
            int onEdge = -1;
            for (int e = 0; e < 3; e++)
                if (inside[e] < eps) onEdge = e;
            if (onEdge < 0) {
                h.tris[t] = {ia, ib, ip};
                h.tris.push_back({ib, ic, ip});
                h.tris.push_back({ic, ia, ip});
            } else {
                int u = h.tris[t][onEdge], v = h.tris[t][(onEdge + 1) % 3], w = h.tris[t][(onEdge + 2) % 3];
                h.tris[t] = {u, ip, w};
                h.tris.push_back({ip, v, w});
                for (size_t t2 = 0; t2 < h.tris.size(); t2++) {
                    for (int e = 0; e < 3; e++)
                        if (h.tris[t2][e] == v && h.tris[t2][(e + 1) % 3] == u) {
                            int x = h.tris[t2][(e + 2) % 3];
                            h.tris[t2] = {v, ip, x};
                            h.tris.push_back({ip, u, x});
                            t2 = h.tris.size();
                            break;
                        }
                }
            }
            inserted++;
            break;
        }
    }
    return inserted;
}

// Source polygons touching each point, keyed by exact position (hull vertices are
// copies of the input points), and each polygon's points.
using PointKey = std::array<double, 3>;
struct PointPolygons {
    std::map<PointKey, std::set<int>> byPoint;
    std::map<int, std::set<PointKey>> points;
    bool empty() const { return byPoint.empty(); }
};

void AddPolygonPoint(PointPolygons& map, const Vec3& p, int polygon) {
    if (polygon < 0) return;
    PointKey k{static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)};
    map.byPoint[k].insert(polygon);
    map.points[polygon].insert(k);
}

// Tags each hull triangle with the source polygon all three of its corners belong to,
// so the map-collision face keeps the modelled polygon (retail faces follow the
// source mesh's quads and n-gons, not a planarity test).
// When the source polygons whose points are all hull vertices cover the hull's surface,
// the map-collision faces are built from those polygons directly: retail faces are the
// modelled polygons, even where neighbours meet at a slight crease the hull's own
// triangulation cuts differently. The hull triangles stay as they are for the PHY.
void BuildPolygonBrush(Hull& h, const PointPolygons& index) {
    auto vertexOf = [&](const PointKey& k) {
        DVec3 p(k[0], k[1], k[2]);
        int best = -1;
        double bestDist = 1e-3 * std::max(1.0, DLen(p) * 1e-3);
        for (size_t i = 0; i < h.verts.size(); i++) {
            double dist = DLen(h.verts[i] - p);
            if (dist < bestDist) {
                bestDist = dist;
                best = static_cast<int>(i);
            }
        }
        return best;
    };
    DVec3 centre;
    for (auto& v : h.verts) centre = centre + v;
    centre = centre * (1.0 / h.verts.size());
    double hullArea = 0;
    for (auto& t : h.tris) hullArea += DLen(DCross(h.verts[t[1]] - h.verts[t[0]], h.verts[t[2]] - h.verts[t[0]])) / 2;
    std::set<int> candidates;
    for (auto& v : h.verts) {
        auto it = index.byPoint.find({v.x, v.y, v.z});
        if (it != index.byPoint.end()) candidates.insert(it->second.begin(), it->second.end());
    }
    std::vector<std::vector<int>> loops;
    double area = 0;
    for (int p : candidates) {
        std::vector<int> loop;
        for (auto& k : index.points.at(p)) {
            int v = vertexOf(k);
            if (v < 0) {
                loop.clear();
                break;
            }
            loop.push_back(v);
        }
        if (loop.size() < 3) continue;
        DVec3 c;
        for (int v : loop) c = c + h.verts[v];
        c = c * (1.0 / loop.size());
        DVec3 out = c - centre;  // a convex hull's face looks away from its centre
        double ol = DLen(out);
        if (ol <= 0) continue;
        out = out * (1 / ol);
        DVec3 u = DCross(out, std::fabs(out.x) < 0.9 ? DVec3(1, 0, 0) : DVec3(0, 1, 0));
        DVec3 w = DCross(out, u);
        std::sort(loop.begin(), loop.end(), [&](int a, int b) {
            DVec3 da = h.verts[a] - c, db = h.verts[b] - c;
            return std::atan2(DDot(da, w), DDot(da, u)) < std::atan2(DDot(db, w), DDot(db, u));
        });
        for (size_t k = 1; k + 1 < loop.size(); k++)
            area += DLen(DCross(h.verts[loop[k]] - h.verts[loop[0]], h.verts[loop[k + 1]] - h.verts[loop[0]])) / 2;
        loops.push_back(loop);
    }
    if (loops.empty() || std::fabs(area - hullArea) > 1e-3 * std::max(1.0, hullArea)) {
        Verbose("collision piece: source polygons cover %.4f of %.4f surface; faces follow the hull", area, hullArea);
        return;
    }
    // T-junctions: a polygon corner lying partway along a neighbour's edge splits that
    // edge, so the loops meet edge to edge
    std::set<int> corners;
    for (auto& loop : loops) corners.insert(loop.begin(), loop.end());
    const double tEps = 1e-3 * std::max(1.0, std::sqrt(hullArea) * 1e-3);
    for (auto& loop : loops)
        for (size_t k = 0; k < loop.size(); k++) {
            const DVec3 &a = h.verts[loop[k]], &b = h.verts[loop[(k + 1) % loop.size()]];
            DVec3 ab = b - a;
            double len = DLen(ab);
            if (len <= 0) continue;
            int split = -1;
            double nearest = len;
            for (int v : corners) {
                if (v == loop[k] || v == loop[(k + 1) % loop.size()]) continue;
                double t = DDot(h.verts[v] - a, ab) / (len * len);
                if (t <= 0 || t >= 1 || DLen(DCross(ab, h.verts[v] - a)) / len > tEps) continue;
                if (t * len < nearest) {
                    nearest = t * len;
                    split = v;
                }
            }
            if (split >= 0) loop.insert(loop.begin() + static_cast<long>(k) + 1, split);  // re-checks the shorter edge next
        }
    // every loop edge must now meet the opposite edge of a neighbouring polygon
    std::set<std::pair<int, int>> directed;
    for (auto& loop : loops)
        for (size_t k = 0; k < loop.size(); k++) directed.insert({loop[k], loop[(k + 1) % loop.size()]});
    int open = 0;
    for (auto& e : directed)
        if (!directed.count({e.second, e.first})) open++;
    if (open) Verbose("collision piece: %d polygon edges have no opposite edge (a face that touches itself)", open);
    h.brushLoops = std::move(loops);
}

void TagPolygons(Hull& h, const PointPolygons& index) {
    if (index.empty()) return;
    BuildPolygonBrush(h, index);
    const auto& map = index.byPoint;
    // hull welding can move a vertex slightly off its input point
    auto lookup = [&](const DVec3& v) -> const std::set<int>* {
        auto it = map.find({v.x, v.y, v.z});
        if (it != map.end()) return &it->second;
        const std::set<int>* best = nullptr;
        double bestDist = 1e-3 * std::max(1.0, DLen(v) * 1e-3);
        for (auto& [k, polys] : map) {
            double dist = DLen(DVec3(k[0], k[1], k[2]) - v);
            if (dist < bestDist) {
                bestDist = dist;
                best = &polys;
            }
        }
        return best;
    };
    h.triPolygon.assign(h.tris.size(), -1);
    // Pieces that touch flush share positions, so a neighbouring piece's coplanar
    // polygon can also contain a triangle's corners; prefer the polygon whose points
    // are mostly this hull's own vertices.
    std::set<PointKey> hullPoints;
    for (auto& v : h.verts) hullPoints.insert({v.x, v.y, v.z});
    std::map<int, double> ownShare;
    auto ownPolygon = [&](const std::set<int>& candidates) {
        int best = -1;
        double bestShare = -1;
        for (int p : candidates) {
            auto it = ownShare.find(p);
            if (it == ownShare.end()) {
                const auto& pts = index.points.at(p);
                int inside = 0;
                for (auto& k : pts) inside += hullPoints.count(k) ? 1 : 0;
                it = ownShare.emplace(p, static_cast<double>(inside) / pts.size()).first;
            }
            if (it->second > bestShare) {
                bestShare = it->second;
                best = p;
            }
        }
        return best;
    };
    for (size_t t = 0; t < h.tris.size(); t++) {
        std::set<int> common;
        for (int c = 0; c < 3; c++) {
            const std::set<int>* polys = lookup(h.verts[h.tris[t][c]]);
            if (!polys) {
                common.clear();
                break;
            }
            if (c == 0) {
                common = *polys;
            } else {
                std::set<int> keep;
                for (int p : common)
                    if (polys->count(p)) keep.insert(p);
                common.swap(keep);
            }
            if (common.empty()) break;
        }
        if (!common.empty()) h.triPolygon[t] = ownPolygon(common);
    }
    // A slightly bent polygon can make the hull pick a vertex outside it; such a
    // triangle joins the polygon it shares an edge with when it lies nearly in that
    // polygon's plane (a 13-sided cap stays one face instead of 12-gon + triangle).
    auto triNormal = [&](size_t t) {
        const DVec3 &a = h.verts[h.tris[t][0]], &b = h.verts[h.tris[t][1]], &c = h.verts[h.tris[t][2]];
        return DCross(b - a, c - a);
    };
    std::map<int, DVec3> polygonNormal;
    for (size_t t = 0; t < h.tris.size(); t++)
        if (h.triPolygon[t] >= 0) polygonNormal[h.triPolygon[t]] = polygonNormal[h.triPolygon[t]] + triNormal(t);
    for (size_t t = 0; t < h.tris.size(); t++) {
        if (h.triPolygon[t] >= 0) continue;
        std::map<int, int> shared;
        for (int c = 0; c < 3; c++) {
            if (const std::set<int>* polys = lookup(h.verts[h.tris[t][c]]))
                for (int p : *polys) shared[p]++;
        }
        DVec3 n = triNormal(t);
        double ln = DLen(n), best = 0.995;
        for (auto& [p, count] : shared) {
            auto pn = polygonNormal.find(p);
            if (count < 2 || pn == polygonNormal.end() || ln <= 0 || DLen(pn->second) <= 0) continue;
            double dot = DDot(n, pn->second) / (ln * DLen(pn->second));
            if (dot > best) {
                best = dot;
                h.triPolygon[t] = p;
            }
        }
    }
    int untagged = static_cast<int>(std::count(h.triPolygon.begin(), h.triPolygon.end(), -1));
    if (untagged)
        Verbose("collision piece: %d of %d hull triangles match no source polygon", untagged, static_cast<int>(h.tris.size()));
}

}  // namespace

void BuildCollisionModel(BuildContext& ctx) {
    QcCollision& qc = ctx.qc.collision;
    if (!qc.present) {
        // retail models without physics point every bone at entry 0 with no pieces
        for (auto& b : ctx.out.bones) b.collisionIndex = 0, b.collisionCount = 0;
        return;
    }
    SrcInst* inst = ctx.Instance(qc.source, qc.sourceScale, false, false, qc.file);
    if (inst->gverts.empty() || inst->faces.empty()) Fatal("collision model %s has no triangles", qc.file.c_str());
    const auto& bones = ctx.bones;
    std::vector<Solid> solids;

    auto jointSurfaceProp = [&](int bone) {
        for (int k = bone; k != -1; k = bones[k].parent)
            for (auto& sp : ctx.qc.jointSurfaceProps)
                if (IEquals(sp.first, bones[k].name)) return sp.second;
        return ctx.qc.surfaceProp;
    };

    if (!qc.joints) {
        // single rigid body in entity space, posed by animation 0 frame 0 (studiomdl ConvertToWorldSpace)
        std::vector<Mat3x4> b2w;
        if (!ctx.anims.empty())
            ctx.CalcBoneTransforms(ctx.anims[0], 0, b2w);
        else {
            b2w.resize(bones.size());
            for (size_t k = 0; k < bones.size(); k++) b2w[k] = bones[k].boneToPose;
        }
        std::vector<Vec3> world(inst->gverts.size());
        std::vector<Vec3> normals;
        for (auto& v : inst->gverts) normals.push_back(v.normal);
        for (size_t i = 0; i < inst->gverts.size(); i++) {
            Vec3 p;
            for (auto& w : inst->gverts[i].weights)
                p += Transform(Concat(b2w[w.bone], Invert(bones[w.bone].boneToPose)), inst->gverts[i].pos) * w.weight;
            world[i] = p;
        }
        std::vector<std::array<int, 3>> faces;
        for (auto& f : inst->faces) faces.push_back({f.v[0], f.v[1], f.v[2]});
        PointPolygons polygons;
        for (auto& f : inst->faces)
            for (int c = 0; c < 3; c++) AddPolygonPoint(polygons, world[f.v[c]], f.polygon);
        std::vector<PointPolygons> pieceIndex;  // per piece: its own source polygons
        Solid s;
        s.name = FileBase(qc.file);
        s.bone = 0;
        s.clientData = 0;
        s.surfaceProp = ctx.qc.surfaceProp;
        if (qc.concave) {
            auto comps = ConnectedComponents(world, normals, faces, qc.weldNormal);
            for (auto& c : comps) {
                if (c.size() < 3) continue;
                std::vector<Vec3> pts;
                for (int i : c) pts.push_back(world[i]);
                Hull h;
                if (!MakeHull(pts, h)) {
                    Warn("collision piece with %d vertices is flat; skipped (check the collision mesh)", static_cast<int>(c.size()));
                    continue;
                }
                InsertSurfacePoints(h, pts);
                s.pieces.push_back(h);
                // the piece's own polygons: faces whose corners are all in it (pieces that
                // touch flush share positions, so a per-solid lookup would mix them)
                std::set<int> members(c.begin(), c.end());
                PointPolygons own;
                for (auto& f : inst->faces)
                    if (members.count(f.v[0]) && members.count(f.v[1]) && members.count(f.v[2]))
                        for (int k = 0; k < 3; k++) AddPolygonPoint(own, world[f.v[k]], f.polygon);
                pieceIndex.push_back(own);
            }
            if (static_cast<int>(s.pieces.size()) > qc.maxConvexPieces) {
                Warn("collision model has %d convex pieces (limit %d): using a single convex hull", static_cast<int>(s.pieces.size()),
                     qc.maxConvexPieces);
                s.pieces.clear();
                pieceIndex.clear();
            }
        }
        if (s.pieces.empty()) {
            Hull h;
            if (!MakeHull(world, h)) Fatal("collision model %s is flat or degenerate", qc.file.c_str());
            InsertSurfacePoints(h, world);
            s.pieces.push_back(h);
            pieceIndex = {polygons};
        } else {
            // loose source points (no face uses them) belong to the piece they lie on
            std::vector<bool> referenced(world.size(), false);
            for (auto& f : inst->faces)
                for (int c = 0; c < 3; c++) referenced[f.v[c]] = true;
            for (size_t i = 0; i < world.size(); i++)
                if (!referenced[i])
                    for (auto& h : s.pieces)
                        if (InsertSurfacePoints(h, {world[i]})) break;
        }
        for (size_t i = 0; i < s.pieces.size(); i++) TagPolygons(s.pieces[i], pieceIndex[i]);
        solids.push_back(s);
    } else {
        // one solid per bone, vertices in bone space
        std::map<int, int> boneMap;  // source-local bone -> source-local bone (merges)
        auto local = [&](const std::string& n) { return inst->src->FindBone(n); };
        for (size_t b = 0; b < inst->bones.size(); b++) boneMap[static_cast<int>(b)] = static_cast<int>(b);
        for (auto& m : qc.merges) {
            int p = local(m.first), c = local(m.second);
            if (p < 0 || c < 0) {
                Warn("$jointmerge %s %s: bone not found", m.first.c_str(), m.second.c_str());
                continue;
            }
            for (auto& kv : boneMap)
                if (kv.second == c) kv.second = p;
        }
        std::set<int> skip;
        for (auto& sk : qc.skipBones) {
            int b = local(sk);
            if (b < 0)
                Warn("can't skip joint %s, not found", sk.c_str());
            else
                skip.insert(b);
        }
        // solids follow the collision source's own joint order, as in studiomdl
        for (size_t lbi = 0; lbi < inst->localToGlobal.size(); lbi++) {
            int lb = static_cast<int>(lbi);
            if (inst->localToGlobal[lb] < 0) continue;
            size_t gb = static_cast<size_t>(inst->localToGlobal[lb]);
            if (skip.count(lb) || boneMap[lb] != lb) continue;
            // faces with any vertex assigned to this bone (CopyFaceVertsByBone)
            std::set<int> verts;
            for (auto& f : inst->faces) {
                bool on = false;
                for (int c = 0; c < 3 && !on; c++)
                    for (auto& w : inst->verts[f.v[c]].weights)
                        if (boneMap[w.bone] == lb) on = true;
                if (on)
                    for (int c = 0; c < 3; c++) verts.insert(f.v[c]);
            }
            if (verts.empty()) continue;
            Mat3x4 inv = Invert(bones[gb].boneToPose);
            std::vector<Vec3> pts;
            for (int v : verts) pts.push_back(Transform(inv, inst->gverts[v].pos));
            PointPolygons polygons;
            for (auto& f : inst->faces)
                if (verts.count(f.v[0]) && verts.count(f.v[1]) && verts.count(f.v[2]))
                    for (int c = 0; c < 3; c++) AddPolygonPoint(polygons, Transform(inv, inst->gverts[f.v[c]].pos), f.polygon);
            std::vector<PointPolygons> pieceIndex;  // per piece: its own source polygons
            Solid s;
            s.name = bones[gb].name;
            s.bone = static_cast<int>(gb);
            s.clientData = static_cast<int>(gb) + 1;
            s.surfaceProp = jointSurfaceProp(static_cast<int>(gb));
            if (qc.concavePerJoint) {
                std::vector<int> vlist(verts.begin(), verts.end());
                std::vector<Vec3> normals;
                for (int v : vlist) normals.push_back(inst->gverts[v].normal);
                std::vector<std::array<int, 3>> fl;
                std::vector<int> flPolygon;
                std::map<int, int> idx;
                for (size_t i = 0; i < vlist.size(); i++) idx[vlist[i]] = static_cast<int>(i);
                for (auto& f : inst->faces)
                    if (idx.count(f.v[0]) && idx.count(f.v[1]) && idx.count(f.v[2])) {
                        fl.push_back({idx[f.v[0]], idx[f.v[1]], idx[f.v[2]]});
                        flPolygon.push_back(f.polygon);
                    }
                for (auto& comp : ConnectedComponents(pts, normals, fl, qc.weldNormal)) {
                    std::vector<Vec3> cp;
                    for (int i : comp) cp.push_back(pts[i]);
                    Hull h;
                    if (MakeHull(cp, h)) {
                        InsertSurfacePoints(h, cp);
                        s.pieces.push_back(h);
                        std::set<int> members(comp.begin(), comp.end());
                        PointPolygons own;
                        for (size_t k = 0; k < fl.size(); k++)
                            if (members.count(fl[k][0]) && members.count(fl[k][1]) && members.count(fl[k][2]))
                                for (int c = 0; c < 3; c++) AddPolygonPoint(own, pts[fl[k][c]], flPolygon[k]);
                        pieceIndex.push_back(own);
                    }
                }
            }
            if (s.pieces.empty()) {
                Hull h;
                if (!MakeHull(pts, h)) {
                    Warn("joint %s has a flat collision mesh; skipped", bones[gb].name.c_str());
                    continue;
                }
                InsertSurfacePoints(h, pts);
                s.pieces.push_back(h);
                pieceIndex = {polygons};
            }
            // loose source points (no face uses them) weighted to this joint belong to
            // the piece they lie on
            std::vector<bool> referenced(inst->verts.size(), false);
            for (auto& f : inst->faces)
                for (int c = 0; c < 3; c++) referenced[f.v[c]] = true;
            for (size_t v = 0; v < inst->verts.size(); v++) {
                if (referenced[v] || inst->verts[v].weights.empty() || boneMap[inst->verts[v].weights[0].bone] != lb) continue;
                Vec3 lp = Transform(inv, inst->gverts[v].pos);
                for (auto& h : s.pieces)
                    if (InsertSurfacePoints(h, {lp})) break;
            }
            for (size_t i = 0; i < s.pieces.size(); i++) TagPolygons(s.pieces[i], pieceIndex[i]);
            // parent: nearest ancestor bone that has a solid (fixed up below)
            int p = bones[gb].parent;
            s.parent = p >= 0 ? bones[p].name : "";
            solids.push_back(s);
        }
        // FixParent: point at the nearest ancestor that owns a solid
        for (auto& s : solids) {
            int p = ctx.FindBone(s.parent);
            std::string found;
            while (p >= 0) {
                bool has = false;
                for (auto& o : solids)
                    if (o.bone == p) has = true;
                if (has) {
                    found = bones[p].name;
                    break;
                }
                p = bones[p].parent;
            }
            s.parent = found;
        }
        if (solids.empty()) Fatal("collision joints produced no solids");
    }

    // per-solid properties
    for (auto& s : solids) {
        s.damping = qc.damping;
        s.rotdamping = qc.rotdamping;
        s.inertia = qc.inertia;
        s.drag = qc.drag;
        for (auto& jp : qc.jointProps)
            if (IEquals(jp.name, s.name)) {
                if (jp.damping >= 0) s.damping = jp.damping;
                if (jp.rotdamping >= 0) s.rotdamping = jp.rotdamping;
                if (jp.inertia >= 0) s.inertia = jp.inertia;
                if (jp.massBias >= 0) s.massBias = jp.massBias;
            }
        for (auto& h : s.pieces) {
            MassProps m = ComputeMassProps(h);
            s.volume += m.volume;
            s.area += m.area;
        }
        // long thin objects get rotational damping (CreateCollide heuristic)
        DVec3 mn(1e30, 1e30, 1e30), mx(-1e30, -1e30, -1e30);
        for (auto& h : s.pieces)
            for (auto& v : h.verts) {
                mn = DVec3(std::min(mn.x, v.x), std::min(mn.y, v.y), std::min(mn.z, v.z));
                mx = DVec3(std::max(mx.x, v.x), std::max(mx.y, v.y), std::max(mx.z, v.z));
            }
        DVec3 size = mx - mn;
        int largest = size.x >= size.y && size.x >= size.z ? 0 : (size.y >= size.z ? 1 : 2);
        double sz[3] = {size.x, size.y, size.z};
        sz[largest] = 0;
        double len = std::sqrt(sz[0] * sz[0] + sz[1] * sz[1] + sz[2] * sz[2]);
        if (len > 0 && size[largest] / len > 9) s.rotdamping = 1.0f;
    }
    double totalVolume = 0;
    for (auto& s : solids) totalVolume += s.volume;
    if (totalVolume <= 0) totalVolume = 1;
    double totalMass = qc.mass;
    if (totalMass < 0 || qc.automass) {
        totalMass = 0;
        for (auto& s : solids) totalMass += s.volume * kCubicMetersPerCubicInch * DensityFor(s.surfaceProp);
        Info("Computed mass: %.2f kg", totalMass);
    }
    for (auto& s : solids) {
        s.mass = static_cast<float>(((s.volume * s.massBias) / totalVolume) * totalMass);
        if (s.mass < 1.0f) s.mass = 1.0f;
    }

    // physics bone indices
    for (size_t i = 0; i < bones.size(); i++) {
        int idx = -1;
        for (int k = static_cast<int>(i); k != -1 && idx < 0; k = bones[k].parent)
            for (size_t si = 0; si < solids.size(); si++)
                if (solids[si].bone == k && (qc.joints || k == 0)) {
                    idx = static_cast<int>(si);
                    break;
                }
        ctx.bones[i].physicsBone = idx < 0 ? 0 : idx;
        if (i < ctx.out.bones.size()) ctx.out.bones[i].physicsBone = ctx.bones[i].physicsBone;
    }

    // --- PHY
    Buffer phy;
    phy.I32(16);
    phy.I32(0);
    phy.I32(static_cast<int>(solids.size()));
    phy.I32(0);  // checksum, patched by the MDL writer
    for (auto& s : solids) {
        DVec3 drag;
        auto surface = BuildCompactSurface(s, drag);
        phy.I32(static_cast<int>(28 + surface.size()));
        phy.U32(0x59485056);  // "VPHY"
        phy.I16(0x100);
        phy.I16(0);
        phy.I32(static_cast<int>(surface.size()));
        phy.F32(static_cast<float>(drag.x));
        phy.F32(static_cast<float>(drag.y));
        phy.F32(static_cast<float>(drag.z));
        phy.I32(0);
        phy.Append(surface);
    }
    std::string text;
    for (size_t i = 0; i < solids.size(); i++) {
        auto& s = solids[i];
        text += "solid {\n";
        KeyInt(text, "index", static_cast<int>(i));
        KeyStr(text, "name", s.name);
        if (!s.parent.empty()) KeyStr(text, "parent", s.parent);
        KeyFloat(text, "mass", s.mass);
        KeyStr(text, "surfaceprop", s.surfaceProp);
        KeyFloat(text, "damping", s.damping);
        KeyFloat(text, "rotdamping", s.rotdamping);
        if (s.drag != -1) KeyFloat(text, "drag", s.drag);
        KeyFloat(text, "inertia", s.inertia);
        KeyFloat(text, "volume", static_cast<float>(s.volume));
        if (s.massBias != 1.0f) KeyFloat(text, "massbias", s.massBias);
        text += "}\n";
    }
    auto solidIndex = [&](const std::string& n) -> int {
        for (size_t i = 0; i < solids.size(); i++)
            if (IEquals(solids[i].name, n)) return static_cast<int>(i);
        return -1;
    };
    for (size_t i = 0; i < solids.size(); i++) {
        auto& s = solids[i];
        if (s.parent.empty()) continue;
        int parent = solidIndex(s.parent);
        if (parent < 0 || parent == static_cast<int>(i)) continue;
        float mn[3] = {0, 0, 0}, mx[3] = {0, 0, 0}, fr[3] = {0, 0, 0};
        for (auto& c : qc.constraints) {
            if (!IEquals(c.joint, s.name)) continue;
            if (c.type == 2) {
                mn[c.axis] = c.min;
                mx[c.axis] = c.max;
                fr[c.axis] = c.friction;
            } else if (c.type == 1) {
                mn[c.axis] = mx[c.axis] = fr[c.axis] = 0;
            } else {
                mn[c.axis] = -360;
                mx[c.axis] = 360;
                fr[c.axis] = c.friction;
            }
        }
        text += "ragdollconstraint {\n";
        KeyInt(text, "parent", parent);
        KeyInt(text, "child", static_cast<int>(i));
        const char* ax[3] = {"x", "y", "z"};
        for (int k = 0; k < 3; k++) {
            KeyFloat(text, (std::string(ax[k]) + "min").c_str(), mn[k]);
            KeyFloat(text, (std::string(ax[k]) + "max").c_str(), mx[k]);
            KeyFloat(text, (std::string(ax[k]) + "friction").c_str(), fr[k]);
        }
        text += "}\n";
    }
    if (qc.noSelfCollisions) {
        text += "collisionrules {\n";
        KeyInt(text, "selfcollisions", 0);
        text += "}\n";
    } else if (!qc.collisionPairs.empty()) {
        text += "collisionrules {\n";
        for (auto& p : qc.collisionPairs) {
            int a = solidIndex(p.first), b = solidIndex(p.second);
            if (a >= 0 && b >= 0 && a != b)
                text += Format("\"collisionpair\" \"%d,%d\"\n", a, b);
            else
                Warn("invalid collision pair (%s, %s)", p.first.c_str(), p.second.c_str());
        }
        text += "}\n";
    }
    if (qc.animatedFriction) {
        text += "animatedfriction {\n";
        KeyFloat(text, "animfrictionmin", qc.afMin);
        KeyFloat(text, "animfrictionmax", qc.afMax);
        KeyFloat(text, "animfrictiontimein", qc.afIn);
        KeyFloat(text, "animfrictiontimeout", qc.afOut);
        KeyFloat(text, "animfrictiontimehold", qc.afHold);
        text += "}\n";
    }
    text += "editparams {\n";
    KeyStr(text, "rootname", qc.rootBone);
    KeyFloat(text, "totalmass", static_cast<float>(totalMass));
    if (qc.concave) KeyInt(text, "concave", 1);
    for (auto& m : qc.merges) KeyStr(text, "jointmerge", m.first + "," + m.second);
    text += "}\n";
    text += qc.text;
    phy.Bytes(text.data(), text.size());
    phy.U8(0);
    ctx.out.physics.phy = phy.Data();
    ctx.out.physics.mass = static_cast<float>(totalMass);
    ctx.out.physics.solidCount = static_cast<int>(solids.size());
    Info("collision: %d solid(s), %d piece(s), %.2f kg", static_cast<int>(solids.size()),
         static_cast<int>(std::accumulate(solids.begin(), solids.end(), size_t(0),
                                          [](size_t a, const Solid& s) { return a + s.pieces.size(); })),
         totalMass);

    // --- map collision block: one brush per convex piece, in solid space. Rigid models
    // count the entries in the header; jointed models reference them from each bone.
    {
        Buffer hdrs;
        std::vector<std::vector<uint8_t>> blobs;
        std::vector<std::array<int, 4>> counts;
        bool ok = !qc.noMapCollision;
        ctx.out.physics.noMapCollision = qc.noMapCollision;
        for (auto& s : solids)
            for (auto& h : s.pieces) {
                int faces, bevels, edges;
                auto blob = BuildBrushPiece(h, faces, bevels, edges);
                if (static_cast<int>(h.verts.size()) > kMaxBrushIndex || faces + bevels > kMaxBrushIndex) {
                    Verbose("collision piece with %d planes and %d vertices: map collision block omitted (limit %d)",
                            faces + bevels, static_cast<int>(h.verts.size()), kMaxBrushIndex);
                    ok = false;
                }
                blobs.push_back(std::move(blob));
                counts.push_back({faces, faces + bevels, edges, static_cast<int>(h.verts.size())});
            }
        if (!ok && !qc.joints && !ctx.out.bones.empty()) {
            // retail rigid models without a block point bone 0 at entry 0 with no pieces
            ctx.out.bones[0].collisionIndex = 0;
            ctx.out.bones[0].collisionCount = 0;
        }
        if (ok && !blobs.empty()) {
            size_t n = blobs.size();
            size_t at = 20 * n;
            for (size_t i = 0; i < n; i++) {
                for (int k = 0; k < 4; k++) hdrs.I32(counts[i][k]);
                hdrs.I32(static_cast<int>(at - i * 20));
                at += blobs[i].size();
            }
            for (auto& bl : blobs) hdrs.Append(bl);
            ctx.out.physics.collisionBlock = hdrs.Data();
            if (!qc.joints) {
                ctx.out.physics.staticCollisionCount = static_cast<int>(n);
            } else {
                int index = 0;
                for (auto& s : solids) {
                    if (s.bone >= 0 && s.bone < static_cast<int>(ctx.out.bones.size())) {
                        ctx.out.bones[s.bone].collisionIndex = index;
                        ctx.out.bones[s.bone].collisionCount = static_cast<int>(s.pieces.size());
                    }
                    index += static_cast<int>(s.pieces.size());
                }
            }
        }
    }

    // expand the hull bounds by the collision AABB (CollisionModel_ExpandBBox). The solid
    // was posed by animation 0 frame 0; retail measures it unposed (bind pose), which
    // differs only when that frame moves the root bone.
    if (!qc.joints && !ctx.out.sequences.empty() && !ctx.out.sequences[0].explicitBox) {
        Mat3x4 unpose;
        if (!ctx.anims.empty()) {
            std::vector<Mat3x4> b2w;
            ctx.CalcBoneTransforms(ctx.anims[0], 0, b2w);
            unpose = Invert(Concat(b2w[0], Invert(bones[0].boneToPose)));
        }
        Vec3 mn(1e30f, 1e30f, 1e30f), mx(-1e30f, -1e30f, -1e30f);
        for (auto& h : solids[0].pieces)
            for (auto& v : h.verts) {
                Vec3 p = Transform(unpose, v.F());
                mn = Min(mn, p);
                mx = Max(mx, p);
            }
        mn -= Vec3(0.25f, 0.25f, 0.25f);
        mx += Vec3(0.25f, 0.25f, 0.25f);
        auto& s0 = ctx.out.sequences[0];
        s0.bmin = Min(s0.bmin, mn);
        s0.bmax = Max(s0.bmax, mx);
    }
}

}  // namespace r2c
