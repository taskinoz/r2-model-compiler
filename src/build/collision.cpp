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
    struct Face {
        DVec3 n;
        double d;
        std::vector<int> tris;
    };
    std::vector<Face> fs;
    std::vector<int> triFace(h.tris.size());
    double scale = 0;
    for (auto& v : h.verts) scale = std::max(scale, DLen(v));
    double dEps = std::max(1e-3, scale * 1e-5);
    for (size_t t = 0; t < h.tris.size(); t++) {
        const DVec3 &a = h.verts[h.tris[t][0]], &b = h.verts[h.tris[t][1]], &c = h.verts[h.tris[t][2]];
        DVec3 n = DCross(b - a, c - a);
        double l = DLen(n);
        if (l > 0) n = n * (1 / l);
        double d = DDot(n, a);
        int f = -1;
        for (size_t k = 0; k < fs.size(); k++)
            if (DDot(fs[k].n, n) > 0.99999 && std::fabs(fs[k].d - d) < dEps) {
                f = static_cast<int>(k);
                break;
            }
        if (f < 0) {
            fs.push_back({n, d, {}});
            f = static_cast<int>(fs.size() - 1);
        }
        fs[f].tris.push_back(static_cast<int>(t));
        triFace[t] = f;
    }
    // directed edge -> face, boundary edges per face
    std::map<std::pair<int, int>, int> dirFace;
    for (size_t t = 0; t < h.tris.size(); t++)
        for (int e = 0; e < 3; e++) dirFace[{h.tris[t][e], h.tris[t][(e + 1) % 3]}] = triFace[t];
    struct Edge {
        int v0, v1, s0, s1;
    };
    std::vector<Edge> es;
    std::set<std::pair<int, int>> done;
    for (size_t f = 0; f < fs.size(); f++)
        for (int t : fs[f].tris)
            for (int e = 0; e < 3; e++) {
                int a = h.tris[t][e], b = h.tris[t][(e + 1) % 3];
                int other = dirFace[{b, a}];
                if (other == static_cast<int>(f)) continue;  // interior edge of a merged face
                auto key = std::make_pair(std::min(a, b), std::max(a, b));
                if (done.count(key)) continue;
                done.insert(key);
                es.push_back({a, b, static_cast<int>(f), other});
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
                s.pieces.push_back(h);
            }
            if (static_cast<int>(s.pieces.size()) > qc.maxConvexPieces) {
                Warn("collision model has %d convex pieces (limit %d): using a single convex hull", static_cast<int>(s.pieces.size()),
                     qc.maxConvexPieces);
                s.pieces.clear();
            }
        }
        if (s.pieces.empty()) {
            Hull h;
            if (!MakeHull(world, h)) Fatal("collision model %s is flat or degenerate", qc.file.c_str());
            s.pieces.push_back(h);
        }
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
        for (size_t gb = 0; gb < bones.size(); gb++) {
            int lb = inst->globalToLocal[gb];
            if (lb < 0 || skip.count(lb) || boneMap[lb] != lb) continue;
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
                std::map<int, int> idx;
                for (size_t i = 0; i < vlist.size(); i++) idx[vlist[i]] = static_cast<int>(i);
                for (auto& f : inst->faces)
                    if (idx.count(f.v[0]) && idx.count(f.v[1]) && idx.count(f.v[2])) fl.push_back({idx[f.v[0]], idx[f.v[1]], idx[f.v[2]]});
                for (auto& comp : ConnectedComponents(pts, normals, fl, qc.weldNormal)) {
                    std::vector<Vec3> cp;
                    for (int i : comp) cp.push_back(pts[i]);
                    Hull h;
                    if (MakeHull(cp, h)) s.pieces.push_back(h);
                }
            }
            if (s.pieces.empty()) {
                Hull h;
                if (!MakeHull(pts, h)) {
                    Warn("joint %s has a flat collision mesh; skipped", bones[gb].name.c_str());
                    continue;
                }
                s.pieces.push_back(h);
            }
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
        bool ok = true;
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
                if (bones.size() == 1) ctx.qc.flags |= STUDIOHDR_FLAGS_RESPAWN_800000;
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

    // expand the hull bounds by the collision AABB (CollisionModel_ExpandBBox)
    if (!qc.joints && !ctx.out.sequences.empty()) {
        Vec3 mn(1e30f, 1e30f, 1e30f), mx(-1e30f, -1e30f, -1e30f);
        for (auto& h : solids[0].pieces)
            for (auto& v : h.verts) {
                mn = Min(mn, v.F());
                mx = Max(mx, v.F());
            }
        mn -= Vec3(0.25f, 0.25f, 0.25f);
        mx += Vec3(0.25f, 0.25f, 0.25f);
        auto& s0 = ctx.out.sequences[0];
        s0.bmin = Min(s0.bmin, mn);
        s0.bmax = Max(s0.bmax, mx);
    }
}

}  // namespace r2c
