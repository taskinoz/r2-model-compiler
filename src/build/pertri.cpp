// Per-triangle AABB tree (studiohdr m_nPerTriAABB*), written for static props.
//
// Layout (version 2), all coordinates quantised to uint16 within the header box:
//   nodes  (20 bytes): bbmin[3], bbmax[3], child[2] (>= 0 node index, < 0 ~leaf index)
//   leaves (0x70)    : bbmin[3], bbmax[3], int vertexIndex, 12 x { uint16 v[3]; uint16 surfaceprop }
//   verts  (6 bytes) : uint16 x, y, z
// The fourth triangle value is the surfaceproperties.txt index (22 = metal in retail).
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <map>

#include "build/context.h"
#include "build/surfaceprops.h"
#include "core/buffer.h"
#include "core/log.h"

namespace r2c {

namespace {

constexpr int kTrisPerLeaf = 12;

struct PTri {
    uint32_t v[3];
    Vec3 centroid;
    Vec3 mn, mx;
    int surfaceProp = -1;
};

}  // namespace

void BuildPerTriAABB(BuildContext& ctx) {
    CPerTri& pt = ctx.out.perTri;
    pt = CPerTri();
    if (ctx.qc.perTriMode == 0) return;
    // static props, and props that may become static (prop_data "allowstatic" "1"),
    // get the tree; everything else is version 0, header only
    const std::string& kv = ctx.out.keyValues;
    size_t at = kv.find("\"allowstatic\"");
    if (at != std::string::npos) at = kv.find_first_not_of(' ', at + 13);
    bool allowStatic = at != std::string::npos && kv.compare(at, 3, "\"1\"") == 0;
    if (ctx.qc.perTriMode != 2 && !ctx.qc.staticProp && !allowStatic) return;

    // LOD 0 triangles of the default model of every body part. Like retail, each mesh
    // vertex is its own point (vertices that only share a position are not merged).
    std::vector<Vec3> verts;
    std::map<std::pair<const CMesh*, uint32_t>, uint32_t> vmap;
    std::vector<PTri> tris;
    for (auto& bp : ctx.out.bodyParts) {
        if (bp.models.empty()) continue;
        const CModel& md = bp.models[0];
        for (auto& me : md.meshes) {
            if (me.lods.empty()) continue;
            auto& idx = me.lods[0].indices;
            for (size_t t = 0; t + 2 < idx.size(); t += 3) {
                PTri tri;
                for (int c = 0; c < 3; c++) {
                    const Vec3& p = me.verts[idx[t + c]].pos;
                    auto key = std::make_pair(&me, idx[t + c]);
                    auto it = vmap.find(key);
                    if (it == vmap.end()) {
                        uint32_t n = static_cast<uint32_t>(verts.size());
                        vmap[key] = n;
                        verts.push_back(p);
                        tri.v[c] = n;
                    } else
                        tri.v[c] = it->second;
                }
                tris.push_back(tri);
            }
        }
    }
    const auto& source = ctx.qc.perTriMesh;
    if (source.present) {
        verts.clear(); tris.clear();
        for (auto& v : source.vertices) verts.emplace_back(v[0], v[1], v[2]);
        for (auto& t : source.triangles) {
            PTri tri;
            for (int c = 0; c < 3; c++) tri.v[c] = t[c];
            tri.surfaceProp = static_cast<int>(t[3]);
            tris.push_back(tri);
        }
    }
    pt.version = 2;
    if (source.present) { pt.bmin = source.bmin; pt.bmax = source.bmax; }
    if (tris.empty()) return;
    // retail starts the maximum at FLT_MIN (smallest positive float), so a maximum
    // below zero is stored as FLT_MIN
    const float tiny = std::numeric_limits<float>::min();
    Vec3 mn = verts[0], mx(tiny, tiny, tiny);
    for (auto& v : verts) {
        mn = Min(mn, v);
        mx = Max(mx, v);
    }
    if (source.present) { mn = Vec3(); mx = Vec3(65535,65535,65535); }
    else { pt.bmin = mn; pt.bmax = mx; }
    auto quant = [&](float v, int k, bool roundUp) -> uint16_t {
        float range = mx[k] - mn[k];
        if (range <= 0) return 0;
        float f = (v - mn[k]) / range * 65535.f;
        f = roundUp ? std::ceil(f) : std::floor(f);
        return static_cast<uint16_t>(std::clamp(f, 0.f, 65535.f));
    };
    auto quantNearest = [&](float v, int k) -> uint16_t {
        float range = mx[k] - mn[k];
        if (range <= 0) return 0;
        float f = std::round((v - mn[k]) / range * 65535.f);
        return static_cast<uint16_t>(std::clamp(f, 0.f, 65535.f));
    };
    for (auto& t : tris) {
        t.mn = t.mx = verts[t.v[0]];
        for (int c = 1; c < 3; c++) {
            t.mn = Min(t.mn, verts[t.v[c]]);
            t.mx = Max(t.mx, verts[t.v[c]]);
        }
        t.centroid = (verts[t.v[0]] + verts[t.v[1]] + verts[t.v[2]]) / 3.f;
    }
    int spIndex = SurfacePropIndex(ctx.qc.surfaceProp);
    if (spIndex < 0) spIndex = 0;

    struct Node {
        Vec3 mn, mx;
        int child[2] = {0, 0};
    };
    struct Leaf {
        Vec3 mn, mx;
        std::vector<int> tris;
    };
    std::vector<Node> nodes;
    std::vector<Leaf> leaves;
    // returns child link: node index or ~leaf
    std::function<int(std::vector<int>)> build = [&](std::vector<int> ts) -> int {
        Vec3 bmn = tris[ts[0]].mn, bmx = tris[ts[0]].mx;
        for (int t : ts) {
            bmn = Min(bmn, tris[t].mn);
            bmx = Max(bmx, tris[t].mx);
        }
        if (ts.size() <= static_cast<size_t>(kTrisPerLeaf)) {
            Leaf l;
            l.mn = bmn;
            l.mx = bmx;
            l.tris = ts;
            leaves.push_back(l);
            return ~static_cast<int>(leaves.size() - 1);
        }
        int id = static_cast<int>(nodes.size());
        nodes.push_back(Node());
        nodes[id].mn = bmn;
        nodes[id].mx = bmx;
        Vec3 cmn = tris[ts[0]].centroid, cmx = cmn;
        for (int t : ts) {
            cmn = Min(cmn, tris[t].centroid);
            cmx = Max(cmx, tris[t].centroid);
        }
        Vec3 ext = cmx - cmn;
        int axis = ext.x >= ext.y && ext.x >= ext.z ? 0 : (ext.y >= ext.z ? 1 : 2);
        std::stable_sort(ts.begin(), ts.end(), [&](int a, int b) { return tris[a].centroid[axis] < tris[b].centroid[axis]; });
        size_t half = ts.size() / 2;
        int l = build(std::vector<int>(ts.begin(), ts.begin() + half));
        int r = build(std::vector<int>(ts.begin() + half, ts.end()));
        nodes[id].child[0] = l;
        nodes[id].child[1] = r;
        return id;
    };
    std::vector<int> all(tris.size());
    for (size_t i = 0; i < all.size(); i++) all[i] = static_cast<int>(i);
    build(all);

    // Number the vertices in leaf order, so each leaf's triangles index a narrow range
    // (leaves store 16-bit offsets from a base). A leaf whose shared vertices still span
    // more than that gets its own copies.
    {
        std::vector<uint32_t> remap(verts.size(), UINT32_MAX);
        std::vector<Vec3> ordered;
        for (auto& l : leaves)
            for (int t : l.tris)
                for (int c = 0; c < 3; c++) {
                    uint32_t& r = remap[tris[t].v[c]];
                    if (r == UINT32_MAX) {
                        r = static_cast<uint32_t>(ordered.size());
                        ordered.push_back(verts[tris[t].v[c]]);
                    }
                }
        if (source.present)
            for (size_t i = 0; i < verts.size(); i++)
                if (remap[i] == UINT32_MAX) {
                    remap[i] = static_cast<uint32_t>(ordered.size());
                    ordered.push_back(verts[i]);
                }
        for (auto& t : tris)
            for (int c = 0; c < 3; c++) t.v[c] = remap[t.v[c]];
        verts = ordered;
        if (verts.size() > 65535)
            for (auto& l : leaves) {
                uint32_t lo = UINT32_MAX, hi = 0;
                for (int t : l.tris)
                    for (int c = 0; c < 3; c++) {
                        lo = std::min(lo, tris[t].v[c]);
                        hi = std::max(hi, tris[t].v[c]);
                    }
                if (l.tris.empty() || hi - lo <= 65535) continue;
                std::map<uint32_t, uint32_t> copy;
                for (int t : l.tris)
                    for (int c = 0; c < 3; c++) {
                        uint32_t v = tris[t].v[c];
                        auto it = copy.find(v);
                        if (it == copy.end()) {
                            it = copy.emplace(v, static_cast<uint32_t>(verts.size())).first;
                            Vec3 p = verts[v];
                            verts.push_back(p);
                        }
                        tris[t].v[c] = it->second;
                    }
            }
    }

    Buffer b;
    for (auto& n : nodes) {
        for (int k = 0; k < 3; k++) b.U16(quant(n.mn[k], k, false));
        for (int k = 0; k < 3; k++) b.U16(quant(n.mx[k], k, true));
        b.I32(n.child[0]);
        b.I32(n.child[1]);
    }
    for (auto& l : leaves) {
        for (int k = 0; k < 3; k++) b.U16(quant(l.mn[k], k, false));
        for (int k = 0; k < 3; k++) b.U16(quant(l.mx[k], k, true));
        uint32_t base = UINT32_MAX;
        for (int t : l.tris)
            for (int c = 0; c < 3; c++) base = std::min(base, tris[t].v[c]);
        if (base == UINT32_MAX || verts.size() <= 65535) base = 0;
        b.I32(static_cast<int32_t>(base));
        for (int i = 0; i < kTrisPerLeaf; i++) {
            if (i < static_cast<int>(l.tris.size())) {
                const PTri& t = tris[l.tris[i]];
                for (int c = 0; c < 3; c++) {
                    uint32_t rel = t.v[c] - base;
                    if (rel > 65535) Fatal("per-triangle collision: leaf vertex range too large");
                    b.U16(static_cast<uint16_t>(rel));
                }
                b.U16(static_cast<uint16_t>(t.surfaceProp >= 0 ? t.surfaceProp : spIndex));
            } else {
                b.Zero(8);
            }
        }
    }
    for (auto& v : verts)
        for (int k = 0; k < 3; k++) b.U16(quantNearest(v[k], k));
    pt.data = b.Data();
    pt.nodeCount = static_cast<int>(nodes.size());
    pt.leafCount = static_cast<int>(leaves.size());
    pt.vertCount = static_cast<int>(verts.size());
}

}  // namespace r2c
