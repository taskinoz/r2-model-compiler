// VVD (v4), VVC (v1) and VTX (OptimizedModel v7) writers.
#include <algorithm>
#include <map>

#include "core/buffer.h"
#include "core/log.h"
#include "write/writers.h"

namespace r2c {

namespace {

// Visits every mesh vertex in VVD order (bodypart, model, mesh).
template <typename F>
void ForEachVertex(const CompiledModel& m, F fn) {
    for (auto& bp : m.bodyParts)
        for (auto& md : bp.models)
            for (auto& me : md.meshes)
                for (size_t i = 0; i < me.verts.size(); i++) fn(md, me, i);
}

int TotalVerts(const CompiledModel& m) {
    int n = 0;
    for (auto& bp : m.bodyParts)
        for (auto& md : bp.models) n += md.NumVerts();
    return n;
}

struct Fixup {
    int lod, start, count;
};

std::vector<Fixup> BuildFixups(const CompiledModel& m) {
    std::vector<Fixup> f;
    int at = 0;
    for (auto& bp : m.bodyParts)
        for (auto& md : bp.models)
            for (auto& me : md.meshes) {
                size_t meshStart = f.size();  // runs never continue across meshes
                for (size_t i = 0; i < me.verts.size(); i++) {
                    int lod = me.vertMaxLod[i];
                    if (f.size() > meshStart && f.back().lod == lod)
                        f.back().count++;
                    else
                        f.push_back({lod, at, 1});
                    at++;
                }
            }
    return f;
}

// Per-LOD vertex counts. With a fixup table retail stores, for each LOD, the end of the
// last run that LOD uses rather than the number of vertices it uses.
void LodCounts(const CompiledModel& m, int out[kMaxLods]) {
    for (int l = 0; l < kMaxLods; l++) out[l] = 0;
    for (auto& bp : m.bodyParts)
        for (auto& md : bp.models)
            for (auto& me : md.meshes)
                for (int l = 0; l < kMaxLods; l++) out[l] += me.lodVertexCount[l];
}

void LodCounts(const std::vector<Fixup>& fixups, int out[kMaxLods]) {
    for (int l = 0; l < kMaxLods; l++) out[l] = 0;
    for (auto& f : fixups)
        for (int l = 0; l <= f.lod && l < kMaxLods; l++) out[l] = std::max(out[l], f.start + f.count);
    for (int l = 1; l < kMaxLods; l++)
        if (out[l] == 0) out[l] = out[l - 1];
}

}  // namespace

// Retail writes the fixup table for every model with LODs and more than one mesh, and
// never for a single mesh (whose coarse-first order needs no remapping)
static std::vector<Fixup> ModelFixups(const CompiledModel& m) {
    size_t numMeshes = 0;
    for (auto& bp : m.bodyParts)
        for (auto& md : bp.models) numMeshes += md.meshes.size();
    if (m.numLods > 1 && numMeshes > 1) return BuildFixups(m);
    return {};
}

static void HeaderLodCounts(const CompiledModel& m, const std::vector<Fixup>& fixups, int out[kMaxLods]) {
    if (fixups.empty())
        LodCounts(m, out);
    else
        LodCounts(fixups, out);
}

std::vector<uint8_t> WriteVVD(const CompiledModel& m, int checksum) {
    Buffer b;
    std::vector<Fixup> fixups = ModelFixups(m);
    int counts[kMaxLods];
    HeaderLodCounts(m, fixups, counts);
    b.U32(0x56534449);  // IDSV
    b.I32(4);
    b.I32(checksum);
    b.I32(m.numLods);
    for (int l = 0; l < kMaxLods; l++) b.I32(counts[l]);
    b.I32(static_cast<int>(fixups.size()));
    size_t fixupStartPos = b.Reserve(4), vertStartPos = b.Reserve(4), tanStartPos = b.Reserve(4);
    b.PatchI32(fixupStartPos, fixups.empty() ? 0 : static_cast<int>(b.Tell()));
    if (!fixups.empty()) b.PatchI32(fixupStartPos, static_cast<int>(b.Tell()));
    for (auto& f : fixups) {
        b.I32(f.lod);
        b.I32(f.start);
        b.I32(f.count);
    }
    b.Align(16);
    if (fixups.empty()) b.PatchI32(fixupStartPos, 64);
    b.PatchI32(vertStartPos, static_cast<int>(b.Tell()));
    ForEachVertex(m, [&](const CModel&, const CMesh& me, size_t i) {
        const CVertex& v = me.verts[i];
        for (int k = 0; k < 3; k++) b.F32(k < v.numBones ? v.weight[k] : 0.f);
        for (int k = 0; k < 3; k++) b.U8(k < v.numBones ? v.bone[k] : 0);
        b.U8(static_cast<uint8_t>(v.numBones));
        b.V3(v.pos);
        b.V3(v.normal);
        b.V2(v.uv);
    });
    b.PatchI32(tanStartPos, static_cast<int>(b.Tell()));
    ForEachVertex(m, [&](const CModel&, const CMesh& me, size_t i) {
        const Vec4& t = me.verts[i].tangent;
        b.F32(t.x);
        b.F32(t.y);
        b.F32(t.z);
        b.F32(t.w);
    });
    return b.Data();
}

std::vector<uint8_t> WriteVVC(const CompiledModel& m, int checksum) {
    Buffer b;
    int counts[kMaxLods];
    HeaderLodCounts(m, ModelFixups(m), counts);  // the same counts as the VVD
    b.U32(0x56434449);  // IDCV
    b.I32(1);
    b.I32(checksum);
    b.I32(m.numLods);
    for (int l = 0; l < kMaxLods; l++) b.I32(counts[l]);
    size_t colPos = b.Reserve(4), uvPos = b.Reserve(4);
    b.Align(16);
    b.PatchI32(colPos, static_cast<int>(b.Tell()));
    ForEachVertex(m, [&](const CModel&, const CMesh& me, size_t i) { b.Bytes(me.verts[i].color, 4); });
    b.Align(16);
    b.PatchI32(uvPos, static_cast<int>(b.Tell()));
    ForEachVertex(m, [&](const CModel&, const CMesh& me, size_t i) { b.V2(me.verts[i].uv2); });
    return b.Data();
}

// ---------------------------------------------------------------------------
// VTX
// ---------------------------------------------------------------------------

namespace {

constexpr int kMaxBonesPerStrip = 160;

struct VStrip {
    std::vector<uint16_t> indices;  // into strip group verts
    int vertOffset = 0, numVerts = 0;
    int indexOffset = 0;
    std::vector<std::pair<int, int>> boneStateChanges;  // hardware id -> bone
    int numBones = 0;
};

struct VVert {
    uint8_t weightIndex[3] = {0, 1, 2};
    uint8_t numBones = 0;
    uint16_t origMeshVertID = 0;
    uint8_t boneID[3] = {0, 0, 0};
};

struct VStripGroup {
    std::vector<VVert> verts;
    std::vector<uint16_t> indices;
    std::vector<VStrip> strips;
    uint8_t flags = 0x02;  // hardware skinned
};

// Forsyth-style vertex cache ordering of a triangle list (indices into the mesh).
std::vector<uint32_t> OptimizeTriangleOrder(const std::vector<uint32_t>& idx, size_t numVerts) {
    const int kCache = 32;
    size_t numTris = idx.size() / 3;
    if (numTris < 2) return idx;
    std::vector<int> valence(numVerts, 0);
    for (uint32_t v : idx) valence[v]++;
    std::vector<int> start(numVerts + 1, 0);
    for (size_t v = 0; v < numVerts; v++) start[v + 1] = start[v] + valence[v];
    std::vector<int> triList(idx.size());
    std::vector<int> fill(start.begin(), start.end() - 1);
    for (size_t t = 0; t < numTris; t++)
        for (int c = 0; c < 3; c++) triList[fill[idx[t * 3 + c]]++] = static_cast<int>(t);
    std::vector<int> remaining = valence;
    std::vector<int> cachePos(numVerts, -1);
    std::vector<float> score(numVerts, 0);
    std::vector<bool> emitted(numTris, false);
    auto vscore = [&](size_t v) -> float {
        if (remaining[v] == 0) return -1.f;
        float s = 0;
        int p = cachePos[v];
        if (p >= 0) {
            if (p < 3)
                s = 0.75f;
            else
                s = std::pow(1.0f - (p - 3) / static_cast<float>(kCache - 3), 1.5f);
        }
        s += 2.0f * std::pow(static_cast<float>(remaining[v]), -0.5f);
        return s;
    };
    for (size_t v = 0; v < numVerts; v++) score[v] = vscore(v);
    std::vector<float> triScore(numTris);
    for (size_t t = 0; t < numTris; t++) triScore[t] = score[idx[t * 3]] + score[idx[t * 3 + 1]] + score[idx[t * 3 + 2]];
    std::vector<int> cache;
    std::vector<uint32_t> out;
    out.reserve(idx.size());
    size_t emittedCount = 0, scan = 0;
    while (emittedCount < numTris) {
        int best = -1;
        float bestScore = -1e30f;
        for (int v : cache)
            for (int i = start[v]; i < start[v + 1]; i++) {
                int t = triList[i];
                if (!emitted[t] && triScore[t] > bestScore) {
                    bestScore = triScore[t];
                    best = t;
                }
            }
        if (best < 0) {
            while (scan < numTris && emitted[scan]) scan++;
            best = static_cast<int>(scan);
        }
        emitted[best] = true;
        emittedCount++;
        for (int c = 0; c < 3; c++) {
            uint32_t v = idx[best * 3 + c];
            out.push_back(v);
            remaining[v]--;
            auto it = std::find(cache.begin(), cache.end(), static_cast<int>(v));
            if (it != cache.end()) cache.erase(it);
            cache.insert(cache.begin(), static_cast<int>(v));
        }
        std::vector<int> evicted;
        if (static_cast<int>(cache.size()) > kCache) {
            evicted.assign(cache.begin() + kCache, cache.end());
            cache.resize(kCache);
        }
        for (int v : evicted) cachePos[v] = -1;
        for (size_t i = 0; i < cache.size(); i++) cachePos[cache[i]] = static_cast<int>(i);
        auto rescore = [&](int v) {
            score[v] = vscore(v);
            for (int i = start[v]; i < start[v + 1]; i++) {
                int t = triList[i];
                if (!emitted[t]) triScore[t] = score[idx[t * 3]] + score[idx[t * 3 + 1]] + score[idx[t * 3 + 2]];
            }
        };
        for (int v : cache) rescore(v);
        for (int v : evicted) rescore(v);
    }
    return out;
}

VStripGroup BuildStripGroup(const CMesh& mesh, const std::vector<uint32_t>& lodIdx, bool staticProp) {
    VStripGroup sg;
    std::vector<uint32_t> idx = OptimizeTriangleOrder(lodIdx, mesh.verts.size());
    std::map<uint32_t, uint16_t> remap;
    std::vector<int> palette;  // hardware bone -> global bone
    std::map<int, int> hw;
    VStrip strip;
    for (uint32_t mv : idx) {
        auto it = remap.find(mv);
        if (it == remap.end()) {
            const CVertex& v = mesh.verts[mv];
            VVert vv;
            vv.origMeshVertID = static_cast<uint16_t>(mv);
            vv.numBones = static_cast<uint8_t>(v.numBones);
            for (int b = 0; b < 3; b++) {
                if (b < v.numBones) {
                    int g = v.bone[b];
                    auto h = hw.find(g);
                    int id;
                    if (h == hw.end()) {
                        id = static_cast<int>(palette.size());
                        palette.push_back(g);
                        hw[g] = id;
                    } else
                        id = h->second;
                    vv.boneID[b] = static_cast<uint8_t>(id);
                } else
                    vv.boneID[b] = 0;
            }
            if (sg.verts.size() >= 65535) Fatal("strip group exceeds 65535 vertices");
            uint16_t ni = static_cast<uint16_t>(sg.verts.size());
            sg.verts.push_back(vv);
            remap[mv] = ni;
            sg.indices.push_back(ni);
        } else {
            sg.indices.push_back(it->second);
        }
    }
    if (palette.size() > kMaxBonesPerStrip)
        Fatal("mesh uses %d bones in one strip (max %d)", static_cast<int>(palette.size()), kMaxBonesPerStrip);
    strip.indexOffset = 0;
    strip.vertOffset = 0;
    strip.numVerts = static_cast<int>(sg.verts.size());
    strip.indices = sg.indices;
    // retail: a strip's bone count is the most weights any of its vertices uses; static
    // props have none and no bone state changes
    strip.numBones = 0;
    if (!staticProp) {
        for (auto& vv : sg.verts) strip.numBones = std::max<int>(strip.numBones, vv.numBones);
        for (size_t i = 0; i < palette.size(); i++) strip.boneStateChanges.push_back({static_cast<int>(i), palette[i]});
    }
    sg.strips.push_back(strip);
    return sg;
}

}  // namespace

std::vector<uint8_t> WriteVTX(const CompiledModel& m, int checksum) {
    Buffer b;
    // header
    b.I32(7);
    b.I32(24);
    b.U16(160);
    b.U16(9);
    b.I32(3);
    b.I32(checksum);
    b.I32(m.numLods);
    size_t matRepPos = b.Reserve(4);
    b.I32(static_cast<int>(m.bodyParts.size()));
    size_t bpOffPos = b.Reserve(4);

    // Build every strip group first.
    struct MeshOut {
        std::vector<VStripGroup> groups;
    };
    struct LodOut {
        std::vector<MeshOut> meshes;
        float switchPoint;
    };
    struct ModelOut {
        std::vector<LodOut> lods;
    };
    std::vector<std::vector<ModelOut>> models(m.bodyParts.size());
    for (size_t bi = 0; bi < m.bodyParts.size(); bi++)
        for (auto& md : m.bodyParts[bi].models) {
            ModelOut mo;
            mo.lods.resize(m.numLods);
            for (int l = 0; l < m.numLods; l++) {
                mo.lods[l].switchPoint = m.lodSwitch[l];
                for (auto& me : md.meshes) {
                    MeshOut meo;
                    if (l < static_cast<int>(me.lods.size()) && !me.lods[l].indices.empty())
                        meo.groups.push_back(BuildStripGroup(me, me.lods[l].indices, (m.flags & 0x10) != 0));  // STUDIOHDR_FLAGS_STATIC_PROP
                    mo.lods[l].meshes.push_back(std::move(meo));
                }
            }
            models[bi].push_back(std::move(mo));
        }

    // body parts
    b.PatchI32(bpOffPos, static_cast<int>(b.Tell()));
    size_t bpStart = b.Tell();
    for (size_t bi = 0; bi < m.bodyParts.size(); bi++) {
        b.I32(static_cast<int>(models[bi].size()));
        b.I32(0);
    }
    // models
    std::vector<size_t> modelStart(m.bodyParts.size());
    for (size_t bi = 0; bi < m.bodyParts.size(); bi++) {
        size_t bpPos = bpStart + bi * 8;
        modelStart[bi] = b.Tell();
        b.PatchI32(bpPos + 4, static_cast<int>(b.Tell() - bpPos));
        for (size_t mi = 0; mi < models[bi].size(); mi++) {
            b.I32(m.numLods);
            b.I32(0);
        }
    }
    // lods
    std::vector<std::vector<size_t>> lodStart(m.bodyParts.size());
    for (size_t bi = 0; bi < m.bodyParts.size(); bi++)
        for (size_t mi = 0; mi < models[bi].size(); mi++) {
            size_t modelPos = modelStart[bi] + mi * 8;
            lodStart[bi].push_back(b.Tell());
            b.PatchI32(modelPos + 4, static_cast<int>(b.Tell() - modelPos));
            for (auto& lo : models[bi][mi].lods) {
                b.I32(static_cast<int>(lo.meshes.size()));
                b.I32(0);
                b.F32(lo.switchPoint);
            }
        }
    // meshes
    struct MeshRef {
        size_t pos;
        const MeshOut* mo;
    };
    std::vector<MeshRef> meshRefs;
    for (size_t bi = 0; bi < m.bodyParts.size(); bi++)
        for (size_t mi = 0; mi < models[bi].size(); mi++)
            for (int l = 0; l < m.numLods; l++) {
                size_t lodPos = lodStart[bi][mi] + l * 12;
                auto& lo = models[bi][mi].lods[l];
                b.PatchI32(lodPos + 4, static_cast<int>(b.Tell() - lodPos));
                for (auto& meo : lo.meshes) {
                    meshRefs.push_back({b.Tell(), &meo});
                    b.I32(static_cast<int>(meo.groups.size()));
                    b.I32(0);
                    b.U8(0);
                }
            }
    // strip groups
    struct SGRef {
        size_t pos;
        const VStripGroup* sg;
    };
    std::vector<SGRef> sgRefs;
    for (auto& mr : meshRefs) {
        b.PatchI32(mr.pos + 4, static_cast<int>(b.Tell() - mr.pos));
        for (auto& sg : mr.mo->groups) {
            sgRefs.push_back({b.Tell(), &sg});
            b.I32(static_cast<int>(sg.verts.size()));
            b.I32(0);
            b.I32(static_cast<int>(sg.indices.size()));
            b.I32(0);
            b.I32(static_cast<int>(sg.strips.size()));
            b.I32(0);
            b.U8(sg.flags);
            b.I32(0);
            b.I32(0);
        }
    }
    // strips
    struct StripRef {
        size_t pos;
        const VStrip* st;
    };
    std::vector<StripRef> stripRefs;
    for (auto& sr : sgRefs) {
        b.PatchI32(sr.pos + 20, static_cast<int>(b.Tell() - sr.pos));
        for (auto& st : sr.sg->strips) {
            stripRefs.push_back({b.Tell(), &st});
            b.I32(static_cast<int>(st.indices.size()));
            b.I32(st.indexOffset);
            b.I32(st.numVerts);
            b.I32(st.vertOffset);
            b.I16(static_cast<int16_t>(st.numBones));
            b.U8(0x01);  // STRIP_IS_TRILIST
            b.I32(static_cast<int>(st.boneStateChanges.size()));
            b.I32(0);
            b.I32(0);
            b.I32(0);
        }
    }
    // verts
    for (auto& sr : sgRefs) {
        b.PatchI32(sr.pos + 4, static_cast<int>(b.Tell() - sr.pos));
        for (auto& v : sr.sg->verts) {
            b.Bytes(v.weightIndex, 3);
            b.U8(v.numBones);
            b.U16(v.origMeshVertID);
            b.Bytes(v.boneID, 3);
        }
    }
    // indices
    for (auto& sr : sgRefs) {
        b.PatchI32(sr.pos + 12, static_cast<int>(b.Tell() - sr.pos));
        for (auto i : sr.sg->indices) b.U16(i);
    }
    // bone state changes
    for (auto& st : stripRefs) {
        b.PatchI32(st.pos + 23, static_cast<int>(b.Tell() - st.pos));
        for (auto& bsc : st.st->boneStateChanges) {
            b.I32(bsc.first);
            b.I32(bsc.second);
        }
    }
    // material replacement lists, one per LOD
    b.PatchI32(matRepPos, static_cast<int>(b.Tell()));
    size_t listStart = b.Tell();
    for (int l = 0; l < m.numLods; l++) {
        b.I32(0);
        b.I32(0);
    }
    struct NameRef {
        size_t pos;
        size_t base;
        std::string name;
    };
    std::vector<NameRef> names;
    for (int l = 0; l < m.numLods; l++) {
        const auto* reps = l < static_cast<int>(m.lodMaterialReplacements.size()) ? &m.lodMaterialReplacements[l] : nullptr;
        size_t lp = listStart + l * 8;
        if (!reps || reps->empty()) continue;
        b.PatchI32(lp, static_cast<int>(reps->size()));
        b.PatchI32(lp + 4, static_cast<int>(b.Tell() - lp));
        for (auto& r : *reps) {
            size_t rp = b.Tell();
            b.I16(static_cast<int16_t>(r.first));
            b.I32(0);
            names.push_back({rp + 2, rp, r.second});
        }
    }
    for (auto& n : names) {
        b.PatchI32(n.pos, static_cast<int>(b.Tell() - n.base));
        b.CString(n.name);
    }
    return b.Data();
}

}  // namespace r2c
