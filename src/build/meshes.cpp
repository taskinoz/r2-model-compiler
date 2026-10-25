// Per-model meshes with all LODs unified into one vertex pool per mesh.
//
// As in studiomdl's UnifyLODs, a mesh exists for each material used by LOD 0;
// lower LODs reuse LOD 0 vertices where they match exactly and append new ones
// otherwise. Each vertex records the coarsest LOD that uses it, and meshes are
// sorted coarse-first so the VVD can drop LOD 0-only vertices when a higher root
// LOD is chosen (the layout retail Titanfall 2 models use).
#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>

#include "build/context.h"
#include "core/log.h"
#include "core/strings.h"

namespace r2c {

namespace {

struct VertKey {
    Vec3 pos, normal;
    Vec2 uv;
    int nb;
    int bone[3];
    float w[3];
    Vec2 uv2;
    uint8_t col[4];
};

bool SameVertex(const CVertex& a, const CVertex& b) {
    if (!(a.pos == b.pos) || !(a.uv == b.uv) || !(a.uv2 == b.uv2)) return false;
    if (Dot(a.normal, b.normal) < 0.99939083f) return false;  // cos(2 deg), studiomdl's normal_blend
    if (a.numBones != b.numBones) return false;
    for (int i = 0; i < a.numBones; i++)
        if (a.bone[i] != b.bone[i] || std::fabs(a.weight[i] - b.weight[i]) > 1e-5f) return false;
    if (std::memcmp(a.color, b.color, 4) != 0) return false;
    return true;
}

size_t HashPos(const CVertex& v) {
    uint32_t h = 2166136261u;
    auto mix = [&](float f) {
        uint32_t b;
        std::memcpy(&b, &f, 4);
        h = (h ^ b) * 16777619u;
    };
    mix(v.pos.x);
    mix(v.pos.y);
    mix(v.pos.z);
    mix(v.uv.x);
    mix(v.uv.y);
    return h;
}

struct MeshBuilder {
    int material;
    std::vector<CVertex> verts;
    std::vector<int> maxLod, minLod;
    std::vector<std::vector<uint32_t>> lodIdx;
    std::unordered_multimap<size_t, int> lookup;

    int Add(const CVertex& v, int lod) {
        size_t h = HashPos(v);
        auto range = lookup.equal_range(h);
        for (auto it = range.first; it != range.second; ++it)
            if (SameVertex(verts[it->second], v)) {
                int i = it->second;
                maxLod[i] = std::max(maxLod[i], lod);
                return i;
            }
        int i = static_cast<int>(verts.size());
        verts.push_back(v);
        maxLod.push_back(lod);
        minLod.push_back(lod);
        lookup.emplace(h, i);
        return i;
    }
};

CVertex MakeVertex(const SrcVertex& sv, const std::vector<GBone>& bones,
                   const std::vector<std::pair<int, int>>& boneReplace) {
    CVertex v;
    v.pos = sv.pos;
    v.normal = sv.normal;
    v.uv = sv.uv;
    v.uv2 = sv.uv2;
    if (sv.hasColor) {
        for (int i = 0; i < 4; i++) v.color[i] = static_cast<uint8_t>(std::clamp((&sv.color.x)[i], 0.f, 1.f) * 255.f + 0.5f);
    }
    std::vector<BoneWeight> w = sv.weights;
    for (auto& bw : w)
        for (auto& r : boneReplace)
            if (bw.bone == r.first) bw.bone = r.second;
    // merge duplicates created by replacement
    std::vector<BoneWeight> m;
    for (auto& bw : w) {
        bool found = false;
        for (auto& x : m)
            if (x.bone == bw.bone) {
                x.weight += bw.weight;
                found = true;
            }
        if (!found) m.push_back(bw);
    }
    std::stable_sort(m.begin(), m.end(), [](const BoneWeight& a, const BoneWeight& b) { return a.weight > b.weight; });
    if (m.size() > 3) m.resize(3);
    float t = 0;
    for (auto& x : m) t += x.weight;
    v.numBones = static_cast<int>(m.size());
    for (int i = 0; i < v.numBones; i++) {
        v.bone[i] = static_cast<uint8_t>(m[i].bone);
        v.weight[i] = t > 0 ? m[i].weight / t : 1.f / v.numBones;
    }
    (void)bones;
    return v;
}

void CalcTangents(CMesh& mesh, const std::vector<int>& minLod) {
    size_t n = mesh.verts.size();
    std::vector<Vec3> sacc(n), tacc(n);
    for (size_t lod = 0; lod < mesh.lods.size(); lod++) {
        auto& idx = mesh.lods[lod].indices;
        for (size_t t = 0; t + 2 < idx.size(); t += 3) {
            uint32_t a = idx[t], b = idx[t + 1], c = idx[t + 2];
            const CVertex &v0 = mesh.verts[a], &v1 = mesh.verts[b], &v2 = mesh.verts[c];
            Vec3 e1 = v1.pos - v0.pos, e2 = v2.pos - v0.pos;
            float s1 = v1.uv.x - v0.uv.x, t1 = v1.uv.y - v0.uv.y;
            float s2 = v2.uv.x - v0.uv.x, t2 = v2.uv.y - v0.uv.y;
            float r = s1 * t2 - s2 * t1;
            Vec3 sdir, tdir;
            if (std::fabs(r) > 1e-12f) {
                float inv = 1.f / r;
                sdir = (e1 * t2 - e2 * t1) * inv;
                tdir = (e2 * s1 - e1 * s2) * inv;
            } else {
                sdir = Vec3(1, 0, 0);
                tdir = Vec3(0, 1, 0);
            }
            for (uint32_t vi : {a, b, c}) {
                if (minLod[vi] != static_cast<int>(lod)) continue;  // use the finest LOD that has this vertex
                sacc[vi] += sdir;
                tacc[vi] += tdir;
            }
        }
    }
    for (size_t i = 0; i < n; i++) {
        Vec3 nrm = mesh.verts[i].normal;
        Vec3 s = sacc[i], t = tacc[i];
        if (Length(s) < 1e-12f) {
            // pick any perpendicular
            s = std::fabs(nrm.x) < 0.9f ? Cross(nrm, Vec3(1, 0, 0)) : Cross(nrm, Vec3(0, 1, 0));
            s = Cross(s, nrm);
            t = Cross(nrm, s);
        }
        bool leftHanded = Dot(Cross(s, t), nrm) < 0.0f;
        Vec3 tv, sv;
        if (!leftHanded) {
            tv = Cross(nrm, s);
            sv = Normalize(Cross(tv, nrm));
            mesh.verts[i].tangent = {sv.x, sv.y, sv.z, 1.f};
        } else {
            tv = Cross(s, nrm);
            sv = Normalize(Cross(nrm, tv));
            mesh.verts[i].tangent = {sv.x, sv.y, sv.z, -1.f};
        }
    }
}

}  // namespace

void BuildContext::BuildMeshes() {
    // Script LOD list: LOD 0 plus each $lod / $shadowlod in order (shadow must be last).
    std::vector<const QcLod*> lods;
    lods.push_back(nullptr);
    for (auto& l : qc.lods)
        if (!l.shadow) lods.push_back(&l);
    for (auto& l : qc.lods)
        if (l.shadow) lods.push_back(&l);
    // studiomdl keeps script order even when switch points are not ascending; retail
    // has such models (e.g. switch 250 before 40), so the order is not sorted here
    int numLods = static_cast<int>(lods.size());
    if (numLods > kMaxLods) Fatal("too many LODs (%d, max %d)", numLods, kMaxLods);
    out.numLods = numLods;
    out.lodSwitch.clear();
    for (auto* l : lods) out.lodSwitch.push_back(l ? l->switchPoint : 0.f);

    // $lod replacematerial becomes the VTX per-LOD material replacement list
    out.lodMaterialReplacements.assign(numLods, {});
    for (int l = 1; l < numLods; l++)
        for (auto& rpl : lods[l]->replaceMaterials) {
            int t = qc.TextureIndex(rpl.first, false);
            if (t < 0) {
                Warn("LOD %d: replacematerial: unknown material \"%s\"", l, rpl.first.c_str());
                continue;
            }
            out.lodMaterialReplacements[l].push_back({t, StripExtension(FixSlashes(rpl.second))});
        }

    int modelIndex = 0;
    int meshId = 0;
    for (auto& bp : qc.bodyParts) {
        CBodyPart obp;
        obp.name = bp.name;
        for (auto& sm : bp.models) {
            CModel om;
            SrcInst* base = bodyInsts[modelIndex++];
            om.name = sm.blank ? "" : sm.file;
            om.blank = sm.blank || !base;
            if (om.blank) {
                obp.models.push_back(om);
                continue;
            }
            // resolve per-LOD sources
            std::vector<SrcInst*> lodInst(numLods, nullptr);
            std::vector<std::vector<std::pair<int, int>>> lodBoneReplace(numLods);
            std::vector<std::vector<std::string>> lodRemoveMesh(numLods);
            lodInst[0] = base;
            std::string baseName = StripExtension(FixSlashes(sm.file));
            for (int l = 1; l < numLods; l++) {
                const QcLod& L = *lods[l];
                bool found = false;
                SrcInst* inst = base;
                for (auto& r : L.replaceModels) {
                    if (IEquals(StripExtension(FixSlashes(r.from)), baseName)) {
                        found = true;
                        inst = r.source ? Instance(r.source, r.scale, r.reverse, true, r.to) : nullptr;
                    }
                }
                for (auto& rm : L.removeModels)
                    if (IEquals(StripExtension(FixSlashes(rm)), baseName)) {
                        found = true;
                        inst = nullptr;
                    }
                (void)found;
                lodInst[l] = inst;
                // bone replacements (bonetreecollapse expands to every descendant)
                for (auto& rb : L.replaceBones) {
                    int a = FindBone(rb.first), b = FindBone(rb.second);
                    if (a < 0 || b < 0) {
                        Warn("LOD %d: replacebone %s -> %s: bone not found", l, rb.first.c_str(), rb.second.c_str());
                        continue;
                    }
                    lodBoneReplace[l].push_back({a, b});
                }
                for (auto& tc : L.boneTreeCollapses) {
                    int root = FindBone(tc);
                    if (root < 0) {
                        Warn("couldn't find bone %s for bonetreecollapse, skipping", tc.c_str());
                        continue;
                    }
                    for (size_t k = 0; k < bones.size(); k++) {
                        for (int p = bones[k].parent; p != -1; p = bones[p].parent)
                            if (p == root) {
                                lodBoneReplace[l].push_back({static_cast<int>(k), root});
                                break;
                            }
                    }
                }
                lodRemoveMesh[l] = L.removeMeshes;
            }
            for (int l = 1; l < numLods; l++)
                if (lodInst[l] && lodInst[l]->gverts.empty() && !lodInst[l]->verts.empty())
                    Fatal("internal: LOD source %s was not remapped", lodInst[l]->label.c_str());

            // materials of LOD 0, in global texture order
            std::vector<int> mats;
            for (size_t f = 0; f < base->faces.size(); f++)
                if (std::find(mats.begin(), mats.end(), base->faceTexture[f]) == mats.end()) mats.push_back(base->faceTexture[f]);
            std::sort(mats.begin(), mats.end());

            for (int mat : mats) {
                MeshBuilder mb;
                mb.material = mat;
                mb.lodIdx.resize(numLods);
                for (int l = 0; l < numLods; l++) {
                    SrcInst* s = lodInst[l];
                    if (!s) continue;
                    // which texture in this LOD maps to the mesh material
                    std::string matName = qc.textures[mat];
                    bool removed = false;
                    for (auto& rm : lodRemoveMesh[l])
                        if (IEquals(FileBase(rm), FileBase(matName))) removed = true;
                    if (removed) continue;
                    for (size_t f = 0; f < s->faces.size(); f++) {
                        int ft = s->faceTexture[f];
                        if (ft < 0) continue;
                        if (ft != mat) continue;
                        const SrcFace& face = s->faces[f];
                        for (int c = 0; c < 3; c++) {
                            CVertex v = MakeVertex(s->gverts[face.v[c]], bones, lodBoneReplace[l]);
                            mb.lodIdx[l].push_back(static_cast<uint32_t>(mb.Add(v, l)));
                        }
                    }
                }
                if (mb.lodIdx[0].empty()) continue;
                // LODs without their own triangles for this mesh draw nothing.

                // sort vertices coarse-LOD first (stable within a LOD)
                std::vector<int> order(mb.verts.size());
                for (size_t i = 0; i < order.size(); i++) order[i] = static_cast<int>(i);
                std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return mb.maxLod[a] > mb.maxLod[b]; });
                std::vector<uint32_t> remap(order.size());
                CMesh mesh;
                mesh.material = mat;
                mesh.meshId = meshId++;
                std::vector<int> minLod(order.size());
                for (size_t i = 0; i < order.size(); i++) {
                    remap[order[i]] = static_cast<uint32_t>(i);
                    mesh.verts.push_back(mb.verts[order[i]]);
                    mesh.vertMaxLod.push_back(mb.maxLod[order[i]]);
                    minLod[i] = mb.minLod[order[i]];
                }
                mesh.lods.resize(numLods);
                for (int l = 0; l < numLods; l++)
                    for (uint32_t i : mb.lodIdx[l]) mesh.lods[l].indices.push_back(remap[i]);
                for (int l = 0; l < kMaxLods; l++) {
                    int c = 0;
                    for (int ml : mesh.vertMaxLod)
                        if (ml >= l) c++;
                    mesh.lodVertexCount[l] = l < numLods ? c : mesh.lodVertexCount[numLods - 1];
                }
                if (mesh.verts.size() > 65535)
                    Fatal("mesh using material \"%s\" in \"%s\" has %d vertices (max 65535); split the mesh", qc.textures[mat].c_str(),
                          sm.file.c_str(), static_cast<int>(mesh.verts.size()));
                CalcTangents(mesh, minLod);
                // bone usage flags per LOD
                for (int l = 0; l < numLods; l++)
                    for (uint32_t i : mesh.lods[l].indices) {
                        const CVertex& v = mesh.verts[i];
                        for (int b = 0; b < v.numBones; b++) bones[v.bone[b]].flags |= (BONE_USED_BY_VERTEX_LOD0 << l);
                    }
                om.meshes.push_back(std::move(mesh));
            }
            float r = 0;
            for (auto& m : om.meshes)
                for (auto& v : m.verts) r = std::max(r, Length(v.pos));
            om.boundingRadius = r;
            obp.models.push_back(std::move(om));
        }
        out.bodyParts.push_back(std::move(obp));
    }
    // body part bases
    int base = 1;
    for (auto& bp : out.bodyParts) {
        bp.base = base;
        base *= std::max<int>(1, static_cast<int>(bp.models.size()));
    }
    // vertex start offsets
    int vstart = 0;
    for (auto& bp : out.bodyParts)
        for (auto& m : bp.models) {
            m.vertexStart = vstart;
            vstart += m.NumVerts();
        }
    // propagate vertex LOD flags to parents (MarkParentBoneLODs)
    for (size_t i = 0; i < bones.size(); i++) {
        int f = bones[i].flags & BONE_USED_BY_VERTEX_MASK;
        for (int p = bones[i].parent; p != -1; p = bones[p].parent) bones[p].flags |= f;
    }
    // vertex colour / uv2 presence
    for (auto& ip : insts) {
        if (!ip->active) continue;
        for (auto& v : ip->verts) {
            if (v.hasColor) out.hasVertexColor = true;
            if (v.hasUV2) out.hasUV2 = true;
        }
    }
    if (qc.flags & STUDIOHDR_FLAGS_USES_VERTEX_COLOR) out.hasVertexColor = true;
    if (qc.flags & STUDIOHDR_FLAGS_USES_UV2) out.hasUV2 = true;
}

}  // namespace r2c
