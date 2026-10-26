// Source instances and the global bone table.
//
// Follows studiomdl's RemapBones(): bones referenced by vertices, attachments, IK
// chains, bone merges and $definebone are kept; everything else is dropped. The
// built-in default rotation is applied to animations (root bones), not to the bind
// pose, except for $staticprop where it is baked into the vertices.
#include <algorithm>

#include "build/context.h"
#include "core/log.h"
#include "core/strings.h"

namespace r2c {

static void ClipRotations(RadianEuler& r) {
    for (int j = 0; j < 3; j++) {
        while (r[j] >= kPi) r[j] -= 2 * kPi;
        while (r[j] < -kPi) r[j] += 2 * kPi;
    }
}

SrcInst* BuildContext::Instance(const std::shared_ptr<Source>& s, float scale, bool reverse, bool active,
                                const std::string& label) {
    for (auto& i : insts)
        if (i->src == s && i->scale == scale && i->reverse == reverse) {
            i->active |= active;
            return i.get();
        }
    auto inst = std::make_unique<SrcInst>();
    inst->src = s;
    inst->label = label;
    inst->scale = scale;
    inst->reverse = reverse;
    inst->active = active;
    inst->bones = s->bones;
    for (auto& b : inst->bones)
        for (auto& rb : qc.renameBones)
            if (IEquals(b.name, rb.first)) {
                b.name = rb.second;
                break;
            }
    inst->frames = s->frames;
    for (auto& f : inst->frames)
        for (auto& b : f) b.pos *= scale;
    inst->verts = s->vertices;
    for (auto& v : inst->verts) v.pos *= scale;
    inst->faces = s->faces;
    if (reverse)
        for (auto& f : inst->faces) std::swap(f.v[1], f.v[2]);
    // Only visible models add materials; collision and animation-only files must not
    // grow the texture table (studiomdl removes collision materials the same way).
    inst->faceTexture.assign(inst->faces.size(), -1);
    inst->meshVertexTexture.assign(s->meshVertices.size(), -1);
    if (active) {
        // textures in first-use order, mesh-only vertices at their place among the faces
        size_t mv = 0;
        for (size_t i = 0; i <= inst->faces.size(); i++) {
            for (; mv < s->meshVertices.size() && s->meshVertices[mv].faceOrder <= i; mv++)
                inst->meshVertexTexture[mv] = qc.TextureIndex(s->materials[s->meshVertices[mv].material], true);
            if (i < inst->faces.size()) inst->faceTexture[i] = qc.TextureIndex(s->materials[inst->faces[i].material], true);
        }
    }
    insts.push_back(std::move(inst));
    return insts.back().get();
}

int BuildContext::FindBone(const std::string& name) const {
    for (size_t i = 0; i < bones.size(); i++)
        if (IEquals(bones[i].name, name)) return static_cast<int>(i);
    return -1;
}

void BuildContext::LoadInstances() {
    for (auto& bp : qc.bodyParts)
        for (auto& sm : bp.models) {
            if (sm.blank || !sm.source) {
                bodyInsts.push_back(nullptr);
                continue;
            }
            bodyInsts.push_back(Instance(sm.source, sm.scale, sm.reverse, true, sm.file));
        }
    for (auto& lod : qc.lods)
        for (auto& r : lod.replaceModels)
            if (r.source) Instance(r.source, r.scale, r.reverse, true, r.to);
    for (auto& a : qc.animations)
        if (a->source) Instance(a->source, a->sourceScale, false, false, a->file);
    if (qc.collision.present && qc.collision.source)
        Instance(qc.collision.source, qc.collision.sourceScale, false, false, qc.collision.file);
}

void BuildContext::MakeStaticProp() {
    Mat3x4 rotated = AngleMatrix(qc.defaultRotation);
    rotated.SetOrigin(qc.defaultAdjust);
    for (auto& ip : insts) {
        SrcInst& s = *ip;
        s.bones[0].name = "static_prop";
        for (auto& b : s.bones) b.parent = -1;
        for (auto& v : s.verts) {
            for (auto& w : v.weights) w.bone = 0;
            // merge into a single weight
            v.weights.resize(1);
            v.weights[0].weight = 1;
            v.pos = Transform(rotated, v.pos);
            v.normal = Rotate(rotated, v.normal);
            Vec3 t = Rotate(rotated, Vec3(v.tangent.x, v.tangent.y, v.tangent.z));
            v.tangent = {t.x, t.y, t.z, v.tangent.w};
        }
        if (qc.autoCenter) {
            // handled after bounds are known (see BuildMeshes)
        }
        s.frames.resize(1);
        s.frames[0][0].pos = Vec3();
        s.frames[0][0].rot = RadianEuler();
    }
    for (auto& a : qc.animations) {
        a->rotation = RadianEuler();
        a->adjust = Vec3();
        a->startFrame = a->source ? a->source->startFrame : 0;
        a->endFrame = a->startFrame;
        a->frameRangeSet = true;
    }
    for (auto& at : qc.attachments) {
        if (!at.exactMatrix) at.local = Concat(rotated, at.local);
        at.bone = "static_prop";
        at.rigid = false;
        at.absolute = false;
    }
    qc.flags |= STUDIOHDR_FLAGS_STATIC_PROP;
}

void BuildContext::BuildRawTransforms(const SrcInst& s, int frame, float scale, const Vec3& shift, const RadianEuler& rotate,
                                      std::vector<Mat3x4>& out) const {
    Mat3x4 rootxform = AngleMatrix(rotate);
    int nf = static_cast<int>(s.frames.size());
    if (frame) frame = frame % nf;
    out.resize(s.bones.size());
    for (size_t k = 0; k < s.bones.size(); k++) {
        Vec3 pos = s.frames[frame][k].pos * scale;
        RadianEuler rot = s.frames[frame][k].rot;
        if (s.bones[k].parent == -1) {
            pos = Rotate(rootxform, pos - shift);
            Mat3x4 bm = Concat(rootxform, AngleMatrix(rot));
            rot = MatrixAngles(bm);
            ClipRotations(rot);
        }
        Mat3x4 bonematrix = AngleMatrix(rot, pos);
        if (s.bones[k].parent == -1)
            out[k] = bonematrix;
        else
            out[k] = Concat(out[s.bones[k].parent], bonematrix);
    }
}

void BuildContext::TranslateAnimations(const SrcInst& s, const std::vector<Mat3x4>& src, std::vector<Mat3x4>& dest) const {
    dest.resize(bones.size());
    for (size_t k = 0; k < bones.size(); k++) {
        int q = s.globalToLocal[k];
        if (q == -1) {
            Mat3x4 bm = AngleMatrix(bones[k].rot, bones[k].pos);
            dest[k] = bones[k].parent >= 0 ? Concat(dest[bones[k].parent], bm) : bm;
        } else {
            dest[k] = Concat(src[q], bones[k].srcRealign);
        }
    }
}

void BuildContext::ConvertAnimation(const SrcInst& s, int frame, float scale, const Vec3& shift, const RadianEuler& rotate,
                                    std::vector<BoneXf>& dest) const {
    std::vector<Mat3x4> srcBoneToWorld, destBoneToWorld;
    BuildRawTransforms(s, frame, scale, shift, rotate, srcBoneToWorld);
    TranslateAnimations(s, srcBoneToWorld, destBoneToWorld);
    dest.resize(bones.size());
    for (size_t k = 0; k < bones.size(); k++) {
        Mat3x4 bm;
        if (bones[k].parent == -1)
            bm = destBoneToWorld[k];
        else
            bm = Concat(Invert(destBoneToWorld[bones[k].parent]), destBoneToWorld[k]);
        dest[k].rot = MatrixAngles(bm);
        dest[k].pos = bm.Origin();
        ClipRotations(dest[k].rot);
    }
    // Where a source bone is the model bone under the same parent with no realignment,
    // take its local values directly. The world round trip above adds float noise
    // (1e-4 to 3e-3 on bones thousands of units out), enough to turn a bone at its
    // bind pose into an animated one.
    auto identity = [](const Mat3x4& m) {
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 4; c++)
                if (std::fabs(m.m[r][c] - (r == c ? 1.f : 0.f)) > 1e-5f) return false;
        return true;
    };
    int nf = static_cast<int>(s.frames.size());
    int fr = frame && nf ? frame % nf : 0;
    for (size_t k = 0; k < bones.size(); k++) {
        int p = bones[k].parent, q = s.globalToLocal[k];
        if (p < 0 || q < 0 || scale != 1.f) continue;
        int sp = s.bones[q].parent;
        if (sp < 0 || s.localToGlobal[sp] != p) continue;
        if (!identity(bones[k].srcRealign) || !identity(bones[p].srcRealign)) continue;
        dest[k].pos = s.frames[fr][q].pos;
        dest[k].rot = s.frames[fr][q].rot;
        ClipRotations(dest[k].rot);
    }
}

void BuildContext::BuildBones() {
    // --- TagUsedBones
    for (auto& ip : insts) {
        SrcInst& s = *ip;
        s.boneflags.assign(s.bones.size(), 0);
        s.boneref.assign(s.bones.size(), 0);
        if (s.active) {
            for (auto& v : s.verts)
                for (auto& w : v.weights) s.boneflags[w.bone] |= BONE_USED_BY_VERTEX_LOD0;
        }
    }
    for (auto& ip : insts) {
        SrcInst& s = *ip;
        for (auto& at : qc.attachments) {
            for (size_t j = 0; j < s.bones.size(); j++) {
                if (!IEquals(at.bone, s.bones[j].name)) continue;
                if (at.rigid) {
                    for (int n = static_cast<int>(j); n != -1; n = s.bones[n].parent)
                        if (s.boneflags[n] & BONE_USED_BY_VERTEX_LOD0) {
                            s.boneflags[n] |= BONE_USED_BY_ATTACHMENT;
                            break;
                        }
                } else
                    s.boneflags[j] |= BONE_USED_BY_ATTACHMENT;
            }
        }
        for (auto& ik : qc.ikChains)
            for (size_t j = 0; j < s.bones.size(); j++)
                if (IEquals(ik.bone, s.bones[j].name)) s.boneflags[j] |= BONE_USED_BY_ATTACHMENT;
        for (auto& bm : qc.boneMerge)
            for (size_t j = 0; j < s.bones.size(); j++)
                if (IEquals(bm, s.bones[j].name)) s.boneflags[j] |= BONE_USED_BY_BONE_MERGE;
        // Later studiomdl versions keep procedural and screen-aligned bones even
        // without vertices; mark them so they survive.
        auto procedural = [&](const std::string& name) {
            for (size_t j = 0; j < s.bones.size(); j++)
                if (IEquals(name, s.bones[j].name)) s.boneflags[j] |= BONE_ALWAYS_PROCEDURAL;
        };
        for (auto& jb : qc.jiggleBones) procedural(jb.bone);
        for (auto& b : qc.axisInterpBones) procedural(b.bone);
        for (auto& b : qc.quatInterpBones) procedural(b.bone);
        for (auto& b : qc.aimAtBones) procedural(b.bone);
        // bones that drive procedural bones must survive even without vertices
        auto keep = [&](const std::string& name) {
            for (size_t j = 0; j < s.bones.size(); j++)
                if (IEquals(name, s.bones[j].name)) s.boneflags[j] |= BONE_USED_BY_ATTACHMENT;
        };
        for (auto& b : qc.axisInterpBones) keep(b.control);
        for (auto& b : qc.quatInterpBones) keep(b.control);
        for (auto& b : qc.aimAtBones) {
            keep(b.parent);
            keep(b.aim);
        }
        for (auto& p : qc.uiPanels)
            for (auto& name : p.parents) keep(name);
        for (auto& name : qc.boneFollowers) keep(name);
        for (auto& sa : qc.screenAlign)
            for (size_t j = 0; j < s.bones.size(); j++)
                if (IEquals(sa.first, s.bones[j].name)) s.boneflags[j] |= sa.second;
        for (auto& hs : qc.hitboxSets)
            for (auto& hb : hs.boxes)
                for (size_t j = 0; j < s.bones.size(); j++)
                    if (IEquals(hb.bone, s.bones[j].name) && s.active) s.boneflags[j] |= BONE_USED_BY_HITBOX;
        for (size_t k = 0; k < s.bones.size(); k++) {
            int f = s.boneflags[k];
            if (!f) continue;
            s.boneref[k] |= f;
            for (int n = s.bones[k].parent; n != -1; n = s.bones[n].parent) s.boneref[n] |= s.boneref[k];
        }
    }

    // --- BuildGlobalBonetable
    bones.clear();
    for (auto& d : qc.defineBones) {
        int k = FindBone(d.name);
        if (k == -1) {
            GBone b;
            b.name = d.name;
            b.parent = d.parent.empty() ? -1 : FindBone(d.parent);
            if (!d.parent.empty() && b.parent == -1)
                Warn("$definebone %s: parent bone %s not found", d.name.c_str(), d.parent.c_str());
            b.preDefined = true;
            b.rawLocal = b.rawLocalOriginal = d.rawLocal;
            bones.push_back(b);
            k = static_cast<int>(bones.size() - 1);
        }
        bones[k].dontCollapse = true;
        bones[k].srcRealign = d.srcRealign;
        bones[k].preAligned = true;
    }
    for (auto& bm : qc.boneMerge) {
        int k = FindBone(bm);
        if (k >= 0) bones[k].flags |= BONE_USED_BY_BONE_MERGE;
    }
    for (auto& ip : insts) {
        SrcInst& s = *ip;
        std::vector<Mat3x4> srcBoneToWorld;
        BuildRawTransforms(s, 0, 1, Vec3(), RadianEuler(), srcBoneToWorld);
        for (size_t j = 0; j < s.bones.size(); j++) {
            if (!s.boneref[j]) continue;
            int k = FindBone(s.bones[j].name);
            if (k == -1) {
                GBone b;
                b.name = s.bones[j].name;
                int n = s.bones[j].parent;
                b.parent = n != -1 ? FindBone(s.bones[n].name) : -1;
                b.flags = s.boneflags[j];
                b.rawLocal = AngleMatrix(s.frames[0][j].rot, s.frames[0][j].pos);
                b.rawLocalOriginal = b.rawLocal;
                b.boneToPose = srcBoneToWorld[j];
                bones.push_back(b);
            } else {
                bones[k].flags |= s.boneflags[j];
            }
        }
    }
    if (bones.empty()) Fatal("model has no bones");

    // --- BuildGlobalBoneToPose
    for (size_t k = 0; k < bones.size(); k++) {
        if (bones[k].parent == -1)
            bones[k].boneToPose = bones[k].rawLocal;
        else
            bones[k].boneToPose = Concat(bones[bones[k].parent].boneToPose, bones[k].rawLocal);
    }

    // --- EnforceHierarchy
    for (auto& h : qc.hierarchy) {
        int k = FindBone(h.first);
        int j = h.second.empty() ? -1 : FindBone(h.second);
        if (k == -1) Fatal("unknown bone \"%s\" in $hierarchy", h.first.c_str());
        if (j == -1 && !h.second.empty()) Fatal("unknown bone \"%s\" in $hierarchy", h.second.c_str());
        bones[k].parent = j;
    }
    {
        bool sorted = true;
        int count = 0;
        do {
            sorted = false;
            count++;
            for (int i = 0; i < static_cast<int>(bones.size()); i++) {
                if (bones[i].parent > i) {
                    int j = bones[i].parent;
                    std::swap(bones[i], bones[j]);
                    for (int k = i; k < static_cast<int>(bones.size()); k++) {
                        if (bones[k].parent == i)
                            bones[k].parent = j;
                        else if (bones[k].parent == j)
                            bones[k].parent = i;
                    }
                    sorted = true;
                }
            }
            if (count > 1000) Fatal("circular bone hierarchy");
        } while (sorted);
    }
    // parents inherit what their children are used by, not the children's own procedural or
    // screen-align flags (else every ancestor of a jiggle bone would stop being animated)
    for (size_t k = 0; k < bones.size(); k++)
        for (int n = bones[k].parent; n != -1; n = bones[n].parent) bones[n].flags |= bones[k].flags & BONE_USED_BY_ANYTHING;

    // --- CollapseBones ($collapsebones / $alwayscollapse)
    if (qc.collapseBones || !qc.alwaysCollapse.empty()) {
        auto hasAnimation = [&](const std::string& n) {
            // studiomdl: bone animates if any animation frame differs from its first frame
            for (auto& a : qc.animations) {
                if (!a->source) continue;
                int b = a->source->FindBone(n);
                if (b < 0) continue;
                auto& fr = a->source->frames;
                for (size_t f = 1; f < fr.size(); f++) {
                    if (Length(fr[f][b].pos - fr[0][b].pos) > 0.01f) return true;
                    for (int c = 0; c < 3; c++)
                        if (std::fabs(fr[f][b].rot[c] - fr[0][b].rot[c]) > 0.01f) return true;
                }
            }
            return false;
        };
        int count = 0;
        for (int k = 0; k < static_cast<int>(bones.size()); k++) {
            if (bones[k].dontCollapse) continue;
            bool forced = false;
            for (auto& c : qc.alwaysCollapse)
                if (IEquals(c, bones[k].name)) forced = true;
            bool procedural = (bones[k].flags & BONE_ALWAYS_PROCEDURAL) != 0;
            bool ik = false;
            for (auto& c : qc.ikChains)
                if (IEquals(c.bone, bones[k].name)) ik = true;
            bool shouldCollapse = forced || (!hasAnimation(bones[k].name) && !procedural && !ik);
            if (!qc.collapseBones && !forced) continue;
            if (bones[k].flags != 0 && !shouldCollapse) continue;
            Info("collapsing %s", bones[k].name.c_str());
            count++;
            int m = bones[k].parent;
            bones.erase(bones.begin() + k);
            for (int j = k; j < static_cast<int>(bones.size()); j++) {
                if (bones[j].parent == k)
                    bones[j].parent = m;
                else if (bones[j].parent >= k)
                    bones[j].parent--;
            }
            k--;
        }
        if (count) Info("Collapsed %d bones", count);
    }

    // --- RebuildLocalPose
    {
        std::vector<Mat3x4> poseToBone(bones.size());
        for (size_t k = 0; k < bones.size(); k++) {
            if (bones[k].parent == -1)
                bones[k].rawLocal = bones[k].boneToPose;
            else
                bones[k].rawLocal = Concat(poseToBone[bones[k].parent], bones[k].boneToPose);
            bones[k].rot = MatrixAngles(bones[k].rawLocal);
            bones[k].pos = bones[k].rawLocal.Origin();
            poseToBone[k] = Invert(bones[k].boneToPose);
        }
    }

    if (bones.size() > 256) Fatal("too many bones used in model: %d (max 256)", static_cast<int>(bones.size()));

    // --- MapSourcesToGlobalBonetable
    for (auto& ip : insts) {
        SrcInst& s = *ip;
        s.globalToLocal.assign(bones.size(), -1);
        s.localToGlobal.assign(s.bones.size(), -1);
        for (size_t j = 0; j < s.bones.size(); j++) {
            int k = FindBone(s.bones[j].name);
            if (k == -1) {
                int m = s.bones[j].parent;
                while (m != -1 && (k = FindBone(s.bones[m].name)) == -1) m = s.bones[m].parent;
                if (k == -1) k = 0;
                s.localToGlobal[j] = k;
            } else {
                s.localToGlobal[j] = k;
                s.globalToLocal[k] = static_cast<int>(j);
            }
        }
    }

    // --- RealignBones (IK chains and $realignbones)
    {
        std::vector<int> childbone(bones.size(), -1);
        for (auto& ik : qc.ikChains) {
            // links are bone, parent, grandparent (studiomdl LinkIKChains)
            int b2 = FindBone(ik.links.size() == 3 ? ik.links[2] : ik.bone);
            if (b2 < 0) continue;
            int b1 = ik.links.size() == 3 ? FindBone(ik.links[1]) : bones[b2].parent;
            int b0 = ik.links.size() == 3 ? FindBone(ik.links[0]) : (b1 >= 0 ? bones[b1].parent : -1);
            if (b0 < 0) continue;
            if (childbone[b0] == -1 || childbone[b0] == b1)
                childbone[b0] = b1;
            else
                Fatal("trying to realign bone \"%s\" with two children", bones[b0].name.c_str());
            if (childbone[b1] == -1 || childbone[b1] == b2)
                childbone[b1] = b2;
            else
                Fatal("trying to realign bone \"%s\" with two children", bones[b1].name.c_str());
        }
        if (qc.realignBones) {
            std::vector<int> children(bones.size(), 0);
            for (auto& b : bones)
                if (b.parent != -1) children[b.parent]++;
            for (size_t k = 0; k < bones.size(); k++)
                if (bones[k].parent != -1 && children[bones[k].parent] == 1) childbone[bones[k].parent] = static_cast<int>(k);
        }
        std::vector<Mat3x4> boneToPose(bones.size());
        for (size_t k = 0; k < bones.size(); k++) boneToPose[k] = bones[k].boneToPose;
        for (size_t k = 0; k < bones.size(); k++) {
            if (bones[k].preAligned || childbone[k] == -1) continue;
            const GBone& c = bones[childbone[k]];
            float d = Length(c.pos);
            if (d - c.pos.x <= 0.01f) continue;
            Vec3 v2 = c.boneToPose.Origin(), v3 = bones[k].boneToPose.Origin();
            Vec3 forward = Normalize(v2 - v3);
            Vec3 f2 = boneToPose[k].Column(0), l2 = boneToPose[k].Column(1), u2 = boneToPose[k].Column(2);
            float d1 = std::fabs(Dot(forward, f2)), d2 = std::fabs(Dot(forward, l2)), d3 = std::fabs(Dot(forward, u2));
            Vec3 up;
            if (d1 <= d2 && d1 <= d3)
                up = Normalize(Cross(forward, f2));
            else if (d2 <= d1 && d2 <= d3)
                up = Normalize(Cross(forward, l2));
            else
                up = Normalize(Cross(forward, u2));
            Vec3 left = Cross(up, forward);
            for (int r = 0; r < 3; r++) {
                boneToPose[k].m[r][0] = forward[r];
                boneToPose[k].m[r][1] = left[r];
                boneToPose[k].m[r][2] = up[r];
                boneToPose[k].m[r][3] = v3[r];
            }
        }
        for (auto& fr : qc.forcedRealign) {
            int k = FindBone(fr.bone);
            if (k == -1) Fatal("unknown bone %s in $forcerealign", fr.bone.c_str());
            boneToPose[k] = Concat(boneToPose[k], AngleMatrix(fr.rot));
        }
        for (size_t k = 0; k < bones.size(); k++) {
            if (bones[k].preAligned) continue;
            bones[k].srcRealign = Concat(Invert(bones[k].boneToPose), boneToPose[k]);
            bones[k].boneToPose = boneToPose[k];
        }
        for (size_t k = 0; k < bones.size(); k++) {
            if (bones[k].preAligned) continue;
            Mat3x4 bm = bones[k].parent == -1 ? bones[k].boneToPose
                                              : Concat(Invert(bones[bones[k].parent].boneToPose), bones[k].boneToPose);
            bones[k].rot = MatrixAngles(bm);
            bones[k].pos = bm.Origin();
        }
        for (size_t k = 0; k < bones.size(); k++) {
            Mat3x4 bm = AngleMatrix(bones[k].rot, bones[k].pos);
            bones[k].boneToPose = bones[k].parent == -1 ? bm : Concat(bones[bones[k].parent].boneToPose, bm);
        }
    }
    Verbose("%d bones", static_cast<int>(bones.size()));
}

void BuildContext::RemapVertices() {
    for (auto& ip : insts) {
        SrcInst& s = *ip;
        if (s.verts.empty()) continue;
        std::vector<Mat3x4> srcBoneToWorld, destBoneToWorld;
        BuildRawTransforms(s, 0, 1, Vec3(), RadianEuler(), srcBoneToWorld);
        TranslateAnimations(s, srcBoneToWorld, destBoneToWorld);
        std::vector<Mat3x4> invDest(destBoneToWorld.size());
        for (size_t k = 0; k < destBoneToWorld.size(); k++) invDest[k] = Invert(destBoneToWorld[k]);
        s.gverts.resize(s.verts.size());
        for (size_t j = 0; j < s.verts.size(); j++) {
            const SrcVertex& v = s.verts[j];
            SrcVertex o = v;
            o.weights.clear();
            Vec3 vdest, ndest, tdest;
            for (auto& w : v.weights) {
                int k = s.localToGlobal[w.bone];
                bool merged = false;
                for (auto& gw : o.weights)
                    if (gw.bone == k) {
                        gw.weight += w.weight;
                        merged = true;
                    }
                if (!merged) o.weights.push_back({k, w.weight});
                Vec3 local = Transform(invDest[k], v.pos);
                vdest += Transform(bones[k].boneToPose, local) * w.weight;
                Vec3 ln = Rotate(invDest[k], v.normal);
                ndest += Rotate(bones[k].boneToPose, ln) * w.weight;
                Vec3 lt = Rotate(invDest[k], Vec3(v.tangent.x, v.tangent.y, v.tangent.z));
                tdest += Rotate(bones[k].boneToPose, lt) * w.weight;
            }
            o.pos = vdest;
            o.normal = Normalize(ndest);
            if (v.hasTangent) {
                Vec3 t = Normalize(tdest);
                o.tangent = {t.x, t.y, t.z, v.tangent.w};
            }
            // keep the strongest three (merging may have changed order)
            std::stable_sort(o.weights.begin(), o.weights.end(),
                             [](const BoneWeight& a, const BoneWeight& b) { return a.weight > b.weight; });
            if (o.weights.size() > 3) o.weights.resize(3);
            float t = 0;
            for (auto& w : o.weights) t += w.weight;
            if (t > 0)
                for (auto& w : o.weights) w.weight /= t;
            s.gverts[j] = o;
        }
    }
}

}  // namespace r2c
