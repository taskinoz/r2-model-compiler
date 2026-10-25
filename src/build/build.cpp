// Build orchestration plus the smaller stages: attachments, IK, hitboxes, bounds,
// sequences, skins and the final bone table.
#include <algorithm>
#include <cmath>
#include <limits>

#include "build/context.h"
#include "core/buffer.h"
#include "core/log.h"
#include "core/strings.h"

namespace r2c {

// The hash the client looks RUI panel meshes up by (a weapon script's "mesh" name):
// MurmurHash3 x86_32 over the name with every byte ANDed with 0xDF (case folding),
// seed 0, with the terminating nul counted in the length. Reproduces all 163 retail
// mp_common panel hashes.
static uint32_t UIPanelHash(const std::string& name) {
    auto rol = [](uint32_t x, int r) { return (x << r) | (x >> (32 - r)); };
    auto mix = [&](uint32_t k) { return rol(k * 0xcc9e2d51u, 15) * 0x1b873593u; };
    uint32_t h = 0, len = 0;
    size_t i = 0;
    for (; i + 4 <= name.size(); i += 4, len += 4) {
        uint32_t w = 0;
        for (int b = 0; b < 4; b++) w |= static_cast<uint32_t>(static_cast<uint8_t>(name[i + b])) << (8 * b);
        h = rol(h ^ mix(w & 0xdfdfdfdfu), 13) * 5 + 0xe6546b64u;
    }
    uint32_t t = 0;
    for (size_t b = 0; i + b < name.size(); b++) t |= static_cast<uint32_t>(static_cast<uint8_t>(name[i + b]) & 0xdf) << (8 * b);
    len += static_cast<uint32_t>(name.size() - i) + 1;  // the length includes the nul
    h ^= mix(t) ^ len;
    h ^= h >> 16; h *= 0x85ebca6bu; h ^= h >> 13; h *= 0xc2b2ae35u;
    return h ^ (h >> 16);
}

void EncodeAnimations(BuildContext& ctx);
void ProcessIKRules(BuildContext& ctx);
void BuildCollisionModel(BuildContext& ctx);
void BuildPerTriAABB(BuildContext& ctx);

void BuildContext::BuildAttachments() {
    for (auto& qa : qc.attachments) {
        CAttachment a;
        a.name = qa.name;
        a.flags = qa.flags;
        int k = FindBone(qa.bone);
        Mat3x4 boneToPose, poseToBone;
        if (k >= 0) {
            boneToPose = bones[k].boneToPose;
            poseToBone = Invert(boneToPose);
        } else {
            // the bone was optimised out: walk up its source hierarchy to a kept bone
            bool found = false;
            for (auto& ip : insts) {
                SrcInst& s = *ip;
                int j = s.src->FindBone(qa.bone);
                if (j < 0) continue;
                std::vector<Mat3x4> b2w;
                BuildRawTransforms(s, 0, 1, Vec3(), RadianEuler(), b2w);
                boneToPose = b2w[j];
                int m = j;
                while (m != -1 && s.globalToLocal[s.localToGlobal[m]] != m) m = s.bones[m].parent;
                if (m == -1) Fatal("unable to find valid bone for attachment %s:%s", qa.name.c_str(), qa.bone.c_str());
                poseToBone = Invert(b2w[m]);
                k = s.localToGlobal[m];
                found = true;
                break;
            }
            if (!found) Fatal("unknown attachment link '%s'", qa.bone.c_str());
        }
        a.bone = k;
        Mat3x4 world = qa.absolute ? qa.local : Concat(boneToPose, qa.local);
        a.local = qa.exactMatrix ? qa.local : Concat(poseToBone, world);
        for (int j = k; j != -1; j = bones[j].parent) bones[j].flags |= BONE_USED_BY_ATTACHMENT;
        out.attachments.push_back(a);
    }
}

void BuildContext::BuildIK() {
    for (auto& c : qc.ikChains) {
        CIKChain ch;
        ch.name = c.name;
        ch.linkType = 0;
        ch.unk = c.unk;
        int k = FindBone(c.bone);
        if (k == -1) Fatal("unknown bone '%s' in ikchain '%s'", c.bone.c_str(), c.name.c_str());
        int k1 = -1, k0 = -1;
        if (c.links.size() == 3) {
            k0 = FindBone(c.links[0]);
            k1 = FindBone(c.links[1]);
            k = FindBone(c.links[2]);
            if (k0 < 0 || k1 < 0 || k < 0) Fatal("ikchain '%s': unknown link bone", c.name.c_str());
        } else {
            k1 = bones[k].parent;
            if (k1 == -1) Fatal("ikchain '%s' too close to root, no parent knee/elbow", c.name.c_str());
            k0 = bones[k1].parent;
            if (k0 == -1) Fatal("ikchain '%s' too close to root, no parent hip/shoulder", c.name.c_str());
        }
        ch.links.resize(3);
        ch.links[0].bone = k0;
        ch.links[1].bone = k1;
        ch.links[2].bone = k;
        ch.links[0].kneeDir = c.knee;
        for (int b : {k0, k1, k}) bones[b].flags |= BONE_USED_BY_ATTACHMENT;
        out.ikChains.push_back(ch);
    }
    // bones an IK chain moves: its links and everything below them (retail, all models)
    std::vector<bool> influenced(bones.size(), false);
    for (auto& ch : out.ikChains)
        for (auto& l : ch.links) influenced[l.bone] = true;
    for (size_t b = 0; b < bones.size(); b++) {
        if (bones[b].parent >= 0 && influenced[bones[b].parent]) influenced[b] = true;
        if (influenced[b]) bones[b].flags |= BONE_IKCHAIN_INFLUENCE;
    }
    for (auto& l : qc.ikAutoplayLocks) {
        int c = qc.FindIKChain(l.chain);
        if (c < 0) Fatal("unknown chain '%s' in ikautoplaylock", l.chain.c_str());
        CIKLock lk;
        lk.chain = c;
        lk.posWeight = l.posWeight;
        lk.localQWeight = l.localQWeight;
        out.ikAutoplayLocks.push_back(lk);
    }
}

void BuildContext::BuildHitboxes() {
    // hit groups inherit from the parent bone (no $hgroup support beyond default 0)
    for (auto& b : bones) b.group = 0;
    // per-bone vertex bounds (also used for sequence boxes)
    for (auto& b : bones) {
        b.bmin = Vec3(9999, 9999, 9999);
        b.bmax = Vec3(-9999, -9999, -9999);
    }
    for (auto* s : bodyInsts) {
        if (!s) continue;
        for (auto& v : s->gverts)
            for (auto& w : v.weights) {
                Vec3 p = Transform(Invert(bones[w.bone].boneToPose), v.pos);
                bones[w.bone].bmin = Min(bones[w.bone].bmin, p);
                bones[w.bone].bmax = Max(bones[w.bone].bmax, p);
            }
    }
    if (qc.hitboxSets.empty() && !qc.autoHitboxes) {
        // Respawn's studiomdl flags the hitboxes as generated but writes none
        CHitboxSet set;
        set.name = "default";
        qc.flags |= STUDIOHDR_FLAGS_AUTOGENERATED_HITBOX;
        out.hitboxSets.push_back(set);
    } else if (qc.hitboxSets.empty()) {
        // $autohitboxes: one box per bone from its vertices, as Valve's studiomdl
        CHitboxSet set;
        set.name = "default";
        qc.flags |= STUDIOHDR_FLAGS_AUTOGENERATED_HITBOX;
        std::vector<Vec3> mn(bones.size()), mx(bones.size());
        for (size_t k = 0; k < bones.size(); k++) {
            mn[k] = bones[k].bmin;
            mx[k] = bones[k].bmax;
        }
        for (size_t k = 0; k < bones.size(); k++) {
            int j = bones[k].parent;
            if (j == -1) continue;
            mn[j] = Min(mn[j], bones[k].pos);
            mx[j] = Max(mx[j], bones[k].pos);
        }
        for (size_t k = 0; k < bones.size(); k++) {
            if (mn[k].x < mx[k].x - 1 && mn[k].y < mx[k].y - 1 && mn[k].z < mx[k].z - 1) {
                CHitbox h;
                h.bone = static_cast<int>(k);
                h.group = bones[k].group;
                h.bmin = mn[k];
                h.bmax = mx[k];
                set.boxes.push_back(h);
            }
        }
        out.hitboxSets.push_back(set);
    } else {
        qc.flags &= ~STUDIOHDR_FLAGS_AUTOGENERATED_HITBOX;
        for (auto& qs : qc.hitboxSets) {
            CHitboxSet set;
            set.name = qs.name;
            for (auto& qh : qs.boxes) {
                CHitbox h;
                h.bone = FindBone(qh.bone);
                if (h.bone == -1) Fatal("cannot find bone %s for hitbox", qh.bone.c_str());
                h.group = qh.group;
                h.bmin = qh.bmin;
                h.bmax = qh.bmax;
                h.name = qh.name;
                h.critOverride = qh.critOverride;
                h.hitDataGroup = qh.hitDataGroup;
                set.boxes.push_back(h);
            }
            out.hitboxSets.push_back(set);
        }
    }
    for (auto& set : out.hitboxSets)
        for (auto& h : set.boxes)
            for (int k = h.bone; k != -1; k = bones[k].parent) bones[k].flags |= BONE_USED_BY_HITBOX;
}

void BuildContext::BuildBounds() {
    // CalcSequenceBoundingBoxes: every posed vertex, plus each bone's hitbox bounds as a
    // transformed AABB. Retail leaves the per-bone vertex bounds out of the AABB step
    // (rotated vertex boxes would overshoot the vertices themselves).
    std::vector<Vec3> rmin(bones.size(), Vec3(9999, 9999, 9999)), rmax(bones.size(), Vec3(-9999, -9999, -9999));
    for (auto& set : out.hitboxSets)
        for (auto& h : set.boxes) {
            rmin[h.bone] = Min(rmin[h.bone], h.bmin);
            rmax[h.bone] = Max(rmax[h.bone], h.bmax);
        }
    for (auto& a : anims) {
        Vec3 bmin(9999, 9999, 9999), bmax(-9999, -9999, -9999);
        for (int j = 0; j < a.numFrames; j++) {
            std::vector<Mat3x4> bt;
            CalcBoneTransforms(a, j, bt);
            std::vector<Mat3x4> pt(bones.size());
            for (size_t k = 0; k < bones.size(); k++) {
                pt[k] = Concat(bt[k], Invert(bones[k].boneToPose));
                if (rmin[k].x <= rmax[k].x) {
                    // TransformAABB
                    for (int c = 0; c < 8; c++) {
                        Vec3 corner((c & 1) ? rmax[k].x : rmin[k].x, (c & 2) ? rmax[k].y : rmin[k].y, (c & 4) ? rmax[k].z : rmin[k].z);
                        Vec3 p = Transform(bt[k], corner);
                        bmin = Min(bmin, p);
                        bmax = Max(bmax, p);
                    }
                }
            }
            for (auto* s : bodyInsts) {
                if (!s) continue;
                for (auto& v : s->gverts) {
                    Vec3 pos;
                    for (auto& w : v.weights) pos += Transform(pt[w.bone], v.pos) * w.weight;
                    bmin = Min(bmin, pos);
                    bmax = Max(bmax, pos);
                }
            }
        }
        if (bmin.x > bmax.x) bmin = bmax = Vec3();
        a.bmin = bmin;
        a.bmax = bmax;
    }
}

void BuildContext::BuildSequences() {
    // nodes for transitions
    std::vector<std::string> nodes;
    auto node = [&](const std::string& n) -> int {
        if (n.empty()) return 0;
        for (size_t i = 0; i < nodes.size(); i++)
            if (IEquals(nodes[i], n)) return static_cast<int>(i) + 1;
        nodes.push_back(n);
        return static_cast<int>(nodes.size());
    };
    auto animIndex = [&](const std::string& n) -> int {
        for (size_t i = 0; i < anims.size(); i++)
            if (IEquals(anims[i].name, n)) return static_cast<int>(i);
        return -1;
    };
    for (auto& qs : qc.sequences) {
        if (qs.isOverride) continue;
        CSequence s;
        s.label = qs.name;
        s.activityName = qs.activity;
        s.actWeight = qs.actWeight;
        // Root motion is an animation property, and noforceloop only steers compilation;
        // retail sequences never carry either bit (0x8000 changes the pose in game).
        s.flags = qs.flags & ~(STUDIO_FRAMEMOVEMENT | STUDIO_NOFORCELOOP);
        s.groupSize[0] = qs.groupSize[0];
        s.groupSize[1] = qs.groupSize[1];
        s.fadeIn = qs.fadeIn;
        s.fadeOut = qs.fadeOut;
        s.entryNode = node(qs.entryNode);
        s.exitNode = node(qs.exitNode);
        s.nodeFlags = qs.nodeFlags;
        s.entryPhase = qs.entryPhase;
        s.exitPhase = qs.exitPhase;
        s.keyValues = qs.keyValues;
        s.ikResetMask = qs.ikResetMask;
        s.unkC4 = qs.unkC4;
        for (auto& am : qs.activityModifiers) s.activityModifiers.push_back({am.name, am.negate});
        if (!qs.cyclePose.empty()) s.cyclePose = qc.FindPoseParam(qs.cyclePose);
        // anim index grid: offset = k * groupsize[0] + j for blend (j, k)
        s.animIndices.assign(s.groupSize[0] * s.groupSize[1], 0);
        for (int i = 0; i < static_cast<int>(qs.animations.size()); i++) {
            int a = animIndex(qs.animations[i]);
            if (a < 0) Fatal("sequence %s: missing animation %s", qs.name.c_str(), qs.animations[i].c_str());
            int j = i % s.groupSize[0], k = i / s.groupSize[0];
            s.animIndices[k * s.groupSize[0] + j] = static_cast<int16_t>(a);
            if (anims[a].flags & STUDIO_HAS_SCALE) s.flags |= STUDIO_HAS_SCALE;  // as retail sequences do
        }
        const WAnim& first = anims[s.animIndices[0]];
        // blends / pose keys
        for (int p = 0; p < 2; p++) {
            if (!qs.paramName[p].empty()) {
                s.paramIndex[p] = qc.FindPoseParam(qs.paramName[p]);
                s.paramStart[p] = qs.paramStart[p];
                s.paramEnd[p] = qs.paramEnd[p];
            }
        }
        if (!qs.poseKeys.empty()) {
            if (static_cast<int>(qs.poseKeys.size()) != s.groupSize[0] + s.groupSize[1])
                Fatal("sequence %s: posekeys needs %d values", qs.name.c_str(), s.groupSize[0] + s.groupSize[1]);
            s.poseKeys = qs.poseKeys;
        } else if (s.groupSize[0] > 1 || s.groupSize[1] > 1) {
            for (int p = 0; p < 2; p++)
                for (int m = 0; m < s.groupSize[p]; m++) {
                    float f = s.groupSize[p] > 1 ? static_cast<float>(m) / (s.groupSize[p] - 1) : 0.f;
                    s.poseKeys.push_back(s.paramStart[p] * (1 - f) + s.paramEnd[p] * f);
                }
        }
        // events
        for (auto& e : qs.events) {
            CEvent ce;
            int k = first.numFrames - 1;
            if (e.frame <= k)
                // studiomdl divides by numframes - 1: a one-frame clip's events get 0/0
                // (x86's negative quiet NaN), as retail stores them
                ce.cycle = k > 0 ? e.frame / static_cast<float>(k) : -std::numeric_limits<float>::quiet_NaN();
            else
                Fatal("event %s (frame %d) out of range in %s", e.name.c_str(), e.frame, qs.name.c_str());
            if (!e.name.empty() && std::isdigit(static_cast<unsigned char>(e.name[0]))) {
                ce.event = std::atoi(e.name.c_str());
                ce.type = 0;
            } else {
                ce.name = e.name;
                ce.type = 1 << 10;  // NEW_EVENT_STYLE
            }
            ce.options = e.options;
            s.events.push_back(ce);
        }
        // autolayers
        for (auto& l : qs.layers) {
            CAutoLayer al;
            int si = qc.FindSequence(l.sequence);
            if (si < 0) Fatal("sequence \"%s\" cannot find autolayer sequence \"%s\"", qs.name.c_str(), l.sequence.c_str());
            al.sequence = si;
            al.flags = l.flags;
            al.pose = l.pose.empty() ? 0 : qc.FindPoseParam(l.pose);
            float d = std::max(1, first.numFrames - 1);
            if (!(l.flags & STUDIO_AL_POSE)) {
                al.start = l.start / d;
                al.peak = l.peak / d;
                al.tail = l.tail / d;
                al.end = l.end / d;
            } else {
                al.start = l.start;
                al.peak = l.peak;
                al.tail = l.tail;
                al.end = l.end;
            }
            s.layers.push_back(al);
        }
        for (auto& l : qs.ikLocks) {
            int c = qc.FindIKChain(l.chain);
            if (c < 0) Fatal("unknown chain '%s' in sequence iklock", l.chain.c_str());
            s.ikLocks.push_back({c, l.posWeight, l.localQWeight, 0});
        }
        // bone weights: the sequence's own list ("seqweights"), else the max over blends
        s.weights.assign(bones.size(), 0.f);
        if (!qs.seqWeights.empty()) {
            auto it = weightLists.find(ToLower(qs.seqWeights));
            if (it == weightLists.end()) Fatal("sequence %s: unknown weightlist '%s'", qs.name.c_str(), qs.seqWeights.c_str());
            s.weights = it->second;
        } else {
            for (int16_t ai : s.animIndices)
                for (size_t n = 0; n < bones.size(); n++) s.weights[n] = std::max(s.weights[n], anims[ai].weight[n]);
        }
        // bounds: union over blends
        Vec3 bmin(9999, 9999, 9999), bmax(-9999, -9999, -9999);
        for (int16_t ai : s.animIndices) {
            bmin = Min(bmin, anims[ai].bmin);
            bmax = Max(bmax, anims[ai].bmax);
        }
        s.bmin = bmin;
        s.bmax = bmax;
        out.sequences.push_back(s);
    }
    // autolayer sequence indices refer to the QC order; remap if declared-only sequences were skipped
    out.nodeNames = nodes;
    size_t nn = nodes.size();
    out.transitions.assign(nn * nn, 0);
    for (auto& s : out.sequences) {
        if (s.entryNode != s.exitNode && s.entryNode && s.exitNode) {
            out.transitions[(s.entryNode - 1) * nn + (s.exitNode - 1)] = static_cast<uint8_t>(s.exitNode);
            if (s.nodeFlags) out.transitions[(s.exitNode - 1) * nn + (s.entryNode - 1)] = static_cast<uint8_t>(s.entryNode);
        }
    }
    // pose parameters
    for (auto& p : qc.poseParams) {
        CPoseParam cp;
        cp.name = p.name;
        cp.flags = p.flags;
        cp.start = p.min;
        cp.end = p.max;
        cp.loop = p.loop;
        out.poseParams.push_back(cp);
    }
}

void BuildContext::BuildSkins() {
    for (auto& t : qc.textures) {
        std::string n = t;
        for (auto& r : qc.renameMaterials)
            if (IEquals(n, StripExtension(r.first))) n = StripExtension(r.second);
        out.textures.push_back(n);  // as written: retail keeps both slash styles
    }
    out.numSkinRef = static_cast<int>(out.textures.size());
    if (qc.cdMaterials.empty()) {
        std::string d = "models/" + StripExtension(FixSlashes(qc.modelName));
        d = DirName(d);
        out.cdTextures.push_back(FixSlashes(d, '\\'));
    } else
        out.cdTextures = qc.cdMaterials;
    int families = 1;
    if (!qc.textureGroups.empty()) families = std::max<int>(1, static_cast<int>(qc.textureGroups[0].size()));
    out.skins.assign(families, std::vector<int16_t>(out.numSkinRef));
    for (int f = 0; f < families; f++)
        for (int j = 0; j < out.numSkinRef; j++) out.skins[f][j] = static_cast<int16_t>(j);
    if (!qc.textureGroups.empty()) {
        auto& g = qc.textureGroups[0];
        for (size_t f = 0; f < g.size(); f++)
            for (size_t j = 0; j < g[f].size() && j < g[0].size(); j++) {
                int base = qc.TextureIndex(g[0][j], false);
                int repl = qc.TextureIndex(g[f][j], false);
                if (base >= 0 && repl >= 0) out.skins[f][base] = static_cast<int16_t>(repl);
            }
        if (qc.textureGroups.size() > 1) Warn("only the first $texturegroup is used");

        // Retail lists each base texture followed by its replacements (column by column)
        // rather than all replacements after the base set; renumber to match.
        std::vector<int> remap(out.textures.size(), -1);
        int next = 0;
        for (int j = 0; j < out.numSkinRef; j++)
            for (int f = 0; f < families; f++) {
                int t = out.skins[f][j];
                if (remap[t] < 0) remap[t] = next++;
            }
        for (auto& r : remap)
            if (r < 0) r = next++;
        std::vector<std::string> textures(out.textures.size());
        for (size_t i = 0; i < remap.size(); i++) textures[remap[i]] = out.textures[i];
        out.textures = textures;
        auto skins = out.skins;
        for (int f = 0; f < families; f++)
            for (int j = 0; j < out.numSkinRef; j++) out.skins[f][remap[j]] = static_cast<int16_t>(remap[skins[f][j]]);
        for (auto& bp : out.bodyParts)
            for (auto& md : bp.models)
                for (auto& me : md.meshes) me.material = remap[me.material];
        for (auto& lod : out.lodMaterialReplacements)
            for (auto& r : lod) r.first = remap[r.first];
    }
}

void BuildContext::FinalizeBones() {
    for (size_t i = 0; i < bones.size(); i++) {
        const GBone& g = bones[i];
        CBone b;
        b.name = g.name;
        b.parent = g.parent;
        b.pos = g.pos;
        b.rot = g.rot;
        b.qAlignment = Quat();
        b.quat = AngleQuaternion(g.rot);  // retail stores it unaligned (w may be negative)
        b.rotscale = g.rotscale;
        b.posscale = Vec3();  // v53 keeps position scales in the animation records
        b.scalescale = Vec3();
        b.boneToPose = g.boneToPose;
        b.poseToBone = Invert(g.boneToPose);
        b.flags = g.flags;
        b.physicsBone = g.physicsBone;
        // surface prop / contents walk up the hierarchy
        b.surfaceProp = qc.surfaceProp;
        b.contents = qc.contents;
        for (int k = static_cast<int>(i); k != -1; k = bones[k].parent) {
            bool found = false;
            for (auto& sp : qc.jointSurfaceProps)
                if (IEquals(sp.first, bones[k].name)) {
                    b.surfaceProp = sp.second;
                    found = true;
                }
            if (found) break;
        }
        for (int k = static_cast<int>(i); k != -1; k = bones[k].parent) {
            bool found = false;
            for (auto& c : qc.jointContents)
                if (IEquals(c.first, bones[k].name)) {
                    b.contents = c.second;
                    found = true;
                }
            if (found) break;
        }
        for (auto& sa : qc.screenAlign)
            if (IEquals(sa.first, g.name)) b.flags |= sa.second;
        out.bones.push_back(b);
    }
    for (auto& jb : qc.jiggleBones) {
        int k = FindBone(jb.bone);
        if (k < 0) {
            Warn("jigglebone %s: bone not in model, ignored", jb.bone.c_str());
            continue;
        }
        CJiggle j;
        j.flags = jb.flags;
        std::copy(std::begin(jb.v), std::end(jb.v), j.v);
        out.bones[k].proctype = 5;  // STUDIO_PROC_JIGGLE
        out.bones[k].jiggle = static_cast<int>(out.jiggles.size());
        out.bones[k].flags |= BONE_ALWAYS_PROCEDURAL;
        out.jiggles.push_back(j);
    }
    // other procedural bones (studiomdl TagProceduralBones / RemapProceduralBones / WriteBoneInfo)
    auto procBone = [&](const std::string& name, const char* what) -> int {
        int k = FindBone(name);
        if (k < 0) Verbose("%s \"%s\" unused", what, name.c_str());
        return k;
    };
    auto needBone = [&](const std::string& name, const std::string& owner) -> int {
        int k = FindBone(name);
        if (k < 0) Fatal("missing control bone \"%s\" for procedural bone \"%s\"", name.c_str(), owner.c_str());
        return k;
    };
    auto quatOf = [](const Mat3x4& m) { return AngleQuaternion(MatrixAngles(m)); };
    auto markChain = [&](int from, int flags) {
        for (int k = from; k != -1; k = bones[k].parent) out.bones[k].flags |= flags;
    };
    for (auto& ax : qc.axisInterpBones) {
        int k = procBone(ax.bone, "axisinterpbone");
        if (k < 0) continue;
        int control = needBone(ax.control, ax.bone);
        Buffer b;
        b.I32(control);
        b.I32(ax.axis);
        for (auto& p : ax.pos) b.V3(p);
        for (auto& q : ax.quat) b.Q(q);
        out.bones[k].proctype = 1;  // STUDIO_PROC_AXISINTERP
        out.bones[k].procData = b.Data();
        out.bones[k].flags |= BONE_ALWAYS_PROCEDURAL;
    }
    for (auto& qi : qc.quatInterpBones) {
        int k = procBone(qi.bone, "quatinterpbone");
        if (k < 0) continue;
        int control = needBone(qi.control, qi.bone);
        Buffer b;
        b.I32(control);
        b.I32(static_cast<int>(qi.triggers.size()));
        b.I32(12);  // triggers follow the struct
        for (auto& t : qi.triggers) {
            Quat trigger = quatOf(Concat(QuaternionMatrix(t.trigger), bones[control].srcRealign));
            Vec3 tmp = t.pos + bones[control].pos * qi.percentage;
            Mat3x4 dest = Concat(QuaternionMatrix(t.quat, tmp), bones[k].srcRealign);
            b.F32(t.tolerance > 0 ? 1.f / t.tolerance : 0.f);
            b.Q(trigger);
            b.V3(dest.Origin());
            b.Q(quatOf(dest));
        }
        out.bones[k].proctype = 2;  // STUDIO_PROC_QUATINTERP
        out.bones[k].procData = b.Data();
        out.bones[k].flags |= BONE_ALWAYS_PROCEDURAL;
        // MarkProceduralBoneChain: the control chain is used wherever the bone is
        int used = out.bones[k].flags & BONE_USED_BY_ANYTHING;
        markChain(control, used);
        markChain(k, used);
    }
    for (auto& am : qc.aimAtBones) {
        int k = procBone(am.bone, "<aimconstraint>");
        if (k < 0) continue;
        int parent = needBone(am.parent, am.bone);
        int attach = -1;
        for (size_t i = 0; i < out.attachments.size(); i++)
            if (out.attachments[i].name == am.aim) attach = static_cast<int>(i);
        int aim = attach >= 0 ? attach : FindBone(am.aim);
        if (aim < 0) Fatal("missing aim control attachment or bone \"%s\" for procedural bone \"%s\"", am.aim.c_str(), am.bone.c_str());
        Buffer b;
        b.I32(parent);
        b.I32(aim);
        b.V3(am.aimVector);
        b.V3(am.upVector);
        b.V3(am.basePos);
        out.bones[k].proctype = attach >= 0 ? 4 : 3;  // STUDIO_PROC_AIMATATTACH / AIMATBONE
        out.bones[k].procData = b.Data();
        out.bones[k].flags |= BONE_ALWAYS_PROCEDURAL;
        int used = out.bones[k].flags & BONE_USED_BY_ANYTHING;
        markChain(parent, used);
        if (attach < 0) markChain(aim, used);
    }
    // RUI panel meshes
    for (auto& qp : qc.uiPanels) {
        CUIPanel p;
        p.name = qp.name;
        p.hash = qp.hasHash ? qp.hash : UIPanelHash(qp.name);
        p.ccw = qp.ccw;
        p.bounds = qp.bounds;
        for (auto& name : qp.parents) p.parents.push_back(needBone(name, qp.name));
        for (auto& v : qp.vertices) {
            if (v.parent < 0 || v.parent >= static_cast<int>(p.parents.size()))
                Fatal("$uipanel %s: vertex parent slot %d out of range", qp.name.c_str(), v.parent);
            p.vertices.push_back({v.parent, v.pos});
        }
        for (auto& f : qp.faces) {
            for (int i : f)
                if (i < 0 || i >= static_cast<int>(p.vertices.size()))
                    Fatal("$uipanel %s: face vertex %d out of range", qp.name.c_str(), i);
            p.faces.push_back({static_cast<int16_t>(f[0]), static_cast<int16_t>(f[1]), static_cast<int16_t>(f[2])});
        }
        out.uiPanels.push_back(std::move(p));
    }
    for (auto& bf : qc.boneFollowers) {
        int k = FindBone(bf);
        if (k < 0)
            Warn("bone follower %s: bone not found", bf.c_str());
        else
            out.boneFollowers.push_back(k);
    }
}

CompiledModel Build(QcModelDef& qc) {
    BuildContext ctx(qc);
    if (qc.modelName.empty()) Fatal("$modelname is missing");
    if (qc.sequences.empty() && qc.includeModels.empty()) {
        // studiomdl requires a sequence; give static meshes an implicit idle from the first body
        for (auto& bp : qc.bodyParts)
            for (auto& sm : bp.models)
                if (!sm.blank && sm.source && qc.sequences.empty()) {
                    auto a = std::make_shared<QcAnimation>();
                    a->name = "@idle";
                    a->implied = true;
                    a->file = sm.file;
                    a->source = sm.source;
                    a->sourceScale = sm.scale;
                    a->adjust = qc.defaultAdjust;
                    a->rotation = qc.defaultRotation;
                    a->startFrame = a->endFrame = sm.source->startFrame;
                    a->frameRangeSet = true;
                    qc.animations.push_back(a);
                    QcSequence s;
                    s.name = "idle";
                    s.animations.push_back(a->name);
                    s.groupSize[0] = s.groupSize[1] = 1;
                    qc.sequences.push_back(s);
                    Info("no $sequence given; added \"idle\" from %s", sm.file.c_str());
                }
        if (qc.sequences.empty()) Fatal("model has no sequences");
    }
    ctx.LoadInstances();
    if (qc.staticProp) ctx.MakeStaticProp();
    ctx.BuildBones();
    ctx.RemapVertices();
    ctx.BuildMeshes();
    ctx.BuildAnimations();
    ctx.BuildAttachments();
    ctx.BuildIK();
    ctx.BuildHitboxes();
    ctx.BuildBounds();
    ctx.BuildSequences();
    ProcessIKRules(ctx);
    EncodeAnimations(ctx);
    ctx.BuildSkins();
    ctx.FinalizeBones();

    CompiledModel& out = ctx.out;
    // a static prop's single bone is marked used by everything, as retail's are
    if (qc.staticProp)
        for (auto& b : out.bones) b.flags |= BONE_USED_BY_ANYTHING;
    out.name = qc.modelName;  // as written (retail names keep their slashes)
    if (!EndsWithI(out.name, ".mdl")) out.name += ".mdl";
    out.surfaceProp = qc.surfaceProp;
    out.contents = qc.contents;
    out.keyValues = qc.keyValues;
    for (auto& t : qc.srcBoneTransforms) {
        CompiledModel::SrcBoneTransform c;
        c.bone = t.bone;
        std::copy(std::begin(t.pre), std::end(t.pre), c.pre);
        std::copy(std::begin(t.post), std::end(t.post), c.post);
        out.srcBoneTransforms.push_back(c);
    }
    out.fadeDistance = qc.fadeDistance;
    out.constDirectionalLightDot = qc.constDirectionalLightDot;
    out.maxEyeDeflection = qc.maxEyeDeflection;
    out.unkString = qc.unkString;
    out.includeModels = qc.includeModels;
    out.eyePosition = qc.eyePosition;
    out.numAllowedRootLods = static_cast<uint8_t>(qc.allowRootLods);
    if (qc.illumSet) {
        out.illumPosition = qc.illumPosition;
        if (!qc.illumBone.empty()) {
            int k = ctx.FindBone(qc.illumBone);
            if (k >= 0) out.illumPosition = Transform(ctx.bones[k].boneToPose, qc.illumPosition);
        }
    } else if (!out.sequences.empty()) {
        // retail: centred horizontally, three quarters of (min + max) vertically
        const Vec3 &mn = out.sequences[0].bmin, &mx = out.sequences[0].bmax;
        out.illumPosition = (mn + mx) * 0.5f;
        out.illumPosition.z = (mn.z + mx.z) * 0.75f;
    }
    // collision may grow the bounds
    BuildCollisionModel(ctx);
    if (!qc.collision.present && qc.headerMass >= 0) out.physics.mass = qc.headerMass;
    if (qc.bboxSet) {
        out.hullMin = qc.bbox[0];
        out.hullMax = qc.bbox[1];
    } else if (!out.sequences.empty()) {
        out.hullMin = out.sequences[0].bmin;
        out.hullMax = out.sequences[0].bmax;
    }
    if (qc.cboxSet) {
        out.viewMin = qc.cbox[0];
        out.viewMax = qc.cbox[1];
    }
    if (out.hasVertexColor) qc.flags |= STUDIOHDR_FLAGS_USES_VERTEX_COLOR;
    if (out.hasUV2) qc.flags |= STUDIOHDR_FLAGS_USES_UV2;
    // retail sets this on every single-bone model, physics or not, unless its collision
    // is per joint or has no map-collision block
    if (out.bones.size() == 1 && !(qc.collision.present && (qc.collision.joints || qc.collision.noMapCollision)))
        qc.flags |= STUDIOHDR_FLAGS_RESPAWN_800000;
    out.flags = qc.flags;
    if (qc.staticProp) out.flags |= STUDIOHDR_FLAGS_STATIC_PROP;
    BuildPerTriAABB(ctx);
    if (out.unkString.empty()) out.unkString = "";
    return std::move(ctx.out);
}

}  // namespace r2c
