// Titanfall 2 (v53) studiohdr_t writer. Section order follows retail models:
// header, source files, bones, jiggle bones, attachments, hitbox sets, bone name
// table, animations, sequences, nodes, bodyparts, IK chains, pose parameters, IK
// locks, include models, textures, skins, keyvalues, linear bones, per-tri AABB,
// strings, then the embedded PHY, collision, VTX, VVD and VVC.
#include <algorithm>
#include <cstring>
#include <numeric>

#include "core/buffer.h"
#include "core/log.h"
#include "core/strings.h"
#include "write/writers.h"

namespace r2c {

namespace {

// studiohdr_t field offsets (v53, 0x2CC bytes)
enum HdrOff : size_t {
    H_ID = 0x00, H_VERSION = 0x04, H_CHECKSUM = 0x08, H_SZNAME = 0x0C, H_NAME = 0x10, H_LENGTH = 0x50,
    H_EYEPOS = 0x54, H_ILLUMPOS = 0x60, H_HULLMIN = 0x6C, H_HULLMAX = 0x78, H_VIEWMIN = 0x84, H_VIEWMAX = 0x90,
    H_FLAGS = 0x9C, H_NUMBONES = 0xA0, H_BONEINDEX = 0xA4, H_NUMBONECONTROLLERS = 0xA8, H_BONECONTROLLERINDEX = 0xAC,
    H_NUMHITBOXSETS = 0xB0, H_HITBOXSETINDEX = 0xB4, H_NUMLOCALANIM = 0xB8, H_LOCALANIMINDEX = 0xBC,
    H_NUMLOCALSEQ = 0xC0, H_LOCALSEQINDEX = 0xC4, H_ACTIVITYLISTVERSION = 0xC8, H_EVENTSINDEXED = 0xCC,
    H_NUMTEXTURES = 0xD0, H_TEXTUREINDEX = 0xD4, H_NUMCDTEXTURES = 0xD8, H_CDTEXTUREINDEX = 0xDC,
    H_NUMSKINREF = 0xE0, H_NUMSKINFAMILIES = 0xE4, H_SKININDEX = 0xE8, H_NUMBODYPARTS = 0xEC, H_BODYPARTINDEX = 0xF0,
    H_NUMLOCALATTACHMENTS = 0xF4, H_LOCALATTACHMENTINDEX = 0xF8, H_NUMLOCALNODES = 0xFC, H_LOCALNODEINDEX = 0x100,
    H_LOCALNODENAMEINDEX = 0x104, H_LOCALNODEUNK = 0x108, H_FLEXDESCINDEX = 0x10C, H_NUMFLEXCONTROLLERS = 0x110,
    H_FLEXCONTROLLERINDEX = 0x114, H_NUMFLEXRULES = 0x118, H_FLEXRULEINDEX = 0x11C, H_NUMIKCHAINS = 0x120,
    H_IKCHAININDEX = 0x124, H_UIPANELCOUNT = 0x128, H_UIPANELOFFSET = 0x12C, H_NUMLOCALPOSEPARAMETERS = 0x130,
    H_LOCALPOSEPARAMINDEX = 0x134, H_SURFACEPROPINDEX = 0x138, H_KEYVALUEINDEX = 0x13C, H_KEYVALUESIZE = 0x140,
    H_NUMLOCALIKAUTOPLAYLOCKS = 0x144, H_LOCALIKAUTOPLAYLOCKINDEX = 0x148, H_MASS = 0x14C, H_CONTENTS = 0x150,
    H_NUMINCLUDEMODELS = 0x154, H_INCLUDEMODELINDEX = 0x158, H_VIRTUALMODEL = 0x15C, H_BONETABLEBYNAMEINDEX = 0x160,
    H_CONSTDIRLIGHTDOT = 0x164, H_ROOTLOD = 0x165, H_NUMALLOWEDROOTLODS = 0x166, H_FADEDISTANCE = 0x168,
    H_NUMFLEXCONTROLLERUI = 0x16C, H_FLEXCONTROLLERUIINDEX = 0x170, H_VERTANIMSCALE = 0x174, H_SURFACEPROPLOOKUP = 0x178,
    H_SOURCEFILENAMEOFFSET = 0x17C, H_NUMSRCBONETRANSFORM = 0x180, H_SRCBONETRANSFORMINDEX = 0x184,
    H_ILLUMPOSATTACHMENTINDEX = 0x188, H_LINEARBONEINDEX = 0x18C, H_BONEFLEXDRIVERCOUNT = 0x190,
    H_BONEFLEXDRIVERINDEX = 0x194, H_PERTRIINDEX = 0x198, H_PERTRINODECOUNT = 0x19C, H_PERTRILEAFCOUNT = 0x1A0,
    H_PERTRIVERTCOUNT = 0x1A4, H_UNKSTRINGOFFSET = 0x1A8, H_VTXOFFSET = 0x1AC, H_VVDOFFSET = 0x1B0, H_VVCOFFSET = 0x1B4,
    H_PHYOFFSET = 0x1B8, H_VTXSIZE = 0x1BC, H_VVDSIZE = 0x1C0, H_VVCSIZE = 0x1C4, H_PHYSIZE = 0x1C8,
    H_COLLISIONOFFSET = 0x1CC, H_STATICCOLLISIONCOUNT = 0x1D0, H_BONEFOLLOWERCOUNT = 0x1D4, H_BONEFOLLOWEROFFSET = 0x1D8,
    H_SIZE = 0x2CC,
};

constexpr size_t kBoneSize = 0xF4, kJiggleSize = 0x78, kAttachSize = 0x5C, kHboxSetSize = 0x0C, kHboxSize = 0x44,
                 kAnimDescSize = 0x5C, kSeqDescSize = 0xE8, kEventSize = 0x50, kAutoLayerSize = 0x18, kIKLockSize = 0x20,
                 kBodyPartSize = 0x10, kModelSize = 0x94, kMeshSize = 0x74, kIKChainSize = 0x20, kIKLinkSize = 0x1C,
                 kPoseParamSize = 0x14, kModelGroupSize = 0x08, kTextureSize = 0x2C, kMovementSize = 0x2C,
                 kIKRuleSize = 0x98;

class MdlWriter {
public:
    explicit MdlWriter(const CompiledModel& m) : m_(m) {}
    std::vector<uint8_t> Run();

private:
    const CompiledModel& m_;
    Buffer b_;
    StringTable st_;

    void Str(size_t field, size_t base, const std::string& s) { st_.Add(field, base, s); }
    void Hdr(size_t off, int32_t v) { b_.PatchI32(off, v); }
    void HdrHere(size_t off) { b_.PatchI32(off, static_cast<int32_t>(b_.Tell())); }
    int32_t Rel(size_t from) const { return static_cast<int32_t>(b_.Tell() - from); }

    void WriteHeader();
    void WriteSourceFiles();
    void WriteBones();
    void WriteAttachments();
    void WriteHitboxes();
    void WriteBoneTable();
    void WriteAnimations();
    void WriteSequences();
    void WriteNodes();
    void WriteBodyParts();
    void WriteIK();
    void WritePoseParams();
    void WriteIncludeModels();
    void WriteTextures();
    void WriteKeyValues();
    void WriteLinearBones();
    void WritePerTri();
};

void MdlWriter::WriteHeader() {
    b_.Reserve(H_SIZE);
    b_.PatchAt<uint32_t>(H_ID, 0x54534449);  // IDST
    Hdr(H_VERSION, 53);
    Str(H_SZNAME, 0, m_.name);
    std::string n = m_.name;
    if (n.size() > 63) n = n.substr(0, 63);
    std::memcpy(&b_.Data()[H_NAME], n.data(), n.size());
    auto v3 = [&](size_t off, const Vec3& v) {
        b_.PatchAt<float>(off, v.x);
        b_.PatchAt<float>(off + 4, v.y);
        b_.PatchAt<float>(off + 8, v.z);
    };
    v3(H_EYEPOS, m_.eyePosition);
    v3(H_ILLUMPOS, m_.illumPosition);
    v3(H_HULLMIN, m_.hullMin);
    v3(H_HULLMAX, m_.hullMax);
    v3(H_VIEWMIN, m_.viewMin);
    v3(H_VIEWMAX, m_.viewMax);
    Hdr(H_FLAGS, m_.flags);
    b_.PatchAt<float>(H_MASS, m_.physics.mass);
    Hdr(H_CONTENTS, m_.contents);
    b_.PatchAt<uint8_t>(H_CONSTDIRLIGHTDOT, m_.constDirectionalLightDot);
    b_.PatchAt<uint8_t>(H_ROOTLOD, m_.rootLod);
    b_.PatchAt<uint8_t>(H_NUMALLOWEDROOTLODS, m_.numAllowedRootLods);
    b_.PatchAt<float>(H_FADEDISTANCE, m_.fadeDistance);
    Str(H_SURFACEPROPINDEX, 0, m_.surfaceProp);
    Str(H_UNKSTRINGOFFSET, 0, m_.unkString);
    Hdr(H_ILLUMPOSATTACHMENTINDEX, m_.illumPositionAttachment);
}

void MdlWriter::WriteSourceFiles() {
    HdrHere(H_SOURCEFILENAMEOFFSET);
    std::string s;
    for (auto& f : m_.sourceFiles) s += f + ";";
    b_.CString(s);
    b_.Zero(3);
    b_.Align(4);
}

void MdlWriter::WriteBones() {
    HdrHere(H_BONEINDEX);
    Hdr(H_NUMBONES, static_cast<int>(m_.bones.size()));
    size_t base = b_.Reserve(kBoneSize * m_.bones.size());
    for (size_t i = 0; i < m_.bones.size(); i++) {
        const CBone& bn = m_.bones[i];
        size_t p = base + i * kBoneSize;
        Str(p + 0x00, p, bn.name);
        b_.PatchI32(p + 0x04, bn.parent);
        for (int c = 0; c < 6; c++) b_.PatchI32(p + 0x08 + c * 4, -1);
        auto f = [&](size_t off, float v) { b_.PatchAt<float>(p + off, v); };
        f(0x20, bn.pos.x); f(0x24, bn.pos.y); f(0x28, bn.pos.z);
        f(0x2C, bn.quat.x); f(0x30, bn.quat.y); f(0x34, bn.quat.z); f(0x38, bn.quat.w);
        f(0x3C, bn.rot.x); f(0x40, bn.rot.y); f(0x44, bn.rot.z);
        f(0x48, bn.scale.x); f(0x4C, bn.scale.y); f(0x50, bn.scale.z);
        f(0x54, bn.posscale.x); f(0x58, bn.posscale.y); f(0x5C, bn.posscale.z);
        f(0x60, bn.rotscale.x); f(0x64, bn.rotscale.y); f(0x68, bn.rotscale.z);
        f(0x6C, bn.scalescale.x); f(0x70, bn.scalescale.y); f(0x74, bn.scalescale.z);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 4; c++) f(0x78 + (r * 4 + c) * 4, bn.poseToBone.m[r][c]);
        f(0xA8, bn.qAlignment.x); f(0xAC, bn.qAlignment.y); f(0xB0, bn.qAlignment.z); f(0xB4, bn.qAlignment.w);
        b_.PatchI32(p + 0xB8, bn.flags);
        b_.PatchI32(p + 0xBC, bn.proctype);
        b_.PatchI32(p + 0xC0, 0);  // procindex, patched below
        b_.PatchI32(p + 0xC4, bn.physicsBone);
        Str(p + 0xC8, p, bn.surfaceProp);
        b_.PatchI32(p + 0xCC, bn.contents);
        b_.PatchI32(p + 0xD0, 0);
        b_.PatchAt<uint16_t>(p + 0xD4, static_cast<uint16_t>(bn.collisionIndex < 0 ? 0xFFFF : bn.collisionIndex));
        b_.PatchAt<uint16_t>(p + 0xD6, static_cast<uint16_t>(bn.collisionCount));
    }
    b_.Align(4);
    for (size_t i = 0; i < m_.bones.size(); i++) {
        const CBone& bn = m_.bones[i];
        if (bn.jiggle < 0) continue;
        size_t p = base + i * kBoneSize;
        b_.PatchI32(p + 0xC0, Rel(p));
        const CJiggle& j = m_.jiggles[bn.jiggle];
        b_.I32(j.flags);
        for (float v : j.v) b_.F32(v);
    }
    b_.Align(4);
    HdrHere(H_BONECONTROLLERINDEX);
}

void MdlWriter::WriteAttachments() {
    HdrHere(H_LOCALATTACHMENTINDEX);
    Hdr(H_NUMLOCALATTACHMENTS, static_cast<int>(m_.attachments.size()));
    for (auto& a : m_.attachments) {
        size_t p = b_.Tell();
        b_.I32(0);
        Str(p, p, a.name);
        b_.U32(a.flags);
        b_.I32(a.bone);
        b_.M34(a.local);
        b_.Zero(32);
    }
}

void MdlWriter::WriteHitboxes() {
    HdrHere(H_HITBOXSETINDEX);
    Hdr(H_NUMHITBOXSETS, static_cast<int>(m_.hitboxSets.size()));
    size_t base = b_.Reserve(kHboxSetSize * m_.hitboxSets.size());
    for (size_t s = 0; s < m_.hitboxSets.size(); s++) {
        auto& set = m_.hitboxSets[s];
        size_t p = base + s * kHboxSetSize;
        Str(p, p, set.name);
        b_.PatchI32(p + 4, static_cast<int>(set.boxes.size()));
        b_.PatchI32(p + 8, Rel(p));
        for (auto& h : set.boxes) {
            size_t q = b_.Tell();
            b_.I32(h.bone);
            b_.I32(h.group);
            b_.V3(h.bmin);
            b_.V3(h.bmax);
            b_.I32(0);
            Str(q + 0x20, q, h.name);
            b_.I32(h.critOverride);
            b_.I32(0);
            Str(q + 0x28, q, h.hitDataGroup);
            b_.Zero(24);
        }
    }
}

void MdlWriter::WriteBoneTable() {
    HdrHere(H_BONETABLEBYNAMEINDEX);
    std::vector<int> order(m_.bones.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return ToLower(m_.bones[a].name) < ToLower(m_.bones[b].name);
    });
    for (int i : order) b_.U8(static_cast<uint8_t>(i));
    b_.Align(4);
}

void MdlWriter::WriteAnimations() {
    HdrHere(H_LOCALANIMINDEX);
    Hdr(H_NUMLOCALANIM, static_cast<int>(m_.animations.size()));
    size_t base = b_.Reserve(kAnimDescSize * m_.animations.size());
    for (size_t i = 0; i < m_.animations.size(); i++) {
        const CAnimation& a = m_.animations[i];
        size_t p = base + i * kAnimDescSize;
        b_.PatchI32(p + 0x00, -static_cast<int32_t>(p));
        Str(p + 0x04, p, a.name);
        b_.PatchAt<float>(p + 0x08, a.fps);
        b_.PatchI32(p + 0x0C, a.flags);
        b_.PatchI32(p + 0x10, a.numFrames);
        if (a.sectionFrames == 0) {
            b_.Align(16);
            b_.PatchI32(p + 0x20, Rel(p));
            b_.Append(a.sections[0]);
        } else {
            b_.PatchI32(p + 0x34, Rel(p));
            b_.PatchI32(p + 0x38, a.sectionFrames);
            size_t secTable = b_.Reserve(4 * a.sections.size());
            for (size_t s = 0; s < a.sections.size(); s++) {
                b_.Align(16);
                b_.PatchI32(secTable + s * 4, Rel(p));
                b_.Append(a.sections[s]);
            }
            b_.PatchI32(p + 0x20, b_.ReadAt<int32_t>(secTable));
        }
        if (!a.movements.empty()) {
            b_.Align(4);
            b_.PatchI32(p + 0x14, static_cast<int>(a.movements.size()));
            b_.PatchI32(p + 0x18, Rel(p));
            for (auto& mv : a.movements) {
                b_.I32(mv.endFrame);
                b_.I32(mv.motionFlags);
                b_.F32(mv.v0);
                b_.F32(mv.v1);
                b_.F32(mv.angle);
                b_.V3(mv.vector);
                b_.V3(mv.position);
            }
        }
        if (!a.frameMovement.empty()) {
            b_.Align(4);
            b_.PatchI32(p + 0x1C, Rel(p));
            b_.Append(a.frameMovement);
        }
        b_.Align(4);
    }
}

void MdlWriter::WriteSequences() {
    HdrHere(H_LOCALSEQINDEX);
    Hdr(H_NUMLOCALSEQ, static_cast<int>(m_.sequences.size()));
    size_t base = b_.Reserve(kSeqDescSize * m_.sequences.size());
    size_t lastWeights = SIZE_MAX;
    const std::vector<float>* lastWeightVals = nullptr;
    for (size_t i = 0; i < m_.sequences.size(); i++) {
        const CSequence& s = m_.sequences[i];
        size_t p = base + i * kSeqDescSize;
        auto I = [&](size_t off, int32_t v) { b_.PatchI32(p + off, v); };
        auto F = [&](size_t off, float v) { b_.PatchAt<float>(p + off, v); };
        I(0x00, -static_cast<int32_t>(p));
        Str(p + 0x04, p, s.label);
        Str(p + 0x08, p, s.activityName);
        int flags = s.flags;
        bool single = true;
        for (int16_t ai : s.animIndices)
            if (m_.animations[ai].numFrames > 1) single = false;
        if (single) flags |= 0x80000;  // STUDIO_SINGLEFRAME, as retail sets it
        I(0x0C, flags);
        I(0x10, -1);  // activity, resolved at load time
        I(0x14, s.actWeight);
        F(0x20, s.bmin.x); F(0x24, s.bmin.y); F(0x28, s.bmin.z);
        F(0x2C, s.bmax.x); F(0x30, s.bmax.y); F(0x34, s.bmax.z);
        I(0x38, static_cast<int>(s.animIndices.size()));
        I(0x44, s.groupSize[0]);
        I(0x48, s.groupSize[1]);
        I(0x4C, s.paramIndex[0]);
        I(0x50, s.paramIndex[1]);
        F(0x54, s.paramStart[0]); F(0x58, s.paramStart[1]);
        F(0x5C, s.paramEnd[0]); F(0x60, s.paramEnd[1]);
        F(0x68, s.fadeIn);
        F(0x6C, s.fadeOut);
        I(0x70, s.entryNode);
        I(0x74, s.exitNode);
        I(0x78, s.nodeFlags);
        F(0x7C, s.entryPhase);
        F(0x80, s.exitPhase);
        F(0x84, s.lastFrame);
        I(0x88, s.nextSeq);
        I(0x8C, s.pose);
        I(0x90, s.numIKRules);
        I(0xB4, s.cyclePose);
        I(0xC0, s.ikResetMask);
        // pose keys (blends only)
        if (!s.poseKeys.empty()) {
            I(0xA0, Rel(p));
            for (float k : s.poseKeys) b_.F32(k);
        }
        // events
        I(0x18, static_cast<int>(s.events.size()));
        I(0x1C, Rel(p));
        for (auto& e : s.events) {
            size_t q = b_.Tell();
            b_.F32(e.cycle);
            b_.I32(e.event);
            b_.I32(e.type);
            b_.FixedString(e.options, 64);
            b_.I32(0);
            if (e.type != 0) Str(q + 0x4C, q, e.name);
        }
        b_.Align(4);
        // autolayers
        I(0x94, static_cast<int>(s.layers.size()));
        I(0x98, Rel(p));
        for (auto& l : s.layers) {
            b_.I16(static_cast<int16_t>(l.sequence));
            b_.I16(static_cast<int16_t>(l.pose));
            b_.I32(l.flags);
            b_.F32(l.start);
            b_.F32(l.peak);
            b_.F32(l.tail);
            b_.F32(l.end);
        }
        // bone weights (shared with the previous sequence when identical)
        if (lastWeightVals && *lastWeightVals == s.weights) {
            I(0x9C, static_cast<int32_t>(lastWeights - p));
        } else {
            I(0x9C, Rel(p));
            lastWeights = b_.Tell();
            lastWeightVals = &s.weights;
            for (float w : s.weights) b_.F32(w);
        }
        // ik locks
        I(0xA4, static_cast<int>(s.ikLocks.size()));
        I(0xA8, Rel(p));
        for (auto& l : s.ikLocks) {
            b_.I32(l.chain);
            b_.F32(l.posWeight);
            b_.F32(l.localQWeight);
            b_.I32(l.flags);
            b_.Zero(16);
        }
        b_.Align(4);
        // anim indices
        I(0x3C, Rel(p));
        for (int16_t ai : s.animIndices) b_.I16(ai);
        b_.Align(4);
        // keyvalues
        I(0xAC, Rel(p));
        if (!s.keyValues.empty()) {
            std::string kv = s.keyValues;
            b_.Bytes(kv.data(), kv.size());
            b_.U8(0);
            I(0xB0, static_cast<int>(kv.size() + 1));
            b_.Align(4);
        }
        // activity modifiers
        I(0xB8, Rel(p));
        I(0xBC, static_cast<int>(s.activityModifiers.size()));
        for (auto& am : s.activityModifiers) {
            size_t q = b_.Tell();
            b_.I32(0);
            Str(q, q, am.name);
            b_.U8(am.negate ? 1 : 0);
            b_.Zero(3);
        }
    }
}

void MdlWriter::WriteNodes() {
    HdrHere(H_LOCALNODENAMEINDEX);
    size_t names = b_.Reserve(4 * m_.nodeNames.size());
    for (size_t i = 0; i < m_.nodeNames.size(); i++) Str(names + i * 4, 0, m_.nodeNames[i]);
    HdrHere(H_LOCALNODEINDEX);
    Hdr(H_NUMLOCALNODES, static_cast<int>(m_.nodeNames.size()));
    b_.Append(m_.transitions);
    b_.Align(4);
}

void MdlWriter::WriteBodyParts() {
    HdrHere(H_BODYPARTINDEX);
    Hdr(H_NUMBODYPARTS, static_cast<int>(m_.bodyParts.size()));
    size_t base = b_.Reserve(kBodyPartSize * m_.bodyParts.size());
    for (size_t i = 0; i < m_.bodyParts.size(); i++) {
        auto& bp = m_.bodyParts[i];
        size_t p = base + i * kBodyPartSize;
        Str(p, p, bp.name);
        b_.PatchI32(p + 4, static_cast<int>(bp.models.size()));
        b_.PatchI32(p + 8, bp.base);
        b_.PatchI32(p + 12, Rel(p));
        size_t mbase = b_.Reserve(kModelSize * bp.models.size());
        for (size_t mi = 0; mi < bp.models.size(); mi++) {
            auto& md = bp.models[mi];
            size_t q = mbase + mi * kModelSize;
            std::string nm = md.name;
            if (nm.size() > 63) nm = nm.substr(nm.size() - 63);
            std::memcpy(&b_.Data()[q], nm.data(), nm.size());
            b_.PatchAt<float>(q + 0x44, md.boundingRadius);
            b_.PatchI32(q + 0x48, static_cast<int>(md.meshes.size()));
            b_.PatchI32(q + 0x50, md.NumVerts());
            b_.PatchI32(q + 0x54, md.vertexStart * 48);
            b_.PatchI32(q + 0x58, md.vertexStart * 16);
            b_.PatchI32(q + 0x7C, md.vertexStart * 4);
            b_.PatchI32(q + 0x80, md.vertexStart * 8);
        }
        for (size_t mi = 0; mi < bp.models.size(); mi++) {
            auto& md = bp.models[mi];
            size_t q = mbase + mi * kModelSize;
            b_.PatchI32(q + 0x4C, Rel(q));
            int voff = 0;
            for (auto& me : md.meshes) {
                size_t r = b_.Tell();
                b_.I32(me.material);
                b_.I32(static_cast<int32_t>(q) - static_cast<int32_t>(r));
                b_.I32(static_cast<int>(me.verts.size()));
                b_.I32(voff);
                b_.Zero(16);  // flexes, material type/param
                b_.I32(me.meshId);
                b_.V3(me.center);
                b_.I32(0);  // modelvertexdata (runtime)
                for (int l = 0; l < kMaxLods; l++) b_.I32(me.lodVertexCount[l]);
                b_.Zero(8);   // runtime pointer
                b_.Zero(24);  // unused
                voff += static_cast<int>(me.verts.size());
            }
        }
    }
    // deprecated flex data all point here, as in retail files
    HdrHere(H_FLEXDESCINDEX);
    HdrHere(H_FLEXCONTROLLERINDEX);
    HdrHere(H_FLEXRULEINDEX);
    HdrHere(H_FLEXCONTROLLERUIINDEX);
}

void MdlWriter::WriteIK() {
    HdrHere(H_IKCHAININDEX);
    Hdr(H_NUMIKCHAINS, static_cast<int>(m_.ikChains.size()));
    size_t base = b_.Reserve(kIKChainSize * m_.ikChains.size());
    for (size_t i = 0; i < m_.ikChains.size(); i++) {
        auto& c = m_.ikChains[i];
        size_t p = base + i * kIKChainSize;
        Str(p, p, c.name);
        b_.PatchI32(p + 4, c.linkType);
        b_.PatchI32(p + 8, static_cast<int>(c.links.size()));
        b_.PatchAt<float>(p + 0x10, c.unk);
    }
    for (size_t i = 0; i < m_.ikChains.size(); i++) {
        size_t p = base + i * kIKChainSize;
        b_.PatchI32(p + 12, Rel(p));
        for (auto& l : m_.ikChains[i].links) {
            b_.I32(l.bone);
            b_.V3(l.kneeDir);
            b_.V3(Vec3());
        }
    }
}

void MdlWriter::WritePoseParams() {
    HdrHere(H_LOCALPOSEPARAMINDEX);
    Hdr(H_NUMLOCALPOSEPARAMETERS, static_cast<int>(m_.poseParams.size()));
    for (auto& pp : m_.poseParams) {
        size_t p = b_.Tell();
        b_.I32(0);
        Str(p, p, pp.name);
        b_.I32(pp.flags);
        b_.F32(pp.start);
        b_.F32(pp.end);
        b_.F32(pp.loop);
    }
    HdrHere(H_LOCALIKAUTOPLAYLOCKINDEX);
    Hdr(H_NUMLOCALIKAUTOPLAYLOCKS, static_cast<int>(m_.ikAutoplayLocks.size()));
    for (auto& l : m_.ikAutoplayLocks) {
        b_.I32(l.chain);
        b_.F32(l.posWeight);
        b_.F32(l.localQWeight);
        b_.I32(l.flags);
        b_.Zero(16);
    }
}

void MdlWriter::WriteIncludeModels() {
    b_.Align(8);
    HdrHere(H_UIPANELOFFSET);
    HdrHere(H_INCLUDEMODELINDEX);
    Hdr(H_NUMINCLUDEMODELS, static_cast<int>(m_.includeModels.size()));
    for (auto& im : m_.includeModels) {
        size_t p = b_.Tell();
        b_.I32(0);
        b_.I32(0);
        Str(p, p, "");
        Str(p + 4, p, im);
    }
}

void MdlWriter::WriteTextures() {
    HdrHere(H_TEXTUREINDEX);
    Hdr(H_NUMTEXTURES, static_cast<int>(m_.textures.size()));
    for (auto& t : m_.textures) {
        size_t p = b_.Tell();
        b_.Zero(kTextureSize);
        Str(p, p, t);
    }
    b_.Align(4);
    HdrHere(H_CDTEXTUREINDEX);
    Hdr(H_NUMCDTEXTURES, static_cast<int>(m_.cdTextures.size()));
    size_t cd = b_.Reserve(4 * m_.cdTextures.size());
    for (size_t i = 0; i < m_.cdTextures.size(); i++) Str(cd + i * 4, 0, m_.cdTextures[i]);
    HdrHere(H_SKININDEX);
    Hdr(H_NUMSKINREF, m_.numSkinRef);
    Hdr(H_NUMSKINFAMILIES, static_cast<int>(m_.skins.size()));
    for (auto& fam : m_.skins)
        for (int16_t v : fam) b_.I16(v);
    b_.Align(4);
}

void MdlWriter::WriteKeyValues() {
    HdrHere(H_KEYVALUEINDEX);
    if (!m_.keyValues.empty()) {
        std::string kv = "mdlkeyvalue\n{\n" + m_.keyValues + "}\n";
        b_.Bytes(kv.data(), kv.size());
        b_.U8(0);
        Hdr(H_KEYVALUESIZE, static_cast<int>(kv.size() + 1));
    }
    b_.Align(4);
    HdrHere(H_SRCBONETRANSFORMINDEX);
}

void MdlWriter::WriteLinearBones() {
    size_t nb = m_.bones.size();
    if (nb <= 1) return;
    b_.Align(4);
    size_t p = b_.Tell();
    HdrHere(H_LINEARBONEINDEX);
    b_.Reserve(0x40);
    b_.PatchI32(p, static_cast<int>(nb));
    b_.PatchI32(p + 0x04, Rel(p));
    for (auto& bn : m_.bones) b_.I32(bn.flags);
    b_.PatchI32(p + 0x08, Rel(p));
    for (auto& bn : m_.bones) b_.I32(bn.parent);
    b_.PatchI32(p + 0x0C, Rel(p));
    for (auto& bn : m_.bones) b_.V3(bn.pos);
    b_.PatchI32(p + 0x10, Rel(p));
    for (auto& bn : m_.bones) b_.Q(bn.quat);
    b_.PatchI32(p + 0x14, Rel(p));
    for (auto& bn : m_.bones) b_.RE(bn.rot);
    b_.PatchI32(p + 0x18, Rel(p));
    for (auto& bn : m_.bones) b_.M34(bn.poseToBone);
    b_.PatchI32(p + 0x20, Rel(p));
    for (auto& bn : m_.bones) b_.V3(bn.rotscale);
    b_.PatchI32(p + 0x24, Rel(p));
    for (auto& bn : m_.bones) b_.Q(bn.qAlignment);
}

void MdlWriter::WritePerTri() {
    b_.Align(4);
    HdrHere(H_PERTRIINDEX);
    const CPerTri& pt = m_.perTri;
    b_.I32(pt.version);
    b_.V3(pt.bmin);
    b_.V3(pt.bmax);
    b_.Zero(32);
    b_.Append(pt.data);
    Hdr(H_PERTRINODECOUNT, pt.nodeCount);
    Hdr(H_PERTRILEAFCOUNT, pt.leafCount);
    Hdr(H_PERTRIVERTCOUNT, pt.vertCount);
    b_.Align(4);
}

std::vector<uint8_t> MdlWriter::Run() {
    WriteHeader();
    WriteSourceFiles();
    WriteBones();
    WriteAttachments();
    WriteHitboxes();
    WriteBoneTable();
    WriteAnimations();
    WriteSequences();
    WriteNodes();
    WriteBodyParts();
    WriteIK();
    WritePoseParams();
    WriteIncludeModels();
    WriteTextures();
    WriteKeyValues();
    WriteLinearBones();
    WritePerTri();
    // bone followers
    if (!m_.boneFollowers.empty()) {
        HdrHere(H_BONEFOLLOWEROFFSET);
        Hdr(H_BONEFOLLOWERCOUNT, static_cast<int>(m_.boneFollowers.size()));
        for (int bf : m_.boneFollowers) b_.I32(bf);
    }
    st_.Write(b_);
    b_.Align(4);

    // checksum over the model body, studiomdl style (shift-add over dwords)
    uint32_t ck = 0;
    for (size_t i = 0; i + 4 <= b_.Size(); i += 4) {
        uint32_t v = b_.ReadAt<uint32_t>(i);
        ck = (ck << 1) + ((ck & 0x8000000) ? 1 : 0) + v;
    }
    int32_t checksum = static_cast<int32_t>(ck);
    Hdr(H_CHECKSUM, checksum);

    // embedded files: PHY, collision, VTX, VVD, VVC (unaligned, as in retail)
    if (!m_.physics.phy.empty()) {
        std::vector<uint8_t> phy = m_.physics.phy;
        std::memcpy(&phy[12], &checksum, 4);  // phyheader_t::checkSum
        HdrHere(H_PHYOFFSET);
        Hdr(H_PHYSIZE, static_cast<int>(phy.size()));
        b_.Append(phy);
    }
    HdrHere(H_COLLISIONOFFSET);
    if (!m_.physics.collisionBlock.empty()) {
        b_.Append(m_.physics.collisionBlock);
        Hdr(H_STATICCOLLISIONCOUNT, m_.physics.staticCollisionCount);
    }
    if (m_.boneFollowers.empty()) HdrHere(H_BONEFOLLOWEROFFSET);
    bool hasMesh = false;
    for (auto& bp : m_.bodyParts)
        for (auto& md : bp.models)
            if (!md.meshes.empty()) hasMesh = true;
    if (hasMesh) {
        auto vtx = WriteVTX(m_, checksum);
        HdrHere(H_VTXOFFSET);
        Hdr(H_VTXSIZE, static_cast<int>(vtx.size()));
        b_.Append(vtx);
        auto vvd = WriteVVD(m_, checksum);
        HdrHere(H_VVDOFFSET);
        Hdr(H_VVDSIZE, static_cast<int>(vvd.size()));
        b_.Append(vvd);
        if (m_.hasVertexColor || m_.hasUV2) {
            auto vvc = WriteVVC(m_, checksum);
            HdrHere(H_VVCOFFSET);
            Hdr(H_VVCSIZE, static_cast<int>(vvc.size()));
            b_.Append(vvc);
        }
    }
    Hdr(H_LENGTH, static_cast<int>(b_.Size()));
    return b_.Data();
}

}  // namespace

std::vector<uint8_t> WriteMDL(const CompiledModel& m) { return MdlWriter(m).Run(); }

}  // namespace r2c
