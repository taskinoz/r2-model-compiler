// Animations: conversion to the global skeleton, studiomdl's processing commands,
// and encoding into Titanfall 2's v53 per-bone RLE records.
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

#include "build/context.h"
#include "core/buffer.h"
#include "core/log.h"
#include "core/strings.h"

namespace r2c {

namespace {

void ClipRot(RadianEuler& r) {
    for (int j = 0; j < 3; j++) {
        while (r[j] >= kPi) r[j] -= 2 * kPi;
        while (r[j] < -kPi) r[j] += 2 * kPi;
    }
}

Quat QuaternionScale(const Quat& p, float t) {
    float sinom = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    sinom = std::min(sinom, 1.f);
    float sinsom = std::sin(std::asin(sinom) * t);
    float s = sinsom / (sinom + FLT_EPSILON);
    Quat q(p.x * s, p.y * s, p.z * s, 0);
    float r = 1.0f - sinsom * sinsom;
    if (r < 0) r = 0;
    r = std::sqrt(r);
    q.w = p.w < 0 ? -r : r;
    return q;
}

// QuaternionSM: s * p then * q
Quat QuaternionSM(float s, const Quat& p, const Quat& q) {
    return QuaternionNormalize(QuaternionMult(QuaternionScale(p, s), q));
}
// QuaternionMA: p * (s * q)
Quat QuaternionMA(const Quat& p, float s, const Quat& q) {
    return QuaternionNormalize(QuaternionMult(p, QuaternionScale(q, s)));
}

RadianEuler QuaternionSMAngles(float s, const RadianEuler& p, const RadianEuler& q) {
    return QuaternionAngles(QuaternionSM(s, AngleQuaternion(p), AngleQuaternion(q)));
}
RadianEuler QuaternionMAAngles(const RadianEuler& p, float s, const RadianEuler& q) {
    return QuaternionAngles(QuaternionMA(AngleQuaternion(p), s, AngleQuaternion(q)));
}

}  // namespace

void BuildContext::CalcBoneTransforms(const WAnim& a, int frame, std::vector<Mat3x4>& transforms) const {
    transforms.resize(bones.size());
    for (size_t k = 0; k < bones.size(); k++) {
        Mat3x4 bm = AngleMatrix(a.sanim[frame][k].rot, a.sanim[frame][k].pos);
        transforms[k] = bones[k].parent == -1 ? bm : Concat(transforms[bones[k].parent], bm);
    }
}

namespace {

struct WeightList {
    std::string name;
    std::vector<float> weight, posweight;
};

}  // namespace

void BuildContext::BuildAnimations() {
    const int nb = static_cast<int>(bones.size());
    int rootIndex = 0;

    // --- weightlists (buildAnimationWeights); index 0 is the implicit default
    std::vector<WeightList> wls;
    {
        WeightList def;
        def.name = "default";
        def.weight.assign(nb, -1);
        def.posweight.assign(nb, -1);
        for (int j = 0; j < nb; j++)
            if (bones[j].parent == -1) def.weight[j] = def.posweight[j] = 1;
        const QcWeightList* qdef = nullptr;
        for (auto& w : qc.weightLists)
            if (IEquals(w.name, "default")) qdef = &w;
        auto apply = [&](WeightList& wl, const QcWeightList& q) {
            for (size_t i = 0; i < q.bones.size(); i++) {
                int k = FindBone(q.bones[i]);
                if (k == -1) Fatal("unknown bone reference '%s' in weightlist '%s'", q.bones[i].c_str(), q.name.c_str());
                wl.weight[k] = q.weights[i];
                wl.posweight[k] = q.posWeights[i];
            }
        };
        if (qdef) apply(def, *qdef);
        wls.push_back(def);
        for (auto& q : qc.weightLists) {
            if (IEquals(q.name, "default")) continue;
            WeightList wl;
            wl.name = q.name;
            wl.weight.resize(nb);
            wl.posweight.resize(nb);
            for (int j = 0; j < nb; j++) {
                if (bones[j].parent != -1) {
                    wl.weight[j] = wls[0].weight[j];
                    wl.posweight[j] = wls[0].posweight[j];
                } else {
                    wl.weight[j] = wl.posweight[j] = 0;
                }
            }
            apply(wl, q);
            wls.push_back(wl);
        }
        for (auto& wl : wls)
            for (int j = 0; j < nb; j++)
                if (wl.weight[j] < 0 && bones[j].parent != -1) {
                    wl.weight[j] = wl.weight[bones[j].parent];
                    wl.posweight[j] = wl.posweight[bones[j].parent];
                }
        for (auto& wl : wls)
            for (int j = 0; j < nb; j++) {
                if (wl.weight[j] < 0) wl.weight[j] = 1;
                if (wl.posweight[j] < 0) wl.posweight[j] = wl.weight[j];
            }
    }
    auto findWL = [&](const std::string& n) -> int {
        for (size_t i = 1; i < wls.size(); i++)
            if (IEquals(wls[i].name, n)) return static_cast<int>(i);
        if (IEquals(n, "default")) return 0;
        Fatal("unknown weightlist '%s'", n.c_str());
    };

    // --- RemapAnimations
    anims.clear();
    for (auto& qa : qc.animations) {
        if (qa->isOverride && !qa->source) {
            Warn("animation \"%s\" was declared but never defined", qa->name.c_str());
            continue;
        }
        WAnim a;
        a.qc = qa;
        a.name = qa->name;
        a.fps = qa->fps;
        a.flags = qa->flags;
        a.motionType = qa->motionType;
        SrcInst* inst = Instance(qa->source, qa->sourceScale, false, false, qa->file);
        a.inst = inst;
        int srcStart = inst->src->startFrame;
        int srcEnd = srcStart + static_cast<int>(inst->frames.size()) - 1;
        int s = qa->startFrame, e = qa->endFrame;
        if (qa->frameRangeSet) {
            s = std::max(s, srcStart);
            e = std::min(e, srcEnd);
        } else {
            s = srcStart;
            e = srcEnd;
        }
        if (qc.staticProp) e = s;
        if (e < s) Fatal("end frame before start frame in %s", qa->name.c_str());
        a.numFrames = e - s + 1;
        a.startFrame = s - srcStart;
        a.sanim.resize(a.numFrames);
        for (int j = 0; j < a.numFrames; j++)
            ConvertAnimation(*inst, a.startFrame + j, qa->scale, qa->adjust, qa->rotation, a.sanim[j]);
        if (qc.staticProp) {
            for (auto& b : a.sanim[0]) {
                b.pos = Vec3();
                b.rot = RadianEuler();
            }
            // single "static_prop" bone at identity; other bones keep the bind pose
            for (int k = 0; k < nb; k++) {
                a.sanim[0][k].pos = bones[k].pos;
                a.sanim[0][k].rot = bones[k].rot;
            }
        }
        anims.push_back(std::move(a));
    }
    auto findAnim = [&](const std::string& n) -> WAnim* {
        for (auto& a : anims)
            if (IEquals(a.name, n)) return &a;
        // a sequence name refers to its first animation
        int si = qc.FindSequence(n);
        if (si >= 0 && !qc.sequences[si].animations.empty())
            for (auto& a : anims)
                if (IEquals(a.name, qc.sequences[si].animations[0])) return &a;
        return nullptr;
    };

    // --- processAnimations
    for (auto& a : anims) {
        auto& sanim = a.sanim;
        // extractUnusedMotion
        {
            int type = a.motionType;
            for (int k = 0; k < nb; k++) {
                if (bones[k].parent != -1) continue;
                BoneXf m = sanim[0][k];
                for (int j = 0; j < a.numFrames; j++) {
                    if (type & STUDIO_X) sanim[j][k].pos.x = m.pos.x;
                    if (type & STUDIO_Y) sanim[j][k].pos.y = m.pos.y;
                    if (type & STUDIO_Z) sanim[j][k].pos.z = m.pos.z;
                    if (type & STUDIO_XR) sanim[j][k].rot.x = m.rot.x;
                    if (type & STUDIO_YR) sanim[j][k].rot.y = m.rot.y;
                    if (type & STUDIO_ZR) sanim[j][k].rot.z = m.rot.z;
                }
            }
        }
        a.weight = wls[0].weight;
        a.posWeight = wls[0].posweight;
        if (a.qc->fudgeLoop && a.numFrames > 1) {
            int j = a.numFrames;
            float scale = 1.f / (j - 1.0f);
            std::vector<BoneXf> f(nb);
            for (int k = 0; k < nb; k++) {
                Vec3 d = sanim[j - 1][k].pos - sanim[0][k].pos;
                f[k].pos = sanim[j - 1][k].pos + d * scale;
                f[k].rot = sanim[0][k].rot;
            }
            sanim.push_back(f);
            a.numFrames++;
        }

        // piecewise movement state
        struct Move {
            int endframe;
            int flags;
            Vec3 pos, vector;
            RadianEuler rot;
            float v0, v1;
        };
        std::vector<Move> moves;
        auto extractLinearMotion = [&](int motiontype, int iStart, int iEnd, int iSrc, WAnim* ref, int iRef) {
            if (a.numFrames <= 1) Fatal("can't extract motion from sequence %s, it has one frame", a.name.c_str());
            iEnd = std::min(iEnd, a.numFrames - 1);
            iSrc = std::min(iSrc, a.numFrames - 1);
            if (iStart >= iEnd) {
                Warn("motion extraction ignored, no frames remaining in %s", a.name.c_str());
                return;
            }
            float fFrame = (iStart + iSrc) / 2.0f;
            int iMid = static_cast<int>(fFrame);
            float s = fFrame - iMid;
            RadianEuler rot;
            if (motiontype & (STUDIO_LXR | STUDIO_LYR | STUDIO_LZR)) {
                Quat q0 = AngleQuaternion(ref->sanim[iRef][rootIndex].rot);
                Quat q1 = AngleQuaternion(sanim[iMid][rootIndex].rot);
                Quat q2 = AngleQuaternion(sanim[iSrc][rootIndex].rot);
                Quat d1 = QuaternionMA(q1, -1, q0);
                Quat d2 = QuaternionMA(q2, -1, q0);
                if (motiontype & STUDIO_LXR) rot.x = QuaternionAngles(QuaternionNormalize(Quat(d2.x, 0, 0, d2.w))).x;
                if (motiontype & STUDIO_LYR) rot.y = QuaternionAngles(QuaternionNormalize(Quat(0, d2.y, 0, d2.w))).y;
                if (motiontype & STUDIO_LZR) {
                    RadianEuler a3 = QuaternionAngles(QuaternionNormalize(Quat(0, 0, d2.z, d2.w)));
                    RadianEuler a5 = QuaternionAngles(QuaternionNormalize(Quat(0, 0, d1.z, d1.w)));
                    if (a3.z > kPi) a5.z -= 2 * kPi;
                    if (a3.z < -kPi) a5.z += 2 * kPi;
                    if (a5.z > kPi) a5.z -= 2 * kPi;
                    if (a5.z < -kPi) a5.z += 2 * kPi;
                    if (a5.z > kPi / 4 && a3.z < 0) a3.z += 2 * kPi;
                    if (a5.z < -kPi / 4 && a3.z > 0) a3.z -= 2 * kPi;
                    rot.z = a3.z;
                }
            }
            Vec3 p0 = Rotate(AngleMatrix(rot), ref->sanim[iRef][rootIndex].pos);
            Vec3 p2 = sanim[iSrc][rootIndex].pos;
            Vec3 p1 = sanim[iMid][rootIndex].pos * (1 - s) + sanim[std::min(iMid + 1, a.numFrames - 1)][rootIndex].pos * s;
            p2 = p2 - p0;
            p1 = p1 - p0;
            if (!(motiontype & STUDIO_LX)) p2.x = p1.x = 0;
            if (!(motiontype & STUDIO_LY)) p2.y = p1.y = 0;
            if (!(motiontype & STUDIO_LZ)) p2.z = p1.z = 0;
            float d1 = Length(p1), d2 = Length(p2);
            float v0 = -1 * d2 + 4 * d1;
            float v1 = 3 * d2 - 4 * d1;
            int numframes = iEnd - iStart + 1;
            float n = static_cast<float>(numframes - 1);
            if (motiontype & STUDIO_LINEAR)
                v0 = v1 = Length(p2);
            else if (v0 < 0) {
                v0 = 0;
                v1 = Length(p2) * 2;
            } else if (v1 < 0) {
                v0 = Length(p2) * 2;
                v1 = 0;
            } else if ((v0 + v1) > 0.01f && (std::fabs(v0 - v1) / (v0 + v1)) < 0.2f)
                v0 = v1 = Length(p2);
            Vec3 v = Normalize(p2);
            Vec3 adjpos;
            RadianEuler adjangle;
            Mat3x4 adjmatrix;
            int j;
            for (j = 0; j < numframes; j++) {
                float t = j / n;
                adjpos = v * (v0 * t + 0.5f * (v1 - v0) * t * t);
                adjangle = RadianEuler(rot.x * t, rot.y * t, rot.z * t);
                adjmatrix = Invert(AngleMatrix(adjangle, adjpos));
                for (int k = 0; k < nb; k++) {
                    if (bones[k].parent != -1) continue;
                    Mat3x4 bm = Concat(adjmatrix, AngleMatrix(sanim[j + iStart][k].rot, sanim[j + iStart][k].pos));
                    sanim[j + iStart][k].rot = MatrixAngles(bm);
                    sanim[j + iStart][k].pos = bm.Origin();
                }
            }
            for (; j + iStart < a.numFrames; j++)
                for (int k = 0; k < nb; k++) {
                    if (bones[k].parent != -1) continue;
                    Mat3x4 bm = Concat(adjmatrix, AngleMatrix(sanim[j + iStart][k].rot, sanim[j + iStart][k].pos));
                    sanim[j + iStart][k].rot = MatrixAngles(bm);
                    sanim[j + iStart][k].pos = bm.Origin();
                }
            Move mv;
            mv.endframe = iEnd;
            mv.flags = motiontype;
            if (!moves.empty()) {
                Mat3x4 bm = Concat(AngleMatrix(moves.back().rot, moves.back().pos), AngleMatrix(adjangle, adjpos));
                mv.rot = MatrixAngles(bm);
                mv.pos = bm.Origin();
                mv.vector = mv.pos - moves.back().pos;
            } else {
                mv.pos = adjpos;
                mv.rot = adjangle;
                mv.vector = mv.pos;
            }
            mv.vector = Normalize(mv.vector);
            mv.v0 = v0;
            mv.v1 = v1;
            moves.push_back(mv);
        };

        int startframe = 0;
        for (auto& cmd : a.qc->cmds) {
            switch (cmd.type) {
            case QcAnimCmd::WEIGHTS: {
                int i = findWL(cmd.str);
                a.weight = wls[i].weight;
                a.posWeight = wls[i].posweight;
                break;
            }
            case QcAnimCmd::SUBTRACT: {
                WAnim* ref = findAnim(cmd.str);
                if (!ref) Fatal("unknown subtract animation '%s' in %s", cmd.str.c_str(), a.name.c_str());
                if (cmd.i0 >= ref->numFrames) Fatal("subtract frame %d out of range for %s", cmd.i0, ref->name.c_str());
                a.flags |= STUDIO_DELTA;
                std::vector<BoneXf> src = ref->sanim[cmd.i0];
                for (int k = 0; k < nb; k++) {
                    if (a.weight[k] <= 0) continue;
                    for (int j = 0; j < a.numFrames; j++) {
                        if (cmd.flags & STUDIO_POST) {
                            sanim[j][k].rot = QuaternionSMAngles(-1, src[k].rot, sanim[j][k].rot);
                            sanim[j][k].pos = sanim[j][k].pos - src[k].pos;
                        } else {
                            sanim[j][k].rot = QuaternionMAAngles(sanim[j][k].rot, -1, src[k].rot);
                            sanim[j][k].pos = src[k].pos - sanim[j][k].pos;
                        }
                    }
                }
                break;
            }
            case QcAnimCmd::FIXUPLOOP: {
                int m = a.numFrames - 1;
                if (m == 0) break;
                std::vector<Vec3> dpos(nb);
                std::vector<Quat> dq(nb);
                for (int k = 0; k < nb; k++) {
                    dpos[k] = sanim[m][k].pos - sanim[0][k].pos;
                    dq[k] = QuaternionMA(AngleQuaternion(sanim[m][k].rot), -1, AngleQuaternion(sanim[0][k].rot));
                }
                for (int k = 0; k < nb; k++)
                    if (bones[k].parent == -1) {
                        if (a.motionType & STUDIO_LX) dpos[k].x = 0;
                        if (a.motionType & STUDIO_LY) dpos[k].y = 0;
                        if (a.motionType & STUDIO_LZ) dpos[k].z = 0;
                    }
                int start = cmd.i0, end = cmd.i1;
                if (end - start > a.numFrames) {
                    end = a.numFrames + start;
                    if (end < 0) {
                        end = 0;
                        start = -(a.numFrames - 1);
                    }
                }
                float nf = static_cast<float>(end - start);
                auto addDeltas = [&](int frame, float s) {
                    for (int k = 0; k < nb; k++) {
                        if (a.weight[k] <= 0) continue;
                        sanim[frame][k].rot = QuaternionAngles(QuaternionSM(s, dq[k], AngleQuaternion(sanim[frame][k].rot)));
                        sanim[frame][k].pos = sanim[frame][k].pos + dpos[k] * s;
                    }
                };
                for (int j = start + 1; j <= 0; j++) {
                    float s = (j - start) / nf;
                    s = 3 * s * s - 2 * s * s * s;
                    addDeltas(m + j, -s);
                }
                for (int j = 0; j < end; j++) {
                    float s = (end - j) / nf;
                    s = 3 * s * s - 2 * s * s * s;
                    addDeltas(j, s);
                }
                break;
            }
            case QcAnimCmd::ANGLE: {
                float da = 0;
                if (!moves.empty()) {
                    Vec3 pos = moves.back().pos;
                    if (pos.x != 0 || pos.y != 0) da = cmd.f0 - RadToDeg(std::atan2(pos.y, pos.x));
                    Mat3x4 yaw = AngleMatrix(RadianEuler(0, 0, DegToRad(da)));
                    for (auto& mv : moves) {
                        mv.pos = Rotate(yaw, mv.pos);
                        mv.vector = Rotate(yaw, mv.vector);
                    }
                } else {
                    Vec3 pos = sanim[a.numFrames - 1][rootIndex].pos - sanim[0][rootIndex].pos;
                    if (pos.x != 0 || pos.y != 0) da = cmd.f0 - RadToDeg(std::atan2(pos.y, pos.x));
                }
                Mat3x4 rootxform = AngleMatrix(RadianEuler(0, 0, DegToRad(da)));
                for (int j = 0; j < a.numFrames; j++)
                    for (int k = 0; k < nb; k++) {
                        if (bones[k].parent != -1) continue;
                        Mat3x4 d = Concat(rootxform, AngleMatrix(sanim[j][k].rot, sanim[j][k].pos));
                        sanim[j][k].rot = MatrixAngles(d);
                        sanim[j][k].pos = d.Origin();
                    }
                break;
            }
            case QcAnimCmd::MOTION:
                extractLinearMotion(cmd.motion, startframe, cmd.i0, cmd.i0, &a, startframe);
                startframe = cmd.i0;
                break;
            case QcAnimCmd::REFMOTION: {
                WAnim* ref = findAnim(cmd.str);
                if (!ref) Fatal("unknown walkalign animation '%s'", cmd.str.c_str());
                extractLinearMotion(cmd.motion, startframe, cmd.i0, cmd.i1, ref, cmd.i2);
                startframe = cmd.i0;
                break;
            }
            case QcAnimCmd::NOANIMATION:
                a.flags |= STUDIO_DELTA | STUDIO_ALLZEROS;
                if (!cmd.flags) {
                    a.numFrames = 1;
                    sanim.resize(1);
                }
                for (auto& fr : sanim)
                    for (auto& b : fr) {
                        b.pos = Vec3();
                        b.rot = RadianEuler();
                    }
                std::fill(a.weight.begin(), a.weight.end(), 0.f);
                std::fill(a.posWeight.begin(), a.posWeight.end(), 0.f);
                a.noAnimation = true;
                break;
            case QcAnimCmd::LINEARDELTA: {
                a.flags |= STUDIO_DELTA;
                int srcframe = a.numFrames - 1;
                std::vector<BoneXf> src0 = sanim[0], src1 = sanim[srcframe];
                for (int k = 0; k < nb; k++)
                    for (int j = 0; j < a.numFrames; j++) {
                        float s = a.numFrames > 1 ? static_cast<float>(j) / (a.numFrames - 1) : 1.f;
                        if (cmd.flags & STUDIO_AL_SPLINE) s = 3 * s * s - 2 * s * s * s;
                        if (a.weight[k] <= 0) continue;
                        BoneXf src;
                        src.pos = src0[k].pos * (1 - s) + src1[k].pos * s;
                        src.rot = QuaternionAngles(QuaternionSlerp(AngleQuaternion(src0[k].rot), AngleQuaternion(src1[k].rot), s));
                        if (cmd.flags & STUDIO_AL_POST) {
                            sanim[j][k].rot = QuaternionSMAngles(-1, src.rot, sanim[j][k].rot);
                            sanim[j][k].pos = sanim[j][k].pos - src.pos;
                        } else {
                            sanim[j][k].rot = QuaternionMAAngles(sanim[j][k].rot, -1, src.rot);
                            sanim[j][k].pos = src.pos - sanim[j][k].pos;
                        }
                    }
                break;
            }
            case QcAnimCmd::COMPRESS: {
                int skip = std::max(1, cmd.i0);
                std::vector<std::vector<BoneXf>> n;
                n.push_back(sanim[0]);
                for (int j = skip; j < a.numFrames; j += skip) n.push_back(sanim[j]);
                sanim = n;
                a.numFrames = static_cast<int>(sanim.size());
                a.fps /= skip;
                break;
            }
            case QcAnimCmd::NUMFRAMES:
                while (a.numFrames < cmd.i0) {
                    sanim.push_back(sanim.back());
                    a.numFrames++;
                }
                if (a.numFrames > cmd.i0 && cmd.i0 > 0) {
                    sanim.resize(cmd.i0);
                    a.numFrames = cmd.i0;
                }
                break;
            case QcAnimCmd::COUNTERROTATE: {
                int bone = FindBone(cmd.str2);
                if (bone == -1) Fatal("unable to find bone %s to counterrotate", cmd.str2.c_str());
                Mat3x4 target;
                if (!cmd.hasTarget) {
                    target = Concat(AngleMatrix(a.qc->rotation), bones[bone].boneToPose);
                } else {
                    target = AngleMatrix(RadianEuler(DegToRad(cmd.angles.z), DegToRad(cmd.angles.x), DegToRad(cmd.angles.y)));
                }
                target.SetOrigin(Vec3());
                for (int j = 0; j < a.numFrames; j++) {
                    std::vector<Mat3x4> b2w;
                    CalcBoneTransforms(a, j, b2w);
                    Mat3x4 t = target;
                    t.SetOrigin(b2w[bone].Origin());
                    b2w[bone] = t;
                    int p = bones[bone].parent;
                    Mat3x4 local = p == -1 ? b2w[bone] : Concat(Invert(b2w[p]), b2w[bone]);
                    sanim[j][bone].rot = MatrixAngles(local);
                    sanim[j][bone].pos = local.Origin();
                }
                break;
            }
            case QcAnimCmd::IKRULE:
            case QcAnimCmd::IKFIXUP:
                Warn("%s: ikrule/ikfixup are not compiled yet; ignored", a.name.c_str());
                break;
            default:
                Warn("%s: animation command %d is not supported yet; ignored", a.name.c_str(), static_cast<int>(cmd.type));
                break;
            }
        }
        if (a.motionType & (STUDIO_LX | STUDIO_LY | STUDIO_LZ | STUDIO_LXR | STUDIO_LYR | STUDIO_LZR)) {
            extractLinearMotion(a.motionType, startframe, a.numFrames - 1, a.numFrames - 1, &a, startframe);
        }
        // realignLooping
        if (a.numFrames > 1 && a.qc->loopRestart) {
            int lr = a.qc->loopRestart;
            if (lr >= a.numFrames) Fatal("loopstart (%d) out of range for animation %s", lr, a.name.c_str());
            std::vector<std::vector<BoneXf>> sh(a.numFrames);
            for (int j = 0; j < a.numFrames - 1; j++) sh[j] = sanim[(j + lr) % (a.numFrames - 1)];
            sh[a.numFrames - 1] = sanim[lr];
            sanim = sh;
        }
        // forceAnimationLoop
        if ((a.flags & STUDIO_LOOPING) && !(a.flags & STUDIO_NOFORCELOOP)) {
            int m = a.numFrames - 1;
            int type = a.motionType;
            for (int k = 0; k < nb; k++) {
                if (!(type & STUDIO_LX)) sanim[m][k].pos.x = sanim[0][k].pos.x;
                if (!(type & STUDIO_LY)) sanim[m][k].pos.y = sanim[0][k].pos.y;
                if (!(type & STUDIO_LZ)) sanim[m][k].pos.z = sanim[0][k].pos.z;
                if (!(type & STUDIO_LXR)) sanim[m][k].rot.x = sanim[0][k].rot.x;
                if (!(type & STUDIO_LYR)) sanim[m][k].rot.y = sanim[0][k].rot.y;
                if (!(type & STUDIO_LZR)) sanim[m][k].rot.z = sanim[0][k].rot.z;
            }
        }
        for (auto& mv : moves) {
            CMovement cm;
            cm.endFrame = mv.endframe;
            cm.motionFlags = mv.flags;
            cm.v0 = mv.v0;
            cm.v1 = mv.v1;
            cm.angle = RadToDeg(mv.rot.z);
            cm.vector = mv.vector;
            cm.position = mv.pos;
            a.movements.push_back(cm);
        }
    }
}

// ---------------------------------------------------------------------------
// v53 encoding
// ---------------------------------------------------------------------------

namespace {

// studiomdl's RLE of int16 values (CompressAnimations)
std::vector<int16_t> EncodeRLE(const std::vector<int16_t>& value) {
    struct Blk {
        int valid = 0, total = 0;
        std::vector<int16_t> v;
    };
    std::vector<Blk> blocks;
    size_t n = value.size();
    blocks.push_back(Blk());
    blocks.back().valid = 1;
    blocks.back().total = 1;
    blocks.back().v.push_back(value[0]);
    for (size_t m = 1; m < n; m++) {
        Blk* p = &blocks.back();
        if (p->total == 255) {
            blocks.push_back(Blk());
            p = &blocks.back();
            p->valid = 1;
            p->v.push_back(value[m]);
        } else if (value[m] != value[m - 1] || (p->total == p->valid && (m < n - 1) && value[m] != value[m + 1])) {
            if (p->total != p->valid) {
                blocks.push_back(Blk());
                p = &blocks.back();
            }
            p->valid++;
            p->v.push_back(value[m]);
        }
        p->total++;
    }
    std::vector<int16_t> out;
    for (auto& b : blocks) {
        uint16_t hdr = static_cast<uint16_t>((b.valid & 0xFF) | ((b.total & 0xFF) << 8));
        out.push_back(static_cast<int16_t>(hdr));
        for (auto v : b.v) out.push_back(v);
    }
    return out;
}

bool StreamIsZero(const std::vector<int16_t>& s) {
    // a single block with one valid value of 0 means "no animation"
    if (s.size() == 2 && s[1] == 0) return true;
    return false;
}

}  // namespace

// Encodes frames [f0, f1] of an animation into one bone chain.
static std::vector<uint8_t> EncodeChain(const BuildContext& ctx, const WAnim& a, int f0, int f1) {
    const auto& bones = ctx.bones;
    const int nb = static_cast<int>(bones.size());
    bool delta = (a.flags & STUDIO_DELTA) != 0;
    int nf = f1 - f0 + 1;
    Buffer buf;
    size_t prevRec = SIZE_MAX;
    int written = 0;
    for (int j = 0; j < nb; j++) {
        if (bones[j].flags & BONE_ALWAYS_PROCEDURAL) continue;
        if (a.weight[j] < 0.001f) continue;

        // deltas from the bind pose (or raw values for delta animations)
        std::vector<Vec3> dpos(nf);
        std::vector<RadianEuler> drot(nf);
        float maxPos = 0;
        for (int n = 0; n < nf; n++) {
            const BoneXf& b = a.sanim[f0 + n][j];
            if (delta) {
                dpos[n] = b.pos;
                float r = a.weight[j] > 0 ? a.posWeight[j] / a.weight[j] : 1.f;
                dpos[n] *= r;
                drot[n] = b.rot;
            } else {
                dpos[n] = b.pos - bones[j].pos;
                for (int c = 0; c < 3; c++) drot[n][c] = b.rot[c] - bones[j].rot[c];
            }
            for (int c = 0; c < 3; c++) {
                while (drot[n][c] >= kPi) drot[n][c] -= 2 * kPi;
                while (drot[n][c] < -kPi) drot[n][c] += 2 * kPi;
            }
            for (int c = 0; c < 3; c++) maxPos = std::max(maxPos, std::fabs(dpos[n][c]));
        }
        // per-record position scale (v53 keeps it in the record), same floor as studiomdl
        float posscale = std::max(maxPos, 128.f) / 32767.f;
        std::vector<int16_t> pv[3], rv[3];
        for (int c = 0; c < 3; c++) {
            pv[c].resize(nf);
            rv[c].resize(nf);
            for (int n = 0; n < nf; n++) {
                // Round rather than truncate: decoding and re-encoding an exact
                // quantization level can otherwise lose one unit to float error.
                // On long bone offsets a lost rotation unit is visibly amplified.
                pv[c][n] = static_cast<int16_t>(std::clamp(std::round(dpos[n][c] / posscale), -32768.f, 32767.f));
                rv[c][n] = static_cast<int16_t>(std::clamp(std::round(drot[n][c] / bones[j].rotscale[c]), -32768.f, 32767.f));
            }
        }
        std::vector<int16_t> ps[3], rs[3];
        bool posZero = true, rotZero = true, posConst = true, rotConst = true;
        for (int c = 0; c < 3; c++) {
            ps[c] = EncodeRLE(pv[c]);
            rs[c] = EncodeRLE(rv[c]);
            if (!StreamIsZero(ps[c])) posZero = false;
            if (!StreamIsZero(rs[c])) rotZero = false;
            for (int n = 1; n < nf; n++) {
                if (pv[c][n] != pv[c][0]) posConst = false;
                if (rv[c][n] != rv[c][0]) rotConst = false;
            }
        }
        if (posZero && rotZero) continue;

        size_t rec = buf.Reserve(32);
        written++;
        uint8_t flags = 0x08;  // RAWSCALE: Titanfall 2 always stores a raw scale
        if (delta) flags |= 0x01;
        buf.PatchAt<float>(rec + 0, posscale);
        buf.PatchAt<uint8_t>(rec + 4, static_cast<uint8_t>(j));

        // rotation
        bool animRot = false;
        if (rotZero) {
            flags |= 0x10;  // NOROT: bind pose (or identity for delta)
        } else if (rotConst || nf == 1) {
            const BoneXf& b = a.sanim[f0][j];
            Quat q = AngleQuaternion(b.rot);
            if (!delta) q = QuaternionAlign(bones[j].qAlignment, q);
            uint64_t packed = PackQuaternion64(q);
            Quat decoded = UnpackQuaternion64(packed);
            RadianEuler rleAngles;
            for (int c = 0; c < 3; ++c)
                rleAngles[c] = rv[c][0] * bones[j].rotscale[c] + (delta ? 0.f : bones[j].rot[c]);
            Quat rle = AngleQuaternion(rleAngles);
            auto error = [&](const Quat& value) {
                double norm = std::sqrt(static_cast<double>(value.x)*value.x + static_cast<double>(value.y)*value.y +
                                        static_cast<double>(value.z)*value.z + static_cast<double>(value.w)*value.w);
                double dot = q.x*value.x + q.y*value.y + q.z*value.z + q.w*value.w;
                double sign = dot < 0 ? -1. : 1.;
                double dx = value.x/norm - sign*q.x, dy = value.y/norm - sign*q.y;
                double dz = value.z/norm - sign*q.z, dw = value.w/norm - sign*q.w;
                return dx*dx + dy*dy + dz*dz + dw*dw;
            };
            if (error(decoded) <= error(rle) + 1e-15) {
                buf.PatchAt<uint64_t>(rec + 8, packed);
                flags |= 0x04;
            } else {
                // Quaternion64 loses angular precision when its omitted W is
                // almost zero. Constant RLE values retain the bone's precision.
                animRot = true;
            }
        } else {
            animRot = true;
        }
        // position: raw Vector48 only when half precision is good enough
        bool animPos = false;
        if (posZero) {
            // zero offsets: bind position
        } else if (posConst || nf == 1) {
            Vec3 p = delta ? dpos[0] : a.sanim[f0][j].pos;
            bool ok = true;
            uint16_t h[3];
            for (int c = 0; c < 3; c++) {
                h[c] = FloatToHalf(p[c]);
                if (std::fabs(HalfToFloat(h[c]) - p[c]) > 0.01f) ok = false;
            }
            if (ok) {
                for (int c = 0; c < 3; c++) buf.PatchAt<uint16_t>(rec + 16 + c * 2, h[c]);
                flags |= 0x02;
            } else
                animPos = true;
        } else
            animPos = true;
        // scale (raw 1,1,1)
        for (int c = 0; c < 3; c++) buf.PatchAt<uint16_t>(rec + 22 + c * 2, FloatToHalf(1.f));

        // animated value streams follow the record
        auto emit = [&](size_t ptrPos, std::vector<int16_t>* streams) {
            for (int c = 0; c < 3; c++) {
                if (StreamIsZero(streams[c])) {
                    buf.PatchAt<int16_t>(ptrPos + c * 2, 0);
                    continue;
                }
                size_t at = buf.Tell();
                long off = static_cast<long>(at - ptrPos);
                if (off > 32767) Fatal("animation %s: bone %s data too large for one section (use $sectionframes)", a.name.c_str(), bones[j].name.c_str());
                buf.PatchAt<int16_t>(ptrPos + c * 2, static_cast<int16_t>(off));
                for (auto v : streams[c]) buf.I16(v);
            }
        };
        if (animRot) emit(rec + 8, rs);
        if (animPos) emit(rec + 16, ps);
        buf.PatchAt<uint8_t>(rec + 5, flags);
        buf.Align(4);
        if (prevRec != SIZE_MAX) buf.PatchI32(prevRec + 28, static_cast<int32_t>(rec - prevRec));
        prevRec = rec;
    }
    if (written == 0) {
        // empty chain: a single terminator record for bone 255
        size_t rec = buf.Reserve(32);
        buf.PatchAt<uint8_t>(rec + 4, 255);
        buf.PatchAt<uint8_t>(rec + 5, 0);
    }
    return buf.Data();
}

void EncodeAnimations(BuildContext& ctx) {
    auto& bones = ctx.bones;
    const int nb = static_cast<int>(bones.size());
    // global rotation scales (CompressAnimations)
    for (int j = 0; j < nb; j++) {
        for (int k = 0; k < 3; k++) {
            float minv = -kPi / 8, maxv = kPi / 8;
            for (auto& a : ctx.anims) {
                for (int n = 0; n < a.numFrames; n++) {
                    float v = (a.flags & STUDIO_DELTA) ? a.sanim[n][j].rot[k] : a.sanim[n][j].rot[k] - bones[j].rot[k];
                    while (v >= kPi) v -= 2 * kPi;
                    while (v < -kPi) v += 2 * kPi;
                    minv = std::min(minv, v);
                    maxv = std::max(maxv, v);
                }
            }
            float scale = (-minv > maxv) ? minv / -32768.f : maxv / 32767.f;
            bones[j].rotscale[k] = scale;
        }
    }
    int sf = ctx.qc.sectionFrames > 0 ? ctx.qc.sectionFrames : 30;
    int sfMin = ctx.qc.sectionFrames > 0 ? ctx.qc.sectionMinFrames : 120;
    for (auto& a : ctx.anims) {
        CAnimation ca;
        ca.name = a.name;
        ca.fps = a.fps;
        ca.flags = a.flags & ~(STUDIO_FRAMEMOVEMENT);
        ca.numFrames = a.numFrames;
        ca.movements = a.movements;
        ca.bmin = a.bmin;
        ca.bmax = a.bmax;
        bool sectioned = a.numFrames >= sfMin && a.numFrames > sf && sf > 0;
        if (!sectioned) {
            ca.sections.push_back(EncodeChain(ctx, a, 0, a.numFrames - 1));
        } else {
            ca.sectionFrames = sf;
            int nsec = (a.numFrames - 1) / sf + 1;
            for (int s = 0; s < nsec; s++) {
                int f0 = s * sf;
                int f1 = std::min(f0 + sf, a.numFrames - 1);
                ca.sections.push_back(EncodeChain(ctx, a, f0, f1));
            }
            // trailing section with only the final frame
            ca.sections.push_back(EncodeChain(ctx, a, a.numFrames - 1, a.numFrames - 1));
        }
        ctx.out.animations.push_back(std::move(ca));
    }
}

}  // namespace r2c
