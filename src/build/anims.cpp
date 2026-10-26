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
        Mat3x4 bm = AngleMatrix(a.sanim[frame][k].rot, a.sanim[frame][k].pos, a.sanim[frame][k].scale);
        transforms[k] = bones[k].parent == -1 ? bm : Concat(transforms[bones[k].parent], bm);
    }
}

namespace {

struct WeightList {
    std::string name;
    std::vector<float> weight, posweight;
};

// Scale for a 16-bit quantized stream: max(floor, maxAbs) / 32767. When the values
// already sit on a slightly coarser grid (a decompiled model), that grid's scale is
// recovered instead so recompiling is lossless. fitVals are the values to test.
double FitScale(const std::vector<double>& fitVals, double maxAbs, double floor) {
    double best = std::max(maxAbs, floor) / 32767;
    if (maxAbs <= floor || fitVals.size() < 8) return best;
    // Raw (half precision) sections of the source are not on the grid, so up to 10% of
    // the values may miss it, and the grid is anchored on each of the largest few values.
    std::vector<double> anchors;
    for (double v : fitVals) anchors.push_back(std::fabs(v));
    std::sort(anchors.begin(), anchors.end(), std::greater<double>());
    anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
    if (anchors.size() > 16) anchors.resize(16);
    auto misses = [&](const std::vector<double>& vals, double s, size_t allowed) {
        size_t m = 0;
        for (double v : vals) {
            double q = v / s;
            if (q > 32767.5 || q < -32768.5 || std::fabs(q - std::round(q)) > 0.05)
                if (++m > allowed) break;
        }
        return m;
    };
    // a spread-out sample rejects almost every candidate cheaply
    std::vector<double> sample;
    size_t step = std::max<size_t>(1, fitVals.size() / 48);
    for (size_t i = 0; i < fitVals.size(); i += step) sample.push_back(fitVals[i]);
    const size_t allowed = fitVals.size() / 10, sampleAllowed = sample.size() / 6;
    for (double anchor : anchors) {
        if (anchor <= floor) break;
        // n = 32768 covers a grid whose most negative value is -32768
        for (int n = 32768; n >= 31000; n--) {
            double s = anchor / n;
            // retail clamps a few off-grid (raw-sourced) values, but never by much
            if (s * 32768 * 1.001 < maxAbs) continue;
            if (misses(sample, s, sampleAllowed) > sampleAllowed) continue;
            if (misses(fitVals, s, allowed) <= allowed) {
                // refine: least squares over the on-grid values, so the recovered step
                // matches the original to float precision (an anchor read back from text
                // is only good to about 1e-6, which on 16k-step values moves a level)
                double vk = 0, kk = 0;
                for (double v : fitVals) {
                    double q = v / s, k = std::round(q);
                    if (std::fabs(q - k) <= 0.05 && k != 0) {
                        vk += v * k;
                        kk += k * k;
                    }
                }
                return kk > 0 ? vk / kk : s;
            }
        }
    }
    return best;
}

// --- two-bone IK (Valve bone_setup Studio_SolveIK / CIKSolver), used by ikfixup

void AlignIKMatrix(Mat3x4& m, const Vec3& alignTo) {
    // column 0 becomes the vector, column 1 = column 2 x column 0, column 2 = 0 x 1
    Vec3 x = Normalize(alignTo);
    Vec3 z = m.Column(2);
    Vec3 y = Normalize(Cross(z, x));
    z = Cross(x, y);
    for (int r = 0; r < 3; r++) {
        m.m[r][0] = x[r];
        m.m[r][1] = y[r];
        m.m[r][2] = z[r];
    }
}

// Places the knee for a two-segment chain (lengths a, b) reaching p, bending towards d.
bool SolveTwoBone(float a, float b, const Vec3& p, const Vec3& d, Vec3& knee) {
    Vec3 x = Normalize(p);
    Vec3 y = Normalize(d - x * Dot(d, x));
    float r = Length(p);
    if (r < 1e-6f) return false;
    float dd = (r + (a * a - b * b) / r) / 2;
    float e = std::sqrt(std::max(0.f, a * a - dd * dd));
    knee = x * dd + y * e;
    return dd > r - b && dd < a;
}

bool SolveIK(int thigh, int kneeBone, int foot, const Vec3& target, std::vector<Mat3x4>& world) {
    constexpr float kKneeMax = 0.9998f;  // about 1 degree short of straight
    Vec3 wThigh = world[thigh].Origin(), wKnee = world[kneeBone].Origin(), wFoot = world[foot].Origin();
    float l1 = Length(wKnee - wThigh), l2 = Length(wFoot - wKnee), l3 = Length(wFoot - wThigh);
    if (l3 > (l1 + l2) * kKneeMax || l3 < 1e-6f) return false;  // too straight to find the knee
    Vec3 kneeDir = Normalize((wKnee - wThigh) - (wFoot - wThigh) * (l1 / l3));
    Vec3 ikFoot = target - wThigh, ikKnee = wKnee - wThigh;
    // exaggerate knee targets for legs that are nearly straight
    float dist = std::max(l1 + l2, Length(target - wThigh) - std::min(l1, l2)) * 100;
    Vec3 targetKnee = ikKnee + kneeDir * dist;
    if (Length(ikFoot) > (l1 + l2) * kKneeMax) ikFoot = Normalize(ikFoot) * ((l1 + l2) * kKneeMax);
    float minDist = std::max(std::fabs(l1 - l2) * 1.15f, std::min(l1, l2) * 0.15f);
    if (Length(ikFoot) < minDist) ikFoot = Normalize(wFoot - wThigh) * minDist;
    Vec3 kneePos;
    if (!SolveTwoBone(l1, l2, ikFoot, targetKnee, kneePos)) return false;
    AlignIKMatrix(world[thigh], kneePos);
    AlignIKMatrix(world[kneeBone], ikFoot - kneePos);
    world[kneeBone].SetOrigin(wThigh + kneePos);
    world[foot].SetOrigin(wThigh + ikFoot);
    return true;
}

// Frame movement scales (retail floors: 128 units, 180 degrees).
Vec4 FrameMovementScales(const std::vector<Vec4>& fm) {
    Vec4 out;
    std::vector<double> vals;
    for (int c = 0; c < 4; c++) {
        vals.clear();
        double maxv = 0;
        for (auto& v : fm) {
            double x = (&v.x)[c];
            maxv = std::max(maxv, std::fabs(x));
            if (x != 0) vals.push_back(x);
        }
        (&out.x)[c] = static_cast<float>(FitScale(vals, maxv, c < 3 ? 128. : 180.));
    }
    return out;
}

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
    for (auto& wl : wls) weightLists[ToLower(wl.name)] = wl.weight;
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
        a.flags = qa->flags | (qa->rawDelta ? STUDIO_DELTA : 0);
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
        for (int j = 0; j < a.numFrames; j++) {
            if (qa->rawDelta) {
                // "delta": the source already holds per-bone local deltas (as a decompiled
                // delta animation does), so they skip the world-space conversion and the
                // default root rotation; bones absent from the source get no delta.
                a.sanim[j].assign(nb, BoneXf());
                for (int k = 0; k < nb; k++) {
                    int l = k < static_cast<int>(inst->globalToLocal.size()) ? inst->globalToLocal[k] : -1;
                    if (l < 0) continue;
                    const BoneFrame& bf = inst->frames[a.startFrame + j][l];
                    a.sanim[j][k].pos = bf.pos * qa->scale;
                    a.sanim[j][k].rot = bf.rot;
                }
            } else {
                ConvertAnimation(*inst, a.startFrame + j, qa->scale, qa->adjust, qa->rotation, a.sanim[j]);
            }
            // bone scale is a local channel: taken straight from the source frame
            for (int k = 0; k < nb; k++) {
                int l = k < static_cast<int>(inst->globalToLocal.size()) ? inst->globalToLocal[k] : -1;
                if (l >= 0) a.sanim[j][k].scale = inst->frames[a.startFrame + j][l].scale;
                else if (!qa->rawDelta) a.sanim[j][k].scale = bones[k].scale;
            }
        }
        for (auto& fr : a.sanim)
            for (auto& b : fr)
                if (std::fabs(b.scale.x - 1) > 1e-4f || std::fabs(b.scale.y - 1) > 1e-4f || std::fabs(b.scale.z - 1) > 1e-4f)
                    a.flags |= STUDIO_HAS_SCALE;
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
        std::vector<Mat3x4> cum(a.numFrames, Identity());  // accumulated extracted motion per frame
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
            Mat3x4 prevMove = moves.empty() ? Identity() : AngleMatrix(moves.back().rot, moves.back().pos);
            if (static_cast<int>(cum.size()) < a.numFrames) cum.resize(a.numFrames, Identity());
            int j;
            for (j = 0; j < numframes; j++) {
                float t = j / n;
                adjpos = v * (v0 * t + 0.5f * (v1 - v0) * t * t);
                adjangle = RadianEuler(rot.x * t, rot.y * t, rot.z * t);
                adjmatrix = Invert(AngleMatrix(adjangle, adjpos));
                cum[j + iStart] = Concat(prevMove, AngleMatrix(adjangle, adjpos));
                for (int k = 0; k < nb; k++) {
                    if (bones[k].parent != -1) continue;
                    Mat3x4 bm = Concat(adjmatrix, AngleMatrix(sanim[j + iStart][k].rot, sanim[j + iStart][k].pos));
                    sanim[j + iStart][k].rot = MatrixAngles(bm);
                    sanim[j + iStart][k].pos = bm.Origin();
                }
            }
            for (int r = j + iStart; r < a.numFrames; r++) cum[r] = Concat(prevMove, AngleMatrix(adjangle, adjpos));
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
                break;  // compiled by ProcessIKRules once the poses are final
            case QcAnimCmd::IKFIXUP: {
                // studiomdl fixupIKErrors: re-solve the leg so the foot stays on its contact
                // position, blended in and out over the rule's range, and bake the result
                if (!cmd.ikrule) break;
                const QcIKRule& q = *cmd.ikrule;
                if (q.type != IK_GROUND && q.type != IK_WORLD) {
                    Warn("%s: ikfixup only handles footstep rules (as studiomdl); ignored", a.name.c_str());
                    break;
                }
                int c = qc.FindIKChain(q.chain);
                if (c < 0) Fatal("%s: unknown chain \"%s\" in ikfixup", a.name.c_str(), q.chain.c_str());
                int foot = FindBone(qc.ikChains[c].bone);
                int knee = foot >= 0 ? bones[foot].parent : -1;
                int thigh = knee >= 0 ? bones[knee].parent : -1;
                if (thigh < 0) Fatal("%s: ikchain \"%s\" is too short for ikfixup", a.name.c_str(), q.chain.c_str());
                const int nf = a.numFrames;
                auto frame = [](int v) { return v < 0 ? 0 : v; };  // unset range values are 0 in studiomdl
                int start = frame(q.start), peak = frame(q.peak), tail = frame(q.tail), end = frame(q.end);
                if (start == 0 && peak == 0 && tail == 0 && end == 0) tail = end = nf - 1;
                if (peak < start) peak += nf - 1;
                if (tail < peak) tail += nf - 1;
                if (end < tail) end += nf - 1;
                int contact = q.contact == -1 ? peak : q.contact;
                int numError = end - start + 1;
                if (end >= nf) numError += 2;
                // Rule weight by frame. studiomdl intends this (IKRuleWeight), but its integer
                // division turns the cycle into 0 or 1; the frame form is the intended curve.
                auto weight = [&](int t) -> float {
                    if (end > nf - 1 && t < start) t += nf - 1;
                    float v;
                    if (t < start) return 0;
                    if (t < peak) v = static_cast<float>(t - start) / std::max(1, peak - start);
                    else if (t < tail) return 1;
                    else if (t < end) v = 1 - static_cast<float>(t - tail) / std::max(1, end - tail);
                    else return 0;
                    return 3 * v * v - 2 * v * v * v;
                };
                std::vector<Mat3x4> world;
                CalcBoneTransforms(a, std::clamp(contact % std::max(1, nf), 0, nf - 1), world);
                const Vec3 footfall = world[foot].Origin();
                for (int k = 0; k < numError; k++) {
                    int t = start + k, fr = t % nf;
                    float s = weight(t);
                    if (s <= 0) continue;
                    CalcBoneTransforms(a, fr, world);
                    Vec3 target = footfall * s + world[foot].Origin() * (1 - s);
                    if (!SolveIK(thigh, knee, foot, target, world)) continue;
                    for (int b : {thigh, knee, foot}) {
                        int p = bones[b].parent;
                        Mat3x4 local = p < 0 ? world[b] : Concat(Invert(world[p]), world[b]);
                        sanim[fr][b].rot = MatrixAngles(local);
                        sanim[fr][b].pos = local.Origin();
                    }
                }
                break;
            }
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
        // Respawn root motion. "rm" (rsx) extracts the root bones' exact yaw and
        // translation relative to frame 0; LX/LY/LZ/LZR extraction above produces a
        // fitted path. Either becomes a frame-movement track unless $legacymovement.
        bool exactRootMotion = (a.qc->flags & STUDIO_FRAMEMOVEMENT) && moves.empty() &&
                               !qc.staticProp && !qc.legacyMovement && !a.qc->hasFmTrack;
        if (a.qc->hasFmTrack && !qc.staticProp) {
            // a stored track: the roots already exclude it, so neither is changed
            if (static_cast<int>(a.qc->fmTrack.size()) != a.numFrames)
                Fatal("animation %s: frame movement track has %d frames, the animation %d", a.name.c_str(),
                      static_cast<int>(a.qc->fmTrack.size()), a.numFrames);
            a.frameMove = a.qc->fmTrack;
            a.fixedFmScales = true;
            a.fmScales = a.qc->fmScales;
            a.flags |= STUDIO_FRAMEMOVEMENT;
            moves.clear();
        } else if (exactRootMotion && !(a.flags & STUDIO_DELTA)) {
            // (delta animations keep their root deltas and carry an all-zero track, as retail does)
            // Retail's yaw is the heading of the root's local Z axis (forward after the
            // default 90 degree rotation) relative to frame 0; pitch and roll stay in the root.
            auto heading = [](const Mat3x4& m) { return std::atan2(m.m[1][2], m.m[0][2]); };
            Mat3x4 t0 = AngleMatrix(sanim[0][rootIndex].rot, sanim[0][rootIndex].pos);
            float h0 = heading(t0);
            for (int j = 0; j < a.numFrames; j++) {
                Mat3x4 t = AngleMatrix(sanim[j][rootIndex].rot, sanim[j][rootIndex].pos);
                RadianEuler yaw(0, 0, heading(t) - h0);
                Vec3 o0 = Concat(AngleMatrix(yaw, Vec3(0, 0, 0)), t0).Origin();
                cum[j] = AngleMatrix(yaw, t.Origin() - o0);
            }
        }
        if ((exactRootMotion || !moves.empty()) && !qc.legacyMovement) {
            a.frameMove.resize(a.numFrames);
            for (int j = 0; j < a.numFrames; j++) {
                Vec3 o = cum[j].Origin();
                // retail keeps yaw wrapped to [-180, 180], which also keeps the stream scale small
                float yaw = RadToDeg(MatrixAngles(cum[j]).z);
                while (yaw > 180.f) yaw -= 360.f;
                while (yaw < -180.f) yaw += 360.f;
                a.frameMove[j] = {o.x, o.y, o.z, yaw};
            }
            // Quantize the track as it will be stored, then take exactly that motion out
            // of the root bones so they absorb the quantization error. Extracted linear
            // motion was already removed from the roots, so only the difference is applied.
            Vec4 fs = FrameMovementScales(a.frameMove);
            for (int j = 0; j < a.numFrames; j++) {
                Vec4& v = a.frameMove[j];
                for (int c = 0; c < 4; c++) {
                    float s = (&fs.x)[c];
                    (&v.x)[c] = std::clamp(std::round((&v.x)[c] / s), -32768.f, 32767.f) * s;
                }
                Mat3x4 corr = Invert(AngleMatrix(RadianEuler(0, 0, DegToRad(v.w)), Vec3(v.x, v.y, v.z)));
                if (!exactRootMotion) corr = Concat(corr, cum[j]);
                for (int k = 0; k < nb; k++) {
                    if (bones[k].parent != -1) continue;
                    Mat3x4 bm = Concat(corr, AngleMatrix(sanim[j][k].rot, sanim[j][k].pos));
                    sanim[j][k].rot = MatrixAngles(bm);
                    sanim[j][k].pos = bm.Origin();
                }
            }
            a.flags |= STUDIO_FRAMEMOVEMENT;
            moves.clear();
        } else if (!a.fixedFmScales) {
            a.flags &= ~STUDIO_FRAMEMOVEMENT;
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
        if (!a.qc->movementRecords.empty()) {
            a.movements.clear();
            int last = -1;
            for (auto& stored : a.qc->movementRecords) {
                if (stored.endFrame < last || stored.endFrame >= a.numFrames || stored.endFrame < 0)
                    Fatal("animation %s: invalid movementrecord end frame", a.name.c_str());
                CMovement cm;
                cm.endFrame = stored.endFrame; cm.motionFlags = stored.motionFlags;
                cm.v0 = stored.v0; cm.v1 = stored.v1; cm.angle = stored.angle;
                cm.vector = stored.vector; cm.position = stored.position;
                a.movements.push_back(cm); last = stored.endFrame;
            }
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

// Position delta from the bind pose (or the weighted raw value for delta animations).
static Vec3 PosDelta(const BuildContext& ctx, const WAnim& a, int j, int f) {
    const BoneXf& b = a.sanim[f][j];
    if (!(a.flags & STUDIO_DELTA)) return b.pos - ctx.bones[j].pos;
    float r = a.weight[j] > 0 ? a.posWeight[j] / a.weight[j] : 1.f;
    return b.pos * r;
}

// The position scale lives in each record, but retail uses one value per bone for the
// whole animation (every section), with studiomdl's floor of 128. When the input is
// already quantized (a decompiled model), the original scale is recovered from the grid
// so that recompiling is lossless; otherwise the scale is maxPos / 32767. The last frame
// of a sectioned animation is left out of the fit: retail's trailing section stores it
// at half precision, so a decompiled value is not on the grid.
static std::vector<float> PositionScales(const BuildContext& ctx, const WAnim& a, bool sectioned) {
    const int nb = static_cast<int>(ctx.bones.size());
    const int fitFrames = sectioned ? a.numFrames - 1 : a.numFrames;
    std::vector<float> scales(nb);
    std::vector<double> vals;
    for (int j = 0; j < nb; j++) {
        vals.clear();
        double maxPos = 0;
        for (int f = 0; f < a.numFrames; f++) {
            Vec3 p = PosDelta(ctx, a, j, f);
            for (int c = 0; c < 3; c++) {
                maxPos = std::max(maxPos, std::fabs(static_cast<double>(p[c])));
                if (f < fitFrames && p[c] != 0) vals.push_back(p[c]);
            }
        }
        scales[j] = static_cast<float>(FitScale(vals, maxPos, 128.));
    }
    return scales;
}

// Encodes frames [f0, f1] of an animation into one bone chain. constFrames: how many
// leading frames decide whether a value is constant (a section's own frames, without
// the frame it shares with the next section, as retail decides it).
static std::vector<uint8_t> EncodeChain(const BuildContext& ctx, const WAnim& a, const std::vector<float>& posScales,
                                        int f0, int f1, int constFrames = -1) {
    const auto& bones = ctx.bones;
    const int nb = static_cast<int>(bones.size());
    bool delta = (a.flags & STUDIO_DELTA) != 0;
    int nf = f1 - f0 + 1;
    const int nfConst = constFrames > 0 ? std::min(constFrames, nf) : nf;
    Buffer buf;
    size_t prevRec = SIZE_MAX;
    int written = 0;
    for (int j = 0; j < nb; j++) {
        if (bones[j].flags & BONE_ALWAYS_PROCEDURAL) continue;
        if (a.weight[j] < 0.001f) continue;

        // deltas from the bind pose (or raw values for delta animations)
        std::vector<Vec3> dpos(nf);
        std::vector<RadianEuler> drot(nf);
        for (int n = 0; n < nf; n++) {
            const BoneXf& b = a.sanim[f0 + n][j];
            dpos[n] = PosDelta(ctx, a, j, f0 + n);
            if (delta) {
                drot[n] = b.rot;
            } else {
                for (int c = 0; c < 3; c++) drot[n][c] = b.rot[c] - bones[j].rot[c];
            }
            for (int c = 0; c < 3; c++) {
                while (drot[n][c] >= kPi) drot[n][c] -= 2 * kPi;
                while (drot[n][c] < -kPi) drot[n][c] += 2 * kPi;
            }
        }
        float posscale = posScales[j];
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
            for (int n = 1; n < nfConst; n++) {
                if (pv[c][n] != pv[c][0]) posConst = false;
                if (rv[c][n] != rv[c][0]) rotConst = false;
            }
        }
        // Scale: raw (Vector48) unless it changes within the chain, then RLE values of
        // (scale - bind scale) / scalescale (no bind scale for delta animations).
        bool scaleIdentity = true, scaleConst = true;
        for (int n = 0; n < nf; n++) {
            const Vec3& s = a.sanim[f0 + n][j].scale;
            const Vec3& s0 = a.sanim[f0][j].scale;
            for (int c = 0; c < 3; c++) {
                if (s[c] != (delta ? 1.f : bones[j].scale[c])) scaleIdentity = false;
                if (std::fabs(s[c] - s0[c]) > 1e-6f) scaleConst = false;
            }
        }
        std::vector<int16_t> ss[3];
        bool animScale = !scaleIdentity && !scaleConst && nf > 1;
        if (scaleConst || nf == 1) {
            double rawError = 0, rleError = 0;
            for (int c = 0; c < 3; c++) {
                float value = a.sanim[f0][j].scale[c];
                float raw = HalfToFloat(FloatToHalf(value));
                float base = delta ? 0.f : bones[j].scale[c];
                float level = std::clamp(std::round((value-base)*2048.f),-32768.f,32767.f);
                float rle = base + level/2048.f;
                rawError += static_cast<double>(raw-value)*(raw-value);
                rleError += static_cast<double>(rle-value)*(rle-value);
            }
            // A constant non-unit bind scale may be exact on its RLE base but not
            // representable as a half. Do not shrink the skinned mesh by rounding it.
            if (rleError < rawError) animScale = true;
        }
        if (animScale) {
            const float step = 1.f / 2048;
            for (int c = 0; c < 3; c++) {
                const float base = delta ? 0.f : bones[j].scale[c];
                std::vector<int16_t> v(nf);
                for (int n = 0; n < nf; n++) {
                    float q = std::round((a.sanim[f0 + n][j].scale[c] - base) / step);
                    if (q > 32767 || q < -32768)
                        Warn("animation %s: bone %s scale %g is out of range", a.name.c_str(), bones[j].name.c_str(),
                             a.sanim[f0 + n][j].scale[c]);
                    v[n] = static_cast<int16_t>(std::clamp(q, -32768.f, 32767.f));
                }
                ss[c] = EncodeRLE(v);
            }
        }
        if (posZero && rotZero && scaleIdentity) continue;

        size_t rec = buf.Reserve(32);
        written++;
        uint8_t flags = animScale ? 0x00 : 0x08;  // RAWSCALE unless the scale is animated
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
        // position: raw Vector48 only when half precision is good enough.
        // Retail always stores a position,
        // even at the bind position (never all-null value pointers).
        bool animPos = false;
        if (posConst || posZero || nf == 1) {
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
        // raw scale (1,1,1 unless the bone holds a constant scale here)
        if (!animScale)
            for (int c = 0; c < 3; c++)
                buf.PatchAt<uint16_t>(rec + 22 + c * 2, FloatToHalf(a.sanim[f0][j].scale[c]));

        // animated value streams follow the record
        // Retail never leaves all three value pointers null in an animated record (the
        // game does not treat that as "bind pose"), so one explicit stream is kept.
        auto emit = [&](size_t ptrPos, std::vector<int16_t>* streams) {
            bool allZero = StreamIsZero(streams[0]) && StreamIsZero(streams[1]) && StreamIsZero(streams[2]);
            for (int c = 0; c < 3; c++) {
                if (StreamIsZero(streams[c]) && !(allZero && c == 0)) {
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
        if (animScale) emit(rec + 22, ss);
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

// mstudioframemovement_t: float scale[4]; short offset[4]; then RLE streams for
// x, y, z and yaw. Default scales match retail (128/32767 units, 180/32767 degrees).
static std::vector<uint8_t> EncodeFrameMovement(const std::vector<Vec4>& fm, const Vec4* given = nullptr) {
    Buffer b;
    b.Reserve(24);
    Vec4 scales = given ? *given : FrameMovementScales(fm);
    for (int c = 0; c < 4; c++) {
        float scale = (&scales.x)[c];
        b.PatchAt<float>(c * 4, scale);
        std::vector<int16_t> vals(fm.size());
        bool zero = true;
        for (size_t f = 0; f < fm.size(); f++) {
            vals[f] = static_cast<int16_t>(scale != 0 ? std::clamp(std::round((&fm[f].x)[c] / scale), -32768.f, 32767.f) : 0.f);
            if (vals[f]) zero = false;
        }
        if (zero) continue;
        size_t at = b.Tell();
        if (at > 32767) Fatal("frame movement track too large");
        b.PatchAt<int16_t>(16 + c * 2, static_cast<int16_t>(at));
        for (int16_t v : EncodeRLE(vals)) b.I16(v);
    }
    b.Align(4);
    return b.Data();
}

// ---------------------------------------------------------------------------
// IK rules (studiomdl ProcessIKRules + CompressIKErrors)
// ---------------------------------------------------------------------------

namespace {

struct WorkRule {
    const QcIKRule* src = nullptr;
    int type = 0, chain = 0, slot = 0, bone = 0;
    int start = 0, peak = 0, tail = 0, end = 0, contact = 0;
    Vec3 pos;
    Quat q = Quat(0, 0, 0, 0);  // studiomdl leaves it zeroed unless the rule type sets it
    float height = 0, radius = 0, floor = 0, endHeight = 0;
    std::string attachment;
    std::vector<std::pair<Vec3, RadianEuler>> error;  // per frame from start
};

// Encodes a rule's per-frame errors as mstudiocompressedikerror_t: float scale[6],
// short offset[6] (from the header), then one RLE stream per component.
std::vector<uint8_t> CompressIKError(const std::vector<std::pair<Vec3, RadianEuler>>& error) {
    Buffer b;
    b.Reserve(36);
    for (int k = 0; k < 6; k++) {
        auto value = [&](size_t n) -> float {
            if (k < 3) return error[n].first[k];
            float v = error[n].second[k - 3];
            while (v >= kPi) v -= 2 * kPi;
            while (v < -kPi) v += 2 * kPi;
            return v;
        };
        float minv = k < 3 ? -128.f : -kPi / 8, maxv = k < 3 ? 128.f : kPi / 8;
        for (size_t n = 0; n < error.size(); n++) {
            minv = std::min(minv, value(n));
            maxv = std::max(maxv, value(n));
        }
        float scale = -minv > maxv ? minv / -32768.f : maxv / 32767.f;
        b.PatchAt<float>(k * 4, scale);
        std::vector<int16_t> v(error.size());
        for (size_t n = 0; n < error.size(); n++)
            v[n] = static_cast<int16_t>(std::clamp(std::round(value(n) / scale), -32768.f, 32767.f));
        if (b.Tell() > 32767) Fatal("IK error stream too large");
        b.PatchAt<int16_t>(24 + k * 2, static_cast<int16_t>(b.Tell()));
        for (int16_t s : EncodeRLE(v)) b.I16(s);
    }
    return b.Data();
}

// A stored error stream (decompiled models): retail's scales and values, re-encoded as is.
std::vector<uint8_t> CompressStoredIKError(const float scales[6], const std::vector<std::array<float, 6>>& errors) {
    Buffer b;
    b.Reserve(36);
    for (int k = 0; k < 6; k++) {
        float scale = scales[k];
        b.PatchAt<float>(k * 4, scale);
        std::vector<int16_t> v(errors.size());
        for (size_t n = 0; n < errors.size(); n++)
            v[n] = static_cast<int16_t>(scale != 0 ? std::clamp(std::round(errors[n][k] / scale), -32768.f, 32767.f) : 0.f);
        if (b.Tell() > 32767) Fatal("stored IK error stream too large");
        b.PatchAt<int16_t>(24 + k * 2, static_cast<int16_t>(b.Tell()));
        for (int16_t s : EncodeRLE(v)) b.I16(s);
    }
    return b.Data();
}

}  // namespace

void ProcessIKRules(BuildContext& ctx) {
    const auto& chains = ctx.out.ikChains;
    const int nb = static_cast<int>(ctx.bones.size());
    for (auto& a : ctx.anims) {
        std::vector<WorkRule> rules;
        for (auto& cmd : a.qc->cmds) {
            if (cmd.type != QcAnimCmd::IKRULE || !cmd.ikrule) continue;
            const QcIKRule& q = *cmd.ikrule;
            WorkRule r;
            r.src = &q;
            r.type = q.type;
            r.chain = ctx.qc.FindIKChain(q.chain);
            if (r.chain < 0) Fatal("%s: unknown chain \"%s\" in ikrule", a.name.c_str(), q.chain.c_str());
            r.slot = q.slot >= 0 ? q.slot : r.chain;
            r.start = q.start;
            r.peak = q.peak;
            r.tail = q.tail;
            r.end = q.end;
            r.contact = q.contact;
            r.height = q.height;
            r.radius = q.radius;
            r.floor = q.floor;
            r.endHeight = q.endHeight;
            r.attachment = q.attachment;
            if (q.hasFakeOrigin) r.pos = q.pos;
            if (q.hasFakeRotate) r.q = q.q;
            rules.push_back(r);
        }
        const int nf = a.numFrames;
        auto findPrev = [&](size_t i) -> WorkRule& {
            for (size_t n = 1; n < rules.size(); n++) {
                size_t j = (i + rules.size() - n) % rules.size();
                if (rules[j].chain == rules[i].chain) return rules[j];
            }
            return rules[i];
        };
        auto findNext = [&](size_t i) -> WorkRule& {
            for (size_t n = 1; n < rules.size(); n++) {
                size_t j = (i + n) % rules.size();
                if (rules[j].chain == rules[i].chain) return rules[j];
            }
            return rules[i];
        };
        // Root motion from the contact frame to frame t, as a transform in frame t's space.
        // studiomdl only offsets by the translation (calcMovement); Respawn's targets follow
        // the whole frame-movement transform, yaw included (retail errors lock to 0 on plant).
        auto motion = [&](int t, int contact) -> Mat3x4 {
            if (a.frameMove.empty() || nf < 1) return Identity();
            auto at = [&](int f) {
                const Vec4& v = a.frameMove[std::clamp(f < nf ? f : f - nf, 0, nf - 1)];
                return AngleMatrix(RadianEuler(0, 0, DegToRad(v.w)), Vec3(v.x, v.y, v.z));
            };
            return Concat(Invert(at(t)), at(contact));
        };
        // Frames past the end of a wrapped rule range: retail evaluates frame t - nf (pose and
        // frame movement alike; a looping animation's frame 0 equals its last frame).
        auto frameOf = [&](int t) { return std::clamp(t < nf ? t : t - nf, 0, nf - 1); };
        std::vector<Mat3x4> boneToWorld;
        auto pose = [&](const WorkRule& r, int t, int sourceFrame) {
            if (r.src->useSource && a.inst) {
                std::vector<Mat3x4> raw;
                const auto& qa = *a.qc;
                ctx.BuildRawTransforms(*a.inst, a.startFrame + frameOf(sourceFrame), qa.scale, qa.adjust, qa.rotation, raw);
                ctx.TranslateAnimations(*a.inst, raw, boneToWorld);
            } else {
                ctx.CalcBoneTransforms(a, frameOf(t), boneToWorld);
            }
        };
        for (size_t i = 0; i < rules.size(); i++) {
            WorkRule& r = rules[i];
            if (r.start == 0 && r.peak == 0 && r.tail == 0 && r.end == 0) r.tail = r.end = nf - 1;
            if (r.start != -1 && r.peak == -1 && r.tail == -1 && r.end != -1) r.peak = r.tail = (r.start + r.end) / 2;
            if (r.start != -1 && r.peak == -1 && r.tail != -1) r.peak = (r.start + r.tail) / 2;
            if (r.peak != -1 && r.tail == -1 && r.end != -1) r.tail = (r.peak + r.end) / 2;
            if (r.peak == -1) r.start = r.peak = 0;
            if (r.tail == -1) r.tail = r.end = nf - 1;
            if (r.contact == -1) r.contact = r.peak;
            const int wrap = std::max(1, nf - 1);
            if (r.start == -1) {
                WorkRule& p = findPrev(i);
                if (p.slot == r.slot) {
                    r.start = r.peak < p.tail ? r.peak + (p.tail - r.peak) / 2 : r.peak + (p.tail - r.peak + nf - 1) / 2;
                    r.start = (r.start + nf / 2) % wrap;
                    p.end = (r.start + nf - 1) % wrap;
                } else {
                    r.start = p.tail;
                    p.end = r.peak;
                }
            }
            if (r.end == -1) {
                WorkRule& n = findNext(i);
                if (n.slot == r.slot) {
                    n.start = n.peak < r.tail ? n.peak + (r.tail - n.peak) / 2 : n.peak + (r.tail - n.peak + nf - 1) / 2;
                    n.start = (n.start + nf / 2) % wrap;
                    r.end = (n.start + nf - 1) % wrap;
                } else {
                    n.start = r.tail;
                    r.end = n.peak;
                }
            }
            if (r.peak < r.start) r.peak += nf - 1;
            if (r.tail < r.peak) r.tail += nf - 1;
            if (r.end < r.tail) r.end += nf - 1;
            if (r.contact < r.start) r.contact += nf - 1;

            int numError = r.end - r.start + 1;
            if (r.end >= nf) numError += 2;
            if (r.src->useSequence)
                Verbose("%s: ikrule usesequence evaluates the animation alone", a.name.c_str());
            const int end = chains[r.chain].links[2].bone;
            switch (r.type) {
            case IK_SELF: {
                r.bone = r.src->bone.empty() ? -1 : ctx.FindBone(r.src->bone);
                if (!r.src->bone.empty() && r.bone < 0) Fatal("unknown bone '%s' in ikrule", r.src->bone.c_str());
                for (int k = 0; k < numError; k++) {
                    pose(r, k + r.start, k + r.start);
                    Mat3x4 local = r.bone >= 0 ? Concat(Invert(boneToWorld[r.bone]), boneToWorld[end]) : boneToWorld[end];
                    r.error.push_back({local.Origin(), MatrixAngles(local)});
                }
                break;
            }
            case IK_ATTACHMENT: {
                if (r.src->bone.empty()) {
                    if (!r.src->hasFakeOrigin && !r.src->hasFakeRotate) r.bone = end;
                    else r.bone = -1;
                } else {
                    r.bone = ctx.FindBone(r.src->bone);
                    if (r.bone < 0) Fatal("unknown bone '%s' in ikrule", r.src->bone.c_str());
                }
                if (r.bone >= 0) {
                    pose(r, r.contact, r.contact);
                    r.pos = boneToWorld[r.bone].Origin();
                    r.q = AngleQuaternion(MatrixAngles(boneToWorld[r.bone]));
                }
                for (int k = 0; k < numError; k++) {
                    int t = k + r.start;
                    pose(r, t, t);
                    Mat3x4 target = Concat(motion(t, r.contact), QuaternionMatrix(r.q, r.pos));
                    Mat3x4 local = Concat(Invert(target), boneToWorld[end]);
                    r.error.push_back({local.Origin(), MatrixAngles(local)});
                }
                break;
            }
            case IK_GROUND: {
                const QcIKChain& qc = ctx.qc.ikChains[r.chain];
                pose(r, r.contact, r.contact);
                Vec3 footfall = Transform(boneToWorld[end], qc.center);
                footfall.z = r.floor;
                r.pos = footfall;
                r.q = Quat(0, 0, 0, 1);
                r.bone = 0;
                for (int k = 0; k < numError; k++) {
                    int t = k + r.start;
                    pose(r, t, r.contact);  // studiomdl samples the contact frame for "usesource"
                    Mat3x4 move = motion(t, r.contact);
                    Vec3 p = Transform(move, r.pos);
                    Vec3 cur = Transform(boneToWorld[end], qc.center);
                    cur.z = p.z;
                    if (t < r.start || t >= r.end) {
                        p = cur;
                    } else if (t < r.peak) {
                        float s = static_cast<float>(r.peak - t) / (r.peak - r.start);
                        s = 3 * s * s - 2 * s * s * s;
                        p = p * (1 - s) + cur * s;
                    } else if (t > r.tail) {
                        float s = static_cast<float>(t - r.tail) / (r.end - r.tail);
                        s = 3 * s * s - 2 * s * s * s;
                        p = p * (1 - s) + cur * s;
                    }
                    Mat3x4 target = Concat(move, QuaternionMatrix(r.q));
                    target.SetOrigin(p);
                    Mat3x4 local = Concat(Invert(target), boneToWorld[end]);
                    r.error.push_back({local.Origin(), MatrixAngles(local)});
                }
                break;
            }
            default:  // IK_RELEASE, IK_UNLATCH, IK_WORLD: no target data
                r.bone = 0;
                break;
            }
        }
        // auto-release chains the animation moves but has no rule for
        if (!(a.flags & STUDIO_DELTA) && !a.qc->noAutoIK) {
            for (int c = 0; c < static_cast<int>(chains.size()); c++) {
                bool used = false;
                for (auto& r : rules) used |= r.chain == c;
                int end = chains[c].links[2].bone;
                if (used || end >= nb || a.weight[end] <= 0) continue;
                WorkRule r;
                r.type = IK_RELEASE;
                r.chain = r.slot = c;
                r.start = r.peak = 0;
                r.tail = r.end = nf - 1;
                r.contact = 0;
                rules.push_back(r);
            }
        }
        a.ikRules.clear();
        for (auto& r : rules) {
            CIKRule o;
            o.type = r.type;
            o.chain = r.chain;
            o.bone = r.bone;
            o.slot = r.slot;
            o.height = r.height;
            o.radius = r.radius;
            o.floor = r.floor;
            o.pos = r.pos;
            o.q = r.q;
            if (r.src && r.src->hasStoredTarget) {
                o.pos = r.src->pos;
                o.q = r.src->q;
            }
            o.endHeight = r.endHeight;
            o.attachment = r.attachment;
            o.iStart = r.start;
            if (nf > 1) {
                const float d = nf - 1.f;
                o.start = r.start / d;
                o.peak = r.peak / d;
                o.tail = r.tail / d;
                o.end = r.end / d;
                o.contact = r.contact / d;
            } else {
                o.start = o.peak = 0;
                o.tail = o.end = 1;
                o.contact = 0;
            }
            if (r.src && r.src->hasErrors)
                o.compressedError = CompressStoredIKError(r.src->errorScales, r.src->errors);
            else if (!r.error.empty())
                o.compressedError = CompressIKError(r.error);
            a.ikRules.push_back(std::move(o));
        }
    }
    // per sequence: rule count is the largest of its animations; animations used by a
    // sequence number their rules (others keep index 0, as studiomdl leaves them)
    for (auto& s : ctx.out.sequences) {
        s.numIKRules = 0;
        for (int16_t ai : s.animIndices) s.numIKRules = std::max(s.numIKRules, static_cast<int>(ctx.anims[ai].ikRules.size()));
        const auto& first = ctx.anims[s.animIndices[0]].ikRules;
        for (int16_t ai : s.animIndices) {
            auto& rules = ctx.anims[ai].ikRules;
            if (rules.size() != first.size())
                Warn("sequence %s: mismatched number of IK rules in its blends", s.label.c_str());
            for (size_t n = 0; n < rules.size(); n++) rules[n].index = static_cast<int>(n);
        }
    }
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
        // a stored scale (decompiled models) keeps the retail quantization grid
        for (auto& rs : ctx.qc.boneRotScales)
            if (IEquals(rs.first, bones[j].name)) bones[j].rotscale = rs.second;
    }
    int sf = ctx.qc.sectionFrames > 0 ? ctx.qc.sectionFrames : 30;
    int sfMin = ctx.qc.sectionFrames > 0 ? ctx.qc.sectionMinFrames : 120;
    for (auto& a : ctx.anims) {
        CAnimation ca;
        ca.name = a.name;
        ca.fps = a.fps;
        // only the bits retail animation descriptors use (looping, snap, delta, all-zeros,
        // 0x20000, frame movement); build-time flags such as noforceloop are not stored
        ca.flags = a.flags & (0x1 | 0x2 | 0x4 | 0x20 | 0x20000 | STUDIO_FRAMEMOVEMENT);
        if (!a.frameMove.empty()) ca.frameMovement = EncodeFrameMovement(a.frameMove, a.fixedFmScales ? &a.fmScales : nullptr);
        else ca.flags &= ~STUDIO_FRAMEMOVEMENT;
        ca.numFrames = a.numFrames;
        ca.movements = a.movements;
        ca.ikRules = a.ikRules;
        ca.bmin = a.bmin;
        ca.bmax = a.bmax;
        bool sectioned = a.numFrames >= sfMin && a.numFrames > sf && sf > 0;
        std::vector<float> posScales = PositionScales(ctx, a, sectioned);
        if (a.qc)
            for (auto& ps : a.qc->posScales) {
                int j = ctx.FindBone(ps.first);
                if (j >= 0 && ps.second > 0) posScales[j] = ps.second;
            }
        if (!sectioned) {
            ca.sections.push_back(EncodeChain(ctx, a, posScales, 0, a.numFrames - 1));
        } else {
            ca.sectionFrames = sf;
            int nsec = (a.numFrames - 1) / sf + 1;
            for (int s = 0; s < nsec; s++) {
                int f0 = s * sf;
                int f1 = std::min(f0 + sf, a.numFrames - 1);
                // The final frame also has its own trailing chain when this is
                // a partial section. It must not change constant-channel choice.
                ca.sections.push_back(EncodeChain(ctx, a, posScales, f0, f1, f1 > f0 ? f1 - f0 : -1));
            }
            // trailing section with only the final frame
            ca.sections.push_back(EncodeChain(ctx, a, posScales, a.numFrames - 1, a.numFrames - 1));
        }
        ctx.out.animations.push_back(std::move(ca));
    }
}

}  // namespace r2c
