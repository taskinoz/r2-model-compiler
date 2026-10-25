// Shared state for the build stages (bones, meshes, animations, collision).
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "build/compiled.h"
#include "qc/qc.h"

namespace r2c {

// A source file as used by one QC reference: scale applied, winding as requested.
struct SrcInst {
    std::shared_ptr<Source> src;
    std::string label;
    float scale = 1;
    bool reverse = false;
    bool active = false;  // used as a visible model (vertex bones count as used)
    std::vector<int> meshVertexTexture;  // global texture per Source::meshVertices entry

    std::vector<SrcBone> bones;
    std::vector<std::vector<BoneFrame>> frames;
    std::vector<SrcVertex> verts;  // positions/normals; weights use local bone indices
    std::vector<SrcFace> faces;
    std::vector<int> faceTexture;  // global texture index per face

    // filled by the bone stages
    std::vector<int> boneflags, boneref;
    std::vector<int> localToGlobal, globalToLocal;
    std::vector<Mat3x4> boneToPose;  // bind pose of frame 0
    // vertices remapped to global bones / global bind pose
    std::vector<SrcVertex> gverts;
};

struct GBone {
    std::string name;
    int parent = -1;
    int flags = 0;
    bool preDefined = false, preAligned = false, dontCollapse = false;
    Mat3x4 rawLocal, rawLocalOriginal;
    Mat3x4 boneToPose;
    Mat3x4 srcRealign;
    Vec3 pos;
    RadianEuler rot;
    Vec3 posscale, rotscale;
    Quat qAlignment;
    int group = 0;
    Vec3 bmin, bmax;
    int physicsBone = 0;
};

// Animation in global bone space, one entry per frame per bone.
struct BoneXf {
    Vec3 pos;
    RadianEuler rot;
    Vec3 scale{1, 1, 1};  // local scale (raw delta for delta animations)
};

struct WAnim {
    std::shared_ptr<QcAnimation> qc;
    SrcInst* inst = nullptr;
    std::string name;
    int numFrames = 1;
    int startFrame = 0;  // source frame index (0-based) of the first frame
    float fps = 30;
    int flags = 0;
    int motionType = 0;
    std::vector<std::vector<BoneXf>> sanim;  // [frame][bone]
    std::vector<float> weight;    // per bone
    std::vector<float> posWeight; // per bone
    std::vector<CMovement> movements;
    std::vector<Vec4> frameMove;  // per frame: x, y, z, yaw (degrees); empty when unused
    std::vector<CIKRule> ikRules;
    Vec3 bmin, bmax;
    bool noAnimation = false;
};

class BuildContext {
public:
    BuildContext(QcModelDef& qc) : qc(qc) {}

    QcModelDef& qc;
    CompiledModel out;

    std::vector<std::unique_ptr<SrcInst>> insts;
    std::vector<GBone> bones;
    std::vector<WAnim> anims;

    SrcInst* Instance(const std::shared_ptr<Source>& s, float scale, bool reverse, bool active, const std::string& label);
    int FindBone(const std::string& name) const;

    // stages
    void LoadInstances();
    void MakeStaticProp();
    void BuildBones();
    void RemapVertices();
    void BuildMeshes();
    void BuildAnimations();
    void BuildSequences();
    void BuildHitboxes();
    void BuildAttachments();
    void BuildIK();
    void BuildBounds();
    void BuildSkins();
    void BuildCollision();
    void BuildPerTri();
    void FinalizeBones();

    // helpers
    void BuildRawTransforms(const SrcInst& s, int frame, float scale, const Vec3& shift, const RadianEuler& rotate,
                            std::vector<Mat3x4>& out) const;
    void TranslateAnimations(const SrcInst& s, const std::vector<Mat3x4>& src, std::vector<Mat3x4>& dest) const;
    void ConvertAnimation(const SrcInst& s, int frame, float scale, const Vec3& shift, const RadianEuler& rotate,
                          std::vector<BoneXf>& dest) const;
    void CalcBoneTransforms(const WAnim& a, int frame, std::vector<Mat3x4>& boneToWorld) const;

    std::map<SrcInst*, int> modelInstLods;  // unused helper
    std::map<std::string, std::vector<float>> weightLists;  // expanded per-bone weights, by lower-case name
    std::vector<SrcInst*> bodyInsts;        // [model index across all bodyparts] -> LOD0 instance (nullptr for blank)
};

// Runs every stage and returns the finished model.
CompiledModel Build(QcModelDef& qc);

}  // namespace r2c
