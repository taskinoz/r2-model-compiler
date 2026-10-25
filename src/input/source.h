// In-memory form of a model source file (SMD or DMX): skeleton, animation frames
// and triangles. Mirrors studiomdl's s_source_t closely enough that the build
// stages can follow the same rules.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/math.h"

namespace r2c {

constexpr int kMaxWeightsPerVertex = 3;  // Titanfall 2 VVD/VTX limit

struct SrcBone {
    std::string name;
    int parent = -1;
};

// Local (parent-relative) transform of one bone in one frame.
struct BoneFrame {
    Vec3 pos;
    RadianEuler rot;
    Vec3 scale{1, 1, 1};  // local bone scale (Respawn animations only; SMD/DMX extension)
};

struct BoneWeight {
    int bone = 0;
    float weight = 1.f;
};

struct SrcVertex {
    Vec3 pos;
    Vec3 normal;
    Vec2 uv;
    std::vector<BoneWeight> weights;
    // Optional extra channels (DMX or extended SMD)
    bool hasColor = false;
    Vec4 color{1, 1, 1, 1};
    bool hasUV2 = false;
    Vec2 uv2;
    // a stored tangent (DMX tangent$0: xyz, w = bitangent sign) is kept as given, and
    // vertices that differ only in it stay apart, as studiomdl's output shows
    bool hasTangent = false;
    Vec4 tangent;
    int sourceId = -1;  // DMX r2c_vertex_id: corners with different ids never merge
};

struct SrcFace {
    int material = 0;  // index into Source::materials
    int v[3] = {0, 0, 0};  // indices into Source::vertices
    int polygon = -1;      // source polygon (a DMX face) the triangle belongs to; -1 for SMD
};

struct Source {
    std::string path;      // resolved path on disk
    std::string name;      // name as written in the QC
    std::vector<SrcBone> bones;
    std::vector<std::vector<BoneFrame>> frames;  // [frame][bone]
    int startFrame = 0;    // first "time" value in the file
    std::vector<std::string> materials;
    std::vector<SrcVertex> vertices;
    std::vector<SrcFace> faces;
    int polygonCount = 0;  // DMX polygons read so far (ids for SrcFace::polygon)
    bool hasMesh() const { return !faces.empty(); }
    int FindBone(const std::string& n) const;
};

// Loads an SMD or DMX by extension. Throws CompileError on failure.
std::shared_ptr<Source> LoadSource(const std::string& path, bool wantMesh);
std::shared_ptr<Source> LoadSMD(const std::string& path);
std::shared_ptr<Source> LoadDMX(const std::string& path);

}  // namespace r2c
