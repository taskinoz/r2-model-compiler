// SMD (Valve studiomdl data) reader.
//
// Matches studiomdl's conventions: texture V is flipped (1 - v), triangle winding is
// reversed, material names lose their extension, and vertex weights are merged,
// sorted, trimmed below 5% and clipped to the strongest three.
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#include "core/log.h"
#include "core/strings.h"
#include "input/source.h"

namespace r2c {

int Source::FindBone(const std::string& n) const {
    for (size_t i = 0; i < bones.size(); i++)
        if (IEquals(bones[i].name, n)) return static_cast<int>(i);
    return -1;
}

void BalanceWeights(std::vector<BoneWeight>& w, size_t maxCount) {
    // merge duplicates
    for (size_t i = 0; i < w.size(); i++)
        for (size_t j = i + 1; j < w.size(); j++)
            if (w[i].bone == w[j].bone) {
                w[i].weight += w[j].weight;
                w[j].weight = 0;
            }
    std::stable_sort(w.begin(), w.end(), [](const BoneWeight& a, const BoneWeight& b) { return a.weight > b.weight; });
    while (w.size() > 1 && w.back().weight < 0.05f) w.pop_back();
    if (w.size() > maxCount) w.resize(maxCount);
    float t = 0;
    for (auto& b : w) t += b.weight;
    if (t <= 0) {
        for (auto& b : w) b.weight = 1.f / w.size();
    } else {
        for (auto& b : w) b.weight /= t;
    }
}

namespace {

struct LineReader {
    std::vector<std::string> lines;
    size_t at = 0;
    bool Next(std::string& out) {
        if (at >= lines.size()) return false;
        out = lines[at++];
        return true;
    }
    int LineNo() const { return static_cast<int>(at); }
};

bool IsEndLine(const std::string& l) {
    std::string t = Trim(l);
    return t == "end";
}

// Parses floats/ints from a line into tokens.
std::vector<std::string> SplitWS(const std::string& s) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) i++;
        if (i >= s.size()) break;
        size_t j = i;
        if (s[i] == '"') {
            j = s.find('"', i + 1);
            if (j == std::string::npos) j = s.size();
            out.push_back(s.substr(i + 1, j - i - 1));
            i = j + 1;
            continue;
        }
        while (j < s.size() && !std::isspace(static_cast<unsigned char>(s[j]))) j++;
        out.push_back(s.substr(i, j - i));
        i = j;
    }
    return out;
}

}  // namespace

std::shared_ptr<Source> LoadSMD(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) Fatal("could not open SMD \"%s\"", path.c_str());
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();

    LineReader rd;
    {
        size_t s = 0;
        for (size_t i = 0; i <= text.size(); i++) {
            if (i == text.size() || text[i] == '\n') {
                std::string l = text.substr(s, i - s);
                if (!l.empty() && l.back() == '\r') l.pop_back();
                rd.lines.push_back(l);
                s = i + 1;
            }
        }
    }

    auto src = std::make_shared<Source>();
    src->path = path;
    std::string line;
    int endFrame = -1;
    bool haveFrames = false;
    std::vector<std::vector<BoneFrame>> framesByTime;  // indexed by time - startFrame
    std::vector<bool> haveFrame;

    auto matIndex = [&](const std::string& rawName) -> int {
        std::string n = StripExtension(Trim(rawName));
        for (size_t i = 0; i < src->materials.size(); i++)
            if (IEquals(src->materials[i], n)) return static_cast<int>(i);
        src->materials.push_back(n);
        return static_cast<int>(src->materials.size() - 1);
    };

    while (rd.Next(line)) {
        auto tok = SplitWS(line);
        if (tok.empty()) continue;
        const std::string& cmd = tok[0];
        if (cmd.rfind("//", 0) == 0 || cmd[0] == ';' || cmd[0] == '#') continue;
        if (IEquals(cmd, "version")) {
            if (tok.size() < 2 || std::atoi(tok[1].c_str()) != 1) Fatal("%s: bad SMD version", path.c_str());
        } else if (IEquals(cmd, "nodes")) {
            while (rd.Next(line)) {
                if (IsEndLine(line)) break;
                auto t = SplitWS(line);
                if (t.size() < 3) continue;
                int idx = std::atoi(t[0].c_str());
                if (idx < 0 || idx > 4096) Fatal("%s(%d): bad bone index", path.c_str(), rd.LineNo());
                if (static_cast<int>(src->bones.size()) <= idx) src->bones.resize(idx + 1);
                src->bones[idx].name = t[1];
                src->bones[idx].parent = std::atoi(t[2].c_str());
            }
        } else if (IEquals(cmd, "skeleton")) {
            int t = -99999999;
            int startFrame = -1;
            while (rd.Next(line)) {
                if (IsEndLine(line)) break;
                auto tk = SplitWS(line);
                if (tk.empty()) continue;
                if (IEquals(tk[0], "time")) {
                    if (tk.size() < 2) continue;
                    int tm = std::atoi(tk[1].c_str());
                    if (startFrame == -1) startFrame = tm;
                    if (tm < startFrame) Fatal("%s(%d): frame %d before start frame", path.c_str(), rd.LineNo(), tm);
                    t = tm - startFrame;
                    if (tm > endFrame) endFrame = tm;
                    if (static_cast<int>(framesByTime.size()) <= t) {
                        framesByTime.resize(t + 1);
                        haveFrame.resize(t + 1, false);
                    }
                    if (!haveFrame[t]) {
                        framesByTime[t].assign(src->bones.size(), BoneFrame());
                        // carry the previous frame forward, as studiomdl does
                        if (t > 0 && haveFrame[t - 1]) framesByTime[t] = framesByTime[t - 1];
                        haveFrame[t] = true;
                    }
                    continue;
                }
                if (tk.size() >= 7) {
                    if (startFrame < 0) Fatal("%s(%d): bone data before 'time'", path.c_str(), rd.LineNo());
                    int idx = std::atoi(tk[0].c_str());
                    if (idx < 0 || idx >= static_cast<int>(src->bones.size())) continue;
                    BoneFrame& bf = framesByTime[t][idx];
                    bf.pos = Vec3(static_cast<float>(std::atof(tk[1].c_str())), static_cast<float>(std::atof(tk[2].c_str())),
                                  static_cast<float>(std::atof(tk[3].c_str())));
                    bf.rot = RadianEuler(static_cast<float>(std::atof(tk[4].c_str())), static_cast<float>(std::atof(tk[5].c_str())),
                                         static_cast<float>(std::atof(tk[6].c_str())));
                    // r2-model-compiler extension: optional local scale "sx sy sz" after the
                    // rotation (studiomdl reads only the first seven values)
                    if (tk.size() >= 10)
                        bf.scale = Vec3(static_cast<float>(std::atof(tk[7].c_str())), static_cast<float>(std::atof(tk[8].c_str())),
                                        static_cast<float>(std::atof(tk[9].c_str())));
                }
            }
            src->startFrame = startFrame < 0 ? 0 : startFrame;
            for (size_t i = 0; i < haveFrame.size(); i++)
                if (!haveFrame[i]) Fatal("%s is missing frame %d", path.c_str(), static_cast<int>(i) + src->startFrame);
            haveFrames = !framesByTime.empty();
        } else if (IEquals(cmd, "triangles")) {
            while (rd.Next(line)) {
                if (IsEndLine(line)) break;
                std::string texname = Trim(line);
                if (texname.empty()) {
                    rd.at += 3;
                    continue;
                }
                std::string low = ToLower(texname);
                bool skip = (low == "null.bmp" || low == "null.tga");
                SrcVertex vtx[3];
                bool ok = true;
                for (int j = 0; j < 3; j++) {
                    if (!rd.Next(line)) Fatal("%s: unexpected end of triangles", path.c_str());
                    auto t = SplitWS(line);
                    if (t.size() < 9) {
                        ok = false;
                        continue;
                    }
                    int bone = std::atoi(t[0].c_str());
                    if (bone < 0 || bone >= static_cast<int>(src->bones.size()))
                        Fatal("%s(%d): bogus bone index %d", path.c_str(), rd.LineNo(), bone);
                    SrcVertex& v = vtx[j];
                    v.pos = Vec3(std::strtof(t[1].c_str(), nullptr), std::strtof(t[2].c_str(), nullptr), std::strtof(t[3].c_str(), nullptr));
                    v.normal = Vec3(std::strtof(t[4].c_str(), nullptr), std::strtof(t[5].c_str(), nullptr), std::strtof(t[6].c_str(), nullptr));
                    v.uv = Vec2(std::strtof(t[7].c_str(), nullptr), 1.f - std::strtof(t[8].c_str(), nullptr));
                    int count = t.size() > 9 ? std::atoi(t[9].c_str()) : 0;
                    if (count > 0) {
                        for (int k = 0; k < count && 11 + k * 2 < static_cast<int>(t.size()); k++) {
                            BoneWeight bw;
                            bw.bone = std::atoi(t[10 + k * 2].c_str());
                            bw.weight = std::strtof(t[11 + k * 2].c_str(), nullptr);
                            if (bw.bone < 0 || bw.bone >= static_cast<int>(src->bones.size()))
                                Fatal("%s(%d): bogus weight bone %d", path.c_str(), rd.LineNo(), bw.bone);
                            v.weights.push_back(bw);
                        }
                        BalanceWeights(v.weights, kMaxWeightsPerVertex);
                    }
                    if (v.weights.empty()) v.weights.push_back({bone, 1.f});
                }
                if (!ok || skip) continue;
                SrcFace face;
                face.material = matIndex(texname);
                int base = static_cast<int>(src->vertices.size());
                for (int j = 0; j < 3; j++) src->vertices.push_back(vtx[j]);
                // studiomdl flips winding (a, c, b)
                face.v[0] = base;
                face.v[1] = base + 2;
                face.v[2] = base + 1;
                src->faces.push_back(face);
            }
        } else if (IEquals(cmd, "vertexanimation")) {
            // Flex animation is not supported by Titanfall 2 models; skip the block.
            while (rd.Next(line))
                if (IsEndLine(line)) break;
        } else {
            Warn("%s(%d): unknown SMD section \"%s\"", path.c_str(), rd.LineNo(), cmd.c_str());
        }
    }

    if (haveFrames) src->frames = std::move(framesByTime);
    if (src->frames.empty()) {
        // a mesh-only file still needs a bind pose
        src->frames.push_back(std::vector<BoneFrame>(src->bones.size()));
    }
    for (auto& fr : src->frames)
        if (fr.size() < src->bones.size()) fr.resize(src->bones.size());
    for (auto& b : src->bones)
        if (b.name.empty()) b.name = "unnamed";
    return src;
}

std::shared_ptr<Source> LoadSource(const std::string& path, bool wantMesh) {
    (void)wantMesh;
    std::string ext = ToLower(GetExtension(path));
    if (ext == "smd" || ext == "vta") return LoadSMD(path);
    if (ext == "dmx") return LoadDMX(path);
    Fatal("unsupported source file type \"%s\"", path.c_str());
}

}  // namespace r2c
