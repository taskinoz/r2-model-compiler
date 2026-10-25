// QC parser. Command semantics follow Valve's studiomdl (2005 SDK) with the
// Respawn-era additions that rsx and Crowbar emit for v53 decompiles.
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <unordered_map>

#include "core/log.h"
#include "core/strings.h"
#include "qc/qc.h"
#include "qc/tokenizer.h"

namespace r2c {

int LookupControl(const std::string& s) {
    static const std::pair<const char*, int> table[] = {
        {"X", STUDIO_X},   {"Y", STUDIO_Y},   {"Z", STUDIO_Z},   {"XR", STUDIO_XR},   {"YR", STUDIO_YR},
        {"ZR", STUDIO_ZR}, {"LX", STUDIO_LX}, {"LY", STUDIO_LY}, {"LZ", STUDIO_LZ},   {"LXR", STUDIO_LXR},
        {"LYR", STUDIO_LYR}, {"LZR", STUDIO_LZR}, {"LM", STUDIO_LINEAR}, {"LQ", STUDIO_QUADRATIC_MOTION},
    };
    for (auto& e : table)
        if (s == e.first) return e.second;  // studiomdl compares case-sensitively
    return -1;
}

int QcModelDef::FindAnimation(const std::string& name) const {
    for (size_t i = 0; i < animations.size(); i++)
        if (IEquals(animations[i]->name, name)) return static_cast<int>(i);
    return -1;
}
int QcModelDef::FindSequence(const std::string& name) const {
    for (size_t i = 0; i < sequences.size(); i++)
        if (IEquals(sequences[i].name, name)) return static_cast<int>(i);
    return -1;
}
int QcModelDef::FindPoseParam(const std::string& name) const {
    for (size_t i = 0; i < poseParams.size(); i++)
        if (IEquals(poseParams[i].name, name)) return static_cast<int>(i);
    return -1;
}
int QcModelDef::FindIKChain(const std::string& name) const {
    for (size_t i = 0; i < ikChains.size(); i++)
        if (IEquals(ikChains[i].name, name)) return static_cast<int>(i);
    return -1;
}
int QcModelDef::TextureIndex(const std::string& rawName, bool add) {
    // stored as written (retail keeps both slash styles), matched slash-insensitively
    std::string name = StripExtension(rawName);
    for (size_t i = 0; i < textures.size(); i++)
        if (IEquals(FixSlashes(textures[i]), FixSlashes(name))) return static_cast<int>(i);
    if (!add) return -1;
    textures.push_back(name);
    return static_cast<int>(textures.size() - 1);
}

namespace {

// QAngle (pitch, yaw, roll) degrees -> RadianEuler (roll, pitch, yaw) radians
RadianEuler QAngleToRadianEuler(float p, float y, float r) { return RadianEuler(DegToRad(r), DegToRad(p), DegToRad(y)); }

class Parser {
public:
    Parser(QcModelDef& m, const QcOptions& o) : m_(m), opt_(o) {}

    void Run(const std::string& path);

private:
    QcModelDef& m_;
    const QcOptions& opt_;
    Tokenizer tk_;
    std::vector<std::string> cdStack_;
    std::string cd_;  // current $cd directory (absolute)
    std::map<std::string, std::shared_ptr<Source>> cache_;

    using Handler = std::function<void()>;
    std::unordered_map<std::string, Handler> handlers_;

    void Register();
    std::string ResolveSource(const std::string& name, const char* defaultExt);
    std::shared_ptr<Source> Load(const std::string& name, bool isModel);

    float F(const char* what) { return tk_.NeedFloat(what); }
    int I(const char* what) { return tk_.NeedInt(what); }
    std::string S(const char* what) { return tk_.Need(what); }
    bool Avail() { return tk_.Available(); }
    Vec3 V3() {
        float x = F("x");
        float y = F("y");
        float z = F("z");
        return Vec3(x, y, z);
    }
    const std::string& T() { return tk_.Text(); }
    bool Is(const char* s) { return IEquals(tk_.Text(), s); }

    // Iterates the tokens of a command whose options may sit on the same line or
    // inside a { } block that can span lines, calling fn for each option token.
    void ForEachOption(const std::function<bool()>& fn);

    void ParseStudio(QcSubModel& sm);
    void CmdBody();
    void CmdModel();
    void CmdBodygroup();
    void CmdLOD(bool shadow);
    void CmdTextureGroup();
    void CmdDefineBone();
    void CmdAttachment();
    void CmdHBox();
    void CmdPoseParameter();
    void CmdIKChain();
    void CmdWeightlist(bool isDefault);
    void CmdAnimation();
    void CmdSequence(bool declare);
    void CmdDeclareAnimation();
    bool ParseAnimationToken(QcAnimation& a);
    bool ParseCmdlistToken(std::vector<QcAnimCmd>& cmds);
    void ParseIKRule(QcIKRule& r);
    void CmdCollision(bool joints);
    void CmdCollisionText();
    void CmdJiggleBone();
    void CmdProceduralBones();
    void CmdUIPanel();
    Vec3 helperBase_;  // current <basepos> while reading a .vrd file
    void CmdKeyValues(std::string& out);
    int ParseContents(int base);
    std::shared_ptr<QcAnimation> ImpliedAnimation(const std::string& seqName, const std::string& file);
};

std::string Parser::ResolveSource(const std::string& name, const char* defaultExt) {
    std::vector<std::string> dirs;
    dirs.push_back(cd_);
    for (auto& d : opt_.searchPaths) dirs.push_back(d);
    std::vector<std::string> exts;
    std::string ext = GetExtension(name);
    if (!ext.empty())
        exts.push_back("");
    else {
        if (defaultExt && *defaultExt) exts.push_back(std::string(".") + defaultExt);
        exts.push_back(".smd");
        exts.push_back(".dmx");
    }
    for (auto& d : dirs) {
        for (auto& e : exts) {
            std::string p = JoinPath(d, name + e);
            if (FileExists(p)) return p;
        }
    }
    return "";
}

std::shared_ptr<Source> Parser::Load(const std::string& name, bool isModel) {
    std::string path = ResolveSource(name, nullptr);
    if (path.empty()) Fatal("%s: could not find source file \"%s\"", tk_.Where().c_str(), name.c_str());
    std::string key = ToLower(std::filesystem::u8path(path).lexically_normal().u8string());
    auto it = cache_.find(key);
    if (it != cache_.end()) return it->second;
    Verbose("loading %s", path.c_str());
    auto src = LoadSource(path, isModel);
    src->name = name;
    if (src->bones.empty()) Fatal("missing all bones in file '%s'", path.c_str());
    cache_[key] = src;
    return src;
}

void Parser::ForEachOption(const std::function<bool()>& fn) {
    int depth = 0;
    while (true) {
        if (depth > 0) {
            if (!tk_.Get(true)) break;
        } else {
            if (!tk_.Available()) {
                // allow a '{' on the next line
                if (tk_.Get(true)) {
                    if (T() == "{" && !tk_.Cur().quoted) {
                        depth++;
                        continue;
                    }
                    tk_.Unget();
                }
                break;
            }
            tk_.Get(false);
        }
        if (!tk_.Cur().quoted && T() == "{") {
            depth++;
            continue;
        }
        if (!tk_.Cur().quoted && T() == "}") {
            depth--;
            if (depth <= 0) break;
            continue;
        }
        if (!fn()) Fatal("%s: unknown option \"%s\"", tk_.Where().c_str(), T().c_str());
    }
}

void Parser::ParseStudio(QcSubModel& sm) {
    sm.file = S("model file");
    sm.scale = m_.defaultScale;
    while (Avail()) {
        tk_.Get(false);
        if (Is("reverse"))
            sm.reverse = true;
        else if (Is("scale"))
            sm.scale = F("scale");
        else if (Is("faces")) {
            S("faces");
            S("faces");
        } else if (Is("bias"))
            S("bias");
        else if (T() == "{") {
            tk_.Unget();
            break;
        } else
            Fatal("%s: unknown model option \"%s\"", tk_.Where().c_str(), T().c_str());
    }
    sm.source = Load(sm.file, true);
    // register textures in first-use order, as studiomdl does while loading
    for (auto& mat : sm.source->materials) m_.TextureIndex(mat, true);
}

void Parser::CmdBody() {
    QcBodyPart bp;
    bp.name = S("body name");
    QcSubModel sm;
    sm.name = bp.name;
    ParseStudio(sm);
    bp.models.push_back(sm);
    m_.bodyParts.push_back(bp);
}

void Parser::CmdModel() {
    QcBodyPart bp;
    bp.name = S("model name");
    QcSubModel sm;
    sm.name = bp.name;
    ParseStudio(sm);
    bp.models.push_back(sm);
    m_.bodyParts.push_back(bp);
    // $model may carry a { } block of flex/eyeball commands; Titanfall 2 has no flexes.
    if (tk_.Get(true)) {
        if (T() == "{" && !tk_.Cur().quoted) {
            tk_.Unget();
            tk_.ReadBlock();
            Verbose("ignoring $model flex/eyeball block for %s", bp.name.c_str());
        } else
            tk_.Unget();
    }
}

void Parser::CmdBodygroup() {
    QcBodyPart bp;
    bp.name = S("bodygroup name");
    if (!tk_.Get(true) || T() != "{") Fatal("%s: expected '{' after $bodygroup", tk_.Where().c_str());
    while (tk_.Get(true)) {
        if (T() == "}") break;
        if (Is("studio")) {
            QcSubModel sm;
            ParseStudio(sm);
            sm.name = FileBase(sm.file);
            bp.models.push_back(sm);
        } else if (Is("blank")) {
            QcSubModel sm;
            sm.name = "blank";
            sm.blank = true;
            bp.models.push_back(sm);
        } else
            Fatal("%s: unknown bodygroup option \"%s\"", tk_.Where().c_str(), T().c_str());
    }
    m_.bodyParts.push_back(bp);
}

void Parser::CmdLOD(bool shadow) {
    QcLod lod;
    lod.shadow = shadow;
    if (shadow) {
        lod.switchPoint = -1;
        if (Avail()) tk_.Get(false);
    } else {
        lod.switchPoint = F("LOD switch value");
    }
    if (!tk_.Get(true) || T() != "{") Fatal("%s: expected '{' for LOD", tk_.Where().c_str());
    while (tk_.Get(true)) {
        if (T() == "}") break;
        if (Is("replacemodel")) {
            QcLod::Replace r;
            r.from = S("model");
            r.to = S("replacement");
            r.scale = m_.defaultScale;
            while (Avail()) {
                tk_.Get(false);
                if (Is("reverse"))
                    r.reverse = true;
                else if (Is("scale"))
                    r.scale = F("scale");
                else {
                    tk_.Unget();
                    break;
                }
            }
            r.source = Load(r.to, true);
            lod.replaceModels.push_back(r);
        } else if (Is("removemodel")) {
            lod.removeModels.push_back(S("model"));
        } else if (Is("replacematerial")) {
            std::string a = S("material");
            std::string b = S("replacement");
            lod.replaceMaterials.push_back({a, b});
        } else if (Is("removemesh")) {
            lod.removeMeshes.push_back(S("material"));
        } else if (Is("replacebone")) {
            std::string a = S("bone");
            std::string b = S("replacement");
            lod.replaceBones.push_back({a, b});
        } else if (Is("bonetreecollapse")) {
            lod.boneTreeCollapses.push_back(S("bone"));
        } else if (Is("nofacial") || Is("facial")) {
        } else if (Is("use_shadowlod_materials")) {
            if (shadow) m_.flags |= STUDIOHDR_FLAGS_USE_SHADOWLOD_MATERIALS;
        } else
            Fatal("%s: invalid LOD option \"%s\"", tk_.Where().c_str(), T().c_str());
    }
    if (shadow) m_.flags |= STUDIOHDR_FLAGS_HASSHADOWLOD;
    m_.lods.push_back(lod);
}

void Parser::CmdTextureGroup() {
    std::string name = S("texturegroup name");
    (void)name;
    std::vector<std::vector<std::string>> group;
    int depth = 0;
    while (tk_.Get(true)) {
        if (!tk_.Cur().quoted && T() == "{") {
            depth++;
            if (depth == 2) group.emplace_back();
            continue;
        }
        if (!tk_.Cur().quoted && T() == "}") {
            depth--;
            if (depth == 0) break;
            continue;
        }
        if (depth == 2) {
            group.back().push_back(StripExtension(T()));
            m_.TextureIndex(T(), true);
        }
    }
    m_.textureGroups.push_back(group);
}

void Parser::CmdDefineBone() {
    QcDefineBone d;
    d.name = S("bone name");
    d.parent = S("parent name");
    float px = F("x"), py = F("y"), pz = F("z");
    float ax = F("pitch"), ay = F("yaw"), az = F("roll");
    d.rawLocal = AngleMatrix(QAngleToRadianEuler(ax, ay, az), Vec3(px, py, pz));
    if (Avail()) {
        d.preAligned = true;
        float rx = F("x"), ry = F("y"), rz = F("z");
        float bx = F("pitch"), by = F("yaw"), bz = F("roll");
        d.srcRealign = AngleMatrix(QAngleToRadianEuler(bx, by, bz), Vec3(rx, ry, rz));
    }
    m_.defineBones.push_back(d);
}

void Parser::CmdAttachment() {
    QcAttachment a;
    a.name = S("attachment name");
    a.bone = S("bone name");
    Vec3 p = V3();
    p *= m_.defaultScale;
    while (Avail()) {
        tk_.Get(false);
        if (Is("absolute")) {
            a.absolute = true;
            a.local = InvertRigid(AngleMatrix(m_.defaultRotation));
        } else if (Is("rigid")) {
            a.rigid = true;
        } else if (Is("world_align")) {
            a.flags |= 0x10000;
        } else if (Is("rotate")) {
            float ang[3] = {0, 0, 0};
            for (int i = 0; i < 3 && Avail(); i++) ang[i] = F("angle");
            a.local = AngleMatrix(QAngleToRadianEuler(ang[0], ang[1], ang[2]));
        } else if (Is("matrix")) {
            // r2c extension: the stored 3x4 bone-space matrix, row by row (retail has some
            // that are not rotations, such as all zeros)
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 4; c++) a.local.m[r][c] = F("matrix value");
            a.exactMatrix = true;
        } else if (Is("x_and_z_axes")) {
            Vec3 x = V3();
            Vec3 z = V3();
            x = Normalize(x);
            z = Normalize(z - x * Dot(z, x));
            Vec3 y = Cross(z, x);
            for (int r = 0; r < 3; r++) {
                a.local.m[r][0] = x[r];
                a.local.m[r][1] = y[r];
                a.local.m[r][2] = z[r];
                a.local.m[r][3] = 0;
            }
        } else
            Fatal("%s: unknown attachment option \"%s\"", tk_.Where().c_str(), T().c_str());
    }
    if (!a.exactMatrix) a.local.SetOrigin(p);
    m_.attachments.push_back(a);
}

void Parser::CmdHBox() {
    if (m_.hitboxSets.empty()) {
        m_.hitboxSets.emplace_back();
        m_.hitboxSets.back().name = "default";
    }
    QcHitbox h;
    h.group = I("hitgroup");
    h.bone = S("bone");
    h.bmin = V3() * m_.defaultScale;
    h.bmax = V3() * m_.defaultScale;
    // optional: name, crit override (r2), hit data group (r2). Numbers that follow
    // the name are the crit flag; any further string is the keyvalues group.
    int extra = 0;
    while (Avail()) {
        tk_.Get(false);
        const std::string& t = T();
        char* end = nullptr;
        long v = std::strtol(t.c_str(), &end, 10);
        bool numeric = !t.empty() && end && *end == 0;
        if (extra == 0 && !numeric)
            h.name = t;
        else if (numeric)
            h.critOverride = static_cast<int>(v);
        else
            h.hitDataGroup = t;
        extra++;
    }
    m_.hitboxSets.back().boxes.push_back(h);
}

void Parser::CmdPoseParameter() {
    QcPoseParam p;
    p.name = S("pose parameter name");
    int idx = m_.FindPoseParam(p.name);
    if (Avail()) {
        p.min = F("min");
        p.minSet = true;
    }
    if (Avail()) {
        p.max = F("max");
        p.maxSet = true;
    }
    while (Avail()) {
        tk_.Get(false);
        if (Is("wrap")) {
            p.flags |= STUDIO_LOOPING;
            p.loop = p.max - p.min;
        } else if (Is("loop")) {
            p.flags |= STUDIO_LOOPING;
            p.loop = F("loop");
        }
    }
    if (idx >= 0)
        m_.poseParams[idx] = p;
    else
        m_.poseParams.push_back(p);
}

void Parser::CmdIKChain() {
    QcIKChain c;
    c.name = S("ikchain name");
    if (m_.FindIKChain(c.name) >= 0) {
        while (Avail()) tk_.Get(false);
        Info("duplicate ikchain \"%s\" ignored", c.name.c_str());
        return;
    }
    c.bone = S("bone");
    while (Avail()) {
        tk_.Get(false);
        int ctrl = LookupControl(T());
        if (ctrl != -1) {
            c.axis = ctrl;
            c.value = F("value");
        } else if (Is("height"))
            c.height = F("height");
        else if (Is("pad"))
            c.radius = F("pad") / 2.f;
        else if (Is("floor"))
            c.floor = F("floor");
        else if (Is("knee"))
            c.knee = V3();
        else if (Is("center"))
            c.center = V3();
        else if (Is("angle_unknown") || Is("unk"))
            c.unk = F("value");
        else if (Is("links")) {
            for (int k = 0; k < 3; k++) c.links.push_back(S("link bone"));
        }
        else
            Warn("%s: unknown ikchain option \"%s\"", tk_.Where().c_str(), T().c_str());
    }
    m_.ikChains.push_back(c);
}

void Parser::CmdWeightlist(bool isDefault) {
    QcWeightList wl;
    wl.name = isDefault ? "default" : S("weightlist name");
    ForEachOption([&]() {
        if (Is("posweight")) {
            if (wl.bones.empty()) Fatal("%s: posweight before bone", tk_.Where().c_str());
            wl.posWeights.back() = F("posweight");
            return true;
        }
        wl.bones.push_back(T());
        float w = F("weight");
        wl.weights.push_back(w);
        wl.posWeights.push_back(w);
        return true;
    });
    for (auto& e : m_.weightLists)
        if (IEquals(e.name, wl.name)) {
            e = wl;
            return;
        }
    m_.weightLists.push_back(wl);
}

void Parser::ParseIKRule(QcIKRule& r) {
    r.chain = S("ikrule chain");
    std::string type = S("ikrule type");
    if (IEquals(type, "touch")) {
        r.type = IK_SELF;
        r.bone = S("bone");
    } else if (IEquals(type, "footstep")) {
        r.type = IK_GROUND;
        int c = m_.FindIKChain(r.chain);
        if (c >= 0) {
            r.height = m_.ikChains[c].height;
            r.floor = m_.ikChains[c].floor;
            r.radius = m_.ikChains[c].radius;
        }
        r.endHeight = -100;  // Respawn default: every retail footstep rule has endHeight -100
    } else if (IEquals(type, "attachment")) {
        r.type = IK_ATTACHMENT;
        r.attachment = S("attachment");
    } else if (IEquals(type, "release")) {
        r.type = IK_RELEASE;
    } else if (IEquals(type, "unlatch")) {
        r.type = IK_UNLATCH;
    } else {
        Fatal("%s: unknown ikrule type \"%s\"", tk_.Where().c_str(), type.c_str());
    }
    r.contact = -1;
    auto frameArg = [&]() -> int {
        std::string t = S("frame");
        if (!t.empty() && t[0] == '.') return -1;
        return std::atoi(t.c_str());
    };
    while (Avail()) {
        tk_.Get(false);
        if (Is("height"))
            r.height = F("height");
        else if (Is("target"))
            r.slot = I("slot");
        else if (Is("range")) {
            r.start = frameArg();
            r.peak = frameArg();
            r.tail = frameArg();
            r.end = frameArg();
        } else if (Is("floor"))
            r.floor = F("floor");
        else if (Is("pad"))
            r.radius = F("pad") / 2.f;
        else if (Is("radius"))
            r.radius = F("radius");
        else if (Is("contact"))
            r.contact = I("contact");
        else if (Is("usesequence")) {
            r.useSequence = true;
            r.useSource = false;
        } else if (Is("usesource")) {
            r.useSequence = false;
            r.useSource = true;
        } else if (Is("fakeorigin")) {
            r.pos = V3();
            r.hasFakeOrigin = true;
        } else if (Is("fakerotate")) {
            float a = F("pitch"), b = F("yaw"), c = F("roll");
            r.q = AngleQuaternion(QAngleToRadianEuler(a, b, c));
            r.hasFakeRotate = true;
        } else if (Is("bone"))
            r.bone = S("bone");
        else if (Is("endheight"))
            r.endHeight = F("endheight");
        else {
            tk_.Unget();
            break;
        }
    }
}

bool Parser::ParseCmdlistToken(std::vector<QcAnimCmd>& cmds) {
    QcAnimCmd c;
    if (Is("fixuploop")) {
        c.type = QcAnimCmd::FIXUPLOOP;
        c.i0 = I("start");
        c.i1 = I("end");
    } else if (StartsWithI(T(), "weightlist")) {
        c.type = QcAnimCmd::WEIGHTS;
        c.str = S("weightlist name");
    } else if (Is("subtract") || Is("presubtract")) {
        bool post = Is("subtract");
        c.type = QcAnimCmd::SUBTRACT;
        c.str = S("animation");
        c.i0 = I("frame");
        if (post) c.flags |= STUDIO_POST;
    } else if (Is("alignto")) {
        c.type = QcAnimCmd::AO;
        c.str = S("animation");
        c.motion = STUDIO_X | STUDIO_Y;
    } else if (Is("align")) {
        c.type = QcAnimCmd::AO;
        c.str = S("animation");
        std::string t = S("control");
        int ctrl;
        while ((ctrl = LookupControl(t)) != -1) {
            c.motion |= ctrl;
            t = S("control");
        }
        if (!c.motion) Fatal("%s: missing controls on align", tk_.Where().c_str());
        c.i0 = std::atoi(t.c_str());
        c.i1 = I("frame");
    } else if (Is("alignboneto")) {
        c.type = QcAnimCmd::AO;
        c.str2 = S("bone");
        c.str = S("animation");
        c.motion = STUDIO_X | STUDIO_Y;
    } else if (Is("match")) {
        c.type = QcAnimCmd::MATCH;
        c.str = S("animation");
    } else if (Is("matchblend")) {
        c.type = QcAnimCmd::MATCHBLEND;
        c.str = S("animation");
        c.i0 = I("srcframe");
        c.i1 = I("destframe");
        c.i2 = I("destpre");
        c.i3 = I("destpost");
    } else if (Is("worldspaceblend")) {
        c.type = QcAnimCmd::WORLDSPACEBLEND;
        c.str = S("animation");
    } else if (Is("worldspaceblendloop")) {
        c.type = QcAnimCmd::WORLDSPACEBLEND;
        c.str = S("animation");
        c.i0 = I("startframe");
        c.flags = 1;
    } else if (Is("rotateto")) {
        c.type = QcAnimCmd::ANGLE;
        c.f0 = F("angle");
    } else if (Is("ikrule") || Is("ikfixup")) {
        c.type = Is("ikrule") ? QcAnimCmd::IKRULE : QcAnimCmd::IKFIXUP;
        c.ikrule = std::make_shared<QcIKRule>();
        ParseIKRule(*c.ikrule);
    } else if (Is("walkframe")) {
        c.type = QcAnimCmd::MOTION;
        c.i0 = I("frame");
        while (Avail()) {
            tk_.Get(false);
            int ctrl = LookupControl(T());
            if (ctrl == -1) {
                tk_.Unget();
                break;
            }
            c.motion |= ctrl;
        }
    } else if (Is("walkalignto")) {
        c.type = QcAnimCmd::REFMOTION;
        c.i0 = I("frame");
        c.i1 = c.i0;
        c.str = S("animation");
        while (Avail()) {
            tk_.Get(false);
            int ctrl = LookupControl(T());
            if (ctrl == -1) {
                tk_.Unget();
                break;
            }
            c.motion |= ctrl;
        }
    } else if (Is("walkalign")) {
        c.type = QcAnimCmd::REFMOTION;
        c.i0 = I("frame");
        c.str = S("animation");
        std::string t;
        while (Avail()) {
            t = S("control");
            int ctrl = LookupControl(t);
            if (ctrl == -1) break;
            c.motion |= ctrl;
        }
        if (!c.motion) Fatal("%s: missing controls on walkalign", tk_.Where().c_str());
        c.i2 = std::atoi(t.c_str());  // ref frame
        c.i1 = I("srcframe");
    } else if (Is("derivative")) {
        c.type = QcAnimCmd::DERIVATIVE;
        c.f0 = F("scale");
    } else if (Is("noanimation") || Is("noanim_keepduration")) {
        c.type = QcAnimCmd::NOANIMATION;
        c.flags = Is("noanim_keepduration") ? 1 : 0;
    } else if (Is("lineardelta") || Is("splinedelta")) {
        c.type = QcAnimCmd::LINEARDELTA;
        c.flags = STUDIO_AL_POST | (Is("splinedelta") ? STUDIO_AL_SPLINE : 0);
    } else if (Is("compress")) {
        c.type = QcAnimCmd::COMPRESS;
        c.i0 = I("frames");
    } else if (Is("numframes")) {
        c.type = QcAnimCmd::NUMFRAMES;
        c.i0 = I("frames");
    } else if (Is("counterrotate")) {
        c.type = QcAnimCmd::COUNTERROTATE;
        c.str2 = S("bone");
    } else if (Is("counterrotateto")) {
        c.type = QcAnimCmd::COUNTERROTATE;
        c.hasTarget = true;
        c.angles = V3();
        c.str2 = S("bone");
    } else {
        return false;
    }
    cmds.push_back(c);
    return true;
}

bool Parser::ParseAnimationToken(QcAnimation& a) {
    if (Is("if")) {
        std::string t = S("condition");
        if (std::atoi(t.c_str()) == 0 && !IEquals(t, "true")) {
            if (tk_.Get(true) && T() == "{") {
                tk_.Unget();
                tk_.ReadBlock();
            }
        }
    } else if (Is("fps")) {
        a.fps = F("fps");
        if (a.fps <= 0) Fatal("%s: fps must be > 0", tk_.Where().c_str());
    } else if (Is("origin")) {
        a.adjust = V3();
    } else if (Is("rotate")) {
        a.rotation.z = DegToRad(F("angle") + 90.f);
    } else if (Is("angles")) {
        a.rotation.x = DegToRad(F("x"));
        a.rotation.y = DegToRad(F("y"));
        a.rotation.z = DegToRad(F("z") + 90.f);
    } else if (Is("scale")) {
        a.scale = F("scale");
    } else if (Is("noforceloop")) {
        a.flags |= STUDIO_NOFORCELOOP;
    } else if (StartsWithI(T(), "loop")) {
        a.flags |= STUDIO_LOOPING;
    } else if (StartsWithI(T(), "startl")) {
        a.loopRestart = I("startloop");
        a.flags |= STUDIO_LOOPING;
    } else if (Is("fudgeloop")) {
        a.fudgeLoop = true;
        a.flags |= STUDIO_LOOPING;
    } else if (StartsWithI(T(), "snap")) {
        a.flags |= STUDIO_SNAP;
    } else if (StartsWithI(T(), "frame")) {
        a.startFrame = I("start frame");
        a.endFrame = I("end frame");
        a.frameRangeSet = true;
    } else if (Is("post")) {
        a.flags |= STUDIO_POST;
    } else if (Is("noautoik")) {
        a.noAutoIK = true;
    } else if (Is("autoik")) {
        a.noAutoIK = false;
    } else if (Is("rm") || Is("rootmotion") || Is("framemovement")) {
        a.flags |= STUDIO_FRAMEMOVEMENT;
    } else if (Is("delta")) {
        // source frames are already local deltas (a decompiled delta animation); kept
        // out of the flags so it is not copied onto sequences like an authored option
        a.rawDelta = true;
    } else if (Is("defaultpose")) {
        // rsx marks animations that only hold the default pose; nothing to do
    } else if (ParseCmdlistToken(a.cmds)) {
    } else if (LookupControl(T()) != -1) {
        a.motionType |= LookupControl(T());
    } else {
        return false;
    }
    return true;
}

std::shared_ptr<QcAnimation> Parser::ImpliedAnimation(const std::string& seqName, const std::string& file) {
    auto a = std::make_shared<QcAnimation>();
    a->implied = true;
    a->name = "@" + seqName;
    a->file = file;
    a->adjust = m_.defaultAdjust;
    a->rotation = m_.defaultRotation;
    a->scale = 1.f;
    a->fps = 30.f;
    a->source = Load(file, false);
    a->sourceScale = m_.defaultScale;
    a->startFrame = a->source->startFrame;
    a->endFrame = a->source->startFrame + static_cast<int>(a->source->frames.size()) - 1;
    m_.animations.push_back(a);
    return a;
}

void Parser::CmdAnimation() {
    std::string name = S("animation name");
    int existing = m_.FindAnimation(name);
    if (existing >= 0) {
        if (!m_.animations[existing]->isOverride) Fatal("%s: duplicate animation name \"%s\"", tk_.Where().c_str(), name.c_str());
        // a $declareanimation placeholder is being defined now
        m_.animations.erase(m_.animations.begin() + existing);
    }
    auto a = std::make_shared<QcAnimation>();
    a->name = name;
    a->file = S("animation file");
    a->source = Load(a->file, false);
    a->sourceScale = m_.defaultScale;
    a->startFrame = a->source->startFrame;
    a->endFrame = a->source->startFrame + static_cast<int>(a->source->frames.size()) - 1;
    a->adjust = m_.defaultAdjust;
    a->rotation = m_.defaultRotation;
    a->scale = 1.f;
    a->fps = 30.f;
    m_.animations.push_back(a);
    ForEachOption([&]() { return ParseAnimationToken(*a); });
}

void Parser::CmdDeclareAnimation() {
    auto a = std::make_shared<QcAnimation>();
    a->name = S("animation name");
    a->isOverride = true;
    m_.animations.push_back(a);
}

void Parser::CmdSequence(bool declare) {
    QcSequence seq;
    seq.name = S("sequence name");
    if (declare) {
        seq.isOverride = true;
        m_.sequences.push_back(seq);
        return;
    }
    int existing = m_.FindSequence(seq.name);
    if (existing >= 0) {
        if (!m_.sequences[existing].isOverride) Fatal("%s: duplicate sequence name \"%s\"", tk_.Where().c_str(), seq.name.c_str());
    }
    std::vector<std::shared_ptr<QcAnimation>> anims;
    auto poseIndex = [&](const std::string& n) {
        int i = m_.FindPoseParam(n);
        if (i < 0) {
            QcPoseParam p;
            p.name = n;
            m_.poseParams.push_back(p);
            i = static_cast<int>(m_.poseParams.size() - 1);
        }
        return i;
    };
    ForEachOption([&]() -> bool {
        if (Is("event")) {
            QcEvent e;
            e.name = S("event name");
            e.frame = I("event frame");
            if (Avail()) {
                tk_.Get(false);
                if (T() == "}" && !tk_.Cur().quoted)
                    tk_.Unget();
                else
                    e.options = T();
            }
            seq.events.push_back(e);
            return true;
        }
        if (Is("activity")) {
            seq.activity = S("activity");
            seq.actWeight = I("weight");
            return true;
        }
        if (StartsWithI(T(), "ACT_")) {
            seq.activity = T();
            seq.actWeight = I("weight");
            return true;
        }
        if (Is("activitymodifier")) {
            QcSequence::ActivityModifier am;
            am.name = S("activity modifier");
            if (Avail()) {
                tk_.Get(false);
                if (Is("negate"))
                    am.negate = true;
                else
                    tk_.Unget();
            }
            seq.activityModifiers.push_back(am);
            return true;
        }
        if (Is("snap")) {
            seq.flags |= STUDIO_SNAP;
            return true;
        }
        if (Is("blendwidth")) {
            seq.groupSize[0] = I("blendwidth");
            return true;
        }
        if (Is("seqweights")) {
            // r2-model-compiler extension: Respawn sequences store a weight list that need
            // not match their animations' encoding weights
            seq.seqWeights = S("weightlist name");
            return true;
        }
        if (Is("posekeys")) {
            // r2-model-compiler extension: "posekeys <count> <values...>", explicit blend
            // positions for axis 0 then axis 1 (retail blends need not be evenly spaced,
            // e.g. a neutral pose at 0)
            int n = I("pose key count");
            seq.poseKeys.clear();
            for (int k = 0; k < n; k++) seq.poseKeys.push_back(F("pose key"));
            return true;
        }
        if (Is("blend")) {
            int i = seq.paramName[0].empty() ? 0 : 1;
            seq.paramName[i] = S("pose parameter");
            int j = poseIndex(seq.paramName[i]);
            seq.paramStart[i] = F("start");
            seq.paramEnd[i] = F("end");
            auto& pp = m_.poseParams[j];
            // studiomdl widens the pose range to cover every blend
            if (!pp.minSet && !pp.maxSet && pp.min == 0 && pp.max == 1) {
                pp.min = std::min(seq.paramStart[i], seq.paramEnd[i]);
                pp.max = std::max(seq.paramStart[i], seq.paramEnd[i]);
                pp.minSet = pp.maxSet = true;
            } else {
                pp.min = std::min(pp.min, std::min(seq.paramStart[i], seq.paramEnd[i]));
                pp.max = std::max(pp.max, std::max(seq.paramStart[i], seq.paramEnd[i]));
            }
            return true;
        }
        if (Is("calcblend")) {
            Warn("%s: calcblend is not supported, ignored", tk_.Where().c_str());
            S("pose");
            S("attachment");
            S("control");
            return true;
        }
        if (Is("blendref") || Is("blendcomp") || Is("blendcenter")) {
            S("animation");
            return true;
        }
        if (Is("node")) {
            seq.entryNode = seq.exitNode = S("node");
            return true;
        }
        if (Is("transition") || Is("rtransition")) {
            bool r = Is("rtransition");
            seq.entryNode = S("entry node");
            seq.exitNode = S("exit node");
            if (r) seq.nodeFlags |= 1;
            return true;
        }
        if (Is("entrynode")) {
            seq.entryNode = S("node");
            return true;
        }
        if (Is("exitnode")) {
            seq.exitNode = S("node");
            return true;
        }
        if (Is("exitphase")) {
            seq.exitPhase = F("exitphase");
            return true;
        }
        if (Is("delta")) {
            seq.flags |= STUDIO_DELTA | STUDIO_POST;
            return true;
        }
        if (Is("worldspace")) {
            seq.flags |= STUDIO_WORLD | STUDIO_POST;
            return true;
        }
        if (Is("post")) {
            seq.flags |= STUDIO_POST;
            return true;
        }
        if (Is("predelta")) {
            seq.flags |= STUDIO_DELTA;
            return true;
        }
        if (Is("local")) {
            seq.flags |= STUDIO_LOCAL;  // retail sets it on some blends without layers
            return true;
        }
        if (Is("autoplay")) {
            seq.flags |= STUDIO_AUTOPLAY;
            return true;
        }
        if (Is("fadein")) {
            seq.fadeIn = F("fadein");
            return true;
        }
        if (Is("fadeout")) {
            seq.fadeOut = F("fadeout");
            return true;
        }
        if (Is("realtime")) {
            seq.flags |= STUDIO_REALTIME;
            return true;
        }
        if (Is("hidden")) {
            seq.flags |= STUDIO_HIDDEN;
            return true;
        }
        if (Is("posecycle")) {
            seq.cyclePose = S("pose parameter");
            poseIndex(seq.cyclePose);
            seq.flags |= STUDIO_CYCLEPOSE;
            return true;
        }
        if (Is("ikresetmask")) {
            seq.ikResetMask = I("mask");
            return true;
        }
        if (Is("unk_c4")) {
            seq.unkC4 = I("value");
            return true;
        }
        if (Is("bbox")) {
            seq.bboxMin = V3();
            seq.bboxMax = V3();
            seq.bboxSet = true;
            return true;
        }
        if (Is("addlayer")) {
            QcAutoLayer l;
            l.sequence = S("sequence");
            seq.layers.push_back(l);
            return true;
        }
        if (Is("iklock")) {
            QcIKLock l;
            l.chain = S("chain");
            l.posWeight = F("posweight");
            l.localQWeight = F("localqweight");
            seq.ikLocks.push_back(l);
            return true;
        }
        if (Is("keyvalues")) {
            CmdKeyValues(seq.keyValues);
            return true;
        }
        if (Is("blendlayer")) {
            QcAutoLayer l;
            l.sequence = S("sequence");
            l.start = F("start");
            l.peak = F("peak");
            l.tail = F("tail");
            l.end = F("end");
            while (Avail()) {
                tk_.Get(false);
                if (Is("xfade"))
                    l.flags |= STUDIO_AL_XFADE;
                else if (Is("spline"))
                    l.flags |= STUDIO_AL_SPLINE;
                else if (Is("noblend"))
                    l.flags |= STUDIO_AL_NOBLEND;
                else if (Is("poseparameter")) {
                    l.flags |= STUDIO_AL_POSE;
                    l.pose = S("pose parameter");
                    poseIndex(l.pose);
                } else if (Is("local")) {
                    l.flags |= STUDIO_AL_LOCAL;
                    seq.flags |= STUDIO_LOCAL;
                } else if (Is("realtime")) {
                    l.flags |= 0x8000;  // STUDIO_AL_REALTIME
                } else if (Is("flags")) {
                    l.flags |= I("layer flags");  // r2c extension: raw bits (e.g. 0x2000)
                } else {
                    tk_.Unget();
                    break;
                }
            }
            seq.layers.push_back(l);
            return true;
        }
        if (!anims.empty()) {
            // animation options in the sequence block apply to the first animation (as in
            // studiomdl) and their flags to the sequence as well
            int saved = anims[0]->flags;
            anims[0]->flags = 0;
            bool used = ParseAnimationToken(*anims[0]);
            int added = anims[0]->flags;
            anims[0]->flags = saved | added;
            seq.flags |= added;
            if (used) return true;
        }
        // otherwise an animation reference: an existing $animation or a file
        int n = m_.FindAnimation(T());
        if (n >= 0)
            anims.push_back(m_.animations[n]);
        else
            anims.push_back(ImpliedAnimation(seq.name, T()));
        return true;
    });
    if (anims.empty()) Fatal("%s: sequence \"%s\" has no animations", tk_.Where().c_str(), seq.name.c_str());
    int numblends = static_cast<int>(anims.size());
    if (seq.groupSize[0] == 0) {
        if (numblends < 4) {
            seq.groupSize[0] = numblends;
            seq.groupSize[1] = 1;
        } else {
            int i = static_cast<int>(std::sqrt(static_cast<float>(numblends)));
            if (i * i == numblends) {
                seq.groupSize[0] = seq.groupSize[1] = i;
            } else
                Fatal("%s: non-square (%d) number of blends without \"blendwidth\" set", tk_.Where().c_str(), numblends);
        }
    } else {
        seq.groupSize[1] = numblends / seq.groupSize[0];
        if (seq.groupSize[0] * seq.groupSize[1] != numblends)
            Fatal("%s: missing animation blends, expected %d found %d", tk_.Where().c_str(), seq.groupSize[0] * seq.groupSize[1], numblends);
    }
    for (int i = 0; i < numblends; i++) {
        if (i > 0 && anims[i]->implied) {
            // CopyAnimationSettings
            anims[i]->fps = anims[0]->fps;
            anims[i]->adjust = anims[0]->adjust;
            anims[i]->scale = anims[0]->scale;
            anims[i]->rotation = anims[0]->rotation;
            anims[i]->motionType = anims[0]->motionType;
            for (auto& c : anims[0]->cmds) anims[i]->cmds.push_back(c);
        }
        // Options written inline in the sequence block apply to the sequence too; a named
        // $animation's own options do not (retail: looping animations under non-looping
        // sequences, e.g. pete_scripted_r1 CASUAL_idleA).
        if (anims[i]->implied) seq.flags |= anims[i]->flags;
        seq.animations.push_back(anims[i]->name);
    }
    if (existing >= 0)
        m_.sequences[existing] = seq;
    else
        m_.sequences.push_back(seq);
}

void Parser::CmdKeyValues(std::string& out) {
    // studiomdl's Option_KeyValues: the block as text, top-level tokens bare and nested
    // ones quoted, each followed by a space; retail key values keep exactly this layout
    if (!tk_.Get(true) || T() != "{") Fatal("%s: expected '{' for keyvalues", tk_.Where().c_str());
    std::string kv = "mdlkeyvalue\n{\n";
    int depth = 1;
    while (tk_.Get(true)) {
        if (!tk_.Cur().quoted && T() == "}") {
            if (--depth <= 0) break;
            kv += " }\n";
        } else if (!tk_.Cur().quoted && T() == "{") {
            kv += "{\n";
            depth++;
        } else if (depth > 1) {
            kv += "\"" + T() + "\" ";
        } else {
            kv += T() + " ";
        }
    }
    if (depth > 0) Fatal("%s: keyvalue block missing matching braces", tk_.Where().c_str());
    out += kv + "}\n";
}

int Parser::ParseContents(int base) {
    int add = 0, remove = 0;
    do {
        std::string t = S("contents");
        if (!t.empty() && t.find_first_not_of("0123456789") == std::string::npos) {
            // a number (r2c extension): the exact contents mask
            base = std::stoi(t);
            add = remove = 0;
        } else if (IEquals(t, "grate")) {
            add |= CONTENTS_GRATE;
            remove |= CONTENTS_SOLID;
        } else if (IEquals(t, "ladder"))
            add |= CONTENTS_LADDER;
        else if (IEquals(t, "solid"))
            add |= CONTENTS_SOLID;
        else if (IEquals(t, "monster"))
            add |= CONTENTS_MONSTER;
        else if (IEquals(t, "notsolid"))
            remove |= CONTENTS_SOLID;
    } while (Avail());
    return (base | add) & ~remove;
}

void Parser::CmdCollision(bool joints) {
    auto& c = m_.collision;
    c.present = true;
    c.joints = joints;
    c.file = S("collision model file");
    c.source = Load(c.file, false);
    c.sourceScale = m_.defaultScale;
    if (tk_.Get(true)) {
        if (T() != "{" || tk_.Cur().quoted) {
            tk_.Unget();
            return;
        }
    } else
        return;
    auto args = [&](int n) {
        std::vector<std::string> a;
        while (static_cast<int>(a.size()) < n && Avail()) {
            tk_.Get(false);
            a.push_back(T());
        }
        return a;
    };
    auto jointProp = [&](const std::string& name) -> QcCollisionJoint& {
        for (auto& j : c.jointProps)
            if (IEquals(j.name, name)) return j;
        c.jointProps.push_back(QcCollisionJoint());
        c.jointProps.back().name = name;
        return c.jointProps.back();
    };
    auto fl = [](const std::vector<std::string>& a, size_t i, float def = 0) {
        return i < a.size() ? static_cast<float>(std::atof(a[i].c_str())) : def;
    };
    while (tk_.Get(true)) {
        if (T() == "}") return;
        std::string cmd = T();
        if (IEquals(cmd, "$mass"))
            c.mass = fl(args(1), 0);
        else if (IEquals(cmd, "$automass"))
            c.automass = true;
        else if (IEquals(cmd, "$inertia"))
            c.inertia = fl(args(1), 0);
        else if (IEquals(cmd, "$damping"))
            c.damping = fl(args(1), 0);
        else if (IEquals(cmd, "$rotdamping"))
            c.rotdamping = fl(args(1), 0);
        else if (IEquals(cmd, "$drag"))
            c.drag = fl(args(1), 0);
        else if (IEquals(cmd, "$rollingdrag"))
            args(1);
        else if (IEquals(cmd, "$concave"))
            c.concave = true;
        else if (IEquals(cmd, "$nomapcollision"))
            // r2c extension: some retail models carry PHY data but no map-collision block
            c.noMapCollision = true;
        else if (IEquals(cmd, "$concaveperjoint")) {
            // splits each joint into convex pieces without the "concave" editparams key
            // (retail jointed models with multi-piece solids never carry it)
            c.concavePerJoint = true;
        } else if (IEquals(cmd, "$maxconvexpieces"))
            c.maxConvexPieces = static_cast<int>(fl(args(1), 0));
        else if (IEquals(cmd, "$masscenter")) {
            auto a = args(3);
            c.hasMassCenter = true;
            c.massCenter = Vec3(fl(a, 0), fl(a, 1), fl(a, 2));
        } else if (IEquals(cmd, "$jointskip")) {
            auto a = args(1);
            if (!a.empty()) c.skipBones.push_back(a[0]);
        } else if (IEquals(cmd, "$jointmerge")) {
            auto a = args(2);
            if (a.size() == 2) c.merges.push_back({a[0], a[1]});
        } else if (IEquals(cmd, "$rootbone")) {
            auto a = args(1);
            if (!a.empty()) c.rootBone = a[0];
        } else if (IEquals(cmd, "$jointconstrain")) {
            auto a = args(6);
            if (a.size() < 5) {
                Warn("%s: bad $jointconstrain", tk_.Where().c_str());
                continue;
            }
            QcJointConstraint jc;
            jc.joint = a[0];
            jc.axis = std::tolower(static_cast<unsigned char>(a[1][0])) - 'x';
            if (IEquals(a[2], "free"))
                jc.type = 0;
            else if (IEquals(a[2], "fixed"))
                jc.type = 1;
            else if (IEquals(a[2], "limit"))
                jc.type = 2;
            else {
                Warn("unknown joint type %s", a[2].c_str());
                continue;
            }
            jc.min = fl(a, 3);
            jc.max = fl(a, 4);
            jc.friction = a.size() >= 6 ? fl(a, 5) : 1.0f;
            if (jc.axis < 0 || jc.axis > 2 || jc.min > jc.max) Fatal("invalid joint constraint for %s", jc.joint.c_str());
            c.constraints.push_back(jc);
        } else if (IEquals(cmd, "$jointinertia")) {
            auto a = args(2);
            if (a.size() == 2) jointProp(a[0]).inertia = fl(a, 1);
        } else if (IEquals(cmd, "$jointdamping")) {
            auto a = args(2);
            if (a.size() == 2) jointProp(a[0]).damping = fl(a, 1);
        } else if (IEquals(cmd, "$jointrotdamping")) {
            auto a = args(2);
            if (a.size() == 2) jointProp(a[0]).rotdamping = fl(a, 1);
        } else if (IEquals(cmd, "$jointmassbias")) {
            auto a = args(2);
            if (a.size() == 2) jointProp(a[0]).massBias = fl(a, 1);
        } else if (IEquals(cmd, "$noselfcollisions"))
            c.noSelfCollisions = true;
        else if (IEquals(cmd, "$jointcollide")) {
            auto a = args(2);
            if (a.size() == 2) c.collisionPairs.push_back({a[0], a[1]});
        } else if (IEquals(cmd, "$animatedfriction")) {
            auto a = args(5);
            if (a.size() == 5) {
                c.animatedFriction = true;
                c.afMin = fl(a, 0);
                c.afMax = fl(a, 1);
                c.afIn = fl(a, 2);
                c.afHold = fl(a, 3);
                c.afOut = fl(a, 4);
            }
        } else if (IEquals(cmd, "$assumeworldspace"))
            c.assumeWorldSpace = true;
        else if (IEquals(cmd, "$remove2d"))
            c.remove2d = true;
        else if (IEquals(cmd, "$weldposition"))
            c.weldPosition = fl(args(1), 0);
        else if (IEquals(cmd, "$weldnormal"))
            c.weldNormal = fl(args(1), 0);
        else {
            Warn("%s: unknown command %s in collision series", tk_.Where().c_str(), cmd.c_str());
            args(16);
        }
    }
}

void Parser::CmdCollisionText() {
    std::string& out = m_.collision.text;
    if (!tk_.Get(true) || T() != "{") return;
    int level = 1;
    while (tk_.Get(true)) {
        if (!tk_.Cur().quoted && T() == "}") {
            level--;
            if (level <= 0) break;
            out += " }\n";
        } else if (!tk_.Cur().quoted && T() == "{") {
            out += "{";
            level++;
        } else if (level > 1) {
            out += "\"" + T() + "\" ";
        } else {
            out += T() + " ";
        }
    }
}

// Field indices into QcJiggleBone::v (order of mstudiojigglebone_t after flags).
enum JiggleField {
    J_LENGTH, J_TIPMASS, J_YAWSTIFF, J_YAWDAMP, J_PITCHSTIFF, J_PITCHDAMP, J_ALONGSTIFF, J_ALONGDAMP, J_ANGLELIMIT,
    J_MINYAW, J_MAXYAW, J_YAWFRICTION, J_YAWBOUNCE, J_MINPITCH, J_MAXPITCH, J_PITCHFRICTION, J_PITCHBOUNCE,
    J_BASEMASS, J_BASESTIFF, J_BASEDAMP, J_BASEMINLEFT, J_BASEMAXLEFT, J_BASELEFTFRICTION, J_BASEMINUP, J_BASEMAXUP,
    J_BASEUPFRICTION, J_BASEMINFWD, J_BASEMAXFWD, J_BASEFWDFRICTION
};

// $proceduralbones "file": a .vrd file holds quaternion-interpolation helpers and aim
// constraints, any other file axis-interpolation bones (studiomdl Load_ProceduralBones).
void Parser::CmdProceduralBones() {
    std::string name = S("procedural bones file");
    std::string path = ResolveSource(name, "vrd");
    if (path.empty()) Fatal("%s: could not find procedural bones file \"%s\"", tk_.Where().c_str(), name.c_str());
    std::ifstream in(std::filesystem::u8path(path));
    std::vector<std::vector<std::string>> lines;
    for (std::string line; std::getline(in, line);) {
        std::vector<std::string> t;
        for (size_t i = 0; i < line.size();) {
            if (std::isspace(static_cast<unsigned char>(line[i]))) { i++; continue; }
            if (line[i] == '"') {
                size_t e = line.find('"', i + 1);
                if (e == std::string::npos) e = line.size();
                t.push_back(line.substr(i + 1, e - i - 1));
                i = e + 1;
            } else {
                size_t e = i;
                while (e < line.size() && !std::isspace(static_cast<unsigned char>(line[e]))) e++;
                t.push_back(line.substr(i, e - i));
                i = e;
            }
        }
        lines.push_back(t);
    }
    auto num = [&](const std::vector<std::string>& t, size_t i) {
        if (i >= t.size()) Fatal("%s: missing value in procedural bone line", path.c_str());
        return static_cast<float>(std::atof(t[i].c_str()));
    };
    auto vec = [&](const std::vector<std::string>& t, size_t i) { return Vec3(num(t, i), num(t, i + 1), num(t, i + 2)); };
    auto degEuler = [&](const std::vector<std::string>& t, size_t i) {
        return RadianEuler(DegToRad(num(t, i)), DegToRad(num(t, i + 1)), DegToRad(num(t, i + 2)));
    };
    helperBase_ = Vec3();
    if (IEquals(GetExtension(path), "vrd")) {
        QcQuatInterpBone* helper = nullptr;
        QcAimAtBone* aim = nullptr;
        for (auto& t : lines) {
            if (t.empty()) continue;
            const std::string& cmd = t[0];
            if (IEquals(cmd, "<helper>") && t.size() >= 5) {
                QcQuatInterpBone b;
                b.bone = t[1];
                b.parent = t[2];
                b.controlParent = t[3];
                b.control = t[4];
                m_.quatInterpBones.push_back(b);
                helper = &m_.quatInterpBones.back();
                aim = nullptr;
            } else if (IEquals(cmd, "<aimconstraint>") && t.size() >= 4) {
                QcAimAtBone b;
                b.bone = t[1];
                b.parent = t[2];
                b.aim = t[3];
                m_.aimAtBones.push_back(b);
                aim = &m_.aimAtBones.back();
                helper = nullptr;
            } else if (aim && IEquals(cmd, "<aimvector>")) {
                aim->aimVector = Normalize(vec(t, 1));
            } else if (aim && IEquals(cmd, "<upvector>")) {
                aim->upVector = Normalize(vec(t, 1));
            } else if (aim && IEquals(cmd, "<basepos>")) {
                aim->basePos = vec(t, 1);
            } else if (helper && IEquals(cmd, "<display>")) {
                helper->percentage = num(t, 4) / 100.f;
            } else if (helper && IEquals(cmd, "<basepos>")) {
                helperBase_ = vec(t, 1);  // added to each following trigger's position
            } else if (helper && IEquals(cmd, "<trigger>")) {
                QcQuatInterpTrigger tr;
                tr.tolerance = DegToRad(num(t, 1));
                tr.trigger = AngleQuaternion(degEuler(t, 2));
                tr.quat = AngleQuaternion(degEuler(t, 5));
                tr.pos = helperBase_ + vec(t, 8);
                helper->triggers.push_back(tr);
            } else {
                Fatal("%s: unable to parse procedural bone data \"%s\"", path.c_str(), cmd.c_str());
            }
        }
    } else {
        QcAxisInterpBone* axis = nullptr;
        Vec3 base;
        for (auto& t : lines) {
            if (t.empty()) continue;
            const std::string& cmd = t[0];
            if (IEquals(cmd, "bone") && t.size() >= 6) {
                QcAxisInterpBone b;
                b.bone = t[1];
                b.control = t[3];
                b.axis = std::atoi(t[5].c_str()) - 1;  // the file uses 1..3
                m_.axisInterpBones.push_back(b);
                axis = &m_.axisInterpBones.back();
            } else if (axis && IEquals(cmd, "basepos")) {
                base = vec(t, 1);
            } else if (axis && IEquals(cmd, "axis") && t.size() >= 8) {
                int j = std::atoi(t[1].c_str());
                if (j < 0 || j > 5) Fatal("%s: axis index %d out of range", path.c_str(), j);
                axis->pos[j] = base + vec(t, 2);
                // the file lists roll, pitch, yaw
                axis->quat[j] = AngleQuaternion(QAngleToRadianEuler(num(t, 6), num(t, 7), num(t, 5)));
            }
            // "version", "proceduralbones", "display", "type", "end": nothing to keep
        }
    }
}

// $uipanel "name" [hash] { parent "bone" | vertex <parent> x y z | face a b c [ccw] |
//                          bounds x0 x1 x2 x3 y0 y1 y2 y3 }
void Parser::CmdUIPanel() {
    QcUIPanel p;
    p.name = S("panel name");
    if (!tk_.Get(true)) Fatal("%s: expected '{' for $uipanel", tk_.Where().c_str());
    if (T() != "{") {
        p.hash = static_cast<uint32_t>(std::strtoul(T().c_str(), nullptr, 0));
        p.hasHash = true;
        if (!tk_.Get(true) || T() != "{") Fatal("%s: expected '{' for $uipanel", tk_.Where().c_str());
    }
    while (tk_.Get(true)) {
        if (T() == "}") break;
        if (Is("parent")) {
            p.parents.push_back(S("bone"));
        } else if (Is("vertex")) {
            QcUIPanel::Vertex v;
            v.parent = I("parent slot");
            v.pos = V3();
            p.vertices.push_back(v);
        } else if (Is("face")) {
            std::array<int, 3> f = {I("a"), I("b"), I("c")};
            if (Avail()) {
                tk_.Get(false);
                if (Is("ccw")) {
                    // the winding bitfield is 32 bits wide
                    if (p.faces.size() >= 32) Fatal("$uipanel %s: only the first 32 faces can be ccw", p.name.c_str());
                    p.ccw |= 1u << p.faces.size();
                }
                else tk_.Unget();
            }
            p.faces.push_back(f);
        } else if (Is("bounds")) {
            std::array<float, 8> b;
            for (auto& v : b) v = F("bound");
            p.bounds.push_back(b);
        } else {
            Fatal("%s: unknown $uipanel option \"%s\"", tk_.Where().c_str(), T().c_str());
        }
    }
    if (p.parents.empty() || p.parents.size() > 2) Fatal("$uipanel %s: needs one or two parent bones", p.name.c_str());
    if (p.vertices.size() > 256) Fatal("$uipanel %s: at most 256 vertices", p.name.c_str());
    if (p.faces.size() > 64) Fatal("$uipanel %s: at most 64 faces", p.name.c_str());
    if (p.bounds.size() != p.faces.size()) Fatal("$uipanel %s: needs one bounds line per face", p.name.c_str());
    m_.uiPanels.push_back(p);
}

void Parser::CmdJiggleBone() {
    QcJiggleBone j;
    j.bone = S("bone");
    // defaults from studiomdl
    j.v[J_LENGTH] = 10;
    j.v[J_TIPMASS] = 1;
    j.v[J_YAWSTIFF] = 100;
    j.v[J_YAWDAMP] = 0;
    j.v[J_PITCHSTIFF] = 100;
    j.v[J_PITCHDAMP] = 0;
    j.v[J_ALONGSTIFF] = 100;
    j.v[J_ALONGDAMP] = 0;
    j.v[J_BASEMASS] = 0;
    j.v[J_BASESTIFF] = 100;
    j.v[J_BASEDAMP] = 0;
    j.v[J_BASEMINLEFT] = j.v[J_BASEMINUP] = j.v[J_BASEMINFWD] = -100;  // stored even without a base spring
    j.v[J_BASEMAXLEFT] = j.v[J_BASEMAXUP] = j.v[J_BASEMAXFWD] = 100;
    if (!tk_.Get(true) || T() != "{") Fatal("%s: expected '{' for $jigglebone", tk_.Where().c_str());
    auto readSection = [&](const std::function<void(const std::string&)>& fn) {
        if (!tk_.Get(true) || T() != "{") Fatal("%s: expected '{'", tk_.Where().c_str());
        while (tk_.Get(true)) {
            if (T() == "}") return;
            fn(ToLower(T()));
        }
    };
    auto deg = [&]() { return DegToRad(F("angle")); };
    while (tk_.Get(true)) {
        if (T() == "}") break;
        if (Is("is_flexible") || Is("is_rigid")) {
            bool flex = Is("is_flexible");
            j.flags |= flex ? 0x01 : 0x02;
            readSection([&](const std::string& k) {
                if (k == "length") j.v[J_LENGTH] = F("length");
                else if (k == "tip_mass") j.v[J_TIPMASS] = F("tip_mass");
                else if (k == "yaw_stiffness") j.v[J_YAWSTIFF] = F("v");
                else if (k == "yaw_damping") j.v[J_YAWDAMP] = F("v");
                else if (k == "pitch_stiffness") j.v[J_PITCHSTIFF] = F("v");
                else if (k == "pitch_damping") j.v[J_PITCHDAMP] = F("v");
                else if (k == "along_stiffness") j.v[J_ALONGSTIFF] = F("v");
                else if (k == "along_damping") j.v[J_ALONGDAMP] = F("v");
                else if (k == "allow_length_flex") j.flags |= 0x20;
                else if (k == "angle_constraint") { j.flags |= 0x10; j.v[J_ANGLELIMIT] = deg(); }
                else if (k == "yaw_constraint") { j.flags |= 0x04; j.v[J_MINYAW] = deg(); j.v[J_MAXYAW] = deg(); }
                else if (k == "yaw_friction") j.v[J_YAWFRICTION] = F("v");
                else if (k == "yaw_bounce") j.v[J_YAWBOUNCE] = F("v");
                else if (k == "pitch_constraint") { j.flags |= 0x08; j.v[J_MINPITCH] = deg(); j.v[J_MAXPITCH] = deg(); }
                else if (k == "pitch_friction") j.v[J_PITCHFRICTION] = F("v");
                else if (k == "pitch_bounce") j.v[J_PITCHBOUNCE] = F("v");
                else Warn("%s: unknown jigglebone option %s", tk_.Where().c_str(), k.c_str());
            });
        } else if (Is("has_base_spring")) {
            j.flags |= 0x40;
            readSection([&](const std::string& k) {
                if (k == "stiffness") j.v[J_BASESTIFF] = F("v");
                else if (k == "damping") j.v[J_BASEDAMP] = F("v");
                else if (k == "base_mass") j.v[J_BASEMASS] = F("v");
                else if (k == "left_constraint") { j.v[J_BASEMINLEFT] = F("v"); j.v[J_BASEMAXLEFT] = F("v"); }
                else if (k == "left_friction") j.v[J_BASELEFTFRICTION] = F("v");
                else if (k == "up_constraint") { j.v[J_BASEMINUP] = F("v"); j.v[J_BASEMAXUP] = F("v"); }
                else if (k == "up_friction") j.v[J_BASEUPFRICTION] = F("v");
                else if (k == "forward_constraint") { j.v[J_BASEMINFWD] = F("v"); j.v[J_BASEMAXFWD] = F("v"); }
                else if (k == "forward_friction") j.v[J_BASEFWDFRICTION] = F("v");
                else Warn("%s: unknown jigglebone base option %s", tk_.Where().c_str(), k.c_str());
            });
        } else if (Is("is_boing")) {
            Warn("%s: is_boing jigglebones are not supported in v53, ignored", tk_.Where().c_str());
            readSection([&](const std::string&) { while (Avail()) tk_.Get(false); });
        } else
            Fatal("%s: unknown jigglebone section \"%s\"", tk_.Where().c_str(), T().c_str());
    }
    m_.jiggleBones.push_back(j);
}

void Parser::Register() {
    auto& h = handlers_;
    h["$modelname"] = [&] {
        std::string n = S("model name");
        if (!n.empty() && (n[0] == '/' || n[0] == '\\')) n = n.substr(1);
        m_.modelName = n;  // as written; output paths normalise their own slashes
    };
    h["$cd"] = [&] {
        std::string d = S("directory");
        cd_ = JoinPath(cd_, d);
    };
    h["$pushd"] = [&] {
        cdStack_.push_back(cd_);
        cd_ = JoinPath(cd_, S("directory"));
    };
    h["$popd"] = [&] {
        if (!cdStack_.empty()) {
            cd_ = cdStack_.back();
            cdStack_.pop_back();
        }
    };
    h["$cdmaterials"] = [&] {
        while (Avail()) {
            std::string d = FixSlashes(S("directory"), '\\');
            if (!d.empty() && d.back() != '\\') d += '\\';
            if (!d.empty() && d[0] == '\\') d = d.substr(1);
            m_.cdMaterials.push_back(d);
        }
    };
    h["$cdtexture"] = [&] { tk_.SkipRestOfCommand(); };
    h["$scale"] = [&] { m_.defaultScale = F("scale"); };
    h["$origin"] = [&] {
        m_.defaultAdjust = V3();
        if (Avail()) m_.defaultRotation.z = DegToRad(F("rotation") + 90.f);
    };
    h["$upaxis"] = [&] {
        std::string a = ToLower(S("axis"));
        if (a == "x") { m_.defaultRotation.x = 0; m_.defaultRotation.y = kPi / 2; }
        else if (a == "-x") { m_.defaultRotation.x = 0; m_.defaultRotation.y = -kPi / 2; }
        else if (a == "y") { m_.defaultRotation.x = kPi / 2; m_.defaultRotation.y = 0; }
        else if (a == "-y") { m_.defaultRotation.x = -kPi / 2; m_.defaultRotation.y = 0; }
        else if (a == "z" || a == "-z") { m_.defaultRotation.x = 0; m_.defaultRotation.y = 0; }
        else Fatal("%s: unknown $upaxis \"%s\"", tk_.Where().c_str(), a.c_str());
    };
    h["$body"] = [&] { CmdBody(); };
    h["$model"] = [&] { CmdModel(); };
    h["$bodygroup"] = [&] { CmdBodygroup(); };
    h["$lod"] = [&] { CmdLOD(false); };
    h["$shadowlod"] = [&] { CmdLOD(true); };
    h["$texturegroup"] = [&] { CmdTextureGroup(); };
    h["$renamematerial"] = [&] {
        std::string a = S("from"), b = S("to");
        m_.renameMaterials.push_back({a, b});
    };
    h["$staticprop"] = [&] { m_.staticProp = true; };
    h["$autocenter"] = [&] { m_.autoCenter = true; };
    h["$opaque"] = [&] {
        m_.flags |= STUDIOHDR_FLAGS_FORCE_OPAQUE;
        m_.flags &= ~STUDIOHDR_FLAGS_TRANSLUCENT_TWOPASS;
    };
    h["$mostlyopaque"] = [&] {
        if (!(m_.flags & STUDIOHDR_FLAGS_FORCE_OPAQUE)) m_.flags |= STUDIOHDR_FLAGS_TRANSLUCENT_TWOPASS;
    };
    h["$obsolete"] = [&] { m_.flags |= STUDIOHDR_FLAGS_OBSOLETE; };
    h["$noforcedfade"] = [&] { m_.flags |= STUDIOHDR_FLAGS_NO_FORCED_FADE; };
    h["$forcephonemecrossfade"] = [&] { m_.flags |= STUDIOHDR_FLAGS_FORCE_PHONEME_CROSSFADE; };
    h["$ambientboost"] = [&] { m_.flags |= STUDIOHDR_FLAGS_AMBIENT_BOOST; };
    h["$donotcastshadows"] = [&] { m_.flags |= STUDIOHDR_FLAGS_DO_NOT_CAST_SHADOWS; };
    h["$casttextureshadows"] = [&] { m_.flags |= STUDIOHDR_FLAGS_CAST_TEXTURE_SHADOWS; };
    h["$subd"] = [&] { m_.flags |= STUDIOHDR_FLAGS_SUBDIVISION_SURFACE; };
    h["$usevertexcolor"] = [&] { m_.flags |= STUDIOHDR_FLAGS_USES_VERTEX_COLOR; };
    h["$useextratexcoords"] = [&] { m_.flags |= STUDIOHDR_FLAGS_USES_UV2; };
    h["$constantdirectionallight"] = [&] {
        m_.flags |= STUDIOHDR_FLAGS_CONSTANT_DIRECTIONAL_LIGHT_DOT;
        float v = F("dot");
        m_.constDirectionalLightDot = static_cast<uint8_t>(std::clamp(v, 0.f, 1.f) * 255);
    };
    h["$fadedistance"] = [&] { m_.fadeDistance = F("distance"); };
    h["$maxeyedeflection"] = [&] { m_.maxEyeDeflection = std::cos(DegToRad(F("angle"))); };
    h["$minlod"] = [&] { m_.minLod = I("lod"); };
    h["$allowrootlods"] = [&] { m_.allowRootLods = I("count"); };
    h["$surfaceprop"] = [&] { m_.surfaceProp = S("surface property"); };
    h["$jointsurfaceprop"] = [&] {
        std::string j = S("joint"), p = S("surface property");
        for (auto& e : m_.jointSurfaceProps)
            if (IEquals(e.first, j)) {
                e.second = p;
                return;
            }
        m_.jointSurfaceProps.push_back({j, p});
    };
    h["$contents"] = [&] { m_.contents = ParseContents(m_.contents); };
    h["$jointcontents"] = [&] {
        std::string j = S("joint");
        int c = ParseContents(CONTENTS_SOLID);
        for (auto& e : m_.jointContents)
            if (IEquals(e.first, j)) {
                e.second = c;
                return;
            }
        m_.jointContents.push_back({j, c});
    };
    h["$mass"] = [&] { m_.headerMass = F("mass"); };
    h["$node"] = [&] { m_.nodes.push_back(S("node name")); };
    h["$localnodeunk"] = [&] { m_.localNodeUnk = I("value"); };
    h["$sourcefile"] = [&] { m_.sourceFiles.push_back(S("source file")); };
    h["$illumposition"] = [&] {
        // studiomdl rotates these by its built-in 90 degrees
        float x = F("x"), y = F("y"), z = F("z");
        m_.illumPosition = Vec3(-y, x, z);
        m_.illumSet = true;
        if (Avail()) m_.illumBone = S("bone");
    };
    h["$eyeposition"] = [&] {
        float x = F("x"), y = F("y"), z = F("z");
        m_.eyePosition = Vec3(-y, x, z);
    };
    h["$bbox"] = [&] {
        m_.bbox[0] = V3();
        m_.bbox[1] = V3();
        m_.bboxSet = true;
    };
    h["$cbox"] = [&] {
        m_.cbox[0] = V3();
        m_.cbox[1] = V3();
        m_.cboxSet = true;
    };
    h["$gamma"] = [&] { F("gamma"); };
    h["$definebone"] = [&] { CmdDefineBone(); };
    h["$renamebone"] = [&] {
        std::string a = S("from"), b = S("to");
        m_.renameBones.push_back({a, b});
    };
    h["$hierarchy"] = h["$heirarchy"] = [&] {
        std::string c = S("child"), p = S("parent");
        m_.hierarchy.push_back({c, p});
    };
    h["$bonemerge"] = [&] { m_.boneMerge.push_back(S("bone")); };
    h["$autohitboxes"] = [&] { m_.autoHitboxes = true; };
    h["$srcbonetransform"] = [&] {
        QcModelDef::SrcBoneTransform t;
        t.bone = S("bone");
        for (float& v : t.pre) v = F("pretransform");
        for (float& v : t.post) v = F("posttransform");
        m_.srcBoneTransforms.push_back(t);
    };
    h["$alwayscollapse"] = [&] { m_.alwaysCollapse.push_back(S("bone")); };
    h["$collapsebones"] = [&] { m_.collapseBones = true; };
    h["$realignbones"] = [&] { m_.realignBones = true; };
    h["$forcerealign"] = [&] {
        QcModelDef::ForcedRealign fr;
        fr.bone = S("bone");
        S("skip");
        { Vec3 d = V3(); fr.rot = RadianEuler(DegToRad(d.x), DegToRad(d.y), DegToRad(d.z)); }
        m_.forcedRealign.push_back(fr);
    };
    h["$screenalign"] = [&] {
        std::string b = S("bone");
        int f = 0x08;
        if (Avail()) {
            std::string t = S("type");
            if (IEquals(t, "cylinder")) f = 0x10;
        }
        m_.screenAlign.push_back({b, f});
    };
    h["$limitrotation"] = [&] {
        m_.limitRotation.push_back(S("bone"));
        while (Avail()) tk_.Get(false);
    };
    h["$jigglebone"] = [&] { CmdJiggleBone(); };
    h["$proceduralbones"] = [&] { CmdProceduralBones(); };
    h["$uipanel"] = [&] { CmdUIPanel(); };
    h["$attachment"] = [&] { CmdAttachment(); };
    h["$hboxset"] = [&] {
        QcHitboxSet s;
        s.name = S("hitbox set name");
        m_.hitboxSets.push_back(s);
    };
    h["$hbox"] = [&] { CmdHBox(); };
    h["$hgroup"] = [&] { tk_.SkipRestOfCommand(); };
    h["$skipboneinbbox"] = [&] { m_.skipBoneInBBox.push_back(S("bone")); };
    h["$poseparameter"] = [&] { CmdPoseParameter(); };
    h["$ikchain"] = [&] { CmdIKChain(); };
    h["$ikautoplaylock"] = [&] {
        QcIKLock l;
        l.chain = S("chain");
        l.posWeight = F("posweight");
        l.localQWeight = F("localqweight");
        m_.ikAutoplayLocks.push_back(l);
    };
    h["$weightlist"] = [&] { CmdWeightlist(false); };
    h["$defaultweightlist"] = [&] { CmdWeightlist(true); };
    h["$animation"] = [&] { CmdAnimation(); };
    h["$sequence"] = [&] { CmdSequence(false); };
    h["$declaresequence"] = [&] { CmdSequence(true); };
    h["$declareanimation"] = [&] { CmdDeclareAnimation(); };
    // the name after "models/" is kept as written, as retail stores it
    h["$includemodel"] = [&] { m_.includeModels.push_back("models/" + S("model")); };
    h["$keyvalues"] = [&] { CmdKeyValues(m_.keyValues); };
    h["$collisionmodel"] = [&] { CmdCollision(false); };
    h["$collisionjoints"] = [&] { CmdCollision(true); };
    h["$collisiontext"] = [&] { CmdCollisionText(); };
    h["$sectionframes"] = [&] {
        m_.sectionFrames = I("frames");
        if (Avail()) m_.sectionMinFrames = I("minframes");
    };
    h["$legacymovement"] = [&] { m_.legacyMovement = true; };
    h["$animblocksize"] = [&] { tk_.SkipRestOfCommand(); };
    h["$bonesaveframe"] = [&] { tk_.SkipRestOfCommand(); };
    h["$unkstring"] = [&] { m_.unkString = S("string"); };
    h["$bonefollower"] = [&] { m_.boneFollowers.push_back(S("bone")); };
    h["$lockbonelengths"] = [&] {};
    h["$unlockdefinebones"] = [&] {};
    h["$lockdefinebones"] = [&] {};
    h["$externaltextures"] = [&] {};
    h["$cliptotextures"] = [&] {};
    h["$usedetailedweights"] = [&] { Warn("$usedetailedweights is an Apex Legends option; ignored"); };
    h["$maxverts"] = [&] { tk_.SkipRestOfCommand(); };
    h["$controller"] = [&] {
        tk_.SkipRestOfCommand();
        Warn("$controller is not supported by Titanfall 2 models; ignored");
    };
    h["$root"] = [&] { S("bone"); };
    h["$cmdlist"] = [&] {
        tk_.SkipRestOfCommand();
        Warn("$cmdlist is not supported; ignored");
    };
}

void Parser::Run(const std::string& path) {
    m_.qcPath = path;
    m_.qcDir = DirName(std::filesystem::absolute(std::filesystem::u8path(path)).u8string());
    cd_ = m_.qcDir;
    Register();
    tk_.LoadFile(path);
    while (tk_.Get(true)) {
        std::string cmd = ToLower(T());
        if (cmd == "$include") {
            std::string inc = S("include file");
            std::string p = JoinPath(cd_, inc);
            if (!FileExists(p)) p = JoinPath(m_.qcDir, inc);
            if (!FileExists(p)) Fatal("%s: could not find include file \"%s\"", tk_.Where().c_str(), inc.c_str());
            tk_.IncludeFile(p);
            continue;
        }
        if (cmd == "$definevariable") {
            std::string name = S("variable name");
            std::string val = S("value");
            tk_.Variables()[name] = val;
            continue;
        }
        if (cmd == "$definemacro") {
            // $definemacro name arg1 arg2 then body lines, each ending in a double backslash
            Tokenizer::Macro mac;
            std::string name = S("macro name");
            while (Avail()) {
                tk_.Get(false);
                if (T() == "\\\\") break;
                mac.params.push_back(T());
            }
            // body: continue while lines end with "\\"
            bool more = true;
            while (more && tk_.Get(true)) {
                more = false;
                Token first = tk_.Cur();
                std::vector<Token> line{first};
                while (tk_.Available()) {
                    tk_.Get(false);
                    if (tk_.Text() == "\\\\") {
                        more = true;
                        break;
                    }
                    line.push_back(tk_.Cur());
                }
                line[0].newlineBefore = true;
                for (auto& t : line) mac.body.push_back(t);
            }
            tk_.Macros()[name] = mac;
            continue;
        }
        if (cmd == "$macro") {
            std::string name = S("macro name");
            auto it = tk_.Macros().find(name);
            if (it == tk_.Macros().end()) Fatal("%s: unknown macro \"%s\"", tk_.Where().c_str(), name.c_str());
            std::map<std::string, std::string> args;
            for (auto& p : it->second.params) args[p] = Avail() ? S("macro argument") : "";
            std::vector<Token> body = it->second.body;
            for (auto& t : body) {
                for (auto& kv : args) {
                    std::string key = "$" + kv.first + "$";
                    size_t at;
                    while ((at = t.text.find(key)) != std::string::npos) t.text.replace(at, key.size(), kv.second);
                }
            }
            tk_.InsertTokens(body);
            continue;
        }
        auto it = handlers_.find(cmd);
        if (it == handlers_.end()) {
            if (!cmd.empty() && cmd[0] == '$')
                Warn("%s: unknown command \"%s\", skipped", tk_.Where().c_str(), T().c_str());
            else
                Warn("%s: unexpected token \"%s\"", tk_.Where().c_str(), T().c_str());
            tk_.SkipRestOfCommand();
            continue;
        }
        it->second();
    }
    // resolve declared-but-never-defined placeholders
    for (auto& s : m_.sequences)
        if (s.isOverride) Warn("sequence \"%s\" was declared but never defined", s.name.c_str());
}

}  // namespace

std::unique_ptr<QcModelDef> ParseQC(const std::string& path, const QcOptions& opt) {
    auto m = std::make_unique<QcModelDef>();
    Parser p(*m, opt);
    p.Run(path);
    return m;
}

}  // namespace r2c
