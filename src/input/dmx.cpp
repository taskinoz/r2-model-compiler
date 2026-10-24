// DMX model reader (binary encodings 1-5 and keyvalues2), as written by Blender
// Source Tools and Valve tools. Builds the same Source structure as the SMD reader:
// skeleton from DmeModel.jointList, bind pose from each joint's DmeTransform, meshes
// from DmeDag/DmeMesh face sets, and animation from DmeChannelsClip logs.
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <map>
#include <sstream>
#include <unordered_map>
#include <variant>

#include "core/log.h"
#include "core/strings.h"
#include "input/source.h"

namespace r2c {

void BalanceWeights(std::vector<BoneWeight>& w, size_t maxCount);

namespace {

enum class AT {
    Unknown, Element, Int, Float, Bool, String, Binary, Time, Color, Vec2, Vec3, Vec4, Angle, Quat, Matrix
};

struct Attr {
    AT type = AT::Unknown;
    bool array = false;
    std::vector<int> elems;          // element indices (-1 null)
    std::vector<std::string> elemGuids;  // keyvalues2 references resolved later
    std::vector<int> ints;
    std::vector<float> floats;       // flattened for vector/color/time types
    std::vector<std::string> strings;
    int Components() const {
        switch (type) {
        case AT::Vec2: return 2;
        case AT::Vec3: case AT::Angle: return 3;
        case AT::Vec4: case AT::Quat: case AT::Color: return 4;
        case AT::Matrix: return 16;
        default: return 1;
        }
    }
};

struct Elem {
    std::string type, name, guid;
    std::map<std::string, Attr> attrs;
    const Attr* Get(const std::string& n) const {
        auto it = attrs.find(n);
        return it == attrs.end() ? nullptr : &it->second;
    }
};

struct Doc {
    std::vector<Elem> elems;
    int root = 0;
    const Elem* E(int i) const { return i >= 0 && i < static_cast<int>(elems.size()) ? &elems[i] : nullptr; }
    const Elem* Child(const Elem* e, const std::string& attr) const {
        if (!e) return nullptr;
        const Attr* a = e->Get(attr);
        if (!a || a->type != AT::Element || a->elems.empty()) return nullptr;
        return E(a->elems[0]);
    }
    std::vector<const Elem*> Children(const Elem* e, const std::string& attr) const {
        std::vector<const Elem*> out;
        if (!e) return out;
        const Attr* a = e->Get(attr);
        if (!a || a->type != AT::Element) return out;
        for (int i : a->elems)
            if (E(i)) out.push_back(E(i));
        return out;
    }
};

// ------------------------------------------------------------------ binary
struct Reader {
    const std::vector<uint8_t>& d;
    size_t p = 0;
    explicit Reader(const std::vector<uint8_t>& data) : d(data) {}
    void Need(size_t n) {
        if (p + n > d.size()) Fatal("DMX: unexpected end of file");
    }
    template <typename T>
    T Get() {
        Need(sizeof(T));
        T v;
        std::memcpy(&v, &d[p], sizeof(T));
        p += sizeof(T);
        return v;
    }
    std::string Str() {
        size_t s = p;
        while (p < d.size() && d[p]) p++;
        std::string r(reinterpret_cast<const char*>(&d[s]), p - s);
        p++;
        return r;
    }
};

AT TypeFromId(int ver, int id, bool& array) {
    array = false;
    if (ver >= 9) {
        static const AT v9[] = {AT::Unknown, AT::Element, AT::Int, AT::Float, AT::Bool, AT::String, AT::Binary, AT::Time,
                                AT::Color, AT::Vec2, AT::Vec3, AT::Vec4, AT::Angle, AT::Quat, AT::Matrix, AT::Int, AT::Int};
        if (id >= 32) {
            array = true;
            id -= 32;
        }
        if (id >= 1 && id <= 16) return v9[id];
        Fatal("DMX: unknown attribute type %d", id);
    }
    // v1/v2: ObjectID at 7 (treated like a binary 16-byte id); v3+ Time at 7
    static const AT single[] = {AT::Unknown, AT::Element, AT::Int, AT::Float, AT::Bool, AT::String, AT::Binary,
                                AT::Time, AT::Color, AT::Vec2, AT::Vec3, AT::Vec4, AT::Angle, AT::Quat, AT::Matrix};
    if (id >= 15 && id <= 28) {
        array = true;
        return single[id - 14];
    }
    if (id >= 1 && id <= 14) return single[id];
    (void)ver;
    Fatal("DMX: unknown attribute type %d", id);
}

template <typename DictFn>
void ReadAttr(Reader& r, int ver, Attr& at, DictFn&& dictStr) {
    int id = r.Get<uint8_t>();
    int rawId = id;
    at.type = TypeFromId(ver, id, at.array);
    if (ver <= 2 && rawId % 14 == 7) at.type = AT::Binary;  // ObjectID in v1/v2
    bool isU64 = ver >= 9 && (rawId == 15 || rawId == 47);
    bool isU8 = ver >= 9 && (rawId == 16 || rawId == 48);
    int count = at.array ? r.Get<int32_t>() : 1;
    for (int k = 0; k < count; k++) {
        if (isU64) {
            at.ints.push_back(static_cast<int>(r.Get<uint64_t>()));
            continue;
        }
        if (isU8) {
            at.ints.push_back(r.Get<uint8_t>());
            continue;
        }
        switch (at.type) {
        case AT::Element: {
            int idx = r.Get<int32_t>();
            if (idx == -2) {
                r.Str();
                idx = -1;
            }
            at.elems.push_back(idx);
            break;
        }
        case AT::Int: at.ints.push_back(r.Get<int32_t>()); break;
        case AT::Float: at.floats.push_back(r.Get<float>()); break;
        case AT::Bool: at.ints.push_back(r.Get<uint8_t>()); break;
        case AT::String: at.strings.push_back((ver >= 4 && !at.array) ? dictStr() : r.Str()); break;
        case AT::Binary: {
            int len = (ver <= 2 && rawId % 14 == 7) ? 16 : r.Get<int32_t>();
            r.Need(len);
            r.p += len;
            break;
        }
        case AT::Time: at.floats.push_back(r.Get<int32_t>() / 10000.f); break;
        case AT::Color:
            for (int c = 0; c < 4; c++) at.floats.push_back(r.Get<uint8_t>() / 255.f);
            break;
        default: {
            int n = at.Components();
            for (int c = 0; c < n; c++) at.floats.push_back(r.Get<float>());
        }
        }
    }
}

Doc ParseBinary(const std::vector<uint8_t>& data, size_t headerEnd, int ver) {
    Doc doc;
    Reader r(data);
    r.p = headerEnd;
    std::vector<std::string> dict;
    bool shortIdx = ver == 2 || ver == 3 || ver == 4;
    bool shortCount = ver == 2 || ver == 3;
    if (ver >= 9) {
        // prefix attributes (names inline, not in the dictionary)
        int np = r.Get<int32_t>();
        for (int i = 0; i < np; i++) {
            int na = r.Get<int32_t>();
            for (int a = 0; a < na; a++) {
                r.Str();
                Attr at;
                ReadAttr(r, ver, at, [&]() { return r.Str(); });
            }
        }
    }
    if (ver >= 2) {
        int n = shortCount ? r.Get<uint16_t>() : r.Get<int32_t>();
        for (int i = 0; i < n; i++) dict.push_back(r.Str());
    }
    auto dictStr = [&]() -> std::string {
        if (ver < 2) return r.Str();
        int i = shortIdx ? r.Get<uint16_t>() : r.Get<int32_t>();
        if (i == -1 || (shortIdx && i == 0xFFFF)) return std::string();  // null string
        if (i < 0 || i >= static_cast<int>(dict.size()))
            Fatal("DMX: bad string index %d (dictionary has %d) at offset %d", i, static_cast<int>(dict.size()), static_cast<int>(r.p));
        return dict[i];
    };
    int ne = r.Get<int32_t>();
    doc.elems.resize(ne);
    for (int i = 0; i < ne; i++) {
        doc.elems[i].type = dictStr();
        doc.elems[i].name = ver >= 4 ? dictStr() : r.Str();
        r.Need(16);
        r.p += 16;
    }
    for (int i = 0; i < ne; i++) {
        int na = r.Get<int32_t>();
        for (int a = 0; a < na; a++) {
            std::string name = dictStr();
            Attr at;
            try {
                ReadAttr(r, ver, at, dictStr);
            } catch (const CompileError& e) {
                Fatal("%s (element %d \"%s\" attribute \"%s\")", e.what(), i, doc.elems[i].name.c_str(), name.c_str());
            }
            doc.elems[i].attrs[name] = std::move(at);
        }
    }
    doc.root = 0;
    return doc;
}

// ------------------------------------------------------------------ keyvalues2
struct KV2 {
    const std::string& s;
    size_t p = 0;
    explicit KV2(const std::string& str) : s(str) {}
    bool Skip() {
        while (p < s.size()) {
            if (std::isspace(static_cast<unsigned char>(s[p])) || s[p] == ',') {
                p++;
                continue;
            }
            if (s[p] == '/' && p + 1 < s.size() && s[p + 1] == '/') {
                while (p < s.size() && s[p] != '\n') p++;
                continue;
            }
            return true;
        }
        return false;
    }
    std::string Tok() {
        if (!Skip()) Fatal("DMX: unexpected end of keyvalues2 text");
        if (s[p] == '{' || s[p] == '}' || s[p] == '[' || s[p] == ']') return std::string(1, s[p++]);
        if (s[p] == '"') {
            p++;
            std::string out;
            while (p < s.size() && s[p] != '"') {
                if (s[p] == '\\' && p + 1 < s.size()) {
                    p++;
                    char c = s[p++];
                    out += c == 'n' ? '\n' : (c == 't' ? '\t' : c);
                    continue;
                }
                out += s[p++];
            }
            p++;
            return out;
        }
        size_t st = p;
        while (p < s.size() && !std::isspace(static_cast<unsigned char>(s[p])) && s[p] != '{' && s[p] != '}') p++;
        return s.substr(st, p - st);
    }
    std::string Peek() {
        size_t save = p;
        std::string t = Tok();
        p = save;
        return t;
    }
};

AT TypeFromName(const std::string& n, bool& array) {
    std::string t = n;
    array = false;
    if (EndsWithI(t, "_array")) {
        array = true;
        t = t.substr(0, t.size() - 6);
    }
    static const std::pair<const char*, AT> names[] = {
        {"element", AT::Element}, {"int", AT::Int}, {"float", AT::Float}, {"bool", AT::Bool}, {"string", AT::String},
        {"binary", AT::Binary}, {"time", AT::Time}, {"color", AT::Color}, {"vector2", AT::Vec2}, {"vector3", AT::Vec3},
        {"vector4", AT::Vec4}, {"qangle", AT::Angle}, {"angle", AT::Angle}, {"quaternion", AT::Quat}, {"matrix", AT::Matrix},
        {"elementid", AT::String}, {"uint64", AT::Int}, {"uint8", AT::Int},
    };
    for (auto& e : names)
        if (IEquals(t, e.first)) return e.second;
    return AT::Unknown;
}

void ParseValue(Attr& at, const std::string& v) {
    switch (at.type) {
    case AT::Int: at.ints.push_back(std::atoi(v.c_str())); break;
    case AT::Bool: at.ints.push_back(v == "1" || IEquals(v, "true")); break;
    case AT::String: at.strings.push_back(v); break;
    case AT::Element: at.elemGuids.push_back(v); break;
    case AT::Float: case AT::Time: at.floats.push_back(std::strtof(v.c_str(), nullptr)); break;
    case AT::Color: {
        std::istringstream ss(v);
        for (int c = 0; c < 4; c++) {
            float f = 0;
            ss >> f;
            at.floats.push_back(f / 255.f);
        }
        break;
    }
    case AT::Binary: case AT::Unknown: break;
    default: {
        std::istringstream ss(v);
        for (int c = 0; c < at.Components(); c++) {
            float f = 0;
            ss >> f;
            at.floats.push_back(f);
        }
    }
    }
}

Doc ParseKV2(const std::string& text, size_t headerEnd) {
    Doc doc;
    KV2 kv(text);
    kv.p = headerEnd;
    std::unordered_map<std::string, int> byGuid;
    std::function<int(const std::string&)> element = [&](const std::string& type) -> int {
        int idx = static_cast<int>(doc.elems.size());
        doc.elems.push_back(Elem());
        doc.elems[idx].type = type;
        if (kv.Tok() != "{") Fatal("DMX: expected '{'");
        while (true) {
            std::string name = kv.Tok();
            if (name == "}") break;
            std::string tname = kv.Tok();
            bool isArray = false;
            AT t = TypeFromName(tname, isArray);
            if (t == AT::Unknown && !isArray) {
                // inline element: "name" "DmeType" { ... }
                int child = element(tname);
                Attr a;
                a.type = AT::Element;
                a.elems.push_back(child);
                doc.elems[idx].attrs[name] = a;
                continue;
            }
            Attr a;
            a.type = t;
            a.array = isArray;
            if (!isArray) {
                std::string v = kv.Tok();
                if (name == "id" && IEquals(tname, "elementid")) {
                    doc.elems[idx].guid = v;
                    byGuid[v] = idx;
                    continue;
                }
                if (name == "name" && t == AT::String) doc.elems[idx].name = v;
                ParseValue(a, v);
            } else {
                if (kv.Tok() != "[") Fatal("DMX: expected '['");
                while (kv.Peek() != "]") {
                    std::string v = kv.Tok();
                    if (t == AT::Element && kv.Peek() == "{") {
                        a.elems.push_back(element(v));
                        a.elemGuids.push_back("");
                        continue;
                    }
                    if (t == AT::Element && IEquals(v, "element")) v = kv.Tok();
                    ParseValue(a, v);
                }
                kv.Tok();
            }
            doc.elems[idx].attrs[name] = std::move(a);
        }
        return idx;
    };
    while (kv.Skip()) {
        std::string type = kv.Tok();
        element(type);
    }
    // resolve element references
    for (auto& e : doc.elems)
        for (auto& kva : e.attrs) {
            Attr& a = kva.second;
            if (a.type != AT::Element) continue;
            if (!a.elemGuids.empty() && a.elems.size() < a.elemGuids.size()) {
                // pure reference list
                std::vector<int> resolved;
                size_t ei = 0;
                for (auto& g : a.elemGuids) {
                    if (g.empty()) {
                        resolved.push_back(ei < a.elems.size() ? a.elems[ei++] : -1);
                        continue;
                    }
                    auto it = byGuid.find(g);
                    resolved.push_back(it == byGuid.end() ? -1 : it->second);
                }
                a.elems = resolved;
            }
        }
    doc.root = 0;
    return doc;
}

// ------------------------------------------------------------------ model extraction
Quat QuatOf(const Attr* a, size_t i = 0) {
    if (!a || a->floats.size() < (i + 1) * 4) return Quat();
    return Quat(a->floats[i * 4], a->floats[i * 4 + 1], a->floats[i * 4 + 2], a->floats[i * 4 + 3]);
}
Vec3 Vec3Of(const Attr* a, size_t i = 0) {
    if (!a || a->floats.size() < (i + 1) * 3) return Vec3();
    return Vec3(a->floats[i * 3], a->floats[i * 3 + 1], a->floats[i * 3 + 2]);
}

Mat3x4 TransformOf(const Doc& doc, const Elem* t) {
    if (!t) return Identity();
    return QuaternionMatrix(QuatOf(t->Get("orientation")), Vec3Of(t->Get("position")));
}

}  // namespace

std::shared_ptr<Source> LoadDMX(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) Fatal("could not open DMX \"%s\"", path.c_str());
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::string head(data.begin(), data.begin() + std::min<size_t>(data.size(), 256));
    size_t he = head.find("-->");
    if (head.rfind("<!--", 0) != 0 || he == std::string::npos) Fatal("%s: not a DMX file", path.c_str());
    std::istringstream hs(head.substr(4, he - 4));
    std::string w, enc, fmt;
    int encVer = 0, fmtVer = 0;
    hs >> w >> w >> enc >> encVer >> w >> fmt >> fmtVer;
    size_t headerEnd = he + 3;
    Doc doc;
    if (enc == "binary") {
        while (headerEnd < data.size() && data[headerEnd] != 0) headerEnd++;
        headerEnd++;
        if (encVer < 1 || (encVer > 5 && encVer != 9)) Fatal("%s: unsupported binary DMX version %d", path.c_str(), encVer);
        doc = ParseBinary(data, headerEnd, encVer);
    } else if (enc == "keyvalues2") {
        std::string text(data.begin(), data.end());
        doc = ParseKV2(text, headerEnd);
    } else {
        Fatal("%s: unsupported DMX encoding \"%s\"", path.c_str(), enc.c_str());
    }
    const Elem* root = doc.E(doc.root);
    const Elem* model = doc.Child(root, "skeleton");
    if (!model) model = doc.Child(root, "model");
    if (!model) Fatal("%s: no DmeModel (skeleton/model) in DMX", path.c_str());

    auto src = std::make_shared<Source>();
    src->path = path;

    // joints: jointList order defines weight indices; parents from the children tree
    std::vector<const Elem*> joints = doc.Children(model, "jointList");
    std::map<const Elem*, int> jointIndex;
    std::map<const Elem*, int> parentOf;
    std::function<void(const Elem*, int)> walk = [&](const Elem* e, int parentJoint) {
        int me = parentJoint;
        auto it = std::find(joints.begin(), joints.end(), e);
        if (it != joints.end() || IEquals(e->type, "DmeJoint")) {
            if (it == joints.end()) {
                joints.push_back(e);
                it = joints.end() - 1;
            }
            parentOf[e] = parentJoint;
            me = static_cast<int>(it - joints.begin());
        }
        for (auto* c : doc.Children(e, "children")) walk(c, me);
    };
    bool modelIsJoint = !joints.empty() && joints[0] == model;  // Source 2 style
    for (auto* c : doc.Children(model, "children")) walk(c, modelIsJoint ? 0 : -1);
    if (joints.empty()) {
        // mesh without a skeleton: one implicit root
        SrcBone b;
        b.name = "root";
        src->bones.push_back(b);
    }
    for (size_t i = 0; i < joints.size(); i++) {
        SrcBone b;
        b.name = joints[i]->name;
        b.parent = parentOf.count(joints[i]) ? parentOf[joints[i]] : -1;
        src->bones.push_back(b);
        jointIndex[joints[i]] = static_cast<int>(i);
    }
    // bind pose from each joint's transform (parent relative)
    std::vector<BoneFrame> bind(src->bones.size());
    std::vector<Mat3x4> world(src->bones.size());
    for (size_t i = 0; i < joints.size(); i++) {
        Mat3x4 m = TransformOf(doc, doc.Child(joints[i], "transform"));
        bind[i].pos = m.Origin();
        bind[i].rot = MatrixAngles(m);
    }
    for (size_t i = 0; i < src->bones.size(); i++) {
        Mat3x4 m = AngleMatrix(bind[i].rot, bind[i].pos);
        world[i] = src->bones[i].parent >= 0 ? Concat(world[src->bones[i].parent], m) : m;
    }
    src->frames.push_back(bind);

    // meshes: every DmeDag with a DmeMesh shape
    std::function<void(const Elem*, Mat3x4)> meshes = [&](const Elem* e, Mat3x4 parentWorld) {
        Mat3x4 here = parentWorld;
        if (IEquals(e->type, "DmeDag") || IEquals(e->type, "DmeJoint")) {
            if (jointIndex.count(e))
                here = world[jointIndex[e]];
            else
                here = Concat(parentWorld, TransformOf(doc, doc.Child(e, "transform")));
        }
        const Elem* shape = doc.Child(e, "shape");
        if (shape && IEquals(shape->type, "DmeMesh")) {
            const Elem* vd = doc.Child(shape, "currentState");
            if (!vd) {
                auto bs = doc.Children(shape, "baseStates");
                if (!bs.empty()) vd = bs[0];
            }
            if (vd) {
                auto attr = [&](std::initializer_list<const char*> names) -> const Attr* {
                    for (auto* n : names)
                        if (auto* a = vd->Get(n)) return a;
                    return nullptr;
                };
                const Attr* pos = attr({"positions", "position$0"});
                const Attr* posIdx = attr({"positionsIndices", "position$0Indices"});
                const Attr* nrm = attr({"normals", "normal$0"});
                const Attr* nrmIdx = attr({"normalsIndices", "normal$0Indices"});
                const Attr* uv = attr({"textureCoordinates", "texcoord$0"});
                const Attr* uvIdx = attr({"textureCoordinatesIndices", "texcoord$0Indices"});
                const Attr* uv2 = attr({"textureCoordinates1", "texcoord$1"});
                const Attr* uv2Idx = attr({"textureCoordinates1Indices", "texcoord$1Indices"});
                const Attr* col = attr({"colors", "color$0", "VertexPaint"});
                const Attr* colIdx = attr({"colorsIndices", "color$0Indices", "VertexPaintIndices"});
                const Attr* jw = attr({"jointWeights", "blendweights$0"});
                const Attr* ji = attr({"jointIndices", "blendindices$0"});
                const Attr* jc = vd->Get("jointCount");
                const Attr* flipV = vd->Get("flipVCoordinates");
                bool flip = flipV && !flipV->ints.empty() && flipV->ints[0];
                int jointCount = jc && !jc->ints.empty() ? jc->ints[0] : 0;
                if (!pos || !posIdx) Fatal("%s: mesh %s has no positions", path.c_str(), e->name.c_str());
                // a DmeDag listed in jointList (Source 2 layout) is only rigid when it has no weights
                bool rigid = jointIndex.count(e) > 0 && !(jw && ji && jointCount > 0);
                int rigidBone = rigid ? jointIndex[e] : 0;
                for (auto* fs : doc.Children(shape, "faceSets")) {
                    const Elem* mat = doc.Child(fs, "material");
                    std::string mtl = "default";
                    if (mat) {
                        const Attr* mn = mat->Get("mtlName");
                        mtl = mn && !mn->strings.empty() ? mn->strings[0] : mat->name;
                    }
                    mtl = FixSlashes(mtl, '/');
                    int matIndex = -1;
                    for (size_t m = 0; m < src->materials.size(); m++)
                        if (IEquals(src->materials[m], StripExtension(mtl))) matIndex = static_cast<int>(m);
                    if (matIndex < 0) {
                        src->materials.push_back(StripExtension(mtl));
                        matIndex = static_cast<int>(src->materials.size() - 1);
                    }
                    const Attr* faces = fs->Get("faces");
                    if (!faces) continue;
                    std::vector<int> poly;
                    auto makeVert = [&](int loop) {
                        SrcVertex v;
                        int pi = posIdx->ints[loop];
                        Vec3 p = Vec3Of(pos, pi);
                        v.pos = Transform(here, p);
                        if (nrm && nrmIdx && loop < static_cast<int>(nrmIdx->ints.size()))
                            v.normal = Normalize(Rotate(here, Vec3Of(nrm, nrmIdx->ints[loop])));
                        if (uv && uvIdx && loop < static_cast<int>(uvIdx->ints.size())) {
                            int ui = uvIdx->ints[loop];
                            v.uv = Vec2(uv->floats[ui * 2], uv->floats[ui * 2 + 1]);
                            if (flip) v.uv.y = 1.f - v.uv.y;
                        }
                        if (uv2 && uv2Idx && loop < static_cast<int>(uv2Idx->ints.size())) {
                            int ui = uv2Idx->ints[loop];
                            v.uv2 = Vec2(uv2->floats[ui * 2], uv2->floats[ui * 2 + 1]);
                            if (flip) v.uv2.y = 1.f - v.uv2.y;
                            v.hasUV2 = true;
                        }
                        if (col && colIdx && loop < static_cast<int>(colIdx->ints.size())) {
                            int ci = colIdx->ints[loop];
                            int comps = col->Components();
                            for (int c = 0; c < 4; c++) (&v.color.x)[c] = c < comps ? col->floats[ci * comps + c] : 1.f;
                            v.hasColor = true;
                        }
                        if (jw && ji && jointCount > 0 && !rigid) {
                            for (int k = 0; k < jointCount; k++) {
                                float wgt = jw->floats[pi * jointCount + k];
                                int bi = ji->ints[pi * jointCount + k];
                                if (wgt > 0 && bi >= 0 && bi < static_cast<int>(src->bones.size())) v.weights.push_back({bi, wgt});
                            }
                            BalanceWeights(v.weights, kMaxWeightsPerVertex);
                        }
                        if (v.weights.empty()) v.weights.push_back({rigidBone, 1.f});
                        return v;
                    };
                    for (int idx : faces->ints) {
                        if (idx >= 0) {
                            poly.push_back(idx);
                            continue;
                        }
                        // triangle fan; studiomdl's winding flip as for SMD (a, c, b)
                        for (size_t k = 1; k + 1 < poly.size(); k++) {
                            SrcFace face;
                            face.material = matIndex;
                            int base = static_cast<int>(src->vertices.size());
                            src->vertices.push_back(makeVert(poly[0]));
                            src->vertices.push_back(makeVert(poly[k]));
                            src->vertices.push_back(makeVert(poly[k + 1]));
                            face.v[0] = base;
                            face.v[1] = base + 2;
                            face.v[2] = base + 1;
                            src->faces.push_back(face);
                        }
                        poly.clear();
                    }
                }
            }
        }
        for (auto* c : doc.Children(e, "children")) meshes(c, here);
    };
    for (auto* c : doc.Children(model, "children")) meshes(c, Identity());
    // a mesh-bearing "model" element can differ from "skeleton"
    const Elem* meshModel = doc.Child(root, "model");
    if (meshModel && meshModel != model)
        for (auto* c : doc.Children(meshModel, "children")) meshes(c, Identity());

    // Source 2 layout lists the DmeModel itself as joint 0; drop it unless vertices use it
    if (modelIsJoint && !src->bones.empty()) {
        bool used = false;
        for (auto& v : src->vertices)
            for (auto& w : v.weights)
                if (w.bone == 0) used = true;
        if (!used) {
            src->bones.erase(src->bones.begin());
            for (auto& b : src->bones) b.parent = b.parent <= 0 ? -1 : b.parent - 1;
            for (auto& v : src->vertices)
                for (auto& w : v.weights) w.bone -= 1;
            for (auto& fr : src->frames) fr.erase(fr.begin());
            joints.erase(joints.begin());
            bind.erase(bind.begin());
            // children of the model become roots: fold the model transform into them
            Mat3x4 mt = TransformOf(doc, doc.Child(model, "transform"));
            for (size_t i = 0; i < src->bones.size(); i++)
                if (src->bones[i].parent == -1) {
                    Mat3x4 m = Concat(mt, AngleMatrix(bind[i].rot, bind[i].pos));
                    bind[i].pos = m.Origin();
                    bind[i].rot = MatrixAngles(m);
                    for (auto& fr : src->frames) fr[i] = bind[i];
                }
            jointIndex.clear();
            for (size_t i = 0; i < joints.size(); i++) jointIndex[joints[i]] = static_cast<int>(i);
            modelIsJoint = false;
        }
    }

    // animation: first clip in animationList
    const Elem* animList = doc.Child(root, "animationList");
    auto clips = doc.Children(animList, "animations");
    if (!clips.empty()) {
        const Elem* clip = clips[0];
        float fps = 30.f;
        if (auto* fr = clip->Get("frameRate")) {
            if (!fr->ints.empty())
                fps = static_cast<float>(fr->ints[0]);
            else if (!fr->floats.empty())
                fps = fr->floats[0];
        }
        float duration = 0;
        if (const Elem* tf = doc.Child(clip, "timeFrame")) {
            if (auto* dur = tf->Get("duration"); dur && !dur->floats.empty())
                duration = dur->floats[0];
            else if (auto* dt = tf->Get("durationTime"); dt && !dt->ints.empty())
                duration = dt->ints[0] / 10000.f;
        }
        // map joint transforms -> bone
        std::map<const Elem*, int> transformBone;
        for (size_t i = 0; i < joints.size(); i++)
            if (const Elem* t = doc.Child(joints[i], "transform")) transformBone[t] = static_cast<int>(i);
        struct Track {
            std::vector<float> t;
            std::vector<float> v;
        };
        std::map<int, Track> posTracks, rotTracks;
        float maxTime = duration;
        for (auto* ch : doc.Children(clip, "channels")) {
            const Elem* to = doc.Child(ch, "toElement");
            const Attr* ta = ch->Get("toAttribute");
            if (!to || !ta || ta->strings.empty() || !transformBone.count(to)) continue;
            int b = transformBone[to];
            const Elem* log = doc.Child(ch, "log");
            auto layers = doc.Children(log, "layers");
            if (layers.empty()) continue;
            const Attr* times = layers[0]->Get("times");
            const Attr* values = layers[0]->Get("values");
            if (!times || !values) continue;
            Track tr;
            if (!times->floats.empty())
                tr.t = times->floats;
            else
                for (int iv : times->ints) tr.t.push_back(iv / 10000.f);
            tr.v = values->floats;
            if (!tr.t.empty()) maxTime = std::max(maxTime, tr.t.back());
            if (ta->strings[0] == "position")
                posTracks[b] = tr;
            else if (ta->strings[0] == "orientation")
                rotTracks[b] = tr;
        }
        if (!posTracks.empty() || !rotTracks.empty()) {
            int numFrames = static_cast<int>(std::lround(maxTime * fps)) + 1;
            src->frames.assign(numFrames, bind);
            auto sample = [](const Track& tr, float time, int comps, float* out) {
                size_t n = tr.t.size();
                if (n == 0) return false;
                size_t i = 0;
                while (i + 1 < n && tr.t[i + 1] <= time + 1e-5f) i++;
                if (i + 1 >= n || time <= tr.t[0]) {
                    size_t k = time <= tr.t[0] ? 0 : n - 1;
                    for (int c = 0; c < comps; c++) out[c] = tr.v[k * comps + c];
                    return true;
                }
                float f = (time - tr.t[i]) / std::max(1e-6f, tr.t[i + 1] - tr.t[i]);
                for (int c = 0; c < comps; c++) out[c] = tr.v[i * comps + c] * (1 - f) + tr.v[(i + 1) * comps + c] * f;
                return true;
            };
            for (int fi = 0; fi < numFrames; fi++) {
                float time = fi / fps;
                for (auto& pt : posTracks) {
                    float v[3];
                    if (sample(pt.second, time, 3, v)) src->frames[fi][pt.first].pos = Vec3(v[0], v[1], v[2]);
                }
                for (auto& rt : rotTracks) {
                    float v[4];
                    if (sample(rt.second, time, 4, v)) {
                        Quat q = QuaternionNormalize(Quat(v[0], v[1], v[2], v[3]));
                        src->frames[fi][rt.first].rot = QuaternionAngles(q);
                    }
                }
            }
        }
    }
    return src;
}

}  // namespace r2c
