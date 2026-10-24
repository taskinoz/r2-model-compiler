"""Structural reader for Titanfall 2 (v53) .mdl files.

Used to survey retail models and to diff compiler output against them.
Field layouts follow the public reverse-engineering notes in rmdlconv and rsx;
this file is an independent implementation.
"""
import struct
import sys
import json

HDR_FIELDS = [
    ("id", "i"), ("version", "i"), ("checksum", "i"), ("sznameindex", "i"),
    ("name", "64s"), ("length", "i"),
    ("eyeposition", "3f"), ("illumposition", "3f"), ("hull_min", "3f"), ("hull_max", "3f"),
    ("view_bbmin", "3f"), ("view_bbmax", "3f"),
    ("flags", "i"),
    ("numbones", "i"), ("boneindex", "i"),
    ("numbonecontrollers", "i"), ("bonecontrollerindex", "i"),
    ("numhitboxsets", "i"), ("hitboxsetindex", "i"),
    ("numlocalanim", "i"), ("localanimindex", "i"),
    ("numlocalseq", "i"), ("localseqindex", "i"),
    ("activitylistversion", "i"), ("eventsindexed", "i"),
    ("numtextures", "i"), ("textureindex", "i"),
    ("numcdtextures", "i"), ("cdtextureindex", "i"),
    ("numskinref", "i"), ("numskinfamilies", "i"), ("skinindex", "i"),
    ("numbodyparts", "i"), ("bodypartindex", "i"),
    ("numlocalattachments", "i"), ("localattachmentindex", "i"),
    ("numlocalnodes", "i"), ("localnodeindex", "i"), ("localnodenameindex", "i"), ("localNodeUnk", "i"),
    ("flexdescindex", "i"),
    ("numflexcontrollers", "i"), ("flexcontrollerindex", "i"),
    ("numflexrules", "i"), ("flexruleindex", "i"),
    ("numikchains", "i"), ("ikchainindex", "i"),
    ("uiPanelCount", "i"), ("uiPanelOffset", "i"),
    ("numlocalposeparameters", "i"), ("localposeparamindex", "i"),
    ("surfacepropindex", "i"),
    ("keyvalueindex", "i"), ("keyvaluesize", "i"),
    ("numlocalikautoplaylocks", "i"), ("localikautoplaylockindex", "i"),
    ("mass", "f"), ("contents", "i"),
    ("numincludemodels", "i"), ("includemodelindex", "i"),
    ("virtualModel", "i"),
    ("bonetablebynameindex", "i"),
    ("constdirectionallightdot", "B"), ("rootLOD", "B"), ("numAllowedRootLODs", "B"), ("unused0", "B"),
    ("fadeDistance", "f"),
    ("numflexcontrollerui", "i"), ("flexcontrolleruiindex", "i"),
    ("flVertAnimFixedPointScale", "f"), ("surfacepropLookup", "i"),
    ("sourceFilenameOffset", "i"),
    ("numsrcbonetransform", "i"), ("srcbonetransformindex", "i"),
    ("illumpositionattachmentindex", "i"),
    ("linearboneindex", "i"),
    ("boneFlexDriverCount", "i"), ("boneFlexDriverIndex", "i"),
    ("perTriAABBIndex", "i"), ("perTriAABBNodeCount", "i"), ("perTriAABBLeafCount", "i"), ("perTriAABBVertCount", "i"),
    ("unkStringOffset", "i"),
    ("vtxOffset", "i"), ("vvdOffset", "i"), ("vvcOffset", "i"), ("phyOffset", "i"),
    ("vtxSize", "i"), ("vvdSize", "i"), ("vvcSize", "i"), ("phySize", "i"),
    ("collisionOffset", "i"), ("staticCollisionCount", "i"),
    ("boneFollowerCount", "i"), ("boneFollowerOffset", "i"),
    ("unused1", "60i"),
]


def _fmt(fields):
    return "<" + "".join(f for _, f in fields)


HDR_FMT = _fmt(HDR_FIELDS)
assert struct.calcsize(HDR_FMT) == 0x2CC, struct.calcsize(HDR_FMT)


def unpack_fields(fields, data, off):
    fmt = _fmt(fields)
    vals = struct.unpack_from(fmt, data, off)
    out = {}
    i = 0
    for name, f in fields:
        n = 1
        digits = "".join(c for c in f if c.isdigit())
        kind = f[-1]
        if digits and kind != "s":
            n = int(digits)
        if n == 1:
            v = vals[i]
            if kind == "s":
                v = v.split(b"\0", 1)[0].decode("latin1")
            out[name] = v
            i += 1
        else:
            out[name] = list(vals[i:i + n])
            i += n
    return out, struct.calcsize(fmt)


def cstr(data, off):
    if off <= 0 or off >= len(data):
        return None
    end = data.find(b"\0", off)
    return data[off:end].decode("latin1")


BONE_FIELDS = [
    ("sznameindex", "i"), ("parent", "i"), ("bonecontroller", "6i"),
    ("pos", "3f"), ("quat", "4f"), ("rot", "3f"), ("scale", "3f"),
    ("posscale", "3f"), ("rotscale", "3f"), ("scalescale", "3f"),
    ("poseToBone", "12f"), ("qAlignment", "4f"),
    ("flags", "i"), ("proctype", "i"), ("procindex", "i"), ("physicsbone", "i"),
    ("surfacepropidx", "i"), ("contents", "i"), ("surfacepropLookup", "i"),
    ("collisionIndex", "H"), ("collisionCount", "H"), ("unused", "7i"),
]
assert struct.calcsize(_fmt(BONE_FIELDS)) == 0xF4

ATTACH_FIELDS = [("sznameindex", "i"), ("flags", "I"), ("localbone", "i"), ("local", "12f"), ("unused", "8i")]
HBOXSET_FIELDS = [("sznameindex", "i"), ("numhitboxes", "i"), ("hitboxindex", "i")]
BBOX_FIELDS = [("bone", "i"), ("group", "i"), ("bbmin", "3f"), ("bbmax", "3f"), ("szhitboxnameindex", "i"),
               ("forceCritPoint", "i"), ("hitdataGroupOffset", "i"), ("unused", "6i")]
ANIMDESC_FIELDS = [
    ("baseptr", "i"), ("sznameindex", "i"), ("fps", "f"), ("flags", "i"), ("numframes", "i"),
    ("nummovements", "i"), ("movementindex", "i"), ("framemovementindex", "i"), ("animindex", "i"),
    ("numikrules", "i"), ("ikruleindex", "i"), ("numlocalhierarchy", "i"), ("localhierarchyindex", "i"),
    ("sectionindex", "i"), ("sectionframes", "i"), ("unused", "8i"),
]
assert struct.calcsize(_fmt(ANIMDESC_FIELDS)) == 0x5C
SEQDESC_FIELDS = [
    ("baseptr", "i"), ("szlabelindex", "i"), ("szactivitynameindex", "i"), ("flags", "i"),
    ("activity", "i"), ("actweight", "i"), ("numevents", "i"), ("eventindex", "i"),
    ("bbmin", "3f"), ("bbmax", "3f"), ("numblends", "i"), ("animindexindex", "i"),
    ("movementindex", "i"), ("groupsize", "2i"), ("paramindex", "2i"), ("paramstart", "2f"),
    ("paramend", "2f"), ("paramparent", "i"), ("fadeintime", "f"), ("fadeouttime", "f"),
    ("localentrynode", "i"), ("localexitnode", "i"), ("nodeflags", "i"), ("entryphase", "f"),
    ("exitphase", "f"), ("lastframe", "f"), ("nextseq", "i"), ("pose", "i"), ("numikrules", "i"),
    ("numautolayers", "i"), ("autolayerindex", "i"), ("weightlistindex", "i"), ("posekeyindex", "i"),
    ("numiklocks", "i"), ("iklockindex", "i"), ("keyvalueindex", "i"), ("keyvaluesize", "i"),
    ("cycleposeindex", "i"), ("activitymodifierindex", "i"), ("numactivitymodifiers", "i"),
    ("ikResetMask", "i"), ("unk_C4", "i"), ("unused", "8i"),
]
assert struct.calcsize(_fmt(SEQDESC_FIELDS)) == 0xE8
TEXTURE_FIELDS = [("sznameindex", "i"), ("unused_flags", "i"), ("used", "i"), ("unused", "8i")]
BODYPART_FIELDS = [("sznameindex", "i"), ("nummodels", "i"), ("base", "i"), ("modelindex", "i")]
MODEL_FIELDS = [
    ("name", "64s"), ("type", "i"), ("boundingradius", "f"), ("nummeshes", "i"), ("meshindex", "i"),
    ("numvertices", "i"), ("vertexindex", "i"), ("tangentsindex", "i"), ("numattachments", "i"),
    ("attachmentindex", "i"), ("numeyeballs", "i"), ("eyeballindex", "i"), ("pad", "4i"),
    ("colorindex", "i"), ("uv2index", "i"), ("unused", "4i"),
]
assert struct.calcsize(_fmt(MODEL_FIELDS)) == 0x94
MESH_FIELDS = [
    ("material", "i"), ("modelindex", "i"), ("numvertices", "i"), ("vertexoffset", "i"),
    ("numflexes", "i"), ("flexindex", "i"), ("materialtype", "i"), ("materialparam", "i"),
    ("meshid", "i"), ("center", "3f"), ("modelvertexdata", "i"), ("numLODVertexes", "8i"),
    ("unk_54", "2i"), ("unused", "6i"),
]
assert struct.calcsize(_fmt(MESH_FIELDS)) == 0x74
IKCHAIN_FIELDS = [("sznameindex", "i"), ("linktype", "i"), ("numlinks", "i"), ("linkindex", "i"),
                  ("unk_10", "f"), ("unused", "3i")]
IKLINK_FIELDS = [("bone", "i"), ("kneeDir", "3f"), ("unused0", "3f")]
POSEPARAM_FIELDS = [("sznameindex", "i"), ("flags", "i"), ("start", "f"), ("end", "f"), ("loop", "f")]
JIGGLE_FIELDS = [("flags", "i")] + [(n, "f") for n in (
    "length tipMass yawStiffness yawDamping pitchStiffness pitchDamping alongStiffness alongDamping "
    "angleLimit minYaw maxYaw yawFriction yawBounce minPitch maxPitch pitchFriction pitchBounce "
    "baseMass baseStiffness baseDamping baseMinLeft baseMaxLeft baseLeftFriction baseMinUp baseMaxUp "
    "baseUpFriction baseMinForward baseMaxForward baseForwardFriction").split()]
LINEARBONE_FIELDS = [("numbones", "i"), ("flagsindex", "i"), ("parentindex", "i"), ("posindex", "i"),
                     ("quatindex", "i"), ("rotindex", "i"), ("posetoboneindex", "i"), ("posscaleindex", "i"),
                     ("rotscaleindex", "i"), ("qalignmentindex", "i"), ("unused", "6i")]
EVENT_FIELDS = [("cycle", "f"), ("event", "i"), ("type", "i"), ("options", "64s"), ("szeventindex", "i")]
AUTOLAYER_FIELDS = [("iSequence", "h"), ("iPose", "h"), ("flags", "i"), ("start", "f"), ("peak", "f"),
                    ("tail", "f"), ("end", "f")]
IKLOCK_FIELDS = [("chain", "i"), ("flPosWeight", "f"), ("flLocalQWeight", "f"), ("flags", "i"), ("unused", "4i")]
IKRULE_FIELDS = [
    ("index", "i"), ("type", "i"), ("chain", "i"), ("bone", "i"), ("slot", "i"), ("height", "f"),
    ("radius", "f"), ("floor", "f"), ("pos", "3f"), ("q", "4f"), ("compressedikerrorindex", "i"),
    ("iStart", "i"), ("ikerrorindex", "i"), ("start", "f"), ("peak", "f"), ("tail", "f"), ("end", "f"),
    ("contact", "f"), ("drop", "f"), ("top", "f"), ("szattachmentindex", "i"), ("endHeight", "f"),
    ("unused", "8i"),
]
SRCBONE_FIELDS = [("sznameindex", "i"), ("pretransform", "12f"), ("posttransform", "12f")]

# VTX
VTX_HDR = [("version", "i"), ("vertCacheSize", "i"), ("maxBonesPerStrip", "H"), ("maxBonesPerFace", "H"),
           ("maxBonesPerVert", "i"), ("checkSum", "i"), ("numLODs", "i"), ("materialReplacementListOffset", "i"),
           ("numBodyParts", "i"), ("bodyPartOffset", "i")]


def u(fields, data, off):
    return unpack_fields(fields, data, off)[0]


def size(fields):
    return struct.calcsize(_fmt(fields))


class Model:
    def __init__(self, data):
        self.data = data
        self.hdr = u(HDR_FIELDS, data, 0)
        h = self.hdr
        if h["id"] != 0x54534449 or h["version"] != 53:
            raise ValueError("not a v53 mdl: id=%x version=%d" % (h["id"], h["version"]))

    def arr(self, fields, base, count):
        sz = size(fields)
        return [u(fields, self.data, base + i * sz) for i in range(count)]

    def bones(self):
        h = self.hdr
        out = []
        sz = size(BONE_FIELDS)
        for i in range(h["numbones"]):
            o = h["boneindex"] + i * sz
            b = u(BONE_FIELDS, self.data, o)
            b["name"] = cstr(self.data, o + b["sznameindex"])
            b["surfaceprop"] = cstr(self.data, o + b["surfacepropidx"])
            if b["procindex"]:
                b["jiggle"] = u(JIGGLE_FIELDS, self.data, o + b["procindex"])
                b["_procoffset"] = o + b["procindex"]
            out.append(b)
        return out


VVD_HDR = [("id", "i"), ("version", "i"), ("checksum", "i"), ("numLODs", "i"), ("numLODVertexes", "8i"),
           ("numFixups", "i"), ("fixupTableStart", "i"), ("vertexDataStart", "i"), ("tangentDataStart", "i")]
VVC_HDR = [("id", "i"), ("version", "i"), ("checksum", "i"), ("numLODs", "i"), ("numLODVertexes", "8i"),
           ("colorDataStart", "i"), ("uv2DataStart", "i")]
PHY_HDR = [("size", "i"), ("id", "i"), ("solidCount", "i"), ("checkSum", "i")]
PERTRI_HDR = [("version", "i"), ("bbmin", "3f"), ("bbmax", "3f"), ("unused", "8i")]
COLL_HDR = [("sideCount", "i"), ("sideAndUnkCount", "i"), ("edgeCount", "i"), ("vertCount", "i"), ("dataOffset", "i")]


def parse_rle_anim(data, off, numbones):
    """Walk the per-bone RLE anim chain starting at off. Returns list of bone headers."""
    out = []
    seen = 0
    while True:
        posscale, bone, flags = struct.unpack_from("<fBB", data, off)
        nxt = struct.unpack_from("<i", data, off + 8 + 20)[0] if False else None
        # layout: header 8 bytes (float, u8, u8, pad2). then data depends on flags
        rec = {"_off": off, "posscale": posscale, "bone": bone, "flags": flags}
        p = off + 8
        if flags & 0x04:  # rawrot Quaternion64
            rec["rot"] = "raw64"; p += 8
        elif not (flags & 0x10):
            rec["rotptr"] = struct.unpack_from("<3h", data, p); p += 6
        if flags & 0x02:
            rec["pos"] = "raw48"; p += 6
        else:
            rec["posptr"] = struct.unpack_from("<3h", data, p); p += 6
        rec["_datalen_guess"] = p - off
        out.append(rec)
        seen += 1
        if seen > 300:
            break
        break
    return out


def survey(path):
    data = open(path, "rb").read()
    m = Model(data)
    h = m.hdr
    offs = {k: v for k, v in h.items() if isinstance(v, int) and (k.endswith("index") or k.endswith("Offset"))
            and k not in ("sznameindex", "surfacepropindex", "illumpositionattachmentindex") and v > 0}
    return m, sorted(offs.items(), key=lambda kv: kv[1])


if __name__ == "__main__":
    for p in sys.argv[1:]:
        m, offs = survey(p)
        print(p, m.hdr["name"], "len", m.hdr["length"], "flags %x" % m.hdr["flags"])
        for k, v in offs:
            print("   %08x %s" % (v, k))


VTX_BODYPART = [("numModels", "i"), ("modelOffset", "i")]
VTX_MODEL = [("numLODs", "i"), ("lodOffset", "i")]
VTX_LOD = [("numMeshes", "i"), ("meshOffset", "i"), ("switchPoint", "f")]
VTX_MESH = [("numStripGroups", "i"), ("stripGroupHeaderOffset", "i"), ("flags", "B")]
VTX_STRIPGROUP = [("numVerts", "i"), ("vertOffset", "i"), ("numIndices", "i"), ("indexOffset", "i"),
                  ("numStrips", "i"), ("stripOffset", "i"), ("flags", "B"), ("numTopologyIndices", "i"),
                  ("topologyOffset", "i")]
VTX_STRIP = [("numIndices", "i"), ("indexOffset", "i"), ("numVerts", "i"), ("vertOffset", "i"),
             ("numBones", "h"), ("flags", "B"), ("numBoneStateChanges", "i"), ("boneStateChangeOffset", "i"),
             ("numTopologyIndices", "i"), ("topologyOffset", "i")]
VTX_VERT = [("boneWeightIndex", "3B"), ("numBones", "B"), ("origMeshVertID", "H"), ("boneID", "3b")]
assert size(VTX_MESH) == 9 and size(VTX_STRIPGROUP) == 33 and size(VTX_STRIP) == 35 and size(VTX_VERT) == 9


def parse_vtx(data, base):
    """Return nested dict tree of the VTX at absolute offset base."""
    hdr = u(VTX_HDR, data, base)
    tree = {"hdr": hdr, "bodyparts": []}
    for bi in range(hdr["numBodyParts"]):
        bo = base + hdr["bodyPartOffset"] + bi * size(VTX_BODYPART)
        bp = u(VTX_BODYPART, data, bo)
        bp["models"] = []
        for mi in range(bp["numModels"]):
            mo = bo + bp["modelOffset"] + mi * size(VTX_MODEL)
            md = u(VTX_MODEL, data, mo)
            md["lods"] = []
            for li in range(md["numLODs"]):
                lo = mo + md["lodOffset"] + li * size(VTX_LOD)
                ld = u(VTX_LOD, data, lo)
                ld["meshes"] = []
                for ei in range(ld["numMeshes"]):
                    eo = lo + ld["meshOffset"] + ei * size(VTX_MESH)
                    me = u(VTX_MESH, data, eo)
                    me["stripgroups"] = []
                    for gi in range(me["numStripGroups"]):
                        go = eo + me["stripGroupHeaderOffset"] + gi * size(VTX_STRIPGROUP)
                        sg = u(VTX_STRIPGROUP, data, go)
                        sg["_off"] = go
                        sg["strips"] = [u(VTX_STRIP, data, go + sg["stripOffset"] + k * size(VTX_STRIP))
                                        for k in range(sg["numStrips"])]
                        me["stripgroups"].append(sg)
                    ld["meshes"].append(me)
                md["lods"].append(ld)
            bp["models"].append(md)
        tree["bodyparts"].append(bp)
    return tree
