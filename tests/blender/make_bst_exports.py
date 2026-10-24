"""Builds a small skinned, animated test rig in Blender and exports it with Blender
Source Tools as SMD and DMX, so r2studiomdl can be tested on real BST output.

blender --background --factory-startup --python make_bst_exports.py -- <bst_dir> <out_dir>
"""
import math
import os
import sys

import bpy

argv = sys.argv[sys.argv.index("--") + 1:]
bst_dir, out_dir = argv[0], os.path.abspath(argv[1])
sys.path.insert(0, bst_dir)
import io_scene_valvesource  # noqa: E402

io_scene_valvesource.register()

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = 30

# armature: root -> spine -> head (Z up, 64 units tall)
arm_data = bpy.data.armatures.new("rig")
arm = bpy.data.objects.new("rig", arm_data)
scene.collection.objects.link(arm)
bpy.context.view_layer.objects.active = arm
bpy.ops.object.mode_set(mode="EDIT")
root = arm_data.edit_bones.new("root")
root.head, root.tail = (0, 0, 0), (0, 0, 16)
spine = arm_data.edit_bones.new("spine")
spine.head, spine.tail, spine.parent = (0, 0, 16), (0, 0, 40), root
head = arm_data.edit_bones.new("head")
head.head, head.tail, head.parent = (0, 0, 40), (0, 0, 64), spine
bpy.ops.object.mode_set(mode="OBJECT")

# mesh: cylinder weighted by height
bpy.ops.mesh.primitive_cylinder_add(vertices=12, radius=8, depth=64, location=(0, 0, 32))
body = bpy.context.active_object
body.name = "body"
mat = bpy.data.materials.new("models/electricalboxes/pipesmulti")
body.data.materials.append(mat)
groups = {n: body.vertex_groups.new(name=n) for n in ("root", "spine", "head")}
for v in body.data.vertices:
    z = (body.matrix_world @ v.co).z
    if z < 16:
        groups["root"].add([v.index], 1.0, "REPLACE")
    elif z < 40:
        groups["spine"].add([v.index], 0.7, "REPLACE")
        groups["root"].add([v.index], 0.3, "REPLACE")
    else:
        groups["head"].add([v.index], 1.0, "REPLACE")
body.parent = arm
mod = body.modifiers.new("arm", "ARMATURE")
mod.object = arm

# action: spine bends 45 degrees over 30 frames
arm.animation_data_create()
act = bpy.data.actions.new("bend")
arm.animation_data.action = act
pb = arm.pose.bones["spine"]
pb.rotation_mode = "XYZ"
for f, ang in ((0, 0), (15, 45), (30, 0)):
    pb.rotation_euler = (math.radians(ang), 0, 0)
    pb.keyframe_insert("rotation_euler", frame=f)
scene.frame_start, scene.frame_end = 0, 30

def export(fmt, encoding=None, dmx_format=None, sub=None):
    scene = bpy.context.scene
    path = os.path.join(out_dir, sub or fmt.lower())
    os.makedirs(path, exist_ok=True)
    scene.vs.export_path = path
    scene.vs.export_format = fmt
    if encoding:
        scene.vs.dmx_encoding = encoding
    if dmx_format:
        scene.vs.dmx_format = dmx_format
    io_scene_valvesource.utils.State.update_scene(scene)
    bpy.ops.export_scene.smd(export_scene=True)
    print("exported", fmt, encoding, dmx_format, "->", path, sorted(os.listdir(path)))

export("SMD")
formats = [f[0] for f in io_scene_valvesource.formats]
print("available DMX formats:", formats)
export("DMX", "2", "1", "dmx_bin2")
export("DMX", "3", "15", "dmx_bin3")
export("DMX", "4", "18", "dmx_bin4")
export("DMX", "5", "18", "dmx_bin5")
export("DMX", "9", "22", "dmx_bin9")
