# r2-model-compiler

[![CI](https://github.com/taskinoz/r2-model-compiler/actions/workflows/ci.yml/badge.svg)](https://github.com/taskinoz/r2-model-compiler/actions/workflows/ci.yml)

`r2studiomdl` is a model compiler for **Titanfall 2** (MDL v53). It reads a QC file with
SMD or DMX sources and writes a single v53 `.mdl` file with its VTX, VVD, VVC and PHY
data embedded, ready to load in game (for example through a Northstar mod). The QC
can come from a Crowbar decompile or be written by hand around Blender Source Tools
exports.

It replaces the old route of compiling a Source v49 model and converting it to v53.
It also writes Respawn's own data directly: root-motion tracks, bone scale animation,
IK rules, procedural bones and weapon UI panels.

> Not affiliated with Respawn Entertainment, Electronic Arts or Valve. You need your
> own copy of Titanfall 2; no game assets are included.

## Features

- Static props, physics props (collision hulls, PHY, ragdolls, the map-collision block)
  and skinned, animated models with LODs, hitboxes, attachments and skins.
- SMD and DMX input (binary DMX 1-5 and 9, keyvalues2), including Blender Source
  Tools output. Positive skinning weights are retained, up to the format's three
  influences per vertex.
- A studiomdl-style command line (`-game`, `-nop4`, `-verbose`, `-nox360`), so Crowbar
  can use it as its compiler.
- Respawn animation data:
  - frame-movement root motion;
  - bone scale animation;
  - delta animations, blend pose keys and sequence weight lists.
- IK chains and rules: touch, footstep, attachment, release and unlatch, with
  `ikfixup`.
- Procedural bones: jiggle, quaternion interpolation, axis interpolation, and aim at a
  bone or attachment.
- Weapon UI panel meshes (`$uipanel`), such as ammo counters and sight screens.
- Bone followers (`$bonefollower`) for animated props with moving collision, vertex
  colours and a second UV set (from DMX).

All 602 retail multiplayer models and 3,422 unique single-player models decompile,
compile and pass full-frame validation: 1.74 million animation frames, 211.67 million
bone poses, render/skin data and collision geometry. The build also passes 62
regressions and representative Northstar runtime checks. See
[VALIDATION.md](VALIDATION.md) for the measured coverage, tolerances and remaining
binary-layout differences.

## Quick start

Download `r2-model-compiler-win64.zip` from the
[releases](../../releases) page and extract it. Then compile a QC into a mod folder:

```powershell
.\r2studiomdl.exe -game "<Titanfall2>\R2Northstar\mods\MyMod\mod" path\to\model.qc
```

The model is written to `models\<$modelname>` beneath the `-game` folder. Use
`-o out\model.mdl` to write a single file instead. The package includes three example
sources (static prop, physics prop, animated prop); see
[examples/README.md](examples/README.md).

Compiling creates the model file; a map or script still has to spawn an entity that
uses it. Physics props need a `prop_data` keyvalues block, as the physics example
shows.

### With Crowbar

In Crowbar's game setup, select `r2studiomdl.exe` as the model compiler and set the
game folder to your mod's `mod` directory. The compiler accepts the arguments
Crowbar passes. Crowbar's own decompiles of Titanfall 2 models are valid input.

### With Blender

Export meshes and animations with Blender Source Tools as SMD or DMX, and reference
them from a QC as you would for Source. The animated example was made this way.

## QC reference: Respawn additions

Common commands follow Valve's studiomdl conventions. These additions cover
Titanfall 2 data; the tested scope is recorded in VALIDATION.md.

### Root motion

An `$animation` or `$sequence` with `rm` stores the root bones' yaw and translation as
a Respawn frame-movement track, as retail models do. `LX`/`LY`/`LZ`/`LZR` motion
extraction is stored the same way. Add `$legacymovement` to the QC to write Valve-style
piecewise movement records instead.

### Bone scale

An SMD skeleton row may carry three extra values after the rotation, the bone's local
scale (default 1): `bone px py pz rx ry rz sx sy sz`. Valve's studiomdl reads only the
first seven values, so such files still load elsewhere. In DMX, a joint transform's
`scale` attribute (float or vector) and `scale` channels are read. Animations that
scale a bone are flagged as retail ones are (0x20000), as are their sequences.

### Animation and sequence options

- `$animation ... delta`: the source already holds per-bone local deltas, as a
  decompiled delta animation does.
- `$sequence ... { posekeys <count> <values...> }`: explicit blend positions, for blends
  that are not evenly spaced.
- `$sequence ... { seqweights <weightlist> }`: the sequence's weight list, when it
  differs from the weights its animations were encoded with.
- A named `$animation`'s own options (such as `loop`) are not copied onto the sequences
  that use it. Options written inside the `$sequence` block are.

### IK

- `$ikchain` and the `ikrule` animation option are compiled (touch, footstep,
  attachment, release and unlatch).
- This includes studiomdl's range inference for `.` values, per-frame target errors
  (footsteps follow the root-motion track), their compressed streams, and automatic
  release rules for chains an animation moves without rules (`noautoik` turns that
  off).
- Footstep rules default to Respawn's `endheight -100`.
- `ikfixup` re-solves the leg so the foot stays on its contact position over the
  rule's range, and bakes the result into the animation.
- `$weightlist` lists apply per animation.

### Procedural bones

- `$jigglebone` blocks are supported.
- `$proceduralbones` accepts studiomdl's two file types:
  - **`.vrd`:** `<helper>`/`<display>`/`<basepos>`/`<trigger>` quaternion-interpolation
    bones, and `<aimconstraint> bone parent target` aim bones. The target may be a
    bone or an attachment; attachments are matched first.
  - **`.txt` (axis interpolation):** `version 1`, `proceduralbones`, then for each bone
    `bone "bone" "parent" "control" "controlparent" <axis 1-3>`, `basepos x y z` and six
    `axis <0-5> x y z roll pitch yaw` poses. The poses are for +X, -X, +Y, -Y, +Z and -Z
    of the control bone's chosen axis.

Driven bones are kept even when no vertices use them. Titanfall 2 evaluates procedural
bones on the client, so server-side attachment queries do not see their effect.

### UI panels

UI panels are the quads a weapon draws its screens on (ammo counters, sights). A weapon
script's `UiData` block names one with `"mesh" "models/..."`, and the game finds it by
a hash of that name. The compiler computes the hash, so a new panel needs only a name:

```
$uipanel "models/weapons/attachments/my_counter"
{
	parent "def_c_screen"          // one or two bones; vertices name a slot (0 or 1)
	vertex 0  0 -1.2 1.1           // slot x y z, local to the slot's bone
	vertex 0  0  1.2 1.1
	vertex 0  0  1.2 0.3
	face 0 1 2 ccw                 // three corners of a parallelogram (the fourth is
	bounds 0 1 1 0  0 0 1 1        //   a + c - b); bounds: x0..x3 y0..y3 per face
}
```

- A hex value after the name (`$uipanel "name" 0x1234abcd`) overrides the hash. The hash
  is MurmurHash3 x86_32 with seed 0, every byte ANDed with 0xDF (so case does not
  matter), and the length counting the terminating nul.
- At most 256 vertices and 64 faces; only the first 32 faces can be `ccw`.

### Collision faces and the map-collision block

Besides the PHY physics data, a Titanfall 2 model with collision carries a
map-collision block: each convex piece stored as a brush of planes, edges and
vertices. Respawn's faces are the polygons of the source collision mesh (Maya quads and
n-gons), not a planarity merge.

- **DMX:** each polygon of a DMX collision mesh becomes one face, as Respawn's
  pipeline does. A slightly bent quad stays one face. Where a polygon corner lies
  partway along a neighbour's edge (a T-junction), that edge is split so the faces meet.
- **SMD:** an SMD holds only triangles, so the compiler merges triangles that lie on
  one plane.
- **Surface points:** points lying on a convex piece's surface without being corners
  (an edge midpoint, the centre of a cylinder cap) stay as vertices, as in retail. In
  DMX such a point can also be a loose vertex (one no face uses).

To match retail collision, export the collision mesh from Blender or Maya as DMX with
its quads intact. Decompiled models write their physics mesh as DMX for the same
reason. Each convex piece is one connected, closed mesh in that file; it imports into
Blender with Blender Source Tools.

`$nomapcollision` inside a `$collisionmodel`/`$collisionjoints` block writes the PHY
data but no map-collision block. A few retail models are built this way.

```
$collisionmodel "props_phys.dmx"
{
	$mass 40
	$concave
}
```

### Vertex colours and a second UV set

A v53 model can carry per-vertex colours and a second UV set (UV2) for materials that
use them. Both are stored in the model's VVC block, and the header flags `0x1000000`
(vertex colour) and `0x2000000` (UV2) announce them. SMD has no field for either, so
use DMX.

In Blender, with Blender Source Tools:

1. Name the UV maps `texcoord$0` (the normal UVs) and `texcoord$1` (UV2). If no map
   is called `texcoord$0`, the map active for rendering is exported as the first set.
2. For vertex colours, name a colour attribute `color$0`.
3. In the Source Engine Export panel, choose DMX. Leave Engine Path empty, then set
   DMX Version to encoding **9** and format **22**. Blender Source Tools exports
   `$`-named layers only at model format 22 or above.

Then reference the DMX as usual:

```
$modelname "props/screen_panel.mdl"
$body body "screen_panel.dmx"
$sequence idle "screen_panel.dmx"
```

The compiler sets the UV2 and vertex-colour flags automatically when the DMX has
`texcoord$1` or `color$0`. `$useextratexcoords` and `$usevertexcolor` set the flags
even without the data; missing values are written as UV (0, 0) and white. As with the
first set, UV2's V coordinate is flipped on import.

Retail Titanfall 2 models use vertex colours (about a quarter of them) but never set
the UV2 flag. The UV2 data is written correctly (covered by a regression test), but
it only shows in game if the model's material reads a second UV set.

### Tangents

A DMX `tangent$0` (or `tangents`) attribute, vector4 with the bitangent sign in w, is
stored as given instead of being computed. Corners that differ only in tangent stay
separate vertices, which is how retail meshes keep vertices apart that match in
position, normal and UV. Decompiled meshes carry retail's tangents for this reason.
Without the attribute, tangents are computed from the UVs as before.

### Retail values

These commands set values retail models store that plain QC cannot express. The
decompiler writes them when a model needs them.

- `$definebone ... scale x y z`: positive, finite local bind scale, after the
  usual position, rotation and optional six alignment values. DMX bind transforms
  also retain scale. SMD skeleton rows may append scale xyz after rotation xyz.
- `$generatedhitboxes`: retain the generated flag alongside explicit `$hbox` data.
- `$boneusage "bone" mask`, `$respawnflag 0|1`, `$zerounusedlods`, and
  `$modelsource "mesh.dmx" "logical/name"`: retain exact usage bits, header flag 0x800000,
  zeroed unused VVD LOD counts, and logical mesh names respectively.
  Bodygroup `blankname "logical/name"` retains the name of an empty model entry.
  Remove retained `$boneusage` values when editing bone use, so normal propagation
  can include newly added attachments or meshes.
- `$pertricollision 0|2` selects the retail triangle-collision variant.
  `$pertrimesh "mesh.pertri"` retains exact quantized collision geometry while the
  compiler constructs a new acceleration tree. Remove it after editing the render
  mesh to regenerate collision. The text format is `r2c_pertri 1`, six bounds
  (minimum xyz then maximum xyz), vertex/triangle counts, uint16 xyz vertex rows,
  then triangle rows `a b c surfaceProperty` with zero-based indices.
  Decompiled QCs also retain an explicit `$bbox`: the header hull can intentionally
  differ from the triangle-collision bounds. Remove it when new source geometry
  should determine the hull bounds.
- Animation `movementrecord endFrame flags v0 v1 angle vx vy vz px py pz` retains
  one legacy movement record. Records must be ordered and end within the animation.
- Inside collision blocks, `$jointmass`, `$jointdrag`, `$jointsurfaceprop`, and
  `$jointparent` set one solid's mass, drag, material, and physical parent.
  `$collisionname` retains a rigid solid's logical name. `$explicitjointconstraints`
  emits only explicitly constrained joints, preserving parents without constraints.
- Collision `$ivpsurface "solid" cx cy cz ix iy iz dx dy dz radius volume` retains
  the original IVP mass center, normalized inertia, drag axes and bounding radius
  (IVP units), plus volume in cubic inches. Remove this and retained `$jointmass`
  values after editing a physics mesh so the compiler recomputes its properties.
  A retained radius that no longer encloses the mesh is rejected.
- Decompiled collision DMX carries `r2c_collision_planes` groups in its bind vertex
  data. Each group retains the source polygon range, literal face/bevel planes,
  and the native float32 IVP points and oriented triangles. The compiler rebuilds
  edge links and acceleration trees. Remove these groups when editing a physics
  mesh to regenerate its surfaces; retained native triangulation does not follow
  edits to the visible DMX polygons. Invalid indices or open surfaces are rejected.

- `$mass <kg>` at the top level: the header mass of a model without physics (inside a
  collision block, `$mass` keeps its usual meaning).
- `$contents <number>`: the exact contents mask (names such as `grate` still work).
- `$srcbonetransform "bone" <12 pre> <12 post>`: a source bone transform, two 3x4
  matrices row by row.
- `$attachment "name" "bone" x y z matrix <12 values>`: the bone-space matrix exactly as
  stored (retail has some that are not rotations, such as all zeros).
- `$autohitboxes`: generate hitboxes as Valve's studiomdl does. Without `$hbox` lines or
  this command, the hitbox set is empty but flagged as generated, as in retail.
- `$ikchain ... links "a" "b" "c"`: the chain's three links when they are not the plain
  parent chain (retail skips twist bones); `unk <value>` sets the chain's unknown float
  (default 0.707).
- Sequence options: `unk_c4 <n>`, `bbox <min x y z> <max x y z>` (the stored box
  instead of a computed one), and `blendlayer` options `realtime` and `flags <n>`.
  Decompiled sequences carry retail's boxes, because some retail boxes do not match
  their animation; delete the `bbox` line after editing an animation so the box is
  recomputed.
- DMX `r2c_vertex_id` (int per corner): corners with different ids never become one
  vertex. Decompiled meshes carry retail's vertex numbers this way, so vertices retail
  kept apart stay apart even when identical in every attribute.
- DMX bind vertex data `r2c_model_space` (bool): coordinates and supplied normals
  and tangents are already in final model space. The compiler remaps bone indices
  but skips bind-pose deformation and the static-prop default rotation. The
  decompiler uses this to preserve native vertex floats, with
  `flipVCoordinates = false` and stored UV/UV2 values. Ordinary Blender exports
  follow the usual source-space import rules. Remove this marker if you want a
  changed bind skeleton to deform the source mesh during compilation.
- `$keyvalues` may appear more than once; each block is stored as its own
  `mdlkeyvalue` text, as retail does.
- `$bonerotscale "bone" x y z` and animation `posscale "bone" step` retain the
  original rotation and position quantization grids.
- Animation `fmtrack "file.fmtrack"` retains a stored root-motion track independently
  of the root bones. The text file starts with four scales, followed by one row of
  `x y z yaw` per animation frame (yaw in degrees). It replaces motion extraction.
- Animation `hasscale` retains flag 0x20000 even when every bone has unit scale.
- `ikrule ... errors "file.ikerr"` retains stored IK errors. The text file starts with
  six scales, followed by rows of position xyz and rotation xyz (radians).
  `storedtarget x y z qx qy qz qw` retains the rule's target and quaternion.
  The decompiler emits both options together. After editing an animation, remove
  `errors` and `storedtarget` to recompute IK, and remove `fmtrack` and use `rm` if
  root motion should be extracted again. Retained data does not follow source edits.

## Decompile and validate

The package includes Python tools requiring Python 3.10 or later and its standard
library. NumPy, if already installed, accelerates full-frame comparisons; it is optional.

```powershell
python tools\decompile53.py original.mdl exported
.\r2studiomdl.exe -o rebuilt.mdl exported\original.qc
python tools\validate53.py rebuilt.mdl --against original.mdl --report validation.json
python tools\roundtrip.py --compiler .\r2studiomdl.exe --jobs 8 --discard-sources --out test-output\fresh path\to\models
python tools\validate_corpus.py --report test-output\fresh\results.json --out test-output\dense --jobs 8
```

`validate53.py` checks binary structure, bone weights and transforms, triangle-tree
reachability and bounds, every animation frame, all material slots across skins and
LODs, collision geometry, and physics properties and constraints. The comparison
uses explicit tolerances recorded in its corpus manifest. A failed validation exits
nonzero. `--against` permits deliberately distinct render/collision meshes; standalone
validation normally also checks render-triangle coverage (`--allow-custom-collision`
disables that check). See VALIDATION.md for measured coverage and remaining limits.

`--discard-sources` keeps rebuilt MDLs and reports but removes generated exports
after each model. It only cleans work directories marked as owned by that run.
Both corpus tools checkpoint results; `--resume` requires unchanged inputs and tools.

## Building from source

Requires Visual Studio 2022 (or later) with the C++ x64 tools, and CMake (included with
Visual Studio).

```powershell
.\build.bat                                   # builds build\r2studiomdl.exe
Push-Location build; ctest --output-on-failure; Pop-Location
```

The CTest suite runs the C++ convex-hull tests and, when Python 3 is available, the
Python compiler regression tests. `.\tools\package.ps1` builds the release ZIP.

### Python tools and tests

The decompiler, comparison and in-game tools in `tools/` and the tests in `tests/` need
Python 3.10 or newer and only its standard library. They run with any Python 3
installation, or with [uv](https://docs.astral.sh/uv/), which picks or downloads a
suitable Python and creates a local `.venv` from `pyproject.toml`:

```powershell
# with uv
uv run python -m unittest discover -s tests -p "test_*.py"
uv run tools\roundtrip.py --help

# with a plain Python installation
python -m unittest discover -s tests -p "test_*.py"
```

The tests use `build\r2studiomdl.exe`; set `R2C_COMPILER` to test another executable.
The examples below use `python`; `uv run` works the same way in front of any of them.

## Testing against the game

The `tools/` folder holds the tools used to validate the compiler. They need your own
Titanfall 2 install.

- `decompile53.py` decompiles v53 models into QC/SMD.
- `roundtrip.py` decompiles retail models, recompiles them and compares the results:

  ```powershell
  python tools\roundtrip.py --jobs 8 --compiler build\r2studiomdl.exe --out test-output\run path\to\models
  ```

- `ingame_test.py` loads compiled models in Titanfall 2 through Northstar and records
  screenshots and measurements:

  ```powershell
  $env:TITANFALL2_DIR = "<Titanfall2 folder with Northstar>"
  python tools\ingame_test.py --sequence idle out\model.mdl=models/r2c_test/model.mdl
  ```

[AGENTS.md](AGENTS.md) documents these tools, how to extract the retail corpus, the
in-game test workflow and its pitfalls. [docs/mdl-v53-format.md](docs/mdl-v53-format.md)
collects the v53 format facts the compiler relies on.

## Limitations

- Output targets Titanfall 2 v53 only. Titanfall 1 and Apex Legends model files are not
  inputs; the compiler reads QC/SMD/DMX.
- Validation covers the tested retail corpus and representative runtime cases.
  Remaining binary-layout differences and coverage limits are in VALIDATION.md.
- Crowbar's command line is reproduced in a test, but its GUI has not been driven
  automatically.

## Acknowledgements

- Valve's studiomdl (Source SDK) defines the QC language and compile behaviour followed
  here.
- The v53 structure notes come from the r-ex projects [rsx](https://github.com/r-ex/rsx)
  and [rmdlconv](https://github.com/r-ex/rmdlconv).
- [Crowbar](https://github.com/ZeqMacaw/Crowbar),
  [Blender Source Tools](https://github.com/Artfunkel/BlenderSourceTools),
  [Northstar](https://github.com/R2Northstar) and
  [tf2vpk](https://github.com/pg9182/tf2vpk) made decompiling, authoring and in-game
  testing possible.

## License

[MIT](LICENSE). Titanfall 2 game files are not covered and are not distributed here.
