# r2-model-compiler

A model compiler for Titanfall 2 (MDL v53). It reads a QC file with SMD/DMX sources,
as written by Crowbar decompiles or exported by Blender Source Tools, and writes a
single v53 `.mdl` with its VTX, VVD, VVC and PHY data embedded.

The compiler supports static props, physics props and skinned, animated models.
Generated models have been loaded in Titanfall 2 through Northstar, including a
Blender animation whose attachment movement was measured in game. See
[VALIDATION.md](VALIDATION.md) for the tested build and remaining limitations.

The Windows package contains `r2studiomdl.exe` and three source examples:

```powershell
.\r2studiomdl.exe -game "C:\Games\Titanfall2" examples\static_prop\box.qc
.\r2studiomdl.exe -game "C:\Games\Titanfall2" examples\physics_prop\physics_box.qc
.\r2studiomdl.exe -game "C:\Games\Titanfall2" examples\animated_prop\animated_prop.qc
```

To install into a Northstar mod, pass its `mod` folder as `-game`. The compiler
writes `models/<$modelname>` beneath that directory. Compiling creates the model
file; a map or script must still spawn an entity that references it. Physics props
need a `prop_data` keyvalues block, as demonstrated by the physics example.

Build with Visual Studio 2022 and CMake:

```powershell
.\build.bat
.\build\r2studiomdl.exe -game C:\Games\Titanfall2 path\model.qc
# Write to an explicit file instead:
.\build\r2studiomdl.exe -o out\model.mdl path\model.qc
```

Inputs include SMD and DMX (binary 1–5 and 9, plus keyvalues2). The command-line
test reproduces Crowbar's working directory, QC basename, `-game`, `-nop4`,
`-verbose` and `-nox360` arguments. To use Crowbar, select this executable as the
game setup's compiler and set the game directory to your mod. Crowbar's GUI has
not been tested directly.

Run the collision hull and corpus-runner regression tests:

```powershell
Push-Location build
ctest --output-on-failure
Pop-Location
# Optional direct Python invocation (also run by CTest when Python is available):
python -m unittest discover -s tests -p test_*.py
```

The hull tests check volume, containment and closed, consistently oriented edges
on near-duplicate points and a float-transformed Blender cylinder. They also
exercise shuffled inputs, different scales, interior points and degenerate inputs.

To compare decompiled retail models with rebuilt output:

```powershell
python tools\roundtrip.py --out test-output\retail path\to\models
# Process independent models in parallel:
python tools\roundtrip.py --jobs 8 --out test-output\retail path\to\models
# Continue an interrupted run using the same compiler, tools and output folder:
python tools\roundtrip.py --resume --out test-output\retail path\to\models
```

Each completed model is saved to `results.json` atomically. Resume skips recorded
results, including failures, when the input's size and timestamp are unchanged.
Changing the compiler or comparison tools requires a fresh run. Use a new output
folder, or omit `--resume`, to rerun failures. Models with the same basename use
separate work folders.

The comparison covers bone names and parents, bind positions, texture names,
triangle counts, sequence names, sampled animation world positions, collision
volume, UI panels and selected attachment/collision counts. A clean comparison is evidence
for those fields; in-game behavior needs separate testing.

`tools/ingame_test.py` creates a separate Northstar profile, loads the supplied
models, runs a collision trace and captures a screenshot. For example:

```powershell
python tools\ingame_test.py --profile R2ModelTest --sequence bend out\model.mdl=models/r2c_test/model.mdl
```

`--probe-attachment name` measures an attachment during playback, and
`--pose-attachments a,b,...` logs those attachments' origins and angles, which is a
quick way to compare a rebuilt model's pose with the original side by side.
`--motion seq1,seq2` plays each sequence with `Anim_PlayWithRefPoint`, so the
entity follows the sequence's root motion, and records origin, yaw and the probe
attachment at 0/25/50/75/95% of each sequence in the report. `--physics`
spawns moving props above the floor and checks that they fall. `--extra-file
local=gamepath` mounts another file in the test mod, such as a weapon script that
names a new UI panel. `--report file.json`
writes the validation result; missing spawns, script errors, timeouts and missing
screenshots return a nonzero exit code. `--npc --ragdoll` exercises an NPC death
path, but a successful spawn/death call alone does not prove ragdoll physics.

To create the Windows package after building and testing:

```powershell
.\tools\package.ps1
```

Bone scale: Respawn animations can scale bones. An SMD skeleton row may carry
three extra values after the rotation, the bone's local x/y/z scale (default 1):
`bone px py pz rx ry rz sx sy sz`. Valve's studiomdl reads only the first seven
values, so such files still load elsewhere. DMX joints may use a `scale` attribute
(float or vector) on their transform, animated through `scale` channels.
Animations that scale any bone are flagged as retail ones are (0x20000), as are their
sequences.

Other QC options for Respawn data: `$animation ... delta` marks a source that already
holds per-bone local deltas (as a decompiled delta animation does), and
`$sequence { ... posekeys <count> <values...> }` sets explicit blend positions where
retail blends are not evenly spaced. A named `$animation`'s own options (such as
`loop`) are not copied onto the sequences that use it; options written inside the
`$sequence` block are.

Procedural bones: `$jigglebone` blocks, and studiomdl's `$proceduralbones` files: a
`.vrd` file (`<helper>`/`<display>`/`<basepos>`/`<trigger>` quaternion-interpolation
bones and `<aimconstraint>` aim-at-bone/attachment bones) or a `.txt` axis-interpolation
file (`version 1` / `proceduralbones` / `bone ... axis` lines). The bones they drive
are kept even without vertices. Titanfall 2 evaluates these on the client (rendering);
server-side attachment queries do not see them.

IK: `$ikchain` and the studiomdl `ikrule` animation option are compiled (touch,
footstep, attachment, release and unlatch), including studiomdl's range inference
for `.` values, per-frame target errors (footsteps follow the frame-movement track),
their compressed streams, and automatic release rules for chains an animation moves
without rules (`noautoik` turns that off). Footstep rules default to Respawn's
`endheight -100`. `ikfixup` (footstep/world rules) re-solves the leg so the foot stays
on its contact position over the rule's range and bakes the result into the
animation. studiomdl's weight curve divides integers and is effectively 0 or 1; the
compiler uses the intended frame-based curve. `$weightlist` lists are honoured per animation, and a sequence's weights
are the largest of its animations'.

Root motion: an `$animation`/`$sequence` with `rm` stores the root bones' yaw and
translation as a Respawn frame-movement track, as retail models do; `LX`/`LY`/`LZ`/
`LZR` motion extraction is stored the same way. Add `$legacymovement` to the QC to
write Valve-style piecewise movement records instead.

UI panels (RUI meshes): the quads a weapon draws its screens on (ammo counters,
sights). A weapon script's `UiData` block names one with `"mesh" "models/..."`, and the
client finds it by a hash of that name. The compiler derives the hash (MurmurHash3
x86_32, seed 0, each byte ANDed with 0xDF so case does not matter, the terminating nul
counted in the length), so a new panel needs only a name:

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

A hex value after the name (`$uipanel "name" 0x1234abcd`) overrides the hash.
At most 256 vertices and 64 faces; only the first 32 faces can be `ccw`.
Decompiled retail panels are written without a hash when it matches the name's.

Limitations: Titanfall 1 and Apex model files are not direct compiler inputs;
this tool consumes QC/SMD/DMX and targets Titanfall 2 v53 output.
