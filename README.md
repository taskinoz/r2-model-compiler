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
volume and selected attachment/collision counts. A clean comparison is evidence
for those fields; in-game behavior needs separate testing.

`tools/ingame_test.py` creates a separate Northstar profile, loads the supplied
models, runs a collision trace and captures a screenshot. For example:

```powershell
python tools\ingame_test.py --profile R2ModelTest --sequence bend out\model.mdl=models/r2c_test/model.mdl
```

`--probe-attachment name` measures an attachment during playback. `--physics`
spawns moving props above the floor and checks that they fall. `--report file.json`
writes the validation result; missing spawns, script errors, timeouts and missing
screenshots return a nonzero exit code. `--npc --ragdoll` exercises an NPC death
path, but a successful spawn/death call alone does not prove ragdoll physics.

To create the Windows package after building and testing:

```powershell
.\tools\package.ps1
```

Root motion: an `$animation`/`$sequence` with `rm` stores the root bones' yaw and
translation as a Respawn frame-movement track, as retail models do; `LX`/`LY`/`LZ`/
`LZR` motion extraction is stored the same way. Add `$legacymovement` to the QC to
write Valve-style piecewise movement records instead.

Limitations: animation IK rules/fixups, procedural bones and UI panel meshes remain
incomplete. Titanfall 1 and Apex model files are not direct compiler inputs;
this tool consumes QC/SMD/DMX and targets Titanfall 2 v53 output.
