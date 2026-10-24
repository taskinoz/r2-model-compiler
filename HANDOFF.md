# Current handoff — 25 October 2026

The compiler is packaged and usable for Titanfall 2 static props, physics props,
and skinned/animated models, including Respawn root motion and bone scale
animation. See README.md for commands and VALIDATION.md for exact evidence and
limitations.

## Deliverables

- build/r2studiomdl.exe: Windows x64 Release, static C++ runtime.
- dist/r2-model-compiler-win64.zip: executable, docs and three authored examples.
- examples/static_prop, examples/physics_prop, examples/animated_prop.
- Source, tests and examples are committed. Generated binaries, packages and
  corpus exports are ignored.

## Verified

- Both CTest groups pass, including eleven Python regression tests.
- Fresh full decompile + compile of all 602 retail mp_common models with the release
  executable: no compile failures or exceptions, and 564/602 match the comparison
  (bones, meshes, sequence/animation flags and pose keys, animation positions, delta
  values, bone scale, physics and map-collision data). Seven models are 0.28-0.92
  inches off in sampled animation positions; 31 differ by 1-3 hull faces/edges/verts
  in a piece. All are listed in VALIDATION.md; no threshold was loosened.
- All six Blender exports (SMD, binary DMX 2/3/4/5/9) compare cleanly. Text
  keyvalues2 also matches its binary fixture.
- In game: physics box falls and rests; Blender head attachment moves; rebuilt grunt
  renders and ragdolls. The rebuilt prowler matches retail in pose (within 0.002
  units) and root motion (origin within 0.013 units over 13,867 units of travel);
  rebuilt drone_frag matches retail's scale-driven head lights, which a control
  without scale misses by 21-23 units.
- Crowbar's command-line invocation is reproduced in a regression test. Its GUI
  was not automated.

## Reports

- test-output/release-corpus5/results.json: current full run with the release build.
- test-output/prowler-ingame, scale-ingame: in-game pose, root-motion and scale reports.
- test-output/corpus-heading, corpus-euler: frame-movement yaw A/B runs.
- Earlier runs (release-corpus*, corpus-final) used older comparisons.
- tools/recheck_corpus.py recompiles unedited cached source exports, checking the
  retail input stamps and exporter/comparison tool hashes first.

## Recent fixes

Frame-movement tracks; retail RLE rules and quantization-grid recovery; delta
animations compiled as deltas (this was what made the rebuilt prowler lie on its
side in game); blend pose keys and sequence flags (snap, local, worldspace) carried
through decompilation; named $animation flags no longer copied onto sequences; bone
scale animation (SMD extension and DMX); jointed map-collision blocks; retail bone
fields. tools/ingame_test.py gained --motion and --pose-attachments.

## Remaining parity work

IK rule/fixup processing, procedural bones and UI panel meshes remain incomplete.
The retail yaw split between the frame-movement track and the root bone cannot be
recovered from exported SMDs, and hull reconstruction differs by a face or vertex
on 31 models. Titanfall 1 and Apex MDL files are not direct compiler inputs; R2 v53
is the target.

## Knowledge provenance

Gravemind projects/r2-model-compiler/index and games/respawn/modding/mdl-v53-format
hold the verified format rules and project status.
