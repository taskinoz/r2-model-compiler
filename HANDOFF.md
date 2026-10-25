# Current handoff — 25 October 2026

The compiler is packaged and usable for Titanfall 2 static props, physics props,
and skinned/animated models, including Respawn root motion, bone scale animation,
IK rules, sequence weight lists, procedural bones and UI panel meshes. See README.md for commands and VALIDATION.md for exact evidence and
limitations.

## Deliverables

- build/r2studiomdl.exe: Windows x64 Release, static C++ runtime.
- dist/r2-model-compiler-win64.zip: executable, docs and three authored examples
  (rebuilt 25 October 2026 with build 90c58cc2...; ZIP SHA-256
  c8965753a621d6a274fdee28fa7e6a77b0d632ad4c8f13f5d195184b797f6ae5).
- examples/static_prop, examples/physics_prop, examples/animated_prop.
- Source, tests and examples are committed. Generated binaries, packages and
  corpus exports are ignored.

## Verified

- Both CTest groups pass, including fifteen Python regression tests.
- Fresh full decompile + compile of all 602 retail mp_common models with the release
  executable: no compile failures or exceptions, and 560/602 match the comparison
  (bones, meshes, sequence/animation flags, pose keys and weight lists, animation
  positions, delta values, bone scale, IK rules, procedural bones, physics and
  map-collision data). Seven models are 0.28-0.92 inches off in sampled animation
  positions, four differ by 0.27-0.50 in IK error samples, and 31 differ by 1-3 hull
  faces/edges/verts in a piece. All are listed in VALIDATION.md; no threshold was loosened.
- All six Blender exports (SMD, binary DMX 2/3/4/5/9) compare cleanly. Text
  keyvalues2 also matches its binary fixture.
- In game: physics box falls and rests; Blender head attachment moves; rebuilt grunt
  renders and ragdolls. The rebuilt prowler matches retail in pose (within 0.002
  units) and root motion (origin within 0.013 units over 13,867 units of travel);
  rebuilt drone_frag matches retail's scale-driven head lights, which a control
  without scale misses by 21-23 units. Quaternion-interpolation, aim-at (bone and
  attachment) and axis-interpolation procedural bones animate on the client (screenshots); the
  server's attachment queries ignore procedural bones. A rebuilt R-201 viewmodel whose ammo-counter panel was renamed
  (`models/r2c_test/My_Counter_Panel`, looked up from a weapon script in lower case)
  shows the counter, through the compiler's derived panel hash.
- All 35 retail models with UI panels (163 panels) round-trip with panel blocks
  byte-identical to retail, hashes included.
- Crowbar's command-line invocation is reproduced in a regression test. Its GUI
  was not automated.

## Reports

- test-output/release-corpus10/results.json: current full run with the release build
  (560/602, the same 42 differences as release-corpus9 with the previous build).
- test-output/prowler-ingame, scale-ingame, proc-ingame, axis-ingame, aim-ingame, ui-ingame: in-game
  pose, root-motion, scale, procedural-bone and UI panel evidence.
- test-output/ui-corpus2: round trip of the 35 retail models with UI panels.
- test-output/corpus-heading, corpus-euler: frame-movement yaw A/B runs.
- Earlier runs (release-corpus2-8, corpus-v2/coll/final/fixed) used older comparisons;
  only their results.json and logs are kept (work folders removed for disk space).
- tools/recheck_corpus.py recompiles unedited cached source exports, checking the
  retail input stamps and exporter/comparison tool hashes first.

## Recent fixes

Frame-movement tracks; retail RLE rules and quantization-grid recovery; delta
animations compiled as deltas (this was what made the rebuilt prowler lie on its
side in game); blend pose keys and sequence flags (snap, local, worldspace) carried
through decompilation; named $animation flags no longer copied onto sequences; bone
scale animation (SMD extension and DMX); jointed map-collision blocks; retail bone
fields; IK rules (with studiomdl's range quirks reproduced); separate sequence weight
lists (`seqweights`); jiggle bones through decompilation and `$proceduralbones`;
parent flag propagation limited to "used by" bits; `ikfixup`; UI panel meshes
(`$uipanel`) with the client's hash (case-folded MurmurHash3, found in client.dll). tools/ingame_test.py gained
--motion, --pose-attachments and --probe-times.

## Remaining parity work

`ikfixup` is compiled and covered by a regression test only (it is baked into
animations, so retail data cannot confirm Respawn's exact behaviour).
The retail yaw split between the frame-movement track and the root bone cannot be
recovered from exported SMDs, and hull reconstruction differs by a face or vertex
on 31 models. Titanfall 1 and Apex MDL files are not direct compiler inputs; R2 v53
is the target.

## Knowledge provenance

Gravemind projects/r2-model-compiler/index and games/respawn/modding/mdl-v53-format
hold the verified format rules and project status.
