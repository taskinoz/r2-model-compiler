# Current handoff — 24 October 2026

The compiler is now packaged and usable for Titanfall 2 static props, physics
props, and skinned/animated models. See README.md for commands and VALIDATION.md
for exact evidence and limitations.

## Deliverables

- build/r2studiomdl.exe: Windows x64 Release, static C++ runtime.
- dist/r2-model-compiler-win64.zip: executable, docs and three authored examples.
- examples/static_prop, examples/physics_prop, examples/animated_prop.
- This continuation builds on commit e50e1ed; source, tests and examples are
  committed together. Generated binaries, packages and corpus exports are ignored.

## Verified

- Both CTest groups pass, including ten Python regression tests.
- All 602 retail mp_common models decompile and compile successfully.
- The packaged release compiler recompiled all 602 cached QC/SMD exports:
  593 match the compared fields, with no compile failures or exceptions.
- Five models have sampled position differences of 0.27–0.92 inches from
  re-quantization. Four gain optional map-collision blocks. All nine are listed
  in VALIDATION.md; no comparison threshold was loosened to hide them.
- All six Blender exports (SMD, binary DMX 2/3/4/5/9) compare cleanly. Text
  keyvalues2 also matches its binary fixture.
- In game: physics box falls 100.719 units and rests on the floor; animated
  Blender head attachment moves 16.5427 units; rebuilt retail grunt renders
  beside original and resolves included ref animation; both original and rebuilt
  grunts become visible collapsed ragdolls. Individual joint limits not measured.
- Crowbar's command-line invocation is reproduced in a regression test. Its GUI
  was not automated.

## Reports

- test-output/corpus-final/results.json: full decompile and development rebuild.
- test-output/release-corpus/results.json: all 602 using release executable.
- test-output/release-corpus/run.json: exact executable/tool fingerprints.
- test-output/*-ingame-report.json and release screenshots: game evidence.
- tools/recheck_corpus.py recompiles unedited cached source exports, checking the
  retail input stamps and exporter/comparison tool hashes first.

## Fixes since the previous agent

Robust convex hulls; smoothing-group-aware collision decomposition; rounded RLE
quantization; accurate Quaternion64 packing and decoded-error-based choice of raw
or RLE rotation; noforceloop handling; quoted event options without breaking QC
Windows paths; BST text DMX unnamed elements; one-frame NaN event import; atomic,
resumable parallel corpus runs; explicit in-game validation reports and cleanup.

## Remaining parity work

IK rule/fixup processing, Respawn-native frame movement tracks, procedural bones,
and UI panel meshes remain incomplete. Legacy movement records are written.
Titanfall 1 and Apex MDL files are not direct compiler inputs. R2 v53 is the target.

## Knowledge provenance

Used Gravemind projects/r2-model-compiler/index revision 2 and
games/respawn/modding/mdl-v53-format revision 1. The project note has now been
updated to revision 3 with these verified results and limitations. Per-triangle
header size is 0x3C bytes, correcting the older format note's 0x34 claim.
