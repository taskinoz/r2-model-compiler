# Tested Windows build — 24 October 2026

This build is usable for Titanfall 2 static props, moving physics props and skinned,
animated models. It is not a claim of complete Valve/Respawn feature parity.

Executable: `build/r2studiomdl.exe` (Windows x64, MSVC Release, static C++ runtime).
SHA-256: `8973e9634697f51b0aa5fe8cbec7022ea11f21e068335da363859b3df15137d5`.

## Automated checks

- Both CTest groups pass: the C++ collision hull suite and ten Python compiler/
  corpus-runner regression tests.
- Blender Source Tools exports from the same scene compile as SMD and binary DMX
  2, 3, 4, 5 and 9. All six outputs match in the compared fields, including two
  physics solids with total volume 24,576 cubic inches.
- Binary DMX and keyvalues2 versions of a mesh and animation produce matching
  models. The text fixture includes BST's unnamed-element syntax.
- A command-line test reproduces Crowbar's working directory, QC basename and
  studiomdl arguments, including a game path containing spaces. Crowbar's GUI
  itself has not been driven.
- The final release executable was tested directly against the full corpus.

## Retail corpus

All **602/602** retail `mp_common` MDL v53 models decompiled and compiled without a
compile failure or decompiler exception. The release executable recompiled the
saved source exports in a separate pass; **593/602** match every compared field.

Reports:

- `test-output/corpus-final/results.json`: complete export and initial rebuild.
- `test-output/release-corpus/results.json`: complete check using the release EXE.
- `test-output/release-corpus/run.json`: compiler and comparison-tool fingerprints.

Reproduce a fresh run:

```powershell
python tools\roundtrip.py --jobs 8 --compiler build\r2studiomdl.exe --out test-output\fresh path\to\retail\models
```

To recompile existing, unedited exports with another build:

```powershell
python tools\recheck_corpus.py --from-report test-output\corpus-final\results.json --compiler build\r2studiomdl.exe --jobs 8 --out test-output\recheck
```

The recheck verifies input model size/timestamps and the exporter/comparison tool
hashes. It assumes the cached QC/SMD exports have not been manually edited.

Compared fields include bone names/parents, bind translations, texture names,
triangle counts by LOD, sequence labels, animation frame counts and sampled
non-delta world positions, physics solid counts/volume, attachment count and the
presence of map-collision data. Animation samples are first, middle and last
frames. This is not a byte-for-byte comparison or exhaustive animation/event test.

The nine remaining comparison differences are retained in the report:

| Model | Difference |
| --- | --- |
| `pete_scripted_skyway` | Sampled world position differs by 0.27 inches |
| `droppod_fireteam` | Sampled world position differs by 0.43 inches |
| `straton_anims` | Sampled world position differs by 0.72 inches |
| `widow` | Sampled world position differs by 0.92 inches |
| `dropship_common` | Sampled world position differs by 0.27 inches |
| `battery_port_animated` | Output includes a map-collision block absent in the input |
| `drone_air_attack_rockets` | Output includes a map-collision block absent in the input |
| `drone_air_attack_plasma` | Output includes a map-collision block absent in the input |
| `projectile_arc_ball` | Output includes a map-collision block absent in the input |

Large world-space animation positions are re-quantized into the format's 16-bit
position streams; these five animations are not claimed lossless. The extra
collision blocks are structurally valid generated data, but remain differences
from the retail files.

## In-game checks (Northstar)

- A compiled box loads, renders and is hit by a collision trace.
- The moving physics-box example falls **100.719 units** and settles on the floor.
  The example includes `prop_data`, which the game requires for a physics prop.
- The animated Blender example plays `bend`; its head attachment moves
  **16.5427 units** between samples. Its collision trace also hits the model.
- The rebuilt retail grunt renders beside the original with matching appearance
  and reference pose. Its included animation library resolves `ref` successfully.
- Retail and rebuilt grunt NPCs both converted into visible, collapsed ragdolls
  on the floor in the side-by-side screenshot. Individual joint-limit angles were
  not measured.

Structured reports and screenshots are under `test-output/`:
`physics-ingame-report.json`, `animation-ingame-report.json`,
`character-ingame-report.json`, `ragdoll-ingame-report.json`,
`physics-release-ingame.png`, `animation-release-ingame.png`, and
`character-release-ingame.png`, and `ragdoll-release-control.png`.

## Fixes in this continuation

- Robust hull welding/visibility and farthest-first construction.
- Collision smoothing-group separation, including touching convex pieces.
- Rounded animation quantization, accurate Quaternion64 packing and selection
  between raw and RLE rotations based on decoded precision.
- `noforceloop` now preserves supplied loop endpoints.
- Escaped event options preserve literal quotes while QC paths retain their
  literal backslashes, including a trailing directory separator.
- Text DMX unnamed elements and malformed-string diagnostics.
- Decompiler support for one-frame retail events with NaN cycle times.
- Checkpointed/resumable parallel corpus runs and explicit in-game pass/fail reports.

## Remaining feature limits

Animation IK rules/fixups, Respawn-native frame-movement tracks, procedural bones
and UI panel meshes remain incomplete. Legacy movement records are emitted.
Ragdoll joint limits have not been measured exhaustively. Titanfall 1 and Apex
MDL files are not direct inputs: this compiler reads QC/SMD/DMX and targets R2 v53.

The ZIP includes the executable, documentation and authored source examples. It
does not include the retail corpus or game textures/materials.
