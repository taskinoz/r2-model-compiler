# Tested Windows build — 24 October 2026

This build is usable for Titanfall 2 static props, moving physics props and skinned,
animated models. It is not a claim of complete Valve/Respawn feature parity.

Executable: `build/r2studiomdl.exe` (Windows x64, MSVC Release, static C++ runtime).
SHA-256: `4d5f04b9303acd9858fc53e5219d35302dbe7f685a2412de45e0097e405130be`.

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
- The release build was tested against the full corpus (see below for the exact build).

## Retail corpus

All **602/602** retail `mp_common` MDL v53 models decompiled and compiled without a
compile failure or decompiler exception. With the stricter comparison below,
**564/602** match every compared field.

Reports:

- `test-output/release-corpus2/results.json` and `release-corpus2.log`: full run
  (fresh decompile and compile) with the release build. It was built from this
  source before a final comment/formatting-only edit; the final executable above
  was re-checked with CTest and a five-model spot run (`test-output/final-spot`).
- `test-output/corpus-heading` and `test-output/corpus-euler`: A/B runs of the two
  frame-movement yaw definitions (heading kept: 7 vs 9 animation differences).

Reproduce a fresh run:

```powershell
python tools
oundtrip.py --jobs 8 --compiler build
2studiomdl.exe --out test-outputresh path	o
etail\models
```

To recompile existing, unedited exports with another build:

```powershell
python tools
echeck_corpus.py --from-report test-output
elease-corpus2
esults.json --compiler build
2studiomdl.exe --jobs 8 --out test-output
echeck
```

The recheck verifies input model size/timestamps and the exporter/comparison tool
hashes. It assumes the cached QC/SMD exports have not been manually edited.

Compared fields include bone names/parents, bind translations, texture names,
triangle counts by LOD, sequence labels, animation frame counts and sampled
non-delta world positions (with frame movement applied), physics solid
counts/volume, attachment count, the map-collision header count, the
face/plane/edge/vertex counts of every collision piece grouped by owning bone, and
each bone's collision reference. Entry order within the block is not compared:
retail jointed models order it by neither bone nor PHY solid. Animation samples are
first, middle and last frames. This is not a byte-for-byte comparison.

The 38 remaining differences are retained in the report:

| Models | Difference |
| --- | --- |
| `pov_pete_core`, `pete_scripted_skyway` | Sampled world position differs by 0.28 inches |
| `r2_prowler` | 0.30 inches |
| `dropship_common` | 0.44 inches |
| `droppod_fireteam` | 0.56 inches |
| `straton_anims` | 0.69 inches |
| `widow` | 0.92 inches |
| 31 models (titans, gibs, weapons, projectiles, `marvin`, `super_spectre_*`) | One or two collision pieces differ by 1-3 faces, bevels, edges or vertices |

The animation differences come from bones 6,000-54,000 units from the model origin
and from how yaw is split between the frame-movement track and the root bone. Retail
does not record that split, and our heading-based split is within 0.003-0.04 degrees
of it. The collision differences come from hull reconstruction: retail keeps
collinear points on hull edges and splits near-coplanar faces slightly differently.
Volumes and solid counts match. Neither kind is claimed lossless.

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
- Respawn frame-movement tracks (`rm`, and LX/LY/LZ/LZR extraction): wrapped yaw
  from the root's heading, quantized before removal so the roots absorb the error.
  The decompiler bakes retail tracks into the roots and emits `rm`. **Frame-movement
  output has only been checked by round trip, not yet in game.**
- Retail RLE rules: one posscale per bone per animation, raw one-frame chains, and
  recovery of an existing quantization grid so decompiled input recompiles losslessly.
- Map-collision block for jointed models (bone references), bevels only where
  dot(n0, n1) < 0.26, at most 63 faces per piece, and retail bone references
  for models without physics.

## Remaining feature limits

Animation IK rules/fixups, procedural bones and UI panel meshes remain incomplete.
Root motion is written as Respawn frame-movement tracks (`$legacymovement` selects
Valve movement records).
Ragdoll joint limits have not been measured exhaustively. Titanfall 1 and Apex
MDL files are not direct inputs: this compiler reads QC/SMD/DMX and targets R2 v53.

The ZIP includes the executable, documentation and authored source examples. It
does not include the retail corpus or game textures/materials.
