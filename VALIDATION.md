# Tested Windows build — 25 October 2026

This build is usable for Titanfall 2 static props, moving physics props and skinned,
animated models, including Respawn root motion (frame movement), bone scale
animation, IK rules, sequence weight lists, procedural bones and UI panel meshes. It is not a claim of complete Valve/Respawn feature parity.

Executable: `build/r2studiomdl.exe` (Windows x64, MSVC Release, static C++ runtime).
SHA-256: `90c58cc2374b1e3e4618b90c8c3ef940f9686de797af1ccd847336d5540af009`.

## Automated checks

- Both CTest groups pass: the C++ collision hull suite and fifteen Python compiler/
  corpus-runner regression tests (including bone scale, IK rules, procedural bones and
  UI panels).
- Blender Source Tools exports from the same scene compile as SMD and binary DMX
  2, 3, 4, 5 and 9. All six outputs match in the compared fields, including two
  physics solids with total volume 24,576 cubic inches.
- Binary DMX and keyvalues2 versions of a mesh and animation produce matching
  models. The text fixture includes BST's unnamed-element syntax.
- A command-line test reproduces Crowbar's working directory, QC basename and
  studiomdl arguments, including a game path containing spaces. Crowbar's GUI
  itself has not been driven.
- The full corpus was run with the executable above (see "Retail corpus").

## Retail corpus

All **602/602** retail `mp_common` MDL v53 models decompiled and compiled without a
compile failure or decompiler exception. **560/602** match every compared field.

Reports:

- `test-output/release-corpus10/results.json` and `release-corpus10.log`: full run
  (fresh decompile and compile) with the executable above, now including UI panels in
  the comparison. The 42 differing models are exactly those of the previous build's
  run (`release-corpus9`, SHA-256 `0b386047...`, before `ikfixup` and UI panels).
- `test-output/ui-corpus2`: all 35 retail models with UI panels (163 panels), fresh
  decompile and compile with the executable above: 35/35 match, and every panel block
  (header hash, bones, vertices, faces, winding and bounds) is byte-identical to retail.
- `test-output/corpus-heading` and `test-output/corpus-euler`: A/B runs of the two
  frame-movement yaw definitions (heading kept: 7 vs 9 animation differences).

Reproduce a fresh run:

```powershell
python tools\roundtrip.py --jobs 8 --compiler build\r2studiomdl.exe --out test-output\fresh path\to\retail\models
```

To recompile existing, unedited exports with another build:

```powershell
python tools\recheck_corpus.py --from-report test-output\release-corpus10\results.json --compiler build\r2studiomdl.exe --jobs 8 --out test-output\recheck
```

The recheck verifies input model size/timestamps and the exporter/comparison tool
hashes. It assumes the cached QC/SMD exports have not been manually edited.

Compared fields: bone names/parents, bind translations, texture names, triangle
counts by LOD, sequence labels, sequence flags and blend pose keys, animation flags
and frame counts, sampled world positions of non-delta animations (with frame
movement applied), sampled raw local values of delta animations, sampled bone scale, IK rules (type,
chain, bone, slot, ranges, footstep parameters, targets and sampled error values),
sequence weight lists, procedural bones (type and parameters), UI panels (hash, name,
parent bones, vertices, faces, winding, bounds),
physics solid counts/volume, attachment count, the map-collision header count, the
face/plane/edge/vertex counts of every collision piece grouped by owning bone, and
each bone's collision reference. Entry order within the collision block is not
compared: retail jointed models order it by neither bone nor PHY solid. Animation
samples are first, middle and last frames. This is not a byte-for-byte comparison.

The 42 remaining differences are retained in the report:

| Models | Difference |
| --- | --- |
| `pov_pete_core`, `pete_scripted_skyway` | Sampled world position differs by 0.28 inches |
| `r2_prowler` | 0.30 inches |
| `dropship_common` | 0.44 inches |
| `droppod_fireteam` | 0.56 inches |
| `straton_anims` | 0.69 inches |
| `widow` | 0.92 inches |
| `titan_medium_sp_core`, `titan_heavy_sp_core`, `titan_medium_mp_reactions`, `titan_medium_mp_core` | IK footstep error samples differ by 0.27-0.50 (about 0.3-0.6 degrees at the comparison's 50-unit lever) |
| 31 models (titans, gibs, weapons, projectiles, `marvin`, `super_spectre_*`) | One or two collision pieces differ by 1-3 faces, bevels, edges or vertices |

The animation differences come from bones 6,000-54,000 units from the model origin
and from how yaw is split between the frame-movement track and the root bone. Retail
does not record that split, and our heading-based split is within 0.003-0.04 degrees
of it. The collision differences come from hull reconstruction: retail keeps
collinear points on hull edges and splits near-coplanar faces slightly differently.
Volumes and solid counts match. The IK residuals follow from the same animation
re-quantization: errors are recomputed from the decoded poses. None is claimed lossless.

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
- **Prowler pose and root motion** (`test-output/prowler-ingame/`): retail and
  rebuilt `r2_prowler` side by side. In `pr_combat_run_forward` the ORIGIN,
  CHESTFOCUS, HEADSHOT, foot and hand attachments agree within 0.002 units and
  0.004 degrees. Played with `Anim_PlayWithRefPoint` (`--motion`), the entity origin
  agrees within 0.013 units and yaw within 0.006 degrees, and the head attachment
  within 0.01 units, over `pr_walk_turn180_left`, `pr_combat_run_forward` and the
  23-second `pr_timeshift_campus_jumpscare`, which travels 13,867 units.
- **Bone scale** (`test-output/scale-ingame/`): retail and rebuilt `drone_frag`
  play `sd_closed_to_open`, whose scaled `def_c_scale_ball` carries the head lights.
  `head_light_1` matches retail at every sample. A control built without scale
  support is 21-23 units off at the start of the same sequence.

- **Procedural bones** (`test-output/proc-ingame/`): a test model whose `driver`
  bone turns 90 degrees. With `$proceduralbones`, the quaternion-interpolation
  follower rises to its 80-unit trigger position and the aim-at bar tilts to point at
  the rising target (`big.png`, `aim.png`); the same model without procedural bones
  stays at the bind pose. Server-side attachment queries do not evaluate procedural
  bones, so this was checked visually on the client.
- **Axis interpolation** (`test-output/axis-ingame/`, model generated by
  `tools/make_axis_test_model.py`): a `driver` bone yaws 0 to 90 degrees, and an axis-interpolation
  bone reads the driver's X axis. Its +X pose is the bind pose; its +Y pose is 60 units
  higher and rolled 45 degrees. At the end of `turn` the procedural model's cube sits
  in the raised, rolled pose, while the same model built without `$proceduralbones`
  stays at the bind pose (`shot.png`; the procedural model is on the left). Holding
  the driver at 45 degrees (`half`) leaves the cube about halfway up and partly rolled
  (`half.png`), as Valve's normalized blend predicts. These are visual checks.
- **Aim at attachment** (`test-output/aim-ingame/`, `tools/make_aim_test_model.py`):
  a bar on `aimer` aims either at the attachment `aim_att`, 40 units above the rising
  `target` bone and marked by the large cube (type 4), or at the bone itself, marked
  by the small cube (type 3). At the end of `rise`, the type-4 bar's extension passes
  through the large cube and the type-3 bar's through the small one
  (`shot2-annotated.png`; type 4 is on the left). In the three-model shot (`shot.png`)
  the build without procedural bones is hidden behind the viewmodel; the earlier
  without-procedural controls stayed at bind.
- **UI panels** (`test-output/ui-ingame/`): the R-201 viewmodel, mounted over
  `ptpov_rspn101`. Rebuilt from decompiled source it shows the ammo counter
  (`A_rebuilt.png`); with the panel hashes zeroed (`B_nohash.png`) or the panels
  removed (`C_nopanels.png`) the counter is gone. In `custom/D_custom.png` the
  counter panel was renamed `models/r2c_test/My_Counter_Panel` with no hash given, and
  a weapon-script override names it in lower case: the counter shows, so the derived
  hash (case-folded) is what the client looks up.

Structured reports and screenshots are under `test-output/`:
`physics-ingame-report.json`, `animation-ingame-report.json`,
`character-ingame-report.json`, `ragdoll-ingame-report.json`,
`prowler-ingame/report.json`, `prowler-ingame/report-pose.json`,
`scale-ingame/report.json`, `scale-ingame/report-control.json`,
`physics-release-ingame.png`, `animation-release-ingame.png`,
`character-release-ingame.png` and `ragdoll-release-control.png`.

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
  from the root's heading, quantized before removal so the roots absorb the error;
  delta animations carry an all-zero track as retail does.
- Retail RLE rules: one posscale per bone per animation, raw one-frame chains, no
  all-null position records, and recovery of an existing quantization grid so
  decompiled input recompiles losslessly.
- Delta animations: decompiled deltas are compiled as deltas (`$animation ... delta`).
  Previously they lost the delta flag, and autoplay head layers then reset the
  prowler's root, so it lay on its side in game.
- Blend pose keys (`posekeys`), `snap`, `local`, `worldspace`/`post` and
  `noanimation`/`noanim_keepduration` survive decompilation. A named `$animation`'s
  own flags are no longer copied onto its sequences, matching retail.
- Bone scale animation: SMD/DMX input, raw or RLE encoding, HAS_SCALE flags.
- IK rules: studiomdl's range inference, footstep/touch/attachment errors following
  the frame-movement track, compression, auto-release, rule indices and sequence
  counts; the decompiler reproduces retail's range-overwrite quirks.
- Sequence weight lists (`seqweights`) kept separate from the animations' encoding
  weights.
- Jiggle bones survive decompilation (and their studiomdl default base limits);
  `$proceduralbones` (.vrd quaternion-interpolation and aim-at bones, .txt axis
  interpolation). Only "used by" flags propagate to parent bones, so a jiggle bone's
  ancestors keep their animation.
- `ikfixup`: studiomdl's two-bone solve (Studio_SolveIK) re-poses the leg so the foot
  holds its contact position, using the intended frame-based weight curve.
- UI panel meshes (`$uipanel`): the RUI header/mesh layout, and the client's panel
  hash, read from client.dll's lookup: MurmurHash3 x86_32, seed 0, bytes ANDed with
  0xDF, length counting the nul. It reproduces all 163 retail hashes.
- Retail bone fields: zero qAlignment and 1/2048 scalescale.
- Map-collision block for jointed models (bone references), bevels only where
  dot(n0, n1) < 0.26, at most 63 faces per piece, and retail bone references
  for models without physics.

## Remaining feature limits

`ikfixup` is compiled (footstep/world rules); it
bakes into animation data, so retail models cannot show its effect and it is verified
by a regression test (planted foot held within 0.1 units, bone lengths preserved).
Root motion is written as Respawn frame-movement tracks (`$legacymovement` selects
Valve movement records).
Ragdoll joint limits have not been measured exhaustively. Titanfall 1 and Apex
MDL files are not direct inputs: this compiler reads QC/SMD/DMX and targets R2 v53.

The ZIP includes the executable, documentation and authored source examples. It
does not include the retail corpus or game textures/materials.
The package was rebuilt from a clean `dist` folder with the executable above and
extracted to a new folder; its `r2studiomdl.exe` has the SHA-256 above, and all three
bundled examples compiled with it.
