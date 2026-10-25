# Tested Windows build — 26 October 2026

This build is usable for Titanfall 2 static props, moving physics props and skinned,
animated models, including Respawn root motion (frame movement), bone scale
animation, IK rules, sequence weight lists, procedural bones, bone followers and UI
panel meshes. It is not a claim of complete Valve/Respawn feature parity.

Executable: `build/r2studiomdl.exe` (Windows x64, MSVC Release, static C++ runtime),
built from commit 0cc71f0 (not yet released). SHA-256:
`158f40dfde02193e15d853e4472a791ad86dc8eedcb2a5a32705a4f66f8f2154`.
(v0.4.0 shipped `e94ee0a3...`, v0.3.0 `a6370348...`, v0.2.0 `67fb263a...` and v0.1.0
`90c58cc2...`. The changes since v0.4.0 are under "Field-level comparison"; those since
v0.2.0 are under "Collision faces from source polygons".)

Paths under `test-output/` name the local report folders the tools produced (JSON
results, logs and screenshots). They are not part of the repository; the commands
below regenerate them from your own copy of the game.

## Automated checks

- Both CTest groups pass: the C++ collision hull suite and twenty-three Python compiler/
  corpus-runner regression tests (including bone scale, IK rules, procedural bones,
  UI panels, bone followers with per-joint concave collision, DMX UV2/vertex
  colour, IK ranges that studiomdl overwrote in order, DMX polygons as map-collision
  faces, `$nomapcollision`, DMX tangents and the retail-value QC extensions).
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
compile failure or decompiler exception. **591/602** match every compared field.

Reports:

- `test-output/mp-final2/results.json` and `mp-final2.log`: full run (fresh decompile
  and compile) with the executable above, and `structdiff-mp-final2.json` for its
  field-level comparison. No collision differences remain; v0.2.0 matched 559/602,
  v0.3.0 587/602 and v0.4.0 591/602.
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
python tools\recheck_corpus.py --from-report test-output\release-corpus17\results.json --compiler build\r2studiomdl.exe --jobs 8 --out test-output\recheck
```

The recheck verifies input model size/timestamps and the exporter/comparison tool
hashes. It assumes the cached QC/SMD exports have not been manually edited.

Compared fields: bone names/parents, bind translations, texture names, triangle
counts by LOD, sequence labels, sequence flags and blend pose keys, animation flags
and frame counts, sampled world positions of non-delta animations (with frame
movement applied), sampled raw local values of delta animations, sampled bone scale, IK rules (type,
chain, bone, slot, ranges, footstep parameters, targets and sampled error values),
sequence weight lists, procedural bones (type and parameters), UI panels (hash, name,
parent bones, vertices, faces, winding, bounds), bone followers,
physics solid counts/volume, attachment count, the map-collision header count, the
face/plane/edge/vertex counts of every collision piece grouped by owning bone, and
each bone's collision reference. Entry order within the collision block is not
compared: retail jointed models order it by neither bone nor PHY solid. Animation
samples are first, middle and last frames. This is not a byte-for-byte comparison.

The 11 remaining differences are retained in the report:

| Models | Difference |
| --- | --- |
| `pov_pete_core`, `pete_scripted_skyway` | Sampled world position differs by 0.28 inches |
| `r2_prowler` | 0.30 inches |
| `dropship_common` | 0.44 inches |
| `droppod_fireteam` | 0.56 inches |
| `straton_anims` | 0.69 inches |
| `widow` | 0.92 inches |
| `titan_medium_sp_core`, `titan_heavy_sp_core`, `titan_medium_mp_reactions`, `titan_medium_mp_core` | IK footstep error samples differ by 0.27-0.50 (about 0.3-0.6 degrees at the comparison's 50-unit lever) |

The animation differences come from bones 6,000-54,000 units from the model origin
and from how yaw is split between the frame-movement track and the root bone. Retail
does not record that split, and our heading-based split is within 0.003-0.04 degrees
of it. The collision differences are explained under "Collision faces from source
polygons" below.
Volumes and solid counts match. The IK residuals follow from the same animation
re-quantization: errors are recomputed from the decoded poses. None is claimed lossless.

## Single-player corpus

The 13 single-player map VPKs (`englishclient_sp_*.bsp.pak000_dir.vpk`) hold 11,709
models. 4,997 are byte-identical to `mp_common` models; the other **3,422** unique v53
models were decompiled and compiled with the executable above. There were no compile
failures or decompiler exceptions, and **3,416/3,422** match every compared field
(`test-output/sp-final2`; v0.2.0 matched 3,223, v0.3.0 3,384 and v0.4.0 3,411). No single-player model uses flexes,
eyeballs or vertex animation, and none sets the UV2 flag.

This corpus found problems the multiplayer set does not exercise. They are fixed in
this build:

- **Bone followers** (28 animated props such as doors, cranes and platforms) are
  decompiled, compared, kept through bone pruning, and written after the collision
  block, as retail stores them.
- **`$concaveperjoint`**: jointed models whose joints own several convex pieces
  (20 models) carry no `"concave"` key in the PHY text. `$concaveperjoint` now splits
  each joint without setting that key, and the decompiler emits it. Before the fix
  these pieces merged into single hulls, with 1.3-80 times the volume.
- **LOD order**: studiomdl keeps `$lod` script order even when switch distances are
  not ascending (retail has 9 such models); the compiler no longer sorts them.
- **Map-collision limit**: single-player rocks keep pieces with up to 82 faces and 99
  planes. The block is now omitted only when a piece exceeds the byte-sized plane or
  vertex indices (255), instead of above 63 faces.
- **IK ranges overwritten in order**: a `.` range value rewrites a neighbouring rule's
  end or start after that rule's errors were computed. When the rewritten range stays
  in order, only the error stream's length shows it. The decompiler now recovers the
  original range from the stream; a footstep's zero error at its contact frame tells
  a rewritten start from a rewritten end. This took `titan_buddy_sp_core` from 59.17
  to an ordinary 0.72 residual.

The 6 remaining differences (the three trinity models and the two giantfan panels
now match, through `$staticcollisioncount` and mesh-only vertices):

| Models | Difference |
| --- | --- |
| `titan_buddy_mp_core` | One attachment rule's error stream carries a constant rotation offset of about 60 degrees in every frame, while the rule's own orientation matches; the sibling animation matches. Treated as stale retail data |
| `titan_buddy_sp_core`, `titan_buddy_scripted_s2s` | IK footstep error samples off by 0.72 and 0.27 (re-quantization over animations of up to 600 frames) |
| `birmingham_fleetscale`, `vehicle_draconis_hero_animated` | Sampled world positions off by 1.24 and 0.58 inches (large, distant animated ships) |
| `titan_buddy_embark` | Retail sets HAS_SCALE (0x20000) on all 54 animations although only one scales; in the 14 other models with scaled animations the flag marks exactly those |

## Field-level comparison (structdiff)

`tools/structdiff.py` dumps every v53 structure (header, bones, sequences,
animations, body parts, meshes, VTX strips, VVD/VVC headers and fixups, hitboxes,
attachments, IK, textures, skins, keyvalues, per-triangle header) into flat field
paths. It compares the rebuilt copy of each model against retail, leaving out offsets,
sizes and checksums. It goes well beyond the summary comparison above.

```powershell
python -I tools\structdiff.py --report test-output\mp-final2\results.json --out test-output\structdiff-mp-final2.json
```

At v0.4.0, 97 field paths differed in `mp_common`. With this build, 17 differ in
`mp_common` and 22 in the single-player corpus. The rules found and fixed on the way
are listed in `docs/mdl-v53-format.md`:
- vertex identity by tangent, and retail vertex numbering kept;
- fixup tables and their LOD counts;
- sequence-box rules, and retail boxes kept;
- bone and header flags;
- unaligned bone quaternions;
- material, mesh, solid and node order;
- names kept as written;
- the per-triangle tree on `allowstatic` props;
- mesh vertices without triangles;
- the string-table opening;
- bounds initialisation quirks.

What still differs, and why:

| Field | Models (MP / SP) | Reason |
| --- | --- | --- |
| Per-triangle tree node/leaf/vertex counts, box | 92 / 1,549 | Retail splits small nodes by a rule not reproduced (leaves of 0-12 triangles), and builds its vertex list from source data the file does not keep. The tree is a collision acceleration structure; it is rebuilt valid, not identical |
| Body-part model names | 124 / 43 | Retail source paths that are absolute, start with `/` or `..`, lack an extension or repeat within a model. The decompiler writes generated names for these |
| Bone vertex-LOD flags | 13 / 32 | Bones retail flags without any stored vertex using them (group bones such as `grp_l_leg_lod0`), probably from zero-weight source weights the VVD drops |
| `anims[*].nummovements` | 2 / 0 | A legacy Valve movement record on two animations that otherwise use no motion flags |
| Bone rotation sign | 1 / 0 | `dropship_common`: one bone's roll is +π in retail and −π after the round trip (the same rotation) |
| `ptpov_test` texture / model type | 1 / 0 | A retail test model whose structures overlap other data |
| Header flags | 0 / 3 | One multi-bone model with 0x800000 (`wf_platform_4`) and two single-bone models, still to be explained |
| Generated hitbox bounds | 0 / 3 | Generated-hitbox vertex sets differ by vertices outside LOD 0, like the per-triangle vertex list |
| Bone scale and animation flags | 0 / 1 | `titan_buddy_embark`: see the table above |

## Collision faces from source polygons

A survey of every retail map-collision block showed why collision face counts differed
from retail:
- Retail faces are polygons of the source collision mesh: 33,223 triangles, 39,523
  quads and 2,404 larger polygons such as cylinder caps, consistent with Respawn's Maya
  pipeline.
- Retail keeps exactly coplanar neighbours apart (0.0 degrees, 0.00002 units).
- Retail merges slightly bent quads, up to 0.01 units off plane.

Since v0.2.0 the compiler takes faces from the source polygons:
- The DMX reader records which polygon each triangle came from.
- The hull's triangles are tagged with the polygon whose points they share. A
  triangle that bulges just past a slightly bent polygon joins it. When convex pieces
  touch flush, a piece's own polygon wins over its neighbour's.
- Each polygon becomes one face, and SMD triangles still merge by plane.
- The decompiler writes the physics mesh as a DMX of retail's face polygons, taken
  from the block, whose vertices are exactly the PHY points.
- `$nomapcollision` reproduces the six legacy retail models built without the block.

Since v0.3.0 the faces are built from the polygon outlines themselves, not from
triangles tagged with a polygon:
- **Per piece:** each convex piece uses only its own polygons.
- **Faces and edges:** a face's plane comes from its loop, and an edge is a loop edge
  shared with the face whose loop runs the other way.
- **T-junctions:** a corner partway along a neighbour's edge splits that edge.
- **Self-touching faces:** a face whose outline passes a vertex twice (three
  `sewer_vent_large` models) pairs its edges by shared corners.
- **Surface points:** points lying on a piece's surface without being corners stay as
  hull vertices, in the PHY triangles and the block. Examples are cylinder-cap centres
  and edge midpoints; 46 retail pieces list such points. The decompiler writes the
  ones no face uses as loose DMX vertices.

Collision differences fell from 31 multiplayer and 185 single-player models (v0.2.0)
to none, apart from the three `trinity` models' stored count of -1.

Regression tests cover:
- a box whose top quad is bent by 0.2 units (6 faces as DMX polygons, 7 as SMD
  triangles);
- a box with a split top, an edge midpoint, a loose point and a T-junction.

The decompiled physics DMX imports into Blender (Blender Source Tools) as closed,
connected meshes, one per piece, mostly quads, with loose points kept.

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
- **Polygon collision** (`test-output/poly-ingame/`): retail `sandbags_curved_01`,
  decompiled with its physics as DMX polygons and rebuilt (it matches retail in every
  compared field), renders identically next to the original. A downward trace at its
  origin hits the rebuilt prop's top (z 39.99). The origin sits near one end of the
  curved shape; the same trace through the retail copy at the same spot reached the
  floor, so the trace grazes the edge and does not compare the two collisions. The
  other spot was under an overhang for either model.
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

## Parity work behind these results

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
The v0.1.0 package was rebuilt from a clean `dist` folder and extracted to a new
folder. Its `r2studiomdl.exe` (`90c58cc2...`) compiled all three bundled examples.
