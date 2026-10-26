# Titanfall 2 validation — 27 October 2026

This build is validated against 4,024 distinct retail Titanfall 2 models: all
602 multiplayer common models and 3,422 unique single-player models. It also has
authored-input regressions and Northstar runtime checks. This is finite evidence
for the tested files and features, rather than a guarantee for every possible QC
or a claim of complete Valve/Respawn studiomdl feature parity.

Windows x64 MSVC Release executable, with the static C++ runtime:
`build/r2studiomdl.exe`.
SHA-256: `9fda259f52d3ef03a3899de1b1ecc6b90f816f9e009554902ee67637fd5ea9bc`.

The reports below live under `test-output/` and are not distributed: they contain
paths to assets extracted from the user's own installation. Reproduce the runs
using your own retail corpus; no game models are included in this repository.

## Automated checks

- Both CTest groups pass: collision hulls and compiler regressions.
- All 62 Python tests pass through the ordinary and locked `uv run` routes.
- Regressions cover SMD and binary/keyvalues2 DMX, Blender Source Tools fixtures,
  local bind scale and inverse binds, Euler poles and quantization endpoints,
  animation section boundaries, raw/RLE precision, root movement, IK samples,
  procedural bones, UI panels, followers, skin weights, mesh attributes and LODs.
- Physics tests cover authored mass centers and damping, per-joint properties and
  constraints, native surface retention, malformed indices, nearly coplanar hulls,
  unit-conversion precision, map planes, triangle collision and safe corpus cleanup.
- A Crowbar-style command-line invocation is covered, including paths with spaces.
  Crowbar's GUI has not been driven.
- The clean ZIP is extracted into a new directory, its compiler hash is verified,
  and all three examples are compiled, decompiled, recompiled and independently
  validated using the packaged tools.

## Complete retail run

`test-output/complete-validation-v4/results.json` records fresh decompile, compile
and summary comparison results. `complete-validation-v4-dense/results.json`
records the separate, full-frame validation. Every input and rebuilt file is
stamped; compiler/tool hashes, runtime versions and thresholds are recorded in
the run manifests. Rebuilt model SHA-256 values are recorded per dense result.

Both runs pass **4,024/4,024**, with zero differences, compile failures or exceptions.
All nine failures from the preceding dense run are fixed, with no newly failing models.

| Coverage | Count |
| --- | ---: |
| Animation frames | 1,742,222 |
| Bone poses | 211,669,360 |
| Stored IK error samples | 122,472 |
| Collision triangles | 11,529,379 |
| Render triangles | 35,054,644 |
| PHY solids | 3,856 |
| PHY pieces | 9,410 |
| Map brushes | 9,400 |

Maximum animation world-position error is 0.030447 inches; maximum local rotation error
is 0.002969 degrees. Scale error is zero. Used render positions and attributes match
exactly; maximum normalized weight error is 1.78813934e-07.


The dense checks cover:

- Bone hierarchy, finite positive bind scale, inverse binds, finite vertex data,
  normalized skin weights, VTX indices and binary spans.
- Every stored animation frame and bone: local translation, rotation and scale,
  hierarchical world position for non-delta animations, exact stored frame
  movement and legacy records, IK targets and every stored IK error sample.
- Every oriented render triangle, including position, normal, tangent, UV/UV2,
  vertex colour and bone weights; material assignments across every skin and LOD.
- Exact quantized triangle-collision geometry, plus acceleration-tree reachability,
  bounds and triangle references. The compiler constructs its own tree.
- PHY solid properties, masses, damping, materials, parents, joint constraints,
  exclusions, friction and break blocks; closed reciprocal edges and piece geometry.
- Map-collision ownership, piece counts, vertex clouds and literal face/bevel
  signed-distance functions over the complete model bounding box.
- Exact header hull/view bounds and flags, bone flags, bodygroup and logical model
  names, including named empty bodygroup entries.

Thresholds are unchanged from the initial dense validation:

| Measurement | Limit |
| --- | --- |
| Animation local/world position | 0.25 inches |
| Animation local rotation | 0.01 degrees |
| Animation scale | 0.01 |
| Render position | 0.001 inches |
| Render attributes and weights | 0.00001 |
| PHY / map-brush geometry | 0.01 inches |

Native PHY ledges can be slightly nonconvex. Identical oriented triangle surfaces
prove exact equality without extending their bounded triangles to infinite
supporting planes. Map brushes retain their native planes: recomputing a nearly
collinear face plane from rounded vertices can change its collision region.
These are explicit source-preservation rules, not increased tolerances.

The previous sampled-summary build passed all 4,024 models but missed real defects:
bind scale, small skin weights, supplied mesh attributes, one-frame precision,
Euler branches near 90/180 degrees, animation section boundaries, rigid physics
transforms and PHY properties. Those defects have regressions and are corrected.
The first dense run with the preceding compiler found nine remaining failures;
the new run compares all files again, rather than only those nine.
Exact metadata checks additionally exposed a 64-byte model name being truncated
to 63 bytes in `ptpov_test`. The writer now retains the complete fixed-width field;
the regression and a Northstar prop with a full-width name both pass.

```powershell
.\build.bat
Push-Location build; ctest --output-on-failure; Pop-Location
uv run --locked python -m unittest discover -s tests -p "test_*.py"
python -I tools\roundtrip.py --compiler build\r2studiomdl.exe --jobs 8 --discard-sources --out test-output\fresh <mp-models> <sp-unique-models>
python -I tools\validate_corpus.py --report test-output\fresh\results.json --out test-output\dense --jobs 8
python -I tools\structdiff.py --report test-output\fresh\results.json --out test-output\fields.json
```

The recorded dense run overlaps compilation by calling the same
`validate_corpus.check` function as files finish. It checks the final source report
hash and unchanged tool hashes. The two sequential public commands above reproduce
the same comparisons without that overlap. Generated exports are pruned only in
owned work directories; the rebuilt MDLs and checkpoint reports are retained.

## Field differences and limits

The final field audit compares all 4,024 models and has eight differing field paths,
recorded in `test-output/complete-validation-v4-structdiff.json`. They are:

| Fields | Models | Explanation |
| --- | ---: | --- |
| Tree node/leaf counts | 1549 | The compiler constructs a different valid partition; decoded collision triangles and tree bounds are verified |
| Tree vertex count | 2 | `wf_platform_04_highway` and `prowler_tower_full` need different vertex duplication when rebasing 16-bit leaf indices; decoded geometry is unchanged |
| Texture ordering and mesh material indices | 2 | `ptpov_test` and `straton_imc_gunship_01_1000x`; every actual material assignment across skins and LODs matches |
| Skin indices | 1 | The gunship uses the corresponding reordered material indices; resolved skin materials match |
| Model type and bounding radius | 1 | The retail `ptpov_test` has overlapping structures and nonsensical values; the compiler writes normal empty-model fields |


Byte identity is not required. Offsets, checksums, string placement, animation
encoding, triangulation of newly authored hulls and acceleration-tree layout may
differ while representing the same tested data. Native collision sidecars retain
typed geometry, not an opaque compiled PHY block: edge links and trees are rebuilt.

Decompiler preservation options must be removed when editing the corresponding
source data. Retained motion/IK tracks, model-space mesh data, collision planes,
native triangulation and mass properties intentionally reproduce the original;
they cannot infer an editor's intended changes. README.md explains these options.

The corpus does not exhaust all maps, every possible source authoring command,
every malformed file or every runtime behavior. Full-frame comparisons decode
stored animations; they do not execute every possible layered sequence, IK state,
procedural bone or entity interaction in the game. The runtime checks below cover
representative cases. No threshold was widened to make a model pass.

## Northstar runtime evidence

Final-build evidence is under `test-output/validated-ingame/`:

| Check | Evidence |
| --- | --- |
| Static prop | `complete-static.json`: renders and the trace hits its top at 64.0313 inches |
| Full-width model name | `complete-full-width.json`: a prop retaining all 64 name bytes loads, renders and collides |
| Physics prop | `complete-physics.json`: falls 100.063 inches and settles |
| Authored animation | `complete-animated.json`: `bend` moves its probe 16.5428 inches |
| Bone scale | `complete-scaled.json`: probe distance grows from 10.4883 to 20.0098 inches across five samples |
| One-frame precision | `complete-precision.json`: one-frame RLE and two-frame control probes agree at logged precision at both times, including 300.1/-100/0.0999602-inch offsets |
| Procedural control | `complete-axis-visible.json/.png`: the cube rises and rolls with axis interpolation; a control omits the procedural definition |
| Retail ragdolls | `complete-ragdoll-ref.json/.png`: retail and rebuilt grunts resolve included `ref`, both traces hit, and both form visible collapsed ragdolls; individual joint-limit angles are not measured |
| Repaired retail hull | `complete-generator.json/.png`: both generators load and render alike; the central trace passes through the opening on both |
| Viewmodel UI | `complete-viewmodel-final.json/.png`: the final rebuilt R-201 replaces its retail viewmodel and displays the ammo counter |

`final-runtime-byte-verification.json` ties all twelve model/control files to the
final compiler. Models tested before the last metadata-only compiler change were
recompiled and proved byte-identical to the mounted test files. The changed R-201
was tested again with its final bytes.


Reports include the SHA-256 of each mounted model, spawn and script status, sampled
motion when requested, and screenshot/log paths. Server attachment queries do not
evaluate client-only procedural bones; those require visual control comparisons.
Viewmodel load checks use explicit `--no-animation` mode when no sequence is
requested. Models without local sequences can still resolve included animations,
as the grunt's `ref` test demonstrates.

Earlier runtime checks also established root-motion playback, bone scale, IK,
quaternion/axis/aim procedural bones, bone followers and viewmodel UI panels.
These earlier reports are historical evidence, separate from the final-build
checks listed above.
