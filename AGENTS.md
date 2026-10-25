# Agent guide

How to work on this repository: commands, test tools, the validation standard, and
pitfalls found while building it. Read `README.md` for user-facing behaviour,
`VALIDATION.md` for the evidence behind the current release, and
`docs/mdl-v53-format.md` for the binary format.

## Layout

- `src/qc`: QC tokenizer and parser. It follows studiomdl's line semantics
  (`$include`, `$definevariable`, `$definemacro`) and adds the Respawn-era options that
  rsx and Crowbar emit for v53 decompiles.
- `src/input`: SMD and DMX readers (binary 1-5 and 9, keyvalues2).
- `src/build`: bones, meshes and LODs, animation processing and v53 RLE encoding,
  frame movement, IK, procedural bones, UI panels, collision (convex hulls, PHY,
  map-collision block) and the per-triangle AABB tree.
- `src/write`: MDL v53, VTX v7, VVD v4 and VVC writers.
- `tests/`:
  - `hull_tests.cpp`: C++ convex-hull tests;
  - `test_compiler.py`: compiler regressions;
  - `test_roundtrip.py`: corpus-runner tests;
  - `blender/`, `dmx/`: input fixtures.
- `tools/`: Python decompiler, comparison and in-game test tools (below).

## Build and test

```powershell
.\build.bat                      # Release x64 into build\ (finds VS 2022 via vswhere)
.\build-dev.bat                  # a second copy in build-dev\, e.g. for A/B runs
Push-Location build; ctest --output-on-failure; Pop-Location
python -m unittest discover -s tests -p "test_*.py"
uv run python -m unittest discover -s tests -p "test_*.py"   # same, through uv
```

The Python side needs 3.10+ and the standard library only (tested on 3.10 and 3.14).
`pyproject.toml` / `uv.lock` let `uv run` work on any script in `tools/`. Keep it
dependency-free; analysis-only packages (capstone, pefile, Pillow) are not
dependencies. GitHub Actions (`.github/workflows/ci.yml`) runs the same build, CTest
and the uv test route on Windows for every push to `main` and every pull request, and
keeps the executable as an artifact. Set `R2C_COMPILER` to test another executable. Add a regression test in
`tests/test_compiler.py` for every feature. The existing tests show the pattern:
- write a small SMD/QC with `dc.write_smd`;
- compile it;
- decode the result with `tools/decompile53.py` helpers;
- decompile and recompile, then assert `roundtrip.compare(...) == []`.

## Tools

- `tools/mdl53.py`: independent v53 reader (header, bones, sequences, animations,
  RLE decode). Use it for surveys and byte-level checks.
- `tools/decompile53.py`: v53 `.mdl` to QC + SMD (+ `.vrd`/`.txt` procedural files),
  written so that the compiler can rebuild the model. When the model has a
  map-collision block, the physics mesh is written as a keyvalues2 DMX
  (`write_dmx_mesh`) whose polygons are the block's faces, welded per convex piece so
  each piece imports into Blender as a closed mesh (points no face uses become loose
  vertices), so the compiler rebuilds
  retail's faces; legacy models without the block get `$nomapcollision`. `decompile(path, out_dir)`
  returns the QC path. It also exposes decoders: `decode_procedural`,
  `decode_ui_panels`, `decode_ik_rules`, `ui_panel_hash`, `write_smd`.
- `tools/roundtrip.py`:
  - decompiles each retail model, recompiles it and compares a summary of fields
    (listed in VALIDATION.md);
  - checkpoints per model in `results.json`; `--jobs N` runs in parallel, `--resume`
    continues a run;
  - a full corpus run takes about 15 GB under `test-output/`, so delete the work
    folders of old runs.
- `tools/recheck_corpus.py`: recompiles cached exports from a previous run with a new
  executable. It checks the input stamps and tool hashes first.
- `tools/phy.py`: PHY / IVP compact-surface reader.
- `tools/ingame_test.py`: in-game harness (below).
- `tools/make_axis_test_model.py`, `tools/make_aim_test_model.py`: generate the
  procedural-bone test models used for the in-game checks.
- `tools/package.ps1`: builds `dist/r2-model-compiler-win64.zip`. Delete
  `dist/r2-model-compiler` first so stale files are not packed. Then extract the ZIP to
  a new folder, check the executable's SHA-256, and compile the three examples with it.
- `tests/blender/make_bst_exports.py`: exports a skinned test rig through Blender
  Source Tools as SMD and DMX 2/3/4/5/9.

## Retail corpus

Extract the models from your own Titanfall 2 install. They are not distributed here.
The corpus is `englishclient_mp_common.bsp.pak000_dir.vpk` (602 models). With
pg9182's `tf2-vpkunpack`:

```
tf2-vpkunpack --include '*.mdl' --exclude '*' <out_dir> <Titanfall2>\vpk\englishclient_mp_common.bsp.pak000_dir.vpk
```

Then:

```powershell
python tools\roundtrip.py --jobs 8 --compiler build\r2studiomdl.exe --out test-output\run <out_dir>\models
```

The single-player corpus is spread over 13 map VPKs
(`englishclient_sp_*.bsp.pak000_dir.vpk`, about 11,700 files). Extract each VPK into
its own folder. Then drop files byte-identical to `mp_common` or to each other (by
SHA-256), keeping a folder structure so equal basenames do not collide. That leaves
3,422 unique models, a run of about 70 GB. Single-player models exercise features
`mp_common` does not: bone followers, multi-piece joints (`$concaveperjoint`),
non-ascending LOD order, large map-collision pieces, and retail models built by older
tools without a map-collision block.

Build experiments in `build-dev` (`build-dev.bat`) while a corpus run is using
`build\r2studiomdl.exe`. Rebuilding the executable a run is using breaks that run.

Validation standard:
- Never loosen a comparison threshold to make a model pass.
- Explain each remaining difference, and list it in VALIDATION.md.
- Compare a new run's failing set against the previous run's: no model should newly
  differ.

## In-game testing (Northstar)

You need Titanfall 2 with Northstar installed. Set `TITANFALL2_DIR` or pass `--game`.

```powershell
python tools\ingame_test.py --sequence idle --report out\report.json out\model.mdl=models/r2c_test/model.mdl
```

How the harness works:
- It creates an isolated profile (`R2ModelTest`) containing the core Northstar mods and
  a generated mod.
- The mod precaches each model, spawns it as a `prop_dynamic` in front of the player
  and plays the sequence.
- The client takes a screenshot, which lands in `<game>\r2\screenshots\*.tga`; the game
  then quits.
- `--report` writes JSON results. The command exits nonzero on missing spawns, script
  errors or timeouts.

Options:
- `--pose-attachments a,b` logs those attachments' origins and angles.
- `--probe-attachment` with `--probe-times` samples one attachment during playback.
- `--motion seq1,seq2` plays each sequence with `Anim_PlayWithRefPoint` and samples the
  origin, yaw and probe attachment, to check root motion.
- `--physics` drops physics props; `--npc --ragdoll` exercises a death ragdoll.
- `--extra-file local=gamepath` mounts a non-model file, such as a weapon script.

Lessons:

- **Compare against retail side by side.** Mount retail and rebuilt copies under
  different paths and compare attachment poses. To bisect a misbehaving rebuild,
  compile QC subsets and compare an ORIGIN-type attachment's angles.
  - This is how the "prowler lying on its side" bug was found: decompiled delta
    animations had lost their delta flag. Byte diffs of bones and animations all
    matched.
- **Spawn layout.** Model i is offset by `(i - (n-1)/2) * spacing + lateral` along the
  player's right, so model 0 is on the left.
  - The viewmodel covers the right of the screen. `--lateral -110` with `--spacing 100`
    keeps two models visible.
  - The model's +X faces the player, and from the default spawn +Y recedes into depth.
  - To judge directions, crop the screenshot and extend lines in it.
- **Server-side queries miss client-only effects.** `GetAttachmentOrigin` ignores
  procedural bones, so check those visually with a control model built without them.
  `GetEntArrayByClass_Expensive` does not exist in the CLIENT VM.
- **Viewmodels and UI panels.** Mount a rebuilt `ptpov_*.mdl` over its retail game path
  (for example `models/weapons/rspn101/ptpov_rspn101.mdl`). The default loadout's R-201
  then uses it. Mount a weapon-script override with `--extra-file` when a panel name
  changes.
  - A viewmodel has no `idle` sequence, so its report "fails" with MISSING sequence;
    the screenshot is still valid.
- **Squirrel quirks.** `foreach` over an array literal does not compile; use a typed
  array variable. `ClientCommand()` exists only in the UI VM. In the CLIENT VM use
  `GetLocalClientPlayer().ClientCommand(...)`.
- **Authentication.** Launching straight into a map can fail with "Authentication
  Failed". The generated SERVER script sets `ns_auth_allow_insecure` before the local
  client connects. The `NO_GAMESERVER_RESPONSE` master-server errors in the log are
  harmless.

## Reverse engineering

- **References** (consult them; don't copy code from them):
  - Valve's 2005 SDK studiomdl and `bone_setup.cpp`: command semantics, IK and
    procedural-bone runtime;
  - r-ex/rsx (AGPL) and r-ex/rmdlconv (GPL): v53 struct notes;
  - Crowbar: decompile conventions and command line;
  - Blender Source Tools: SMD/DMX output;
  - pg9182/tf2vpk: VPK extraction.
- **Game binaries:** when a runtime rule is unknown, disassemble the game's DLLs.
  - The UI panel hash was found this way: Python + `capstone` + `pefile` on
    `bin\x64_retail\client.dll`.
  - Scan for the instructions that read the header fields in question (e.g.
    `[reg + 0x128]` / `[reg + 0x12c]`), follow the compared value back to the code that
    writes it, and port the function.
  - Check the port against every retail sample.
- **`dis.py`:** don't name a Python script `dis.py`; it shadows the standard-library
  module that capstone imports.
- **Run untrusted inputs isolated:** run Python on extracted game data with `-I`, and
  keep scripts outside the extraction folder.

## Shell pitfalls (Windows)

- Python edits passed through a bash heredoc can turn `\\n` and `\\` into real newlines
  or lone backslashes inside string literals. Prefer a file editor for code containing
  escapes, or build the strings with `chr(92)`.
- Run the compiler and `roundtrip.py` from PowerShell with backslash paths. Some
  forward-slash paths from bash failed to resolve.

## Keeping docs current

After a feature lands:
- update README.md (user-facing syntax);
- update VALIDATION.md (executable SHA-256, test count, corpus result, in-game
  evidence);
- update `docs/mdl-v53-format.md` for any new format fact;
- then rebuild the package if releasing.
