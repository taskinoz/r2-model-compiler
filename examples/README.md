# Example models

These examples use a material already installed with Titanfall 2. No game assets
are included here. Run commands from the compiler package folder.

```powershell
# A fixed map prop, with collision:
.\r2studiomdl.exe -game "C:\Games\Titanfall2" examples\static_prop\box.qc

# A moving physics prop; includes the prop_data block required by the game:
.\r2studiomdl.exe -game "C:\Games\Titanfall2" examples\physics_prop\physics_box.qc

# A Blender-exported skinned cylinder with a bend sequence, attachment and
# separate physics shapes for two bones:
.\r2studiomdl.exe -game "C:\Games\Titanfall2" examples\animated_prop\animated_prop.qc
```

Output is written under the game's `models/r2c_examples/` folder. To compile into
a Northstar mod instead, supply that mod's `mod` directory as `-game`, so its
`models` directory receives the generated MDL.

Use a static entity for the static example, a physics entity for the physics
example, and a dynamic entity playing the `bend` sequence for the animated example.
An in-game entity still needs to reference the model; compiling does not spawn it.

You can also use `-o path\model.mdl` to write a standalone file without mounting
it into a game or mod.
