$modelname "r2c_examples/physics_box.mdl"
$cdmaterials ""
$body "body" "box.smd"
$surfaceprop "metal"
$sequence "idle" "box.smd"
$collisionmodel "box.smd" { $mass 50 }
$keyvalues
{
    prop_data
    {
        "base" "Metal.Small"
    }
}
