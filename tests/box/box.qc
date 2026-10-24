$modelname "r2c_test/box.mdl"
$cdmaterials ""
$body "body" "box.smd"
$staticprop
$surfaceprop "metal"
$sequence "idle" "box.smd"
$collisionmodel "box.smd" { $concave $mass 50 }
