$modelname "r2c_examples/animated_prop.mdl"
$cdmaterials ""
$body "body" "body.smd"
$sequence "idle" "body.smd"
$sequence "bend" "bend.smd" fps 30 loop
$attachment "probe" "head" 0 0 0
$collisionjoints "physics.smd"
{
    $mass 20
    $jointconstrain "head" x limit -25 25 0
    $jointconstrain "head" y limit -25 25 0
    $jointconstrain "head" z limit -25 25 0
}
