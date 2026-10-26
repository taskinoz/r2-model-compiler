param([string]$OutputDirectory = (Join-Path $PSScriptRoot '..\dist'))
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
$compiler = Join-Path $projectRoot 'build\r2studiomdl.exe'
if (!(Test-Path -LiteralPath $compiler)) { throw 'Run build.bat before packaging.' }
$package = Join-Path $OutputDirectory 'r2-model-compiler'
New-Item -ItemType Directory -Path $package -Force | Out-Null
Copy-Item -LiteralPath $compiler -Destination $package
Copy-Item -LiteralPath (Join-Path $projectRoot 'README.md') -Destination $package
Copy-Item -LiteralPath (Join-Path $projectRoot 'LICENSE') -Destination $package
if (Test-Path -LiteralPath (Join-Path $projectRoot 'VALIDATION.md')) {
    Copy-Item -LiteralPath (Join-Path $projectRoot 'VALIDATION.md') -Destination $package
}
Copy-Item -LiteralPath (Join-Path $projectRoot 'examples') -Destination $package -Recurse -Force
Copy-Item -LiteralPath (Join-Path $projectRoot 'docs') -Destination $package -Recurse -Force
$toolDestination = Join-Path $package 'tools'
New-Item -ItemType Directory -Path $toolDestination -Force | Out-Null
foreach ($tool in @('decompile53.py', 'mdl53.py', 'phy.py', 'roundtrip.py', 'structdiff.py', 'recheck_corpus.py', 'validate53.py', 'validate_corpus.py')) {
    Copy-Item -LiteralPath (Join-Path $projectRoot "tools\$tool") -Destination $toolDestination
}
$archive = Join-Path $OutputDirectory 'r2-model-compiler-win64.zip'
Compress-Archive -Path (Join-Path $package '*') -DestinationPath $archive -Force
Get-FileHash -LiteralPath $archive -Algorithm SHA256
