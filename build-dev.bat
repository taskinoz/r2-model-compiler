@echo off
rem Builds a second copy in build-dev (Release, x64) for side-by-side testing, with the newest Visual Studio that has the C++ tools.
setlocal
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (echo vswhere.exe not found: install Visual Studio 2022 with C++ tools & exit /b 1)
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSDIR=%%i"
if not defined VSDIR (echo No Visual Studio installation with the C++ x64 tools was found & exit /b 1)
call "%VSDIR%\Common7\Tools\VsDevCmd.bat" -arch=amd64 >nul
cmake -S "%~dp0." -B "%~dp0build-dev" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release || exit /b 1
cmake --build "%~dp0build-dev" || exit /b 1
