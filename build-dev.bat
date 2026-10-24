@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=amd64 >nul
cmake -S "%~dp0." -B "%~dp0build-dev" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release >nul || exit /b 1
cmake --build "%~dp0build-dev" || exit /b 1

