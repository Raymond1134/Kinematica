@echo off
set CMAKE=%USERPROFILE%\vcpkg\downloads\tools\cmake-3.31.10-windows\cmake-3.31.10-windows-x86_64\bin\cmake.exe
"%CMAKE%" -S . -B build -DCMAKE_TOOLCHAIN_FILE=%USERPROFILE%\vcpkg\scripts\buildsystems\vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
"%CMAKE%" --build build --config Release
build\Release\app.exe