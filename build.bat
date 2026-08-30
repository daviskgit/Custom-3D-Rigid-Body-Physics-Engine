@echo off
REM Build the physics engine directly with the MSYS2 UCRT64 g++ toolchain,
REM no MSYS shell / make required. Run from plain cmd or PowerShell.
setlocal
set GCC_BIN=C:\msys64\ucrt64\bin
set PATH=%GCC_BIN%;%PATH%

if not exist bin mkdir bin

echo Compiling...
"%GCC_BIN%\g++.exe" -std=c++17 -O2 -DNDEBUG -Wall -Wextra -Wno-unused-parameter -Isrc ^
    src\collision.cpp src\broadphase.cpp src\dynamics.cpp ^
    src\renderer.cpp src\main.cpp ^
    -o bin\physics_engine.exe ^
    -lopengl32 -lgdi32 -luser32 -lkernel32 -mwindows -static -static-libgcc -static-libstdc++

if errorlevel 1 (
    echo Build FAILED.
    exit /b 1
)
echo Build OK: bin\physics_engine.exe
endlocal
