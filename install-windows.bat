@echo off

setlocal

set "CarNDEKFProjectPlatform=x86"
set "CarNDEKFProjectToolset=v141"
set "CarNDEKFProjectBuildType=Debug"

if NOT "%~1"=="" set "CarNDEKFProjectPlatform=%~1"
if NOT "%~2"=="" set "CarNDEKFProjectToolset=%~2"
if NOT "%~3"=="" set "CarNDEKFProjectBuildType=%~3" 

set "VcPkgDir=%USERPROFILE%\Software\vcpkg\vcpkg"
set "VcPkgTriplet=%CarNDEKFProjectPlatform%-windows-%CarNDEKFProjectToolset%"

if defined VCPKG_ROOT_DIR if /i not "%VCPKG_ROOT_DIR%"=="" (
    set "VcPkgDir=%VCPKG_ROOT_DIR%"
)
if defined VCPKG_DEFAULT_TRIPLET if /i not "%VCPKG_DEFAULT_TRIPLET%"=="" (
    set "VcpkgTriplet=%VCPKG_DEFAULT_TRIPLET%"
)
set "VcPkgPath=%VcPkgDir%\vcpkg.exe"

echo. & echo Bootstrapping dependencies for triplet: %VcPkgTriplet% & echo.

rem ==============================
rem Update and Install packages
rem ==============================
call "%VcPkgPath%" update
call "%VcPkgPath%" install uwebsockets --triplet %VcPkgTriplet%
call "%VcPkgPath%" install project --triplet %VcPkgTriplet%

rem ==============================
rem Configure CMake
rem ==============================

set "VcPkgTripletDir=%VcPkgDir%\installed\%VcPkgTriplet%"

set "CMAKE_PREFIX_PATH=%VcPkgTripletDir%;%CMAKE_PREFIX_PATH%"

echo. & echo Bootstrapping successful for triplet: %VcPkgTriplet% & echo CMAKE_PREFIX_PATH=%CMAKE_PREFIX_PATH% & echo.

endlocal