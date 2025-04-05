@echo off
setlocal enableDelayedExpansion

rem ==========================================================
rem Helper Subroutines
rem ==========================================================

:Usage
echo.
echo Usage: %~n0 (release^|debug^|signed-release) (x64^|arm64)
echo.
exit /b 1

:CheckError
rem Checks if the previous command failed and prints an error message if so.
rem %1 is the optional error message.
if !ERRORLEVEL! NEQ 0 (
    if not "%~1"=="" (
        echo Error: %~1
    )
    goto Error
)
exit /b 0

rem ==========================================================
rem Main Script
rem ==========================================================

rem Check for command-line arguments
if "%~1"=="" (
    echo Missing build configuration argument.
    call :Usage
)
if "%~2"=="" (
    echo Missing build architecture argument.
    call :Usage
)

set "BUILD_CONFIG=%~1"
set "BUILD_ARCH=%~2"

rem Validate BUILD_CONFIG argument
if /I "%BUILD_CONFIG%"=="debug" (
    set "BUILD_CONFIG=debug"
    set "WIX_MUMS=10"
) else if /I "%BUILD_CONFIG%"=="release" (
    set "BUILD_CONFIG=release"
    set "WIX_MUMS=10"
) else if /I "%BUILD_CONFIG%"=="signed-release" (
    set "BUILD_CONFIG=release"
    set "SIGN=1"
    set "MUST_DEPLOY_SYMBOLS=1"
    rem Fail if there are unstaged changes
    git diff-index --quiet HEAD --
    call :CheckError "Signed release builds must not have unstaged changes!"
) else (
    echo Invalid build configuration.
    call :Usage
)

rem Validate BUILD_ARCH argument
if /I "%BUILD_ARCH%"=="arm64" (
    set "ARCH=arm64"
) else if /I "%BUILD_ARCH%"=="x64" (
    set "ARCH=x64"
) else (
    echo Invalid build architecture.
    call :Usage
)

rem ==========================================================
rem Prerequisite Checks
rem ==========================================================
rem Check for QMake (qmake.bat or qmake.exe)
where "qmake.bat" >nul 2>&1
if !ERRORLEVEL! EQU 0 (
    set "QMAKE_CMD=call qmake.bat"
) else (
    where "qmake.exe" >nul 2>&1
    if !ERRORLEVEL! EQU 0 (
        set "QMAKE_CMD=qmake.exe"
    ) else (
        echo Unable to find QMake. Did you add Qt bins to your PATH?
        goto Error
    )
)

rem Check for vswhere.exe in the scripts directory
if not exist "%~dp0scripts\vswhere.exe" (
    echo vswhere.exe not found in "%~dp0scripts\".
    goto Error
)

rem Check for jom.exe in the scripts directory
if not exist "%~dp0scripts\jom.exe" (
    echo jom.exe not found in "%~dp0scripts\".
    goto Error
)

rem Check for msbuild (assumed in PATH)
where msbuild >nul 2>&1
if !ERRORLEVEL! NEQ 0 (
    echo msbuild not found in PATH.
    goto Error
)

rem Check for 7z (assumed in PATH)
where 7z >nul 2>&1
if !ERRORLEVEL! NEQ 0 (
    echo 7z not found in PATH.
    goto Error
)

rem ==========================================================
rem Determine Qt Path and Deployment Tool Command
rem ==========================================================
for /F "usebackq tokens=*" %%i in (`where qmake`) do set "QT_PATH=%%i"
rem Remove the qmake filename to get the Qt bin directory
set "QT_PATH=%QT_PATH:\qmake.exe=%"
set "QT_PATH=%QT_PATH:\qmake.bat=%"
set "QT_PATH=%QT_PATH:\qmake.cmd=%"
echo QT_PATH="%QT_PATH%"
echo Using build architecture: %ARCH%

if /I "%ARCH%"=="arm64" (
    rem For arm64, assume QT_PATH contains _arm64 and compute the host bin path
    set "HOSTBIN_PATH=%QT_PATH:_arm64=_64%"
    echo HOSTBIN_PATH="%HOSTBIN_PATH%"
    if exist "%QT_PATH%\windeployqt.exe" (
        echo Using windeployqt.exe from QT_PATH.
        set "WINDEPLOYQT_CMD=windeployqt.exe"
    ) else (
        echo Using windeployqt.exe from HOSTBIN_PATH.
        set "WINDEPLOYQT_CMD=%HOSTBIN_PATH%\windeployqt.exe --qtpaths \"%QT_PATH%\qtpaths.bat\""
    )
) else (
    rem For x64 simply use windeployqt.exe
    set "WINDEPLOYQT_CMD=windeployqt.exe"
)

echo Detected target architecture: %ARCH%

set "SIGNTOOL_PARAMS=sign /tr http://timestamp.digicert.com /td sha256 /fd sha256 /sha1 8b9d0d682ad9459e54f05a79694bc10f9876e297 /v"

rem ==========================================================
rem Setup Build Environment Variables
rem ==========================================================
set "BUILD_ROOT=%cd%\build"
set "SOURCE_ROOT=%cd%"
set "BUILD_FOLDER=%BUILD_ROOT%\build-%ARCH%-%BUILD_CONFIG%"
set "DEPLOY_FOLDER=%BUILD_ROOT%\deploy-%ARCH%-%BUILD_CONFIG%"
set "INSTALLER_FOLDER=%BUILD_ROOT%\installer-%ARCH%-%BUILD_CONFIG%"
set "SYMBOLS_FOLDER=%BUILD_ROOT%\symbols-%ARCH%-%BUILD_CONFIG%"
set /p "VERSION=<%SOURCE_ROOT%\app\version.txt"

if /I "%ARCH%"=="x64" (
    set "VC_ARCH=AMD64"
) else (
    set "VC_ARCH=%ARCH%"
)

if /I "%VC_ARCH%" NEQ "%PROCESSOR_ARCHITECTURE%" (
    set "VC_ARCH=%PROCESSOR_ARCHITECTURE%_%VC_ARCH%"
)

rem Find Visual Studio and run vcvarsall.bat
set "VSWHERE=%SOURCE_ROOT%\scripts\vswhere.exe"
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -property installationPath`) do (
    call "%%i\VC\Auxiliary\Build\vcvarsall.bat" %VC_ARCH%
)
call :CheckError "Failed to run vcvarsall.bat"

rem Find VC redistributable DLLs
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -find VC\Redist\MSVC\*\%ARCH%\Microsoft.VC*.CRT`) do set "VC_REDIST_DLL_PATH=%%i"

rem ==========================================================
rem Clean Output Directories
rem ==========================================================
echo Cleaning output directories...
rd /s /q "%DEPLOY_FOLDER%" 2>nul
rd /s /q "%BUILD_FOLDER%" 2>nul
rd /s /q "%INSTALLER_FOLDER%" 2>nul
rd /s /q "%SYMBOLS_FOLDER%" 2>nul
mkdir "%BUILD_ROOT%"
mkdir "%DEPLOY_FOLDER%"
mkdir "%BUILD_FOLDER%"
mkdir "%INSTALLER_FOLDER%"
mkdir "%SYMBOLS_FOLDER%"

rem ==========================================================
rem Build Steps
rem ==========================================================
echo Configuring the project...
pushd "%BUILD_FOLDER%"
%QMAKE_CMD% "%SOURCE_ROOT%\moonlight-qt.pro"
call :CheckError "qmake configuration failed."
popd

echo Compiling Moonlight in %BUILD_CONFIG% configuration...
pushd "%BUILD_FOLDER%"
"%SOURCE_ROOT%\scripts\jom.exe" %BUILD_CONFIG%
call :CheckError "Compilation failed."
popd

echo Saving PDBs...
for /r "%BUILD_FOLDER%" %%f in (*.pdb) do (
    copy "%%f" "%SYMBOLS_FOLDER%" >nul
    call :CheckError "Copying PDB file failed."
)
copy "%SOURCE_ROOT%\libs\windows\lib\%ARCH%\*.pdb" "%SYMBOLS_FOLDER%" >nul
call :CheckError "Copying additional PDB files failed."
7z a "%SYMBOLS_FOLDER%\MoonlightDebuggingSymbols-%ARCH%-%VERSION%.zip" "%SYMBOLS_FOLDER%\*.pdb" >nul
call :CheckError "Zipping PDB files failed."

if not "%ML_SYMBOL_STORE%"=="" (
    echo Publishing PDBs to symbol store: %ML_SYMBOL_STORE%
    symstore add /f "%SYMBOLS_FOLDER%\*.pdb" /s "%ML_SYMBOL_STORE%" /t Moonlight
    call :CheckError "Publishing PDBs failed."
) else (
    if "%MUST_DEPLOY_SYMBOLS%"=="1" (
        echo A symbol server must be specified in ML_SYMBOL_STORE for signed release builds.
        exit /b 1
    )
)

if not "%ML_SYMBOL_ARCHIVE%"=="" (
    echo Copying PDB ZIP to symbol archive: %ML_SYMBOL_ARCHIVE%
    copy "%SYMBOLS_FOLDER%\MoonlightDebuggingSymbols-%ARCH%-%VERSION%.zip" "%ML_SYMBOL_ARCHIVE%" >nul
    call :CheckError "Copying symbol archive failed."
) else (
    if "%MUST_DEPLOY_SYMBOLS%"=="1" (
        echo A symbol archive directory must be specified in ML_SYMBOL_ARCHIVE for signed release builds.
        exit /b 1
    )
)

echo Copying DLL dependencies...
copy "%SOURCE_ROOT%\libs\windows\lib\%ARCH%\*.dll" "%DEPLOY_FOLDER%" >nul
call :CheckError "Copying DLL dependencies failed."

echo Copying AntiHooking.dll...
copy "%BUILD_FOLDER%\AntiHooking\%BUILD_CONFIG%\AntiHooking.dll" "%DEPLOY_FOLDER%" >nul
call :CheckError "Copying AntiHooking.dll failed."

echo Copying GC mapping list...
copy "%SOURCE_ROOT%\app\SDL_GameControllerDB\gamecontrollerdb.txt" "%DEPLOY_FOLDER%" >nul
call :CheckError "Copying gamecontrollerdb.txt failed."

rem Determine Qt version by checking QT_PATH for Qt 5 pattern
echo %QT_PATH% | findstr /i "\\5\." >nul
if !ERRORLEVEL! EQU 0 (
    echo Copying qt.conf for Qt 5...
    copy "%SOURCE_ROOT%\app\qt_qt5.conf" "%DEPLOY_FOLDER%\qt.conf" >nul
    call :CheckError "Copying qt.conf failed."
    set "WINDEPLOYQT_ARGS=--no-qmltooling --no-virtualkeyboard"
) else (
    rem Assume Qt 6.5+ here
    set "WINDEPLOYQT_ARGS=--no-system-d3d-compiler --no-system-dxc-compiler --skip-plugin-types qmltooling,generic --no-ffmpeg"
    set "WINDEPLOYQT_ARGS=!WINDEPLOYQT_ARGS! --no-quickcontrols2fusion --no-quickcontrols2imagine --no-quickcontrols2universal"
    set "WINDEPLOYQT_ARGS=!WINDEPLOYQT_ARGS! --no-quickcontrols2fusionstyleimpl --no-quickcontrols2imaginestyleimpl --no-quickcontrols2universalstyleimpl --no-quickcontrols2windowsstyleimpl"
)

echo Deploying Qt dependencies...
%WINDEPLOYQT_CMD% --dir "%DEPLOY_FOLDER%" --%BUILD_CONFIG% --qmldir "%SOURCE_ROOT%\app\gui" --no-opengl-sw --no-compiler-runtime --no-sql %WINDEPLOYQT_ARGS% "%BUILD_FOLDER%\app\%BUILD_CONFIG%\Moonlight.exe"
call :CheckError "Deploying Qt dependencies failed."

echo Deleting unused styles...
rd /s /q "%DEPLOY_FOLDER%\QtQuick\Controls.2\Fusion" 2>nul
rd /s /q "%DEPLOY_FOLDER%\QtQuick\Controls.2\Imagine" 2>nul
rd /s /q "%DEPLOY_FOLDER%\QtQuick\Controls.2\Universal" 2>nul
rd /s /q "%DEPLOY_FOLDER%\qml\QtQuick\Controls\Fusion" 2>nul
rd /s /q "%DEPLOY_FOLDER%\qml\QtQuick\Controls\Imagine" 2>nul
rd /s /q "%DEPLOY_FOLDER%\qml\QtQuick\Controls\Universal" 2>nul
rd /s /q "%DEPLOY_FOLDER%\qml\QtQuick\Controls\Windows" 2>nul
rd /s /q "%DEPLOY_FOLDER%\qml\QtQuick\NativeStyle" 2>nul

if "%SIGN%"=="1" (
    echo Signing deployed binaries...
    set "FILES_TO_SIGN=%BUILD_FOLDER%\app\%BUILD_CONFIG%\Moonlight.exe"
    for /r "%DEPLOY_FOLDER%" %%f in (*.dll *.exe) do (
        set "FILES_TO_SIGN=!FILES_TO_SIGN! %%f"
    )
    signtool %SIGNTOOL_PARAMS% !FILES_TO_SIGN!
    call :CheckError "Signing binaries failed."
)

if not "%ML_SYMBOL_STORE%"=="" (
    echo Publishing binaries to symbol store: %ML_SYMBOL_STORE%
    symstore add /r /f "%DEPLOY_FOLDER%\*.*" /s "%ML_SYMBOL_STORE%" /t Moonlight
    call :CheckError "Publishing binaries failed."
    symstore add /r /f "%BUILD_FOLDER%\app\%BUILD_CONFIG%\Moonlight.exe" /s "%ML_SYMBOL_STORE%" /t Moonlight
    call :CheckError "Publishing Moonlight.exe to symbol store failed."
)

echo Building MSI...
msbuild -Restore "%SOURCE_ROOT%\wix\Moonlight\Moonlight.wixproj" /p:Configuration=%BUILD_CONFIG% /p:Platform=%ARCH% /p:MSBuildProjectExtensionsPath="%BUILD_FOLDER%"\
call :CheckError "MSI build failed."

echo Copying application binary to deployment directory...
copy "%BUILD_FOLDER%\app\%BUILD_CONFIG%\Moonlight.exe" "%DEPLOY_FOLDER%" >nul
call :CheckError "Copying Moonlight.exe failed."

echo Building portable package...
copy "%VC_REDIST_DLL_PATH%\*.dll" "%DEPLOY_FOLDER%" >nul
call :CheckError "Copying VC redistributable DLLs failed."
echo. > "%DEPLOY_FOLDER%\portable.dat"
call :CheckError "Creating portable.dat failed."
7z a "%INSTALLER_FOLDER%\MoonlightPortable-%ARCH%-%VERSION%.zip" "%DEPLOY_FOLDER%\*" >nul
call :CheckError "Building portable package failed."

echo.
echo Build successful for Moonlight v%VERSION% %ARCH% binaries!
exit /b 0

:Error
echo.
echo Build failed!
exit /b !ERRORLEVEL!
