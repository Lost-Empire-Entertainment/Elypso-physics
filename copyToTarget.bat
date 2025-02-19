@echo off

echo.
echo ====================================================
echo Preinstall cleanup
echo ====================================================
echo.

set "origin=%~dp0"

set "engine=%origin%\..\Elypso-engine\Engine"

:: Build and install origin folders
set "origin_build_release_folder=%origin%\build-release"
set "origin_build_debug_folder=%origin%\build-debug"
set "origin_release_folder=%origin%\install-release"
set "origin_debug_folder=%origin%\install-debug"

:: Resolve paths before checking if they are valid
for /f "delims=" %%i in ("%origin_build_release_folder%") do set "origin_build_release_folder=%%~fi"
for /f "delims=" %%i in ("%origin_build_debug_folder%") do set "origin_build_debug_folder=%%~fi"
for /f "delims=" %%i in ("%origin_release_folder%") do set "origin_release_folder=%%~fi"
for /f "delims=" %%i in ("%origin_debug_folder%") do set "origin_debug_folder=%%~fi"

:: Remove build and install origin folders if they exist
if exist "%origin_build_release_folder%" (
	rmdir /s /q "%origin_build_release_folder%"
	echo Deleted build release folder.
)
if exist "%origin_build_debug_folder%" (
	rmdir /s /q "%origin_build_debug_folder%"
	echo Deleted build debug release folder.
)
if exist "%origin_release_folder%" (
	rmdir /s /q "%origin_release_folder%"
	echo Deleted install release folder.
)
if exist "%origin_debug_folder%" (
	rmdir /s /q "%origin_debug_folder%"
	echo Deleted install debug folder.
)

echo.
echo ====================================================
echo Installing dlls and libs
echo ====================================================
echo.

:: Run release and debug batch files
cmd /c "build_windows_release.bat"
cmd /c "build_windows_debug.bat"

echo.
echo ====================================================
echo Initializing copy batch file
echo ====================================================
echo.

:: Physics library target folders
set "physics_folder=%engine%\..\_external_shared\elypsophysics\"
set "physics_release=%physics_folder%\release"
set "physics_debug=%physics_folder%\debug"

:: Origin dll and lib paths
set "origin_release_dll=%origin_release_folder%\bin\ElypsoPhysics.dll"
set "origin_release_lib=%origin_release_folder%\lib\ElypsoPhysics.lib"
set "origin_debug_dll=%origin_debug_folder%\bin\ElypsoPhysicsD.dll"
set "origin_debug_lib=%origin_debug_folder%\lib\ElypsoPhysicsD.lib"

:: Physics dll and lib paths
set "physics_release_dll=%physics_release%\ElypsoPhysics.dll"
set "physics_release_lib=%physics_release%\ElypsoPhysics.lib"
set "physics_debug_dll=%physics_debug%\ElypsoPhysicsD.dll"
set "physics_debug_lib=%physics_debug%\ElypsoPhysicsD.lib"

:: Engine dll paths
set "engine_dlls_folder=%engine%\files\external dlls"
set "engine_release_dll=%engine_dlls_folder%\release\ElypsoPhysics.dll"
set "engine_debug_dll=%engine_dlls_folder%\debug\ElypsoPhysicsD.dll"

:: Game dll paths
set "game_dlls_folder=%engine%\..\Game\files\external dlls"
set "game_release_dll=%game_dlls_folder%\release\ElypsoPhysics.dll"
set "game_debug_dll=%game_dlls_folder%\debug\ElypsoPhysicsD.dll"

:: Header origin paths
set "header_1_origin=%origin%\include\collider.hpp"
set "header_2_origin=%origin%\include\collisiondetection.hpp"
set "header_3_origin=%origin%\include\gameobjecthandle.hpp"
set "header_4_origin=%origin%\include\physicsworld.hpp"
set "header_5_origin=%origin%\include\rigidbody.hpp"

:: Header target paths
set "header_1_target=%physics_folder%\collider.hpp"
set "header_2_target=%physics_folder%\collisiondetection.hpp"
set "header_3_target=%physics_folder%\gameobjecthandle.hpp"
set "header_4_target=%physics_folder%\physicsworld.hpp"
set "header_5_target=%physics_folder%\rigidbody.hpp"

:: Resolve paths before checking if they are valid
for /f "delims=" %%i in ("%physics_folder%") do set "physics_folder=%%~fi"
for /f "delims=" %%i in ("%physics_release%") do set "physics_release=%%~fi"
for /f "delims=" %%i in ("%physics_debug%") do set "physics_debug=%%~fi"

for /f "delims=" %%i in ("%origin_release_dll%") do set "origin_release_dll=%%~fi"
for /f "delims=" %%i in ("%origin_debug_dll%") do set "origin_debug_dll=%%~fi"

for /f "delims=" %%i in ("%engine_release_dll%") do set "engine_release_dll=%%~fi"
for /f "delims=" %%i in ("%engine_debug_dll%") do set "engine_debug_dll=%%~fi"

for /f "delims=" %%i in ("%game_release_dll%") do set "game_release_dll=%%~fi"
for /f "delims=" %%i in ("%game_debug_dll%") do set "game_debug_dll=%%~fi"

for /f "delims=" %%i in ("%header_1_origin%") do set "header_1_origin=%%~fi"
for /f "delims=" %%i in ("%header_2_origin%") do set "header_2_origin=%%~fi"
for /f "delims=" %%i in ("%header_3_origin%") do set "header_3_origin=%%~fi"
for /f "delims=" %%i in ("%header_4_origin%") do set "header_4_origin=%%~fi"
for /f "delims=" %%i in ("%header_5_origin%") do set "header_5_origin=%%~fi"

for /f "delims=" %%i in ("%header_1_target%") do set "header_1_target=%%~fi"
for /f "delims=" %%i in ("%header_2_target%") do set "header_2_target=%%~fi"
for /f "delims=" %%i in ("%header_3_target%") do set "header_3_target=%%~fi"
for /f "delims=" %%i in ("%header_4_target%") do set "header_4_target=%%~fi"
for /f "delims=" %%i in ("%header_5_target%") do set "header_5_target=%%~fi"

:: Check if assigned paths are valid
if not exist "%physics_folder%" (
	echo Error: Invalid physics folder path '%physics_folder%'!
	pause
	exit /b 1
) else (
	echo Successfully found physics folder path!
)

if not exist "%origin_release_dll%" (
	echo Error: Invalid release dll path '%origin_release_dll%'!
	pause
	exit /b 1
) else (
	echo Successfully found release dll path!
)
if not exist "%origin_debug_dll%" (
	echo Error: Invalid debug dll path '%origin_debug_dll%'!
	pause
	exit /b 1
) else (
	echo Successfully found debug dll path!
)

if not exist "%header_1_origin%" (
	echo Error: Invalid header 1 path '%header_1_origin%'!
	pause
	exit /b 1
) else (
	echo Successfully found header 1 path!
)
if not exist "%header_2_origin%" (
	echo Error: Invalid header 2 path '%header_2_origin%'!
	pause
	exit /b 1
) else (
	echo Successfully found header 2 path!
)
if not exist "%header_3_origin%" (
	echo Error: Invalid header 3 path '%header_3_origin%'!
	pause
	exit /b 1
) else (
	echo Successfully found header 3 path!
)
if not exist "%header_4_origin%" (
	echo Error: Invalid header 4 path '%header_4_origin%'!
	pause
	exit /b 1
) else (
	echo Successfully found header 4 path!
)
if not exist "%header_5_origin%" (
	echo Error: Invalid header 5 path '%header_5_origin%'!
	pause
	exit /b 1
) else (
	echo Successfully found header 5 path!
)

echo.
echo ====================================================
echo Initialize succeeded!
echo ====================================================
echo.

:: Create physics release and debug folders if they dont exist
if not exist "%physics_release%" (
	mkdir "%physics_release%"
	echo Created physics release folder.
)
if not exist "%physics_debug%" (
	mkdir "%physics_debug%"
	echo Created physics debug folder.
)

:: Copy dlls and libs to physics folders
copy "%origin_release_dll%" "%physics_release_dll%" /Y >nul
copy "%origin_release_lib%" "%physics_release_lib%" /Y >nul
copy "%origin_debug_dll%" "%physics_debug_dll%" /Y >nul
copy "%origin_debug_lib%" "%physics_debug_lib%" /Y >nul
echo Copied origin dlls and libs to physics release and debug folders.

:: Copy engine and game dlls
copy "%origin_release_dll%" "%engine_release_dll%" /Y >nul
copy "%origin_debug_dll%" "%engine_debug_dll%" /Y >nul
copy "%origin_release_dll%" "%game_release_dll%" /Y >nul
copy "%origin_debug_dll%" "%game_debug_dll%" /Y >nul
echo Copied origin dlls and libs to engine and game release and debug folders.

:: Copy headers to physics folder
copy "%header_1_origin%" "%header_1_target%" /Y >nul
copy "%header_2_origin%" "%header_2_target%" /Y >nul
copy "%header_3_origin%" "%header_3_target%" /Y >nul
copy "%header_4_origin%" "%header_4_target%" /Y >nul
copy "%header_5_origin%" "%header_5_target%" /Y >nul
echo Copied headers to physics folder.

echo.
echo ====================================================
echo Finished copying files!
echo ====================================================
echo.

pause
exit /b 0