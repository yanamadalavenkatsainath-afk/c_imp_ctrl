@echo off
setlocal EnableExtensions
if /I "%~1"=="--inner" goto :build_inner

if not exist "build_results" mkdir "build_results"
for /f %%I in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') do set BUILD_TS=%%I
set "BUILD_DIR=build_results\run_%BUILD_TS%"
mkdir "%BUILD_DIR%"
set "BUILD_LOG=%BUILD_DIR%\build.log"

echo === Saving build output to %BUILD_LOG% ===
call "%~f0" --inner > "%BUILD_LOG%" 2>&1
set "BUILD_RC=%errorlevel%"

type "%BUILD_LOG%"
> "build_results\latest_path.txt" echo %CD%\%BUILD_DIR%
echo.
echo === Build log saved: %BUILD_LOG% ===
exit /b %BUILD_RC%

:build_inner
REM ================================================================
REM build.bat -- Compile GNC C library + run SIL verification
REM v4: added closed_loop_sil.py to Python test sequence
REM ================================================================

echo === Ensuring Python package structure in flight sim ===
set FLIGHT_SIM=C:\Users\Venkat\OneDrive\Desktop\appex\flight sim
if not exist "%FLIGHT_SIM%\estimation\__init__.py" (
    type nul > "%FLIGHT_SIM%\estimation\__init__.py"
    echo   Created: estimation\__init__.py
) else ( echo   OK: estimation\__init__.py exists )
if not exist "%FLIGHT_SIM%\control\__init__.py" (
    type nul > "%FLIGHT_SIM%\control\__init__.py"
    echo   Created: control\__init__.py
)
if not exist "sim_python\__init__.py" (
    type nul > "sim_python\__init__.py"
    echo   Created: sim_python\__init__.py
)

echo.
echo === Generating pyrightconfig.json ===
python sim_python\gen_pyrightconfig.py
if %errorlevel% neq 0 ( echo pyrightconfig generation FAILED & exit /b 1 )

echo.
echo === Compiling GNC library (full stack) ===
gcc -fPIC -shared -O2 -DMEKF_NO_CMSIS ^
    -Isrc_c ^
    -o gnc_lib.dll ^
    src_c/th_ekf.c ^
    src_c/mekf.c ^
    src_c/rpod_ctrl.c ^
    src_c/terminal_filter.c ^
    src_c/port_tracker.c ^
    src_c/quest.c ^
    src_c/adcs.c ^
    src_c/mode_manager.c ^
    src_c/lambert_solver.c ^
    src_c/chief_pose_estimator.c ^
    src_c/flight_loop.c ^
    -lm
if %errorlevel% neq 0 ( echo BUILD FAILED & exit /b 1 )
echo Built: gnc_lib.dll

echo.
echo === Compiling flight_loop standalone smoke test ===
gcc -O2 -DMEKF_NO_CMSIS -DFLIGHT_LOOP_STANDALONE -Isrc_c ^
    -o flight_loop_test.exe ^
    src_c/flight_loop.c ^
    src_c/th_ekf.c ^
    src_c/mekf.c ^
    src_c/rpod_ctrl.c ^
    src_c/terminal_filter.c ^
    src_c/port_tracker.c ^
    src_c/quest.c ^
    src_c/adcs.c ^
    src_c/mode_manager.c ^
    src_c/lambert_solver.c ^
    src_c/chief_pose_estimator.c ^
    -lm
if %errorlevel% neq 0 ( echo FLIGHT LOOP BUILD FAILED & exit /b 1 )

echo.
echo === Compiling unit tests ===
gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_thekf.exe tests/test_thekf.c src_c/th_ekf.c -lm
if %errorlevel% neq 0 ( echo TH-EKF TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_mekf.exe tests/test_mekf.c src_c/mekf.c -lm
if %errorlevel% neq 0 ( echo MEKF TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_rpod.exe tests/test_rpod.c src_c/rpod_ctrl.c -lm
if %errorlevel% neq 0 ( echo RPOD TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_quest.exe tests/test_quest.c src_c/quest.c -lm
if %errorlevel% neq 0 ( echo QUEST TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_adcs.exe tests/test_adcs.c src_c/adcs.c -lm
if %errorlevel% neq 0 ( echo ADCS TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_mode_manager.exe tests/test_mode_manager.c src_c/mode_manager.c -lm
if %errorlevel% neq 0 ( echo MODE MANAGER TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_lambert.exe tests/test_lambert.c src_c/lambert_solver.c -lm
if %errorlevel% neq 0 ( echo LAMBERT TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_chief_pose.exe tests/test_chief_pose.c src_c/chief_pose_estimator.c -lm
if %errorlevel% neq 0 ( echo CHIEF POSE TEST BUILD FAILED & exit /b 1 )

echo.
echo === Running flight_loop smoke test ===
flight_loop_test.exe
if %errorlevel% neq 0 ( echo FLIGHT LOOP SMOKE TEST FAILED & exit /b 1 )

echo.
echo === Running C unit tests ===
test_thekf.exe
if %errorlevel% neq 0 ( echo TH-EKF TESTS FAILED & exit /b 1 )
test_mekf.exe
if %errorlevel% neq 0 ( echo MEKF TESTS FAILED & exit /b 1 )
test_rpod.exe
if %errorlevel% neq 0 ( echo RPOD TESTS FAILED & exit /b 1 )
test_quest.exe
if %errorlevel% neq 0 ( echo QUEST TESTS FAILED & exit /b 1 )
test_adcs.exe
if %errorlevel% neq 0 ( echo ADCS TESTS FAILED & exit /b 1 )
test_mode_manager.exe
if %errorlevel% neq 0 ( echo MODE MANAGER TESTS FAILED & exit /b 1 )
test_lambert.exe
if %errorlevel% neq 0 ( echo LAMBERT TESTS FAILED & exit /b 1 )
test_chief_pose.exe
if %errorlevel% neq 0 ( echo CHIEF POSE TESTS FAILED & exit /b 1 )

echo.
echo === Running Python SIL comparison ===
python sim_python/verify_sil.py
if %errorlevel% neq 0 ( echo SIL VERIFICATION FAILED & exit /b 1 )

echo.
echo === Running Real-Time SIL verification ===
python sim_python/verify_realtime_sil.py
if %errorlevel% neq 0 ( echo REALTIME SIL FAILED & exit /b 1 )

echo.
echo === Running Closed-Loop SIL verification ===
python sim_python/closed_loop_sil.py
if %errorlevel% neq 0 ( echo CLOSED LOOP SIL FAILED & exit /b 1 )

echo.
echo === ALL PASS ===


