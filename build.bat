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
    src_c/capture_gate.c ^
    src_c/rpod_ctrl.c ^
    src_c/terminal_filter.c ^
    src_c/port_tracker.c ^
    src_c/spin_sync_controller.c ^
    src_c/nmc_guidance.c ^
    src_c/keepout_planner.c ^
    src_c/quest.c ^
    src_c/adcs.c ^
    src_c/mode_manager.c ^
    src_c/lambert_solver.c ^
    src_c/rpod_sequencer.c ^
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
    src_c/capture_gate.c ^
    src_c/rpod_ctrl.c ^
    src_c/terminal_filter.c ^
    src_c/port_tracker.c ^
    src_c/spin_sync_controller.c ^
    src_c/nmc_guidance.c ^
    src_c/keepout_planner.c ^
    src_c/quest.c ^
    src_c/adcs.c ^
    src_c/mode_manager.c ^
    src_c/lambert_solver.c ^
    src_c/rpod_sequencer.c ^
    src_c/chief_pose_estimator.c ^
    -lm
if %errorlevel% neq 0 ( echo FLIGHT LOOP BUILD FAILED & exit /b 1 )

echo.
echo === Compiling unit tests ===
gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_thekf.exe tests/test_thekf.c src_c/th_ekf.c -lm
if %errorlevel% neq 0 ( echo TH-EKF TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_mekf.exe tests/test_mekf.c src_c/mekf.c -lm
if %errorlevel% neq 0 ( echo MEKF TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_capture_gate.exe tests/test_capture_gate.c src_c/capture_gate.c -lm
if %errorlevel% neq 0 ( echo CAPTURE GATE TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_port_tracker.exe tests/test_port_tracker.c src_c/port_tracker.c -lm
if %errorlevel% neq 0 ( echo PORT TRACKER TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_terminal_filter.exe tests/test_terminal_filter.c src_c/terminal_filter.c -lm
if %errorlevel% neq 0 ( echo TERMINAL FILTER TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_spin_sync_controller.exe tests/test_spin_sync_controller.c src_c/spin_sync_controller.c -lm
if %errorlevel% neq 0 ( echo SPIN SYNC CONTROLLER TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_nmc_guidance.exe tests/test_nmc_guidance.c src_c/nmc_guidance.c -lm
if %errorlevel% neq 0 ( echo NMC GUIDANCE TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_keepout_planner.exe tests/test_keepout_planner.c src_c/keepout_planner.c -lm
if %errorlevel% neq 0 ( echo KEEPOUT PLANNER TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_rpod.exe tests/test_rpod.c src_c/rpod_ctrl.c src_c/capture_gate.c -lm
if %errorlevel% neq 0 ( echo RPOD TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_quest.exe tests/test_quest.c src_c/quest.c -lm
if %errorlevel% neq 0 ( echo QUEST TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_adcs.exe tests/test_adcs.c src_c/adcs.c -lm
if %errorlevel% neq 0 ( echo ADCS TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_mode_manager.exe tests/test_mode_manager.c src_c/mode_manager.c -lm
if %errorlevel% neq 0 ( echo MODE MANAGER TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_lambert.exe tests/test_lambert.c src_c/lambert_solver.c -lm
if %errorlevel% neq 0 ( echo LAMBERT TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS -Isrc_c -o test_rpod_sequencer.exe tests/test_rpod_sequencer.c src_c/rpod_sequencer.c src_c/rpod_ctrl.c src_c/capture_gate.c src_c/lambert_solver.c -lm
if %errorlevel% neq 0 ( echo RPOD SEQUENCER TEST BUILD FAILED & exit /b 1 )

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
test_capture_gate.exe
if %errorlevel% neq 0 ( echo CAPTURE GATE TESTS FAILED & exit /b 1 )
test_port_tracker.exe
if %errorlevel% neq 0 ( echo PORT TRACKER TESTS FAILED & exit /b 1 )
test_terminal_filter.exe
if %errorlevel% neq 0 ( echo TERMINAL FILTER TESTS FAILED & exit /b 1 )
test_spin_sync_controller.exe
if %errorlevel% neq 0 ( echo SPIN SYNC CONTROLLER TESTS FAILED & exit /b 1 )
test_nmc_guidance.exe
if %errorlevel% neq 0 ( echo NMC GUIDANCE TESTS FAILED & exit /b 1 )
test_keepout_planner.exe
if %errorlevel% neq 0 ( echo KEEPOUT PLANNER TESTS FAILED & exit /b 1 )
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
test_rpod_sequencer.exe
if %errorlevel% neq 0 ( echo RPOD SEQUENCER TESTS FAILED & exit /b 1 )
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
echo === Running SIL maturity fault matrix ===
python sim_python/sil_maturity_suite.py
if %errorlevel% neq 0 ( echo SIL MATURITY MATRIX FAILED & exit /b 1 )

echo.
echo === Running SIL long-duration soak ===
python sim_python/sil_soak.py
if %errorlevel% neq 0 ( echo SIL SOAK FAILED & exit /b 1 )

echo.
echo === Running Closed-Loop SIL verification ===
python sim_python/closed_loop_sil.py
if %errorlevel% neq 0 ( echo CLOSED LOOP SIL FAILED & exit /b 1 )

echo.
echo === ALL PASS ===


