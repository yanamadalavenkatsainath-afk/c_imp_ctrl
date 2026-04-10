@echo off
REM ================================================================
REM build.bat — Compile GNC C library + run verification
REM
REM CMSIS-DSP note:
REM   Desktop SIL: -DMEKF_NO_CMSIS uses linalg.h fallback.
REM   ARM/STM32:   remove that flag, link CMSIS-DSP. Same C source.
REM
REM Run from Satellite_GNC\ root:
REM   build.bat
REM ================================================================

echo === Ensuring Python package structure in flight sim ===
REM Python needs __init__.py in subfolders to treat them as packages.
REM This creates empty ones if they don't exist — safe to run repeatedly.
set FLIGHT_SIM=C:\Users\Venkat\OneDrive\Desktop\appex\flight sim
if not exist "%FLIGHT_SIM%\estimation\__init__.py" (
    type nul > "%FLIGHT_SIM%\estimation\__init__.py"
    echo   Created: estimation\__init__.py
) else (
    echo   OK: estimation\__init__.py exists
)
if not exist "%FLIGHT_SIM%\control\__init__.py" (
    type nul > "%FLIGHT_SIM%\control\__init__.py"
    echo   Created: control\__init__.py
)
if not exist "%FLIGHT_SIM%\sim_python\__init__.py" (
    type nul > "sim_python\__init__.py"
    echo   Created: sim_python\__init__.py
)

echo.

REM -Isrc_c is required: th_ekf.c and mekf.c both #include "linalg.h"
REM  which lives in src_c\ — without this flag gcc cannot find it.
REM rpod_ctrl.c is included so wrapper.py can call RPOD_prox_ops / RPOD_terminal.
gcc -fPIC -shared -O2 -DMEKF_NO_CMSIS ^
    -Isrc_c ^
    -o gnc_lib.dll ^
    src_c/th_ekf.c src_c/mekf.c src_c/rpod_ctrl.c ^
    -lm
if %errorlevel% neq 0 ( echo BUILD FAILED & exit /b 1 )
echo Built: gnc_lib.dll

REM -Isrc_c needed here too: test includes "../src_c/th_ekf.h" which
REM  itself includes "linalg.h" via th_ekf.c — the -I. previously used
REM  only worked if linalg.h was in the project root, not src_c\.
gcc -O2 -DMEKF_NO_CMSIS ^
    -Isrc_c ^
    -o test_thekf.exe ^
    tests/test_thekf.c src_c/th_ekf.c ^
    -lm
if %errorlevel% neq 0 ( echo TH-EKF TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS ^
    -Isrc_c ^
    -o test_mekf.exe ^
    tests/test_mekf.c src_c/mekf.c ^
    -lm
if %errorlevel% neq 0 ( echo MEKF TEST BUILD FAILED & exit /b 1 )

gcc -O2 -DMEKF_NO_CMSIS ^
    -Isrc_c ^
    -o test_rpod.exe ^
    tests/test_rpod.c src_c/rpod_ctrl.c ^
    -lm
if %errorlevel% neq 0 ( echo RPOD TEST BUILD FAILED & exit /b 1 )

echo.
echo === Running C unit tests ===
test_thekf.exe
if %errorlevel% neq 0 ( echo TH-EKF TESTS FAILED & exit /b 1 )

test_mekf.exe
if %errorlevel% neq 0 ( echo MEKF TESTS FAILED & exit /b 1 )

test_rpod.exe
if %errorlevel% neq 0 ( echo RPOD TESTS FAILED & exit /b 1 )

echo.
echo === Running Python SIL comparison ===
python sim_python/verify_sil.py
if %errorlevel% neq 0 ( echo SIL VERIFICATION FAILED & exit /b 1 )

echo.
echo === ALL PASS ===