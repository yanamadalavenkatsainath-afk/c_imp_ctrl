@echo off
REM ================================================================
REM build.bat — Compile GNC C library + run SIL verification
REM Now includes flight_loop.c for PC real-time SIL
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
if %errorlevel% neq 0 ( echo pyrightconfig generation FAILED &amp; exit /b 1 )

echo.
echo === Compiling GNC library (with flight_loop) ===
gcc -fPIC -shared -O2 -DMEKF_NO_CMSIS ^
    -Isrc_c ^
    -o gnc_lib.dll ^
    src_c/th_ekf.c src_c/mekf.c src_c/rpod_ctrl.c src_c/flight_loop.c ^
    -lm
if %errorlevel% neq 0 ( echo BUILD FAILED & exit /b 1 )
echo Built: gnc_lib.dll

echo.
echo === Compiling flight_loop standalone smoke test ===
gcc -O2 -DMEKF_NO_CMSIS -DFLIGHT_LOOP_STANDALONE -Isrc_c ^
    -o flight_loop_test.exe ^
    src_c/flight_loop.c src_c/th_ekf.c src_c/mekf.c src_c/rpod_ctrl.c ^
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

echo.
echo === Running Python SIL comparison ===
python sim_python/verify_sil.py
if %errorlevel% neq 0 ( echo SIL VERIFICATION FAILED & exit /b 1 )

echo.
echo === Running Real-Time SIL verification ===
python sim_python/verify_realtime_sil.py
if %errorlevel% neq 0 ( echo REALTIME SIL FAILED & exit /b 1 )

echo.
echo === ALL PASS ===