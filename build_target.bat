@echo off
setlocal EnableExtensions

set "ARM_GCC=%ARM_GCC%"
if "%ARM_GCC%"=="" set "ARM_GCC=arm-none-eabi-gcc"

where "%ARM_GCC%" >nul 2>nul
if %errorlevel% neq 0 (
    echo TARGET TOOLCHAIN NOT FOUND: %ARM_GCC%
    echo Install arm-none-eabi-gcc or set ARM_GCC to the target compiler path.
    exit /b 2
)

if not exist target_build mkdir target_build
del /q target_build\*.o target_build\*.su target_build\*.a 2>nul

set CFLAGS=-Os -DTARGET_BUILD -DMEKF_NO_CMSIS -ffunction-sections -fdata-sections -fstack-usage -Wall -Wextra -Isrc_c
set SRCS=src_c\th_ekf.c src_c\mekf.c src_c\capture_gate.c src_c\rpod_ctrl.c src_c\terminal_filter.c src_c\port_tracker.c src_c\spin_sync_controller.c src_c\nmc_guidance.c src_c\keepout_planner.c src_c\quest.c src_c\adcs.c src_c\mode_manager.c src_c\lambert_solver.c src_c\rpod_sequencer.c src_c\chief_pose_estimator.c src_c\flight_loop.c

for %%S in (%SRCS%) do (
    echo [CC] %%S
    "%ARM_GCC%" %CFLAGS% -c %%S -o target_build\%%~nS.o
    if %errorlevel% neq 0 exit /b 1
)

where arm-none-eabi-ar >nul 2>nul
if %errorlevel% equ 0 (
    arm-none-eabi-ar rcs target_build\libgnc_fsw.a target_build\*.o
    if %errorlevel% neq 0 exit /b 1
    echo Built: target_build\libgnc_fsw.a
) else (
    echo NOTE: arm-none-eabi-ar not found; object compile completed, archive skipped.
)

where arm-none-eabi-size >nul 2>nul
if %errorlevel% equ 0 arm-none-eabi-size target_build\*.o

python tools\analyze_stack_usage.py target_build
exit /b %errorlevel%
