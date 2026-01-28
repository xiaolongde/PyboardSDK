@echo off
REM ============================================================
REM GDB 调试启动脚本
REM 请先运行 start_gdb_server.bat 启动 JLink GDB Server
REM ============================================================

set GDB_PATH=arm-none-eabi-gdb
set ELF_FILE=build\PyboardSDK.elf
set GDB_INIT=gdb_init.cfg

echo ============================================================
echo Starting GDB Debug Session
echo ELF File: %ELF_FILE%
echo ============================================================
echo.
echo Make sure JLink GDB Server is running (start_gdb_server.bat)
echo.

%GDB_PATH% -x %GDB_INIT% %ELF_FILE%

pause
