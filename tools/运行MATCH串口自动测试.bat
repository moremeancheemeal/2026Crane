@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"
set "SCRIPT=%~dp0match_serial_test.py"

rem Find a per-user Python that can import pyserial.
for /d %%D in ("%LocalAppData%\Programs\Python\Python*") do (
    if exist "%%~fD\python.exe" (
        "%%~fD\python.exe" -c "import serial" >nul 2>nul
        if not errorlevel 1 (
            echo Python: %%~fD\python.exe
            "%%~fD\python.exe" "%SCRIPT%" %*
            goto finished
        )
    )
)

rem Fall back to PATH and Python Launcher, with the same import check.
python -c "import serial" >nul 2>nul
if not errorlevel 1 (
    echo Python: python
    python "%SCRIPT%" %*
    goto finished
)

py -3 -c "import serial" >nul 2>nul
if not errorlevel 1 (
    echo Python: py -3
    py -3 "%SCRIPT%" %*
    goto finished
)

echo No Python environment with pyserial was found.
echo Install it and retry: python -m pip install pyserial
set "SCRIPT_RESULT=3"
goto end

:finished
set "SCRIPT_RESULT=%errorlevel%"

:end
echo.
if "%~1"=="" pause
exit /b %SCRIPT_RESULT%
