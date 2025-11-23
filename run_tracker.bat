@echo off
setlocal

set ORIENTATIONS=horizontal_right horizontal_left vertical_right vertical_left

if "%1"=="-h" goto help
if "%1"=="--help" goto help

python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo Python is not installed. Please install Python and try again.
    pause
    exit /b 1
)

if not exist venv (
    echo Creating virtual environment...
    python -m venv venv
)

call venv\Scripts\activate.bat

echo Installing requirements...
pip install -r requirements.txt

echo Running tracker...
python wmr_tracker.py %*

pause
endlocal
exit /b 0

:help
echo Usage: run_tracker.bat [--orientation ^<orientation^>]
echo Available orientations: %ORIENTATIONS%
pause
exit /b 0
