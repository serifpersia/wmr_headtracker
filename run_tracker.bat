@echo off
setlocal

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
pip install -r requirements.txt >nul 2>&1

echo.
echo ==================================================
echo           WMR SENSOR INITIALIZATION
echo ==================================================
echo.
echo Step 1: Triggering sensor wake-up on Interface 1...

python wmr_tracker.py 1 --wake

echo.
echo Step 2: Waiting for sensor reset...

timeout /t 3 /nobreak >nul

echo.
echo Step 3: Starting Tracking on Interface 0...
echo ==================================================

python wmr_tracker.py 0

pause
endlocal
