@echo off
rem Double-click to set up (first time) and open the active aero simulator.
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (py -3 aero.py %*) else (python aero.py %*)
if errorlevel 1 pause
