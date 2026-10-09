@echo off
rem ==================================================================
rem  DRONE_GUI.cmd -- double-click: Ekko's ground station page,
rem  served from THIS laptop, opened in the default browser.
rem
rem  Runs in: Windows. gcs_radio.py (system python, no ROS) serves the
rem  aircraft's own page on http://localhost:8090. Telemetry, buoys and
rem  the fence come off udpin:127.0.0.1:14543; the camera and the
rem  buttons go to ekko.local over WiFi. It runs in its own window,
rem  titled "Drone GUI (laptop)"; close that window (or Ctrl+C in it)
rem  to stop it. If the server is already answering, this just opens
rem  the browser - no second one.
rem
rem  This does NOT start anything on the radio. Exactly one of these
rem  must own COM8 and feed 127.0.0.1:14543:
rem    - QGroundControl, with Telemetry > MAVLink Forwarding enabled
rem    - tools\scripts\radio_link.py --port COM8
rem    - OCS\FLEET_LINK.cmd (boat and drone together)
rem  The server window says "telemetry is flowing" once one of them is.
rem
rem  Extra arguments go to gcs_radio.py, e.g.
rem    DRONE_GUI.cmd --jetson 192.168.8.50 --port 8091
rem  Set DRONE_GUI_NO_BROWSER=1 to skip opening the browser.
rem
rem  The probe is curl.exe and the server window is an ordinary visible
rem  one: this PC's malware protection deleted a launcher that used a
rem  hidden background window and a scripted web-request loop
rem  (2026-09-30; see Boat\rx26_asv\tools\scripts\BOAT_GUI.cmd).
rem ==================================================================
setlocal EnableDelayedExpansion
set "PORT=8090"
set "PREV="
rem --port N or --port=N (the for loop splits on = as well)
for %%a in (%*) do (
  if "!PREV!"=="--port" set "PORT=%%~a"
  set "PREV=%%~a"
)
set "PROBE=http://127.0.0.1:%PORT%/"

curl.exe -sf -o nul -m 1 "%PROBE%"
if not errorlevel 1 (
  echo The Drone GUI is already running on port %PORT%.
  goto :open
)

python -c "import sys" >nul 2>&1
if errorlevel 1 goto :nopython

echo Starting the Drone GUI server in its own window...
rem cmd /k keeps that window open if the server exits, so a startup
rem error (port in use, missing module) can be read instead of vanishing.
start "Drone GUI (laptop)" cmd /k python "%~dp0gcs_radio.py" %*

set "UP="
echo Waiting for it to answer on port %PORT% (up to 15 s)...
for /l %%n in (1,1,15) do if not defined UP call :probe
if not defined UP goto :noanswer

:open
if defined DRONE_GUI_NO_BROWSER goto :done
start "" "http://localhost:%PORT%/"
:done
exit /b 0

:nopython
echo.
echo *** "python" was not found, or is the Microsoft Store stub.
echo     Install Python 3 for Windows with pymavlink and pyserial:
echo         python -m pip install pymavlink pyserial
echo.
pause
exit /b 2

:noanswer
echo.
echo *** Nothing answered on port %PORT% within 15 s. Read the window
echo     titled "Drone GUI (laptop)" for the reason.
echo.
pause
exit /b 2

rem one round of the wait. ping is the one-second sleep: timeout.exe
rem refuses to run when stdin is not a console and would end the wait
rem instantly.
:probe
curl.exe -sf -o nul -m 1 "%PROBE%" && set "UP=1"
if not defined UP ping -n 2 127.0.0.1 >nul
exit /b 0
