@echo off
title DJI RoboMaster S1 - Global Launch
echo ====================================================
echo    ROBOMASTER S1 FULL LAUNCH (SERVER + AI VISION)
echo ====================================================
echo.

if not "%RPI_LOG_IP%"=="" (
    echo [📡] Remote Log Streaming : ACTIVE -^> %RPI_LOG_IP%:9999 (Raspberry Pi)
)

echo [1/3] Starting RoboMaster server and camera bridge...
if not "%RPI_LOG_IP%"=="" (
    start "RoboMaster S1 Server" /d "%~dp0robomaster" robomaster_server.exe 10.156.149.194 %RPI_LOG_IP%
) else (
    start "RoboMaster S1 Server" /d "%~dp0robomaster" robomaster_server.exe
)

echo [2/3] Opening Web Cockpit (http://localhost:8080)...
timeout /t 3 /nobreak >nul
start http://localhost:8080

echo [3/3] Launching Local AI Vision (YOLOv8 nano)...
echo [*] Automatic Fire       : DISABLED by default (activate in Web Cockpit)
echo [*] Sentry Turret Standby: DISABLED by default (activate in Web Cockpit)
echo [*] Target & Controls    : 100%% via Web Cockpit (http://localhost:8080)
echo.

cd /d "%~dp0"
if not "%RPI_LOG_IP%"=="" (
    .\.venv\Scripts\python.exe ai_vision.py --rpi-ip %RPI_LOG_IP% %*
) else (
    .\.venv\Scripts\python.exe ai_vision.py %*
)

echo.
echo [*] Stopping server and cleaning up processes...
taskkill /fi "WINDOWTITLE eq RoboMaster S1 Server*" /f >nul 2>&1
taskkill /im robomaster_server.exe /f >nul 2>&1
