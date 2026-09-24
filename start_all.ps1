# Global PowerShell Launcher for DJI RoboMaster S1 (Server + Web Cockpit + Local AI)
$host.UI.RawUI.WindowTitle = "DJI RoboMaster S1 - Global Launch"

Write-Host "====================================================" -ForegroundColor Cyan
Write-Host "   ROBOMASTER S1 FULL LAUNCH (SERVER + AI VISION)   " -ForegroundColor Yellow
Write-Host "====================================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "[1/3] Starting RoboMaster server and camera bridge..." -ForegroundColor Green
$serverProcess = Start-Process -FilePath "$PSScriptRoot\robomaster\robomaster_server.exe" -WorkingDirectory "$PSScriptRoot\robomaster" -PassThru

Write-Host "[2/3] Opening Web Cockpit (http://localhost:8080)..." -ForegroundColor Green
Start-Sleep -Seconds 3
Start-Process "http://localhost:8080"

Write-Host "[3/3] Launching Local AI Vision (YOLOv8 nano)..." -ForegroundColor Green
Write-Host "[*] Automatic Fire       : DISABLED by default (activate in Web Cockpit)"
Write-Host "[*] Sentry Turret Standby: DISABLED by default (activate in Web Cockpit)"
Write-Host "[*] Target & Controls    : 100% via Web Cockpit (http://localhost:8080)`n"

try {
    & "$PSScriptRoot\.venv\Scripts\python.exe" "$PSScriptRoot\ai_vision.py" @args
} finally {
    Write-Host "`n[*] Stopping RoboMaster server..." -ForegroundColor Yellow
    if ($serverProcess -and -not $serverProcess.HasExited) {
        Stop-Process -Id $serverProcess.Id -Force -ErrorAction SilentlyContinue
    }
}
