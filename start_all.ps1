# Global PowerShell Launcher for DJI RoboMaster S1 (Server + Web Cockpit + Local AI)
param(
    [string]$RpiIP = $env:RPI_LOG_IP,
    [string]$RobotIP = "10.156.149.194"
)

$host.UI.RawUI.WindowTitle = "DJI RoboMaster S1 - Global Launch"

Write-Host "====================================================" -ForegroundColor Cyan
Write-Host "   ROBOMASTER S1 FULL LAUNCH (SERVER + AI VISION)   " -ForegroundColor Yellow
Write-Host "====================================================" -ForegroundColor Cyan
Write-Host ""

if ($RpiIP) {
    Write-Host "[📡] Remote Log Streaming : ACTIVÉ -> $RpiIP:9999 (Raspberry Pi)" -ForegroundColor Magenta
} else {
    Write-Host "[📡] Remote Log Streaming : Non configuré au démarrage (configurable via Web Cockpit)" -ForegroundColor DarkGray
}

Write-Host "[1/3] Starting RoboMaster server and camera bridge..." -ForegroundColor Green
$serverArgs = @($RobotIP)
if ($RpiIP) {
    $serverArgs += $RpiIP
}
$serverProcess = Start-Process -FilePath "$PSScriptRoot\robomaster\robomaster_server.exe" -ArgumentList $serverArgs -WorkingDirectory "$PSScriptRoot\robomaster" -PassThru

Write-Host "[2/3] Opening Web Cockpit (http://localhost:8080)..." -ForegroundColor Green
Start-Sleep -Seconds 3
Start-Process "http://localhost:8080"

Write-Host "[3/3] Launching Local AI Vision (YOLOv8 nano)..." -ForegroundColor Green
Write-Host "[*] Automatic Fire       : DISABLED by default (activate in Web Cockpit)"
Write-Host "[*] Sentry Turret Standby: DISABLED by default (activate in Web Cockpit)"
Write-Host "[*] Target & Controls    : 100% via Web Cockpit (http://localhost:8080)`n"

$pythonArgs = @("$PSScriptRoot\ai_vision.py")
if ($RpiIP) {
    $pythonArgs += "--rpi-ip"
    $pythonArgs += $RpiIP
}
$pythonArgs += $args

try {
    & "$PSScriptRoot\.venv\Scripts\python.exe" @pythonArgs
} finally {
    Write-Host "`n[*] Stopping RoboMaster server..." -ForegroundColor Yellow
    if ($serverProcess -and -not $serverProcess.HasExited) {
        Stop-Process -Id $serverProcess.Id -Force -ErrorAction SilentlyContinue
    }
}
