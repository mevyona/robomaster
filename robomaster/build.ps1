$gccDir = "C:\Users\mev\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
$goExe = "C:\Program Files\Go\bin\go.exe"

$env:PATH = "$gccDir;C:\Program Files\Go\bin;$env:PATH"
$env:CGO_ENABLED = "1"
$env:CC = "gcc"

Write-Host "Compilation du serveur RoboMaster..."
& $goExe build -o robomaster_server.exe .\cmd\server\main.go
if ($LASTEXITCODE -eq 0) {
    Copy-Item -Path robomaster_server.exe -Destination ..\robomaster_server.exe -Force -ErrorAction SilentlyContinue
    Write-Host "[+] Compilation réussie : robomaster_server.exe généré et synchronisé !" -ForegroundColor Green
} else {
    Write-Host "[!] Échec de la compilation" -ForegroundColor Red
}
