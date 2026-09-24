#!/usr/bin/env bash
# ==============================================================================
# Script de lancement global pour Raspberry Pi / Linux
# Lance le Serveur RoboMaster + Cockpit Web (port 8080) + IA Vision Locale
# ==============================================================================

ROBOT_IP=${1:-"10.156.149.194"}
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$DIR"

echo "=========================================================="
echo "   LANCEMENT GLOBAL ROBOMASTER S1 (SERVEUR + VISION IA)   "
echo "=========================================================="
echo "[*] Adresse IP cible du robot : $ROBOT_IP"

# 1. Démarrage du serveur RoboMaster
echo -e "\n[1/3] Démarrage du serveur RoboMaster et du bridge caméra..."
if [ -f "robomaster/robomaster_server.exe" ]; then
    SERVER_EXEC="robomaster/robomaster_server.exe"
elif [ -f "robomaster_server.exe" ]; then
    SERVER_EXEC="./robomaster_server.exe"
else
    echo "[!] Erreur : robomaster_server.exe introuvable."
    exit 1
fi

# Exécution avec Box64 + Wine sur ARM64, ou Wine standard sur x86
if command -v box64 &> /dev/null && command -v wine &> /dev/null; then
    box64 wine "$SERVER_EXEC" "$ROBOT_IP" > /dev/null 2>&1 &
    SERVER_PID=$!
elif command -v wine &> /dev/null; then
    wine "$SERVER_EXEC" "$ROBOT_IP" > /dev/null 2>&1 &
    SERVER_PID=$!
else
    echo "[!] Wine / Box64 n'est pas installé. Exécution impossible de $SERVER_EXEC."
    echo "    Veuillez exécuter ./setup_rpi.sh pour installer les composants."
    exit 1
fi

# Fonction de nettoyage à l'arrêt
cleanup() {
    echo -e "\n[*] Arrêt du serveur RoboMaster (PID: $SERVER_PID)..."
    kill -9 "$SERVER_PID" 2>/dev/null || true
    echo "[✓] Système arrêté proprement."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 2. Attente de la disponibilité du serveur
echo "[2/3] Initialisation du serveur et du flux vidéo..."
sleep 3

# Détection de l'adresse IP locale du Raspberry Pi pour affichage
RPI_IP=$(hostname -I | awk '{print $1}')
echo -e "\n[🚀] Cockpit Web disponible à l'adresse :"
echo "     👉 http://localhost:8080"
if [ -n "$RPI_IP" ]; then
    echo "     👉 http://${RPI_IP}:8080 (depuis PC, tablette ou smartphone sur le réseau)"
fi
echo ""

# 3. Lancement de l'IA Vision (Python)
echo "[3/3] Lancement de l'IA Vision Locale (YOLOv8 nano)..."
if [ -d ".venv" ]; then
    source .venv/bin/activate
fi

python3 ai_vision.py --url http://localhost:8080 "$@"
