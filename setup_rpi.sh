#!/usr/bin/env bash
# ==============================================================================
# Script d'installation automatique pour Raspberry Pi (OS 64-bit aarch64)
# Projet : DJI RoboMaster S1 - Serveur & Vision IA Locale
# ==============================================================================

set -e

echo "=========================================================="
echo "   INSTALLATION ROBOMASTER S1 SUR RASPBERRY PI 64-BIT     "
echo "=========================================================="

ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    echo "[!] AVERTISSEMENT : Ce projet nécessite un OS 64-bit (aarch64)."
    echo "    Architecture détectée : $ARCH"
    echo "    Veuillez installer Raspberry Pi OS 64-bit (Debian Bookworm recommandé)."
fi

# 1. Mise à jour des dépôts
echo -e "\n[1/5] Mise à jour des paquets système..."
sudo apt update && sudo apt upgrade -y

# 2. Dépendances système pour Python, OpenCV et multimédia
echo -e "\n[2/5] Installation des dépendances système..."
sudo apt install -y python3 python3-pip python3-venv python3-dev \
    git curl wget libgl1 libglib2.0-0 libgomp1 libatlas-base-dev

# 3. Installation de Box64 et Wine (pour exécuter le bridge UnityBridge x86_64)
echo -e "\n[3/5] Configuration de Box64 & Wine64 pour le bridge DJI..."
if ! command -v box64 &> /dev/null; then
    echo "[+] Ajout du dépôt Pi-Apps / Ryan Fortner pour Box64 & Wine..."
    wget https://ryanfortner.github.io/box64-debs/box64.list -O /tmp/box64.list
    sudo mv /tmp/box64.list /etc/apt/sources.list.d/box64.list
    wget -qO- https://ryanfortner.github.io/box64-debs/KEY.gpg | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/box64-debs-archive-keyring.gpg > /dev/null
    
    # Dépôt Wine64 x86_64 pour box64
    wget https://ryanfortner.github.io/box86-debs/box86.list -O /tmp/box86.list || true
    
    sudo apt update
    sudo apt install -y box64-rpi4arm64 || sudo apt install -y box64 || echo "[!] Box64 sera à installer via Pi-Apps si échec apt"
    sudo apt install -y wine wine64 || true
else
    echo "[✓] Box64 est déjà installé."
fi

# 4. Création de l'environnement virtuel Python
echo -e "\n[4/5] Configuration de l'environnement virtuel Python..."
if [ ! -d ".venv" ]; then
    python3 -m venv .venv
fi

source .venv/bin/activate
pip install --upgrade pip setuptools wheel
pip install opencv-python-headless requests numpy
pip install ultralytics --extra-index-url https://download.pytorch.org/whl/cpu

# 5. Rendre les scripts exécutables
chmod +x start_all.sh setup_rpi.sh 2>/dev/null || true

echo -e "\n=========================================================="
echo "   [✓] INSTALLATION TERMINÉE AVEC SUCCÈS SUR RASPBERRY PI  "
echo "=========================================================="
echo "Pour lancer le système :"
echo "  ./start_all.sh [IP_DU_ROBOT]"
echo "Exemple : ./start_all.sh 10.156.149.194"
echo "Le Cockpit Web sera accessible sur : http://<IP_DU_PI>:8080"
