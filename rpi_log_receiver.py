#!/usr/bin/env python3
"""
=============================================================================
DJI ROBOMASTER S1 - RASPBERRY PI REMOTE LOG RECEIVER
=============================================================================
Ce script s'exécute sur le Raspberry Pi (ou tout PC distant) pour recevoir,
afficher en direct dans le terminal avec des couleurs, et sauvegarder dans
un fichier local ('rpi_robot_actions.log') tous les logs d'actions émis par
le robot et l'IA s'exécutant sur Windows.

Transport : UDP (port 9999 par défaut) - Non bloquant et ultra résilient.
Dépendances : Aucune dépendance externe (uniquement Python 3 standard).

Usage sur le Raspberry Pi :
    python3 rpi_log_receiver.py
    python3 rpi_log_receiver.py --port 9999 --log-file rpi_robot_actions.log
=============================================================================
"""

import argparse
import datetime
import os
import re
import socket
import sys
import time

# Codes de couleurs ANSI pour affichage terminal
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"

RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
WHITE = "\033[97m"

BG_RED = "\033[41m"
BG_DARK = "\033[40m"

def get_local_ip():
    """Détecte l'adresse IP locale du Raspberry Pi sur le réseau WiFi ou Ethernet."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Se connecte virtuellement à une IP externe pour résoudre l'interface réseau active
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip

def colorize_log(raw_line):
    """
    Colorise intelligemment la ligne de log selon sa catégorie.
    Format attendu : [YYYY-MM-DD HH:MM:SS] [CATEGORY] Message
    """
    line = raw_line.strip()
    match = re.match(r"^(\[[0-9\-: ]+\])\s*(\[[A-Za-z0-9_\-]+\])\s*(.*)$", line)
    if not match:
        return f"{DIM}{line}{RESET}"

    timestamp_part, cat_part, msg_part = match.groups()
    cat_upper = cat_part.upper()

    ts_colored = f"{DIM}{timestamp_part}{RESET}"

    if "FIRE" in cat_upper or "TIR" in cat_upper:
        cat_colored = f"{BOLD}{RED}{cat_part}{RESET}"
        msg_colored = f"{BOLD}{RED}💥 {msg_part}{RESET}"
    elif "LOCK" in cat_upper:
        cat_colored = f"{BOLD}{YELLOW}{cat_part}{RESET}"
        msg_colored = f"{BOLD}{YELLOW}🎯 {msg_part}{RESET}"
    elif "TURRET" in cat_upper or "GIMBAL" in cat_upper:
        cat_colored = f"{CYAN}{cat_part}{RESET}"
        msg_colored = f"{CYAN}{msg_part}{RESET}"
    elif "AI" in cat_upper or "VISION" in cat_upper or "TRACK" in cat_upper or "TARGET" in cat_upper:
        cat_colored = f"{MAGENTA}{cat_part}{RESET}"
        msg_colored = f"{MAGENTA}🤖 {msg_part}{RESET}"
    elif "CONN" in cat_upper or "READY" in cat_upper or "OK" in cat_upper:
        cat_colored = f"{BOLD}{GREEN}{cat_part}{RESET}"
        msg_colored = f"{GREEN}🟢 {msg_part}{RESET}"
    elif "DISCONN" in cat_upper or "HALT" in cat_upper:
        cat_colored = f"{BOLD}{YELLOW}{cat_part}{RESET}"
        msg_colored = f"{YELLOW}🛑 {msg_part}{RESET}"
    elif "CONFIG" in cat_upper:
        cat_colored = f"{BLUE}{cat_part}{RESET}"
        msg_colored = f"{BLUE}⚙️  {msg_part}{RESET}"
    elif "SAFETY" in cat_upper or "FACE" in cat_upper:
        cat_colored = f"{BOLD}{YELLOW}{cat_part}{RESET}"
        msg_colored = f"{BOLD}{YELLOW}🛡️  {msg_part}{RESET}"
    elif "ERR" in cat_upper:
        cat_colored = f"{BOLD}{WHITE}{BG_RED}{cat_part}{RESET}"
        msg_colored = f"{BOLD}{RED}⚠️  {msg_part}{RESET}"
    elif "TEST" in cat_upper:
        cat_colored = f"{BOLD}{CYAN}{cat_part}{RESET}"
        msg_colored = f"{CYAN}📡 {msg_part}{RESET}"
    else:
        cat_colored = f"{WHITE}{cat_part}{RESET}"
        msg_colored = msg_part

    return f"{ts_colored} {cat_colored} {msg_colored}"

def main():
    parser = argparse.ArgumentParser(description="DJI RoboMaster S1 - Raspberry Pi Log Receiver")
    parser.add_argument("--port", type=int, default=9999, help="UDP listening port (default: 9999)")
    parser.add_argument("--log-file", default="rpi_robot_actions.log", help="Local log output file")
    args = parser.parse_args()

    local_ip = get_local_ip()

    print(f"\n{BOLD}{CYAN}===================================================={RESET}")
    print(f"{BOLD}{YELLOW}    DJI ROBOMASTER S1 - RASPBERRY PI LOG RECEIVER   {RESET}")
    print(f"{BOLD}{CYAN}===================================================={RESET}")
    print(f"{BOLD}📡 UDP Port         :{RESET} {GREEN}{args.port}{RESET}")
    print(f"{BOLD}📁 Log File Output  :{RESET} {WHITE}{os.path.abspath(args.log_file)}{RESET}")
    print(f"{BOLD}📍 Raspberry Pi IP  :{RESET} {BOLD}{GREEN}{local_ip}{RESET}")
    print(f"{BOLD}{CYAN}----------------------------------------------------{RESET}")
    print(f"👉 {WHITE}Renseignez cette IP sur Windows via :{RESET}")
    print(f"   1. Le Web Cockpit : Dans la carte {BOLD}'Raspberry Pi Log Streaming'{RESET} -> {GREEN}{local_ip}{RESET}")
    print(f"   2. En ligne de commande : {DIM}.\\start_all.ps1 -RpiIP {local_ip}{RESET}")
    print(f"   3. Variable d'env : {DIM}$env:RPI_LOG_IP = '{local_ip}'{RESET}")
    print(f"{BOLD}{CYAN}----------------------------------------------------{RESET}")
    print(f"{YELLOW}[*] En attente de logs depuis le robot sous Windows... (Ctrl+C pour quitter){RESET}\n")

    # Création du socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", args.port))
    except Exception as e:
        print(f"{BOLD}{RED}[!] Impossible de lier le port UDP {args.port}: {e}{RESET}")
        sys.exit(1)

    count = 0
    try:
        with open(args.log_file, "a", encoding="utf-8") as f:
            f.write(f"\n--- SESSION RECEIVER DEMARREE LE {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ---\n")
            f.flush()

            while True:
                data, addr = sock.recvfrom(4096)
                if not data:
                    continue
                try:
                    text = data.decode("utf-8", errors="replace")
                except Exception:
                    text = str(data)

                # Écrire dans le fichier local
                f.write(text)
                if not text.endswith("\n"):
                    f.write("\n")
                f.flush()

                # Affichage en direct colorisé
                for line in text.splitlines():
                    if line.strip():
                        count += 1
                        print(colorize_log(line))
                        sys.stdout.flush()

    except KeyboardInterrupt:
        print(f"\n{BOLD}{YELLOW}[*] Arrêt du récepteur de logs. Total de logs reçus : {count}{RESET}")
    finally:
        sock.close()
        print(f"{GREEN}[✓] Socket UDP fermé et logs sauvegardés dans '{args.log_file}'.{RESET}")

if __name__ == "__main__":
    main()
