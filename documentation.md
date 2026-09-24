# Documentation Complète : Contrôle, Vision IA Locale & Suivi de Cible DJI RoboMaster S1 (PC & Raspberry Pi)

Ce guide fournit une documentation technique complète, de A à Z, permettant à **toute personne ne connaissant pas le projet** de le comprendre, de l'installer et de le faire fonctionner soit sur un **PC (Windows)**, soit sur un **Raspberry Pi (Linux ARM64)** connecté à un **DJI RoboMaster S1**.

---

## Sommaire
1. [Vue d'Ensemble & Objectifs du Projet](#1-vue-densemble--objectifs-du-projet)
2. [Architecture Globale du Système](#2-architecture-globale-du-système)
3. [Arborescence Complète des Fichiers](#3-arborescence-complète-des-fichiers)
4. [Connexion Réseau au Robot (Wi-Fi & IP)](#4-connexion-réseau-au-robot-wi-fi--ip)
5. [Déploiement sur Raspberry Pi (Guide Pas à Pas)](#5-déploiement-sur-raspberry-pi-guide-pas-à-pas)
6. [Déploiement sur PC Windows (Guide Pas à Pas)](#6-déploiement-sur-pc-windows-guide-pas-à-pas)
7. [Fonctionnement du Cœur Serveur Go & UnityBridge](#7-fonctionnement-du-cœur-serveur-go--unitybridge)
8. [Module d'IA Vision Locale (Python & YOLOv8)](#8-module-dia-vision-locale-python--yolov8)
9. [Algorithme de Ciblage & Filtrage Intelligent](#9-algorithme-de-ciblage--filtrage-intelligent)
10. [Algorithme d'Asservissement Tourelle & Tir Automatique](#10-algorithme-dasservissement-tourelle--tir-automatique)
11. [Le Cockpit Web Tactique (Port 8080)](#11-le-cockpit-web-tactique-port-8080)
12. [Optimisations Spécifiques au Raspberry Pi (NCNN & Performance)](#12-optimisations-spécifiques-au-raspberry-pi-ncnn--performance)
13. [Résolution des Pannes (Troubleshooting)](#13-résolution-des-pannes-troubleshooting)

---

## 1. Vue d'Ensemble & Objectifs du Projet

Le but de ce projet est de transformer le **DJI RoboMaster S1** en une **tourelle sentinelle de surveillance autonome et intelligente** pilotée sans fil par un PC ou un Raspberry Pi :
- **Sécurité totale en intérieur** : les roues du châssis sont **strictement verrouillées et désactivées**, éliminant tout risque de mouvement incontrôlé ou de chute. Seuls la caméra et les axes de la tourelle (Pitch / Yaw) pivotent.
- **Cockpit Web ultra-fluide (60 FPS)** : affichage du flux vidéo HD en direct sans saccade avec réticule tactique (HUD Canvas). Accessible depuis n'importe quel navigateur (PC, smartphone, tablette) sur le réseau local.
- **IA de détection locale (YOLOv8)** : exécution du modèle de détection d'objets en local sur la machine hôte (0% de charge sur le navigateur web).
- **Cibles restreintes & intelligentes** :
  1. **Personne (`person`)** : algorithme calculant la personne **la plus proche** de la caméra et ignorant les personnes en arrière-plan. Protection du visage intégrée (visée torse sécurisée).
  2. **Bouteille (`bottle`)**.
  3. **Canette de soda (`can` / `cup`)**.
- **Verrouillage & Tir Automatique (Auto-Fire)** : dès que la cible sélectionnée est centrée dans le viseur pendant ~350 ms, la mire passe au rouge, verrouille la cible et déclenche automatiquement **1 tir infrarouge unique** (son laser synthétisé + LED du canon).
- **Enchaînement Intelligent & Changement de Cible après Tir** : dès qu'une cible est touchée, elle est enregistrée comme éliminée (`💥 HIT`) et le robot bascule automatiquement sur la cible suivante non touchée. Si toutes les cibles en vue sont éliminées, la patrouille sentinelle 360° reprend automatiquement.
- **Mode Standby Tourelle Sentinelle 360°** : en l'absence de cible, la tourelle effectue un balayage panoramique continu d'amplitude maximale gauche/droite. Dès qu'une cible entre dans le champ de vision, le balayage s'interrompt instantanément pour engager le suivi, le verrouillage et le tir.

---

## 2. Architecture Globale du Système

Le système repose sur un découplage en 3 couches indépendantes :

```
                  ┌─────────────────────────────────────────┐
                  │          DJI RoboMaster S1              │
                  │   IP: 10.156.149.194 (Wi-Fi)            │
                  │   - Caméra H264                         │
                  │   - Moteurs Tourelle (Pitch/Yaw)        │
                  │   - Canon Infrarouge (LED + Speaker)    │
                  │   - Châssis / Roues (VERROUILLÉES)      │
                  └────────────────────┬────────────────────┘
                                       │
                      Protocole propriétaire UnityBridge
                                       │
                                       ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                    SERVEUR LOCAL EN GO (Port 8080)                        │
│               (Exécuté sur PC Windows ou Raspberry Pi via Wine/Box64)     │
│                                                                           │
│  - Utilise la bibliothèque CGO + unitybridge.dll                          │
│  - Décode le flux vidéo H264 vers RGB / JPEG                              │
│  - Expose l'API REST (/api/gimbal, /api/fire, /api/status, /snapshot)     │
│  - Distribue le flux MJPEG (/video)                                       │
│  - Centralise les cibles (/api/target) et détections (/api/detections)    │
└──────────────────────┬─────────────────────────────▲──────────────────────┘
                       │                             │
        /snapshot (JPEG)                             │ /api/gimbal (Asservissement)
                       │                             │ /api/fire (Auto-tir)
                       ▼                             │ /api/detections (HUD)
┌────────────────────────────────────────┐           │
│        IA LOCALE (Python YOLOv8)       │           │
│         (ai_vision.py)                 ├───────────┘
│                                        │
│  - YOLOv8 nano (Inférence CPU/NEON)    │
│  - Filtre personne la plus proche      │
│  - Détection bouteille / canette       │
│  - Calcul d'erreur PID & centrage      │
│  - Verrouillage & Cooldown de tir      │
└────────────────────────────────────────┘
                       │
       Coordonnées des boîtes & Statut Lock
                       │
                       ▼
┌───────────────────────────────────────────────────────────────────────────┐
│                       COCKPIT WEB HTML5 / CANVAS                          │
│                       (http://<IP_HOTE>:8080)                             │
│                                                                           │
│  - Rendu Canvas 60 FPS sans lag (0 calcul IA dans le navigateur)          │
│  - Viseur dynamique (Vert = suivi, Rouge = Lock / Tir)                    │
│  - Sélecteur de cible en direct (synchronisé avec l'IA sans redémarrage)  │
│  - Interrupteur marche/arrêt de l'Auto-Tir                                │
│  - Contrôle manuel d'orientation aux flèches du clavier                   │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Arborescence Complète des Fichiers

Voici l'arborescence des fichiers du projet :

```
Robomaster S1/
│
├── AGENTS.md                   # Règles strictes de maintenance du projet et de documentation
├── README.md                   # Documentation d'origine
├── documentation.md            # La présente documentation technique exhaustive (PC & Raspberry Pi)
│
├── requirements.txt            # Dépendances Python (ultralytics, opencv, requests, numpy)
├── setup_rpi.sh                # Script d'installation automatique pour Raspberry Pi 64-bit
├── start_all.sh                # Script de lancement tout-en-un pour Raspberry Pi / Linux
├── start_all.bat               # Lanceur rapide Windows (Batch)
├── start_all.ps1               # Lanceur complet Windows (PowerShell)
│
├── ai_vision.py                # Cœur de l'IA de vision locale (YOLOv8, asservissement tourelle, tir auto)
├── detect_objects.py           # Script de test de détection simple
├── robomaster_api.py           # Bibliothèque cliente Python simplifiée pour interagir avec le serveur Go
├── robot_actions.log           # Fichier journal horodaté de toutes les actions du robot
│
├── robomaster_server.exe       # Binaire exécutable Windows du serveur Go
├── unitybridge.dll             # Bibliothèque dynamique DJI requise pour communiquer avec le robot
├── yolov8n.pt                  # Poids neuronaux YOLOv8 nano (détection des objets)
├── yolov8n-pose.pt             # Poids neuronaux YOLOv8 pose (protection du visage & ciblage torse)
│
└── robomaster/                 # Code source complet du module Go & UnityBridge
    ├── client.go               # Client principal de connexion au robot
    ├── go.mod                  # Dépendances Go
    ├── go.sum                  # Sommes de contrôle Go
    ├── build.ps1               # Script de compilation Go Windows
    ├── robomaster_server.exe   # Copie locale du binaire serveur
    ├── unitybridge.dll         # Copie locale de la DLL UnityBridge
    │
    ├── cmd/
    │   └── server/
    │       └── main.go         # Code source du serveur HTTP, décodeur vidéo et API REST
    │
    ├── module/                 # Modules de contrôle matériel
    │   ├── camera/             # Gestion du flux vidéo H264
    │   ├── chassis/            # Contrôle du châssis (désactivé pour sécurité)
    │   ├── gimbal/             # Contrôle de la tourelle (Pitch & Yaw)
    │   ├── gun/                # Contrôle du canon (tir infrarouge)
    │   ├── robot/              # Batterie, état du système
    │   ├── controller/         # Gestionnaire de contrôle
    │   ├── gamepad/            # Gestionnaire de manette
    │   └── sdcard/             # Carte SD du robot
    │
    ├── unitybridge/            # Wrapper Go de la bibliothèque UnityBridge DJI
    │   ├── wrapper/            # Bindings CGO
    │   └── install/            # Outils d'installation des bibliothèques
    │
    └── support/                # Utilitaires (PID, Logger, Découverte réseau, Chiffrement)
```

---

## 4. Connexion Réseau au Robot (Wi-Fi & IP)

Le DJI RoboMaster S1 supporte deux modes de connexion Wi-Fi :

### Mode 1 : Connexion Directe (Robot = Point d'Accès Wi-Fi)
1. Basculer le commutateur situé derrière la caméra du robot sur la position **Wi-Fi** (icône antenne).
2. Allumer le robot (un appui court puis un appui long sur le bouton batterie).
3. Sur votre Raspberry Pi ou PC, connectez-vous au réseau Wi-Fi diffusé par le robot :
   - **SSID** : `RM-S1_XXXXXX` (affiché sur l'étiquette sous le robot).
   - **Mot de passe par défaut** : `12341234`
4. L'adresse IP du robot en mode direct est généralement :
   - `192.168.2.1`

### Mode 2 : Mode Routeur (Robot et Hôte connectés à la même Box / Routeur) - Recommandé
1. Configurer le robot via l'application mobile DJI RoboMaster pour qu'il rejoigne votre réseau Wi-Fi local (ou le point d'accès de votre box).
2. Le robot obtient une adresse IP sur votre réseau local (par exemple : `10.156.149.194` ou `192.168.1.50`).
3. Connectez le Raspberry Pi (ou le PC) au même réseau Wi-Fi ou par câble Ethernet.
4. Vérifiez la connectivité réseau avec un ping :
   ```bash
   ping 10.156.149.194
   ```

---

## 5. Déploiement sur Raspberry Pi (Guide Pas à Pas)

### 5.1 Matériel Recommandé
- **Raspberry Pi 4 (4 Go ou 8 Go)** ou **Raspberry Pi 5 (4 Go ou 8 Go)**.
- Carte microSD (32 Go minimum, classe A2 recommandée) ou SSD USB3.
- Système d'exploitation : **Raspberry Pi OS 64-bit (Debian Bookworm)**.  
  *(Attention : un OS 32-bit n'est pas compatible avec PyTorch 64-bit et Box64).*
- Alimentation officielle Raspberry Pi (pour éviter les sous-tensions lors de l'inférence IA).

### 5.2 Pourquoi Box64 + Wine sur Raspberry Pi ?
DJI fournit la bibliothèque propriétaire `unitybridge` uniquement sous forme de binaire Windows x86_64 (`unitybridge.dll`) et de bibliothèque Android Bionic (`libunitybridge.so`). DJI ne fournit aucun binaire natif GNU/Linux glibc.
Pour exécuter le serveur Go sur le processeur ARM64 du Raspberry Pi, la solution standard et éprouvée consiste à utiliser **Box64** (émulateur d'instructions x86_64 vers ARM64 haute performance) combiné à **Wine64**. Cela permet d'exécuter `robomaster_server.exe` avec une consommation CPU minime (~5-10%), tandis que l'IA Vision tourne **nativement** en Python 64-bit sur les cœurs ARM du Pi.

### 5.3 Installation en 1 Commande sur le Raspberry Pi

1. Clonez le projet ou copiez les fichiers sur votre Raspberry Pi :
   ```bash
   git clone git@github.com:mevyona/robomaster.git
   cd robomaster
   ```

2. Rendez le script d'installation exécutable et lancez-le :
   ```bash
   chmod +x setup_rpi.sh start_all.sh
   ./setup_rpi.sh
   ```
   Ce script installe automatiquement :
   - Les paquets système Linux nécessaires (`python3-venv`, `libgl1`, etc.).
   - **Box64** et **Wine64** via les dépôts optimisés ARM64.
   - L'environnement virtuel Python `.venv` et les bibliothèques d'IA (`ultralytics`, `opencv-python-headless`, `requests`, `numpy`).

### 5.4 Lancement du Système sur le Raspberry Pi

Lancez l'ensemble (Serveur + IA Vision) en une seule commande en spécifiant l'adresse IP du robot :

```bash
./start_all.sh 10.156.149.194
```
*(Remplacez `10.156.149.194` par l'IP de votre robot).*

### 5.5 Accès au Cockpit Web depuis n'importe où
Une fois lancé sur le Raspberry Pi, ouvrez un navigateur web depuis n'importe quel ordinateur, tablette ou smartphone connecté au même réseau :
```
http://<ADRESSE_IP_DU_RASPBERRY_PI>:8080
```
Exemple : `http://192.168.1.45:8080`

---

## 6. Déploiement sur PC Windows (Guide Pas à Pas)

### 6.1 Prérequis Logiciels
1. **Python 3.10 ou 3.11** (avec la case "Add Python to PATH" cochée lors de l'installation).
2. **Git** pour Windows.
3. Les poids YOLOv8 (`yolov8n.pt` et `yolov8n-pose.pt`, déjà inclus dans le dépôt).

### 6.2 Installation sur Windows
1. Ouvrez un terminal PowerShell dans le dossier du projet :
   ```powershell
   python -m venv .venv
   .\.venv\Scripts\Activate.ps1
   pip install -r requirements.txt
   ```

### 6.3 Lancement sur Windows
Exécutez simplement le script PowerShell :
```powershell
.\start_all.ps1
```
Le script démarre automatiquement le serveur Go, ouvre votre navigateur sur `http://localhost:8080`, et démarre l'IA de vision locale.

---

## 7. Fonctionnement du Cœur Serveur Go & UnityBridge

Le fichier source [`robomaster/cmd/server/main.go`](file:///c:/Users/mev/Downloads/Robomaster%20S1/robomaster/cmd/server/main.go) compile le binaire `robomaster_server.exe`.

### Fonctionnalités principales :
1. **Connexion & Authentification au Robot** : Établit la liaison TCP/UDP sécurisée via l'API `unitybridge`.
2. **Décodage Vidéo H264 vers JPEG** : La caméra DJI envoie un flux continu de trames H264. Le callback vidéo Go décode chaque trame en image RGB/JPEG et la transmet :
   - Au flux MJPEG multi-clients sur `/video`.
   - À l'endpoint de capture instantanée sur `/snapshot`.
3. **Sécurité Totale du Châssis** : À l'initialisation et à chaque commande, la fonction `stopChassis()` est invoquée, maintenant les 4 moteurs de roues à une vitesse de 0.
4. **API REST de Contrôle** :
   - `POST /api/gimbal?pitch=P&yaw=Y` : Ajuste la vitesse de rotation de la tourelle.
   - `POST /api/fire` : Déclenche un tir infrarouge unique (impulsion LED + son).
   - `GET /api/status` : Retourne l'état de connexion et le pourcentage de batterie.
   - `POST /api/target?target=can|person|bottle|all` : Change la cible active.
   - `POST /api/autofire?enabled=true|false` : Active ou désactive le tir automatique.
   - `POST /api/standby?enabled=true|false` : Active ou désactive le balayage sentinelle 360°.
   - `GET /api/logs` : Fournit le journal d'activité en temps réel.

---

## 8. Module d'IA Vision Locale (Python & YOLOv8)

Le script [`ai_vision.py`](file:///c:/Users/mev/Downloads/Robomaster%20S1/ai_vision.py) assure l'intelligence artificielle en local :

- **Pas de latence navigateur** : Le modèle YOLOv8 nano tourne entièrement sur le processeur de la machine hôte.
- **Récupération des images** : Télécharge les images JPEG fraîches depuis `/snapshot` à ~15-20 FPS.
- **Analyse & Détection** : Détecte les objets présents dans l'image avec un seuil de confiance paramétrable (défaut : 25%).
- **Envoi des détections au Cockpit Web** : Les coordonnées normalisées des boîtes englobantes, le statut de verrouillage (`is_locked`) et les tirs (`just_fired`) sont envoyés via `POST /api/detections` pour affichage sur le HUD Canvas du navigateur.

---

## 9. Algorithme de Ciblage & Filtrage Intelligent

### 9.1 Filtrage Strict des Objets
Seules les 3 classes de cibles suivantes sont retenues, toutes les autres détections sont écartées :
- `person` (classe COCO 0)
- `bottle` (classe COCO 39)
- `cup` / `can` (classe COCO 41)

### 9.2 Règle de la Personne la Plus Proche
Si plusieurs personnes sont visibles dans le champ de la caméra, l'algorithme calcule la surface de la boîte englobante ($Surface = Largeur \times Hauteur$). La personne ayant la surface la plus grande (donc la plus proche du robot) est sélectionnée comme cible prioritaire. Les personnes en arrière-plan sont ignorées.

### 9.3 Protection du Visage & Visée Sécurisée du Torse
Grâce au modèle `yolov8n-pose.pt`, les points clés anatomiques humains (yeux, nez, épaules, hanches) sont extraits :
- Une zone d'exclusion stricte (**NO-FIRE FACE ZONE**) est définie autour de la tête.
- Le point de visée est automatiquement déplacé au centre du torse.
- Tout tir est immédiatement bloqué si le canon pointe vers la tête.

### 9.4 Suivi des Cibles et Mémoire d'Élimination (`SimpleObjectTracker`)
- Chaque objet détecté se voit attribuer un identifiant unique (`track_id`).
- Lorsqu'une cible est verrouillée et qu'un tir est émis, elle est marquée comme **éliminée (`💥 HIT`)** pendant un temps de refroidissement (cooldown de 30 secondes).
- La tourelle ne s'attarde pas sur une cible déjà touchée et bascule automatiquement sur la cible suivante non touchée.
- Dès que toutes les cibles en vue sont éliminées, la patrouille sentinelle 360° reprend.

---

## 10. Algorithme d'Asservissement Tourelle & Tir Automatique

### 10.1 Asservissement Proportionnel (PID)
Le centre de l'image est défini à $(X=320, Y=180)$ pour une résolution de $640 \times 360$.  
L'erreur horizontale ($Err_X$) et verticale ($Err_Y$) entre le centre de la cible et le centre du réticule est calculée :
$$Err_X = \frac{Center_X - 320}{320}, \quad Err_Y = \frac{Center_Y - 180}{180}$$

La vitesse angulaire envoyée aux moteurs de la tourelle est proportionnelle à cette erreur :
- Si la cible est loin du centre : vitesse élevée pour un recadrage rapide.
- Si la cible approche du centre : décélération douce pour éviter les oscillations (zone morte de $\pm 3\%$).

### 10.2 Verrouillage & Déclenchement du Tir
1. Si l'erreur combinée $|Err_X| < 0.08$ et $|Err_Y| < 0.08$ (cible dans la mire centrale), un chronomètre de verrouillage s'enclenche.
2. Si la cible reste centrée pendant au moins **350 ms**, le statut passe à `LOCKED` (le réticule devient rouge).
3. Le tir infrarouge est déclenché (`POST /api/fire`).
4. L'action est inscrite dans `robot_actions.log` avec l'angle de la tourelle et le pourcentage de batterie.
5. La cible est marquée `HIT` et la tourelle engage la cible suivante.

---

## 11. Le Cockpit Web Tactique (Port 8080)

Accessible à l'adresse `http://<IP_HOTE>:8080`, l'interface graphique offre :
- **HUD Tactique Canvas 60 FPS** : Réticule vert lors du suivi, rouge clignotant lors du verrouillage et du tir.
- **Sélecteur de Cible en Direct** : Choix entre `Canette`, `Personne`, `Bouteille` ou `Toutes les cibles` sans redémarrer le script d'IA.
- **Bouton Auto-Tir** : Activation / désactivation instantanée de l'autorisation de tir.
- **Bouton Sentinelle 360°** : Marche / Arrêt du balayage de patrouille automatique.
- **Contrôle Manuel au Clavier** : Utilisation des flèches directionnelles du clavier pour orienter la tourelle manuellement, et touche Espace pour faire feu.
- **Console de Journal d'Action** : Visualisation en direct des tirs et détections.

---

## 12. Optimisations Spécifiques au Raspberry Pi (NCNN & Performance)

Sur un Raspberry Pi 4 ou 5, plusieurs optimisations permettent d'augmenter le nombre d'images par seconde (FPS) de l'IA :

### 1. Utilisation du format NCNN (Recommandé sur Raspberry Pi 4/5)
NCNN est un framework d'inférence de réseaux neuronaux développé par Tencent, ultra-optimisé pour les processeurs ARM avec instructions NEON.  
Pour exporter le modèle YOLOv8 nano au format NCNN :
```bash
source .venv/bin/activate
yolo export model=yolov8n.pt format=ncnn
```
Cela génère un dossier `yolov8n_ncnn_model/`. YOLOv8 peut ensuite charger ce modèle directement :
```python
model = YOLO("yolov8n_ncnn_model")
```
*Gain observé : passage de ~7 FPS (PyTorch CPU) à **25-30+ FPS** sur Raspberry Pi 5 !*

### 2. Réduire la taille de résolution d'inférence (imgsz)
Par défaut, YOLOv8 traite les images en $640 \times 640$. Sur Raspberry Pi 4, spécifier une résolution de 320 ou 416 pixels divise le temps de calcul par 2 tout en conservant une excellente précision pour des personnes et canettes à moyenne distance :
```python
results = self.model(frame, imgsz=320, conf=self.conf_threshold)
```

---

## 13. Résolution des Pannes (Troubleshooting)

### Problème 1 : `Failed to connect to RoboMaster S1 (10.156.149.194)`
- **Cause** : L'adresse IP du robot a changé ou le Raspberry Pi / PC n'est pas sur le même réseau Wi-Fi.
- **Solution** :
  1. Vérifiez l'adresse IP attribuée au robot sur votre box ou via l'application DJI.
  2. Testez le ping : `ping <IP_DU_ROBOT>`.
  3. Relancez le script en passant la nouvelle adresse : `./start_all.sh <NOUVELLE_IP>`.

### Problème 2 : `The UnityBridge library is not available` ou erreur DLL sur Raspberry Pi
- **Cause** : `robomaster_server.exe` a été lancé directement sans Box64 et Wine.
- **Solution** : Exécutez `./setup_rpi.sh` pour installer Box64 et Wine64, puis utilisez toujours `./start_all.sh` qui configure l'environnement d'émulation automatiquement.

### Problème 3 : Le flux vidéo ne s'affiche pas sur le Cockpit Web (`/video` noir ou chargement infini)
- **Cause** : Le robot est en veille ou la caméra n'a pas été initialisée par le bridge.
- **Solution** :
  1. Redémarrez le robot (appui court puis long sur la batterie).
  2. Redémarrez le serveur avec `./start_all.sh`.

### Problème 4 : Pas de tir automatique alors que la cible est centrée
- **Cause** : L'Auto-Tir est désactivé dans le cockpit web ou le délai de centrage (350 ms) n'a pas été atteint.
- **Solution** :
  1. Vérifiez que le bouton `Auto-Tir : ACTIF` est activé dans le cockpit web (`http://<IP>:8080`).
  2. Vérifiez que la cible n'est pas déjà marquée `HIT` (temps de réactivation de 30 secondes).

### Problème 5 : Erreur `Le paquet « libatlas-base-dev » n'a pas de version susceptible d'être installée`
- **Cause** : `libatlas-base-dev` est un ancien paquet obsolète supprimé des versions modernes de Debian 12 (Bookworm) et 13 (Trixie).
- **Solution** : Il est remplacé par `libopenblas-dev`. Le script [`setup_rpi.sh`](file:///c:/Users/mev/Downloads/Robomaster%20S1/setup_rpi.sh) a été mis à jour pour installer automatiquement `libopenblas-dev`. Si vous effectuez une installation manuelle :
  ```bash
  sudo apt install -y python3 python3-pip python3-venv python3-dev git curl wget libgl1 libgomp1 libopenblas-dev
  ```

---
*Ce document est maintenu à jour à chaque modification du projet conformément aux directives d'`AGENTS.md`.*


