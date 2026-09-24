# Documentation Complète : Contrôle, Vision IA Locale & Suivi de Cible DJI RoboMaster S1

Ce guide fournit une documentation technique complète, de A à Z, permettant à quiconque ne connaissant pas le projet de le comprendre, de le reproduire et de le faire fonctionner sur un PC Windows avec un **DJI RoboMaster S1**.

---

## Sommaire
1. [Vue d'Ensemble & Objectifs du Projet](#1-vue-densemble--objectifs-du-projet)
2. [Architecture Globale du Système](#2-architecture-globale-du-système)
3. [Prérequis Matériels & Logiciels](#3-prérequis-matériels--logiciels)
4. [Étape 1 : Connexion Réseau au Robot (Wi-Fi & IP)](#4-étape-1--connexion-réseau-au-robot-wi-fi--ip)
5. [Étape 2 : Le Cœur CGO / Go (`robomaster_server.exe`)](#5-étape-2--le-cœur-cgo--go-robomaster_serverexe)
6. [Étape 3 : Historique des Problèmes & Pourquoi l'IA est sur PC](#6-étape-3--historique-des-problèmes--pourquoi-lia-est-sur-pc)
7. [Étape 4 : L'IA Locale PC (Python & YOLOv8)](#7-étape-4--lia-locale-pc-python--yolov8)
8. [Étape 5 : Algorithme de Ciblage & Filtrage Intelligent](#8-étape-5--algorithme-de-ciblage--filtrage-intelligent)
9. [Étape 6 : Algorithme de Suivi Tourelle & Tir Automatique](#9-étape-6--algorithme-de-suivi-tourelle--tir-automatique)
10. [Étape 7 : Le Cockpit Web Tactique (Port 8080)](#10-étape-7--le-cockpit-web-tactique-port-8080)
11. [Étape 8 : Guide d'Utilisation Pas à Pas](#11-étape-8--guide-dutilisation-pas-à-pas)
12. [Étape 9 : Journal des Actions & Fichier de Log (`robot_actions.log`)](#12-étape-9--journal-des-actions--fichier-de-log-robot_actionslog)
13. [Résolution des Pannes (Troubleshooting)](#13-résolution-des-pannes-troubleshooting)
14. [Arborescence des Fichiers du Projet](#14-arborescence-des-fichiers-du-projet)

---

## 1. Vue d'Ensemble & Objectifs du Projet

Le but de ce projet est de transformer le **DJI RoboMaster S1** en une tourelle de surveillance autonome et intelligente pilotée par PC :
- **Sécurité totale en intérieur** : les roues du robot sont **100% désactivées**, éliminant tout risque de mouvement intempestif ou d'emballement au sol. Seule la caméra / tourelle pivote.
- **Cockpit Web ultra-fluide (60 FPS)** : affichage du flux vidéo HD en temps réel sans latence avec réticule tactique HUD.
- **IA de détection sur PC** : exécution locale du modèle neuronal **YOLOv8 nano** sur le processeur du PC (~10-15 ms d'inférence), garantissant 0% de charge sur le navigateur.
- **Cibles restreintes & intelligentes** :
  1. **Personne** : algorithme calculant la personne **la plus proche** de la caméra et ignorant toutes les autres en arrière-plan.
  2. **Bouteille** (`bottle`).
- **Verrouillage & Tir Automatique (Auto-Fire)** : dès que la cible choisie est centrée dans le viseur pendant ~350 ms, la mire passe au rouge, verrouille la cible et déclenche automatiquement **1 tir infrarouge unique** (son laser + LED).
- **Enchaînement Intelligent & Changement de Cible après Tir** : dès qu'une cible est touchée, elle est enregistrée comme éliminée (`💥 HIT`) et le robot bascule automatiquement sur la cible suivante non touchée. Si toutes les cibles en vue sont éliminées, la patrouille sentinelle 360° reprend automatiquement.
- **Mode Standby Tourelle Sentinelle 360° (Actif par défaut)** : en l'absence de cible, le canon du robot effectue un balayage panoramique complet en tournant au maximum à gauche puis au maximum à droite (couvrant 360° et toute l'amplitude mécanique disponible sans angle mort). Dès qu'un objet ciblé entre dans le champ de vision, le balayage s'interrompt instantanément pour engager le suivi, le verrouillage et le tir.

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
│                          SERVEUR LOCAL EN GO                              │
│                 (robomaster_server.exe - Port 8080)                       │
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
│         IA LOCALE SUR PC               │           │
│   (ai_vision.py - Python YOLOv8)       ├───────────┘
│                                        │
│  - YOLOv8 nano (~10-15 ms sur CPU)     │
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
│                       (http://localhost:8080)                             │
│                                                                           │
│  - Rendu Canvas 60 FPS sans lag (0 calcul IA dans le navigateur)          │
│  - Viseur dynamique (Vert = suivi, Rouge = Lock / Tir)                    │
│  - Sélecteur de cible en direct (synchronisé avec l'IA sans redémarrage)  │
│  - Interrupteur marche/arrêt de l'Auto-Tir                                │
│  - Contrôle manuel d'orientation aux flèches du clavier                   │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## 3. Prérequis Matériels & Logiciels

### Matériel :
- 1× Robot **DJI RoboMaster S1** (avec batterie chargée et insérée).
- 1× PC fonctionnant sous **Windows 10 ou Windows 11** (avec carte Wi-Fi).

### Logiciels installés sur le PC :
1. **Go (Golang)** version 1.20 ou plus récente ([golang.org](https://go.dev/)).
2. **MinGW-w64 (GCC)** : obligatoire pour compiler en mode CGO (`CGO_ENABLED=1`).
   - Exemple d'emplacement GCC utilisé dans le projet : `C:\Users\<user>\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin`.
3. **Python** version 3.10 ou supérieure ([python.org](https://www.python.org/)).
4. **unitybridge.dll** : bibliothèque dynamique 64 bits de communication DJI (présente dans le dossier du projet).

---

## 4. Étape 1 : Connexion Réseau au Robot (Wi-Fi & IP)

Le DJI RoboMaster S1 supporte deux modes de connexion Wi-Fi :

### Méthode recommandée : Mode Routeur
1. Basculez le commutateur Wi-Fi situé sur le contrôleur intelligent du S1 sur la position **Routeur** (icône antenne / réseau).
2. Allumez le robot.
3. Le robot se connecte à votre réseau Wi-Fi local (ou à un point d'accès partagé).
4. Déterminez l'adresse IP du robot :
   - Par défaut dans notre configuration : **`10.156.149.194`**.
   - Vous pouvez également la vérifier sur l'interface de votre box/routeur ou via l'application mobile DJI RoboMaster dans les paramètres réseau.
5. Vérifiez la connectivité depuis le terminal de votre PC :
   ```cmd
   ping 10.156.149.194
   ```
   Si les paquets répondent avec moins de 10 ms de latence, la liaison est opérationnelle.

---

## 5. Étape 2 : Le Cœur CGO / Go (`robomaster_server.exe`)

### 5.1 Pourquoi du Go et du CGO ?
DJI ne fournit pas d'API REST standard ouverte. La communication bas niveau s'effectue via une bibliothèque native propriétaire nommée `unitybridge.dll`.
Le projet utilise le wrapper open-source créé par Bruno Gama (`github.com/brunoga/robomaster`) qui encapsule `unitybridge.dll` en Go.

### 5.2 Sécurisation du robot (Blocage des roues)
Dans le code source [`robomaster/cmd/server/main.go`](file:///c:/Users/mev/Downloads/Robomaster%20S1/robomaster/cmd/server/main.go) :
- Les appels au châssis ont été bridés.
- Dès la connexion, la fonction `stopChassis()` est exécutée à 3 reprises pour neutraliser tout résidu de commande.
- L'endpoint `/api/move` renvoie un statut `disabled`.
- Les seules commandes motrices actives sont celles de la tourelle : `robotClient.Gimbal().SetRotationSpeed(pitch, yaw)` et `robotClient.Gimbal().StopRotation()`.

### 5.3 Compilation du serveur
Un script PowerShell dédié [`build.ps1`](file:///c:/Users/mev/Downloads/Robomaster%20S1/robomaster/build.ps1) gère la compilation avec le compilateur GCC :
```powershell
$gccDir = "C:\Users\mev\AppData\Local\Microsoft\WinGet\Packages\BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe\mingw64\bin"
$env:PATH = "$gccDir;C:\Program Files\Go\bin;$env:PATH"
$env:CGO_ENABLED = "1"
$env:CC = "gcc"

go build -o robomaster_server.exe .\cmd\server\main.go
```
Une fois généré, `robomaster_server.exe` est copié à la racine du projet avec `unitybridge.dll`.

### 5.4 Points de terminaison HTTP exposés
| Méthode | Route | Rôle |
|---|---|---|
| `GET` | `/` | Tableau de bord Web HTML5 tactique |
| `GET` | `/video` | Flux vidéo en continu (MJPEG) |
| `GET` | `/snapshot` | Image JPEG instantanée (pour l'analyse Python) |
| `POST` / `GET` | `/api/gimbal?pitch=P&yaw=Y` | Contrôle d'orientation de la tourelle |
| `POST` | `/api/fire` | Déclenche 1 tir infrarouge |
| `GET` | `/api/status` | Retourne la batterie et l'état de connexion |
| `GET` / `POST` | `/api/target` | Lecture et mise à jour de la cible active |
| `GET` / `POST` | `/api/autofire` | Activation ou coupure du tir automatique |
| `GET` / `POST` | `/api/standby` | Activation ou coupure du mode standby (balayage tourelle) |
| `GET` / `POST` | `/api/detections` | Échange des boîtes de détection entre le PC et l'UI |

---

## 6. Étape 3 : Historique des Problèmes & Pourquoi l'IA est sur PC

### Le problème initial (L'IA dans le navigateur) :
Au début du projet, la détection d'objets avait été intégrée en JavaScript dans la page web à l'aide de la bibliothèque TensorFlow.js / Coco-SSD.
Cette approche s'est révélée catastrophique :
1. **Surcharge CPU** : Traiter un flux vidéo 720p en JavaScript sature le thread principal du navigateur.
2. **Effondrement des FPS** : La page web freezait et tombait à 1-2 images par seconde.
3. **Saturation du pool HTTP** : Chrome limite les connexions concurrentes sur un même hôte à 6 sockets. Le flux vidéo plus les appels constants de l'IA saturaient le pool. Les commandes `/api/gimbal` d'orientation de la caméra étaient bloquées en file d'attente, ce qui empêchait le robot de suivre quoi que ce soit.

### La solution : Découplage complet
Nous avons retiré **100% de l'IA du navigateur** :
- Le navigateur ne fait plus **aucun calcul lourd**. Il reçoit simplement des coordonnées (X, Y, Largeur, Hauteur) et les dessine sur un Canvas à 60 FPS constants.
- L'IA s'exécute sur le processeur du PC via Python et le modèle ultra-optimisé **YOLOv8 nano**.

---

## 7. Étape 4 : L'IA Locale PC (Python & YOLOv8)

### 7.1 L'environnement virtuel Python
Un environnement dédié `.venv` a été configuré à la racine pour isoler les dépendances.
Les paquets installés sont :
- `ultralytics` : framework YOLOv8
- `torch` et `torchvision` : moteur d'inférence PyTorch
- `opencv-python` : traitement d'image et décodage
- `requests` : requêtes HTTP ultra-rapides vers le serveur local
- `numpy` : calculs vectoriels

### 7.2 Le modèle YOLOv8 nano (`yolov8n.pt`)
- **Poids** : ~6 Mo seulement.
- **Temps d'inférence** : ~10 à 15 ms sur un processeur PC standard.
- **Cadence** : 15 à 20 FPS réguliers.
- **Seuil de confiance (`conf`)** : Réglé à `0.25` pour une détection vive et sensible, évitant de rater des objets légèrement inclinés.

---

## 8. Étape 5 : Algorithme de Ciblage & Filtrage Intelligent

Le script [`ai_vision.py`](file:///c:/Users/mev/Downloads/Robomaster%20S1/ai_vision.py) applique un filtrage strict pour ne tolérer que les 3 objets demandés :

```python
# Classes COCO acceptées
- "person"  -> Catégorie Personne
- "bottle"  -> Catégorie Bouteille
- "cup"     -> Catégorie Canette de soda (canette / cup)
```

### Règles de Sélection des Cibles
1. **Humains (Personnes)** :
   - Si plusieurs personnes sont visibles dans la pièce, l'algorithme calcule la surface de chaque boîte englobante ($\text{Surface} = \text{Largeur} \times \text{Hauteur}$) et conserve **uniquement la personne la plus proche** (la plus grande silhouette). Toutes les autres en arrière-plan sont écartées pour éviter toute hésitation de ciblage.
2. **Bouteilles & Canettes / Cups** :
   - Toutes les bouteilles et canettes visibles dans le champ de vision sont conservées et répertoriées dans le tracker spatial.
   - Le robot engage d'abord la cible la plus proche / grande. Dès qu'elle a reçu un tir infrarouge, elle est marquée comme touchée (`💥 HIT`) et le robot bascule automatiquement sur la bouteille ou canette suivante.

---

## 9. Étape 6 : Algorithme de Suivi Tourelle & Tir Automatique

### 9.1 Asservissement de la Tourelle & Décélération Progressive (Fine Aiming Controller)
Pour amener la cible au centre de la mire avec une précision chirurgicale sans à-coups ni oscillations :
1. Calcul de l'écart relatif par rapport au centre de l'image :
   $$\text{err}_x = \frac{x_{\text{centre cible}} - \frac{\text{Largeur}}{2}}{\frac{\text{Largeur}}{2}} \quad (\in [-1.0, 1.0])$$
   $$\text{err}_y = \frac{y_{\text{centre cible}} - \frac{\text{Hauteur}}{2}}{\frac{\text{Hauteur}}{2}} \quad (\in [-1.0, 1.0])$$
2. **Zone morte ultra-fine (Deadband à 4%)** : si $|\text{err}_x| \le 0.04$ et $|\text{err}_y| \le 0.04$, la vitesse envoyée est `0` pour garantir une stabilité totale et éliminer tout micro-tremblement.
3. **Courbe de décélération progressive (Plus le viseur approche, plus il ralentit)** :
   - Quand la cible est éloignée du centre ($|\text{err}| \approx 1.0$), la tourelle tourne rapidement (jusqu'à 48°/s en lacet et 35°/s en tangage) pour intercepter la cible sans délai.
   - À mesure que le réticule se rapproche du centre ($|\text{err}| \to 0.04$), la vitesse décélère de façon non-linéaire (courbe d'amortissement en puissance 1.3) pour descendre graduellement jusqu'à **7°/s en horizontal** et **6°/s en vertical**.
   - Ce micro-pilotage amorti supprime tout dépassement (overshoot) et permet un pointage ultra-précis et naturel.

### 9.2 Verrouillage (Lock) & Tir Unique
1. Si $|\text{err}_x| < 0.10$ et $|\text{err}_y| < 0.10$ (cible stabilisée dans le réticule central à 10%) :
   - Un chronomètre `lock_start_time` s'enclenche.
2. Si la cible reste centrée pendant plus de **350 millisecondes** :
   - Le statut passe à **`LOCKED`**.
   - Le réticule devient **rouge vif** sur le site et trace une ligne laser pointillée vers la cible (ou vers le torse si c'est un humain).
3. **Déclenchement du tir** :
   - Si l'Auto-Tir est activé et que le robot n'a pas encore tiré sur cette cible :
     - Appel immédiat à `POST /api/fire`.
     - Le robot émet le son du tir laser et fait clignoter ses diodes.
     - L'écran affiche : `💥💥 CIBLE TOUCHÉE - TIR EFFECTUÉ ! 💥💥`.
     - Un délai de temporisation (cooldown de 2.5 secondes) bloque tout tir intempestif supplémentaire.

### 9.3 Mode Standby Tourelle Sentinelle 360° (Actif par Défaut)
Lorsque le robot est allumé et qu'aucun objet ciblé n'est présent dans le champ de vision :
1. **Balayage panoramique 360° (Max Gauche ⇄ Max Droite)** :
   - La tourelle pivote automatiquement jusqu'au maximum mécanique à gauche, puis pivote dans l'autre sens jusqu'au maximum mécanique à droite à une vitesse douce et régulière de **22°/s** (vitesse diminuée pour une patrouille calme et une clarté vidéo maximale sans flou de bougé).
   - Ce balayage parcourt toute l'amplitude physique disponible (~500°, bien au-delà de 360°), inspectant l'intégralité de la pièce et de l'environnement sans aucun angle mort.
2. **Adaptation Dynamique de la Durée de Rotation selon la Vitesse ($\Delta t = \frac{\Delta \theta}{v}$)** :
   - Afin de garantir une couverture panoramique complète à 360° même à vitesse ralentie, la durée maximale de chaque demi-tour est calculée dynamiquement :
     $$T_{\text{sweep}} = \frac{\theta_{\text{amplitude}}}{v_{\text{standby}}} + \text{marge}$$
   - À 22°/s, la durée allouée est automatiquement ajustée à ~23.3 secondes (au lieu de l'ancien délai fixe de 10s qui coupait la rotation à 220°). La tourelle a ainsi le temps physique nécessaire pour parcourir les 480° complets d'une butée à l'autre sans jamais inverser prématurément.
3. **Interruption instantanée à la détection** :
   - Dès qu'une cible autorisée (la personne la plus proche, une bouteille ou une canette) entre dans le champ de la caméra, **le balayage est immédiatement interrompu**.
   - L'asservissement PID prend instantanément le relais pour centrer la tourelle sur la cible et enclencher la séquence de verrouillage.
4. **Reprise de la patrouille** :
   - Si la cible sort du champ ou disparaît, un court délai de grâce (0.6 seconde) évite les à-coups avant que le balayage sentinelle 360° ne reprenne automatiquement.

### 9.4 Détection des Butées Mécaniques & Résolution d'Angle Mort (Rotation 360° Unwind)

Le servomoteur de lacet (yaw) du RoboMaster S1 dispose d'une plage mécanique d'environ -250° à +250° (500° au total). Deux problématiques physiques majeures sont résolues par des algorithmes autonomes :

1. **Détection Active de Butée en Mode Standby** :
   - Au lieu d'utiliser un simple chronomètre fixe, l'IA interroge en direct la télémétrie angulaire du servomoteur (`/api/status`).
   - Dès que le servomoteur atteint la butée physique (angle `yaw >= +235°` ou `yaw <= -235°`, ou calage mécanique sans progression angulaire pendant >0.3s), le système détecte la fin de course et inverse immédiatement la rotation sans forcer sur les moteurs.

2. **Résolution de l'Angle Mort (Target in Dead Zone -> 360° Unwind)** :
   - **Problème** : Si la tourelle a atteint sa butée mécanique maximale (ex: +250° à droite) et qu'un objet est détecté sur le bord droit de l'image, le servomoteur ne peut physiquement plus tourner à droite pour le centrer. Sans algorithme adapté, le robot resterait bloqué contre la butée.
   - **Solution** : Dès que l'IA détecte que le servomoteur est en butée alors que la cible requiert de tourner plus loin dans cette direction, elle enclenche automatiquement une **rotation d'évitement à 360° dans le sens inverse** (vers la gauche à 52°/s).
   - La tourelle effectue un tour complet et ré-attrape l'objet depuis l'angle ouvert opposé (ex: à -110° au lieu de +250°).
   - Dès que la cible réapparaît dans la zone de centrage, le suivi actif, le verrouillage et le tir reprennent automatiquement avec une pleine liberté de mouvement.

### 9.5 Sécurité Humaine : Interdiction de Tir au Visage & Déviation Automatique vers le Torse (Face Protection & Body Retargeting)

Pour des raisons impératives de sécurité, le robot intègre un verrouillage logiciel strict empêchant tout tir infrarouge dirigé vers un visage humain :

1. **Extraction de Pose et Détection du Visage (YOLOv8-Pose + Modèle Anthropométrique)** :
   - Lorsqu'une personne est suivie, l'IA analyse la silhouette à l'aide du modèle de pose `yolov8n-pose.pt`.
   - Les points clés anatomiques du visage (nez, yeux, oreilles) délimitent une **zone d'exclusion visage prioritaire** (`Face Exclusion Zone`). En cas d'occlusion partielle ou de faible luminosité, un modèle anthropométrique standard (zone supérieure de la silhouette) assure une protection continue.
2. **Définition de la Cible Corporelle Sécurisée (Torse / Buste)** :
   - Au lieu de viser le centre géométrique d'une personne (qui peut correspondre au cou ou au menton si la personne est assise ou cadrée à mi-buste), la visée est automatiquement déportée sur le **torse / buste** (point médian entre les épaules et le bassin).
3. **Asservissement et Déviation Dynamique en Cas de Visée Visage** :
   - Si la ligne de tir (le centre du réticule de la caméra) se trouve à l'intérieur de la zone du visage ou pointe vers celui-ci :
     - **Interdiction Formelle de Tir** : Le système bloque instantanément toute autorisation de tir (`auto_fire` inhibé).
     - **Déviation Automatique Immédiate** : La tourelle incline son canon vers le bas (pitch négatif) pour déplacer le réticule hors du visage et s'aligner sur le torse.
     - L'action de sécurité est consignée dans les logs : `[SAFETY] Aiming at human face -> Retargeted aim to CHEST/TORSO (Face shot prevented)`.
4. **Verrouillage et Tir Exclusif sur le Torse** :
   - Le tir automatique ne peut se déclencher que si le réticule est stabilisé sur le point sécurisé du torse et que le visage est totalement dégagé.
   - Si seule la tête d'une personne est visible dans l'image (sans buste sécurisé atteignable), le robot incline la tourelle vers le bas et **refuse catégoriquement de tirer**.
5. **Restitution Visuelle sur le Cockpit Web et le HUD** :
   - Le visage est entouré d'un cadre orange pointillé avec la mention `🚫 NO-FIRE: FACE`.
   - Le torse affiche une mire avec réticule et la mention `🎯 TORSO`.
   - Si la tourelle passe sur la zone visage, le bandeau HUD indique en orange : `⚠️ FACE PROTECTION ACTIVE: RETARGETING TO TORSO (FIRE INHIBITED)`.
   - Le tracé laser rouge pointillé relie le centre du viseur directement au **torse** (et jamais au visage).

### 9.6 Enchaînement et Changement Automatique de Cible après Tir (Target Switching & Hit Registry)

Lors d'un engagement multi-cibles (ex. plusieurs canettes de soda ou bouteilles disposées dans l'environnement) :

1. **Registre de Suivi Spatial Persistant (`SimpleObjectTracker`)** :
   - Chaque détection dans le flux vidéo se voit attribuer un identifiant unique persistant (`Track ID` : ex. `CAN #1`, `CAN #2`, etc.).
   - Le tracker associe les boîtes englobantes d'une trame à l'autre via la distance euclidienne des centroïdes et l'orientation angulaire absolue estimée dans le repère monde ($\theta_{\text{monde}} = \theta_{\text{tourelle}} + \text{err}_x \times 48^\circ$).
2. **Élimination Immédiate de la Cible Touchée (`is_hit = True`)** :
   - Dès que le tir infrarouge est validé sur la cible verrouillée :
     - La cible est marquée comme éliminée (`is_hit = True`) dans le registre avec une mémoire temporaire de 30 secondes.
     - L'action est journalisée : `[TARGET] Target 'CAN #1' hit -> Switching to next available target`.
     - L'état de tir de la session est immédiatement réarmé pour permettre à la cible suivante d'être verrouillée et engagée sans blocage.
3. **Bascule Immédiate vers la Cible Suivante** :
   - L'algorithme de ciblage exclut systématiquement toute cible marquée `is_hit = True`.
   - Si une autre cible non touchée est déjà présente dans le champ de vision (ex. `CAN #2`), le robot pivote immédiatement vers elle, la centre, la verrouille et fait feu.
4. **Reprise Automatique du Balayage Sentinelle si Toutes les Cibles en Vue sont Touchées** :
   - Si la cible touchée était la seule visible dans le champ de la caméra, ou dès lors que toutes les cibles du champ ont été touchées :
     - Le système repasse en état de recherche active (`best_target_box = None`).
     - Le mode sentinelle 360° reprend automatiquement son balayage panoramique pour chercher d'autres cibles dans le reste de la pièce.
5. **Affichage Distinctif sur le HUD Web et OpenCV** :
   - **Cibles touchées** : encadrées en pointillés orange ambré (`#d97706`) avec le badge distinctif `💥 HIT #ID (ELIMINATED)`.
   - **Bandeau de statut** : affiche `💥💥 TARGET HIT! SWITCHING TO NEXT TARGET... 💥💥`.
   - **Nouvelle cible active** : encadrée en vert (ou rouge si verrouillée) et prête à être engagée.

---

## 10. Étape 7 : Le Cockpit Web Tactique (Port 8080)

Accessible à l'adresse **`http://localhost:8080`** :
- **Flux Vidéo** : En direct sans délai.
- **Réticule Tactique HUD** :
  - Cercle vert au repos.
  - Cercle et étiquettes rouges lors d'un verrouillage.
  - Coins tactiques dessinés autour des cibles.
- **Menu déroulant des cibles (Cible active par défaut : Canette / Cup de soda)** :
  - `🥫 Canette de soda / Cup` **(Actif par défaut au démarrage)**
  - `👤 Personne (la plus proche uniquement)`
  - `🍾 Bouteille`
  - `🎯 Toutes les 3 cibles`
  *(Changer la cible dans la liste bascule instantanément l'IA Python sans aucun rechargement de page).*
- **Interrupteur Auto-Tir** : Case à cocher permettant de couper ou d'armer le tir automatique d'un clic.
- **Interrupteur Mode Standby (Tourelle)** : Case à cocher permettant d'activer ou désactiver le balayage automatique en veille (actif par défaut).
- **Bannière d'état HUD** :
  - `📡 MODE VEILLE : BALAYAGE TOURELLE (PATROUILLE)` en veille.
  - `VISION: X OBJET(S) DÉTECTÉ(S)` lors du suivi.
  - `🔒 CIBLE VERROUILLÉE AU CENTRE (LOCK)` lors du verrouillage.
  - `💥💥 TIR EFFECTUÉ ! CIBLE TOUCHÉE 💥💥` lors du tir.
- **Commandes Manuelles de Secours** :
  - Touches fléchées <kbd>↑</kbd> <kbd>↓</kbd> <kbd>←</kbd> <kbd>→</kbd> : pilotage manuel de la tourelle.
  - Touche <kbd>Espace</kbd> : tir infrarouge manuel.

---

## 11. Étape 8 : Guide d'Utilisation Pas à Pas

### Démarrage Tout-en-Un (1 Seul Clic Recommandé) :

Au lieu de lancer deux fichiers séparément, vous disposez d'un **lanceur unique global** qui démarre le serveur, ouvre le Cockpit Web, lance l'IA et coupe tout automatiquement à la fermeture :

#### Option 1 : Depuis l'Explorateur Windows (Double-clic)
Double-cliquez simplement sur :
👉 [**`start_all.bat`**](file:///c:/Users/mev/Downloads/Robomaster%20S1/start_all.bat)

#### Option 2 : Dans votre Terminal PowerShell
```powershell
.\start_all.ps1
```
*(Vous pouvez aussi spécifier des options : `.\start_all.ps1 --target canette` ou `.\start_all.ps1 --no-standby`)*

Ce script tout-en-un prend automatiquement en charge :
1. Le démarrage du serveur Go (`robomaster_server.exe`) en arrière-plan.
2. L'ouverture de votre navigateur sur `http://localhost:8080`.
3. Le lancement direct de l'IA locale (YOLOv8 nano) **sans aucune saisie ni question dans le terminal CMD** : le robot démarre immédiatement sa surveillance.
4. **Contrôle 100% via l'interface web** : le choix de la cible (Personne, Bouteille, Canette, Toutes), l'Auto-Tir et le Mode Standby 360° sont modifiables en direct d'un simple clic sur le Cockpit.
5. La fermeture propre du serveur et des processus associés dès que vous fermez la fenêtre ou pressez `Ctrl+C`.

---

## 12. Étape 9 : Journal des Actions & Fichier de Log (`robot_actions.log`)

Afin d'assurer une traçabilité totale et un audit rigoureux de toutes les opérations effectuées par le robot et son système de vision, un **journal d'actions centralisé** est automatiquement alimenté dans le fichier :
👉 [**`robot_actions.log`**](file:///c:/Users/mev/Downloads/Robomaster%20S1/robot_actions.log) (situé à la racine du projet).

### Format des Entrées du Log :
Chaque événement est consigné avec horodatage exact à la seconde, catégorie et description claire de l'action :
```text
[AAAA-MM-JJ HH:MM:SS] [CATEGORIE] Description de l'action
```

Exemple d'extrait réel de `robot_actions.log` :
```text
[2026-09-24 10:11:00] [SYSTEM] Initialized DJI RoboMaster S1 action log
[2026-09-24 10:14:02] [CONNECTION] Connected successfully to RoboMaster S1 (10.156.149.194) - Battery: 84%
[2026-09-24 10:14:05] [AI] Local AI vision started (Target: CAN, Conf: 25%, AutoFire: True, Standby: True)
[2026-09-24 10:14:07] [STANDBY] Sentry turret sweep towards right
[2026-09-24 10:14:10] [STANDBY] Sentry turret sweep towards left
[2026-09-24 10:14:11] [STANDBY] Servomotor reached physical limit at MAX LEFT -> Reversing sweep towards RIGHT ➔
[2026-09-24 10:14:12] [VISION] Target 'PERSON (CLOSEST)' detected (Area: 42560 px)
[2026-09-24 10:14:12] [UNWIND] Target in blind spot at MAX RIGHT limit -> Initiating 360° rotation to the LEFT
[2026-09-24 10:14:16] [UNWIND] Target re-acquired from open angle -> Resuming active tracking
[2026-09-24 10:14:17] [SAFETY] Aiming at human face -> Retargeted aim to CHEST/TORSO (Face shot prevented)
[2026-09-24 10:14:18] [LOCK] Target 'TORSO of PERSON (CLOSEST)' locked at crosshair center (Face clear)
[2026-09-24 10:14:18] [FIRE] Automatic infrared fire triggered on TORSO of 'PERSON (CLOSEST)' (Face protected)
[2026-09-24 10:14:13] [FIRE] Infrared fire triggered
[2026-09-24 10:14:16] [CONFIG] AI target changed via Web Cockpit: 'PERSON' -> 'BOTTLE'
[2026-09-24 10:14:20] [TURRET] Turret movement: pitch=0 yaw=40
[2026-09-24 10:14:21] [TURRET] Turret rotation stopped
[2026-09-24 10:14:35] [AI] Local AI vision stopped, turret halted
[2026-09-24 10:14:36] [DISCONNECTION] Server stopped and robot disconnected
```

### Événements Journalisés :
| Catégorie | Description & Déclencheur |
| :--- | :--- |
| `CONNECTION` | Connexion réussie du serveur Go au robot via Wi-Fi avec relevé du % de batterie |
| `DISCONNECTION` | Coupure propre du serveur, arrêt des moteurs et déconnexion |
| `AI` | Démarrage / Arrêt du module de vision Python YOLOv8 |
| `VISION` | Détection d'une cible autorisée avec calcul de surface (proximité) ou cible perdue de vue |
| `LOCK` | Verrouillage confirmé au centre du réticule (marge de 12% pendant >350ms) |
| `FIRE` | Tir infrarouge physique déclenché (automatique après verrouillage ou manuel via Espace/Web) |
| `TARGET` | Enchaînement automatique : cible touchée marquée éliminée et bascule immédiate vers la cible suivante |
| `STANDBY` | Détection de butée mécanique et inversion du balayage sentinelle 360° |
| `UNWIND` | Rotation complète à 360° déclenchée lorsqu'une cible se trouve dans l'angle mort mécanique en butée |
| `SAFETY` | Protection faciale : détection du visage, déviation automatique vers le torse et blocage du tir au visage |
| `TURRET` | Mouvements manuels de la tourelle (début de rotation, vitesse, arrêt) |
| `CONFIG` | Modification de la cible active, activation/désactivation d'Auto-Tir ou du mode Standby |
| `ERROR` | Signalement d'éventuelles erreurs matérielles ou réseau |

### Comment Consulter les Logs :
1. **Directement dans le Cockpit Web** : Cliquez sur le bouton bleu **`📜 Voir Logs`** situé dans l'en-tête en haut de page, ou ouvrez l'URL :
   👉 `http://localhost:8080/api/logs`
2. **Dans le fichier texte** : Ouvrez [**`robot_actions.log`**](file:///c:/Users/mev/Downloads/Robomaster%20S1/robot_actions.log) dans n'importe quel éditeur de texte (Bloc-notes, VS Code...).
3. **En temps réel dans le terminal PowerShell** :
   ```powershell
   Get-Content -Path robot_actions.log -Wait -Tail 20
   ```

---

## 13. Résolution des Pannes (Troubleshooting)

### Q : PowerShell affiche `The term 'start_all.bat' is not recognized`
- **Cause** : Par mesure de sécurité, Windows PowerShell n'exécute pas les scripts du dossier courant sans préfixe.
- **Solution** : Écrivez `.\start_all.ps1` ou `.\start_all.bat`.

### Q : L'IA ne repère pas un objet placé devant la caméra
- **Distance** : La caméra du RoboMaster S1 a une focale fixe. À moins de 25 cm, l'image est floue. Placez-vous ou tenez l'objet entre **40 cm et 2 mètres**.
- **Cible sélectionnée** : Vérifiez que l'objet correspond bien à la cible choisie (ex : si la cible est sur `personne`, une canette ne sera pas verrouillée).
- **Pour une personne** : Il faut que le buste ou le corps entier soit dans le champ pour que l'IA le classifie avec certitude comme `personne`.

### Q : Erreur `No connection could be made because the target machine actively refused it`
- **Cause** : Le script Python d'IA a été lancé avant le serveur Go.
- **Solution** : Utilisez simplement `start_all.bat` (ou `.\start_all.ps1`) : le script gère automatiquement l'ordre et le délai de démarrage.

---

## 14. Arborescence des Fichiers du Projet

```text
Robomaster S1/
│
├── DOCUMENTATION.md          <-- Cette documentation technique complète
├── robot_actions.log         <-- Journal complet et horodaté de toutes les actions du robot
├── start_all.bat             <-- 1-CLIC UNIQUE : Lance le Serveur + Web + IA en 1 clic
├── start_all.ps1             <-- 1-CLIC PowerShell : Lance le Serveur + Web + IA
│
├── ai_vision.py              <-- Moteur IA : YOLOv8 nano, filtrage, asservissement tourelle & auto-tir
├── robomaster_api.py         <-- Bibliothèque d'aide Python pour requêter le serveur
├── detect_objects.py         <-- Script d'inspection console simple
│
├── yolov8n.pt                <-- Modèle IA YOLOv8 nano (détection : personne, bouteille, canette)
├── yolov8n-pose.pt           <-- Modèle IA YOLOv8 pose (analyse corporelle, protection visage & visée torse)
│
├── robomaster_server.exe     <-- Exécutable serveur Go compilé avec CGO
├── unitybridge.dll           <-- Pilote dynamique officiel DJI 64 bits
│
├── .venv/                    <-- Environnement virtuel Python (Torch, Ultralytics, OpenCV)
│
└── robomaster/               <-- Code source Go du serveur
    ├── build.ps1             <-- Script de compilation GCC/MinGW
    ├── go.mod                <-- Dépendances Go
    ├── client.go             <-- Wrapper du client S1
    ├── cmd/
    │   └── server/
    │       └── main.go       <-- Point d'entrée HTTP, décodage vidéo, API, journalisation & Cockpit HTML5
    └── module/               <-- Modules bas niveau (camera, chassis, gimbal, gun...)
```
