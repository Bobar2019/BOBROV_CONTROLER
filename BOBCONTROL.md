# BOB-CONTROL - Contrôleur Temps Réel ROV

**Version :** 1.0.1  
**Plateforme :** ESP32-S3 (PlatformIO / Arduino Core / FreeRTOS)  
**Auteur :** Didier Dero  
**Projet associé :** BOB-ROV (Cockpit Raspberry Pi 5)  
**Date :** Septembre 2026  

---

## 🎯 Philosophie & Rôle Système

**BOB-CONTROL** est le micrologiciel temps réel dédié à la gestion basse couche du sous-marin **BOB-ROV**. Embarqué sur un **ESP32-S3**, il isole les fonctions critiques de pilotage matériel et d'acquisition de données du système d'exploitation hôte (Raspberry Pi 5).

Il remplit un triple rôle :
1. **Acquisition & Fusion Capteurs :** Lecture haute fréquence des deux bus I2C matériels (centrale inertielle BNO085 et wattmètres INA226 sur le bus n°1 ; pression/profondeur MS5837 sur le bus n°2, partagé avec le PCA9685).
2. **Contrôleur de Vol PID :** Stabilisation automatique roulis/tangage/cap/profondeur avec mixage différentiel sur les 8 propulseurs.
3. **Génération PWM Matérielle :** Contrôle autonome de 16 canaux PWM indépendants via PCA9685 (8 propulseurs, servos pan/tilt, pince, gradateurs d'éclairage), avec serveur Web/WebSocket autonome et portail captif pour la configuration et le banc de test.

---

## 🏗️ Architecture Matérielle & Adressage

### 1. Périphériques I2C — Deux Bus Matériels

L'ESP32-S3 possède deux contrôleurs I2C indépendants, exploités pour séparer les périphériques :

| Composant | Rôle | Adresse I2C | Bus I2C | Description technique |
| :--- | :--- | :--- | :--- | :--- |
| **BNO085** | IMU 9-DOF | `0x4A` | n°1 (GPIO 10/11) | Fusion inertielle embarquée (protocole SH-2 via Adafruit BNO08x) : Rotation Vector 9-DOF (cap magnétique) + Game Rotation Vector (secours) + gyroscope calibré @ 50 Hz. Monté tourné de 90° (X capteur vers la droite du ROV) : quaternion **et** gyroscope remappés à la source dans le repère véhicule via `BNO085_MOUNT_YAW_DEG` (−90°) |
| **INA226 #1** | Puissance RPi 5 | `0x41` | n°1 (GPIO 10/11) | Tension, courant et puissance consommée par le Pi 5 |
| **INA226 #2** | Puissance Aux/LEDs | `0x44` | n°1 (GPIO 10/11) | Mesure de ligne d'éclairage et accessoires 12V |
| **INA226 #3** | Puissance Moteurs | `0x45` | n°1 (GPIO 10/11) | Mesure de ligne de puissance batterie/propulsion principale |
| **MS5803-30BA / MS5837** | Pression & Profondeur | `0x76` | **n°2 (GPIO 6/7)** | Capteur piézorésistif haute résolution étanche, compensation 24-bit (coefficients C1-C6) |
| **PCA9685** | Contrôleur PWM 16 Ch | `0x40` | **n°2 (GPIO 6/7)** | Générateur PWM 12 bits autonome (VCC 3.3V, V+ 5V/Servo) |

> **Note — pourquoi deux bus :** le MS5837-30BA branché sur le même bus que le BNO085 perturbait ses lectures SHTP (tempête de NACK, horizon figé). Le firmware répartit donc les périphériques sur les deux contrôleurs I2C : bus n°1 `Wire` (GPIO 10/11 par défaut, configurables en NVS — BNO085 + INA226, propriété exclusive de la tâche capteurs) et bus n°2 `Wire1` (GPIO 6/7 par défaut, configurables en NVS — MS5837 + PCA9685, partagé entre la tâche capteurs et la tâche de contrôle). Chaque bus possède ses propres résistances de pull-up (4,7 kΩ vers 3V3).

### 2. Liaison Série RPi 5 — Deux Modes Configurables

Le protocole binaire RPi 5 ↔ ESP32-S3 peut emprunter **deux interfaces physiques** au choix, sélectionnable via l'onglet Paramètres de l'interface Web et persisté en NVS :

| Mode | Interface | Broches / Port | Côté RPi 5 | Description |
| :--- | :--- | :--- | :--- | :--- |
| **USB-CDC Natif** | `Serial` (USB) | Port USB-C de l'ESP32-S3 | `/dev/ttyACM0` | Protocole binaire sur le port USB natif. Le debug moniteur partage le même port. |
| **UART Matériel** | `Serial1` (UART1) | RX: **GPIO 1**, TX: **GPIO 2** | UART GPIO (921 600 bauds) | Protocole binaire dédié sur UART matériel. USB-CDC reste exclusivement réservé au debug. |

> **Note :** Par défaut, le mode **USB-CDC** est actif. Le choix est mémorisé en NVS (clé `comm_iface`) et rechargé à chaque démarrage. Un **redémarrage** est nécessaire après changement via l'interface Web.

#### Broches UART (mode UART matériel uniquement)

| Signal | GPIO | Direction | Description |
| :--- | :--- | :--- | :--- |
| **UART1 RX** | GPIO 1 | ESP32 ← RPi 5 | Réception trame descendante |
| **UART1 TX** | GPIO 2 | ESP32 → RPi 5 | Émission trame montante |

### 3. Affectation des Canaux PWM (PCA9685)

* **Canaux 0 à 7 :** 8 × ESC Propulseurs ($1000\,\mu\text{s}$ à $2000\,\mu\text{s}$, neutre à $1500\,\mu\text{s}$)
* **Canaux 8 & 9 :** Servomoteurs Pan & Tilt Caméra ($50\text{ Hz}$)
* **Canaux 10 & 11 :** Outil / Pince & Rotation — **Bidirectionnel** par défaut (neutre $1500\,\mu\text{s}$)
* **Canaux 12 & 13 :** Gradateurs MOSFET pour projecteurs LED — **Unidirectionnel** par défaut ($1000\,\mu\text{s} = 0\,\% \rightarrow 2000\,\mu\text{s} = 100\,\%$)
* **Canaux 14 & 15 :** Canaux auxiliaires / Extensions — **Bidirectionnel** par défaut

> **Typage configurable (canaux 10 à 15) :** chaque canal 10–15 est typable individuellement en **Bidirectionnel** (neutre $1500\,\mu\text{s}$, plage $1000-2000\,\mu\text{s}$) ou **Unidirectionnel / pleine échelle** ($1000\,\mu\text{s} = 0\,\%$, $2000\,\mu\text{s} = 100\,\%$) depuis l'onglet Paramètres — persistance NVS (clé `pwm_types` : masque 6 bits, bit n ↔ canal 10+n, 1 = bidirectionnel ; défaut `0x33`), appliqué **à chaud** sans redémarrage. Le typage ne modifie pas la plage des consignes (toujours bornées à 1000–2000 µs) : il détermine la **position de sécurité** appliquée par le failsafe (1500 µs en bidirectionnel, 1000 µs en unidirectionnel) et le rendu dynamique des barregraphes du **PWM Monitor**.

### 4. Sorties Tout-ou-Rien — 4 × GPIO ON/OFF

Quatre sorties numériques auxiliaires pilotées directement par l'ESP32-S3 (relais, éclairage tout-ou-rien, accessoires) :

| Sortie | GPIO par défaut | Commande (bit de `gpio_cmd`) | État (bit de `gpio_state`) |
| :--- | :--- | :--- | :--- |
| 1 | GPIO 15 | bit 0 | bit 0 |
| 2 | GPIO 16 | bit 1 | bit 1 |
| 3 | GPIO 17 | bit 2 | bit 2 |
| 4 | GPIO 18 | bit 3 | bit 3 |

* **Assignation configurable :** les numéros de GPIO sont modifiables dans l'onglet Paramètres et persistés en NVS (clés `gpio_p1`..`gpio_p4`) ; ils ne sont appliqués qu'**au redémarrage** (comme les broches I2C). Validation : broches 0–48 hors GPIO 19/20 (USB), hors broches I2C actives (deux bus) et UART1, sans doublon.
* **Initialisation :** au démarrage, les 4 broches sont configurées en sortie à l'état **BAS (OFF)** ; elles restent pilotables **même désarmé** (sorties auxiliaires).
* **Sécurité :** le déclenchement du watchdog série (perte du flux RPi 5) et l'**E-Stop** forcent les 4 sorties à **OFF** — l'état réel est relu physiquement et remonté dans `gpio_state`.

---

## 📡 Protocole Série Binaire RPi 5 ↔ ESP32-S3

La communication s'effectue à **921 600 bauds** sur l'interface sélectionnée (USB-CDC natif ou UART matériel GPIO 1/2). Toutes les trames sont à taille fixe avec validation par somme de contrôle CRC16-CCITT.

### 1. Trame Descendante : RPi 5 → ESP32-S3 (40 octets @ 20-50 Hz)

```
[0xAA][0x55][TYPE][MODE][ARM_STATE][PWM_0 ... PWM_15][GPIO_CMD][CRC16_HIGH][CRC16_LOW]
```

* **Header :** `0xAA`, `0x55` (2 octets)
* **Type :** `0x01` (1 octet)
* **Mode :** `uint8_t` (`0` = Passif, `1` = AUTO Roulis et Tangage, `2` = Auto-Full)
* **Arm State :** `uint8_t` (`0` = Disarmed, `1` = Armed, `2` = Emergency Stop)
* **PWM Channels (0–15) :** `uint16_t[16]` (32 octets, offsets 5–36, consigne en $\mu\text{s}$)
* **GPIO Command :** `uint8_t` (offset 37, masque 4 bits des sorties ON/OFF — bit 0 = sortie 1 … bit 3 = sortie 4, `1` = ON, `0` = OFF ; bits 4–7 ignorés)
* **CRC16 :** `uint16_t` (2 octets aux offsets [38] fort / [39] faible, polynôme 0x1021, init 0xFFFF — **stocké gros-boutiste** : octet fort puis faible, calculé sur les octets `[2..37]`)

### 2. Trame Montante : ESP32-S3 → RPi 5 (73 octets @ 100 Hz)

```
[0x55][0xAA][TYPE][STATUS][QUAT_W,X,Y,Z][GYRO_X,Y,Z][PRESS,TEMP][INA_1,2,3][PWM_ACT_0...15][GPIO_STATE][CRC16]
```

* **Header :** `0x55`, `0xAA` (2 octets)
* **Type :** `0x02` (1 octet)
* **Status Flags :** `uint8_t` (Bit 0: réservé, Bit 1: Liaison établie — au moins une trame valide reçue depuis le boot, Bit 2: Autopilote actif — réservé, jamais émis, Bit 3: Watchdog déclenché, Bit 4: Sorties physiques neutralisées / Dry-Run)
* **Quaternions IMU :** `int16_t[4]` (8 octets, $W, X, Y, Z$ scalés $\times 10\,000$, repère véhicule — montage BNO085 remappé à la source)
* **Gyroscope :** `int16_t[3]` (6 octets, $\omega_x, \omega_y, \omega_z$ scalés $\times 100$, repère véhicule)
* **Pression & Température :** `int32_t[2]` (8 octets, $0.1\text{ mbar}$ et $0.01\,^\circ\text{C}$)
* **Télémétrie Puissance :** `uint16_t[6]` (12 octets, $V$ et $I$ pour Pi, Aux et Propulsion en $mV$ / $mA$)
* **PWM Effectifs :** `uint16_t[16]` (32 octets, offsets 38–69, impulsions réelles envoyées aux actionneurs)
* **GPIO State :** `uint8_t` (offset 70, état **réel relu** des 4 sorties ON/OFF — bit 0 = sortie 1 … bit 3 = sortie 4, `1` = ON)
* **CRC16 :** `uint16_t` (2 octets aux offsets [71] fort / [72] faible, **gros-boutiste**, calculé sur les octets `[2..70]`)

### 3. Watchdog Failsafe

Un watchdog logiciel surveille la réception de trames descendantes. Si aucune trame valide n'est reçue pendant le **timeout configurable** (défaut : 500 ms), chaque canal PWM est automatiquement forcé à sa **position de sécurité** ($1500\,\mu\text{s}$ en bidirectionnel — défaut canaux 0–11, 14–15 ; $1000\,\mu\text{s}$ / $0\,\%$ en unidirectionnel — défaut canaux 12–13, voir « Typage configurable ») et les **4 sorties ON/OFF sont forcées à OFF**.

> **📄 Documentation détaillée pour le développement côté RPi 5 :** voir [PROTOCOLE_RPI5.md](PROTOCOLE_RPI5.md) — description octet par octet des deux trames et des masques de sorties ON/OFF (`gpio_cmd` / `gpio_state`), sémantique exacte des bits de statut, comportements watchdog / armement / maître / dry-run, exemple Python prêt à l'emploi et vecteurs de test.

#### Watchdog IMU — Récupération BNO085 en 3 niveaux

Indépendamment du watchdog série, un **watchdog dédié surveille le flux de quaternions du BNO085** (Rotation Vector ou Game Rotation Vector — le gyroscope ne compte pas). Constat opérationnel : en plongée ou après une longue période d'inactivité, la communication avec le capteur (seul sur le bus I2C n°1) peut se figer définitivement. Si aucun quaternion n'est reçu pendant **1 s** (`BNO085_STALL_TIMEOUT_MS`), la récupération escalade automatiquement :

| Niveau | Déclenchement | Action | Propulsion |
| :--- | :--- | :--- | :--- |
| **1 — Reprise douce SH-2** | 1er échec (stall 1 s sans quaternion) | Ré-initialisation logicielle du capteur (réécriture des rapports SH-2) | **Forcée au neutre** pendant la récupération |
| **2 — Reset matériel du bus I2C** | Échec du niveau 1, ou blocage persistant du contrôleur I2C (`ESP_ERR_INVALID_STATE`) | Destruction de l'instance `Adafruit_BNO08x`, `Wire.end()`, **9 impulsions SCL + condition STOP** (bit-bang), réinitialisation complète de `Wire` (400 kHz, timeout 50 ms), puis recréation de l'instance et réinit de tous les capteurs | **Forcée au neutre** |
| **3 — Ultime secours : redémarrage contrôlé** | **3 tentatives de niveau 2** restées sans effet (`BNO085_REBOOT_ATTEMPTS`) **ou 8 s** sans aucun quaternion (`BNO085_REBOOT_TIMEOUT_MS`) | 1) tous les canaux PWM à leur position de sécurité (neutre $1500\,\mu\text{s}$, ou $1000\,\mu\text{s}$ / $0\,\%$ pour les canaux unidirectionnels), 2) log explicite sur le moniteur série, 3) `esp_restart()` — redémarrage automatique de l'ESP32 | **Position de sécurité physiquement maintenue** pendant tout le reboot (voir ci-dessous) |

**Garantie de maintien au neutre pendant la récupération / le reboot :**

1. **Pendant les niveaux 1 et 2** — la tâche de contrôle (100 Hz) verrouille les canaux 0-7 (propulseurs) à $1500\,\mu\text{s}$ **à chaque cycle** tant que le flux de quaternions est mort (`isBNO085Stalled()`) : une consigne RPi 5 réappliquée entre deux tentatives ne peut jamais repartir avec une attitude inconnue. Les servos/auxiliaires (8-15) ne sont pas concernés.
2. **Au niveau 3** — la neutralisation est doublée d'une écriture physique inconditionnelle sur les 16 canaux (même en Dry-Run) : le **PCA9685 est un générateur autonome qui conserve ses registres pendant le redémarrage de l'ESP32** — les propulseurs restent au neutre tout le reboot ; au redémarrage, le firmware repart en mode Dry-Run et réapplique le neutre.

> **Garde-fou anti-boucle :** le niveau 3 ne se déclenche que si le capteur **a déjà fonctionné depuis le boot** (au moins un quaternion reçu). Un capteur jamais fonctionnel dès la mise sous tension n'est pas redémarré en boucle : il reste en **mode espacé** (polling SHTP coupé, badge rouge + message Web, une tentative complète toutes les 30 s). Ce comportement garantit la récupération automatique du cas opérationnel réel (capteur qui gèle en plongée) sans risquer une boucle de redémarrages sur un capteur défaillant au boot.

---

## 🌐 Interface Web Embarquée & Portail Captif

L'ESP32-S3 héberge une application Web autonome stockée dans la mémoire flash **LittleFS**. Le design suit la charte graphique **BOB CNC** (thème sombre, polices Michroma + Source Code Pro, accents cyan/magenta).

### Portail Captif Automatique

Le point d'accès Wi-Fi (`BOB-CONTROL_AP`, mot de passe : `bobrov2026`) inclut un serveur DNS captif qui redirige **toutes** les requêtes DNS vers `192.168.4.1`. Des handlers HTTP spécifiques interceptent les URLs de détection de portail captif pour chaque OS :

| OS | URL de détection interceptée |
| :--- | :--- |
| Android / ChromeOS | `/generate_204`, `/gen_204`, `/hotspot-detect.html` |
| iOS / macOS | `/library/test/success.html` |
| Windows | `/connecttest.txt`, `/redirect` |
| Firefox | `/canonical.html` |
| Kindle / Fire OS | `/kindle-wifi/wifistub.html` |

La page s'ouvre automatiquement dès la connexion au réseau Wi-Fi.

### Tableau de Bord — Navigation par Tuiles Réorganisables

L'interface principale est un **tableau de bord** composé de tuiles cliquables disposées en grille 2×N. Chaque tuile représente une fonction du système. L'utilisateur peut :

1. **Cliquer sur une tuile** → affiche la section correspondante en pleine page avec un bouton retour (↩).
2. **Cliquer « Réorganiser »** ou **appui long (500 ms) sur une tuile** → active le mode édition : toutes les tuiles se mettent à trembler (jiggle iOS ±1,5°, **désynchronisé par tuile** via data-tile-id), et la tuile sous le doigt se **soulève immédiatement** — elle suit le doigt sans attendre un nouvel appui.
3. **Glisser-déposer les tuiles** → réordonne en temps réel, la tuile survolée fait place dynamiquement (Pointer Events unifiés souris/tactile, **repli touchstart/touchmove/touchend** pour les navigateurs anciens ; jamais l’attribut `draggable` HTML5, inopérant sur iOS Safari). La position est **sauvegardée dans `localStorage` à chaque dépôt** (ordre propre à l’appareil).
4. **Cliquer « Terminer »** ou **taper en dehors des tuiles** → quitte le mode édition.

### Fonctions du Tableau de Bord (6 Sections)

| Tuile | Contenu |
| :--- | :--- |
| **Wi-Fi** | **Connexion actuelle** temps réel : SSID, adresse IP, passerelle (DHCP), masque, RSSI, MAC. Infos point d'accès local (SSID, IP, clients). Scan des réseaux environnants. Connexion STA non-bloquante (sans tuer le point d'accès). |
| **PWM Monitor** | 16 barregraphes temps réel (WebSocket). 8 jauges moteurs bidirectionnelles centrées à $1500\,\mu\text{s}$, servos, gradateurs, auxiliaires. **Rendu dynamique des canaux 10–15 selon leur typage** : Bidirectionnel → curseur centré sur le neutre ($1500\,\mu\text{s}$) ; Unidirectionnel → jauge pleine échelle $0 - 100\,\%$ avec dégradé de couleur progressif vert → orange → rouge. **Commutateur Sorties Matérielles** (Mode Témoin / Dry-Run) : badge d'avertissement ambre/vert. |
| **Télémétrie** | **Cadre ROV 3D** : modèle 3D du ROV (`bob_rov_3D.glb`, LittleFS `/3D/`) animé en temps réel par l'IMU (roulis/tangage/cap) — three.js r128 servi **localement** (`/vendor/three.min.js`, aucun CDN : le ROV diffuse son AP sans Internet) et parseur GLB minimal embarqué (couleurs plates, sans texture) ; orbite/zoom à la souris, double-clic = vue initiale, grille polaire et flèche du Nord en référence de cap, cases « Filaire » / « Vert » (wireframe, lignes teintées vert) ; réalignement du modèle à 90° (R3D_MODEL_YAW_DEG) car l’avant exporté d’Onshape ne suit pas la convention glTF (-Z). **Cap magnétique BNO085** : le Rotation Vector 9-DOF (gyro + accéléro + magnétomètre) est la source principale du cap — cap absolu, le Game Rotation Vector relatif restant en secours automatique si le flux s'interrompt ; état de la boussole affiché en continu (source + calibration 0–3, auto-calibration en déplaçant le ROV en « 8 »). **Bouton « Régler Nord »** : tare du cap appliquée à la source — télémétrie, HSI, modèle 3D et PID de cap partagent le même référentiel ; ajustage fin de l'écart résiduel en cap magnétique (une fois suffit), référence obligatoire après chaque boot en secours relatif ; à faire en mode PASSIF. Assiette Euler (Roll/Pitch/Yaw), profondeur, pression, température. 3 wattmètres instantanés. **Sélecteur de Mode de Pilotage** : 3 boutons radio (PASSIF / AUTO ROULIS ET TANGAGE / AUTO FULL) avec badge Maître (RPi 5 vert / Manuel ESP32 orange). Instruments aviation : horizon artificiel et HSI (Heading Situation Indicator) avec lissage LERP. Diagnostic capteurs (LED BNO085, MS5803, INA226 ×3, PCA9685). |
| **Banc de Test** | 16 curseurs de commande manuelle **adaptés au typage configuré** (µs 1000–2000 en bidirectionnel ; **0–100 %** avec conversion automatique en µs en unidirectionnel) + **4 sorties GPIO ON/OFF**, avec verrou de sécurité matériel. Chaque sortie affiche son **nom personnalisé** (Paramètres → NVS). Bouton « Tout en position de sécurité » appliquant **par canal** le neutre adapté (1500 µs / 0 %). |
| **Paramètres** | Sélection liaison série RPi 5 (USB-CDC / UART GPIO 1/2). **Configuration Bus I2C n°1 / n°2** : broches SDA/SCL avec sauvegarde NVS et redémarrage automatique du contrôleur après 3 s — même comportement pour les deux bus (broches appliquées au boot ; le bus n°2 est partagé avec les sorties PWM). **Scanner I2C** : scan des deux bus (0x01–0x7F, colonne Bus) avec identification des périphériques. **Typage des canaux PWM 10–15** : 6 sélecteurs Bidirectionnel / Unidirectionnel, appliqués à chaud et persistés en NVS. **Sorties tout-ou-rien** : assignation des 4 GPIO ON/OFF (GPIO 15/16/17/18 par défaut) avec validation (doublons et broches réservées refusés — USB, I2C actifs, UART1), persistance NVS et redémarrage automatique après 3 s ; badges d'état ON/OFF en temps réel et **boutons de test individuels** (forçage direct via `POST /api/gpio/test`, écrasé par le watchdog, l'E-Stop ou une trame RPi 5 maître). **Mise à jour OTA** : téléversement sans câble du micrologiciel (firmware.bin) ou des fichiers Web (littlefs.bin) avec progression, neutralisation de la propulsion et redémarrage automatique. PID (Roll, Pitch, Yaw, Profondeur) : Kp, Ki, Kd. Timeout watchdog série réglable. **Calibration Altimètre (QNH)** : interrupteur d'activation (persisté NVS) et bouton « Actualiser le QNH maintenant » avec spinner (Open-Meteo — Chamoson, VS) — ne corrige que l'altitude barométrique hors de l'eau (voir Notes Techniques). |
| **Câblage** | Schéma visuel ESP32-S3 (`esp32s3.png`). Liaison RPi 5 dynamique selon le mode actif (USB-CDC ou UART GPIO 1/2) avec diagrammes ASCII et tables de correspondance. Tableaux complets des deux bus I2C : broches SDA/SCL (bus n°1 GPIO 10/11, bus n°2 GPIO 6/7), adresses et fonctions de chaque module (IMU, wattmètres, baromètre, PCA9685). |

### Connexion Wi-Fi Externe (Mode STA)

Le firmware fonctionne en mode **WIFI_AP_STA** (point d'accès + station simultanés). La connexion à un réseau Wi-Fi externe utilise `WiFi.begin()` **sans** `WiFi.disconnect()` pour ne pas interrompre le point d'accès. Un callback d'événements Wi-Fi (`ARDUINO_EVENT_WIFI_STA_GOT_IP`) suit l'état de connexion de manière asynchrone.

---

## ⚙️ Configuration PlatformIO

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
board_build.partitions = bob_control_8mb.csv
board_build.filesystem = littlefs
build_flags = 
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1

lib_deps = 
    mathieucarbou/ESPAsyncWebServer @ ^3.6.0
    mathieucarbou/AsyncTCP @ ^3.3.2
    bblanchon/ArduinoJson @ ^7.0.0
    adafruit/Adafruit PWM Servo Driver Library @ ^3.0.1
    adafruit/Adafruit BNO08x @ ^1.2.5
    robtillaart/INA226 @ ^0.6.0
```

> **Note :** Les bibliothèques `ESPAsyncWebServer` et `AsyncTCP` utilisent le fork maintenu par `mathieucarbou` (compatible ESP32 Arduino Core récent). L'ancien dépôt `me_no-dev` est obsolète et provoque des erreurs de linking.

> **Table de partitions personnalisée (`bob_control_8mb.csv`) :** dérivée de `default_8MB.csv` — partitions applicatives OTA réduites de 3,3 Mo à **2,5 Mo** chacune (firmware ~1,2 Mo) au profit de LittleFS, étendu de 1,5 Mo à **~2,9 Mo** pour héberger le modèle 3D du ROV (`data/3D/bob_rov_3D.glb`, servi sur `/3D/bob_rov_3D.glb` avec le MIME `model/gltf-binary` via une route explicite — l'extension `.glb` est inconnue de la table MIME du core Arduino) ainsi que three.js (`data/vendor/three.min.js`, r128 UMD licence MIT). **Toute modification de la table de partitions impose un reflash complet par USB** : `pio run -t erase && pio run -t upload && pio run -t uploadfs` — la table (offset 0x8000) ne peut pas être écrite par OTA.

---

## 📂 Structure du Répertoire

```
BOB-CONTROL/
├── README.md                  # Résumé du projet (GitHub)
├── BOBCONTROL.md              # Spécification et documentation (ce fichier)
├── PROTOCOLE_RPI5.md          # Protocole série RPi 5 : trames octet par octet + exemple Python
├── platformio.ini             # Configuration PlatformIO
├── data/                      # Fichiers LittleFS (interface Web)
│   ├── index.html             # Tableau de bord (tuiles + sections)
│   ├── style.css              # Thème sombre BOB CNC + drag & drop
│   └── app.js                 # WebSocket, navigation, drag & drop
├── include/
│   ├── config.h               # Broches, adresses I2C, UART, constantes réseau
│   ├── protocol.h             # Structures de trames packed + CRC16-CCITT + constantes MODE_*
│   ├── autopilot.h            # Structures PIDParams (Roll/Pitch/Yaw/Depth), AutopilotConfig, pidCompute()
│   ├── flight_controller.h    # Classe FlightController : modes PID, mixage moteur, priorité RPi5/Web
│   └── gpio_outputs.h         # 4 sorties tout-ou-rien ON/OFF : pins NVS, init OFF, état relu
└── src/
    ├── main.cpp               # Setup séquentiel + tâche vTaskControl (Core 1, 100 Hz) + corrections PID
    ├── web_server.cpp         # Serveur HTTP, DNS captif OS-aware, WebSocket, REST API
    ├── serial_comm.cpp        # USB-CDC / UART GPIO : abstraction Stream*, machine à états, watchdog failsafe
    ├── sensor_driver.cpp      # Drivers I2C réels — bus n°1 (BNO085, INA226 ×3) + bus n°2 (MS5803/MS5837)
    ├── flight_controller.cpp  # PID 4 axes (Roll/Pitch/Yaw/Depth) + mixage M5-M8 + gestion priorité maître
    ├── pwm_controller.cpp     # PCA9685 (bus I2C n°2) : conversion µs→ticks, typage canaux 10-15, failsafe + mode Témoin (Dry-Run)
    └── gpio_outputs.cpp       # 4 sorties ON/OFF : masque gpio_cmd, relecture pad, persistance NVS des broches
```

---

## 🔧 Commandes de Téléversement

```bash
# Téléverser le firmware
pio run -t upload

# Téléverser les fichiers web (LittleFS)
pio run -t uploadfs

# Moniteur série debug (USB-CDC)
pio device monitor -b 115200
```

---

## 📋 Notes Techniques

* **Mémoire :** RAM ~15%, Flash ~47% (marge confortable pour extensions).
* **Contrôleur de Vol (Flight Controller) :** 3 modes de pilotage avec PID 4 axes et mixage différentiel sur les 8 propulseurs :
  - **PASSIF (0)** — Consignes PWM appliquées telles quelles (manuel direct).
  - **AUTO ROULIS ET TANGAGE (1)** — Stabilisation PID du roulis et du tangage (cible 0°) sur les moteurs verticaux M5-M8.
  - **AUTO FULL (2)** — Stabilisation complète Roll + Pitch + Yaw + Profondeur avec mixage sur M1-M8.
  Le clamp 1000-2000 µs est appliqué après la somme totale de toutes les corrections pour éviter la saturation prématurée. En mode Web sans RPi 5, les propulseurs sont forcés à 1500 µs pour validation sur établi.
* **Gestion de Priorité Maître :** Le RPi 5 est maître absolu tant qu'une trame est reçue (< 1s). En l'absence de communication RPi 5, l'interface Web ESP32 prend le contrôle. Si le Web tente de changer le mode pendant que le RPi 5 est maître, la commande est ignorée et un message série est émis.
* **Mode Témoin / Dry-Run :** Au boot, les sorties physiques PCA9685 sont neutralisées par défaut (`physicalOutputsEnabled = false`). Les consignes PWM reçues (RPi 5 ou simulateur) sont mémorisées dans le buffer d'état et affichées dans les 16 barregraphes, mais le PCA9685 applique la position de sécurité de chaque canal : 1500 µs en bidirectionnel (défaut canaux 0–11, 14–15) et 1000 µs / 0 % en unidirectionnel (défaut canaux 12–13). Un commutateur dans l'onglet PWM Monitor permet d'activer/désactiver les sorties. Le Bit 4 de la trame montante (`STATUS_BIT_DRYRUN = 0x10`) signale cet état au RPi 5. Commande WebSocket `{output_enable: true/false}` ou REST `POST /api/pwm/output-enable`.
* **Typage des canaux PWM 10–15 :** chaque canal est configurable individuellement en **Bidirectionnel** (neutre 1500 µs, plage 1000–2000 µs) ou **Unidirectionnel / pleine échelle** (1000 µs = 0 % → 2000 µs = 100 %), depuis l'onglet Paramètres. Persistance NVS sous forme d'un masque 6 bits (clé `pwm_types`, bit n ↔ canal 10+n, 1 = bidirectionnel ; défaut `0x33` : CH10/11 et CH14/15 bidirectionnels, CH12/13 unidirectionnels), appliqué **à chaud** sans redémarrage. Le typage détermine la **position de sécurité** du failsafe (1500 µs en bidirectionnel, 1000 µs en unidirectionnel) et le rendu des barregraphes du PWM Monitor (curseur centré sur le neutre, ou jauge 0–100 % avec dégradé vert → orange → rouge). API REST : `GET/POST /api/pwm/config`.
* **Sorties Tout-ou-Rien (GPIO ON/OFF ×4) :** 4 sorties numériques auxiliaires (GPIO 15/16/17/18 par défaut, configurables dans l'onglet Paramètres et persistés en NVS — clés `gpio_p1`..`gpio_p4`, appliqués au redémarrage), initialisées en sortie à l'état BAS (OFF) au boot. Commandées par le masque `gpio_cmd` de la trame descendante (bit 0 = sortie 1), pilotables **même désarmé** ; forcées à OFF par le watchdog série et l'E-Stop. L'état réel (relu physiquement sur le pad) est remonté dans `gpio_state` (trame montante, 100 Hz) et affiché par badges dans l'interface. Boutons de test (établi) : `POST /api/gpio/test` (masque 4 bits) force directement l'état des sorties — la réponse renvoie l'état réel relu et le flag `rpi5_master`. API REST : `GET/POST /api/gpio/config`, `POST /api/gpio/test`.
* **Noms des sorties (×20) :** chacune des 20 sorties — 16 canaux PWM et 4 GPIO ON/OFF — peut recevoir un **nom personnalisé** (20 caractères max) depuis la carte « Noms des sorties » de l'onglet Paramètres. Persistés en NVS (namespace « config », clé `names`) sous forme d'un JSON unique de 20 chaînes (une entrée vide = nom par défaut, restitué côté interface), appliqués **à chaud** sans redémarrage : libellés du Banc de Test, curseurs et barregraphes du PWM Monitor, info-bulles du modèle 3D et carte GPIO. Noms nettoyés des caractères HTML/de contrôle en double (firmware + interface, rendu systématique en `textContent` — aucune injection possible), longueur bornée en octets sans couper l'UTF-8, écriture NVS **vérifiée par relecture** (HTTP 500 en cas d'échec). API REST : `GET/POST /api/names`.
* **Calibration altimétrique QNH (v1.0.1) :** option **non destructive** calibrant l'altimètre hors de l'eau sur le QNH réel d'Internet — carte « Calibration Altimètre (QNH) » de l'onglet Paramètres (interrupteur « Activer la calibration QNH automatique (Internet) », bouton « Actualiser le QNH maintenant » avec spinner pendant la requête, texte d'état « QNH actuel utilisé : XXXX.X mbar »). Le firmware interroge l'API Open-Meteo (`https://api.open-meteo.com/v1/forecast?latitude=46.20&longitude=7.23&current=pressure_msl` — Chamoson, VS) dans une **tâche détachée** `QnhFetch` (jamais dans la boucle de contrôle ni dans les handlers async) avec un **timeout court de 2000 ms** (connexion + réponse) ; au démarrage, la requête n'est lancée que si l'option est activée **et** la station Wi-Fi connectée à Internet, **10 s après le boot** (laisser le STA joindre le réseau). **Le QNH ne modifie que la formule d'altitude barométrique** (h = 44330·(1 − (P/QNH)^0.190284)) : la **profondeur immergée reste strictement fondée sur la tare de surface locale** (ΔP) et la boucle de contrôle n'est jamais impactée. **Fallback transparent** sur la valeur standard **1013.25 mbar** si l'option est désactivée, si Internet est absent, si la requête échoue ou si la réponse sort des bornes physiques acceptées (850–1100 mbar) — la désactivation réapplique immédiatement le standard ; une valeur valide est appliquée à chaud (`SensorDriver::setQnh()`, protégé par mutex) et **persistée en NVS** (clé `current_qnh`, réappliquée au boot suivant ; HTTPS sans vérification du certificat, l'ESP32 n'ayant pas d'horloge RTC fiable).

**Offset matériel du capteur 30 Bars (compensation logicielle) :** le MS5803-30BA présente un offset d'usine d'environ 25 mbar — insignifiant sous 300 m d'eau, mais ~200 m d'erreur d'altitude dans l'air. Chaque requête Open-Meteo (automatique au boot ou « Actualiser maintenant ») extrait en plus du QNH le champ racine `elevation` (altitude topographique réelle du lieu), calcule la pression atmosphérique théorique locale `P_theo = QNH·(1 − 0.0065·elev/288.15)^5.255`, lit la pression absolue brute instantanée `P_brute` du MS5803 et en déduit l'offset matériel `hardware_offset = P_brute − P_theo`. Garde-fous : calcul uniquement capteur **hors de l'eau** (immergé, la colonne d'eau fausserait tout), pression plausible et élévation fournie, offset borné à ±100 mbar (`MS5803_HW_OFFSET_MAX_MBAR` — au-delà, jugé aberrant : l'ancien offset reste). Une valeur valide est appliquée à chaud (`SensorDriver::setHwOffset()`, protégé par mutex) et **persistée en NVS** (clé `hw_offset`, réappliquée au boot). Dans la boucle de télémétrie, cet offset est compensé **uniquement dans le calcul de l'altitude** : `P_corrigée = P_brute − hardware_offset` puis `Altitude = 44330·(1 − (P_corrigée/QNH)^0.190284)` — **la tare de surface et le calcul de profondeur (ΔP) continuent d'utiliser exclusivement la pression absolue brute, strictement intouchés**. La carte des Paramètres affiche l'offset compensé lorsqu'il est non nul (« offset matériel : +XX.X mbar »).

> *Pourquoi doter un drone sous-marin d'un altimètre aéronautique de haute précision ? Par pur refus de laisser une erreur matérielle de 25 mbar dicter sa loi. Cette précision altimétrique, d'une utilité totalement discutable, est le fruit d'un over-engineering assumé : un ROV conçu en Suisse se doit d'être aussi précis dans l'air que sous l'eau, même si cela ne sert absolument à rien.*

API REST : `GET /api/qnh/config` (état de l'option, QNH appliqué, offset matériel, diagnostic), `POST /api/qnh/config` (`enable=true|false`), `POST /api/qnh/refresh` **alias `/api/calibrate_qnh`** (requête manuelle : QNH + calcul d'offset — 503 si pas de connexion Internet, « busy » si déjà en cours).
* **Centrale Inertielle BNO085 :** Intégrée via la bibliothèque **Adafruit BNO08x** (protocole SH-2 par paquets, et non par lectures de registres). Rapports activés à 50 Hz : **Rotation Vector 9-DOF** (gyro + accéléro + magnétomètre — source principale, cap magnétique absolu), **Game Rotation Vector** (relatif, secours automatique si le flux magnétique s'interrompt) et gyroscope calibré. **Montage tourné de 90°** autour de la verticale (X capteur → droite du ROV, Y → arrière) : quaternion **et** gyroscope sont remappés à la source dans le repère véhicule via `BNO085_MOUNT_YAW_DEG` (−90°), avant extraction des angles d'Euler embarquée — télémétrie série, horizon artificiel, HSI, modèle 3D et PID de cap partagent ainsi le même référentiel (lever l'avant → tangage +, penché à droite → roulis +). Auto-guérison du flux magnétique : re-activation du Rotation Vector renvoyée toutes les 5 s tant qu'aucun rapport n'arrive. **Supervision anti-gel :** le flux de quaternions est surveillé en continu par le watchdog 3 niveaux (ré-init SH-2 → reset bus I2C matériel → redémarrage contrôlé de l'ESP32 avec propulsion neutralisée) — un figement en plongée ou après une longue inactivité ne nécessite plus aucun redémarrage manuel (voir « Watchdog IMU »). Le capteur peut nécessiter un court délai après la mise sous tension ; un reboot relance l'initialisation si le BNO085 n'était pas prêt au premier démarrage. Il possède son propre bus dédié (bus n°1) — le MS5837 perturbant ses lectures SHTP lorsqu'ils partageaient le même bus, ce dernier a été déplacé sur le bus n°2.
* **Propriété des Bus I2C :** **Bus n°1** (`Wire`, GPIO 10/11 par défaut — BNO085 + INA226) : propriété exclusive de la tâche capteurs (Core 1). Les demandes de scan issues du serveur Web (Core 0) sont mises en file (`_scanReq`) et exécutées par la tâche capteurs au prochain cycle — plus aucune collision possible entre le bit-bang SCL du scan et une transaction en cours (les changements de broches ne passent plus par la file : sauvegarde NVS vérifiée + redémarrage, voir `saveBusPins`). **Bus n°2** (`Wire1`, GPIO 6/7 — MS5837 + PCA9685) : partagé entre la tâche capteurs (lecture pression 2 Hz) et la tâche de contrôle (écritures PCA9685 100 Hz) ; chaque transaction est atomique et sérialisée par le verrou interne de `Wire1` (un mutex par instance TwoWire — aucune contention notable, le MS5837 n'occupant le bus que ~20 ms par 500 ms). Le bus n°2 est configuré une seule fois au boot dans `SensorDriver::begin()`, avant la création des deux tâches, afin que sa fréquence/timeout ne changent jamais pendant une transaction. Séquence de récupération de bus bloqué (9 impulsions SCL + condition STOP) avant chaque scan — bus n°1 uniquement (le bus n°2 étant partagé, aucun bit-bang n'y est possible) ; timeout réduit à 10 ms pendant le scan des deux bus.
* **Watchdog I2C global :** compteur d'échecs de transactions consécutifs (instrumenté dans les wrappers `_i2cRead8/16`, `_i2cWrite` du bus n°1 et `_i2c2Read16`/`_i2c2Write`/`_readMS5803` du bus n°2 ; les lectures INA226 réussies du bus n°1 remettent le compteur à zéro en continu). Au-delà de 30 échecs (ou si tous les capteurs vus au boot sont perdus), reset du bus n°1 (Wire.end + 9 impulsions SCL + Wire.begin) et réinitialisation complète de tous les capteurs — BNO085 compris, même s'il avait été raté au boot ; le MS5837 est réinitialisé sur son propre bus n°2 sans reset de ce dernier (partagé avec la tâche de contrôle). Cooldown de 5 s entre deux récupérations. Complété par la supervision BNO085 sur le flux de quaternions (stall 1000 ms sans quaternion (Rotation Vector ou Game Rotation Vector) — le gyro ne compte pas — → escalade en 3 niveaux : ré-init SHTP légère, puis reset bus I2C matériel, puis redémarrage contrôlé de l'ESP32 après 3 resets bus sans rétablissement ou 8 s sans quaternion ; propulsion verrouillée au neutre pendant toute la récupération — voir la section « Watchdog IMU » ; un capteur jamais fonctionnel depuis le boot reste en mode espacé : polling SHTP coupé, badge rouge + message Web, une tentative complète toutes les 30 s, sans redémarrage en boucle). Détection de perte du MS5803 : après 3 échecs de lecture consécutifs (~1,5 s), le capteur est déclaré perdu et n'est plus pollé ; il est re-sondé toutes les 10 s pour prendre en compte un rebranchage à chaud.
* **Persistance NVS :** Les identifiants Wi-Fi, paramètres PID (Roll, Pitch, Yaw, Depth), choix d'interface série, **broches I2C (SDA/SCL)**, **typage des canaux PWM 10–15** (clé `pwm_types`), **broches des 4 sorties ON/OFF** (clés `gpio_p1`..`gpio_p4`), **noms personnalisés des 20 sorties** (namespace « config », clé `names`) et **calibration altimétrique QNH** (clés `qnh_auto_enable`, `current_qnh` et `hw_offset`) sont sauvegardés en NVS via la classe `Preferences`.
* **Mise à jour OTA (sans câble USB) :** Carte « Mise à jour OTA » de l'onglet Paramètres : téléversement du micrologiciel (`firmware.bin`, produit par `pio run`) vers la partition applicative inactive, ou des fichiers Web (`littlefs.bin`, produit par `pio run -t buildfs`) vers la partition LittleFS. Le schéma de partitions `bob_control_8mb.csv` contient deux partitions OTA (`ota_0`/`ota_1` de 2,5 Mo) : l'ancien firmware reste la partition de boot tant que la nouvelle n'est pas validée — une coupure en cours d'écriture ne brike jamais le contrôleur. Sécurités : propulsion neutralisée (Dry-Run + neutre PCA9685) AVANT la première écriture flash (les écritures suspendent brièvement les deux cœurs) ; LittleFS démonté avant réécriture (remonté en cas d'échec, sinon redémarrage) ; octet magie ESP `0xE9` vérifié au premier bloc (refuse l'inversion firmware / image FS — la corrompant silencieusement) ; redémarrage différé 3 s après validation complète ; l'interface ne se recharge que lorsque `GET /api/system/status` montre un uptime reparti de zéro (preuve du redémarrage effectif, comme pour les broches I2C). API REST : `POST /api/ota/firmware`, `POST /api/ota/filesystem` (multipart), `GET /api/system/status`.
* **Version Git dynamique (pied de page) :** le script `scripts/git_version.py` (exécuté avant chaque compilation via `extra_scripts = pre:` dans `platformio.ini`) récupère `git describe --tags --always --dirty` et génère l'en-tête `include/git_version.h` définissant la macro `GIT_VERSION` (fichier régénéré seulement si le contenu change — builds incrémentaux préservés ; ignoré par Git ; fallback `"dev"` hors dépôt). Pourquoi un en-tête plutôt que `-D GIT_VERSION="..."` : la couche SCons/PlatformIO altère les guillemets des `CPPDEFINES` contenant des tirets, livrant un corps de macro déquoté que le compilateur rejette. La version est exposée par `GET /api/system/status` (champ `version`) et affichée par le pied de page global de l'interface (« BOB-CONTROL vX | Auteur : Dero Didier | Licence MIT »), rafraîchi au chargement par `app.js` (libellé statique conservé si l'API est indisponible) — les deux images (firmware + LittleFS) doivent être flashées pour la voir remonter.
* **Diagnostic I2C :** Scanner complet des DEUX bus I2C (adresses 0x01–0x7F, colonne Bus dans les résultats) accessible depuis l'onglet Paramètres. Identification automatique des périphériques (PCA9685, INA226 ×3, BNO085, MS5803/MS5837). Configuration dynamique des broches SDA/SCL des deux bus (`POST /api/i2c/config`, paramètre `bus=1|2`) : sauvegarde NVS vérifiée par relecture puis redémarrage différé de 3 s pour les DEUX bus (broches appliquées au boot ; le bus n°2 est partagé avec les sorties PWM — un `Wire1.end()` en vol couperait la propulsion ; le bus n°1 suit la même procédure pour une interface uniforme). L'interface ne se recharge qu'une fois l'API revenue avec les nouvelles broches (preuve du redémarrage effectif). Récupération automatique de bus bloqué avant chaque scan du bus n°1. Persistance NVS : les broches des deux bus (clés `i2c_sda`/`i2c_scl` et `i2c2_sda`/`i2c2_scl`) sont rechargées au boot. La page « Câblage & Schémas » suit les broches réellement configurées (schémas, brochage et tableaux mis à jour dynamiquement). API REST : `GET /api/i2c/scan`, `GET /api/i2c/config`.
* **Architecture FreeRTOS :** Tâches épinglées par cœur (Core 0 : Wi-Fi/WebServer/DNS, Core 1 : Série/Capteurs/PWM/Contrôle). Le bus I2C n°2 (Wire1) est partagé entre la tâche capteurs et la tâche de contrôle — transactions sérialisées par le verrou interne de Wire1 (voir « Propriété des Bus I2C »).
