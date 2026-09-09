# BOB-CONTROL - Contrôleur Temps Réel ROV

**Version :** 2.0.0  
**Plateforme :** ESP32-S3 (PlatformIO / Arduino Core / FreeRTOS)  
**Auteur :** Didier Dero  
**Projet associé :** BOB-ROV (Cockpit Raspberry Pi 5)  
**Date :** Août 2026  

---

## 🎯 Philosophie & Rôle Système

**BOB-CONTROL** est le micrologiciel temps réel dédié à la gestion basse couche du sous-marin **BOB-ROV**. Embarqué sur un **ESP32-S3**, il isole les fonctions critiques de pilotage matériel et d'acquisition de données du système d'exploitation hôte (Raspberry Pi 5).

Il remplit un triple rôle :
1. **Acquisition & Fusion Capteurs :** Lecture haute fréquence du bus I2C (centrale inertielle BNO085, pression/profondeur MS5803/MS5837, wattmètres INA226).
2. **Contrôleur de Vol PID :** Stabilisation automatique roulis/tangage/cap/profondeur avec mixage différentiel sur les 8 propulseurs.
3. **Génération PWM Matérielle :** Contrôle autonome de 16 canaux PWM indépendants via PCA9685 (8 propulseurs, servos pan/tilt, pince, gradateurs d'éclairage), avec serveur Web/WebSocket autonome et portail captif pour la configuration et le banc de test.

---

## 🏗️ Architecture Matérielle & Adressage

### 1. Périphériques I2C

| Composant | Rôle | Adresse I2C | Description technique |
| :--- | :--- | :--- | :--- |
| **BNO085** | IMU 9-DOF | `0x4A` | Fusion inertielle embarquée (protocole SH-2 via Adafruit BNO08x) : Game Rotation Vector (quaternions) + gyroscope calibré @ 50 Hz |
| **MS5803-30BA / MS5837** | Pression & Profondeur | `0x76` | Capteur piézorésistif haute résolution étanche, compensation 24-bit (coefficients C1-C6) |
| **INA226 (Bus 1)** | Puissance RPi 5 | `0x41` | Tension, courant et puissance consommée par le Pi 5 |
| **INA226 (Bus 2)** | Puissance Aux/LEDs | `0x44` | Mesure de ligne d'éclairage et accessoires 12V |
| **INA226 (Bus 3)** | Puissance Moteurs | `0x45` | Mesure de ligne de puissance batterie/propulsion principale |
| **PCA9685** | Contrôleur PWM 16 Ch | `0x40` | Générateur PWM 12 bits autonome (VCC 3.3V, V+ 5V/Servo) |

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
* **Canaux 10 & 11 :** Outil / Pince & Rotation ($1000 - 2000\,\mu\text{s}$)
* **Canaux 12 & 13 :** Gradateurs MOSFET pour projecteurs LED ($0 - 100\,\%$)
* **Canaux 14 & 15 :** Canaux auxiliaires / Extensions futures

---

## 📡 Protocole Série Binaire RPi 5 ↔ ESP32-S3

La communication s'effectue à **921 600 bauds** sur l'interface sélectionnée (USB-CDC natif ou UART matériel GPIO 1/2). Toutes les trames sont à taille fixe avec validation par somme de contrôle CRC16-CCITT.

### 1. Trame Descendante : RPi 5 → ESP32-S3 (39 octets @ 20-50 Hz)

```
[0xAA][0x55][TYPE][MODE][ARM_STATE][PWM_0 ... PWM_15][CRC16_HIGH][CRC16_LOW]
```

* **Header :** `0xAA`, `0x55` (2 octets)
* **Type :** `0x01` (1 octet)
* **Mode :** `uint8_t` (`0` = Passif, `1` = AUTO Roulis et Tangage, `2` = Auto-Full)
* **Arm State :** `uint8_t` (`0` = Disarmed, `1` = Armed, `2` = Emergency Stop)
* **PWM Channels (0–15) :** `uint16_t[16]` (32 octets, consigne en $\mu\text{s}$)
* **CRC16 :** `uint16_t` (2 octets, polynôme 0x1021)

### 2. Trame Montante : ESP32-S3 → RPi 5 (71 octets @ 50-100 Hz)

```
[0x55][0xAA][TYPE][STATUS][QUAT_W,X,Y,Z][GYRO_X,Y,Z][PRESS,TEMP][INA_1,2,3][PWM_ACT_0...15][CRC16]
```

* **Header :** `0x55`, `0xAA` (2 octets)
* **Type :** `0x02` (1 octet)
* **Status Flags :** `uint8_t` (Bit 0: réservé, Bit 1: Armé, Bit 2: Autopilote actif, Bit 3: Watchdog déclenché, Bit 4: Sorties physiques neutralisées / Dry-Run)
* **Quaternions IMU :** `int16_t[4]` (8 octets, $W, X, Y, Z$ scalés $\times 10\,000$)
* **Gyroscope :** `int16_t[3]` (6 octets, $\omega_x, \omega_y, \omega_z$ scalés $\times 100$)
* **Pression & Température :** `int32_t[2]` (8 octets, $0.1\text{ mbar}$ et $0.01\,^\circ\text{C}$)
* **Télémétrie Puissance :** `uint16_t[6]` (12 octets, $V$ et $I$ pour Pi, Aux et Propulsion en $mV$ / $mA$)
* **PWM Effectifs :** `uint16_t[16]` (32 octets, impulsions réelles envoyées aux actionneurs)
* **CRC16 :** `uint16_t` (2 octets)

### 3. Watchdog Failsafe

Un watchdog logiciel surveille la réception de trames descendantes. Si aucune trame valide n'est reçue pendant le **timeout configurable** (défaut : 500 ms), tous les canaux PWM sont automatiquement forcés au neutre ($1500\,\mu\text{s}$) pour la sécurité des propulseurs.

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
2. **Cliquer "Réorganiser"** → active le mode édition (animation wobble + curseur grab).
3. **Glisser-déposer les tuiles** → réordonne en temps réel (drag & drop multi-plateforme via Pointer Events, fonctionne sur tactile et souris).
4. **Cliquer "Terminer"** → sauvegarde l'ordre dans `localStorage` (persistant entre les rechargements).

### Fonctions du Tableau de Bord (6 Sections)

| Tuile | Contenu |
| :--- | :--- |
| **Wi-Fi** | **Connexion actuelle** temps réel : SSID, adresse IP, passerelle (DHCP), masque, RSSI, MAC. Infos point d'accès local (SSID, IP, clients). Scan des réseaux environnants. Connexion STA non-bloquante (sans tuer le point d'accès). |
| **PWM Monitor** | 16 barregraphes temps réel (WebSocket). 8 jauges moteurs bidirectionnelles centrées à $1500\,\mu\text{s}$, servos, gradateurs, auxiliaires. **Commutateur Sorties Matérielles** (Mode Témoin / Dry-Run) : badge d'avertissement ambre/vert. |
| **Télémétrie** | Assiette Euler (Roll/Pitch/Yaw), profondeur, pression, température. 3 wattmètres instantanés. **Sélecteur de Mode de Pilotage** : 3 boutons radio (PASSIF / AUTO ROULIS ET TANGAGE / AUTO FULL) avec badge Maître (RPi 5 vert / Manuel ESP32 orange). Instruments aviation : horizon artificiel et HSI (Heading Situation Indicator) avec lissage LERP. Diagnostic capteurs (LED BNO085, MS5803, INA226 ×3, PCA9685). |
| **Banc de Test** | 16 sliders de commande manuelle avec verrou de sécurité matériel. Bouton "Tout au Neutre". |
| **Paramètres** | Sélection liaison série RPi 5 (USB-CDC / UART GPIO 1/2). **Configuration I2C** : broches SDA/SCL avec sauvegarde NVS. **Scanner I2C** : scan complet 0x01–0x7F avec identification des périphériques. PID (Roll, Pitch, Yaw, Profondeur) : Kp, Ki, Kd. Timeout watchdog série réglable. |
| **Câblage** | Schéma visuel ESP32-S3 (`esp32s3.png`). Liaison RPi 5 dynamique selon le mode actif (USB-CDC ou UART GPIO 1/2) avec diagrammes ASCII et tables de correspondance. Tableau complet du bus I2C : broches SDA/SCL, adresses et fonctions de chaque module (IMU, baromètre, INA226, PCA9685). |

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
board_build.partitions = default_8MB.csv
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

> **Note :** Les bibliothèques `ESPAsyncWebServer` et `AsyncTCP` utilisent le fork maintenu par `mathieucarbou` (compatible ESP32 Arduino Core récent). L'ancien dépôt `me-no-dev` est obsolète et provoque des erreurs de linking.

---

## 📂 Structure du Répertoire

```
BOB-CONTROL/
├── README.md                  # Résumé du projet (GitHub)
├── BOBCONTROL.md              # Spécification et documentation (ce fichier)
├── platformio.ini             # Configuration PlatformIO
├── data/                      # Fichiers LittleFS (interface Web)
│   ├── index.html             # Tableau de bord (tuiles + sections)
│   ├── style.css              # Thème sombre BOB CNC + drag & drop
│   └── app.js                 # WebSocket, navigation, drag & drop
├── include/
│   ├── config.h               # Broches, adresses I2C, UART, constantes réseau
│   ├── protocol.h             # Structures de trames packed + CRC16-CCITT + constantes MODE_*
│   ├── autopilot.h            # Structures PIDParams (Roll/Pitch/Yaw/Depth), AutopilotConfig, pidCompute()
│   └── flight_controller.h    # Classe FlightController : modes PID, mixage moteur, priorité RPi5/Web
└── src/
    ├── main.cpp               # Setup séquentiel + tâche vTaskControl (Core 1, 100 Hz) + corrections PID
    ├── web_server.cpp         # Serveur HTTP, DNS captif OS-aware, WebSocket, REST API
    ├── serial_comm.cpp        # USB-CDC / UART GPIO : abstraction Stream*, machine à états, watchdog failsafe
    ├── sensor_driver.cpp      # Drivers I2C réels (BNO085 via Adafruit SH-2, MS5803/MS5837, INA226 ×3)
    ├── flight_controller.cpp  # PID 4 axes (Roll/Pitch/Yaw/Depth) + mixage M5-M8 + gestion priorité maître
    └── pwm_controller.cpp     # PCA9685 : conversion µs→ticks, failsafe + mode Témoin (Dry-Run)
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

* **Mémoire :** RAM ~14%, Flash ~36% (marge confortable pour extensions).
* **Contrôleur de Vol (Flight Controller) :** 3 modes de pilotage avec PID 4 axes et mixage différentiel sur les 8 propulseurs :
  - **PASSIF (0)** — Consignes PWM appliquées telles quelles (manuel direct).
  - **AUTO ROULIS ET TANGAGE (1)** — Stabilisation PID du roulis et du tangage (cible 0°) sur les moteurs verticaux M5-M8.
  - **AUTO FULL (2)** — Stabilisation complète Roll + Pitch + Yaw + Profondeur avec mixage sur M1-M8.
  Le clamp 1000-2000 µs est appliqué après la somme totale de toutes les corrections pour éviter la saturation prématurée. En mode Web sans RPi 5, les propulseurs sont forcés à 1500 µs pour validation sur établi.
* **Gestion de Priorité Maître :** Le RPi 5 est maître absolu tant qu'une trame est reçue (< 1s). En l'absence de communication RPi 5, l'interface Web ESP32 prend le contrôle. Si le Web tente de changer le mode pendant que le RPi 5 est maître, la commande est ignorée et un message série est émis.
* **Mode Témoin / Dry-Run :** Au boot, les sorties physiques PCA9685 sont neutralisées par défaut (`physicalOutputsEnabled = false`). Les consignes PWM reçues (RPi 5 ou simulateur) sont mémorisées dans le buffer d'état et affichées dans les 16 barregraphes, mais le PCA9685 envoie le neutre (1500 µs canaux 0–11, 0 % canaux 12–15). Un commutateur dans l'onglet PWM Monitor permet d'activer/désactiver les sorties. Le Bit 4 de la trame montante (`STATUS_BIT_DRYRUN = 0x10`) signale cet état au RPi 5. Commande WebSocket `{output_enable: true/false}` ou REST `POST /api/pwm/output-enable`.
* **Centrale Inertielle BNO085 :** Intégrée via la bibliothèque **Adafruit BNO08x** (protocole SH-2 par paquets, et non par lectures de registres). Rapports activés à 50 Hz : Game Rotation Vector (quaternions) et gyroscope calibré. Conversion quaternion → Euler embarquée. Le capteur peut nécessiter un court délai après la mise sous tension ; un reboot relance l'initialisation si le BNO085 n'était pas prêt au premier démarrage.
* **Protection du Bus I2C :** Flag `_i2cBusy` volatile empêche les accès concurrents entre la tâche capteurs (Core 1) et le scan/reinit I2C (Core 0). Séquence de récupération de bus bloqué (9 impulsions SCL + condition STOP) avant chaque scan pour libérer un SDA coincé. Timeout Wire réduit à 10 ms pendant le scan pour un temps total < 1.3 s.
* **Persistance NVS :** Les identifiants Wi-Fi, paramètres PID (Roll, Pitch, Yaw, Depth), choix d'interface série et **broches I2C (SDA/SCL)** sont sauvegardés en NVS via la classe `Preferences`.
* **Diagnostic I2C :** Scanner complet du bus I2C (adresses 0x01–0x7F) accessible depuis l'onglet Paramètres. Identification automatique des périphériques (PCA9685, INA226 ×3, BNO085, MS5803/MS5837). Configuration dynamique des broches SDA/SCL avec réinitialisation du bus sans reflash (`POST /api/i2c/config`). Récupération automatique de bus bloqué avant chaque scan. Persistance NVS : les broches configurées sont rechargées au boot. API REST : `GET /api/i2c/scan`, `GET /api/i2c/config`.
* **Architecture FreeRTOS :** Tâches épinglées par cœur (Core 0 : Wi-Fi/WebServer/DNS, Core 1 : Série/Capteurs/PWM/Contrôle).
