# BOB-CONTROL - Contrôleur Temps Réel & Simulateur ROV

**Version :** 1.0.0  
**Plateforme :** ESP32-S3 (PlatformIO / Arduino Core / FreeRTOS)  
**Auteur :** Didier Dero  
**Projet associé :** BOB-ROV (Cockpit Raspberry Pi 5)  
**Date :** Août 2026  

---

## 🎯 Philosophie & Rôle Système

**BOB-CONTROL** est le micrologiciel temps réel dédié à la gestion basse couche du sous-marin **BOB-ROV**. Embarqué sur un **ESP32-S3**, il isole les fonctions critiques de pilotage matériel et d'acquisition de données du système d'exploitation hôte (Raspberry Pi 5).

Il remplit un triple rôle :
1. **Acquisition & Fusion Capteurs :** Lecture haute fréquence du bus I2C (Centrale inertielle, Pression/Profondeur, Wattmètres de puissance).
2. **Génération PWM Matérielle :** Contrôle autonome de 16 canaux PWM indépendants via PCA9685 (8 propulseurs, servos pan/tilt, pince, gradateurs d'éclairage).
3. **Simulateur Embarqué & Banc de Test :** Émulation logicielle dynamique de l'ensemble des capteurs avec serveur Web/WebSocket autonome et portail captif pour validation sans matériel connecté.

---

## 🏗️ Architecture Matérielle & Adressage

### 1. Périphériques I2C

| Composant | Rôle | Adresse I2C | Description technique |
| :--- | :--- | :--- | :--- |
| **BNO085** | IMU 9-DOF | `0x4A` | Fusion inertielle embarquée, Quaternions, Vitesse angulaire |
| **MS5803-30BA** | Pression & Profondeur | `0x76` | Capteur piézorésistif haute résolution étanche (30 bar) |
| **INA226 (Bus 1)** | Puissance RPi 5 | `0x41` | Tension, courant et puissance consommée par le Pi 5 |
| **INA226 (Bus 2)** | Puissance Aux/LEDs | `0x44` | Mesure de ligne d'éclairage et accessoires 12V |
| **INA226 (Bus 3)** | Puissance Moteurs | `0x45` | Mesure de ligne de puissance batterie/propulsion principale |
| **PCA9685** | Contrôleur PWM 16 Ch | `0x40` | Générateur PWM 12 bits autonome (VCC 3.3V, V+ 5V/Servo) |

### 2. Affectation des Canaux PWM (PCA9685)

* **Canaux 0 à 7 :** 8 × ESC Propulseurs ($1000\,\mu\text{s}$ à $2000\,\mu\text{s}$, neutre à $1500\,\mu\text{s}$)
* **Canaux 8 & 9 :** Servomoteurs Pan & Tilt Caméra ($50\text{ Hz}$)
* **Canaux 10 & 11 :** Outil / Pince & Rotation ($1000 - 2000\,\mu\text{s}$)
* **Canaux 12 & 13 :** Gradateurs MOSFET pour projecteurs LED ($0 - 100\,\%$)
* **Canaux 14 & 15 :** Canaux auxiliaires / Extensions futures

---

## 📡 Protocole Série Binaire RPi 5 ↔ ESP32-S3

La communication s'effectue via liaison série bidirectionnelle (USB-CDC natif ou UART matériel à 921 600 bauds). Toutes les trames sont à taille fixe avec validation par somme de contrôle CRC16-CCITT.

### 1. Trame Descendante : RPi 5 → ESP32-S3 (38 octets @ 20-50 Hz)

[0xAA][0x55][TYPE][MODE][ARM_STATE][PWM_0 ... PWM_15][CRC16_HIGH][CRC16_LOW]


* **Header :** `0xAA`, `0x55` (2 octets)
* **Type :** `0x01` (1 octet)
* **Mode :** `uint8_t` (`0` = Manuel, `1` = Auto-Roll, `2` = Depth-Hold, `3` = Full-Stabilized)
* **Arm State :** `uint8_t` (`0` = Disarmed, `1` = Armed, `2` = Emergency Stop)
* **PWM Channels (0–15) :** `uint16_t[16]` (32 octets, consigne en $\mu\text{s}$)
* **CRC16 :** `uint16_t` (2 octets, polynôme 0x1021)

### 2. Trame Montante : ESP32-S3 → RPi 5 (71 octets @ 50-100 Hz)

[0x55][0xAA][TYPE][STATUS][QUAT_W,X,Y,Z][GYRO_X,Y,Z][PRESS,TEMP][INA_1,2,3][PWM_ACT_0...15][CRC16]


* **Header :** `0x55`, `0xAA` (2 octets)
* **Type :** `0x02` (1 octet)
* **Status Flags :** `uint8_t` (Bit 0: Mode Simu actif, Bit 1: Armé, Bit 2: Autopilote actif, Bit 3: Watchdog déclenché)
* **Quaternions IMU :** `int16_t[4]` (8 octets, $W, X, Y, Z$ scalés $\times 10\,000$)
* **Gyroscope :** `int16_t[3]` (6 octets, $\omega_x, \omega_y, \omega_z$ scalés $\times 100$)
* **Pression & Température :** `int32_t[2]` (8 octets, $0.1\text{ mbar}$ et $0.01\,^\circ\text{C}$)
* **Télémétrie Puissance :** `uint16_t[6]` (12 octets, $V$ et $I$ pour Pi, Aux et Propulsion en $mV$ / $mA$)
* **PWM Effectifs :** `uint16_t[16]` (32 octets, impulsions réelles envoyées aux actionneurs)
* **CRC16 :** `uint16_t` (2 octets)

---

## 🌐 Interface Web Embarquée & Portail Captif

L'ESP32-S3 héberge une application Web autonome stylisée selon la charte industrielle BOB CNC (thème sombre, réactivité WebSocket, navigation par onglets) stockée dans la mémoire flash LittleFS.

### Fonctionnalités Réseau
* **Portail Captif Automatique :** Point d'accès Wi-Fi autonome (`BOB-CONTROL_AP` / IP : `192.168.4.1`) avec serveur DNS redirigeant automatiquement les clients connectés vers l'IHM.
* **Scan & Connexion à chaud :** Balayage asynchrone des points d'accès Wi-Fi environnants avec indication de puissance (dBm) et mémorisation NVS des identifiants réseau.

### Découpage de l'IHM (5 Onglets)
1. **Wi-Fi & Réseau :** Interface de scan, sélection SSID, saisie clé de sécurité et affichage du statut IP.
2. **PWM Monitor :** 16 barregraphes temps réel affichant les consignes appliquées (8 jauges moteurs centrées à $1500\,\mu\text{s}$, 4 angles servos, 2 éclairages, 2 auxiliaires).
3. **Télémétrie & Capteurs :** Affichage d'assiette 3D/Euler, profondeur, pressions et puissances instantanées. Bouton de bascule **[ SIMULATEUR / RÉEL ]**.
4. **Banc de Test :** Sliders de commande manuelle avec verrou de sécurité pour tester les actionneurs sans le Raspberry Pi 5.
5. **Paramètres & Sécurité :** Configuration des boucles d'asservissement PID et réglage du timeout de sécurité série (Watchdog Failsafe).

---

## ⚙️ Configuration Environnement PlatformIO

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
    me-no-dev/ESPAsyncWebServer @ ^1.2.3
    me-no-dev/AsyncTCP @ ^1.1.1
    bblanchon/ArduinoJson @ ^7.0.0
    adafruit/Adafruit PWM Servo Driver Library @ ^3.0.1
    adafruit/Adafruit BNO08x @ ^1.2.5
    robtillaart/INA226 @ ^0.6.0
📂 Structure du Répertoire PlatformIO
BOB-CONTROL/
├── BOBCONTROL.md           # Spécification et documentation
├── platformio.ini          # Configuration PlatformIO
├── data/                   # Fichiers LittleFS (Web)
│   ├── index.html          # SPA interface BOB CNC
│   ├── style.css           # Thème sombre et layout
│   └── app.js              # WebSocket et logique IHM
├── include/
│   ├── config.h            # Broches, adresses I2C et constantes
│   ├── protocol.h          # Structures de trames et CRC16
│   └── autopilot.h         # Algorithmes d'asservissement PID
└── src/
    ├── main.cpp            # Initialisation et boucles FreeRTOS
    ├── web_server.cpp      # Serveur HTTP, DNS Captif et WebSocket
    ├── serial_comm.cpp     # Gestionnaire de liaison série RPi 5
    ├── sensor_driver.cpp   # Drivers I2C et moteur de simulation
    └── pwm_controller.cpp  # Pilote PCA9685 et sécurité Failsafe