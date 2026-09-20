# BOB-CONTROL — Firmware ESP32-S3 pour ROV

![Version](https://img.shields.io/badge/version-v1.7.0-00d9ff?style=for-the-badge)

Firmware temps réel embarqué sur **ESP32-S3** pour le contrôle basse couche du sous-marin **BOB-ROV**.

## Matériel

- **MCU :** ESP32-S3 (PlatformIO, Arduino Core, FreeRTOS)
- **IMU :** BNO085 (`0x4A`) via Adafruit BNO08x (protocole SH-2 — Rotation Vector 9-DOF, montage remappé à la source)
- **PWM :** PCA9685 (16 canaux, 50 Hz)
- **Propulseurs :** 8 × ESC brushless (M1-M4 horizontaux, M5-M8 verticaux)
- **Capteurs :** 3 × INA226 (wattmètres), MS5803/MS5837 (pression/profondeur)
- **Liaison :** RPi 5 via USB-CDC natif ou UART GPIO (921 600 bauds)

## Fonctionnalités

- **Protocole binaire** RPi 5 ↔ ESP32-S3 avec CRC16-CCITT et watchdog failsafe
- **Assistances superposables** (masque de bits, protocole v1.4.0) avec PID 4 axes et mixage différentiel :
  - `0x01` **Auto R/T** — Stabilisation PID du roulis et du tangage sur M5-M8
  - `0x02` **Tenue de Profondeur** — PID altitude (profondeur capturée à l'activation)
  - `0x04` **Auto Cap** — PID lacet sur M1-M4 (cap capturé à l'activation)
  - `0x08` **Retour Surface** — Prioritaire : verticaux vers la surface
  - Combinaisons libres (ex. `0x03` = Auto R/T + profondeur ; `0x07` ≡ ancien AUTO FULL) ; le champ `status` de la télémétrie confirme les boucles réellement actives
- **Interface Web embarquée** (LittleFS) avec portail captif automatique
- **WebSocket temps réel** : télémétrie IMU, pression, PWM, puissance (20 Hz)
- **API REST** : configuration Wi-Fi, I2C, PID, mode série
- **Instruments aviation** : horizon artificiel + HSI (Canvas 2D, lissage LERP)
- **Modèle 3D temps réel** : ROV animé par l'IMU (three.js r128 local, GLB servi depuis LittleFS)
- **Mode Témoin (Dry-Run)** : sorties physiques neutralisées pour banc de test
- **Gestion de priorité** : RPi 5 maître absolu < 1s, sinon bascule Web ESP32

## Build & Flash

```bash
# Compiler
pio run

# Téléverser firmware
pio run -t upload

# Téléverser fichiers Web (LittleFS)
pio run -t uploadfs

# Moniteur série debug
pio device monitor -b 115200
```

## Accès Interface Web

Connectez-vous au point d'accès Wi-Fi **BOB-CONTROL_AP** (mot de passe : `bobrov2026`), puis ouvrez `http://192.168.4.1`.

## Documentation

- [PROTOCOLE_RPI5.md](PROTOCOLE_RPI5.md) — protocole série RPi 5 ↔ ESP32-S3 : trames octet par octet, sémantique des statuts, exemple Python prêt à l'emploi, vecteurs de test.
- [BOBCONTROL.md](BOBCONTROL.md) pour la spécification complète (protocole série, architecture, brochage, API).

## Licence

MIT
