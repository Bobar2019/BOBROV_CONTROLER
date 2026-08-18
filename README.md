# BOB-CONTROL — Firmware ESP32-S3 pour ROV

Firmware temps réel embarqué sur **ESP32-S3** pour le contrôle basse couche du sous-marin **BOB-ROV**.

## Matériel

- **MCU :** ESP32-S3 (PlatformIO, Arduino Core, FreeRTOS)
- **IMU :** GY-91 (MPU9250 + AK8963 + BMP280) sur bus I2C
- **PWM :** PCA9685 (16 canaux, 50 Hz)
- **Propulseurs :** 8 × ESC brushless (M1-M4 horizontaux, M5-M8 verticaux)
- **Capteurs :** 3 × INA226 (wattmètres), MS5803/BME280 (pression)
- **Liaison :** RPi 5 via USB-CDC natif ou UART GPIO (921 600 bauds)

## Fonctionnalités

- **Protocole binaire** RPi 5 ↔ ESP32-S3 avec CRC16-CCITT et watchdog failsafe
- **3 modes de pilotage** avec PID 4 axes et mixage différentiel :
  - `PASSIF` — Manuel direct (consignes appliquées telles quelles)
  - `AUTO ROULIS` — Stabilisation PID du roulis sur M5-M8
  - `AUTO FULL` — Stabilisation complète Roll + Pitch + Yaw + Profondeur
- **Interface Web embarquée** (LittleFS) avec portail captif automatique
- **WebSocket temps réel** : télémétrie IMU, pression, PWM, puissance (20 Hz)
- **API REST** : configuration Wi-Fi, capteurs, I2C, PID, mode série
- **Instruments aviation** : horizon artificiel + HSI (Canvas 2D, lissage LERP)
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

Voir [BOBCONTROL.md](BOBCONTROL.md) pour la spécification complète (protocole série, architecture, brochage, API).

## Licence

MIT
