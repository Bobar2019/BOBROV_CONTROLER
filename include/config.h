/**
 * @file config.h
 * @brief Configuration matérielle globale du projet BOB-CONTROL.
 *
 * Ce fichier centralise l'ensemble des constantes matérielles, adresses I2C,
 * assignations de broches, paramètres réseau, fréquences de tâches FreeRTOS,
 * énumérations de capteurs et profils HAL pour le contrôleur ROV.
 *
 * @author Didier Dero
 * @version 1.1.0
 * @date Août 2026
 * @copyright MIT License
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* =========================================================================
 * SECTION 1 : CONFIGURATION I2C
 * ========================================================================= */

/** @brief Broche SDA du bus I2C principal (GPIO 10 sur DevKitC-1) */
constexpr uint8_t  I2C_SDA_PIN          = 10;

/** @brief Broche SCL du bus I2C principal (GPIO 11 sur DevKitC-1) */
constexpr uint8_t  I2C_SCL_PIN          = 11;

/** @brief Fréquence du bus I2C en Hz (400 kHz Fast Mode) */
constexpr uint32_t I2C_FREQ_HZ         = 400000UL;

/** @brief Numéro du port I2C matériel utilisé */
constexpr int      I2C_PORT_NUM         = 0;

/* =========================================================================
 * SECTION 2 : ADRESSES I2C DES PÉRIPHÉRIQUES
 * ========================================================================= */

/** @brief Adresse I2C du contrôleur PWM 16 canaux PCA9685 */
constexpr uint8_t  PCA9685_I2C_ADDR     = 0x40;

/** @brief Adresse I2C de la centrale inertielle BNO085 (9-DOF) */
constexpr uint8_t  BNO085_I2C_ADDR      = 0x4A;

/** @brief Adresse I2C du capteur de pression MS5803-30BA / MS5837 */
constexpr uint8_t  MS5803_I2C_ADDR      = 0x76;

/** @brief Adresse I2C du wattmètre INA226 n°1 (alimentation RPi 5) */
constexpr uint8_t  INA226_1_I2C_ADDR    = 0x41;

/** @brief Adresse I2C du wattmètre INA226 n°2 (auxiliaire / LEDs) */
constexpr uint8_t  INA226_2_I2C_ADDR    = 0x44;

/** @brief Adresse I2C du wattmètre INA226 n°3 (propulsion moteur) */
constexpr uint8_t  INA226_3_I2C_ADDR    = 0x45;

/* =========================================================================
 * SECTION 3 : ÉNUMÉRATIONS CAPTEURS & PROFILS HAL
 * ========================================================================= */

/**
 * @brief État de connexion d'un composant matériel.
 */
enum SensorStatus : uint8_t {
    SENSOR_DISCONNECTED = 0,    ///< Non détecté sur le bus I2C
    SENSOR_CONNECTED    = 1     ///< Détecté et opérationnel
};

/* =========================================================================
 * SECTION 3b : ÉNUMÉRATION INTERFACE DE COMMUNICATION RPi5
 * ========================================================================= */

/**
 * @brief Type d'interface physique pour la liaison binaire RPi5 ↔ ESP32-S3.
 *
 * Sélectionnable via l'interface Web, persistée en NVS (clé "comm_iface").
 * - USB_CDC  : utilise le port USB natif de l'ESP32-S3 (/dev/ttyACM0 côté RPi).
 * - UART_GPIO : utilise Serial1 sur broches dédiées (RX=GPIO1, TX=GPIO2).
 */
enum SerialInterfaceType : uint8_t {
    COMM_INTERFACE_USB_CDC  = 0,    ///< Port USB natif (Serial) — /dev/ttyACM0
    COMM_INTERFACE_UART_GPIO = 1    ///< UART matériel Serial1 (RX: GPIO1, TX: GPIO2)
};

/* =========================================================================
 * SECTION 4 : CONFIGURATION PCA9685 ET PWM
 * ========================================================================= */

/** @brief Fréquence de sortie du PCA9685 en Hz (standard servo) */
constexpr uint16_t PCA9685_FREQ_HZ      = 50;

/** @brief Résolution du compteur PCA9685 (12 bits) */
constexpr uint16_t PCA9685_RESOLUTION   = 4096;

/** @brief Nombre total de canaux PWM gérés */
constexpr uint8_t  NUM_PWM_CHANNELS     = 16;

/** @brief Impulsion neutre en microsecondes (ESC / servo standard) */
constexpr uint16_t PWM_NEUTRAL_US       = 1500;

/** @brief Impulsion minimale en microsecondes */
constexpr uint16_t PWM_MIN_US           = 1000;

/** @brief Impulsion maximale en microsecondes */
constexpr uint16_t PWM_MAX_US           = 2000;

/* =========================================================================
 * SECTION 5 : CONFIGURATION RÉSEAU WI-FI ET POINT D'ACCÈS
 * ========================================================================= */

/** @brief SSID du point d'accès Wi-Fi autonome */
constexpr const char* AP_SSID           = "BOB-CONTROL_AP";

/** @brief Mot de passe du point d'accès (min. 8 caractères WPA2) */
constexpr const char* AP_PASSWORD       = "bobrov2026";

/** @brief Adresse IP statique du point d'accès */
constexpr const char* AP_IP             = "192.168.4.1";

/** @brief Canal Wi-Fi du point d'accès */
constexpr uint8_t  AP_CHANNEL           = 6;

/** @brief Nombre maximum de clients Wi-Fi simultanés */
constexpr uint8_t  AP_MAX_CLIENTS       = 4;

/* =========================================================================
 * SECTION 6 : CONFIGURATION DE LA LIAISON SÉRIE
 * ========================================================================= */

/** @brief Vitesse de la liaison série RPi5 (UART1) en bauds */
constexpr uint32_t SERIAL_BAUD_RATE     = 921600UL;

/** @brief Vitesse du port debug USB-CDC en bauds */
constexpr uint32_t DEBUG_BAUD_RATE      = 115200UL;

/**
 * @brief Broches UART1 pour la liaison série RPi5 (mode UART GPIO).
 *
 * Serial1 (UART1) est dédié au protocole binaire RPi5 ↔ ESP32-S3.
 * En mode USB-CDC, ces broches ne sont pas utilisées.
 * GPIO 1 = RX1 (données RPi → ESP32)
 * GPIO 2 = TX1 (données ESP32 → RPi)
 */
constexpr uint8_t  UART1_RX_PIN         = 1;
constexpr uint8_t  UART1_TX_PIN         = 2;

/** @brief Port UART utilisé pour la liaison RPi5 (1 = UART1) */
constexpr int      SERIAL_RPI_PORT      = 1;

/* =========================================================================
 * SECTION 7 : FRÉQUENCES ET PÉRIODES DES TÂCHES FREERTOS
 * ========================================================================= */

/** @brief Période de la tâche de contrôle (lecture trame + PWM) en ms */
constexpr uint32_t TASK_CONTROL_PERIOD_MS   = 10;

/** @brief Période de la tâche capteurs en ms (acquisition 100 Hz) */
constexpr uint32_t TASK_SENSORS_PERIOD_MS   = 10;

/**
 * @brief Période de lecture du baromètre MS5803 en ms (2 Hz).
 *
 * La conversion ADC du MS5803 (OSR 4096) impose ~9 ms par canal, soit ~20 ms
 * pour pression + température. Deux échantillons par seconde suffisent pour la
 * profondeur : la lecture est donc ralentie à 2 Hz et effectuée HORS mutex dans
 * _taskLoop, afin de ne plus bloquer la télémétrie WebSocket ni le PID.
 */
constexpr uint32_t MS5803_READ_PERIOD_MS    = 500;

/**
 * @brief Pression atmosphérique standard au niveau de la mer (mbar).
 * Référence de la formule barométrique internationale pour l'altitude réelle
 * affichée lorsque le capteur est hors de l'eau.
 */
constexpr float SEA_LEVEL_PRESSURE_MBAR     = 1013.25f;

/**
 * @brief Seuil d'immersion du MS5803 (mbar).
 * Si ΔP = P_mesurée − P_surface > ce seuil, le capteur est considéré immergé
 * (~15 cm d'eau douce). En dessous, il est « hors de l'eau » (mode altimètre).
 */
constexpr float MS5803_IMMERSION_THRESHOLD_MBAR = 15.0f;

/** @brief Masse volumique de l'eau douce (kg/m³) pour le calcul de profondeur */
constexpr float FRESH_WATER_DENSITY         = 1000.0f;

/** @brief Accélération gravitationnelle (m/s²) */
constexpr float GRAVITY_MSS                 = 9.81f;

/** @brief Période d'émission de la télémétrie montante en ms */
constexpr uint32_t TASK_SERIAL_TX_PERIOD_MS = 10;

/** @brief Période de broadcast WebSocket en ms */
constexpr uint32_t TASK_WS_BROADCAST_MS     = 50;

/* =========================================================================
 * SECTION 8 : TIMEOUTS ET SÉCURITÉ FAILSAFE
 * ========================================================================= */

/** @brief Timeout watchdog série en ms (défaut, paramétrable via NVS) */
constexpr uint32_t SERIAL_TIMEOUT_DEFAULT_MS = 500;

/** @brief Stack size pour les tâches FreeRTOS en octets */
constexpr uint32_t STACK_SIZE_SENSORS   = 4096;
constexpr uint32_t STACK_SIZE_SERIAL    = 4096;
constexpr uint32_t STACK_SIZE_CONTROL   = 4096;
constexpr uint32_t STACK_SIZE_WEB       = 8192;

/** @brief Priorités des tâches FreeRTOS */
constexpr uint8_t  PRIORITY_CONTROL     = 5;   ///< Tâche critique la plus haute
constexpr uint8_t  PRIORITY_SERIAL      = 4;
constexpr uint8_t  PRIORITY_SENSORS     = 3;
constexpr uint8_t  PRIORITY_WEB         = 2;

/** @brief Épinglage des cœurs (Core 0 = Wi-Fi, Core 1 = Temps réel) */
constexpr int      CORE_WIFI            = 0;
constexpr int      CORE_RT              = 1;

#endif /* CONFIG_H */
