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
 * SECTION 1 : CONFIGURATION I2C — DEUX BUS MATÉRIELS
 *
 * Bus n°1 (Wire,  GPIO 10/11) : BNO085 + 3× INA226 — propriété exclusive
 *                              de la tâche capteurs (Core 1).
 * Bus n°2 (Wire1, GPIO 6/7)   : MS5837 + PCA9685 — broches par défaut,
 *                              configurables en NVS (page Paramètres) ;
 *                              partagé entre la tâche capteurs (pression
 *                              2 Hz) et la tâche de contrôle (PWM 100 Hz) ;
 *                              chaque transaction est atomique et sérialisée
 *                              par le verrou interne de Wire1 (un par
 *                              instance TwoWire).
 *
 * Historique : le MS5837-30BA branché sur le même bus que le BNO085
 * perturbait ses lectures SHTP (tempête de NACK → horizon figé) — d'où la
 * séparation physique sur les deux contrôleurs I2C de l'ESP32-S3.
 * Chaque bus possède ses propres résistances de pull-up (4,7 kΩ vers 3V3).
 * ========================================================================= */

/** @brief Broche SDA du bus I2C n°1 — BNO085 + 3× INA226 (GPIO 10) */
constexpr uint8_t  I2C_SDA_PIN          = 10;

/** @brief Broche SCL du bus I2C n°1 — BNO085 + 3× INA226 (GPIO 11) */
constexpr uint8_t  I2C_SCL_PIN          = 11;

/** @brief Broche SDA du bus I2C n°2 — défaut GPIO 6 (clé NVS « i2c2_sda ») */
constexpr uint8_t  I2C_BUS2_SDA_PIN     = 6;

/** @brief Broche SCL du bus I2C n°2 — défaut GPIO 7 (clé NVS « i2c2_scl ») */
constexpr uint8_t  I2C_BUS2_SCL_PIN     = 7;

/**
 * @brief Fréquence du bus I2C en Hz (400 kHz Fast Mode).
 *
 * Valeur de la version de référence fonctionnelle (BNO085 + MS5803 + INA226
 * + PCA9685 stables). Un passage expérimental à 100 kHz avait coïncidé avec
 * des échecs d'initialisation du BNO085 — non retenu.
 */
constexpr uint32_t I2C_FREQ_HZ         = 400000UL;

/** @brief Numéro du port I2C matériel utilisé */
constexpr int      I2C_PORT_NUM         = 0;

/**
 * @brief Timeout matériel I2C en ms (>= 50 ms).
 * Laisse au coprocesseur ARM du BNO085 le temps d'étirer l'horloge pendant ses
 * traitements internes, sans abandonner prématurément la transaction.
 */
constexpr uint32_t I2C_TIMEOUT_MS      = 50UL;

/** @brief Délai de boot BNO085 (ms) avant la 1re commande I2C après mise sous tension */
constexpr uint32_t BNO085_BOOT_DELAY_MS = 1000UL;  /* le coprocesseur ARM du BNO085 démarre en ~1 s */

/** @brief Nombre maximal de tentatives d'initialisation du BNO085 */
constexpr uint8_t  BNO085_INIT_MAX_ATTEMPTS = 5;

/** @brief Délai (ms) entre deux tentatives d'initialisation du BNO085 */
constexpr uint32_t BNO085_INIT_RETRY_DELAY_MS = 200UL;

/** @brief Nombre max de rapports SH-2 lus par interrogation (borne la boucle de drain) */
constexpr uint8_t  BNO085_MAX_EVENTS_PER_READ = 8;

/**
 * @brief Timeout (ms) sans donnée BNO085 avant auto-récupération.
 * Si le capteur est connecté mais ne renvoie plus aucun rapport (bus bloqué,
 * ESP_ERR_INVALID_STATE après NACK), on tente une ré-init du capteur puis, si
 * besoin, une réinitialisation logicielle propre du bus I2C.
 */
constexpr uint32_t BNO085_STALL_TIMEOUT_MS = 1000UL;

/**
 * @brief Fraîcheur maximale (ms) du Rotation Vector pour rester source de cap.
 *
 * Au-delà (rapport magnétique jamais activé ou flux interrompu), le Game
 * Rotation Vector (relatif, sans magnéto) prend le relais comme source de cap.
 */
constexpr uint32_t BNO085_MAG_STALE_MS = 200UL;

/**
 * @brief Récupération BNO085 : nombre de tentatives douces avant reset bus direct.
 *
 * Les premières récupérations restent légères (ré-init SHTP du seul capteur) ;
 * dès que les échecs s'enchaînent, on passe d'emblée au reset complet du bus.
 */
constexpr uint8_t  BNO085_STREAK_BUS_RESET = 2;

/**
 * @brief Récupération BNO085 : nombre d'échecs avant « mode espacé » (muet).
 *
 * Au-delà, le capteur est déclaré muet : le polling SHTP est STOPPÉ (chaque NACK
 * de lecture coûte ~30 ms de log série bloquant et étouffe la tâche capteurs —
 * les INA226/MS5803 en paissent), badge rouge + message Web explicite, et une
 * tentative complète n'a plus lieu que toutes les BNO085_BACKOFF_MS.
 */
constexpr uint8_t  BNO085_STREAK_BACKOFF   = 5;

/** @brief Récupération BNO085 : période (ms) des tentatives en mode espacé */
constexpr uint32_t BNO085_BACKOFF_MS       = 30000UL;

/**
 * @brief Remappage de montage du BNO085 : rotation fixe autour de l'axe Z (°).
 *
 * Le capteur est monté tourné de 90° dans le plan horizontal du ROV : son axe
 * X pointe vers la DROITE du ROV (X+) et son axe Y vers l'ARRIÈRE (Y−), axe Z
 * vers le bas. Le quaternion SH-2 livrant l'attitude dans le repère du CAPTEUR,
 * l'attitude du ROV en était « tournée » : lever l'avant (Y+) faisait croître
 * le ROLL et lever la droite (X+) le PITCH (tangage/roulis permutés sur tous
 * les consommateurs), avec un cap décalé de 90° masqué par la tare Nord.
 *
 * _readBNO085() compose le quaternion à droite par une rotation de
 * BNO085_MOUNT_YAW_DEG autour de Z, AVANT le scalage et l'extraction des
 * Euler (correction unique à la source : télémétrie web, instruments, PID
 * et trame série RPi 5 partagent le même repère). Le repère véhicule obtenu
 * suit la convention aéronautique NED : X = avant (Y+ ROV), Y = droite
 * (X+ ROV), Z = bas — lever l'avant → pitch + ; penché à droite (droite qui
 * descend) → roll +.
 *
 * Valeurs : 0 = capteur aligné (X capteur vers l'avant) ; -90 = montage
 * actuel (X capteur vers la droite) ; +90 = montage tourné dans l'autre sens.
 */
constexpr float    BNO085_MOUNT_YAW_DEG   = -90.0f;

/**
 * @brief Watchdog I2C global : seuil d'échecs consécutifs avant reset du bus.
 *
 * Quand le bus I2C s'effondre (NACK → ESP_ERR_INVALID_STATE dans le driver
 * i2c-ng), TOUTES les transactions suivantes échouent en boucle tant que
 * Wire.end()/begin() n'est pas rejoué. Les lectures des wattmètres INA226
 * (600/s) servent de sonde : au-delà de ce seuil d'échecs consécutifs, on
 * déclenche un reset bus + réinitialisation de tous les capteurs présents.
 * Couvre le cas où le BNO085 n'a PAS été vu au boot (la supervision BNO085
 * seule ne déclenche alors jamais → NACK en boucle infinie, badge rouge).
 */
constexpr uint16_t I2C_WATCHDOG_FAIL_THRESHOLD = 30;

/** @brief Watchdog I2C : délai minimal (ms) entre deux reset bus (anti-spam) */
constexpr uint32_t I2C_WATCHDOG_COOLDOWN_MS = 5000UL;

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
 * @brief Nombre d'échecs de lecture MS5803 consécutifs avant déclaration de perte.
 *
 * Dès que le capteur est débranché, ses lectures NACK (~2/s). Rien dans
 * _readMS5803 ne le déconnecte : sans ce seuil, il resterait pollé en NACK
 * toutes les 500 ms indéfiniment — seules les chaînes de récupération bus
 * (qui repassent par _initMS5803) remarqueraient son absence.
 */
constexpr uint8_t  MS5803_LOST_AFTER_FAILS  = 3;

/** @brief Période (ms) de re-sonde d'un MS5803 déclaré perdu (rebranchage à chaud) */
constexpr uint32_t MS5803_REPROBE_MS        = 10000UL;

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

/**
 * @brief Stack de la tâche capteurs en octets.
 * Généreusement dimensionné : la chaîne d'auto-récupération (_recoverBNO085 →
 * _recoverI2CBus → _initBNO085 → begin_I2C → sh2_open) s'exécute désormais
 * dans cette tâche et consomme nettement plus de pile qu'une lecture simple.
 */
constexpr uint32_t STACK_SIZE_SENSORS   = 6144;

/** @brief Stack size pour les autres tâches FreeRTOS en octets */
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
