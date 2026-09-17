/**
 * @file sensor_driver.h
 * @brief Pilote I2C multi-capteurs avec abstraction HAL.
 *
 * Capteurs supportés :
 *   - IMU : BNO085 (0x4A) via bibliothèque Adafruit BNO08x (protocole SH-2)
 *   - Baro : MS5803-30BA / MS5837 (0x76)
 *   - Wattmètres : 3× INA226 (0x41, 0x44, 0x45)
 *   - PWM : PCA9685 (0x40)
 *
 * Scanner dynamique I2C au boot, forçage à zéro si capteur absent.
 *
 * Deux bus I2C matériels, broches NVS-configurables : n°1 (Wire, GPIO 10/11
 * par défaut — BNO085 + INA226) et n°2 (Wire1, GPIO 6/7 par défaut —
 * MS5837 + PCA9685, partagé avec la tâche de contrôle).
 *
 * @author Didier Dero
 * @version 2.1.0
 * @date Septembre 2026
 */

#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <Arduino.h>
#include <stdint.h>
#include <vector>
#include "config.h"

/* Forward declaration de la bibliothèque BNO08x */
class Adafruit_BNO08x;

/* =========================================================================
 * STRUCTURES DE DONNÉES CAPTEURS
 * ========================================================================= */

/** @brief Données de la centrale inertielle (quaternions + gyroscope + Euler) */
struct IMUData {
    int16_t quat[4];    ///< Quaternions W, X, Y, Z scalés ×10000
    int16_t gyro[3];    ///< Vitesse angulaire X, Y, Z scalés ×100 (°/s)
    float   euler[3];   ///< Angles d'Euler Roll, Pitch, Yaw en degrés
    float   heading_offset; ///< Tare Nord : offset appliqué au yaw (°, ±180)
    uint8_t mag_active; ///< 1 = cap magnétique (Rotation Vector 9-DOF BNO085), 0 = secours relatif
    uint8_t mag_cal;    ///< Calibration SH-2 du magnétomètre (0–3 ; 3 = calibré)
    uint8_t mag_cfg_ok; ///< Diagnostic : dernier enableReport RV TRANSMIS (SH-2 n'accuse pas réception)
    uint32_t rv_reports;  ///< Diagnostic : rapports Rotation Vector reçus depuis l'init
    uint32_t grv_reports; ///< Diagnostic : rapports Game Rotation Vector reçus depuis l'init
};

/** @brief Données de puissance des 3 wattmètres INA226 */
struct PowerData {
    uint16_t voltage[3];    ///< Tension en mV pour chaque wattmètre
    uint16_t current[3];    ///< Courant en mA pour chaque wattmètre
};

/** @brief Données de pression, profondeur et altitude (MS5803) */
struct PressureData {
    int32_t pressure_mbar;  ///< Pression absolue en dixièmes de mbar
    int32_t temperature;    ///< Température en centièmes de °C
    float   altitude_m;     ///< Repère vertical unifié (m) : 0 hors de l'eau, − = profondeur (utilisé par le PID)
    float   surface_mbar;   ///< Pression de surface étalonnée (tare, mbar)
    float   depth_m;        ///< Profondeur eau douce (m, ≥ 0) ; 0 si hors de l'eau
    float   baro_alt_m;     ///< Altitude barométrique réelle (m) ; figée en immersion
    bool    immersed;       ///< true si ΔP > seuil d'immersion (capteur sous l'eau)
};

/**
 * @brief Statut de détection de chaque composant matériel.
 */
struct SensorBankStatus {
    SensorStatus bno085;        ///< IMU BNO085
    SensorStatus ms5803;        ///< Baromètre MS5803-30BA / MS5837
    SensorStatus ina226[3];     ///< 3× Wattmètres INA226
    SensorStatus pca9685;       ///< Contrôleur PWM PCA9685
};

/**
 * @brief Résultat du scan I2C pour un périphérique détecté.
 */
struct I2CDeviceInfo {
    uint8_t address;        ///< Adresse I2C (7 bits, 0x01–0x7F)
    uint8_t bus;            ///< Bus I2C : 1 = capteurs (Wire) | 2 = MS5837 + PCA9685 (Wire1)
    char    hexStr[8];      ///< Adresse en hexadécimal (ex: "0x68")
    char    name[48];       ///< Nom du composant probable
};

/* =========================================================================
 * CLASSE SENSORDRIVER
 * ========================================================================= */

/**
 * @brief Classe de gestion des capteurs I2C avec HAL.
 *
 * Fournit un accès thread-safe aux données capteurs via mutex FreeRTOS.
 * Les capteurs non détectés ont leurs valeurs forcées à 0.
 */
class SensorDriver {
public:
    SensorDriver();

    /**
     * @brief Initialise le bus I2C, scanne les capteurs et lance la tâche.
     *
     * Séquence :
     * 1. Création du mutex FreeRTOS
     * 2. Initialisation du bus I2C (SDA, SCL, 400 kHz)
     * 3. Scanner I2C pour détecter les puces présentes
     * 4. Initialisation BNO085 et MS5803
     * 5. Lancement de la tâche FreeRTOS d'acquisition sur Core 1
     */
    void begin();

    /** @brief Copie thread-safe des données IMU courantes */
    void getIMUData(IMUData& out);

    /** @brief Copie thread-safe des données de puissance courantes */
    void getPowerData(PowerData& out);

    /** @brief Copie thread-safe des données de pression courantes */
    void getPressureData(PressureData& out);

    /** @brief Copie thread-safe du statut de détection des capteurs */
    void getSensorStatus(SensorBankStatus& out);

    /**
     * @brief Demande un nouvel étalonnage de la pression de surface (tare).
     *
     * La pression ambiante courante sera mémorisée comme P_surface à la
     * prochaine lecture MS5803. À effectuer capteur HORS de l'eau. Thread-safe
     * (simple signal volatile lu par la tâche capteurs).
     */
    void requestSurfaceTare();

    /**
     * @brief Demande une remise à zéro du cap sur le Nord réel (tare Nord).
     *
     * Le cap courant (brut) sera mémorisé comme nouveau zéro à la prochaine
     * lecture BNO085 : l'offset est ensuite appliqué à la source, si bien que
     * télémétrie, HSI, modèle 3D et PID de cap partagent le même référentiel.
     * À effectuer ROV immobile, nez pointé vers le Nord réel. Thread-safe
     * (simple signal volatile lu par la tâche capteurs).
     */
    void requestNorthTare();

    /**
     * @brief Dernière erreur d'initialisation du BNO085 ("" si OK).
     *
     * Exposée à l'interface Web pour diagnostiquer un badge rouge sans câble
     * USB : "begin_I2C(0x4A) echoue" ou "enableReport GAME_ROT echoue".
     */
    String getLastBNO085Error();

    /**
     * @brief true si le flux de quaternions BNO085 est mort (stall en cours).
     *
     * Vrai lorsque le capteur était présent/initialisé et a déjà fourni au
     * moins un quaternion depuis le boot, mais qu'aucun nouveau quaternion
     * n'est arrivé depuis plus de BNO085_STALL_TIMEOUT_MS : récupération
     * (niveaux 1/2) en cours ou redémarrage de niveau 3 imminent.
     * Utilisé par la tâche de contrôle pour verrouiller les propulseurs au
     * neutre pendant toute la phase de récupération.
     */
    bool isBNO085Stalled() const;

    /* -----------------------------------------------------------------
     * DIAGNOSTIC I2C — SCAN & CONFIGURATION DYNAMIQUE DES BROCHES
     * ----------------------------------------------------------------- */

    /**
     * @brief Scanne le bus I2C et retourne la liste des périphériques trouvés.
     */
    std::vector<I2CDeviceInfo> scanI2CBus();

    /** @brief Retourne la broche SDA actuellement configurée */
    uint8_t getCurrentSDA() const { return _currentSDA; }

    /** @brief Retourne la broche SCL actuellement configurée */
    uint8_t getCurrentSCL() const { return _currentSCL; }

    /** @brief Retourne la broche SDA du bus n°2 (Wire1) actuellement configurée */
    uint8_t getCurrent2SDA() const { return _current2SDA; }

    /** @brief Retourne la broche SCL du bus n°2 (Wire1) actuellement configurée */
    uint8_t getCurrent2SCL() const { return _current2SCL; }

    /**
     * @brief Enregistre les broches d'un bus I2C en NVS (appliquées au boot).
     *
     * @param bus 1 = bus capteurs (Wire) | 2 = bus MS5837 + PCA9685 (Wire1)
     *
     * Écriture vérifiée par relecture. Aucune application à chaud : le bus n°2
     * est partagé avec la tâche de contrôle (PCA9685 @ 100 Hz — un
     * Wire1.end()/begin() en vol couperait les sorties PWM) et le bus n°1
     * suit la même procédure pour une interface uniforme. L'appelant programme
     * le redémarrage.
     */
    bool saveBusPins(uint8_t bus, uint8_t sda, uint8_t scl);

private:
    /* -- État interne -- */
    IMUData         _imu;               ///< Données IMU courantes
    PowerData       _power;             ///< Données puissance courantes
    PressureData    _pressure;          ///< Données pression courantes
    SensorBankStatus _status;           ///< Statut de détection par composant
    SemaphoreHandle_t _mutex;           ///< Mutex de protection des données

    /* -- File de requêtes bus I2C (tâche Web → tâche capteurs) --
     * La tâche Web ne touche JAMAIS le bus directement : elle poste une requête
     * et attend ; la tâche capteurs (unique propriétaire du bus) l'exécute au
     * prochain passage de boucle. Élimine la course critique où le bit-bang SCL
     * du scan interrompait une transaction en cours → effondrement du bus en
     * NACK / ESP_ERR_INVALID_STATE permanent. */
    volatile bool   _scanReq;           ///< Scan I2C demandé par la tâche Web
    volatile bool   _scanDone;          ///< Résultat du scan prêt à être lu
    std::vector<I2CDeviceInfo> _scanResult; ///< Dernier résultat de scan (handshake _scanDone)

    /* -- I2C bas niveau -- */
    uint8_t _currentSDA;               ///< Broche SDA active (bus n°1)
    uint8_t _currentSCL;               ///< Broche SCL active (bus n°1)
    uint8_t _current2SDA;              ///< Broche SDA active (bus n°2 / Wire1)
    uint8_t _current2SCL;              ///< Broche SCL active (bus n°2 / Wire1)
    void _i2cWrite(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t _i2cRead8(uint8_t addr, uint8_t reg);
    uint16_t _i2cRead16(uint8_t addr, uint8_t reg);
    bool _i2cDevicePresent(uint8_t addr);

    /* -- I2C n°2 bas niveau (Wire1 : MS5837 + PCA9685) -- */
    void _i2c2Write(uint8_t addr, uint8_t b1, uint8_t b2);   ///< Écriture 2 octets (commande + valeur)
    uint16_t _i2c2Read16(uint8_t addr, uint8_t reg);         ///< Lecture 16 bits (registre)
    bool _i2c2DevicePresent(uint8_t addr);                   ///< Sonde de présence

    /* -- BNO085 via bibliothèque Adafruit -- */
    Adafruit_BNO08x* _bno08x;          ///< Instance BNO08x (null si non initialisé)

    /* -- Scanner I2C -- */
    void _scanI2CBus();

    /* -- Requêtes bus de la tâche Web, exécutées par la tâche capteurs -- */
    void _processBusRequests();               ///< Exécute les requêtes scan en attente
    void _doScan();                           ///< Corps du scan : récupération bus + scan complet

    /* -- Initialisation capteurs individuels -- */
    bool _initBNO085();
    bool _initMS5803();
    bool _initINA226(uint8_t addr, uint8_t index);

    /* -- Lecture capteurs réels (HAL routage) -- */
    void _readIMU();
    void _readPower();
    void _readBNO085();
    bool _readMS5803(PressureData& out);   ///< Conversion ADC + compensation dans `out` ; true si lecture valide
    void _readINA226();

    /* -- Supervision / auto-récupération BNO085 (sans broche INT) -- */
    void _recoverBNO085();              ///< Ré-init capteur puis bus I2C si nécessaire (hors mutex)
    void _bno085EmergencyRestart(const char* reason);  ///< Niveau 3 : neutre propulsion + esp_restart() (ne retourne pas)
    void _resetI2CBus();                ///< Réinit logicielle propre du bus I2C (9 impulsions SCL)

    /* -- Zéros en mode réel pour capteurs absents -- */
    void _zeroMissingSensors();

    /* -- Calibrage MS5803 -- */
    uint16_t _ms5803_cal[6];            ///< Coefficients de calibration C1-C6
    uint32_t _lastMS5803Read;           ///< Horodatage (ms) dernière lecture/sonde MS5803 (2 Hz ; 10 s si perdu)
    uint8_t  _ms5803ConsecFails;        ///< Lectures MS5803 consécutives en échec (détection de perte)

    /* -- Tare surface / détection d'immersion MS5803 -- */
    float   _surfaceMbar;               ///< Pression de surface étalonnée (tare, mbar)
    volatile bool _tareRequested;       ///< true = tare à (re)capturer à la prochaine lecture
    float   _lastBaroAlt;               ///< Dernière altitude barométrique (figée en immersion)

    /* -- Tare Nord : remise à zéro du cap sur le Nord réel (bouton interface) -- */
    float   _headingOffset;             ///< Offset de cap appliqué au yaw (°, ±180)
    volatile bool _northTareRequested;  ///< true = capturer le cap brut comme nouveau zéro Nord

    /* -- Sources de quaternions BNO085 : Rotation Vector (magnétique) prioritaire,
     *       Game Rotation Vector (relatif) en secours — choix dans _readBNO085 -- */
    float    _rvQuat[4];        ///< Dernier quaternion Rotation Vector (9-DOF magnétique)
    float    _grvQuat[4];       ///< Dernier quaternion Game Rotation Vector (relatif)
    uint32_t _rvTime;           ///< Horodatage (ms) du dernier rapport Rotation Vector
    uint32_t _grvTime;          ///< Horodatage (ms) du dernier rapport Game Rotation Vector
    uint8_t  _rvStatus;         ///< Calibration SH-2 du Rotation Vector (0–3)
    bool     _rvCfgOk;          ///< Dernier enableReport Rotation Vector transmis avec succès
    uint32_t _rvReports;        ///< Rapports Rotation Vector reçus (diagnostic)
    uint32_t _grvReports;       ///< Rapports Game Rotation Vector reçus (diagnostic)
    uint32_t _rvLastRetryMs;    ///< Dernière ré-activation RV (auto-guérison, throttle 5 s)

    /* -- Buffers de travail (accès exclusif tâche capteurs ; publiés sous mutex) -- */
    IMUData   _imuWork;                 ///< Accumulateur IMU (écrit hors mutex par la tâche capteurs)
    PowerData _powerWork;               ///< Accumulateur puissance (écrit hors mutex par la tâche capteurs)

    /* -- Supervision BNO085 sans INT : cadencement + auto-récupération -- */
    bool     _bno085Present;            ///< BNO085 vu sur le bus au boot (à superviser en continu)
    uint32_t _lastBNO085Event;          ///< Horodatage (ms) dernier QUATERNION reçu / dernière récupération
    uint32_t _lastBNO085QuatTime;       ///< Horodatage (ms) dernier QUATERNION RÉEL — jamais réarmé par les tentatives ; 0 = jamais fonctionné depuis le boot (garde-fou anti-boucle du niveau 3)
    uint32_t _lastBNO085Attempt;        ///< Horodatage (ms) de fin de la dernière tentative de récupération
    uint8_t  _bno085FailStreak;         ///< Tentatives consécutives n'ayant PAS restauré le flux de quaternions
    uint16_t _bno085Recoveries;         ///< Compteur de récupérations (diagnostic)
    char     _bno085Err[48];            ///< Dernière erreur d'init BNO085 (diagnostic Web)

    /* -- Diagnostic BNO085 -- */
    void _setBNO085Err(const char* msg);   ///< Mémorise la dernière erreur d'init (sous mutex)

    /* -- Watchdog I2C global (échecs en cascade → reset bus + réinit capteurs) -- */
    uint16_t _i2cConsecFails;           ///< Échecs I2C consécutifs (toutes lectures confondues)
    uint32_t _lastBusRecovery;          ///< Horodatage (ms) du dernier reset bus watchdog
    bool     _busHadSensors;            ///< Au moins un capteur vu au boot (arme le watchdog « bus mort »)
    bool _anySensorConnected() const;   ///< true si au moins un capteur est CONNECTED
    void _recoverI2CBus();              ///< Reset bus + réinitialisation de tous les capteurs présents

    /* -- Tâche FreeRTOS -- */
    static void _taskEntry(void* param);
    void _taskLoop();
};

/* Instance globale extern (définie dans sensor_driver.cpp) */
extern SensorDriver g_sensors;

#endif /* SENSOR_DRIVER_H */
