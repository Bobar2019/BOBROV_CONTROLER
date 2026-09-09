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
 * @author Didier Dero
 * @version 2.0.0
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
};

/** @brief Données de puissance des 3 wattmètres INA226 */
struct PowerData {
    uint16_t voltage[3];    ///< Tension en mV pour chaque wattmètre
    uint16_t current[3];    ///< Courant en mA pour chaque wattmètre
};

/** @brief Données de pression et température */
struct PressureData {
    int32_t pressure_mbar;  ///< Pression en dixièmes de mbar
    int32_t temperature;    ///< Température en centièmes de °C
    float   altitude_m;     ///< Altitude/profondeur en mètres (0 = surface, − = profondeur)
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

    /* -----------------------------------------------------------------
     * DIAGNOSTIC I2C — SCAN & CONFIGURATION DYNAMIQUE DES BROCHES
     * ----------------------------------------------------------------- */

    /**
     * @brief Scanne le bus I2C et retourne la liste des périphériques trouvés.
     */
    std::vector<I2CDeviceInfo> scanI2CBus();

    /**
     * @brief Réinitialise le bus I2C avec de nouvelles broches SDA/SCL.
     */
    void reinitI2C(uint8_t sda, uint8_t scl);

    /** @brief Retourne la broche SDA actuellement configurée */
    uint8_t getCurrentSDA() const { return _currentSDA; }

    /** @brief Retourne la broche SCL actuellement configurée */
    uint8_t getCurrentSCL() const { return _currentSCL; }

    /**
     * @brief Définit les broches I2C avant appel à begin().
     */
    void setCurrentPins(uint8_t sda, uint8_t scl) { _currentSDA = sda; _currentSCL = scl; }

private:
    /* -- État interne -- */
    IMUData         _imu;               ///< Données IMU courantes
    PowerData       _power;             ///< Données puissance courantes
    PressureData    _pressure;          ///< Données pression courantes
    SensorBankStatus _status;           ///< Statut de détection par composant
    SemaphoreHandle_t _mutex;           ///< Mutex de protection des données
    volatile bool   _i2cBusy;           ///< true = bus I2C réservé (scan/reinit)

    /* -- I2C bas niveau -- */
    uint8_t _currentSDA;               ///< Broche SDA active
    uint8_t _currentSCL;               ///< Broche SCL active
    void _i2cWrite(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t _i2cRead8(uint8_t addr, uint8_t reg);
    uint16_t _i2cRead16(uint8_t addr, uint8_t reg);
    bool _i2cDevicePresent(uint8_t addr);

    /* -- BNO085 via bibliothèque Adafruit -- */
    Adafruit_BNO08x* _bno08x;          ///< Instance BNO08x (null si non initialisé)

    /* -- Scanner I2C -- */
    void _scanI2CBus();

    /* -- Initialisation capteurs individuels -- */
    bool _initBNO085();
    bool _initMS5803();
    bool _initINA226(uint8_t addr, uint8_t index);

    /* -- Lecture capteurs réels (HAL routage) -- */
    void _readIMU();
    void _readPressure();
    void _readPower();
    void _readBNO085();
    void _readMS5803();
    void _readINA226();

    /* -- Zéros en mode réel pour capteurs absents -- */
    void _zeroMissingSensors();

    /* -- Calibrage MS5803 -- */
    uint16_t _ms5803_cal[6];            ///< Coefficients de calibration C1-C6

    /* -- Tâche FreeRTOS -- */
    static void _taskEntry(void* param);
    void _taskLoop();
};

/* Instance globale extern (définie dans sensor_driver.cpp) */
extern SensorDriver g_sensors;

#endif /* SENSOR_DRIVER_H */
