/**
 * @file sensor_driver.h
 * @brief Pilote I2C multi-capteurs avec abstraction HAL et moteur de simulation.
 *
 * Couche d'abstraction matérielle (HAL) unifiée supportant plusieurs modèles
 * de capteurs interchangeables :
 *   - IMU : BNO085 (0x4A) et MPU9250/GY-91 (0x68 + magnéto 0x0C)
 *   - Baro : MS5803-30BA (0x76) et BME280/BMP280 (0x76 ou 0x77)
 *   - Wattmètres : 3× INA226 (0x41, 0x44, 0x45)
 *   - PWM : PCA9685 (0x40)
 *
 * Scanner dynamique I2C au boot, sélection manuelle/auto via NVS,
 * forçage à zéro en mode réel si capteur absent.
 *
 * @author Didier Dero
 * @version 1.1.0
 * @date Août 2026
 */

#ifndef SENSOR_DRIVER_H
#define SENSOR_DRIVER_H

#include <Arduino.h>
#include <stdint.h>
#include <vector>
#include "config.h"

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
    float   depth_m;        ///< Profondeur estimée en mètres
};

/**
 * @brief Statut de détection de chaque composant matériel.
 *
 * Maintient l'état individuel de chaque périphérique I2C
 * pour affichage dans l'interface Web (badges LED).
 */
struct SensorBankStatus {
    SensorStatus bno085;        ///< IMU BNO085
    SensorStatus mpu9250;       ///< IMU MPU9250 / GY-91
    SensorStatus ms5803;        ///< Baromètre MS5803-30BA
    SensorStatus bme280;        ///< Baromètre BME280 / BMP280
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
 * @brief Classe de gestion des capteurs I2C avec HAL et simulateur.
 *
 * Fournit un accès thread-safe aux données capteurs via mutex FreeRTOS.
 * La commutation Mode Réel ↔ Mode Simulateur se fait à chaud.
 * En mode Réel, les capteurs non détectés ont leurs valeurs forcées à 0.
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
     * 4. Initialisation des capteurs selon profil NVS / détection
     * 5. Bascule auto en mode simulateur si aucun capteur détecté
     * 6. Lancement de la tâche FreeRTOS d'acquisition sur Core 1
     *
     * @param imuProfile   Profil IMU sélectionné (AUTO, BNO085, MPU9250)
     * @param baroProfile  Profil baro sélectionné (AUTO, MS5803, BME280)
     */
    void begin(IMUProfile imuProfile = IMU_TYPE_AUTO,
               BaroProfile baroProfile = BARO_TYPE_AUTO);

    /** @brief Active ou désactive la simulation de l'IMU (MPU9250) */
    void setSimIMU(bool enabled);
    /** @brief Retourne true si l'IMU est en mode simulé */
    bool isSimIMU() const;

    /** @brief Active ou désactive la simulation du baromètre (BME280/MS5803) */
    void setSimPressure(bool enabled);
    /** @brief Retourne true si le baromètre est en mode simulé */
    bool isSimPressure() const;

    /** @brief Active ou désactive la simulation des wattmètres (INA226) */
    void setSimPower(bool enabled);
    /** @brief Retourne true si les wattmètres sont en mode simulé */
    bool isSimPower() const;

    /** @brief Retourne true si AU MOINS un capteur est en mode simulé (compatibilité) */
    bool isSimMode() const;

    /** @brief Copie thread-safe des données IMU courantes */
    void getIMUData(IMUData& out);

    /** @brief Copie thread-safe des données de puissance courantes */
    void getPowerData(PowerData& out);

    /** @brief Copie thread-safe des données de pression courantes */
    void getPressureData(PressureData& out);

    /** @brief Copie thread-safe du statut de détection des capteurs */
    void getSensorStatus(SensorBankStatus& out);

    /** @brief Retourne le profil IMU actif */
    IMUProfile getActiveIMU() const { return _activeIMU; }

    /** @brief Retourne le profil baro actif */
    BaroProfile getActiveBaro() const { return _activeBaro; }

    /** @brief Retourne true si le magnétomètre AK8963 du MPU9250 est accessible (9-DOF) */
    bool isMagReady() const { return _magReady; }

    /** @brief Enregistre le Yaw actuel comme référence (Tare Nord).
     *  Retourne l'offset appliqué en degrés. */
    float tareYaw();

    /** @brief Retourne l'offset Yaw actuel (degrés) */
    float getYawOffset() const { return _yawOffset; }

    /** @brief Change le profil IMU à chaud (réinitialise la détection) */
    void setIMUProfile(IMUProfile profile);

    /** @brief Change le profil baro à chaud (réinitialise la détection) */
    void setBaroProfile(BaroProfile profile);

    /* -----------------------------------------------------------------
     * FORÇAGE MANUEL DU CAP MAGNÉTIQUE (YAW)
     * ----------------------------------------------------------------- */

    /**
     * @brief Active ou désactive le forçage manuel du cap (Yaw).
     *
     * Quand activé, le Yaw courant rampe de façon fluide vers la consigne
     * (vitesse max ~45°/s) avec gestion du passage par le Nord (359°↔0°).
     * Les quaternions et angles Euler sont recalculés et propagés dans la
     * trame montante vers le RPi 5.
     *
     * @param targetDeg  Cap cible en degrés [0..360).
     * @param enabled    true pour activer, false pour désactiver.
     */
    void setHeadingOverride(float targetDeg, bool enabled);

    /** @brief Retourne true si le forçage de cap est actif */
    bool isHeadingOverride() const;

    /** @brief Retourne le cap cible courant en degrés */
    float getHeadingTarget() const;

    /** @brief Retourne le cap actuel (après rampe) en degrés */
    float getHeadingCurrent() const;

    /* -----------------------------------------------------------------
     * DIAGNOSTIC I2C — SCAN & CONFIGURATION DYNAMIQUE DES BROCHES
     * ----------------------------------------------------------------- */

    /**
     * @brief Scanne le bus I2C et retourne la liste des périphériques trouvés.
     *
     * Balaie les adresses 0x01 à 0x7F. Pour chaque ACK reçu, identifie
     * le composant probable (IMU, baromètre, wattmètre, PWM driver).
     *
     * @return Vecteur de I2CDeviceInfo pour chaque adresse ACK.
     */
    std::vector<I2CDeviceInfo> scanI2CBus();

    /**
     * @brief Réinitialise le bus I2C avec de nouvelles broches SDA/SCL.
     *
     * Termine Wire, sauvegarde en NVS et relance avec les nouveaux GPIO.
     *
     * @param sda Nouvelle broche SDA.
     * @param scl Nouvelle broche SCL.
     */
    void reinitI2C(uint8_t sda, uint8_t scl);

    /** @brief Retourne la broche SDA actuellement configurée */
    uint8_t getCurrentSDA() const { return _currentSDA; }

    /** @brief Retourne la broche SCL actuellement configurée */
    uint8_t getCurrentSCL() const { return _currentSCL; }

    /**
     * @brief Définit les broches I2C avant appel à begin() (utilisé au boot depuis NVS).
     * @param sda Broche SDA.
     * @param scl Broche SCL.
     */
    void setCurrentPins(uint8_t sda, uint8_t scl) { _currentSDA = sda; _currentSCL = scl; }

private:
    /* -- État interne -- */
    volatile bool   _simIMU;            ///< true = IMU simulée
    volatile bool   _simPressure;       ///< true = baromètre simulé
    volatile bool   _simPower;          ///< true = wattmètres simulés
    IMUData         _imu;               ///< Données IMU courantes
    PowerData       _power;             ///< Données puissance courantes
    PressureData    _pressure;          ///< Données pression courantes
    SensorBankStatus _status;           ///< Statut de détection par composant
    SemaphoreHandle_t _mutex;           ///< Mutex de protection des données
    volatile bool   _i2cBusy;           ///< true = bus I2C réservé (scan/reinit)

    /* -- Profils actifs -- */
    IMUProfile  _activeIMU;             ///< Profil IMU résolu après détection
    BaroProfile _activeBaro;            ///< Profil baro résolu après détection

    /* -- I2C bas niveau -- */
    uint8_t _currentSDA;               ///< Broche SDA active
    uint8_t _currentSCL;               ///< Broche SCL active
    void _i2cWrite(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t _i2cRead8(uint8_t addr, uint8_t reg);
    uint16_t _i2cRead16(uint8_t addr, uint8_t reg);
    bool _i2cDevicePresent(uint8_t addr);

    /* -- Scanner I2C -- */
    void _scanI2CBus();

    /* -- Initialisation capteurs individuels -- */
    bool _initBNO085();
    bool _initMPU9250();
    bool _initMS5803();
    bool _initBME280();
    bool _initINA226(uint8_t addr, uint8_t index);

    /* -- Lecture capteurs réels (HAL routage) -- */
    void _readIMU();
    void _readPressure();
    void _readPower();
    void _readBNO085();
    void _readMPU9250();
    void _readMS5803();
    void _readBME280();
    void _readINA226();

    /* -- Zéros en mode réel pour capteurs absents -- */
    void _zeroMissingSensors();

    /* -- Forçage de cap : rampe fluide -- */
    void _applyHeadingOverride();           ///< Applique la rampe sur _imu.yaw + quaternions
    volatile bool   _headingEnabled;        ///< true = forçage actif
    volatile float  _headingTarget;         ///< Cap cible en degrés [0..360)
    float           _headingCurrent;        ///< Cap courant (rampe) en degrés
    unsigned long   _headingLastMs;         ///< Timestamp dernière itération de rampe

    /* -- Moteur de simulation -- */
    void _updateSimulation();
    void _simAttitude(float t);
    void _simDepth(float dt);
    void _simPowerData(float t);

    /* -- Tâche FreeRTOS -- */
    static void _taskEntry(void* param);
    void _taskLoop();

    /* -- Calibrage MS5803 -- */
    uint16_t _ms5803_cal[6];            ///< Coefficients de calibration C1-C6

    /* -- Calibrage BME280 -- */
    bool _bme280Initialized;            ///< Flag d'initialisation BME280
    uint8_t _bme280Addr;                ///< Adresse I2C effective du BME280

    /* -- MPU9250 mode -- */
    volatile bool _magReady;            ///< true = magnétomètre AK8963 accessible (9-DOF)
    float _mpu9250Yaw;                  ///< Yaw accumulé en 6-DOF (intégration gyro Z)
    float _yawOffset = 0.0f;            ///< Offset de tare (degrés, soustrait au Yaw)
};

/* Instance globale extern (définie dans sensor_driver.cpp) */
extern SensorDriver g_sensors;

#endif /* SENSOR_DRIVER_H */
