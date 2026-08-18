/**
 * @file sensor_driver.cpp
 * @brief Implémentation du pilote I2C multi-capteurs avec HAL et simulation.
 *
 * Scanner I2C dynamique, abstraction HAL pour IMU (BNO085/MPU9250) et
 * baromètre (MS5803/BME280), forçage à zéro en mode réel si capteur absent.
 *
 * @author Didier Dero
 * @version 1.1.0
 */

#include "sensor_driver.h"
#include "config.h"
#include <math.h>
#include <Wire.h>
#include <Preferences.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */
SensorDriver g_sensors;

/* =========================================================================
 * CONSTANTES DE SIMULATION
 * ========================================================================= */
static constexpr float SIM_SWELL_PERIOD     = 6.0f;
static constexpr float SIM_ROLL_AMP         = 5.0f;
static constexpr float SIM_PITCH_AMP        = 3.0f;
static constexpr float SIM_DEPTH_INIT       = 2.0f;
static constexpr uint16_t SIM_BATT_NOMINAL_MV = 12600;
static constexpr uint16_t SIM_BATT_MIN_MV     = 10800;

/* =========================================================================
 * CONSTANTES FORÇAGE DE CAP (HEADING OVERRIDE)
 * ========================================================================= */

/**
 * @brief Vitesse maximale de rotation pour la rampe de cap (°/s).
 *
 * Fixée à 45°/s pour un mouvement fluide et réaliste sans saut brusque.
 * À 100 Hz (période 10 ms), cela représente 0.45° par itération.
 */
static constexpr float HEADING_MAX_RATE_DEG_S = 45.0f;

/* =========================================================================
 * REGISTRES MPU9250
 * ========================================================================= */
/** @brief Registre WHO_AM_I du MPU9250 (doit retourner 0x71 ou 0x73) */
static constexpr uint8_t MPU9250_WHOAMI_REG   = 0x75;
/** @brief Registre de gestion de l'alimentation et du reset */
static constexpr uint8_t MPU9250_PWR_MGMT1    = 0x6B;
/** @brief Registre de configuration du gyroscope */
static constexpr uint8_t MPU9250_GYRO_CONFIG  = 0x1B;
/** @brief Registre de configuration de l'accéléromètre */
static constexpr uint8_t MPU9250_ACCEL_CONFIG = 0x1C;
/** @brief Premier registre de données accéléro (AX_H) */
static constexpr uint8_t MPU9250_ACCEL_XOUT_H = 0x3B;
/** @brief Premier registre de données gyroscope (GX_H) */
static constexpr uint8_t MPU9250_GYRO_XOUT_H  = 0x43;
/** @brief Registre de contrôle utilisateur (I2C Master enable, reset) */
static constexpr uint8_t MPU9250_USER_CTRL    = 0x6A;
/** @brief Registre configuration broche INT (Bypass I2C) */
static constexpr uint8_t MPU9250_INT_PIN_CFG  = 0x37;
/** @brief Registre WIA (WHO_AM_I) de l'AK8963 (doit retourner 0x48) */
static constexpr uint8_t AK8963_WIA_REG       = 0x00;
/** @brief Registre de contrôle du magnétomètre AK8963 */
static constexpr uint8_t AK8963_CNTL1         = 0x0A;
/** @brief Registre contrôle 2 (reset) du magnétomètre AK8963 */
static constexpr uint8_t AK8963_CNTL2         = 0x0B;

/* =========================================================================
 * REGISTRES BME280
 * ========================================================================= */
/** @brief Registre ID du BME280 (doit retourner 0x60) */
static constexpr uint8_t BME280_ID_REG     = 0xD0;
/** @brief Registre de contrôle humidité */
static constexpr uint8_t BME280_CTRL_HUM   = 0xF2;
/** @brief Registre de contrôle mesure */
static constexpr uint8_t BME280_CTRL_MEAS  = 0xF4;
/** @brief Registre de configuration */
static constexpr uint8_t BME280_CONFIG_REG = 0xF5;
/** @brief Premier registre de données pression */
static constexpr uint8_t BME280_PRESS_REG  = 0xF7;

/* Calibration BME280 (registres 0x88-0x9F et 0xE1-0xE7) */
static uint16_t _bme280_dig_T1;
static int16_t  _bme280_dig_T2, _bme280_dig_T3;
static uint16_t _bme280_dig_P1;
static int16_t  _bme280_dig_P2, _bme280_dig_P3, _bme280_dig_P4;
static int16_t  _bme280_dig_P5, _bme280_dig_P6, _bme280_dig_P7;
static int16_t  _bme280_dig_P8, _bme280_dig_P9;
static int32_t  _bme280_t_fine;  ///< Variable interne de compensation

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

SensorDriver::SensorDriver()
    : _simMode(true)
    , _mutex(nullptr)
    , _activeIMU(IMU_TYPE_AUTO)
    , _activeBaro(BARO_TYPE_AUTO)
    , _headingEnabled(false)
    , _headingTarget(0.0f)
    , _headingCurrent(0.0f)
    , _headingLastMs(0)
    , _bme280Initialized(false)
    , _bme280Addr(BME280_I2C_ADDR_PRI)
{
    memset(&_imu, 0, sizeof(_imu));
    memset(&_power, 0, sizeof(_power));
    memset(&_pressure, 0, sizeof(_pressure));
    memset(&_status, 0, sizeof(_status));
    memset(_ms5803_cal, 0, sizeof(_ms5803_cal));
    _imu.quat[0] = 10000; /* Quaternion identité W=10000 */
    _currentSDA = I2C_SDA_PIN;
    _currentSCL = I2C_SCL_PIN;
    _magReady = false;
    _mpu9250Yaw = 0.0f;
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

void SensorDriver::begin(IMUProfile imuProfile, BaroProfile baroProfile) {
    _mutex = xSemaphoreCreateMutex();
    _activeIMU  = imuProfile;
    _activeBaro = baroProfile;

    /* Initialisation du bus I2C matériel */
    Wire.begin(_currentSDA, _currentSCL);
    Wire.setClock(I2C_FREQ_HZ);
    delay(100);

    Serial.println("[SENSORS] Bus I2C initialisé (SDA=" + String(_currentSDA)
                   + ", SCL=" + String(_currentSCL) + ", 400 kHz)");

    /* Scanner le bus I2C pour détecter les puces présentes */
    _scanI2CBus();

    /* Initialisation de l'IMU selon le profil */
    bool imu_ok = false;
    if (_activeIMU == IMU_TYPE_AUTO) {
        /* AUTO : tester d'abord la présence physique avant l'init complète */
        bool bno_present = _i2cDevicePresent(BNO085_I2C_ADDR);
        bool mpu_present = _i2cDevicePresent(MPU9250_I2C_ADDR);

        if (bno_present && _initBNO085()) {
            _activeIMU = IMU_TYPE_BNO085;
            imu_ok = true;
        } else if (mpu_present && _initMPU9250()) {
            _activeIMU = IMU_TYPE_MPU9250;
            imu_ok = true;
        } else {
            /* Essayer BNO085 même si non détecté (cas edge) */
            if (_initBNO085()) { _activeIMU = IMU_TYPE_BNO085; imu_ok = true; }
            else if (_initMPU9250()) { _activeIMU = IMU_TYPE_MPU9250; imu_ok = true; }
        }
    } else if (_activeIMU == IMU_TYPE_BNO085) {
        if (_initBNO085()) imu_ok = true;
    } else if (_activeIMU == IMU_TYPE_MPU9250) {
        if (_initMPU9250()) imu_ok = true;
    }

    /* Marquer l'IMU non utilisée comme DISCONNECTED */
    if (_activeIMU == IMU_TYPE_BNO085) {
        _status.mpu9250 = SENSOR_DISCONNECTED;
    } else if (_activeIMU == IMU_TYPE_MPU9250) {
        _status.bno085 = SENSOR_DISCONNECTED;
    }
    if (!imu_ok) {
        Serial.println("[SENSORS] Aucune IMU détectée");
    }

    /* Initialisation du baromètre selon le profil */
    bool baro_ok = false;
    if (_activeBaro == BARO_TYPE_AUTO) {
        /* AUTO : tester BME280 EN PREMIER (WHO_AM_I fiable à 0xD0=0x60)
         * puis MS5803 en fallback (pas de WHO_AM_I, ACK seul = ambigu) */
        bool bme_present = _i2cDevicePresent(BME280_I2C_ADDR_PRI)
                        || _i2cDevicePresent(BME280_I2C_ADDR_SEC)
                        || _i2cDevicePresent(BMP280_I2C_ADDR_ALT);

        if (bme_present && _initBME280()) {
            _activeBaro = BARO_TYPE_BME280;
            baro_ok = true;
        } else if (_initMS5803()) {
            _activeBaro = BARO_TYPE_MS5803;
            baro_ok = true;
        } else if (_initBME280()) {
            /* Fallback BME280 même sans détection presence */
            _activeBaro = BARO_TYPE_BME280;
            baro_ok = true;
        }
    } else if (_activeBaro == BARO_TYPE_MS5803) {
        if (_initMS5803()) baro_ok = true;
    } else if (_activeBaro == BARO_TYPE_BME280) {
        if (_initBME280()) baro_ok = true;
    }

    if (_activeBaro == BARO_TYPE_MS5803) {
        _status.bme280 = SENSOR_DISCONNECTED;
    } else if (_activeBaro == BARO_TYPE_BME280) {
        _status.ms5803 = SENSOR_DISCONNECTED;
    }
    if (!baro_ok) {
        Serial.println("[SENSORS] Aucun baromètre détecté");
    }

    /* Initialisation des 3 wattmètres INA226 */
    _initINA226(INA226_1_I2C_ADDR, 0);
    _initINA226(INA226_2_I2C_ADDR, 1);
    _initINA226(INA226_3_I2C_ADDR, 2);

    /* Vérification du PCA9685 */
    if (_i2cDevicePresent(PCA9685_I2C_ADDR)) {
        _status.pca9685 = SENSOR_CONNECTED;
        Serial.println("[SENSORS] PCA9685 détecté (0x40)");
    } else {
        _status.pca9685 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] PCA9685 NON détecté");
    }

    /* Basculer en mode simu si aucune IMU ni baromètre */
    if (!imu_ok && !baro_ok) {
        Serial.println("[SENSORS] Aucun capteur principal → mode SIMULATEUR forcé");
        _simMode = true;
    }

    Serial.println("[SENSORS] IMU active : " + String(
        _activeIMU == IMU_TYPE_BNO085 ? "BNO085" :
        _activeIMU == IMU_TYPE_MPU9250 ? "MPU9250" : "AUTO(NONE)"));
    Serial.println("[SENSORS] Baro actif : " + String(
        _activeBaro == BARO_TYPE_MS5803 ? "MS5803" :
        _activeBaro == BARO_TYPE_BME280 ? "BME280" : "AUTO(NONE)"));

    /* Lancement de la tâche d'acquisition capteurs sur Core 1 */
    xTaskCreatePinnedToCore(
        _taskEntry, "Sensors", STACK_SIZE_SENSORS,
        this, PRIORITY_SENSORS, nullptr, CORE_RT
    );
}

/* =========================================================================
 * SCANNER I2C
 * ========================================================================= */

/**
 * @brief Scanne le bus I2C et log les adresses répondantes.
 *
 * Parcourt les adresses 0x03 à 0x77 et vérifie la présence
 * d'un périphérique via un beginTransmission/endTransmission.
 */
void SensorDriver::_scanI2CBus() {
    Serial.println("[SENSORS] === Scan I2C ===");
    uint8_t count = 0;
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[SENSORS]   0x");
            Serial.print(addr, HEX);
            Serial.println(" trouvé");
            count++;
        }
    }
    Serial.println("[SENSORS] " + String(count) + " périphérique(s) détecté(s)");
}

/**
 * @brief Identifie le composant probable à partir d'une adresse I2C.
 * @param addr Adresse I2C (7 bits).
 * @return Chaîne statique décrivant le composant.
 */
static const char* _identifyI2CDevice(uint8_t addr) {
    switch (addr) {
        case 0x0C: return "AK8963 (Magnétomètre MPU9250)";
        case 0x40: return "PCA9685 (Driver PWM 16 Ch)";
        case 0x41: return "INA226 #1 (Wattmètre RPi)";
        case 0x44: return "INA226 #2 (Wattmètre Aux)";
        case 0x45: return "INA226 #3 (Wattmètre Moteurs)";
        case 0x4A: return "BNO085 (IMU 9-DOF)";
        case 0x68: return "MPU9250 / MPU6050 (IMU)";
        case 0x69: return "MPU9250 (IMU, AD0=HIGH)";
        case 0x76: return "MS5803-30BA / BME280 (Pression)";
        case 0x77: return "BME280 / BMP280 (Pression)";
        default:   return "Périphérique inconnu";
    }
}

/**
 * @brief Identifie le composant par lecture de registres si nécessaire.
 * @param addr Adresse I2C (7 bits).
 * @param buf  Buffer de sortie pour le nom.
 * @param len  Taille du buffer.
 */
static void _identifyI2CDeviceDetail(uint8_t addr, char* buf, size_t len) {
    /* Lecture registre helper inline */
    auto readReg = [](uint8_t a, uint8_t reg) -> uint8_t {
        Wire.beginTransmission(a);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return 0xFF;
        Wire.requestFrom(a, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0xFF;
    };

    switch (addr) {
        case 0x68: {
            uint8_t whoami = readReg(addr, 0x75);
            if (whoami == 0x71 || whoami == 0x73 || whoami == 0x75)
                snprintf(buf, len, "MPU9250 (IMU 6/9-DOF, WHO=0x%02X)", whoami);
            else if (whoami == 0x68)
                snprintf(buf, len, "MPU6050 (IMU 6-DOF, WHO=0x68)");
            else
                snprintf(buf, len, "IMU inconnue (WHO=0x%02X)", whoami);
            break;
        }
        case 0x0C: {
            uint8_t wia = readReg(addr, 0x00);
            if (wia == 0x48)
                snprintf(buf, len, "AK8963 (Magnétomètre GY-91)");
            else
                snprintf(buf, len, "AK8963 (WIA=0x%02X)", wia);
            break;
        }
        case 0x76: {
            uint8_t id = readReg(addr, 0xD0);
            if (id == 0x60)
                snprintf(buf, len, "BME280 (Pression/Temp, ID=0x60)");
            else if (id == 0x58)
                snprintf(buf, len, "BMP280 (Pression/Temp, ID=0x58)");
            else
                snprintf(buf, len, "MS5803-30BA (Pression, regD0=0x%02X)", id);
            break;
        }
        case 0x77: {
            uint8_t id = readReg(addr, 0xD0);
            if (id == 0x60)
                snprintf(buf, len, "BME280 (Pression/Temp, ID=0x60)");
            else
                snprintf(buf, len, "BMP280 (Pression, ID=0x%02X)", id);
            break;
        }
        case 0x4C: {
            uint8_t id = readReg(addr, 0xD0);
            if (id == 0x58)
                snprintf(buf, len, "BMP280 (Pression/Temp, ID=0x58)");
            else if (id == 0x60)
                snprintf(buf, len, "BME280 (Pression/Temp, ID=0x60)");
            else
                snprintf(buf, len, "Périphérique inconnu (regD0=0x%02X)", id);
            break;
        }
        default: {
            const char* name = _identifyI2CDevice(addr);
            strncpy(buf, name, len - 1);
            buf[len - 1] = '\0';
            break;
        }
    }
}

/**
 * @brief Scan I2C public : active bypass MPU9250 puis balaie 0x01–0x7F.
 *
 * Si un MPU9250 est détecté à 0x68, le bypass I2C est activé AVANT le scan
 * pour rendre l'AK8963 (0x0C) visible. L'identification utilise la lecture
 * de registres WHO_AM_I / ID pour distinguer BME280 de MS5803.
 */
std::vector<I2CDeviceInfo> SensorDriver::scanI2CBus() {
    /* Activer le bypass MPU9250 pour révéler l'AK8963 (SANS reset) */
    Wire.beginTransmission(MPU9250_I2C_ADDR);
    if (Wire.endTransmission() == 0) {
        /* MPU présent → désactiver I2C master + activer bypass (pas de reset) */
        _i2cWrite(MPU9250_I2C_ADDR, MPU9250_USER_CTRL, 0x00);
        delay(10);
        _i2cWrite(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, 0x02);
        delay(50);
    }

    std::vector<I2CDeviceInfo> devices;
    for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            I2CDeviceInfo info;
            info.address = addr;
            snprintf(info.hexStr, sizeof(info.hexStr), "0x%02X", addr);
            _identifyI2CDeviceDetail(addr, info.name, sizeof(info.name));
            devices.push_back(info);
        }
    }
    return devices;
}

/**
 * @brief Réinitialise le bus I2C avec de nouvelles broches SDA/SCL.
 *
 * Séquence :
 * 1. Terminer Wire (Wire.end())
 * 2. Sauvegarder en NVS (Preferences)
 * 3. Mettre à jour _currentSDA/_currentSCL
 * 4. Relancer Wire.begin(sda, scl, 400kHz)
 */
void SensorDriver::reinitI2C(uint8_t sda, uint8_t scl) {
    Serial.println("[SENSORS] Réinit I2C : SDA=" + String(sda) + ", SCL=" + String(scl));

    /* Terminer le bus actuel */
    Wire.end();
    delay(50);

    /* Sauvegarder en NVS */
    Preferences prefs;
    prefs.begin("config", false);
    prefs.putUChar("i2c_sda", sda);
    prefs.putUChar("i2c_scl", scl);
    prefs.end();

    /* Mettre à jour les membres courants */
    _currentSDA = sda;
    _currentSCL = scl;

    /* Relancer le bus */
    Wire.begin(sda, scl);
    Wire.setClock(I2C_FREQ_HZ);
    delay(100);

    Serial.println("[SENSORS] Bus I2C réinitialisé (SDA=" + String(sda)
                   + ", SCL=" + String(scl) + ", 400 kHz)");
}

/* =========================================================================
 * CONTRÔLE DU MODE SIMULATEUR
 * ========================================================================= */

void SensorDriver::setSimMode(bool enabled) {
    _simMode = enabled;
    Serial.println(String("[SENSORS] Mode : ") + (enabled ? "SIMULATEUR" : "REEL"));
}

bool SensorDriver::isSimMode() const { return _simMode; }

/**
 * @brief Enregistre le Yaw actuel comme référence 0° (Tare Nord).
 *
 * Le cap actuel du MPU9250 devient le "nord" de référence.
 * L'offset est soustrait à chaque lecture dans _readMPU9250().
 *
 * @return L'offset appliqué en degrés.
 */
float SensorDriver::tareYaw() {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _yawOffset = _mpu9250Yaw;
        float offset = _yawOffset;
        xSemaphoreGive(_mutex);
        Serial.println("[SENSORS] Tare Nord : offset Yaw = " + String(offset, 1) + "°");
        return offset;
    }
    return _yawOffset;
}

void SensorDriver::setIMUProfile(IMUProfile profile) {
    Serial.println("[SENSORS] Changement profil IMU → " + String((uint8_t)profile));

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _activeIMU = profile;

        /* Réinitialiser l'IMU sélectionnée */
        bool ok = false;
        if (profile == IMU_TYPE_AUTO) {
            bool bno_present = _i2cDevicePresent(BNO085_I2C_ADDR);
            bool mpu_present = _i2cDevicePresent(MPU9250_I2C_ADDR);
            if (bno_present && _initBNO085()) {
                _activeIMU = IMU_TYPE_BNO085; ok = true;
            } else if (mpu_present && _initMPU9250()) {
                _activeIMU = IMU_TYPE_MPU9250; ok = true;
            } else {
                if (_initBNO085()) { _activeIMU = IMU_TYPE_BNO085; ok = true; }
                else if (_initMPU9250()) { _activeIMU = IMU_TYPE_MPU9250; ok = true; }
            }
        } else if (profile == IMU_TYPE_BNO085) {
            if (_initBNO085()) ok = true;
        } else if (profile == IMU_TYPE_MPU9250) {
            if (_initMPU9250()) ok = true;
        }

        /* Marquer l'autre IMU comme déconnectée */
        if (_activeIMU == IMU_TYPE_BNO085) _status.mpu9250 = SENSOR_DISCONNECTED;
        else if (_activeIMU == IMU_TYPE_MPU9250) _status.bno085 = SENSOR_DISCONNECTED;

        /* Sortir du mode simu si IMU détectée */
        if (ok) _simMode = false;

        xSemaphoreGive(_mutex);
        Serial.println("[SENSORS] IMU active : " + String(
            _activeIMU == IMU_TYPE_BNO085 ? "BNO085" :
            _activeIMU == IMU_TYPE_MPU9250 ? "MPU9250" : "NONE")
            + (ok ? " (OK)" : " (ÉCHEC)"));
    }
}

void SensorDriver::setBaroProfile(BaroProfile profile) {
    Serial.println("[SENSORS] Changement profil Baro → " + String((uint8_t)profile));

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _activeBaro = profile;

        bool ok = false;
        if (profile == BARO_TYPE_AUTO) {
            bool bme_present = _i2cDevicePresent(BME280_I2C_ADDR_PRI)
                            || _i2cDevicePresent(BME280_I2C_ADDR_SEC)
                            || _i2cDevicePresent(BMP280_I2C_ADDR_ALT);
            if (bme_present && _initBME280()) {
                _activeBaro = BARO_TYPE_BME280; ok = true;
            } else if (_initMS5803()) {
                _activeBaro = BARO_TYPE_MS5803; ok = true;
            } else if (_initBME280()) {
                _activeBaro = BARO_TYPE_BME280; ok = true;
            }
        } else if (profile == BARO_TYPE_MS5803) {
            if (_initMS5803()) ok = true;
        } else if (profile == BARO_TYPE_BME280) {
            if (_initBME280()) ok = true;
        }

        if (_activeBaro == BARO_TYPE_MS5803) _status.bme280 = SENSOR_DISCONNECTED;
        else if (_activeBaro == BARO_TYPE_BME280) _status.ms5803 = SENSOR_DISCONNECTED;

        if (ok) _simMode = false;

        xSemaphoreGive(_mutex);
        Serial.println("[SENSORS] Baro actif : " + String(
            _activeBaro == BARO_TYPE_MS5803 ? "MS5803" :
            _activeBaro == BARO_TYPE_BME280 ? "BME280" : "NONE")
            + (ok ? " (OK)" : " (ÉCHEC)"));
    }
}

/* =========================================================================
 * FORÇAGE MANUEL DU CAP MAGNÉTIQUE (YAW)
 * ========================================================================= */

/**
 * @brief Active ou désactive le forçage de cap (Yaw).
 *
 * À l'activation, initialise _headingCurrent au Yaw courant pour éviter
 * tout saut. La rampe démarre ensuite à la prochaine itération de la tâche.
 *
 * @param targetDeg  Cap cible en degrés [0..360).
 * @param enabled    true pour activer, false pour désactiver.
 */
void SensorDriver::setHeadingOverride(float targetDeg, bool enabled) {
    /* Normaliser dans [0..360) */
    while (targetDeg < 0.0f)   targetDeg += 360.0f;
    while (targetDeg >= 360.0f) targetDeg -= 360.0f;

    _headingTarget = targetDeg;

    if (enabled && !_headingEnabled) {
        /* Initialiser _headingCurrent au Yaw courant pour éviter un saut */
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _headingCurrent = _imu.euler[2];
            if (_headingCurrent < 0.0f) _headingCurrent += 360.0f;
            xSemaphoreGive(_mutex);
        }
        _headingLastMs = millis();
    }

    _headingEnabled = enabled;
    Serial.println(String("[SENSORS] Forçage cap : ") +
                   (enabled ? "ACTIVÉ → " + String((int)targetDeg) + "°" : "DÉSACTIVÉ"));
}

bool SensorDriver::isHeadingOverride() const { return _headingEnabled; }
float SensorDriver::getHeadingTarget() const { return _headingTarget; }
float SensorDriver::getHeadingCurrent() const { return _headingCurrent; }

/**
 * @brief Applique la rampe de cap sur les données IMU courantes.
 *
 * Algorithme :
 * 1. Calculer l'écart angulaire signé entre _headingCurrent et _headingTarget
 *    en passant par le plus court chemin (gestion 359°↔0°).
 * 2. Limiter le pas à HEADING_MAX_RATE_DEG_S × dt (45°/s max).
 * 3. Avancer _headingCurrent et normaliser dans [0..360).
 * 4. Écraser _imu.euler[2] (Yaw) avec la valeur rampée.
 * 5. Recalculer les quaternions depuis (Roll, Pitch, Yaw_override) pour
 *    propagation cohérente dans la trame montante vers le RPi 5.
 *
 * @note Appelée dans _taskLoop() sous mutex, après lecture IMU ou simulation.
 */
void SensorDriver::_applyHeadingOverride() {
    if (!_headingEnabled) return;

    /* Calcul du dt réel depuis le dernier appel */
    unsigned long now = millis();
    float dt = (now - _headingLastMs) / 1000.0f;
    _headingLastMs = now;
    if (dt > 0.1f) dt = 0.1f;   /* Plafonner pour éviter les gros sauts */
    if (dt <= 0.0f) return;

    /* ---- Écart angulaire signé (plus court chemin, gestion du Nord) ---- */
    float diff = _headingTarget - _headingCurrent;
    /* Ramener diff dans [-180..+180] */
    if (diff > 180.0f)  diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    /* ---- Rampe : limiter la vitesse à HEADING_MAX_RATE_DEG_S ---- */
    float maxStep = HEADING_MAX_RATE_DEG_S * dt;
    float step = diff;
    if (step > maxStep)  step = maxStep;
    if (step < -maxStep) step = -maxStep;

    /* Avancer le cap courant */
    _headingCurrent += step;
    /* Normaliser dans [0..360) */
    if (_headingCurrent < 0.0f)    _headingCurrent += 360.0f;
    if (_headingCurrent >= 360.0f) _headingCurrent -= 360.0f;

    /* ---- Écraser le Yaw Euler ---- */
    _imu.euler[2] = _headingCurrent;

    /* ---- Recalculer les quaternions depuis (Roll, Pitch, Yaw_forcé) ---- */
    float roll_r  = _imu.euler[0] * M_PI / 360.0f;   /* half-angle */
    float pitch_r = _imu.euler[1] * M_PI / 360.0f;
    float yaw_r   = _headingCurrent   * M_PI / 360.0f;

    float cr = cosf(roll_r),  sr = sinf(roll_r);
    float cp = cosf(pitch_r), sp = sinf(pitch_r);
    float cy = cosf(yaw_r),   sy = sinf(yaw_r);

    /* Quaternion ZYX (Yaw-Pitch-Roll) → W, X, Y, Z */
    _imu.quat[0] = (int16_t)((cr*cp*cy + sr*sp*sy) * 10000.0f);
    _imu.quat[1] = (int16_t)((sr*cp*cy - cr*sp*sy) * 10000.0f);
    _imu.quat[2] = (int16_t)((cr*sp*cy + sr*cp*sy) * 10000.0f);
    _imu.quat[3] = (int16_t)((cr*cp*sy - sr*sp*cy) * 10000.0f);

    /* Gyro Z (yaw rate) : estimer depuis le pas effectué */
    _imu.gyro[2] = (int16_t)((step / dt) * 100.0f);
}

/* =========================================================================
 * ACCÈS THREAD-SAFE AUX DONNÉES
 * ========================================================================= */

void SensorDriver::getIMUData(IMUData& out) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&out, &_imu, sizeof(IMUData));
        xSemaphoreGive(_mutex);
    }
}

void SensorDriver::getPowerData(PowerData& out) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&out, &_power, sizeof(PowerData));
        xSemaphoreGive(_mutex);
    }
}

void SensorDriver::getPressureData(PressureData& out) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&out, &_pressure, sizeof(PressureData));
        xSemaphoreGive(_mutex);
    }
}

void SensorDriver::getSensorStatus(SensorBankStatus& out) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&out, &_status, sizeof(SensorBankStatus));
        xSemaphoreGive(_mutex);
    }
}

/* =========================================================================
 * I2C BAS NIVEAU
 * ========================================================================= */

void SensorDriver::_i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t SensorDriver::_i2cRead8(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom((uint8_t)addr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

uint16_t SensorDriver::_i2cRead16(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom((uint8_t)addr, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint16_t val = (uint16_t)Wire.read() << 8;
    val |= Wire.read();
    return val;
}

bool SensorDriver::_i2cDevicePresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

/* =========================================================================
 * INITIALISATION DES CAPTEURS INDIVIDUELS
 * ========================================================================= */

bool SensorDriver::_initBNO085() {
    uint8_t whoami = _i2cRead8(BNO085_I2C_ADDR, 0x00);
    if (whoami == 0xA0) {
        _status.bno085 = SENSOR_CONNECTED;
        Serial.println("[SENSORS] BNO085 détecté (0x4A)");
        return true;
    }
    _status.bno085 = SENSOR_DISCONNECTED;
    Serial.println("[SENSORS] BNO085 NON détecté (whoami=0x" + String(whoami, HEX) + ")");
    return false;
}

/**
 * @brief Initialise le MPU9250/GY-91 avec fallback 6-DOF robuste.
 *
 * Séquence :
 * 1. Vérifie WHO_AM_I (0x75) → 0x71 ou 0x73
 * 2. Reset périphérique + auto-select horloge PLL
 * 3. Désactive le maître I2C interne (USER_CTRL bit 5 = 0)
 * 4. Active le Bypass I2C (INT_PIN_CFG bit 1 = 1) → accès AK8963
 * 5. Tente init AK8963 (WIA=0x48, mode continu 16-bit 8Hz)
 * 6. Configure gyro ±500°/s, accéléro ±4g
 * 7. AK8963 OK → 9-DOF (_magReady=true), sinon → 6-DOF (_magReady=false)
 *
 * L'IMU est toujours CONNECTÉE si WHO_AM_I correct (même sans magnéto).
 *
 * @return true si le MPU9250 est présent et initialisé.
 */
bool SensorDriver::_initMPU9250() {
    _magReady = false;
    _mpu9250Yaw = 0.0f;

    /* Étape 1 : Vérification WHO_AM_I */
    uint8_t whoami = _i2cRead8(MPU9250_I2C_ADDR, MPU9250_WHOAMI_REG);
    if (whoami != 0x71 && whoami != 0x73 && whoami != 0x75 && whoami != 0x68) {
        _status.mpu9250 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MPU9250 NON détecté (whoami=0x" + String(whoami, HEX) + ")");
        return false;
    }

    /* Étape 2 : Reset périphérique puis auto-select horloge PLL */
    _i2cWrite(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT1, 0x80); /* Reset */
    delay(100);
    _i2cWrite(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT1, 0x01); /* Auto clock */
    delay(10);

    /* Étape 3 : Désactiver maître I2C interne (bit 5 I2C_MST_EN) */
    _i2cWrite(MPU9250_I2C_ADDR, MPU9250_USER_CTRL, 0x00);
    delay(10);

    /* Étape 4 : Activer Bypass I2C (bit 1 BYPASS_EN) */
    _i2cWrite(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, 0x02);
    delay(10);

    /* Étape 5 : Tenter init magnétomètre AK8963 */
    Wire.beginTransmission(AK8963_I2C_ADDR);
    uint8_t mag_err = Wire.endTransmission();
    if (mag_err == 0) {
        uint8_t wia = _i2cRead8(AK8963_I2C_ADDR, AK8963_WIA_REG);
        if (wia == 0x48) {
            /* Reset magnéto */
            _i2cWrite(AK8963_I2C_ADDR, AK8963_CNTL2, 0x01);
            delay(100);
            /* Mode continu 16-bit 8Hz : 0x12 */
            _i2cWrite(AK8963_I2C_ADDR, AK8963_CNTL1, 0x12);
            delay(10);
            _magReady = true;
            Serial.println("[SENSORS] MPU9250 : AK8963 magnétomètre OK (9-DOF)");
        } else {
            Serial.println("[SENSORS] MPU9250 : AK8963 WIA=0x" + String(wia, HEX)
                           + " (attendu 0x48) → mode 6-DOF");
        }
    } else {
        Serial.println("[SENSORS] MPU9250 : AK8963 non accessible (err=" + String(mag_err)
                       + ") → mode 6-DOF (Gyro+Accel)");
    }

    /* Étape 6 : Configurer gyro et accéléro */
    _i2cWrite(MPU9250_I2C_ADDR, 0x1A, 0x03);               /* DLPF_CFG=3 → BW 42Hz (anti-jitter) */
    _i2cWrite(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG, 0x08);    /* FS_SEL=1 → ±500°/s, 65.5 LSB/°/s */
    _i2cWrite(MPU9250_I2C_ADDR, MPU9250_ACCEL_CONFIG, 0x08);   /* AFS_SEL=1 → ±4g, 8192 LSB/g */

    /* Étape 7 : Toujours CONNECTÉE si WHO_AM_I OK */
    _status.mpu9250 = SENSOR_CONNECTED;
    Serial.println("[SENSORS] MPU9250/GY-91 initialisé en mode "
                   + String(_magReady ? "9-DOF" : "6-DOF") + " (0x68)");
    return true;
}

bool SensorDriver::_initMS5803() {
    /* Vérifier que ce n'est PAS un BME280 à la même adresse (0x76) */
    if (_i2cDevicePresent(MS5803_I2C_ADDR)) {
        uint8_t maybe_bme = _i2cRead8(MS5803_I2C_ADDR, BME280_ID_REG);
        if (maybe_bme == BME280_ID_VALUE) {
            Serial.println("[SENSORS] MS5803 : BME280 détecté à 0x76 (ID=0x60) → skip MS5803");
            _status.ms5803 = SENSOR_DISCONNECTED;
            return false;
        }
    } else {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 NON détecté (0x76 absent)");
        return false;
    }

    Wire.beginTransmission(MS5803_I2C_ADDR);
    Wire.write(0x1E);  /* RESET command */
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 NON détecté (RESET err=" + String(err) + ")");
        return false;
    }
    delay(10);

    /* Lecture des 6 coefficients de calibration (C1 à C6) */
    bool valid_cal = false;
    for (uint8_t i = 0; i < 6; i++) {
        _ms5803_cal[i] = _i2cRead16(MS5803_I2C_ADDR, 0xA2 + (i * 2));
        if (_ms5803_cal[i] != 0 && _ms5803_cal[i] != 0xFFFF) valid_cal = true;
    }

    if (!valid_cal) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 : calibration PROM invalide → rejeté");
        return false;
    }

    _status.ms5803 = SENSOR_CONNECTED;
    Serial.println("[SENSORS] MS5803-30BA initialisé (0x76)");
    return true;
}

/**
 * @brief Initialise le BME280.
 *
 * Vérifie le registre ID (0xD0) → 0x60.
 * Configure : oversampling ×1 T, ×1 P, mode normal, standby 1000ms.
 * Lit les coefficients de calibration pression et température.
 */
bool SensorDriver::_initBME280() {
    /* Essayer les trois adresses possibles (0x76, 0x77, 0x4C) */
    if (_i2cDevicePresent(BME280_I2C_ADDR_PRI)) {
        _bme280Addr = BME280_I2C_ADDR_PRI;
    } else if (_i2cDevicePresent(BME280_I2C_ADDR_SEC)) {
        _bme280Addr = BME280_I2C_ADDR_SEC;
    } else if (_i2cDevicePresent(BMP280_I2C_ADDR_ALT)) {
        _bme280Addr = BMP280_I2C_ADDR_ALT;
    } else {
        _status.bme280 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] BME280/BMP280 NON détecté (0x76/0x77/0x4C)");
        return false;
    }

    uint8_t id = _i2cRead8(_bme280Addr, BME280_ID_REG);
    if (id != BME280_ID_VALUE && id != BMP280_ID_VALUE) {
        _status.bme280 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] BME280/BMP280 ID invalide : 0x" + String(id, HEX));
        return false;
    }

    bool isBMP = (id == BMP280_ID_VALUE);
    Serial.println("[SENSORS] " + String(isBMP ? "BMP280" : "BME280")
                   + " détecté à 0x" + String(_bme280Addr, HEX)
                   + " (ID=0x" + String(id, HEX) + ")");

    /* Configuration : humidity oversampling ×1 */
    _i2cWrite(_bme280Addr, BME280_CTRL_HUM, 0x01);
    /* Temp ×1, Pression ×1, mode normal */
    _i2cWrite(_bme280Addr, BME280_CTRL_MEAS, 0x25);
    /* Standby 1000ms, filter off */
    _i2cWrite(_bme280Addr, BME280_CONFIG_REG, 0xA0);

    /* Lecture des coefficients de calibration température */
    _bme280_dig_T1 = (uint16_t)(_i2cRead8(_bme280Addr, 0x89) << 8 | _i2cRead8(_bme280Addr, 0x88));
    _bme280_dig_T2 = (int16_t)(_i2cRead8(_bme280Addr, 0x8B) << 8 | _i2cRead8(_bme280Addr, 0x8A));
    _bme280_dig_T3 = (int16_t)(_i2cRead8(_bme280Addr, 0x8D) << 8 | _i2cRead8(_bme280Addr, 0x8C));

    /* Calibration pression */
    _bme280_dig_P1 = (uint16_t)(_i2cRead8(_bme280Addr, 0x8F) << 8 | _i2cRead8(_bme280Addr, 0x8E));
    _bme280_dig_P2 = (int16_t)(_i2cRead8(_bme280Addr, 0x91) << 8 | _i2cRead8(_bme280Addr, 0x90));
    _bme280_dig_P3 = (int16_t)(_i2cRead8(_bme280Addr, 0x93) << 8 | _i2cRead8(_bme280Addr, 0x92));
    _bme280_dig_P4 = (int16_t)(_i2cRead8(_bme280Addr, 0x95) << 8 | _i2cRead8(_bme280Addr, 0x94));
    _bme280_dig_P5 = (int16_t)(_i2cRead8(_bme280Addr, 0x97) << 8 | _i2cRead8(_bme280Addr, 0x96));
    _bme280_dig_P6 = (int16_t)(_i2cRead8(_bme280Addr, 0x99) << 8 | _i2cRead8(_bme280Addr, 0x98));
    _bme280_dig_P7 = (int16_t)(_i2cRead8(_bme280Addr, 0x9B) << 8 | _i2cRead8(_bme280Addr, 0x9A));
    _bme280_dig_P8 = (int16_t)(_i2cRead8(_bme280Addr, 0x9D) << 8 | _i2cRead8(_bme280Addr, 0x9C));
    _bme280_dig_P9 = (int16_t)(_i2cRead8(_bme280Addr, 0x9F) << 8 | _i2cRead8(_bme280Addr, 0x9E));

    _bme280Initialized = true;
    _status.bme280 = SENSOR_CONNECTED;
    Serial.println("[SENSORS] " + String(isBMP ? "BMP280" : "BME280")
                   + " OK @ 0x" + String(_bme280Addr, HEX)
                   + " dig_T1=" + String(_bme280_dig_T1)
                   + " dig_T2=" + String(_bme280_dig_T2)
                   + " dig_T3=" + String(_bme280_dig_T3)
                   + " dig_P1=" + String(_bme280_dig_P1)
                   + " dig_P2=" + String(_bme280_dig_P2)
                   + " dig_P3=" + String(_bme280_dig_P3));
    return true;
}

bool SensorDriver::_initINA226(uint8_t addr, uint8_t index) {
    uint16_t mfr_id = _i2cRead16(addr, 0xFE);
    if (mfr_id != 0x5449) {
        _status.ina226[index] = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] INA226[" + String(index) + "] @ 0x" + String(addr, HEX) + " NON détecté");
        return false;
    }

    /* Registre calibration (0x05) : shunt 0.01Ω, max 8A */
    _i2cWrite(addr, 0x05, (uint8_t)(2098 >> 8));
    _i2cWrite(addr, 0x05, (uint8_t)(2098 & 0xFF));

    _status.ina226[index] = SENSOR_CONNECTED;
    Serial.println("[SENSORS] INA226[" + String(index) + "] @ 0x" + String(addr, HEX) + " initialisé");
    return true;
}

/* =========================================================================
 * LECTURE CAPTEURS — HAL ROUTAGE
 * ========================================================================= */

void SensorDriver::_readIMU() {
    if (_activeIMU == IMU_TYPE_BNO085 && _status.bno085 == SENSOR_CONNECTED) {
        _readBNO085();
    } else if (_activeIMU == IMU_TYPE_MPU9250 && _status.mpu9250 == SENSOR_CONNECTED) {
        _readMPU9250();
    } else {
        /* IMU absente en mode réel → forcer à zéro */
        memset(&_imu.quat, 0, sizeof(_imu.quat));
        _imu.quat[0] = 10000; /* Identité */
        memset(&_imu.gyro, 0, sizeof(_imu.gyro));
        memset(&_imu.euler, 0, sizeof(_imu.euler));
    }
}

void SensorDriver::_readPressure() {
    if (_activeBaro == BARO_TYPE_MS5803 && _status.ms5803 == SENSOR_CONNECTED) {
        _readMS5803();
    } else if (_activeBaro == BARO_TYPE_BME280 && _status.bme280 == SENSOR_CONNECTED) {
        _readBME280();
    } else {
        /* Baromètre absent en mode réel → forcer à zéro */
        _pressure.pressure_mbar = 0;
        _pressure.temperature   = 0;
        _pressure.depth_m       = 0.0f;
    }
}

void SensorDriver::_readPower() {
    const uint8_t addrs[3] = {INA226_1_I2C_ADDR, INA226_2_I2C_ADDR, INA226_3_I2C_ADDR};
    for (uint8_t i = 0; i < 3; i++) {
        if (_status.ina226[i] == SENSOR_CONNECTED) {
            /* Lecture tension bus (registre 0x04) : LSB = 1.25 mV */
            uint16_t bus_raw = _i2cRead16(addrs[i], 0x04);
            _power.voltage[i] = (uint16_t)((float)bus_raw * 1.25f);
            /* Lecture courant via shunt (registre 0x01) */
            uint16_t shunt_raw = _i2cRead16(addrs[i], 0x01);
            int16_t shunt_signed = (int16_t)shunt_raw;
            float current_a = ((float)shunt_signed * 2.5e-6f) / 0.01f;
            _power.current[i] = (uint16_t)(fabsf(current_a) * 1000.0f);
        } else {
            _power.voltage[i] = 0;
            _power.current[i] = 0;
        }
    }
}

/* =========================================================================
 * LECTURE BNO085 (quaternions + gyroscope)
 * ========================================================================= */

void SensorDriver::_readBNO085() {
    /* Quaternions : registres 0x20-0x27 */
    Wire.beginTransmission(BNO085_I2C_ADDR);
    Wire.write(0x20);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)BNO085_I2C_ADDR, (uint8_t)8);
        if (Wire.available() >= 8) {
            int16_t raw[4];
            for (uint8_t i = 0; i < 4; i++) {
                raw[i] = (int16_t)(Wire.read() | (Wire.read() << 8));
            }
            _imu.quat[0] = (int16_t)((float)raw[3] / 32768.0f * 10000.0f);
            _imu.quat[1] = (int16_t)((float)raw[0] / 32768.0f * 10000.0f);
            _imu.quat[2] = (int16_t)((float)raw[1] / 32768.0f * 10000.0f);
            _imu.quat[3] = (int16_t)((float)raw[2] / 32768.0f * 10000.0f);
        }
    }

    /* Gyroscope : registres 0x18-0x1D */
    Wire.beginTransmission(BNO085_I2C_ADDR);
    Wire.write(0x18);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)BNO085_I2C_ADDR, (uint8_t)6);
        if (Wire.available() >= 6) {
            for (uint8_t i = 0; i < 3; i++) {
                int16_t raw = (int16_t)(Wire.read() | (Wire.read() << 8));
                _imu.gyro[i] = (int16_t)((float)raw / 256.0f * 100.0f);
            }
        }
    }

    /* Conversion quaternion → Euler */
    float w = (float)_imu.quat[0] / 10000.0f;
    float x = (float)_imu.quat[1] / 10000.0f;
    float y = (float)_imu.quat[2] / 10000.0f;
    float z = (float)_imu.quat[3] / 10000.0f;
    _imu.euler[0] = atan2f(2.0f*(w*x+y*z), 1.0f-2.0f*(x*x+y*y)) * 180.0f/M_PI;
    float sinp = 2.0f*(w*y-z*x);
    if (fabsf(sinp) >= 1.0f) sinp = copysignf(1.0f, sinp);
    _imu.euler[1] = asinf(sinp) * 180.0f / M_PI;
    _imu.euler[2] = atan2f(2.0f*(w*z+x*y), 1.0f-2.0f*(y*y+z*z)) * 180.0f/M_PI;
}

/* =========================================================================
 * LECTURE MPU9250 (accéléro + gyroscope → Euler + quaternion)
 * ========================================================================= */

/**
 * @brief Lit les données brutes du MPU9250 avec fusion 6-DOF / 9-DOF.
 *
 * Accéléro ±4g : sensibilité 8192 LSB/g → registres 0x3B-0x40
 * Gyroscope ±500°/s : sensibilité 65.5 LSB/°/s → registres 0x43-0x48
 *
 * Mode 6-DOF (sans magnéto) :
 *   - Roll/Pitch : filtre complémentaire (accéléro 98% + gyro 2%)
 *   - Yaw : intégration du gyroscope Z (dérive acceptable court terme)
 *
 * Mode 9-DOF (AK8963 disponible) : non implémenté ici (lecture mag future).
 */
void SensorDriver::_readMPU9250() {
    static unsigned long _lastMpuMicros = 0;
    unsigned long now = micros();
    float dt = (float)(now - _lastMpuMicros) / 1000000.0f;
    if (dt < 0.0001f || dt > 0.1f) dt = (float)TASK_SENSORS_PERIOD_MS / 1000.0f;
    _lastMpuMicros = now;

    /* ---- Lecture accéléro (6 octets depuis 0x3B) ---- */
    float ax_g = 0, ay_g = 0, az_g = 0;
    bool accel_ok = false;
    Wire.beginTransmission(MPU9250_I2C_ADDR);
    Wire.write(MPU9250_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)MPU9250_I2C_ADDR, (uint8_t)6);
        if (Wire.available() >= 6) {
            int16_t ax = (int16_t)(Wire.read() << 8 | Wire.read());
            int16_t ay = (int16_t)(Wire.read() << 8 | Wire.read());
            int16_t az = (int16_t)(Wire.read() << 8 | Wire.read());
            ax_g = (float)ax / 8192.0f;
            ay_g = (float)ay / 8192.0f;
            az_g = (float)az / 8192.0f;
            accel_ok = true;
        }
    }

    /* ---- Lecture gyroscope (6 octets depuis 0x43) ---- */
    float gx_dps = 0, gy_dps = 0, gz_dps = 0;
    bool gyro_ok = false;
    Wire.beginTransmission(MPU9250_I2C_ADDR);
    Wire.write(MPU9250_GYRO_XOUT_H);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)MPU9250_I2C_ADDR, (uint8_t)6);
        if (Wire.available() >= 6) {
            int16_t gx_r = (int16_t)(Wire.read() << 8 | Wire.read());
            int16_t gy_r = (int16_t)(Wire.read() << 8 | Wire.read());
            int16_t gz_r = (int16_t)(Wire.read() << 8 | Wire.read());
            /* 65.5 LSB/°/s */
            gx_dps = (float)gx_r / 65.5f;
            gy_dps = (float)gy_r / 65.5f;
            gz_dps = (float)gz_r / 65.5f;
            gyro_ok = true;

            /* Télémétrie gyro ×100 */
            _imu.gyro[0] = (int16_t)(gx_dps * 100.0f);
            _imu.gyro[1] = (int16_t)(gy_dps * 100.0f);
            _imu.gyro[2] = (int16_t)(gz_dps * 100.0f);
        }
    }

    /* ---- Filtre complémentaire + intégration Yaw ---- */
    if (accel_ok) {
        /* Roll/Pitch depuis accéléro */
        float accelRoll  = atan2f(ay_g, az_g) * 180.0f / M_PI;
        float accelPitch = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g)) * 180.0f / M_PI;

        if (gyro_ok) {
            /* Filtre complémentaire : 98% gyro intégré + 2% accéléro (anti-dérive) */
            _imu.euler[0] = 0.98f * (_imu.euler[0] + gx_dps * dt) + 0.02f * accelRoll;
            _imu.euler[1] = 0.98f * (_imu.euler[1] + gy_dps * dt) + 0.02f * accelPitch;
        } else {
            _imu.euler[0] = accelRoll;
            _imu.euler[1] = accelPitch;
        }

        /* Yaw : intégration gyro Z (6-DOF) + tare */
        if (gyro_ok) {
            _mpu9250Yaw += gz_dps * dt;
            /* Normaliser dans [0..360) */
            while (_mpu9250Yaw >= 360.0f) _mpu9250Yaw -= 360.0f;
            while (_mpu9250Yaw < 0.0f)    _mpu9250Yaw += 360.0f;
            /* Appliquer l'offset de tare */
            float yawTared = _mpu9250Yaw - _yawOffset;
            while (yawTared >= 360.0f) yawTared -= 360.0f;
            while (yawTared < 0.0f)    yawTared += 360.0f;
            _imu.euler[2] = yawTared;
        }
    }

    /* ---- Construire quaternion approximatif depuis Euler ---- */
    float roll_r = _imu.euler[0] * M_PI / 180.0f;
    float pitch_r = _imu.euler[1] * M_PI / 180.0f;
    float yaw_r = _imu.euler[2] * M_PI / 180.0f;
    float cr = cosf(roll_r), sr = sinf(roll_r);
    float cp = cosf(pitch_r), sp = sinf(pitch_r);
    float cy = cosf(yaw_r), sy = sinf(yaw_r);
    _imu.quat[0] = (int16_t)((cr*cp*cy + sr*sp*sy) * 10000.0f);
    _imu.quat[1] = (int16_t)((sr*cp*cy - cr*sp*sy) * 10000.0f);
    _imu.quat[2] = (int16_t)((cr*sp*cy + sr*cp*sy) * 10000.0f);
    _imu.quat[3] = (int16_t)((cr*cp*sy - sr*sp*cy) * 10000.0f);
}

/* =========================================================================
 * LECTURE MS5803-30BA
 * ========================================================================= */

void SensorDriver::_readMS5803() {
    _i2cWrite(MS5803_I2C_ADDR, 0x48, 0x00);
    delay(10);
    Wire.beginTransmission(MS5803_I2C_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom((uint8_t)MS5803_I2C_ADDR, (uint8_t)3);
        if (Wire.available() >= 3) {
            uint32_t D1 = ((uint32_t)Wire.read() << 16)
                        | ((uint32_t)Wire.read() << 8) | Wire.read();
            _i2cWrite(MS5803_I2C_ADDR, 0x58, 0x00);
            delay(10);
            Wire.beginTransmission(MS5803_I2C_ADDR);
            Wire.write(0x00);
            if (Wire.endTransmission(false) == 0) {
                Wire.requestFrom((uint8_t)MS5803_I2C_ADDR, (uint8_t)3);
                if (Wire.available() >= 3) {
                    uint32_t D2 = ((uint32_t)Wire.read() << 16)
                                | ((uint32_t)Wire.read() << 8) | Wire.read();
                    int32_t dT = (int32_t)D2 - ((int32_t)_ms5803_cal[4] << 8);
                    int32_t TEMP = 2000 + ((int64_t)dT * _ms5803_cal[5] >> 23);
                    int64_t OFF  = ((int64_t)_ms5803_cal[1] << 16)
                                 + (((int64_t)_ms5803_cal[3] * dT) >> 7);
                    int64_t SENS = ((int64_t)_ms5803_cal[0] << 15)
                                 + (((int64_t)_ms5803_cal[2] * dT) >> 8);
                    int32_t P = (int32_t)(((int64_t)D1 * SENS >> 21) - OFF) >> 13;
                    _pressure.pressure_mbar = P;
                    _pressure.temperature   = TEMP;
                    float press_mbar = (float)P * 0.1f;
                    float depth = (press_mbar - 1013.25f) / (1025.0f * 9.81f * 0.01f);
                    _pressure.depth_m = (depth > 0.0f) ? depth : 0.0f;
                }
            }
        }
    }
}

/* =========================================================================
 * LECTURE BME280 (pression + température)
 * ========================================================================= */

/**
 * @brief Lit pression et température du BME280 avec compensation.
 *
 * Lit 6 octets bruts depuis 0xF7 (pression) et 0xFA (température),
 * puis applique l'algorithme de compensation Bosch.
 */
void SensorDriver::_readBME280() {
    if (!_bme280Initialized) return;

    /* Lecture 6 octets : P[3] + T[3] depuis 0xF7 */
    Wire.beginTransmission(_bme280Addr);
    Wire.write(BME280_PRESS_REG);
    if (Wire.endTransmission(false) != 0) return;
    Wire.requestFrom(_bme280Addr, (uint8_t)6);
    if (Wire.available() < 6) return;

    uint8_t buf[6];
    for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();

    /* ADC 20 bits (BMP280/BME280 : MSB, LSB, XLSB) */
    int32_t adc_P = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

    /* =====================================================================
     * COMPENSATION TEMPÉRATURE — algorithme Bosch (32-bit, Adafruit-compat)
     * Retourne t_fine (interne) et température en °C (float).
     * ===================================================================== */
    float var1f = ((float)adc_T / 16384.0f  - (float)_bme280_dig_T1 / 1024.0f)
                  * (float)_bme280_dig_T2;
    float var2f = (((float)adc_T / 131072.0f - (float)_bme280_dig_T1 / 8192.0f)
                  * ((float)adc_T / 131072.0f - (float)_bme280_dig_T1 / 8192.0f))
                  * (float)_bme280_dig_T3;
    float t_fine = var1f + var2f;

    /* Température en centièmes de °C pour la structure PressureData */
    float tempC = t_fine / 5120.0f;            /* °C réel */
    _pressure.temperature = (int32_t)(tempC * 100.0f);  /* 0.01°C */

    /* =====================================================================
     * COMPENSATION PRESSION — algorithme Bosch 32-bit float (Adafruit BMP280)
     * Retourne la pression en Pascals (Pa).
     * Identique à la formule Adafruit_BMP280::readPressure().
     * ===================================================================== */
    var1f = t_fine / 2.0f - 64000.0f;
    var2f = var1f * var1f * (float)_bme280_dig_P6 / 32768.0f;
    var2f = var2f + var1f * (float)_bme280_dig_P5 * 2.0f;
    var2f = var2f / 4.0f + (float)_bme280_dig_P4 * 65536.0f;
    var1f = ((float)_bme280_dig_P3 * var1f * var1f / 524288.0f
             + (float)_bme280_dig_P2 * var1f) / 524288.0f;
    var1f = (1.0f + var1f / 32768.0f) * (float)_bme280_dig_P1;

    if (var1f == 0.0f) {
        _pressure.pressure_mbar = 0;
    } else {
        float p = 1048576.0f - (float)adc_P;
        p = ((p - var2f / 4096.0f) * 6250.0f) / var1f;
        var1f = (float)_bme280_dig_P9 * p * p / 2147483648.0f;
        var2f = p * (float)_bme280_dig_P8 / 32768.0f;
        p = p + (var1f + var2f + (float)_bme280_dig_P7) / 16.0f;
        /* p est maintenant en Pascals (Pa).
         * Conversion en 0.1 mbar : 1 Pa = 0.01 mbar = 0.1 × 0.1 mbar
         * donc pressure_mbar = Pa / 10 */
        _pressure.pressure_mbar = (int32_t)(p / 10.0f);
    }

    /* Profondeur relative : 0 en surface (air libre) */
    float press_mbar = (float)_pressure.pressure_mbar * 0.1f;
    float depth = (press_mbar - 1013.25f) / (1025.0f * 9.81f * 0.01f);
    _pressure.depth_m = (depth > 0.0f) ? depth : 0.0f;
}

/* =========================================================================
 * ZÉROS POUR CAPTEURS ABSENTS (MODE RÉEL)
 * ========================================================================= */

void SensorDriver::_zeroMissingSensors() {
    /* Cette fonction est appelée implicitement par les méthodes HAL
     * (_readIMU, _readPressure, _readPower) qui vérifient chacune
     * le statut SENSOR_CONNECTED avant de lire. Si non connecté,
     * les valeurs sont déjà forcées à 0 dans ces méthodes. */
}

/* =========================================================================
 * MOTEUR DE SIMULATION DYNAMIQUE
 * ========================================================================= */

void SensorDriver::_updateSimulation() {
    float t  = millis() / 1000.0f;
    float dt = (float)TASK_SENSORS_PERIOD_MS / 1000.0f;
    _simAttitude(t);
    _simDepth(dt);
    _simPower(t);
    _pressure.pressure_mbar = (int32_t)((1013.25f + _pressure.depth_m * 100.0f) * 10.0f);
    _pressure.temperature = 2050;
}

void SensorDriver::_simAttitude(float t) {
    float omega1 = 2.0f * M_PI / SIM_SWELL_PERIOD;
    float omega2 = omega1 * 0.7f;
    float roll  = SIM_ROLL_AMP * sinf(omega1*t) + SIM_ROLL_AMP*0.3f*sinf(omega2*t+0.5f);
    float pitch = SIM_PITCH_AMP * sinf(omega1*t*0.8f+1.0f) + SIM_PITCH_AMP*0.2f*cosf(omega2*t);
    float yaw   = 180.0f + 5.0f * sinf(t * 0.1f);
    float cr=cosf(roll*M_PI/360.0f), sr=sinf(roll*M_PI/360.0f);
    float cp=cosf(pitch*M_PI/360.0f), sp=sinf(pitch*M_PI/360.0f);
    float cy=cosf(yaw*M_PI/360.0f), sy=sinf(yaw*M_PI/360.0f);
    _imu.quat[0] = (int16_t)((cr*cp*cy+sr*sp*sy)*10000.0f);
    _imu.quat[1] = (int16_t)((sr*cp*cy-cr*sp*sy)*10000.0f);
    _imu.quat[2] = (int16_t)((cr*sp*cy+sr*cp*sy)*10000.0f);
    _imu.quat[3] = (int16_t)((cr*cp*sy-sr*sp*cy)*10000.0f);
    _imu.euler[0]=roll; _imu.euler[1]=pitch; _imu.euler[2]=yaw;
    _imu.gyro[0] = (int16_t)(SIM_ROLL_AMP*omega1*cosf(omega1*t)*100.0f);
    _imu.gyro[1] = (int16_t)(SIM_PITCH_AMP*omega1*0.8f*cosf(omega1*t*0.8f+1.0f)*100.0f);
    _imu.gyro[2] = (int16_t)(5.0f*0.1f*cosf(t*0.1f)*100.0f);
}

void SensorDriver::_simDepth(float dt) {
    static float simDepth = SIM_DEPTH_INIT;
    static float simVSpeed = 0.0f;
    simVSpeed += ((float)(random(-10,10))/1000.0f)*dt;
    simVSpeed *= 0.99f;
    simDepth += simVSpeed * dt;
    if (simDepth < 0.0f)  { simDepth=0.0f;  simVSpeed=0.0f; }
    if (simDepth > 30.0f) { simDepth=30.0f; simVSpeed=0.0f; }
    _pressure.depth_m = simDepth;
}

void SensorDriver::_simPower(float t) {
    float discharge = (float)(millis()/1000UL) / 7200.0f;
    uint16_t batt = (uint16_t)(SIM_BATT_NOMINAL_MV - discharge*(SIM_BATT_NOMINAL_MV-SIM_BATT_MIN_MV));
    if (batt < SIM_BATT_MIN_MV) batt = SIM_BATT_MIN_MV;
    _power.voltage[0] = batt;
    _power.current[0] = 2500 + (int16_t)(200.0f*sinf(t*0.5f));
    _power.voltage[1] = (uint16_t)(batt*0.98f);
    _power.current[1] = 800 + (int16_t)(100.0f*sinf(t*0.3f));
    float thrust = 0.5f + 0.5f*fabsf(sinf(t*0.2f));
    _power.voltage[2] = (uint16_t)(batt - thrust*200.0f);
    _power.current[2] = (uint16_t)(3000.0f + thrust*5000.0f);
}

/* =========================================================================
 * TÂCHE FREERTOS
 * ========================================================================= */

void SensorDriver::_taskEntry(void* param) {
    static_cast<SensorDriver*>(param)->_taskLoop();
}

void SensorDriver::_taskLoop() {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        if (!_simMode) {
            /* Mode RÉEL : lecture capteurs avec forçage zéro si absent */
            xSemaphoreTake(_mutex, portMAX_DELAY);
            _readIMU();
            _readPressure();
            _readPower();
            /* Forçage de cap manuel : override Yaw après lecture IMU */
            _applyHeadingOverride();
            xSemaphoreGive(_mutex);
        } else {
            /* Mode SIMULATEUR : données synthétiques */
            xSemaphoreTake(_mutex, portMAX_DELAY);
            _updateSimulation();
            /* Forçage de cap manuel : override Yaw après simulation */
            _applyHeadingOverride();
            xSemaphoreGive(_mutex);
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
    }
}
