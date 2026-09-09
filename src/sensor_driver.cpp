/**
 * @file sensor_driver.cpp
 * @brief Implémentation du pilote I2C multi-capteurs avec HAL.
 *
 * Capteurs supportés :
 *   - IMU : BNO085 (0x4A) via Adafruit BNO08x (protocole SH-2)
 *   - Baro : MS5803-30BA / MS5837 (0x76)
 *   - Wattmètres : 3× INA226 (0x41, 0x44, 0x45)
 *   - PWM : PCA9685 (0x40) — géré séparément
 *
 * @author Didier Dero
 * @version 2.0.0
 */

#include "sensor_driver.h"
#include "config.h"
#include <math.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_BNO08x.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */
SensorDriver g_sensors;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

SensorDriver::SensorDriver()
    : _mutex(nullptr)
    , _i2cBusy(false)
    , _bno08x(nullptr)
{
    memset(&_imu, 0, sizeof(_imu));
    memset(&_power, 0, sizeof(_power));
    memset(&_pressure, 0, sizeof(_pressure));
    memset(&_status, 0, sizeof(_status));
    memset(_ms5803_cal, 0, sizeof(_ms5803_cal));
    _imu.quat[0] = 10000; /* Quaternion identité W=10000 */
    _currentSDA = I2C_SDA_PIN;
    _currentSCL = I2C_SCL_PIN;
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

void SensorDriver::begin() {
    /* Créer le mutex */
    _mutex = xSemaphoreCreateMutex();

    /* Charger les broches I2C depuis NVS si disponibles */
    Preferences prefs;
    prefs.begin("config", true);
    _currentSDA = prefs.getUChar("i2c_sda", I2C_SDA_PIN);
    _currentSCL = prefs.getUChar("i2c_scl", I2C_SCL_PIN);
    prefs.end();

    /* Initialiser le bus I2C */
    Wire.begin(_currentSDA, _currentSCL);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    delay(100);

    Serial.println("[SENSORS] Bus I2C initialisé (SDA=" + String(_currentSDA)
                   + ", SCL=" + String(_currentSCL) + ", 400 kHz)");

    /* Scanner le bus I2C pour détecter les puces présentes */
    _scanI2CBus();

    /* Initialisation IMU : BNO085 uniquement */
    bool imu_ok = false;
    if (_i2cDevicePresent(BNO085_I2C_ADDR)) {
        if (_initBNO085()) imu_ok = true;
    } else {
        /* Tenter quand même (cas edge : délai d'alimentation) */
        if (_initBNO085()) imu_ok = true;
    }
    if (!imu_ok) {
        Serial.println("[SENSORS] Aucune IMU détectée (BNO085 attendu à 0x4A)");
    }

    /* Initialisation baromètre : MS5803 uniquement */
    bool baro_ok = false;
    if (_i2cDevicePresent(MS5803_I2C_ADDR)) {
        if (_initMS5803()) baro_ok = true;
    }
    if (!baro_ok) {
        Serial.println("[SENSORS] Aucun baromètre détecté (MS5803/MS5837 attendu à 0x76)");
    }

    /* Initialisation INA226 */
    const uint8_t ina_addrs[3] = {INA226_1_I2C_ADDR, INA226_2_I2C_ADDR, INA226_3_I2C_ADDR};
    for (uint8_t i = 0; i < 3; i++) {
        _initINA226(ina_addrs[i], i);
    }

    /* PCA9685 */
    if (_i2cDevicePresent(PCA9685_I2C_ADDR)) {
        _status.pca9685 = SENSOR_CONNECTED;
        Serial.println("[SENSORS] PCA9685 détecté (0x40)");
    } else {
        _status.pca9685 = SENSOR_DISCONNECTED;
    }

    Serial.println("[SENSORS] Résumé : BNO085=" + String(imu_ok ? "OK" : "ABSENT")
                   + ", MS5803=" + String(baro_ok ? "OK" : "ABSENT"));

    /* Lancement de la tâche d'acquisition sur Core 1 */
    xTaskCreatePinnedToCore(
        _taskEntry, "Sensors", STACK_SIZE_SENSORS,
        this, PRIORITY_SENSORS, nullptr, CORE_RT
    );
}

/* =========================================================================
 * SCANNER I2C
 * ========================================================================= */

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
 */
static const char* _identifyI2CDevice(uint8_t addr) {
    switch (addr) {
        case 0x40: return "PCA9685 (Driver PWM 16 Ch)";
        case 0x41: return "INA226 #1 (Wattmètre RPi)";
        case 0x44: return "INA226 #2 (Wattmètre Aux)";
        case 0x45: return "INA226 #3 (Wattmètre Moteurs)";
        case 0x4A: return "BNO085 (IMU 9-DOF)";
        case 0x76: return "MS5803-30BA / MS5837 (Pression)";
        default:   return "Périphérique inconnu";
    }
}

std::vector<I2CDeviceInfo> SensorDriver::scanI2CBus() {
    /* Bloquer l'accès I2C par la tâche capteurs pendant le scan */
    _i2cBusy = true;
    delay(5);

    /* Récupération de bus I2C bloqué : 9 impulsions SCL pour libérer SDA */
    pinMode(_currentSCL, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(_currentSCL, LOW);
        delayMicroseconds(50);
        digitalWrite(_currentSCL, HIGH);
        delayMicroseconds(50);
    }
    /* Condition STOP pour libérer le bus */
    pinMode(_currentSDA, OUTPUT);
    digitalWrite(_currentSDA, LOW);
    delayMicroseconds(50);
    digitalWrite(_currentSCL, HIGH);
    delayMicroseconds(50);
    digitalWrite(_currentSDA, HIGH);
    delayMicroseconds(50);

    /* Réinitialiser Wire après la récupération */
    Wire.end();
    delay(10);
    Wire.begin(_currentSDA, _currentSCL);
    Wire.setClock(I2C_FREQ_HZ);
    delay(10);

    /* Réduire le timeout I2C pour accélérer le scan */
    unsigned long prevTimeout = Wire.getTimeOut();
    Wire.setTimeOut(10);

    std::vector<I2CDeviceInfo> devices;
    for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            I2CDeviceInfo info;
            info.address = addr;
            snprintf(info.hexStr, sizeof(info.hexStr), "0x%02X", addr);
            const char* name = _identifyI2CDevice(addr);
            strncpy(info.name, name, sizeof(info.name) - 1);
            info.name[sizeof(info.name) - 1] = '\0';
            devices.push_back(info);
        }
    }

    /* Restaurer le timeout d'origine */
    Wire.setTimeOut(prevTimeout);
    _i2cBusy = false;
    return devices;
}

void SensorDriver::reinitI2C(uint8_t sda, uint8_t scl) {
    Serial.println("[SENSORS] Réinit I2C : SDA=" + String(sda) + ", SCL=" + String(scl));

    _i2cBusy = true;
    delay(10);

    Wire.end();
    delay(50);

    Preferences prefs;
    prefs.begin("config", false);
    prefs.putUChar("i2c_sda", sda);
    prefs.putUChar("i2c_scl", scl);
    prefs.end();

    _currentSDA = sda;
    _currentSCL = scl;

    Wire.begin(sda, scl);
    Wire.setClock(I2C_FREQ_HZ);
    delay(100);

    _i2cBusy = false;
    Serial.println("[SENSORS] Bus I2C réinitialisé (SDA=" + String(sda)
                   + ", SCL=" + String(scl) + ", 400 kHz)");
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
    /* Libérer l'instance précédente si re-init */
    if (_bno08x) {
        delete _bno08x;
        _bno08x = nullptr;
    }

    _bno08x = new Adafruit_BNO08x();

    /* Tenter l'initialisation I2C à l'adresse 0x4A */
    if (!_bno08x->begin_I2C(BNO085_I2C_ADDR)) {
        _status.bno085 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] BNO085 : échec begin_I2C(0x4A)");
        delete _bno08x;
        _bno08x = nullptr;
        return false;
    }

    /* Activer les rapports Game Rotation Vector (quaternions) à 50 Hz */
    if (!_bno08x->enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
        _status.bno085 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] BNO085 : échec enableReport GAME_ROTATION_VECTOR");
        delete _bno08x;
        _bno08x = nullptr;
        return false;
    }

    /* Activer le gyroscope à 50 Hz */
    if (!_bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)) {
        Serial.println("[SENSORS] BNO085 : avertissement enableReport GYROSCOPE_CALIBRATED échoué");
    }

    _status.bno085 = SENSOR_CONNECTED;
    Serial.println("[SENSORS] BNO085 détecté et initialisé (0x4A) — Game Rotation Vector + Gyroscope @ 50 Hz");
    return true;
}

bool SensorDriver::_initMS5803() {
    if (!_i2cDevicePresent(MS5803_I2C_ADDR)) {
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
    Serial.println("[SENSORS] MS5803-30BA / MS5837 initialisé (0x76)");
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
    if (_status.bno085 == SENSOR_CONNECTED) {
        _readBNO085();
    } else {
        /* IMU absente → forcer à zéro */
        memset(&_imu.quat, 0, sizeof(_imu.quat));
        _imu.quat[0] = 10000; /* Identité */
        memset(&_imu.gyro, 0, sizeof(_imu.gyro));
        memset(&_imu.euler, 0, sizeof(_imu.euler));
    }
}

void SensorDriver::_readPressure() {
    if (_status.ms5803 == SENSOR_CONNECTED) {
        _readMS5803();
    } else {
        /* Baromètre absent → forcer à zéro */
        _pressure.pressure_mbar = 0;
        _pressure.temperature   = 0;
        _pressure.altitude_m    = 0.0f;
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
 * LECTURE BNO085 (quaternions + gyroscope via protocole SH-2)
 * ========================================================================= */

void SensorDriver::_readBNO085() {
    if (!_bno08x) return;

    /* Lire les rapports disponibles (non-bloquant) */
    sh2_SensorValue_t report;
    bool hasNewQuat = false;

    while (_bno08x->getSensorEvent(&report)) {
        if (report.sensorId == SH2_GAME_ROTATION_VECTOR) {
            /* Quaternion i, j, k, real */
            _imu.quat[0] = (int16_t)(report.un.gameRotationVector.real * 10000.0f);
            _imu.quat[1] = (int16_t)(report.un.gameRotationVector.i * 10000.0f);
            _imu.quat[2] = (int16_t)(report.un.gameRotationVector.j * 10000.0f);
            _imu.quat[3] = (int16_t)(report.un.gameRotationVector.k * 10000.0f);
            hasNewQuat = true;
        } else if (report.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            /* Gyroscope en rad/s → convertir en °/s et scaler ×100 */
            const float rad_to_deg = 180.0f / M_PI;
            _imu.gyro[0] = (int16_t)(report.un.gyroscope.x * rad_to_deg * 100.0f);
            _imu.gyro[1] = (int16_t)(report.un.gyroscope.y * rad_to_deg * 100.0f);
            _imu.gyro[2] = (int16_t)(report.un.gyroscope.z * rad_to_deg * 100.0f);
        }
    }

    /* Conversion quaternion → Euler uniquement si nouveau quaternion */
    if (hasNewQuat) {
        float w = (float)_imu.quat[0] / 10000.0f;
        float x = (float)_imu.quat[1] / 10000.0f;
        float y = (float)_imu.quat[2] / 10000.0f;
        float z = (float)_imu.quat[3] / 10000.0f;

        /* Roll (X) */
        _imu.euler[0] = atan2f(2.0f*(w*x+y*z), 1.0f-2.0f*(x*x+y*y)) * 180.0f/M_PI;

        /* Pitch (Y) */
        float sinp = 2.0f*(w*y-z*x);
        if (fabsf(sinp) >= 1.0f) sinp = copysignf(1.0f, sinp);
        _imu.euler[1] = asinf(sinp) * 180.0f / M_PI;

        /* Yaw (Z) */
        _imu.euler[2] = atan2f(2.0f*(w*z+x*y), 1.0f-2.0f*(y*y+z*z)) * 180.0f/M_PI;
    }
}

/* =========================================================================
 * LECTURE MS5803 (pression + température + profondeur)
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
                    /* Capteur immergé : profondeur positive vers le bas,
                     * stockée comme altitude négative (repère vertical unifié) */
                    float depth = (press_mbar - 1013.25f) / (1025.0f * 9.81f * 0.01f);
                    _pressure.altitude_m = -(depth > 0.0f ? depth : 0.0f);
                }
            }
        }
    }
}

/* =========================================================================
 * ZÉROS POUR CAPTEURS ABSENTS
 * ========================================================================= */

void SensorDriver::_zeroMissingSensors() {
    /* Géré implicitement par _readIMU/_readPressure/_readPower
     * qui forcent à 0 si le capteur n'est pas SENSOR_CONNECTED. */
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
        /* Si le bus I2C est réservé (scan/reinit), sauter les lectures */
        if (_i2cBusy) {
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
            continue;
        }

        xSemaphoreTake(_mutex, portMAX_DELAY);

        _readIMU();
        _readPressure();
        _readPower();

        xSemaphoreGive(_mutex);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
    }
}
