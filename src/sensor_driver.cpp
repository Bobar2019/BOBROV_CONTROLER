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
 * Répartition sur les deux bus I2C matériels de l'ESP32-S3 :
 *   - Bus n°1 (Wire,  GPIO 10/11) : BNO085 + 3× INA226 — propriété
 *     exclusive de la tâche capteurs.
 *   - Bus n°2 (Wire1, GPIO 6/7)   : MS5837 + PCA9685 — partagé entre la
 *     tâche capteurs (pression 2 Hz) et la tâche de contrôle (PWM 100 Hz) ;
 *     transactions atomiques sérialisées par le verrou interne de Wire1.
 *     (Le MS5837 branché sur le bus n°1 perturbait les lectures SHTP du BNO085.)
 *
 * @author Didier Dero
 * @version 2.1.0
 */

#include "sensor_driver.h"
#include "config.h"
#include "pwm_controller.h"   /* g_pwm : neutralisation de sécurité (niveau 3) */
#include <math.h>
#include <Wire.h>
#include <esp_system.h>       /* esp_restart() : redémarrage contrôlé (niveau 3) */
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
    , _bno08x(nullptr)
{
    memset(&_imu, 0, sizeof(_imu));
    memset(&_power, 0, sizeof(_power));
    memset(&_pressure, 0, sizeof(_pressure));
    memset(&_status, 0, sizeof(_status));
    memset(_ms5803_cal, 0, sizeof(_ms5803_cal));
    _imu.quat[0] = 10000; /* Quaternion identité W=10000 */
    _currentSDA  = I2C_SDA_PIN;
    _currentSCL  = I2C_SCL_PIN;
    _current2SDA = I2C_BUS2_SDA_PIN;
    _current2SCL = I2C_BUS2_SCL_PIN;
    _lastMS5803Read = 0;
    _ms5803ConsecFails = 0;    /* détection de perte MS5803 (re-sonde si débranché) */
    /* Tare surface : P0 par défaut, (re)capturée à la 1re lecture MS5803 valide */
    _surfaceMbar   = SEA_LEVEL_PRESSURE_MBAR;
    _tareRequested = true;
    _lastBaroAlt   = 0.0f;
    /* Tare Nord : offset nul au départ — capturé au 1er « Régler Nord » (ajustage
     * fin en cap magnétique, référence obligatoire en secours relatif) */
    _headingOffset      = 0.0f;
    _northTareRequested = false;
    /* Sources de quaternions BNO085 : Rotation Vector magnétique prioritaire,
     * Game Rotation Vector relatif en secours (choix dans _readBNO085) */
    memset(_rvQuat,  0, sizeof(_rvQuat));
    memset(_grvQuat, 0, sizeof(_grvQuat));
    _rvTime   = 0;   /* jamais reçu → secours relatif jusqu'au 1er rapport magnétique */
    _grvTime  = 0;
    _rvStatus = 0;
    _rvCfgOk       = false;
    _rvReports     = 0;
    _grvReports    = 0;
    _rvLastRetryMs = 0;
    /* Buffers de travail (tâche capteurs) + supervision BNO085 sans INT */
    memset(&_imuWork, 0, sizeof(_imuWork));
    memset(&_powerWork, 0, sizeof(_powerWork));
    _imuWork.quat[0]  = 10000; /* Identité */
    _bno085Present    = false;
    _lastBNO085Event  = 0;
    _lastBNO085QuatTime = 0;    /* 0 = aucun quaternion reçu depuis le boot (garde-fou anti-boucle du niveau 3) */
    _lastBNO085Attempt = 0;     /* fin de la dernière tentative de récupération */
    _bno085FailStreak = 0;      /* tentatives sans retour du flux de quaternions */
    _bno085Recoveries = 0;
    _bno085Err[0]     = '\0';   /* dernière erreur d'init BNO085 (diagnostic web) */
    _i2cConsecFails   = 0;      /* watchdog I2C global */
    _lastBusRecovery  = 0;
    _busHadSensors    = false;  /* armé dans begin() si le bus voit des capteurs */

    /* File de requêtes bus (tâche Web → tâche capteurs) — scans uniquement */
    _scanReq = false;   _scanDone = true;   /* aucune requête en attente */
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

void SensorDriver::begin() {
    /* Créer le mutex */
    _mutex = xSemaphoreCreateMutex();

    /* Charger les broches I2C des deux bus depuis NVS si disponibles
     * (bus n°1 : clés i2c_sda / i2c_scl — bus n°2 : clés i2c2_sda / i2c2_scl,
     * configurables depuis la page Paramètres de l'interface Web) */
    Preferences prefs;
    prefs.begin("config", true);
    _currentSDA  = prefs.getUChar("i2c_sda", I2C_SDA_PIN);
    _currentSCL  = prefs.getUChar("i2c_scl", I2C_SCL_PIN);
    _current2SDA = prefs.getUChar("i2c2_sda", I2C_BUS2_SDA_PIN);
    _current2SCL = prefs.getUChar("i2c2_scl", I2C_BUS2_SCL_PIN);
    prefs.end();

    /* Initialiser le bus I2C n°1 — 400 kHz Fast Mode + timeout 50 ms
     * (configuration de la version de référence fonctionnelle). */
    Wire.begin(_currentSDA, _currentSCL);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);

    /* Bus I2C n°2 (Wire1) : MS5837 + PCA9685, broches chargées depuis le NVS
     * ci-dessus (défauts GPIO 6/7). Configuré ICI, avant la création de la
     * tâche capteurs (fin de begin()) et celle de la tâche de contrôle
     * (main.cpp) : fréquence/timeout ne sont ensuite plus jamais modifiés
     * pendant une transaction. Les transactions concurrentes des deux tâches
     * sont sérialisées par le verrou interne de Wire1 (une par instance). */
    Wire1.begin(_current2SDA, _current2SCL);
    Wire1.setClock(I2C_FREQ_HZ);
    Wire1.setTimeOut(I2C_TIMEOUT_MS);

    /* Délai de mise sous tension : le coprocesseur ARM du BNO085 doit démarrer
     * avant la toute première commande I2C (y compris le scan ci-dessous). */
    delay(BNO085_BOOT_DELAY_MS);

    Serial.println("[SENSORS] Bus I2C n°1 initialisé (SDA=" + String(_currentSDA)
                   + ", SCL=" + String(_currentSCL) + ", "
                   + String(I2C_FREQ_HZ / 1000) + " kHz, timeout "
                   + String(I2C_TIMEOUT_MS) + " ms)");
    Serial.println("[SENSORS] Bus I2C n°2 initialisé (SDA=" + String(_current2SDA)
                   + ", SCL=" + String(_current2SCL) + ", "
                   + String(I2C_FREQ_HZ / 1000) + " kHz) — MS5837 + PCA9685");

    /* Scanner le bus I2C pour détecter les puces présentes */
    _scanI2CBus();

    /* Mémoriser la présence du BNO085 : la tâche capteurs le supervisera en
     * continu (auto-récupération) même après un effondrement du bus. */
    _bno085Present = _i2cDevicePresent(BNO085_I2C_ADDR);

    /* Initialisation IMU : BNO085 avec boucle de réessais. La 1re tentative est
     * DIRECTE (comme la version de référence qui fonctionnait : le scan I2C ne
     * perturbe pas le handshake SHTP de begin_I2C). Le reset matériel du bus
     * n'intervient qu'aux tentatives suivantes, si un essai a laissé le bus
     * en erreur (NACK → ESP_ERR_INVALID_STATE). */
    bool imu_ok = false;
    for (uint8_t attempt = 1; attempt <= BNO085_INIT_MAX_ATTEMPTS && !imu_ok; attempt++) {
        if (attempt > 1) {
            _resetI2CBus();
            delay(BNO085_INIT_RETRY_DELAY_MS);
        }

        if (_initBNO085()) {
            imu_ok = true;
            if (attempt > 1) {
                Serial.println("[SENSORS] BNO085 initialisé à la tentative "
                               + String(attempt) + "/" + String(BNO085_INIT_MAX_ATTEMPTS));
            }
        } else {
            Serial.println("[SENSORS] BNO085 : échec tentative "
                           + String(attempt) + "/" + String(BNO085_INIT_MAX_ATTEMPTS)
                           + (attempt < BNO085_INIT_MAX_ATTEMPTS
                              ? ", nouvel essai dans " + String(BNO085_INIT_RETRY_DELAY_MS) + " ms"
                              : ""));
            if (attempt < BNO085_INIT_MAX_ATTEMPTS) delay(BNO085_INIT_RETRY_DELAY_MS);
        }
    }
    if (!imu_ok) {
        Serial.println("[SENSORS] Aucune IMU détectée (BNO085 attendu à 0x4A) après "
                       + String(BNO085_INIT_MAX_ATTEMPTS) + " tentatives");
    }

    /* Initialisation baromètre : MS5837 sur le bus n°2 */
    bool baro_ok = false;
    if (_i2c2DevicePresent(MS5803_I2C_ADDR)) {
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

    /* PCA9685 (bus n°2, partagé avec la tâche de contrôle) */
    if (_i2c2DevicePresent(PCA9685_I2C_ADDR)) {
        _status.pca9685 = SENSOR_CONNECTED;
        Serial.println("[SENSORS] PCA9685 détecté (0x40, bus n°2)");
    } else {
        _status.pca9685 = SENSOR_DISCONNECTED;
    }

    /* Mémoriser si le bus avait au moins un capteur : arme le watchdog
     * « bus mort » (tentative de réinit complète périodique si TOUT est perdu
     * ultérieurement — évite l'état silencieusement mort sans issue). */
    _busHadSensors = (imu_ok || baro_ok ||
                      _status.ina226[0] == SENSOR_CONNECTED ||
                      _status.ina226[1] == SENSOR_CONNECTED ||
                      _status.ina226[2] == SENSOR_CONNECTED);

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
    Serial.println("[SENSORS] === Scan I2C (bus n°1 + bus n°2) ===");
    uint8_t count = 0;
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[SENSORS]   0x");
            Serial.print(addr, HEX);
            Serial.println(" trouvé (bus n°1)");
            count++;
        }
    }
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) {
            Serial.print("[SENSORS]   0x");
            Serial.print(addr, HEX);
            Serial.println(" trouvé (bus n°2)");
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

/**
 * @brief Sonde toutes les adesses d'un bus I2C et accumule les périphériques
 *        qui ACK, étiquetés avec leur numéro de bus (1 = Wire, 2 = Wire1).
 */
static void _scanOneBus(TwoWire& bus, uint8_t busNo,
                        std::vector<I2CDeviceInfo>& devices) {
    for (uint8_t addr = 0x01; addr <= 0x7F; addr++) {
        bus.beginTransmission(addr);
        if (bus.endTransmission() == 0) {
            I2CDeviceInfo info;
            info.address = addr;
            info.bus     = busNo;
            snprintf(info.hexStr, sizeof(info.hexStr), "0x%02X", addr);
            const char* name = _identifyI2CDevice(addr);
            strncpy(info.name, name, sizeof(info.name) - 1);
            info.name[sizeof(info.name) - 1] = '\0';
            devices.push_back(info);
        }
    }
}

std::vector<I2CDeviceInfo> SensorDriver::scanI2CBus() {
    /* La tâche Web ne touche pas le bus : la requête est exécutée par la tâche
     * capteurs (unique propriétaire du bus) au prochain passage de boucle. */
    _scanDone = false;
    _scanReq  = true;

    /* Attendre l'exécution — un cycle capteurs dure ~10-30 ms ; en conditions
     * dégradées (récupérations watchdog en cours), laisser jusqu'à 3 s. */
    uint32_t t0 = millis();
    while (!_scanDone && (millis() - t0) < 3000UL) {
        delay(5);
    }

    if (!_scanDone) {
        Serial.println("[SENSORS] Scan I2C : tâche capteurs injoignable (timeout 3 s)");
        std::vector<I2CDeviceInfo> vide;
        return vide;
    }
    return _scanResult;
}

void SensorDriver::_doScan() {
    /* Récupération de bus I2C n°1 bloqué : 9 impulsions SCL pour libérer SDA.
     * Possible uniquement parce que ce bus appartient en exclusivité à cette
     * tâche — JAMAIS sur le bus n°2, partagé avec les écritures PCA9685 de la
     * tâche de contrôle. */
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

    /* Réinitialiser Wire (bus n°1) après la récupération */
    Wire.end();
    delay(10);
    Wire.begin(_currentSDA, _currentSCL);
    Wire.setClock(I2C_FREQ_HZ);
    delay(10);

    /* Réduire le timeout I2C des deux bus pour accélérer le scan */
    unsigned long prevTimeout  = Wire.getTimeOut();
    unsigned long prevTimeout1 = Wire1.getTimeOut();
    Wire.setTimeOut(10);
    Wire1.setTimeOut(10);

    std::vector<I2CDeviceInfo> devices;
    _scanOneBus(Wire,  1, devices);   /* bus n°1 : BNO085 + INA226 */
    _scanOneBus(Wire1, 2, devices);   /* bus n°2 : MS5837 + PCA9685 */

    /* Restaurer les timeouts d'origine ; le bit-bang ci-dessus étant déjà une
     * récupération de bus, réarmer aussi le compteur du watchdog I2C global. */
    Wire.setTimeOut(prevTimeout);
    Wire1.setTimeOut(prevTimeout1);
    _i2cConsecFails = 0;
    _scanResult = devices;
}

/* -------------------------------------------------------------------------
 * BROCHES I2C (bus n°1 ou n°2) — persistance NVS
 *
 * Les nouvelles broches sont enregistrées en NVS puis appliquées au prochain
 * démarrage : le bus n°2 est partagé avec la tâche de contrôle (PCA9685 à
 * 100 Hz — un Wire1.end()/begin() en vol couperait les sorties PWM), et le
 * bus n°1 suit la même procédure pour une interface uniforme. L'écriture est
 * vérifiée par RELECTURE : si le NVS refuse, la valeur n'est pas confirmée et
 * l'appelant ne programme pas le redémarrage.
 * ------------------------------------------------------------------------- */
bool SensorDriver::saveBusPins(uint8_t bus, uint8_t sda, uint8_t scl) {
    const char* keySda = (bus == 2) ? "i2c2_sda" : "i2c_sda";
    const char* keyScl = (bus == 2) ? "i2c2_scl" : "i2c_scl";

    Preferences prefs;
    if (!prefs.begin("config", false)) {
        Serial.println("[SENSORS] Broches bus n°" + String(bus)
                       + " : échec d'ouverture NVS (écriture)");
        return false;
    }
    /* 0xFF = sentinelle impossible (GPIO limités à 0-48) pour la relecture */
    const bool ok = prefs.putUChar(keySda, sda) == 1
                 && prefs.putUChar(keyScl, scl) == 1
                 && prefs.getUChar(keySda, 0xFF) == sda
                 && prefs.getUChar(keyScl, 0xFF) == scl;
    prefs.end();

    if (!ok) {
        Serial.println("[SENSORS] Broches bus n°" + String(bus)
                       + " : échec d'écriture NVS — non enregistrées");
        return false;
    }
    Serial.println("[SENSORS] Broches bus n°" + String(bus) + " enregistrées (NVS) : SDA="
                   + String(sda) + ", SCL=" + String(scl)
                   + " — appliquées au prochain démarrage");
    return true;
}

/* -------------------------------------------------------------------------
 * REQUÊTES BUS DE LA TÂCHE WEB — exécution différée par la tâche capteurs
 *
 * Le bus I2C appartient en exclusivité à la tâche capteurs. Les demandes de
 * scan issues du serveur Web (core 0) sont mises en file et exécutées au
 * début du prochain cycle — plus aucune collision possible entre le bit-bang
 * SCL du scan et une transaction en cours. (Les changements de broches ne
 * passent plus par ici : sauvegarde NVS + redémarrage, voir saveBusPins.)
 * ------------------------------------------------------------------------- */
void SensorDriver::_processBusRequests() {
    if (_scanReq) {
        _scanReq = false;
        _doScan();
        _scanDone = true;
    }
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

void SensorDriver::requestSurfaceTare() {
    /* Simple signal volatile : la tâche capteurs recapture P_surface à la
     * prochaine lecture MS5803 (voir _readMS5803). Thread-safe par atomicité
     * d'un booléen. */
    _tareRequested = true;
}

void SensorDriver::requestNorthTare() {
    /* Signal volatile : la tâche capteurs capture le cap brut courant comme
     * nouveau zéro Nord à la prochaine lecture BNO085 (voir _readBNO085).
     * Thread-safe par atomicité d'un booléen, comme la tare surface. */
    _northTareRequested = true;
}

/* -------------------------------------------------------------------------
 * Diagnostic BNO085 : dernière erreur d'initialisation, exposée à l'interface
 * Web (permet de savoir POURQUOI le badge est rouge sans câble USB).
 * ------------------------------------------------------------------------- */
void SensorDriver::_setBNO085Err(const char* msg) {
    if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        strncpy(_bno085Err, msg, sizeof(_bno085Err) - 1);
        _bno085Err[sizeof(_bno085Err) - 1] = '\0';
        xSemaphoreGive(_mutex);
    }
}

String SensorDriver::getLastBNO085Error() {
    String out;
    if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        out = String(_bno085Err);
        xSemaphoreGive(_mutex);
    }
    return out;
}

/* =========================================================================
 * I2C BAS NIVEAU
 *
 * Toutes les fonctions instrumentent le compteur d'échecs consécutifs du
 * watchdog I2C global : un échec l'incrémente, un succès le remet à zéro.
 * Au-delà de I2C_WATCHDOG_FAIL_THRESHOLD échecs d'affilée, la tâche capteurs
 * déclenche _recoverI2CBus() (reset bus + réinit de tous les capteurs).
 * ========================================================================= */

void SensorDriver::_i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    if (Wire.endTransmission() != 0) _i2cConsecFails++;
}

uint8_t SensorDriver::_i2cRead8(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { _i2cConsecFails++; return 0; }
    Wire.requestFrom((uint8_t)addr, (uint8_t)1);
    if (!Wire.available()) { _i2cConsecFails++; return 0; }
    _i2cConsecFails = 0;
    return Wire.read();
}

uint16_t SensorDriver::_i2cRead16(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { _i2cConsecFails++; return 0; }
    Wire.requestFrom((uint8_t)addr, (uint8_t)2);
    if (Wire.available() < 2) { _i2cConsecFails++; return 0; }
    uint16_t val = (uint16_t)Wire.read() << 8;
    val |= Wire.read();
    _i2cConsecFails = 0;
    return val;
}

bool SensorDriver::_i2cDevicePresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

/* -- I2C n°2 (Wire1 : MS5837 + PCA9685) : mêmes wrappers pour le second
 *    contrôleur matériel. Les échecs alimentent le même compteur global —
 *    les lectures INA226 réussies du bus n°1 le remettent à zéro en continu. */
void SensorDriver::_i2c2Write(uint8_t addr, uint8_t b1, uint8_t b2) {
    Wire1.beginTransmission(addr);
    Wire1.write(b1);
    Wire1.write(b2);
    if (Wire1.endTransmission() != 0) _i2cConsecFails++;
}

uint16_t SensorDriver::_i2c2Read16(uint8_t addr, uint8_t reg) {
    Wire1.beginTransmission(addr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) { _i2cConsecFails++; return 0; }
    Wire1.requestFrom((uint8_t)addr, (uint8_t)2);
    if (Wire1.available() < 2) { _i2cConsecFails++; return 0; }
    uint16_t val = (uint16_t)Wire1.read() << 8;
    val |= Wire1.read();
    _i2cConsecFails = 0;
    return val;
}

bool SensorDriver::_i2c2DevicePresent(uint8_t addr) {
    Wire1.beginTransmission(addr);
    return (Wire1.endTransmission() == 0);
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

    /* Tenter l'initialisation I2C à l'adresse 0x4A.
     * NOTE : le message d'échec est mémorisé dans _bno085Err pour être lu
     * depuis l'interface Web (diagnostic sans câble USB). */
    if (!_bno08x->begin_I2C(BNO085_I2C_ADDR)) {
        _status.bno085 = SENSOR_DISCONNECTED;
        _setBNO085Err("begin_I2C(0x4A) echoue");
        Serial.println("[SENSORS] BNO085 : échec begin_I2C(0x4A)");
        delete _bno08x;
        _bno08x = nullptr;
        return false;
    }

    /* Activer le Game Rotation Vector (relatif, sans magnéto) à 50 Hz — en
     * PREMIER : c'est la commande d'ouverture de la config de référence qui
     * fonctionnait ; le flux d'attitude démarre au plus tôt et le secours est
     * garanti avant la source principale. */
    const bool grvOk = _bno08x->enableReport(SH2_GAME_ROTATION_VECTOR, 20000);
    if (!grvOk) {
        Serial.println("[SENSORS] BNO085 : avertissement enableReport GAME_ROTATION_VECTOR échoué");
    }
    delay(50);   /* laisser le coprocesseur appliquer la commande avant la suivante */

    /* Activer le Rotation Vector (9-DOF : gyro + accéléro + MAGNÉTOMÈTRE) à
     * 50 Hz — source PRINCIPALE du cap : heading magnétique absolu, la raison
     * d'être du BNO085 sur ce ROV. Le capteur auto-calibre son magnétomètre au
     * fil des orientations rencontrées (déplacer le ROV en « 8 » accélère la
     * calibration — niveau 0–3 rapporté à l'interface via report.status).
     * NOTE : SH-2 n'accuse JAMAIS réception d'un Set Feature (sh2_setSensorConfig
     * ne garantit que l'écriture I2C) — une commande perdue = aucun rapport,
     * sans erreur visible : l'auto-guérison de _readBNO085 la renvoie alors
     * périodiquement jusqu'au démarrage du flux. */
    _rvCfgOk = _bno08x->enableReport(SH2_ROTATION_VECTOR, 20000);
    if (!_rvCfgOk) {
        Serial.println("[SENSORS] BNO085 : avertissement enableReport ROTATION_VECTOR échoué (I2C)");
    }
    delay(50);

    /* Compteurs de diagnostic remis à zéro pour cette session capteur */
    _rvReports = 0;  _grvReports = 0;  _rvLastRetryMs = 0;

    /* Aucune source de quaternions disponible → échec d'initialisation */
    if (!_rvCfgOk && !grvOk) {
        _status.bno085 = SENSOR_DISCONNECTED;
        _setBNO085Err("enableReport ROT/GAME echoue");
        Serial.println("[SENSORS] BNO085 : échec enableReport ROTATION_VECTOR et GAME_ROTATION_VECTOR");
        delete _bno08x;
        _bno08x = nullptr;
        return false;
    }

    /* Activer le gyroscope à 50 Hz (échec non fatal) */
    if (!_bno08x->enableReport(SH2_GYROSCOPE_CALIBRATED, 20000)) {
        Serial.println("[SENSORS] BNO085 : avertissement enableReport GYROSCOPE_CALIBRATED échoué");
    }

    _status.bno085 = SENSOR_CONNECTED;
    _setBNO085Err("");
    _lastBNO085Event = millis();   /* délai de grâce post-init pour la supervision */
    Serial.println("[SENSORS] BNO085 détecté et initialisé (0x4A) — Rotation Vector (magnétique) "
                   + String(_rvCfgOk ? "ON" : "OFF") + " + Game RV (secours) " + String(grvOk ? "ON" : "OFF")
                   + " + Gyroscope @ 50 Hz");
    return true;
}

bool SensorDriver::_initMS5803() {
    if (!_i2c2DevicePresent(MS5803_I2C_ADDR)) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 NON détecté (0x76 absent du bus n°2)");
        return false;
    }

    Wire1.beginTransmission(MS5803_I2C_ADDR);
    Wire1.write(0x1E);  /* RESET command */
    uint8_t err = Wire1.endTransmission();
    if (err != 0) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 NON détecté (RESET err=" + String(err) + ")");
        return false;
    }
    delay(10);

    /* Lecture des 6 coefficients de calibration (C1 à C6) */
    bool valid_cal = false;
    for (uint8_t i = 0; i < 6; i++) {
        _ms5803_cal[i] = _i2c2Read16(MS5803_I2C_ADDR, 0xA2 + (i * 2));
        if (_ms5803_cal[i] != 0 && _ms5803_cal[i] != 0xFFFF) valid_cal = true;
    }

    if (!valid_cal) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 : calibration PROM invalide → rejeté");
        return false;
    }

    _status.ms5803 = SENSOR_CONNECTED;
    _ms5803ConsecFails = 0;
    Serial.println("[SENSORS] MS5803-30BA / MS5837 initialisé (0x76, bus n°2)");
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
 *
 * Ces routines écrivent dans les buffers de travail (_imuWork / _powerWork),
 * JAMAIS directement dans _imu / _power. Elles sont appelées HORS mutex par la
 * tâche capteurs : un getSensorEvent() bloquant (timeout I2C 50 ms) ne gèle donc
 * plus les autres tâches (PID, WebSocket). La publication vers _imu / _power se
 * fait ensuite en une courte copie atomique SOUS mutex (voir _taskLoop).
 * ========================================================================= */

void SensorDriver::_readIMU() {
    if (_status.bno085 == SENSOR_CONNECTED) {
        _readBNO085();
    } else {
        /* IMU absente → forcer à zéro (buffer de travail) */
        memset(&_imuWork.quat, 0, sizeof(_imuWork.quat));
        _imuWork.quat[0] = 10000; /* Identité */
        memset(&_imuWork.gyro, 0, sizeof(_imuWork.gyro));
        memset(&_imuWork.euler, 0, sizeof(_imuWork.euler));
        _imuWork.heading_offset = 0.0f;   /* pas de cap → pas d'offset affichable */
        _imuWork.mag_active = 0;          /* IMU absente → pas de boussole non plus */
        _imuWork.mag_cal    = 0;
        _imuWork.mag_cfg_ok = 0;          /* diagnostics vides sans capteur */
        _imuWork.rv_reports  = 0;
        _imuWork.grv_reports = 0;
    }
}

void SensorDriver::_readPower() {
    const uint8_t addrs[3] = {INA226_1_I2C_ADDR, INA226_2_I2C_ADDR, INA226_3_I2C_ADDR};
    for (uint8_t i = 0; i < 3; i++) {
        if (_status.ina226[i] == SENSOR_CONNECTED) {
            /* Lecture tension bus (registre 0x04) : LSB = 1.25 mV */
            uint16_t bus_raw = _i2cRead16(addrs[i], 0x04);
            _powerWork.voltage[i] = (uint16_t)((float)bus_raw * 1.25f);
            /* Lecture courant via shunt (registre 0x01) */
            uint16_t shunt_raw = _i2cRead16(addrs[i], 0x01);
            int16_t shunt_signed = (int16_t)shunt_raw;
            float current_a = ((float)shunt_signed * 2.5e-6f) / 0.01f;
            _powerWork.current[i] = (uint16_t)(fabsf(current_a) * 1000.0f);
        } else {
            _powerWork.voltage[i] = 0;
            _powerWork.current[i] = 0;
        }
    }
}

/* =========================================================================
 * LECTURE BNO085 (quaternions + gyroscope via protocole SH-2)
 *
 * Drain des rapports disponibles, comme la version de référence fonctionnelle :
 * getSensorEvent() retourne false dès que le buffer SHTP du capteur est vide.
 * Le drain est simplement BORNÉ (BNO085_MAX_EVENTS_PER_READ) pour ne jamais
 * monopoliser le bus si le capteur déverse.
 * ========================================================================= */

/* Ramène un angle en degrés dans l'intervalle (−180, +180] (wrap-around cap) */
static inline float _wrapDeg180(float a) { return a - 360.0f * roundf(a / 360.0f); }

void SensorDriver::_readBNO085() {
    if (!_bno08x) return;

    sh2_SensorValue_t report;
    bool gotQuat = false;
    for (uint8_t n = 0; n < BNO085_MAX_EVENTS_PER_READ && _bno08x->getSensorEvent(&report); n++) {
        /* QUATERNION reçu → le flux vital est vivant : réarmer ICI la
         * supervision BNO085 (et uniquement ici — réarmer sur n'importe
         * quel rapport, gyroscope compris, laissait un capteur dont les
         * quaternions ne partaient plus poller en NACK à l'infini sans
         * jamais déclencher la récupération). */
        if (report.sensorId == SH2_ROTATION_VECTOR) {
            /* Rotation Vector 9-DOF (gyro + accéléro + MAGNÉTOMÈTRE) — source
             * PRINCIPALE du cap : heading magnétique, la raison d'être du
             * BNO085 sur ce ROV. Chaque rapport porte son niveau de calibration
             * (report.status, 0–3), exposé à l'interface. */
            _lastBNO085Event  = millis();
            _bno085FailStreak = 0;
            _rvTime   = _lastBNO085Event;
            _rvStatus = report.status;
            _rvQuat[0] = report.un.rotationVector.real;
            _rvQuat[1] = report.un.rotationVector.i;
            _rvQuat[2] = report.un.rotationVector.j;
            _rvQuat[3] = report.un.rotationVector.k;
            _rvReports++;   /* diagnostic : le flux magnétique vit */
            gotQuat = true;
        } else if (report.sensorId == SH2_GAME_ROTATION_VECTOR) {
            /* Game Rotation Vector (relatif, SANS magnéto) — SECOURS si le flux
             * Rotation Vector s'interrompt ou n'a pas pu être activé à l'init. */
            _lastBNO085Event  = millis();
            _bno085FailStreak = 0;
            _grvTime  = _lastBNO085Event;
            _grvQuat[0] = report.un.gameRotationVector.real;
            _grvQuat[1] = report.un.gameRotationVector.i;
            _grvQuat[2] = report.un.gameRotationVector.j;
            _grvQuat[3] = report.un.gameRotationVector.k;
            _grvReports++;   /* diagnostic : le secours relatif vit */
            gotQuat = true;
        } else if (report.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            /* Gyroscope en rad/s → °/s, scalé ×100 (comme la version de référence).
             * Axes remappés dans le repère VÉHICULE avec la même rotation de
             * montage BNO085_MOUNT_YAW_DEG que le quaternion : la vitesse
             * angulaire est un vecteur exprimé dans les axes du capteur, il
             * faut la faire suivre la rotation de repère pour rester cohérente
             * avec les Euler et le quaternion publiés (trame série RPi 5). */
            const float rad_to_deg = 180.0f / M_PI;
            const float gx = report.un.gyroscope.x * rad_to_deg;
            const float gy = report.un.gyroscope.y * rad_to_deg;
            const float gz = report.un.gyroscope.z * rad_to_deg;
            const float cg = cosf(BNO085_MOUNT_YAW_DEG * (float)M_PI / 180.0f);
            const float sg = sinf(BNO085_MOUNT_YAW_DEG * (float)M_PI / 180.0f);
            _imuWork.gyro[0] = (int16_t)(( cg * gx + sg * gy) * 100.0f);
            _imuWork.gyro[1] = (int16_t)((-sg * gx + cg * gy) * 100.0f);
            _imuWork.gyro[2] = (int16_t)(gz * 100.0f);
        }
    }

    /* ---- CHRONO DU WATCHDOG NIVEAU 3 : seul un QUATERNION RÉEL le réarme
     *      (jamais les tentatives de récupération, qui utilisent
     *      _lastBNO085Event — sinon le critère « délai critique » ne pourrait
     *      jamais conclure). 0 = capteur jamais fonctionnel depuis le boot. ---- */
    if (gotQuat) _lastBNO085QuatTime = millis();

    /* ---- AUTO-GUÉRISON Rotation Vector : SH-2 n'accuse pas les Set Feature
     *      (sh2_setSensorConfig ne garantit que l'écriture I2C). Si aucun
     *      rapport magnétique n'arrive — commande perdue au boot, hub occupé,
     *      activation refusée — on RENVOIE la configuration périodiquement
     *      (throttle 5 s) jusqu'au démarrage du flux. Coût plancher : une
     *      écriture I2C de ~15 octets toutes les 5 s au pire. ---- */
    const uint32_t nowMs = millis();
    if ((_rvTime == 0 || nowMs - _rvTime > 2000UL) &&
        nowMs - _rvLastRetryMs >= 5000UL) {
        _rvLastRetryMs = nowMs;
        _rvCfgOk = _bno08x->enableReport(SH2_ROTATION_VECTOR, 20000);
        Serial.println(String("[SENSORS] BNO085 : re-activation Rotation Vector ") +
                       (_rvCfgOk ? "envoyee" : "echouee (I2C)"));
    }

    if (!gotQuat) return;   /* pas de nouveau quaternion ce cycle */

    /* ---- Choix de la source : magnétique prioritaire, relatif en secours ----
     * Rotation Vector frais (≤ BNO085_MAG_STALE_MS) → cap magnétique ; sinon
     * Game Rotation Vector. Le basculement est rare (RV non activé à l'init
     * ou flux interrompu) et décidé une seule fois par cycle, jamais entre
     * deux rapports du même drain — évite les sauts de cap. */
    const bool   useMag = (_rvTime != 0) && (millis() - _rvTime <= BNO085_MAG_STALE_MS);
    const float* q = useMag ? _rvQuat : _grvQuat;

    /* ---- REMAPPAGE DE MONTAGE : quaternion capteur → repère véhicule ----
     * Le BNO085 est physiquement tourné de BNO085_MOUNT_YAW_DEG autour de la
     * verticale : X capteur vers la DROITE du ROV (X+), Y vers l'ARRIÈRE
     * (Y−), Z vers le bas. Composer À DROITE par la rotation fixe (qw, 0, 0,
     * qz) change le repère de DÉPART du quaternion (capteur → véhicule) sans
     * toucher au repère terrestre : le sens de variation du cap est préservé,
     * seul son zéro est décalé (absorbé par la tare Nord). Sans cela, lever
     * l'avant (Y+) faisait croître le ROLL et lever la droite (X+) le PITCH —
     * tangage/roulis PERMUTÉS sur l'horizon, le HSI, le modèle 3D et le PID.
     * Repère véhicule obtenu (NED : avant X, droite Y, bas Z) : lever l'avant
     * → pitch + ; penché à droite (droite qui descend) → roll + ; lever la
     * droite → roll −. Correction unique à la SOURCE, avant le scalage (trame
     * série RPi 5) et l'extraction des Euler ci-dessous. */
    const float halfRad = BNO085_MOUNT_YAW_DEG * (float)M_PI / 360.0f;
    const float qw = cosf(halfRad);
    const float qz = sinf(halfRad);
    float w = q[0]*qw - q[3]*qz;
    float x = q[1]*qw + q[2]*qz;
    float y = q[2]*qw - q[1]*qz;
    float z = q[3]*qw + q[0]*qz;

    _imuWork.quat[0] = (int16_t)(w * 10000.0f);
    _imuWork.quat[1] = (int16_t)(x * 10000.0f);
    _imuWork.quat[2] = (int16_t)(y * 10000.0f);
    _imuWork.quat[3] = (int16_t)(z * 10000.0f);

    /* Conversion quaternion → Euler — (w, x, y, z) : quaternion remappé dans
     * le repère véhicule, conservé à pleine précision (pas de relecture
     * quantifiée depuis _imuWork.quat). */

    /* Roll (X) */
    _imuWork.euler[0] = atan2f(2.0f*(w*x+y*z), 1.0f-2.0f*(x*x+y*y)) * 180.0f/M_PI;

    /* Pitch (Y) */
    float sinp = 2.0f*(w*y-z*x);
    if (fabsf(sinp) >= 1.0f) sinp = copysignf(1.0f, sinp);
    _imuWork.euler[1] = asinf(sinp) * 180.0f / M_PI;

    /* Yaw (Z) — magnétique tant que la source Rotation Vector est active.
     * Signe inversé : le quaternion SH-2 vit dans le repère NED (axe Z vers le
     * BAS), donc la formule ZYX rend un yaw croissant en sens ANTI-horaire vu
     * du dessus, alors que le cap aéronautique croît dans le sens HORAIRE
     * (nord → est). Roll et pitch ne sont pas concernés (axes X/Y du NED) :
     * une rotation à droite du capteur doit afficher un cap vers l'est. */
    _imuWork.euler[2] = -atan2f(2.0f*(w*z+x*y), 1.0f-2.0f*(y*y+z*z)) * 180.0f/M_PI;

    /* ---- Tare Nord (bouton « Régler Nord » de l'interface) ----
     * Avec le magnétomètre actif, la tare devient un AJUSTAGE FIN : elle absorbe
     * l'écart résiduel entre le Nord magnétique mesuré et le cap réel du ROV
     * (déclinaison locale ou léger biais). En secours relatif (Game Rotation
     * Vector, zéro arbitraire au boot), elle redevient la référence de cap
     * obligatoire. L'offset est appliqué À LA SOURCE (euler[2]) pour que
     * télémétrie, HSI, modèle 3D et PID de cap partagent le même Nord. */
    if (_northTareRequested) {
        _headingOffset = _wrapDeg180(_imuWork.euler[2]);
        _northTareRequested = false;
    }
    _imuWork.euler[2]       = _wrapDeg180(_imuWork.euler[2] - _headingOffset);
    _imuWork.heading_offset = _headingOffset;

    /* État de la boussole pour l'interface (source active + calibration 0–3) */
    _imuWork.mag_active = useMag ? 1 : 0;
    _imuWork.mag_cal    = useMag ? _rvStatus : 0;

    /* Diagnostics de la boussole (télémétrie : mag_cfg_ok / rv_reports /
     * grv_reports) — distinguent « commande transmise » de « flux vivant ». */
    _imuWork.mag_cfg_ok  = _rvCfgOk ? 1 : 0;
    _imuWork.rv_reports  = _rvReports;
    _imuWork.grv_reports = _grvReports;
}

/* =========================================================================
 * AUTO-RÉCUPÉRATION BNO085 / BUS I2C (sans broche INT) — WATCHDOG 3 NIVEAUX
 *
 * Quand le coprocesseur ARM du BNO085 renvoie un NACK, le driver i2c-ng de
 * l'ESP-IDF bascule en ESP_ERR_INVALID_STATE et chaque lecture SHTP échoue
 * (~30 ms de driver bloquant par tentative) alors que le reste du bus
 * continue de fonctionner. La récupération ESCALADE en trois niveaux :
 *   1. ré-init SHTP légère du seul capteur (premier échec) ;
 *   2. reset bus I2C matériel + réinit complète de tous les capteurs
 *      (échecs suivants — voir _recoverI2CBus / _resetI2CBus) ;
 *   3. ULTIME SECOURS : le capteur a FONCTIONNÉ puis demeure muet après
 *      BNO085_REBOOT_ATTEMPTS tentatives de niveau 2 (ou
 *      BNO085_REBOOT_TIMEOUT_MS sans quaternion — voir _taskLoop) →
 *      propulsion au neutre + esp_restart() (_bno085EmergencyRestart). Un
 *      capteur JAMAIS fonctionnel depuis le boot est exempté du niveau 3 :
 *      il retombe sur le mode espacé (tentative toutes les BNO085_BACKOFF_MS,
 *      badge rouge) pour ne pas boucler sur des redémarrages sans fin.
 * Toujours HORS mutex (le PID et le WebSocket continuent de tourner) ; le bus
 * n'ayant qu'un seul propriétaire (cette tâche), aucune protection de
 * concurrence supplémentaire n'est nécessaire côté I2C.
 * ========================================================================= */

void SensorDriver::_recoverBNO085() {
    /* ---- NIVEAU 3 (ultime secours) : le capteur A DÉJÀ fonctionné depuis le
     *      boot (_lastBNO085QuatTime != 0) mais BNO085_REBOOT_ATTEMPTS
     *      tentatives de niveau 2 n'ont rien restauré → neutralisation de la
     *      propulsion + redémarrage contrôlé de l'ESP32 (ne retourne jamais).
     *      Le garde-fou « jamais fonctionnel » laisse un capteur à demi-vivant
     *      dès la mise sous tension au MODE ESPACÉ ci-dessous : aucune boucle
     *      de redémarrages sans fin. ---- */
    if (_lastBNO085QuatTime != 0 &&
        _bno085FailStreak >= BNO085_STREAK_BUS_RESET - 1 + BNO085_REBOOT_ATTEMPTS) {
        _bno085EmergencyRestart("tentatives de recuperation epuisees");
    }

    /* ---- MODE ESPACÉ (capteur muet) : polling coupé, une tentative complète
     *      toutes les BNO085_BACKOFF_MS seulement — chaque NACK de lecture
     *      coûte ~30 ms de driver bloquant et étoufferait la tâche capteurs. ---- */
    if (_bno085FailStreak >= BNO085_STREAK_BACKOFF) {
        if (millis() - _lastBNO085Attempt < BNO085_BACKOFF_MS) {
            /* Sécurité anti-tempête : si la dernière tentative périodique avait
             * reconnecté le capteur sans restaurer le flux de quaternions, le
             * statut est repassé CONNECTED et le polling ré-empoisonne le bus —
             * re-couper immédiatement. */
            if (_status.bno085 == SENSOR_CONNECTED) {
                _status.bno085 = SENSOR_DISCONNECTED;
                _setBNO085Err("capteur muet : tentatives espacees 30 s");
                Serial.println("[SENSORS] BNO085 : toujours muet → polling SHTP coupé");
            }
            return;
        }
        Serial.println("[SENSORS] BNO085 muet : tentative périodique (reset bus + réinit complète)");
        _bno085Recoveries++;
        _lastBNO085Attempt = millis();
        _recoverI2CBus();
        /* Si la réinit réussit, le polling reprend ; le streak ne sera annulé
         * que par un quaternion réellement reçu (voir _readBNO085). */
        return;
    }

    /* ---- PHASE ACTIVE ---- */
    _bno085Recoveries++;
    _lastBNO085Attempt = millis();
    _bno085FailStreak++;

    /* Niveau 1 (premier échec seulement) : ré-init SHTP légère du capteur. */
    if (_bno085FailStreak < BNO085_STREAK_BUS_RESET) {
        Serial.println("[SENSORS] BNO085 : aucun quaternion depuis "
                       + String(BNO085_STALL_TIMEOUT_MS) + " ms → récupération n°"
                       + String(_bno085Recoveries) + " (réinit capteur)");
        if (_initBNO085()) {
            return;   /* init OK — le streak sera annulé au 1er quaternion reçu */
        }
        /* L'init elle-même échoue : passer directement au reset du bus. */
    }

    /* Niveau 2 : reset bus + réinitialisation COMPLÈTE de tous les capteurs
     * (délégation au watchdog I2C global) : l'effondrement a pu corrompre
     * l'état des INA226 / MS5803, et le BNO085 est réinitialisé même s'il
     * avait été raté au boot. */
    Serial.println("[SENSORS] BNO085 : reset bus I2C + réinit complète (échec n°"
                   + String(_bno085FailStreak) + ")");
    _recoverI2CBus();

    /* Capteur définitivement muet : couper le polling SHTP et espacer. */
    if (_bno085FailStreak >= BNO085_STREAK_BACKOFF &&
        _status.bno085 != SENSOR_CONNECTED) {
        _status.bno085 = SENSOR_DISCONNECTED;
        _setBNO085Err("capteur muet : tentatives espacees 30 s");
        Serial.println("[SENSORS] BNO085 : déclaré muet — polling coupé, tentatives toutes les "
                       + String(BNO085_BACKOFF_MS / 1000) + " s");
    }
}

/* =========================================================================
 * NIVEAU 3 — ULTIME SECOURS : NEUTRE FORCÉ + REDÉMARRAGE CONTRÔLÉ DE L'ESP32
 *
 * Dernier recours quand le BNO085 — déjà fonctionnel — ne répond toujours
 * plus après l'épuisement des niveaux 1 et 2 : le coprocesseur SH-2 ou le
 * contrôleur I2C de l'ESP32 est dans un état que le logiciel ne sait plus
 * réparer. Un redémarrage complet est la seule issue connue.
 *
 * ORDRE CRITIQUE POUR LA SÉCURITÉ :
 *   1. neutralisation immédiate des 16 canaux PCA9685 à 1500 µs — le
 *      PCA9685 est un générateur autonome qui CONSERVE ses registres pendant
 *      le reboot : les propulseurs restent physiquement au neutre tout le
 *      redémarrage ;
 *   2. logs explicites sur le moniteur série ;
 *   3. esp_restart() — le firmware repart en dry-run, sorties au neutre.
 * Appelée depuis la tâche capteurs (HORS mutex) ; ne retourne JAMAIS.
 * ========================================================================= */

void SensorDriver::_bno085EmergencyRestart(const char* reason) {
    /* 1. NEUTRALISATION IMMÉDIATE de la propulsion — écriture physique des
     *    1500 µs sur les 16 canaux (setAllNeutral est inconditionnel, même en
     *    dry-run : la sécurité primant sur le mode de test). */
    g_pwm.setAllNeutral();

    /* 2. Journalisation explicite (moniteur série) */
    Serial.println();
    Serial.println("[SENSORS] ==================================================");
    Serial.println("[SENSORS] !!! NIVEAU 3 : BNO085 IRRÉCUPÉRABLE (" + String(reason) + ") !!!");
    Serial.println("[SENSORS] Récupération SH-2 (niv.1) et reset bus I2C (niv.2) épuisés.");
    Serial.println("[SENSORS] Propulsion FORCÉE AU NEUTRE (1500 us, 16 canaux PCA9685).");
    Serial.println("[SENSORS] REDÉMARRAGE CONTRÔLÉ de l'ESP32 dans 200 ms (esp_restart).");
    Serial.println("[SENSORS] Le neutre physique est maintenu par le PCA9685 autonome");
    Serial.println("[SENSORS] pendant tout le redémarrage ; le firmware repart en dry-run.");
    Serial.println("[SENSORS] ==================================================");

    /* 3. Vider le FIFO série USB-CDC avant le reset, puis redémarrer. */
    Serial.flush();
    delay(200);
    esp_restart();   /* ne retourne jamais */
}

/* =========================================================================
 * VERROU PROPULSION « FLUX IMU MORT » — lu par la tâche de contrôle (100 Hz)
 * ========================================================================= */

bool SensorDriver::isBNO085Stalled() const {
    /* Vrai uniquement si le capteur A DÉJÀ fourni un quaternion depuis le boot
     * (garde-fou : le délai de démarrage du capteur et le mode espacé d'un
     * capteur jamais fonctionnel ne verrouillent pas la propulsion — ils sont
     * couverts par le neutre par défaut du boot). */
    return _bno085Present && _lastBNO085QuatTime != 0 &&
           (millis() - _lastBNO085QuatTime) > BNO085_STALL_TIMEOUT_MS;
}

/* =========================================================================
 * WATCHDOG I2C GLOBAL — RÉCUPÉRATION DU BUS
 *
 * Quand le bus I2C s'effondre (NACK → ESP_ERR_INVALID_STATE dans le driver
 * i2c-ng), TOUTES les transactions suivantes échouent en boucle : les lectures
 * INA226 (600/s) font alors grimper _i2cConsecFails très vite. Au-delà du seuil
 * I2C_WATCHDOG_FAIL_THRESHOLD, on reset le bus et on réinitialise TOUS les
 * capteurs présents — y compris le cas où le BNO085 n'a pas été vu au boot
 * (la supervision BNO085 seule ne déclencherait jamais : NACK infinis,
 * badge rouge permanent, télémétrie figée).
 * Exécuté HORS mutex par la tâche capteurs (unique propriétaire du bus) ;
 * cooldown I2C_WATCHDOG_COOLDOWN_MS partagé avec le déclencheur watchdog.
 * ========================================================================= */

void SensorDriver::_recoverI2CBus() {
    Serial.println("[SENSORS] Récupération bus I2C : " + String(_i2cConsecFails)
                   + " échecs consécutifs → reset bus + réinit de tous les capteurs");

    /* Détruire l'instance BNO08x AVANT le reset bus (état I2C interne obsolète) */
    if (_bno08x) {
        delete _bno08x;
        _bno08x = nullptr;
    }

    _resetI2CBus();
    delay(300);   /* Laisse le firmware SH-2 du BNO085 redémarrer après le reset du bus */

    /* Réinitialiser TOUS les capteurs — chacun vérifie lui-même sa présence
     * sur le bus, donc un capteur raté au boot peut réapparaître ici. */
    if (_bno085Present || _i2cDevicePresent(BNO085_I2C_ADDR)) {
        _bno085Present = true;
        if (_initBNO085()) {
            Serial.println("[SENSORS] Récupération bus I2C : BNO085 réinitialisé avec succès");
        } else {
            _setBNO085Err("init echoue apres reset bus");
        }
    }

    _initMS5803();   /* RESET + relecture PROM (vérifie seul sa présence) */

    /* INA226 : réinitialiser les trois wattmètres */
    const uint8_t ina_addrs[3] = {INA226_1_I2C_ADDR, INA226_2_I2C_ADDR, INA226_3_I2C_ADDR};
    for (uint8_t i = 0; i < 3; i++) {
        _initINA226(ina_addrs[i], i);
    }

    _i2cConsecFails  = 0;
    _lastBusRecovery = millis();
}

bool SensorDriver::_anySensorConnected() const {
    return (_status.bno085    == SENSOR_CONNECTED ||
            _status.ms5803    == SENSOR_CONNECTED ||
            _status.ina226[0] == SENSOR_CONNECTED ||
            _status.ina226[1] == SENSOR_CONNECTED ||
            _status.ina226[2] == SENSOR_CONNECTED);
}

void SensorDriver::_resetI2CBus() {
    /* 1. Libérer le contrôleur I2C matériel */
    Wire.end();
    delay(20);

    /* 2. Bit-bang : 9 impulsions d'horloge sur SCL pour débloquer un esclave
     *    qui retiendrait SDA au niveau bas (cas classique après un NACK). */
    pinMode(_currentSCL, OUTPUT);
    digitalWrite(_currentSCL, HIGH);
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(_currentSCL, LOW);
        delayMicroseconds(5);
        digitalWrite(_currentSCL, HIGH);
        delayMicroseconds(5);
    }
    /* Condition de STOP : SDA passe au haut pendant que SCL est haut */
    pinMode(_currentSDA, OUTPUT);
    digitalWrite(_currentSDA, LOW);
    delayMicroseconds(5);
    digitalWrite(_currentSCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(_currentSDA, HIGH);
    delayMicroseconds(5);

    /* 3. Rendre les broches au contrôleur I2C et le réarmer (400 kHz + timeout) */
    pinMode(_currentSDA, INPUT);
    pinMode(_currentSCL, INPUT);
    Wire.begin(_currentSDA, _currentSCL);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(I2C_TIMEOUT_MS);
    delay(20);

    Serial.println("[SENSORS] Bus I2C réinitialisé (9 impulsions SCL, SDA="
                   + String(_currentSDA) + ", SCL=" + String(_currentSCL) + ")");
}

/* =========================================================================
 * LECTURE MS5803 (pression + température + profondeur)
 * ========================================================================= */

bool SensorDriver::_readMS5803(PressureData& out) {
    /* ---- Conversion D1 (pression), OSR 4096 : ~9 ms — bus n°2 (Wire1) ---- */
    _i2c2Write(MS5803_I2C_ADDR, 0x48, 0x00);
    delay(10);
    Wire1.beginTransmission(MS5803_I2C_ADDR);
    Wire1.write(0x00);
    if (Wire1.endTransmission(false) != 0) { _i2cConsecFails++; return false; }
    Wire1.requestFrom((uint8_t)MS5803_I2C_ADDR, (uint8_t)3);
    if (Wire1.available() < 3) { _i2cConsecFails++; return false; }
    uint32_t D1 = ((uint32_t)Wire1.read() << 16)
                | ((uint32_t)Wire1.read() << 8) | Wire1.read();

    /* ---- Conversion D2 (température), OSR 4096 : ~9 ms — bus n°2 (Wire1) ---- */
    _i2c2Write(MS5803_I2C_ADDR, 0x58, 0x00);
    delay(10);
    Wire1.beginTransmission(MS5803_I2C_ADDR);
    Wire1.write(0x00);
    if (Wire1.endTransmission(false) != 0) { _i2cConsecFails++; return false; }
    Wire1.requestFrom((uint8_t)MS5803_I2C_ADDR, (uint8_t)3);
    if (Wire1.available() < 3) { _i2cConsecFails++; return false; }
    uint32_t D2 = ((uint32_t)Wire1.read() << 16)
                | ((uint32_t)Wire1.read() << 8) | Wire1.read();
    _i2cConsecFails = 0;   /* transaction complète réussie */

    /* ---- Compensation 24 bits (coefficients C1-C6) ---- */
    int32_t dT   = (int32_t)D2 - ((int32_t)_ms5803_cal[4] << 8);
    int32_t TEMP = 2000 + ((int64_t)dT * _ms5803_cal[5] >> 23);
    int64_t OFF  = ((int64_t)_ms5803_cal[1] << 16)
                 + (((int64_t)_ms5803_cal[3] * dT) >> 7);
    int64_t SENS = ((int64_t)_ms5803_cal[0] << 15)
                 + (((int64_t)_ms5803_cal[2] * dT) >> 8);
    int32_t P = (int32_t)(((int64_t)D1 * SENS >> 21) - OFF) >> 13;

    out.pressure_mbar = P;
    out.temperature   = TEMP;
    const float press_mbar = (float)P * 0.1f;   /* dixièmes de mbar → mbar */

    /* ---- Tare surface : capturée à la 1re lecture valide ou sur demande (bouton).
     *      P_surface = pression atmosphérique ambiante mesurée capteur hors de l'eau. ---- */
    if (_tareRequested) {
        _surfaceMbar   = press_mbar;
        _tareRequested = false;
    }
    out.surface_mbar = _surfaceMbar;

    /* ---- Détection surface / immersion : ΔP = P_mesurée − P_surface ---- */
    const float deltaP = press_mbar - _surfaceMbar;
    if (deltaP > MS5803_IMMERSION_THRESHOLD_MBAR) {
        /* IMMERGÉ (bathymètre) : profondeur eau douce = ΔP(Pa) / (ρ·g).
         * ΔP en mbar × 100 → Pa ; /(1000·9.81) → mètres. Altitude figée. */
        out.immersed   = true;
        out.depth_m    = (deltaP * 100.0f) / (FRESH_WATER_DENSITY * GRAVITY_MSS);
        out.baro_alt_m = _lastBaroAlt;              /* figée en immersion */
        out.altitude_m = -out.depth_m;              /* repère PID : négatif sous la surface */
    } else {
        /* HORS DE L'EAU (altimètre) : profondeur nulle, altitude barométrique réelle
         * via la formule internationale h = 44330·(1 − (P/P0)^0.190284). */
        out.immersed = false;
        out.depth_m  = 0.0f;
        _lastBaroAlt = 44330.0f *
            (1.0f - powf(press_mbar / SEA_LEVEL_PRESSURE_MBAR, 0.190284f));
        out.baro_alt_m = _lastBaroAlt;
        out.altitude_m = 0.0f;                      /* repère PID : à la surface */
    }
    return true;
}

/* =========================================================================
 * ZÉROS POUR CAPTEURS ABSENTS
 * ========================================================================= */

void SensorDriver::_zeroMissingSensors() {
    /* Géré implicitement par _readIMU/_readPower et par la publication sous
     * mutex dans _taskLoop, qui forcent à 0 si le capteur n'est pas connecté. */
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
        /* Requêtes bus de la tâche Web (scan / réinit broches) : exécutées par
         * cette tâche, seule propriétaire du bus — jamais d'accès concurrent. */
        _processBusRequests();

        uint32_t now = millis();

        /* ---- NIVEAU 3 (critère TEMPS, prioritaire) : le capteur A fonctionné
         *      depuis le boot puis plus AUCUN quaternion pendant
         *      BNO085_REBOOT_TIMEOUT_MS — les niveaux 1/2 (supervision
         *      ci-dessous) ont échoué ou sont trop lents : propulsion au neutre
         *      puis redémarrage contrôlé immédiat (ne retourne jamais). ---- */
        if (_bno085Present && _lastBNO085QuatTime != 0 &&
            (now - _lastBNO085QuatTime) > BNO085_REBOOT_TIMEOUT_MS) {
            _bno085EmergencyRestart("delai critique sans quaternion IMU");
        }

        /* ---- SUPERVISION BNO085 (sans broche INT) : si le capteur était présent
         *      au boot mais n'a fourni aucun QUATERNION depuis
         *      BNO085_STALL_TIMEOUT_MS (le gyro ne compte pas — voir
         *      _readBNO085), ses lectures SHTP échouent probablement en NACK.
         *      On tente une auto-récupération HORS mutex (le mutex reste libre
         *      pour le PID / WebSocket pendant la réinit). ---- */
        if (_bno085Present &&
            now - _lastBNO085Event >= BNO085_STALL_TIMEOUT_MS) {
            _recoverBNO085();
            _lastBNO085Event = millis();   /* réarme le watchdog quoi qu'il arrive */
            now = _lastBNO085Event;
        }

        /* ---- WATCHDOG I2C GLOBAL : échecs en cascade OU bus mort → reset + réinit ----
         *      Les lectures INA226 (600/s) instrumentent _i2cConsecFails. Si le bus
         *      est en ESP_ERR_INVALID_STATE chronique, le seuil est atteint en ~50 ms
         *      → reset bus + réinitialisation de tous les capteurs présents.
         *      Couvre le cas où le BNO085 n'a PAS été vu au boot (sinon la boucle
         *      de NACK infinie du moniteur série ne serait jamais réparée), et
         *      l'état « tous les capteurs perdus » (sinon une récupération ratée
         *      laisserait le système mort en silence : plus de lectures, plus
         *      d'échecs comptés, plus rien pour déclencher la suite). ---- */
        if ((_i2cConsecFails >= I2C_WATCHDOG_FAIL_THRESHOLD ||
             (_busHadSensors && !_anySensorConnected())) &&
            (now - _lastBusRecovery) >= I2C_WATCHDOG_COOLDOWN_MS) {
            _recoverI2CBus();
            _lastBNO085Event = millis();   /* réarme aussi la supervision BNO085 */
            now = millis();
        }

        /* ---- Baromètre MS5803 : conversion ADC lente (~20 ms), throttée à 2 Hz
         *      et exécutée HORS mutex. Les delay() de conversion ne bloquent donc
         *      plus les autres tâches (contrôle PID, broadcast WebSocket) qui
         *      attendent _mutex — c'était la cause du gel de l'affichage.
         *      Capteur débranché : déclaration de perte après MS5803_LOST_AFTER_FAILS
         *      échecs (sinon il resterait pollé en NACK toutes les 500 ms à vie),
         *      puis re-sonde périodique pour un rebranchage à chaud. ---- */
        PressureData pressLocal;
        bool pressUpdated = false;
        if (_status.ms5803 == SENSOR_CONNECTED &&
            (now - _lastMS5803Read >= MS5803_READ_PERIOD_MS)) {
            _lastMS5803Read = now;
            pressUpdated = _readMS5803(pressLocal);
            if (pressUpdated) {
                _ms5803ConsecFails = 0;
            } else if (++_ms5803ConsecFails >= MS5803_LOST_AFTER_FAILS) {
                _ms5803ConsecFails = 0;
                _status.ms5803 = SENSOR_DISCONNECTED;
                Serial.println("[SENSORS] MS5803 perdu (échecs de lecture consécutifs)"
                               " — re-sonde toutes les " + String(MS5803_REPROBE_MS / 1000) + " s");
            }
        } else if (_status.ms5803 == SENSOR_DISCONNECTED &&
                   (now - _lastMS5803Read >= MS5803_REPROBE_MS)) {
            /* Une sonde = une transaction vers 0x76 sur le bus n°2 (NACK immédiat si absent) */
            _lastMS5803Read = now;
            if (_i2c2DevicePresent(MS5803_I2C_ADDR)) {
                Serial.println("[SENSORS] MS5803 réapparu sur le bus n°2 → réinitialisation");
                _initMS5803();   /* réinit complète (PROM) ; reconnecté si OK */
            }
        }

        /* ---- IMU + puissance : lectures I2C exécutées HORS mutex dans les
         *      buffers de travail. getSensorEvent() peut bloquer jusqu'au timeout
         *      I2C (50 ms) sans INT ; hors mutex, cela ne gèle plus les lecteurs. ---- */
        _readIMU();
        _readPower();

        /* ---- PUBLICATION : courte copie atomique des buffers SOUS mutex ---- */
        xSemaphoreTake(_mutex, portMAX_DELAY);
        _imu   = _imuWork;
        _power = _powerWork;
        if (pressUpdated) {
            _pressure = pressLocal;                 /* copie rapide des nouvelles valeurs */
        } else if (_status.ms5803 != SENSOR_CONNECTED) {
            _pressure.pressure_mbar = 0;            /* baromètre absent → forcer à zéro */
            _pressure.temperature   = 0;
            _pressure.altitude_m    = 0.0f;
            _pressure.surface_mbar  = 0.0f;
            _pressure.depth_m       = 0.0f;
            _pressure.baro_alt_m    = 0.0f;
            _pressure.immersed      = false;
        }
        xSemaphoreGive(_mutex);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
    }
}
