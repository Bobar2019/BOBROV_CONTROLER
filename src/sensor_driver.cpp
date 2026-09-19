/**
 * @file sensor_driver.cpp
 * @brief Implémentation du pilote I2C multi-capteurs avec HAL.
 *
 * Capteurs supportés :
 *   - IMU : BNO085 (déf. 0x4A) via Adafruit BNO08x (protocole SH-2)
 *   - Baro : MS5803-30BA / MS5837 (déf. 0x76)
 *   - Wattmètres : INA3221 triple canal (déf. 0x40 — V_BAT + 2× ACS770 40 mV/A)
 *     et 2× INA226 résiduels (déf. 0x44, 0x45) — v1.1.0
 *   - PWM : PCA9685 (déf. 0x40, bus n°2) — géré séparément
 *
 * Répartition sur les deux bus I2C matériels de l'ESP32-S3 — ROUTAGE
 * DYNAMIQUE PLUG & PLAY (v1.3.0) : chaque module est assigné à son BUS et à
 * son ADRESSE par g_i2cRouter (NVS, page Paramètres) ; toutes les
 * transactions du pilote utilisent ce couple routé et les valeurs ci-dessous
 * ne sont que les DÉFAUTS de la topologie :
 *   - Bus n°1 (Wire,  GPIO 10/11) : BNO085 + INA3221 + 2× INA226
 *   - Bus n°2 (Wire1, GPIO 6/7)   : MS5837 + PCA9685
 *
 * Règles de sûreté appliquées au routage :
 *   - Le bus du PCA9685 n'est JAMAIS bit-bangé ni reseté en vol (les sorties
 *     PWM 100 Hz de la tâche de contrôle y vivent) — le watchdog et le scan
 *     ne réinitialisent que le bus non-PWM.
 *   - Le verrou _inaBusLock suit le bus ROUTÉ de l'INA3221 : la routine
 *     « Check Propulseurs » ne croise jamais une transaction de la tâche
 *     capteurs, quel que soit le bus.
 *   - CONFLIT GÉNÉRALISÉ (v1.3.0) : deux modules routés sur le MÊME bus à la
 *     MÊME adresse sont refusés par I2CRouter::save() et par l'API de
 *     sauvegarde (historique : INA3221 0x40 / PCA9685 0x40 sur bus distincts).
 *
 * @author Didier Dero
 * @version 2.3.0
 */

#include "sensor_driver.h"
#include "config.h"
#include "i2c_router.h"     /* g_i2cRouter : routage dynamique des bus (v1.2.0) */
#include "pwm_controller.h"   /* g_pwm : neutralisation de sécurité (niveau 3) */
#include "simulator.h"        /* g_sim : simulateur de test virtuel (établi) */
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
    , _inaBusLock(nullptr)
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
    /* QNH (v1.0.1) : standard au démarrage — remplacé par la valeur NVS/
     * Open-Meteo via setQnh() depuis le serveur Web si l'option est activée. */
    _qnhMbar       = SEA_LEVEL_PRESSURE_MBAR;
    /* Offset matériel (v1.0.1) : nul au démarrage — remplacé par la valeur
     * NVS via setHwOffset() si une calibration Open-Meteo a déjà eu lieu. */
    _hwOffsetMbar  = 0.0f;
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

    /* Carte de routage I2C Plug & Play (v1.2.0) : chargée AVANT toute sonde
     * ou initialisation — chaque capteur sera attaché au bus routé
     * (&Wire ou &Wire1). Premier démarrage / NVS vide → topologie par défaut
     * (i2c_router.cpp), aucun démarrage impossible. */
    g_i2cRouter.load();

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

    /* Verrou du bus ROUTÉ de l'INA3221 (v1.1.0, généralisé v1.2.0) : la
     * tâche capteurs reste propriétaire du bus, mais la routine « Check
     * Propulseurs » (tâche dédiée, post-WiFi) doit lire les courants ACS770
     * PENDANT les impulsions moteur — les deux se partagent CE bus par ce
     * verrou, une transaction à la fois, jamais d'accès croisé (mémoire :
     * bus effondré par concurrence). */
    _inaBusLock = xSemaphoreCreateMutex();

    /* Bus I2C n°2 (Wire1) : broches chargées depuis le NVS
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
                   + String(I2C_FREQ_HZ / 1000) + " kHz) — modules selon la carte de routage ci-dessus");

    /* Scan complet des deux bus I2C (récupération + timeouts réduits) —
     * remplace le scanner boot brut (v1.3.0) : _doScan publie son résultat
     * (mutex) pour la carte « Routage I2C Plug & Play » (GET /api/i2c/routing). */
    _doScan();

    /* Mémoriser la présence du BNO085 (sondé sur SON bus ET son adresse
     * ROUTÉS) : la tâche capteurs le supervisera en continu
     * (auto-récupération) même après un effondrement du bus. */
    _bno085Present = _i2cDevicePresent(g_i2cRouter.wireOf(I2C_DEV_BNO085),
                                       g_i2cRouter.addrOf(I2C_DEV_BNO085));

    /* Bus et broches du BNO085 (reset matériel des tentatives suivantes) */
    const uint8_t imuBusNo = g_i2cRouter.busOf(I2C_DEV_BNO085);

    /* Initialisation IMU : BNO085 avec boucle de réessais. La 1re tentative est
     * DIRECTE (comme la version de référence qui fonctionnait : le scan I2C ne
     * perturbe pas le handshake SHTP de begin_I2C). Le reset matériel du bus
     * n'intervient qu'aux tentatives suivantes, si un essai a laissé le bus
     * en erreur (NACK → ESP_ERR_INVALID_STATE) — sur le bus ROUTÉ du capteur,
     * opération sûre au boot (aucune tâche n'émet encore, même si ce bus
     * héberge le PCA9685). */
    bool imu_ok = false;
    for (uint8_t attempt = 1; attempt <= BNO085_INIT_MAX_ATTEMPTS && !imu_ok; attempt++) {
        if (attempt > 1) {
            _resetI2CBus(*g_i2cRouter.wireOfBus(imuBusNo), _sdaOf(imuBusNo), _sclOf(imuBusNo));
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
        Serial.println("[SENSORS] Aucune IMU détectée (BNO085 attendu à 0x"
                       + String(g_i2cRouter.addrOf(I2C_DEV_BNO085), HEX) + ") après "
                       + String(BNO085_INIT_MAX_ATTEMPTS) + " tentatives");
    }

    /* Initialisation baromètre : MS5837 sur son bus et son adresse routés */
    bool baro_ok = false;
    if (_i2cDevicePresent(g_i2cRouter.wireOf(I2C_DEV_MS5803),
                          g_i2cRouter.addrOf(I2C_DEV_MS5803))) {
        if (_initMS5803()) baro_ok = true;
    }
    if (!baro_ok) {
        Serial.println("[SENSORS] Aucun baromètre détecté (MS5803/MS5837 attendu à 0x"
                       + String(g_i2cRouter.addrOf(I2C_DEV_MS5803), HEX) + ")");
    }

    /* Initialisation triple wattmètre INA3221 (v1.1.0 — ex INA226 n°1) :
     * canal 1 = tension batterie 1, canaux 2/3 = courants ACS770 horizontaux /
     * verticaux. Puis les deux INA226 de la batterie 2 (électronique de
     * commande, projecteur LED) — chacun sur SON adresse routée (v1.3.0). */
    _initINA3221();
    const uint8_t ina_addrs[2] = {g_i2cRouter.addrOf(I2C_DEV_INA226_2),
                                  g_i2cRouter.addrOf(I2C_DEV_INA226_3)};
    for (uint8_t i = 0; i < 2; i++) {
        _initINA226(ina_addrs[i], i);
    }

    /* PCA9685 : sondé sur son bus ET son adresse routés — PWMController::begin()
     * l'initialise juste après sur le MÊME bus (main.cpp : g_sensors.begin()
     * précède g_pwm.begin(), les deux bus sont déjà configurés ici). */
    if (_i2cDevicePresent(g_i2cRouter.wireOf(I2C_DEV_PCA9685),
                          g_i2cRouter.addrOf(I2C_DEV_PCA9685))) {
        _status.pca9685 = SENSOR_CONNECTED;
        Serial.println("[SENSORS] PCA9685 détecté (0x" + String(g_i2cRouter.addrOf(I2C_DEV_PCA9685), HEX)
                       + ", bus n°" + String(g_i2cRouter.busOf(I2C_DEV_PCA9685)) + ")");
    } else {
        _status.pca9685 = SENSOR_DISCONNECTED;
    }

    /* Mémoriser si le bus avait au moins un capteur : arme le watchdog
     * « bus mort » (tentative de réinit complète périodique si TOUT est perdu
     * ultérieurement — évite l'état silencieusement mort sans issue). */
    _busHadSensors = (imu_ok || baro_ok ||
                      _status.ina3221 == SENSOR_CONNECTED ||
                      _status.ina226[0] == SENSOR_CONNECTED ||
                      _status.ina226[1] == SENSOR_CONNECTED);

    Serial.println("[SENSORS] Résumé : BNO085=" + String(imu_ok ? "OK" : "ABSENT")
                   + ", MS5803=" + String(baro_ok ? "OK" : "ABSENT"));

    /* Lancement de la tâche d'acquisition sur Core 1 */
    xTaskCreatePinnedToCore(
        _taskEntry, "Sensors", STACK_SIZE_SENSORS,
        this, PRIORITY_SENSORS, nullptr, CORE_RT
    );
}

/* =========================================================================
 * SCANNER I2C — identification par routage (voir _doScan / _scanOneBus)
 * ========================================================================= */

/**
 * @brief Identifie le composant probable à partir d'une adresse I2C et du bus.
 *
 * La CARTE DE ROUTAGE arbitre (v1.3.0) : un module est reconnu quand
 * l'adresse détectée est SON adresse routée sur SON bus routé — les six
 * modules ayant des adresses librement assignables (choisies parmi celles
 * découvertes par le scan), c'est le seul étiquetage fiable. Une adresse
 * qui ne correspond à aucun routage retombe sur des heuristiques de plage
 * (« non routé ») : un module fraîchement câblé apparaît ainsi dans le scan
 * AVANT d'être assigné dans la table « Routage I2C Plug & Play ».
 * Deux modules routés au même endroit (refusé par l'API de sauvegarde, mais
 * possible via NVS éditée à la main) sont signalés tels quels.
 */
static const char* _identifyI2CDevice(uint8_t addr, uint8_t bus) {
    /* Buffer statique : les appelants (scan) recopient le nom immédiatement
     * dans I2CDeviceInfo.name. */
    static char conflictBuf[48];

    uint8_t matches = 0;
    I2CDeviceId last = I2C_DEV_COUNT;
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;
        if (g_i2cRouter.addrOf(dev) == addr && g_i2cRouter.busOf(dev) == bus) {
            matches++;
            last = dev;
        }
    }
    if (matches == 1) {
        return I2CRouter::nameOf(last);   /* module routé à cette adresse */
    }
    if (matches > 1) {
        snprintf(conflictBuf, sizeof(conflictBuf),
                 "Conflit routage : %u modules à 0x%02X (bus n°%u)",
                 (unsigned)matches, addr, bus);
        return conflictBuf;
    }

    /* Aucun module routé à cette adresse : heuristiques de diagnostic */
    if (addr == 0x4A || addr == 0x4B) {
        return "BNO085 (adresse non routée)";
    }
    if (addr >= 0x40 && addr <= 0x43) {
        return "INA3221/PCA9685 (non routé)";
    }
    if (addr >= 0x44 && addr <= 0x4F) {
        return "INA226 (non routé)";
    }
    if (addr == 0x76 || addr == 0x77) {
        return "MS5803/MS5837 (non routé)";
    }
    return "Périphérique inconnu";
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
            const char* name = _identifyI2CDevice(addr, busNo);
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

std::vector<I2CDeviceInfo> SensorDriver::getLastScan() {
    /* Copie sous mutex : _doScan publie le résultat sous le même verrou —
     * jamais de lecture d'un vector en cours d'assignation. */
    std::vector<I2CDeviceInfo> out;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        out = _scanResult;
        xSemaphoreGive(_mutex);
    }
    return out;
}

void SensorDriver::_doScan() {
    /* Récupération du bus NON-PWM bloqué : 9 impulsions SCL pour libérer SDA.
     * Le bit-bang ne touche JAMAIS le bus routé du PCA9685 — partagé avec
     * les écritures PWM 100 Hz de la tâche de contrôle, un reset en vol
     * couperait la propulsion. Le bus récupéré est donc celui qui n'héberge
     * PAS le PCA9685 (par défaut le bus n°1 des capteurs). */
    const uint8_t recoveryBus = I2CRouter::otherBus(g_i2cRouter.pwmBus());
    const uint8_t recSda = _sdaOf(recoveryBus);
    const uint8_t recScl = _sclOf(recoveryBus);

    pinMode(recScl, OUTPUT);
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(recScl, LOW);
        delayMicroseconds(50);
        digitalWrite(recScl, HIGH);
        delayMicroseconds(50);
    }
    /* Condition STOP pour libérer le bus */
    pinMode(recSda, OUTPUT);
    digitalWrite(recSda, LOW);
    delayMicroseconds(50);
    digitalWrite(recScl, HIGH);
    delayMicroseconds(50);
    digitalWrite(recSda, HIGH);
    delayMicroseconds(50);

    /* Réinitialiser le contrôleur du bus récupéré après le bit-bang */
    TwoWire& recWire = *g_i2cRouter.wireOfBus(recoveryBus);
    recWire.end();
    delay(10);
    recWire.begin(recSda, recScl);
    recWire.setClock(I2C_FREQ_HZ);
    delay(10);

    /* Réduire le timeout I2C des deux bus pour accélérer le scan */
    unsigned long prevTimeout  = Wire.getTimeOut();
    unsigned long prevTimeout1 = Wire1.getTimeOut();
    Wire.setTimeOut(10);
    Wire1.setTimeOut(10);

    std::vector<I2CDeviceInfo> devices;
    _scanOneBus(Wire,  1, devices);   /* bus n°1 — modules selon routage */
    _scanOneBus(Wire1, 2, devices);   /* bus n°2 — modules selon routage */

    /* Restaurer les timeouts d'origine ; le bit-bang ci-dessus étant déjà une
     * récupération de bus, réarmer aussi le compteur du watchdog I2C global. */
    Wire.setTimeOut(prevTimeout);
    Wire1.setTimeOut(prevTimeout1);
    _i2cConsecFails = 0;

    /* Journaliser les périphériques trouvés (diagnostic moniteur série) */
    for (const auto& d : devices) {
        Serial.println("[SENSORS]   " + String(d.hexStr) + " trouvé (bus n°"
                       + String(d.bus) + ") — " + String(d.name));
    }
    Serial.println("[SENSORS] " + String((unsigned)devices.size()) + " périphérique(s) détecté(s)");

    /* Publier le résultat sous mutex : lisible par getLastScan() (tâche Web,
     * carte « Routage I2C Plug & Play » — GET /api/i2c/routing) et par
     * scanI2CBus() après _scanDone. */
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        _scanResult = devices;
        xSemaphoreGive(_mutex);
    } else {
        _scanResult = devices;   /* mutex indisponible (improbable) : publication directe */
    }
}

/* -------------------------------------------------------------------------
 * BROCHES I2C (bus n°1 ou n°2) — persistance NVS
 *
 * Les nouvelles broches sont enregistrées en NVS puis appliquées au prochain
 * démarrage : chaque bus peut héberger le PCA9685 selon le routage (un
 * Wire.end()/begin() en vol couperait les sorties PWM 100 Hz de la tâche de
 * contrôle), les deux bus suivent donc la même procédure uniforme. L'écriture
 * est vérifiée par RELECTURE : si le NVS refuse, la valeur n'est pas
 * confirmée et l'appelant ne programme pas le redémarrage.
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

void SensorDriver::setQnh(float qnhMbar) {
    /* Garde-fou : une valeur aberrante (réponse corrompue) ne doit jamais
     * polluer la formule d'altitude — hors plage physique → ignorée. */
    if (qnhMbar < QNH_MIN_MBAR || qnhMbar > QNH_MAX_MBAR) return;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _qnhMbar = qnhMbar;
        xSemaphoreGive(_mutex);
    }
    /* Mutex occupé (rare) : l'ancien QNH reste — la valeur NVS sera
     * réappliquée au prochain démarrage (non destructif). */
}

void SensorDriver::setHwOffset(float offsetMbar) {
    /* Garde-fou : un offset calculé aberrant (élévation/QNH corrompus,
     * capteur dérivant) ne doit jamais polluer l'altitude — hors borne →
     * ignoré, l'ancien offset reste appliqué. */
    if (fabsf(offsetMbar) > MS5803_HW_OFFSET_MAX_MBAR) return;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _hwOffsetMbar = offsetMbar;
        xSemaphoreGive(_mutex);
    }
    /* Mutex occupé (rare) : l'ancien offset reste — la valeur NVS sera
     * réappliquée au prochain démarrage (non destructif). */
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
 * I2C BAS NIVEAU ROUTÉ
 *
 * Une seule famille de wrappers, paramétrée par le bus : chaque appel désigne
 * son contrôleur par un pointeur TwoWire* issu de g_i2cRouter.wireOf()
 * (&Wire ou &Wire1 selon le routage du périphérique concerné). Toutes les
 * fonctions instrumentent le compteur d'échecs consécutifs du watchdog I2C
 * global : un échec l'incrémente, un succès le remet à zéro. Au-delà de
 * I2C_WATCHDOG_FAIL_THRESHOLD échecs d'affilée, la tâche capteurs déclenche
 * _recoverI2CBus() (reset bus non-PWM + réinit de tous les capteurs).
 * ========================================================================= */

void SensorDriver::_i2cWrite(TwoWire* bus, uint8_t addr, uint8_t reg, uint8_t val) {
    bus->beginTransmission(addr);
    bus->write(reg);
    bus->write(val);
    if (bus->endTransmission() != 0) _i2cConsecFails++;
}

uint16_t SensorDriver::_i2cRead16(TwoWire* bus, uint8_t addr, uint8_t reg) {
    bus->beginTransmission(addr);
    bus->write(reg);
    if (bus->endTransmission(false) != 0) { _i2cConsecFails++; return 0; }
    bus->requestFrom((uint8_t)addr, (uint8_t)2);
    if (bus->available() < 2) { _i2cConsecFails++; return 0; }
    uint16_t val = (uint16_t)bus->read() << 8;
    val |= bus->read();
    _i2cConsecFails = 0;
    return val;
}

bool SensorDriver::_i2cDevicePresent(TwoWire* bus, uint8_t addr) {
    bus->beginTransmission(addr);
    return (bus->endTransmission() == 0);
}

/* -- Écriture 2 octets (commande + valeur — conversions ADC du MS5803).
 *    Les échecs alimentent le même compteur global ; les lectures réussies
 *    des wattmètres (INA3221 + INA226, bus routé) le remettent à zéro. -- */
void SensorDriver::_i2cWrite2B(TwoWire* bus, uint8_t addr, uint8_t b1, uint8_t b2) {
    bus->beginTransmission(addr);
    bus->write(b1);
    bus->write(b2);
    if (bus->endTransmission() != 0) _i2cConsecFails++;
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

    /* Tenter l'initialisation I2C à l'adresse ROUTÉE SUR LE BUS ROUTÉ du
     * capteur (v1.3.0 : bus &Wire/&Wire1 ET adresse selon la carte NVS).
     * NOTE : le message d'échec est mémorisé dans _bno085Err pour être lu
     * depuis l'interface Web (diagnostic sans câble USB). */
    const uint8_t addr = g_i2cRouter.addrOf(I2C_DEV_BNO085);
    if (!_bno08x->begin_I2C(addr, g_i2cRouter.wireOf(I2C_DEV_BNO085))) {
        _status.bno085 = SENSOR_DISCONNECTED;
        _setBNO085Err((String("begin_I2C(0x") + String(addr, HEX) + ") echoue").c_str());
        Serial.println("[SENSORS] BNO085 : échec begin_I2C(0x" + String(addr, HEX) + ")");
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
    Serial.println("[SENSORS] BNO085 détecté et initialisé (0x" + String(addr, HEX) + ") — Rotation Vector (magnétique) "
                   + String(_rvCfgOk ? "ON" : "OFF") + " + Game RV (secours) " + String(grvOk ? "ON" : "OFF")
                   + " + Gyroscope @ 50 Hz");
    return true;
}

bool SensorDriver::_initMS5803() {
    /* Bus et adresse routés du baromètre (v1.3.0) */
    TwoWire* bus = g_i2cRouter.wireOf(I2C_DEV_MS5803);
    const uint8_t addr = g_i2cRouter.addrOf(I2C_DEV_MS5803);

    if (!_i2cDevicePresent(bus, addr)) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 NON détecté (0x" + String(addr, HEX)
                       + " absent du bus n°" + String(g_i2cRouter.busOf(I2C_DEV_MS5803)) + ")");
        return false;
    }

    bus->beginTransmission(addr);
    bus->write(0x1E);  /* RESET command */
    uint8_t err = bus->endTransmission();
    if (err != 0) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 NON détecté (RESET err=" + String(err) + ")");
        return false;
    }
    delay(10);

    /* Lecture des 6 coefficients de calibration (C1 à C6) */
    bool valid_cal = false;
    for (uint8_t i = 0; i < 6; i++) {
        _ms5803_cal[i] = _i2cRead16(bus, addr, 0xA2 + (i * 2));
        if (_ms5803_cal[i] != 0 && _ms5803_cal[i] != 0xFFFF) valid_cal = true;
    }

    if (!valid_cal) {
        _status.ms5803 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] MS5803 : calibration PROM invalide → rejeté");
        return false;
    }

    _status.ms5803 = SENSOR_CONNECTED;
    _ms5803ConsecFails = 0;
    Serial.println("[SENSORS] MS5803-30BA / MS5837 initialisé (0x" + String(addr, HEX) + ", bus n°"
                   + String(g_i2cRouter.busOf(I2C_DEV_MS5803)) + ")");
    return true;
}

bool SensorDriver::_initINA226(uint8_t addr, uint8_t index) {
    /* Bus routé du wattmètre : index 0 → INA226 n°2, index 1 → INA226 n°3 */
    TwoWire* bus = g_i2cRouter.wireOf(index == 0 ? I2C_DEV_INA226_2 : I2C_DEV_INA226_3);
    uint16_t mfr_id = _i2cRead16(bus, addr, 0xFE);
    if (mfr_id != 0x5449) {
        _status.ina226[index] = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] INA226[" + String(index) + "] @ 0x" + String(addr, HEX) + " NON détecté");
        return false;
    }

    /* Registre calibration (0x05) : shunt 0.01Ω, max 8A */
    _i2cWrite(bus, addr, 0x05, (uint8_t)(2098 >> 8));
    _i2cWrite(bus, addr, 0x05, (uint8_t)(2098 & 0xFF));

    _status.ina226[index] = SENSOR_CONNECTED;
    Serial.println("[SENSORS] INA226[" + String(index) + "] @ 0x" + String(addr, HEX) + " initialisé");
    return true;
}

/* -------------------------------------------------------------------------
 * INA3221 (v1.1.0) — triple wattmètre, pilote maison (registres TI SBVS197).
 * Canal 1 : tension batterie V_BAT (bus). Canaux 2/3 : signal absolu des
 * ACS770-100U (0,5 V à 0 A → 4,5 V à 100 A) lu sur le REGISTRE DE TENSION
 * BUS — le registre shunt (±163,84 mV) plafonnait la mesure à ±4,09 A.
 * ------------------------------------------------------------------------- */

bool SensorDriver::_initINA3221() {
    /* Bus et adresse routés du triple wattmètre (v1.3.0) — LE bus protégé
     * par _inaBusLock pour la routine « Check Propulseurs ». */
    TwoWire* bus = g_i2cRouter.wireOf(I2C_DEV_INA3221);
    const uint8_t addr = g_i2cRouter.addrOf(I2C_DEV_INA3221);
    uint16_t mfr_id = _i2cRead16(bus, addr, INA3221_REG_MFR_ID);
    if (mfr_id != INA3221_MFR_ID) {
        _status.ina3221 = SENSOR_DISCONNECTED;
        Serial.println("[SENSORS] INA3221 @ 0x" + String(addr, HEX) + " NON détecté (ID fabricant 0x"
                       + String(mfr_id, HEX) + ")");
        return false;
    }

    /* Configuration : 3 canaux shunt+bus en continu, 588 µs, moyennage ×16
     * (INA3221_CONFIG_VALUE) — stabilise les mesures ACS770 (bruit Hall). */
    uint8_t cfg_msb = (uint8_t)(INA3221_CONFIG_VALUE >> 8);
    uint8_t cfg_lsb = (uint8_t)(INA3221_CONFIG_VALUE & 0xFF);
    _i2cWrite(bus, addr, INA3221_REG_CONFIG, cfg_msb);
    _i2cWrite(bus, addr, INA3221_REG_CONFIG, cfg_lsb);

    _status.ina3221 = SENSOR_CONNECTED;
    Serial.println("[SENSORS] INA3221 @ 0x" + String(addr, HEX)
                   + " initialisé (3 canaux — V_BAT + 2× ACS770 40 mV/A, bus n°"
                   + String(g_i2cRouter.busOf(I2C_DEV_INA3221)) + ")");
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
    /* SIMULATEUR DE TEST VIRTUEL (établi) : quand la simulation est active,
     * AUCUNE transaction I2C n'est émise pour la puissance — les canaux 0-2
     * sont injectés par _readINA3221 (branche simulation ci-dessous) et les
     * canaux 3-4 (INA226 #2/#3) reçoivent la tension « batterie 2 » simulée,
     * courant nul. Les capteurs réels reprennent la main dès la désactivation
     * (état RAM volatile, jamais persisté). */
    const SimValues sim = g_sim.get();
    if (sim.active) {
        _readINA3221();
        _powerWork.voltage[3] = _powerWork.voltage[4] = (uint16_t)(sim.vbat2V * 1000.0f);
        _powerWork.current[3] = _powerWork.current[4] = 0;
        return;
    }

    /* INA3221 (v1.1.0) : canaux 0-2 du buffer de puissance. */
    _readINA3221();

    /* Deux INA226 résiduels : canaux 3-4 du buffer (adresses routées v1.3.0). */
    const uint8_t addrs[2] = {g_i2cRouter.addrOf(I2C_DEV_INA226_2),
                              g_i2cRouter.addrOf(I2C_DEV_INA226_3)};
    for (uint8_t i = 0; i < 2; i++) {
        const uint8_t outIdx = 3 + i;
        if (_status.ina226[i] == SENSOR_CONNECTED) {
            TwoWire* bus = g_i2cRouter.wireOf(i == 0 ? I2C_DEV_INA226_2 : I2C_DEV_INA226_3);
            /* Lecture tension bus (registre 0x04) : LSB = 1.25 mV */
            uint16_t bus_raw = _i2cRead16(bus, addrs[i], 0x04);
            _powerWork.voltage[outIdx] = (uint16_t)((float)bus_raw * 1.25f);
            /* Lecture courant via shunt (registre 0x01) */
            uint16_t shunt_raw = _i2cRead16(bus, addrs[i], 0x01);
            int16_t shunt_signed = (int16_t)shunt_raw;
            float current_a = ((float)shunt_signed * 2.5e-6f) / 0.01f;
            _powerWork.current[outIdx] = (uint32_t)(fabsf(current_a) * 1000.0f);
        } else {
            _powerWork.voltage[outIdx] = 0;
            _powerWork.current[outIdx] = 0;
        }
    }
}

bool SensorDriver::_readINA3221() {
    /* SIMULATEUR DE TEST VIRTUEL (établi) : valeurs injectées AVANT tout
     * contrôle d'état — le test fonctionne même INA3221 absent de l'établi et
     * AUCUNE transaction I2C n'est émise. Canaux 2/3 : le courant simulé
     * (banc H/V, voir _simBankCurrentMA et simulator.h) est reconverti en
     * signal ACS770 équivalent (500 mV à 0 A puis 40 mV/A) comme le capteur
     * réel, pour une télémétrie homogène entre modes réel et simulé. */
    const SimValues sim = g_sim.get();
    if (sim.active) {
        _powerWork.voltage[0] = (uint16_t)(sim.vbat1V * 1000.0f);
        _powerWork.current[0] = 0;
        const uint8_t simCh[2] = {MOTOR_CURRENT_CH_HORIZ, MOTOR_CURRENT_CH_VERT};
        for (uint8_t ch = 0; ch < 2; ch++) {
            const int32_t ma = _simBankCurrentMA(simCh[ch]);
            _powerWork.voltage[1 + ch] = (uint16_t)(ACS770_ZERO_A_MV
                + ((float)ma / 1000.0f) * (ACS770_SENSITIVITY_V_PER_A * 1000.0f));
            _powerWork.current[1 + ch] = (uint32_t)ma;
        }
        return true;
    }

    if (_status.ina3221 != SENSOR_CONNECTED) {
        _powerWork.voltage[0] = 0; _powerWork.current[0] = 0;
        _powerWork.voltage[1] = 0; _powerWork.current[1] = 0;
        _powerWork.voltage[2] = 0; _powerWork.current[2] = 0;
        return false;
    }

    /* Bus et adresse routés du triple wattmètre (v1.3.0) */
    TwoWire* bus = g_i2cRouter.wireOf(I2C_DEV_INA3221);
    const uint8_t addr = g_i2cRouter.addrOf(I2C_DEV_INA3221);

    /* Canal 1 : tension batterie V_BAT (registre bus — bits 14:3, LSB 8 mV). */
    uint16_t bus_raw = _i2cRead16(bus, addr, INA3221_REG_BUS1);
    _powerWork.voltage[0] = (uint16_t)((float)(bus_raw >> 3) * INA3221_BUS_LSB_MV);

    /* Canaux 2/3 : signal ACS770-100U (tension absolue 0,5–4,5 V câblée sur
     * IN−) → courant. Lecture sur le REGISTRE DE TENSION BUS (bits 14:3,
     * LSB 8 mV, plage 26 V) : le registre shunt (±163,84 mV) saturait toute
     * la mesure à ±4,09 A. Formule constructeur : I (A) = (V_bus_mV − 500)/40. */
    const uint8_t busRegs[2] = {INA3221_REG_BUS2, INA3221_REG_BUS3};
    for (uint8_t ch = 0; ch < 2; ch++) {
        uint16_t bus_raw = _i2cRead16(bus, addr, busRegs[ch]);
        const float bus_mV = (float)(bus_raw >> 3) * INA3221_BUS_LSB_MV;
        if (bus_mV < ACS770_VALID_MIN_MV) {
            /* Signal hors plage (capteur débranché / lecture I2C à 0) : la
             * formule donnerait −12,5 A → fausse surintensité. */
            _powerWork.voltage[1 + ch] = 0;
            _powerWork.current[1 + ch] = 0;
            continue;
        }
        const float current_a = (bus_mV - ACS770_ZERO_A_MV)
                              / (ACS770_SENSITIVITY_V_PER_A * 1000.0f);
        _powerWork.voltage[1 + ch] = (uint16_t)bus_mV;       /* signal ACS770 brut (mV) */
        _powerWork.current[1 + ch] = (uint32_t)(fabsf(current_a) * 1000.0f);
    }

    _powerWork.current[0] = 0;   /* canal 1 : tension seule (pas de shunt) */
    return true;
}

int32_t SensorDriver::readACS770CurrentmA(uint8_t channel) {
    /* channel = indice PowerData du courant (config.h) : 1 = horizontaux
     * (INA3221 canal 2), 2 = verticaux (INA3221 canal 3) — même sémantique
     * que PowerData::current[], pour un seul jeu de constantes côté appelant. */
    if (channel != MOTOR_CURRENT_CH_HORIZ && channel != MOTOR_CURRENT_CH_VERT) return -1;

    /* SIMULATEUR DE TEST VIRTUEL (établi) : court-circuit AVANT le contrôle
     * d'état et le verrou I2C — la mesure injectée est disponible même sans
     * INA3221 (établi sans matériel) et n'émet AUCUNE transaction vers le bus
     * partagé avec la tâche capteurs. */
    if (g_sim.get().active) return _simBankCurrentMA(channel);

    if (_status.ina3221 != SENSOR_CONNECTED) return -2;

    /* Verrou du bus ROUTÉ de l'INA3221 : la tâche capteurs (propriétaire du
     * bus) et la routine « Check Propulseurs » (tâche dédiée) doivent se
     * succéder SANS jamais croiser deux transactions I2C — même discipline
     * que la file de requêtes scan/réinit broches (mémoire : bus effondré
     * par accès concurrents). */
    if (_inaBusLock == nullptr ||
        xSemaphoreTake(_inaBusLock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return -3;
    }

    /* Signal ACS770-100U lu sur le registre de tension BUS du canal (tension
     * absolue 0,5–4,5 V, voir _readINA3221) — jamais le registre shunt. */
    const uint8_t reg = (channel == MOTOR_CURRENT_CH_HORIZ)
                      ? INA3221_REG_BUS2 : INA3221_REG_BUS3;
    uint16_t bus_raw = _i2cRead16(g_i2cRouter.wireOf(I2C_DEV_INA3221),
                                   g_i2cRouter.addrOf(I2C_DEV_INA3221), reg);
    const float bus_mV = (float)(bus_raw >> 3) * INA3221_BUS_LSB_MV;

    xSemaphoreGive(_inaBusLock);

    if (bus_mV < ACS770_VALID_MIN_MV) {
        /* Signal hors plage (capteur débranché / lecture I2C à 0) : lecture
         * impossible pour l'appelant (la formule donnerait faussement 12,5 A). */
        return -4;
    }

    const float current_a = (bus_mV - ACS770_ZERO_A_MV)
                          / (ACS770_SENSITIVITY_V_PER_A * 1000.0f);
    return (int32_t)(fabsf(current_a) * 1000.0f);
}

/* =========================================================================
 * SIMULATEUR DE TEST VIRTUEL — COURANT DE BANC (voir simulator.h)
 * ========================================================================= */

int32_t SensorDriver::_simBankCurrentMA(uint8_t channel) {
    const SimValues sim = g_sim.get();
    const bool horiz = (channel == MOTOR_CURRENT_CH_HORIZ);

    /* Masque des moteurs du banc : M1-M4 (bits 0-3) ou M5-M8 (bits 4-7). */
    const uint8_t bankMask = horiz
        ? (uint8_t)((1u << MOTOR_HORIZ_COUNT) - 1u)
        : (uint8_t)(((1u << MOTOR_VERT_COUNT) - 1u) << MOTOR_VERT_FIRST);

    /* Sémantique (simulator.h) : le courant du banc circule tant qu'au moins
     * un moteur bloqué coché du banc n'est pas isolé ; quand l'identification
     * cumulative (g_pwm.isolateMotor) a coupé tous les bloqués du banc, il
     * retombe à 0 — la chaîne de sécurité y voit un vrai moteur bloqué dont
     * l'isolement vient de faire chuter le courant. */
    const uint8_t stalled = sim.stallMask & bankMask;
    if (stalled != 0 && (stalled & (uint8_t)~g_pwm.getIsolatedMask()) == 0) {
        return 0;
    }

    /* Bornage défensif (l'API /api/simulator borne déjà) : 0..SIM_CURRENT_MAX_A. */
    float amps = horiz ? sim.curH_A : sim.curV_A;
    if (amps < 0.0f) amps = 0.0f;
    if (amps > SIM_CURRENT_MAX_A) amps = SIM_CURRENT_MAX_A;
    return (int32_t)(amps * 1000.0f);
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
 * I2C_WATCHDOG_FAIL_THRESHOLD, on reset le bus NON-PWM et on réinitialise
 * TOUS les capteurs présents — y compris le cas où le BNO085 n'a pas été vu au boot
 * (la supervision BNO085 seule ne déclencherait jamais : NACK infinis,
 * badge rouge permanent, télémétrie figée).
 * Exécuté HORS mutex par la tâche capteurs (unique propriétaire du bus) ;
 * cooldown I2C_WATCHDOG_COOLDOWN_MS partagé avec le déclencheur watchdog.
 * ========================================================================= */

void SensorDriver::_recoverI2CBus() {
    /* Reset UNIQUEMENT le bus NON-PWM (v1.2.0) : celui qui héberge le PCA9685
     * ne doit jamais être bit-bangé/reseté en vol (sorties PWM 100 Hz de la
     * tâche de contrôle). Les capteurs routés sur le bus PWM sont réinitialisés
     * (ré-écriture de leurs registres) mais leur bus ne peut pas être réparé
     * matériellement à chaud — limitation documentée du routage. */
    const uint8_t recoveryBus = I2CRouter::otherBus(g_i2cRouter.pwmBus());

    Serial.println("[SENSORS] Récupération bus I2C (non-PWM n°" + String(recoveryBus)
                   + ") : " + String(_i2cConsecFails)
                   + " échecs consécutifs → reset bus + réinit de tous les capteurs");

    /* Détruire l'instance BNO08x AVANT le reset bus (état I2C interne obsolète) */
    if (_bno08x) {
        delete _bno08x;
        _bno08x = nullptr;
    }

    _resetI2CBus(*g_i2cRouter.wireOfBus(recoveryBus),
                 _sdaOf(recoveryBus), _sclOf(recoveryBus));
    delay(300);   /* Laisse le firmware SH-2 du BNO085 redémarrer après le reset du bus */

    /* Réinitialiser TOUS les capteurs — chacun vérifie lui-même sa présence
     * sur SON bus routé, donc un capteur raté au boot peut réapparaître ici. */
    if (_bno085Present ||
        _i2cDevicePresent(g_i2cRouter.wireOf(I2C_DEV_BNO085), g_i2cRouter.addrOf(I2C_DEV_BNO085))) {
        _bno085Present = true;
        if (_initBNO085()) {
            Serial.println("[SENSORS] Récupération bus I2C : BNO085 réinitialisé avec succès");
        } else {
            _setBNO085Err("init echoue apres reset bus");
        }
    }

    _initMS5803();   /* RESET + relecture PROM (vérifie seul sa présence) */

    /* INA3221 puis INA226 résiduels : réinitialiser les wattmètres,
     * chacun sur SON adresse routée (v1.3.0) */
    _initINA3221();
    const uint8_t ina_addrs[2] = {g_i2cRouter.addrOf(I2C_DEV_INA226_2),
                                  g_i2cRouter.addrOf(I2C_DEV_INA226_3)};
    for (uint8_t i = 0; i < 2; i++) {
        _initINA226(ina_addrs[i], i);
    }

    _i2cConsecFails  = 0;
    _lastBusRecovery = millis();
}

bool SensorDriver::_anySensorConnected() const {
    return (_status.bno085    == SENSOR_CONNECTED ||
            _status.ms5803    == SENSOR_CONNECTED ||
            _status.ina3221   == SENSOR_CONNECTED ||
            _status.ina226[0] == SENSOR_CONNECTED ||
            _status.ina226[1] == SENSOR_CONNECTED);
}

void SensorDriver::_resetI2CBus(TwoWire& bus, uint8_t sda, uint8_t scl) {
    /* 1. Libérer le contrôleur I2C matériel */
    bus.end();
    delay(20);

    /* 2. Bit-bang : 9 impulsions d'horloge sur SCL pour débloquer un esclave
     *    qui retiendrait SDA au niveau bas (cas classique après un NACK). */
    pinMode(scl, OUTPUT);
    digitalWrite(scl, HIGH);
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(scl, LOW);
        delayMicroseconds(5);
        digitalWrite(scl, HIGH);
        delayMicroseconds(5);
    }
    /* Condition de STOP : SDA passe au haut pendant que SCL est haut */
    pinMode(sda, OUTPUT);
    digitalWrite(sda, LOW);
    delayMicroseconds(5);
    digitalWrite(scl, HIGH);
    delayMicroseconds(5);
    digitalWrite(sda, HIGH);
    delayMicroseconds(5);

    /* 3. Rendre les broches au contrôleur I2C et le réarmer (400 kHz + timeout) */
    pinMode(sda, INPUT);
    pinMode(scl, INPUT);
    bus.begin(sda, scl);
    bus.setClock(I2C_FREQ_HZ);
    bus.setTimeOut(I2C_TIMEOUT_MS);
    delay(20);

    Serial.println("[SENSORS] Bus I2C réinitialisé (9 impulsions SCL, SDA="
                   + String(sda) + ", SCL=" + String(scl) + ")");
}

/* =========================================================================
 * LECTURE MS5803 (pression + température + profondeur)
 * ========================================================================= */

bool SensorDriver::_readMS5803(PressureData& out) {
    /* Bus et adresse routés du baromètre (v1.3.0) — conversions ADC lentes (~9 ms chacune) */
    TwoWire* bus = g_i2cRouter.wireOf(I2C_DEV_MS5803);
    const uint8_t addr = g_i2cRouter.addrOf(I2C_DEV_MS5803);

    /* ---- Conversion D1 (pression), OSR 4096 : ~9 ms ---- */
    _i2cWrite2B(bus, addr, 0x48, 0x00);
    delay(10);
    bus->beginTransmission(addr);
    bus->write(0x00);
    if (bus->endTransmission(false) != 0) { _i2cConsecFails++; return false; }
    bus->requestFrom((uint8_t)addr, (uint8_t)3);
    if (bus->available() < 3) { _i2cConsecFails++; return false; }
    uint32_t D1 = ((uint32_t)bus->read() << 16)
                | ((uint32_t)bus->read() << 8) | bus->read();

    /* ---- Conversion D2 (température), OSR 4096 : ~9 ms ---- */
    _i2cWrite2B(bus, addr, 0x58, 0x00);
    delay(10);
    bus->beginTransmission(addr);
    bus->write(0x00);
    if (bus->endTransmission(false) != 0) { _i2cConsecFails++; return false; }
    bus->requestFrom((uint8_t)addr, (uint8_t)3);
    if (bus->available() < 3) { _i2cConsecFails++; return false; }
    uint32_t D2 = ((uint32_t)bus->read() << 16)
                | ((uint32_t)bus->read() << 8) | bus->read();
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
         * via la formule internationale h = 44330·(1 − (P/P0)^0.190284).
         * P0 = QNH courant (standard 1013.25 par défaut, valeur Open-Meteo si
         * l'option est activée — v1.0.1) : ne corrige QUE l'altitude, jamais la
         * tare surface ni la profondeur immergée. */
        out.immersed = false;
        out.depth_m  = 0.0f;
        /* P_corrigée = P_brute − offset matériel : la compensation ne touche
         * QUE l'altitude (v1.0.1). Tare de surface et ΔP/profondeur restent
         * fondés sur la pression absolue brute (press_mbar) — cloisonnement
         * critique altitude/profondeur préservé de bout en bout. */
        const float pCorrMbar = press_mbar - _hwOffsetMbar;
        _lastBaroAlt = 44330.0f *
            (1.0f - powf(pCorrMbar / _qnhMbar, 0.190284f));
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
        /* Verrou du bus ROUTÉ de l'INA3221, partagé avec la routine « Check
         * Propulseurs » (v1.1.0, généralisé v1.2.0) : toute l'itération
         * ci-dessous (requêtes bus, récupérations, lectures) reste
         * propriétaire exclusive du bus. En cas d'échec d'acquisition
         * (impulsion de check en cours — une lecture I2C au pire ~50 ms de
         * timeout), on SAUTE les accès I2C de CE cycle : la publication et le
         * cadencement 100 Hz continuent, jamais de blocage indéfini. */
        const bool inaBusOwned = (_inaBusLock != nullptr) &&
            (xSemaphoreTake(_inaBusLock, pdMS_TO_TICKS(100)) == pdTRUE);

        /* Requêtes bus de la tâche Web (scan / réinit broches) : exécutées par
         * cette tâche, seule propriétaire du bus — jamais d'accès concurrent. */
        if (inaBusOwned) _processBusRequests();

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
            /* Une sonde = une transaction vers l'adresse routée, sur le bus routé
             * (NACK immédiat si absent) */
            _lastMS5803Read = now;
            if (_i2cDevicePresent(g_i2cRouter.wireOf(I2C_DEV_MS5803),
                                  g_i2cRouter.addrOf(I2C_DEV_MS5803))) {
                Serial.println("[SENSORS] MS5803 réapparu sur son bus routé → réinitialisation");
                _initMS5803();   /* réinit complète (PROM) ; reconnecté si OK */
            }
        }

        /* ---- IMU + puissance : lectures I2C exécutées HORS mutex dans les
         *      buffers de travail. getSensorEvent() peut bloquer jusqu'au timeout
         *      I2C (50 ms) sans INT ; hors mutex, cela ne gèle plus les lecteurs.
         *      Bus de l'INA3221 occupé par le check propulseurs : valeurs du
         *      cycle précédent republiées (stables) — la reprise suit au prochain
         *      cycle. ---- */
        if (inaBusOwned) {
            _readIMU();
            _readPower();
        }

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

        if (inaBusOwned) xSemaphoreGive(_inaBusLock);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SENSORS_PERIOD_MS));
    }
}
