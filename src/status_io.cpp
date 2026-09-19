/**
 * @file status_io.cpp
 * @brief Implémentation des entrées de sécurité et du bandeau LED WS2812 (v1.0.0).
 *
 * Tâche dédiée « StatusIO » (Core 0, 40 Hz, sous la priorité Web) :
 *  - échantillonnage anti-rebond des 2 entrées à polarité configurable (~75 ms) ;
 *  - rendu du bandeau NeoPixel selon la matrice NVS des 5 états (effet,
 *    couleur et luminosité 1-100 % par état — économie d'énergie), avec
 *    détection de changement (aucun rafraîchissement inutile) et cadence
 *    bornée pour les effets animés (LED_ANIM_FRAME_MS).
 *
 * La tâche tourne sur le Core Wi-Fi : le bit-bang NeoPixel (interruptions
 * coupées quelques centaines de µs, ≈3 ms au pire pour 100 LEDs) ne doit
 * jamais retarder la boucle de contrôle temps réel du Core 1.
 *
 * L'alarme « voie d'eau » est publiée via isWaterAlarm() : c'est la tâche de
 * contrôle (100 Hz) qui écrase le mode de vol en Retour Surface via
 * FlightController::setSurfaceOverride() — une seule source d'écriture du
 * mode, jamais deux tâches concurrentes.
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Septembre 2026
 */

#include "status_io.h"
#include "sensor_driver.h"      /* isBNO085Stalled (état « Avarie ») */
#include "motor_manager.h"      /* isFaultActive / getIsolatedMask (état « Avarie ») */
#include "flight_controller.h"  /* getActiveMode / isSurfaceReturnActive */
#include "serial_comm.h"        /* getLastFrameTime / isWatchdogTriggered */

#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Instance globale des entrées de sécurité + bandeau LED d'état */
StatusIO g_statusIo;

/* =========================================================================
 * HELPERS INTERNES
 * ========================================================================= */

/**
 * @brief Lit une entrée numérique selon sa polarité.
 *
 * @param pin        Broche à lire (STATUS_IO_PIN_NONE filtré par l'appelant)
 * @param activeHigh true = actif au niveau haut (+3,3 V), false = actif à GND
 * @return true si l'entrée est au niveau ACTIF.
 */
static inline bool _inputActive(uint8_t pin, bool activeHigh) {
    const bool level = (digitalRead(pin) == HIGH);
    return activeHigh ? level : !level;
}

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

StatusIO::StatusIO()
    : _mutex(nullptr)
    , _armPin(STATUS_IO_PIN_NONE)
    , _waterPin(STATUS_IO_PIN_NONE)
    , _ledPin(STATUS_IO_PIN_NONE)
    , _ledCountApplied(LED_WS2812_NUM_DEFAULT)
    , _armActiveHigh(false)
    , _waterActiveHigh(false)
    , _strip(nullptr)
    , _armStable(false)
    , _waterStable(false)
    , _armDebounce(0)
    , _waterDebounce(0)
    , _activeLedState(255)
    , _paintedState(255)   /* sentinelle : force une première peinture */
    , _paintedR(0)
    , _paintedG(0)
    , _paintedB(0)
    , _lastAnimMs(0)
{
    /* Défauts complets : broches non assignées, polarité active GND, matrice
     * selon config.h */
    _cfg.armPin   = -1;
    _cfg.waterPin = -1;
    _cfg.ledPin   = -1;
    _cfg.ledCount = LED_WS2812_NUM_DEFAULT;
    _cfg.armActiveHigh   = STATUS_IO_ACTIVE_LOW;
    _cfg.waterActiveHigh = STATUS_IO_ACTIVE_LOW;
    for (uint8_t i = 0; i < LED_STATE_COUNT; i++) {
        _cfg.led[i].effect     = LED_DEFAULT_EFFECT[i];
        _cfg.led[i].r          = LED_DEFAULT_RGB[i][0];
        _cfg.led[i].g          = LED_DEFAULT_RGB[i][1];
        _cfg.led[i].b          = LED_DEFAULT_RGB[i][2];
        _cfg.led[i].brightness = LED_WS2812_BRI_DEFAULT;
    }
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

void StatusIO::begin() {
    _mutex = xSemaphoreCreateMutex();

    /* ---- 1. Chargement de la configuration NVS (namespace « statusio ») ----
     * Chaque lecture retombe sur les défauts si la clé est absente. */
    {
        Preferences prefs;
        prefs.begin("statusio", true);

        const uint8_t armIn   = prefs.getUChar("in_arm",  STATUS_IO_PIN_NONE);
        const uint8_t waterIn = prefs.getUChar("in_water", STATUS_IO_PIN_NONE);
        const uint8_t ledIn   = prefs.getUChar("led_pin", STATUS_IO_PIN_NONE);

        /* Blob de la matrice : relu seulement si complet (25 octets).
         * Migration transparente de l'ancien format 20 octets (effet +
         * couleur seuls) : la luminosité est reprise de l'ancienne clé
         * globale « led_bri » quand elle existe. */
        {
            uint8_t raw[LED_STATE_COUNT * sizeof(LedStateConfig)];
            const size_t n = prefs.getBytes("led_cfg", raw, sizeof(raw));
            if (n == sizeof(raw)) {
                memcpy(_cfg.led, raw, sizeof(raw));
            } else if (n == LED_STATE_COUNT * 4) {
                const uint8_t oldBri = prefs.getUChar("led_bri", LED_WS2812_BRI_DEFAULT);
                for (uint8_t i = 0; i < LED_STATE_COUNT; i++) {
                    _cfg.led[i].effect     = raw[i * 4 + 0];
                    _cfg.led[i].r          = raw[i * 4 + 1];
                    _cfg.led[i].g          = raw[i * 4 + 2];
                    _cfg.led[i].b          = raw[i * 4 + 3];
                    _cfg.led[i].brightness = oldBri;
                }
                Serial.println("[STATUS] Matrice LED migrée au format v2 (luminosité par état)");
            }
        }
        _cfg.ledCount = prefs.getUChar("led_count", LED_WS2812_NUM_DEFAULT);
        _cfg.armActiveHigh   = prefs.getUChar("in_arm_hi",   STATUS_IO_ACTIVE_LOW) ? 1 : 0;
        _cfg.waterActiveHigh = prefs.getUChar("in_water_hi", STATUS_IO_ACTIVE_LOW) ? 1 : 0;
        prefs.end();

        /* Bornage défensif : une NVS corrompue ne doit jamais produire une
         * broche inexistante ni un nombre de LEDs hors bornes. */
        _armPin   = (armIn   != STATUS_IO_PIN_NONE && armIn   <= 48
                     && !isMemoryBusPin(armIn))   ? armIn   : STATUS_IO_PIN_NONE;
        _waterPin = (waterIn != STATUS_IO_PIN_NONE && waterIn <= 48
                     && !isMemoryBusPin(waterIn)) ? waterIn : STATUS_IO_PIN_NONE;
        _ledPin   = (ledIn   != STATUS_IO_PIN_NONE && ledIn   <= 48
                     && !isMemoryBusPin(ledIn))   ? ledIn   : STATUS_IO_PIN_NONE;
        _cfg.armPin   = (_armPin   == STATUS_IO_PIN_NONE) ? -1 : (int8_t)_armPin;
        _cfg.waterPin = (_waterPin == STATUS_IO_PIN_NONE) ? -1 : (int8_t)_waterPin;
        _cfg.ledPin   = (_ledPin   == STATUS_IO_PIN_NONE) ? -1 : (int8_t)_ledPin;
        if (_cfg.ledCount < LED_WS2812_NUM_MIN) _cfg.ledCount = LED_WS2812_NUM_MIN;
        if (_cfg.ledCount > LED_WS2812_NUM_MAX) _cfg.ledCount = LED_WS2812_NUM_MAX;
        for (uint8_t i = 0; i < LED_STATE_COUNT; i++) {
            if (_cfg.led[i].effect > LED_EFFECT_OFF) _cfg.led[i].effect = LED_EFFECT_SOLID;
            if (_cfg.led[i].brightness < LED_WS2812_BRI_MIN) _cfg.led[i].brightness = LED_WS2812_BRI_MIN;
            if (_cfg.led[i].brightness > LED_WS2812_BRI_MAX) _cfg.led[i].brightness = LED_WS2812_BRI_MAX;
        }

        /* GPIO 35-37 (bus SPI flash/PSRAM interne des modules octaux) : une
         * affectation héritée est ignorée — à reconfigurer depuis le Web. */
        if ((armIn   != STATUS_IO_PIN_NONE && armIn   <= 48 && isMemoryBusPin(armIn))
         || (waterIn != STATUS_IO_PIN_NONE && waterIn <= 48 && isMemoryBusPin(waterIn))
         || (ledIn   != STATUS_IO_PIN_NONE && ledIn   <= 48 && isMemoryBusPin(ledIn))) {
            Serial.println("[STATUS] GPIO 35-37 réservés au bus flash/PSRAM (module octal) — affectation ignorée");
        }
    }

    /* ---- 2. Entrées numériques : pull selon polarité + état initial ---- */
    _armActiveHigh   = (_cfg.armActiveHigh   != 0);
    _waterActiveHigh = (_cfg.waterActiveHigh != 0);
    if (_armPin != STATUS_IO_PIN_NONE) {
        pinMode(_armPin, _armActiveHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
        _armStable = _inputActive(_armPin, _armActiveHigh);
    }
    if (_waterPin != STATUS_IO_PIN_NONE) {
        pinMode(_waterPin, _waterActiveHigh ? INPUT_PULLDOWN : INPUT_PULLUP);
        _waterStable = _inputActive(_waterPin, _waterActiveHigh);
    }

    /* ---- 3. Bandeau NeoPixel (instance créée au boot uniquement) ---- */
    if (_ledPin != STATUS_IO_PIN_NONE) {
        _ledCountApplied = _cfg.ledCount;
        _strip = new Adafruit_NeoPixel(_ledCountApplied, _ledPin, NEO_GRB + NEO_KHZ800);
        if (_strip != nullptr) {
            _strip->begin();
            _strip->clear();
            _strip->show();   /* bandeau éteint au démarrage */
        }
    }

    Serial.println("[STATUS] Entrées numériques : armement="
                   + String(_armPin == STATUS_IO_PIN_NONE
                            ? String("non assigné")
                            : "GPIO " + String(_armPin)
                              + (_armActiveHigh ? " (actif +3,3 V)" : " (actif GND)"))
                   + ", voie d'eau="
                   + String(_waterPin == STATUS_IO_PIN_NONE
                            ? String("non assigné")
                            : "GPIO " + String(_waterPin)
                              + (_waterActiveHigh ? " (actif +3,3 V)" : " (actif GND)")));
    if (_strip != nullptr) {
        Serial.println("[STATUS] Bandeau LED WS2812 : GPIO " + String(_ledPin)
                       + " — " + String(_ledCountApplied) + " LEDs, matrice 5 états (NVS statusio)");
    } else {
        Serial.println("[STATUS] Bandeau LED WS2812 : non assigné (inactif)");
    }

    /* ---- 4. Tâche dédiée : échantillonnage des entrées + rendu du bandeau ----
     * Core 0 (Wi-Fi) : le rendu NeoPixel (bit-bang, interruptions coupées)
     * reste hors du Core temps réel. */
    xTaskCreatePinnedToCore(
        _taskEntry, "StatusIO", STACK_SIZE_STATUSIO,
        this, PRIORITY_STATUSIO, nullptr, CORE_WIFI
    );
}

/* =========================================================================
 * ACCÈS THREAD-SAFE (SERVEUR WEB / TÉLÉMÉTRIE)
 * ========================================================================= */

void StatusIO::getConfig(StatusIOConfig& out) {
    /* Pré-remplir hors verrou : si la prise du mutex échouait (théorique,
     * il n'est tenu que quelques µs), un appelant Web écrirait des zéros
     * au POST suivant. Une copie potentiellement antérieure reste sûre. */
    out = _cfg;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        out = _cfg;
        xSemaphoreGive(_mutex);
    }
}

bool StatusIO::setConfig(const StatusIOConfig& cfg) {
    /* Bornage AVANT toute écriture (garde-fou identique au chargement NVS). */
    StatusIOConfig c = cfg;
    if (c.armPin   < -1 || c.armPin   > 48
        || (c.armPin   >= 0 && isMemoryBusPin((uint8_t)c.armPin)))   c.armPin   = -1;
    if (c.waterPin < -1 || c.waterPin > 48
        || (c.waterPin >= 0 && isMemoryBusPin((uint8_t)c.waterPin))) c.waterPin = -1;
    if (c.ledPin   < -1 || c.ledPin   > 48
        || (c.ledPin   >= 0 && isMemoryBusPin((uint8_t)c.ledPin)))   c.ledPin   = -1;
    c.armActiveHigh   = (c.armActiveHigh   != 0) ? 1 : 0;
    c.waterActiveHigh = (c.waterActiveHigh != 0) ? 1 : 0;
    if (c.ledCount < LED_WS2812_NUM_MIN) c.ledCount = LED_WS2812_NUM_MIN;
    if (c.ledCount > LED_WS2812_NUM_MAX) c.ledCount = LED_WS2812_NUM_MAX;
    for (uint8_t i = 0; i < LED_STATE_COUNT; i++) {
        if (c.led[i].effect > LED_EFFECT_OFF) c.led[i].effect = LED_EFFECT_SOLID;
        if (c.led[i].brightness < LED_WS2812_BRI_MIN) c.led[i].brightness = LED_WS2812_BRI_MIN;
        if (c.led[i].brightness > LED_WS2812_BRI_MAX) c.led[i].brightness = LED_WS2812_BRI_MAX;
    }

    /* NVS : écriture vérifiée par relecture (convention du projet). */
    Preferences prefs;
    if (!prefs.begin("statusio", false)) return false;
    prefs.putUChar("in_arm",     (uint8_t)(c.armPin   < 0 ? STATUS_IO_PIN_NONE : c.armPin));
    prefs.putUChar("in_water",   (uint8_t)(c.waterPin < 0 ? STATUS_IO_PIN_NONE : c.waterPin));
    prefs.putUChar("in_arm_hi",   c.armActiveHigh);
    prefs.putUChar("in_water_hi", c.waterActiveHigh);
    prefs.putUChar("led_pin",  (uint8_t)(c.ledPin   < 0 ? STATUS_IO_PIN_NONE : c.ledPin));
    prefs.putUChar("led_count", c.ledCount);
    prefs.putBytes("led_cfg", c.led, sizeof(c.led));
    prefs.remove("led_bri");   /* clé du format v1, remplacée par la luminosité par état */

    bool ok = true;
    ok = ok && (prefs.getUChar("in_arm", 0xFE)
                == (uint8_t)(c.armPin   < 0 ? STATUS_IO_PIN_NONE : c.armPin));
    ok = ok && (prefs.getUChar("in_water", 0xFE)
                == (uint8_t)(c.waterPin < 0 ? STATUS_IO_PIN_NONE : c.waterPin));
    ok = ok && (prefs.getUChar("in_arm_hi", 0xFE)   == c.armActiveHigh);
    ok = ok && (prefs.getUChar("in_water_hi", 0xFE) == c.waterActiveHigh);
    ok = ok && (prefs.getUChar("led_pin", 0xFE)
                == (uint8_t)(c.ledPin   < 0 ? STATUS_IO_PIN_NONE : c.ledPin));
    ok = ok && (prefs.getUChar("led_count", 0xFE) == c.ledCount);
    {
        LedStateConfig rd[LED_STATE_COUNT];
        ok = ok && (prefs.getBytes("led_cfg", rd, sizeof(rd)) == sizeof(rd))
                && (memcmp(rd, c.led, sizeof(rd)) == 0);
    }
    prefs.end();
    if (!ok) {
        Serial.println("[STATUS] Écriture NVS impossible (configuration non enregistrée)");
        return false;
    }

    /* Application à chaud de la matrice (broches et polarités : au boot) */
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cfg = c;
        xSemaphoreGive(_mutex);
    }
    return true;
}

bool StatusIO::isArmClosed() {
    return _armStable;
}

bool StatusIO::isWaterAlarm() {
    return _waterStable;
}

int8_t StatusIO::getArmPin() const {
    return (_armPin == STATUS_IO_PIN_NONE) ? -1 : (int8_t)_armPin;
}

int8_t StatusIO::getWaterPin() const {
    return (_waterPin == STATUS_IO_PIN_NONE) ? -1 : (int8_t)_waterPin;
}

bool StatusIO::getArmActiveHigh() const {
    return _armActiveHigh;
}

bool StatusIO::getWaterActiveHigh() const {
    return _waterActiveHigh;
}

int8_t StatusIO::getLedPin() const {
    return (_ledPin == STATUS_IO_PIN_NONE) ? -1 : (int8_t)_ledPin;
}

uint8_t StatusIO::getLedCount() const {
    return _ledCountApplied;
}

uint8_t StatusIO::getActiveLedState() const {
    return _activeLedState;
}

/* =========================================================================
 * TÂCHE FREERTOS : échantillonnage + rendu
 * ========================================================================= */

void StatusIO::_taskEntry(void* param) {
    static_cast<StatusIO*>(param)->_taskLoop();
}

void StatusIO::_taskLoop() {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        _sampleInputs();
        _updateLed(millis());
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_STATUSIO_PERIOD_MS));
    }
}

/* =========================================================================
 * ENTRÉES NUMÉRIQUES (anti-rebond ~75 ms)
 * ========================================================================= */

/**
 * @brief Échantillonne les 2 entrées et valide un changement d'état après
 *        STATUS_IO_DEBOUNCE_SAMPLES échantillons identiques consécutifs.
 *
 * Les états validés (_armStable / _waterStable) sont publiés en volatile et
 * relus par la tâche de contrôle : la voie d'eau y déclenche l'écrasement du
 * mode de vol en Retour Surface.
 */
void StatusIO::_sampleInputs() {
    /* -- Armement / Allumage (interrupteur magnétique) -- */
    if (_armPin != STATUS_IO_PIN_NONE) {
        const bool raw = _inputActive(_armPin, _armActiveHigh);
        if (raw != _armStable) {
            if (++_armDebounce >= STATUS_IO_DEBOUNCE_SAMPLES) {
                _armStable = raw;
                _armDebounce = 0;
                Serial.println(raw
                    ? "[STATUS] Armement : FERMÉ (entrée active)"
                    : "[STATUS] Armement : OUVERT");
            }
        } else {
            _armDebounce = 0;
        }
    }

    /* -- Auxiliaire / Détecteur Voie d'eau -- */
    if (_waterPin != STATUS_IO_PIN_NONE) {
        const bool raw = _inputActive(_waterPin, _waterActiveHigh);
        if (raw != _waterStable) {
            if (++_waterDebounce >= STATUS_IO_DEBOUNCE_SAMPLES) {
                _waterStable = raw;
                _waterDebounce = 0;
                Serial.println(raw
                    ? "[STATUS] ALARME voie d'eau — Retour Surface forcé !"
                    : "[STATUS] Détecteur voie d'eau : SEC (retour à la normale)");
            }
        } else {
            _waterDebounce = 0;
        }
    }
}

/* =========================================================================
 * BANDEAU LED D'ÉTAT (rendu avec détection de changement)
 * ========================================================================= */

/**
 * @brief Détermine l'état à signaler, applique l'effet configuré et ne
 *        rafraîchit le bandeau que si une couleur doit réellement changer.
 *
 * Priorité d'affichage (premier actif) :
 *   Avarie > Retour Surface > Autopilote actif > RPi5 Connecté > Attente.
 */
void StatusIO::_updateLed(unsigned long now) {
    if (_strip == nullptr) {
        _activeLedState = 255;
        return;
    }

    /* -- Copie de la matrice (modifiable à chaud depuis le Web) -- */
    LedStateConfig ledCfg[LED_STATE_COUNT];
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(ledCfg, _cfg.led, sizeof(ledCfg));
        xSemaphoreGive(_mutex);
    } else {
        return;   /* config momentanément indisponible : conserver l'affichage */
    }

    /* -- État à signaler (priorité décroissante) -- */
    uint8_t st = LED_ST_NET_WAIT;
    if (g_motors.isFaultActive() || g_motors.getIsolatedMask() != 0
        || g_sensors.isBNO085Stalled()) {
        st = LED_ST_FAULT;
    } else if (g_flightCtrl.isSurfaceReturnActive()) {
        st = LED_ST_SURFACE_RETURN;
    } else if ((g_flightCtrl.getActiveMode() & (MODE_BIT_RT | MODE_BIT_DEPTH | MODE_BIT_CAP)) != 0) {
        st = LED_ST_AUTOPILOT;
    } else if (g_serial.getLastFrameTime() > 0 && !g_serial.isWatchdogTriggered()) {
        st = LED_ST_RPI5_CONNECTED;
    }
    _activeLedState = st;

    /* -- Phase de l'effet configuré pour cet état -- */
    const LedEffect fx = (LedEffect)ledCfg[st].effect;
    bool on = true;
    uint8_t scale = 255;
    if (fx == LED_EFFECT_OFF) {
        on = false;
    } else if (fx == LED_EFFECT_PULSE) {
        const float phase = (float)(now % LED_PULSE_PERIOD_MS) / (float)LED_PULSE_PERIOD_MS;
        scale = (uint8_t)((sinf(phase * 2.0f * PI) * 0.5f + 0.5f) * 255.0f);
    } else if (fx == LED_EFFECT_STROBE) {
        on = (now % LED_STROBE_PERIOD_MS) < LED_STROBE_ON_MS;
    }

    /* Luminosité de l'état (1-100 %) appliquée après l'effet — économie d'énergie */
    const uint8_t r = on ? (uint8_t)((uint16_t)ledCfg[st].r * scale / 255u * ledCfg[st].brightness / 100u) : 0;
    const uint8_t g = on ? (uint8_t)((uint16_t)ledCfg[st].g * scale / 255u * ledCfg[st].brightness / 100u) : 0;
    const uint8_t b = on ? (uint8_t)((uint16_t)ledCfg[st].b * scale / 255u * ledCfg[st].brightness / 100u) : 0;

    /* -- Détection de changement : aucun show() inutile -- */
    if (st == _paintedState && r == _paintedR && g == _paintedG && b == _paintedB) {
        return;
    }
    /* Effet animé sur l'état déjà peint : borner la cadence (coût du bit-bang) */
    if (st == _paintedState && fx == LED_EFFECT_PULSE
        && (now - _lastAnimMs) < LED_ANIM_FRAME_MS) {
        return;
    }

    _fill(r, g, b);
    _strip->show();

    _lastAnimMs   = now;
    _paintedState = st;
    _paintedR     = r;
    _paintedG     = g;
    _paintedB     = b;
}

/**
 * @brief Peint toutes les LEDs avec la couleur demandée (sans show()).
 */
void StatusIO::_fill(uint8_t r, uint8_t g, uint8_t b) {
    for (uint16_t i = 0; i < _ledCountApplied; i++) {
        _strip->setPixelColor(i, r, g, b);
    }
}
