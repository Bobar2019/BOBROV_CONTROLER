/**
 * @file gpio_outputs.cpp
 * @brief Implémentation des 4 sorties tout-ou-rien (GPIO ON/OFF).
 *
 * Chargement des broches depuis la NVS (namespace « config », clés
 * « gpio_p1 »..« gpio_p4 »), initialisation en sortie à l'état BAS (OFF) au
 * démarrage, application du masque gpio_cmd reçu du RPi 5 avec relecture de
 * l'état réel, et forçage à OFF en sécurité (watchdog série / E-Stop).
 *
 * @author Didier Dero
 * @version 1.0.0
 */

#include "gpio_outputs.h"
#include <Preferences.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Instance globale des sorties tout-ou-rien */
GPIOOutputs g_gpio;

/* =========================================================================
 * CLÉS NVS
 * ========================================================================= */

/** @brief Clés NVS des broches (namespace « config ») — ordre = sorties 1 à 4 */
static const char* const GPIO_PIN_KEYS[GPIO_NUM_OUTPUTS] = { "gpio_p1", "gpio_p2", "gpio_p3", "gpio_p4" };

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

/**
 * @brief Constructeur : broches par défaut, état OFF, section critique initialisée.
 */
GPIOOutputs::GPIOOutputs()
    : _state(0)
    , _mux(portMUX_INITIALIZER_UNLOCKED)
{
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        _pins[i] = GPIO_OUTPUT_PINS_DEFAULT[i];
    }
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

/**
 * @brief Charge les broches depuis la NVS puis les initialise en sortie à OFF.
 *
 * Une valeur NVS hors plage (0-48) est remplacée par la broche par défaut —
 * protège contre une NVS corrompue ou un GPIO inexistant sur ESP32-S3.
 */
void GPIOOutputs::begin() {
    /* ---- 1. Chargement des broches assignées (NVS) ---- */
    Preferences prefs;
    prefs.begin("config", true);   /* Lecture seule */
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        uint8_t pin = prefs.getUChar(GPIO_PIN_KEYS[i], GPIO_OUTPUT_PINS_DEFAULT[i]);
        if (pin > 48) {
            pin = GPIO_OUTPUT_PINS_DEFAULT[i];   /* valeur corrompue → défaut */
        }
        _pins[i] = pin;
    }
    prefs.end();

    /* ---- 2. Initialisation en sortie à l'état BAS (OFF) ---- */
    portENTER_CRITICAL(&_mux);
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        pinMode(_pins[i], OUTPUT);
        digitalWrite(_pins[i], LOW);
    }
    _state = _readMask();   /* doit valoir 0 */
    portEXIT_CRITICAL(&_mux);

    Serial.printf("[GPIO] Sorties ON/OFF initialisées à OFF : GPIO %u, %u, %u, %u\n",
                  _pins[0], _pins[1], _pins[2], _pins[3]);
}

/* =========================================================================
 * PILOTAGE
 * ========================================================================= */

/**
 * @brief Applique le masque de commande (bit 0 = sortie 1) et relit l'état réel.
 *
 * L'écriture est ignorée si le masque demandé est identique à l'état courant
 * (cas fréquent : trame descendante répétée à 50 Hz).
 */
void GPIOOutputs::applyCommand(uint8_t mask) {
    mask &= 0x0F;
    portENTER_CRITICAL(&_mux);
    if (mask != _state) {
        _writeMask(mask);
        _state = _readMask();
    }
    portEXIT_CRITICAL(&_mux);
}

/**
 * @brief Force les 4 sorties à OFF (failsafe watchdog série / E-Stop).
 *
 * N'émet le journal qu'à la première transition (évite 50 lignes/s tant que
 * l'E-Stop est maintenu par le RPi 5).
 */
void GPIOOutputs::allOff() {
    portENTER_CRITICAL(&_mux);
    const uint8_t prev = _state;
    _writeMask(0);
    _state = _readMask();
    portEXIT_CRITICAL(&_mux);

    if (prev != 0) {
        Serial.println("[GPIO] Failsafe : sorties ON/OFF forcées à OFF");
    }
}

/**
 * @brief Retourne l'état réel courant sous forme de masque 4 bits.
 */
uint8_t GPIOOutputs::getState() const {
    return _state & 0x0F;
}

/**
 * @brief Copie les 4 broches actuellement assignées dans un tableau.
 */
void GPIOOutputs::getPins(uint8_t* out) const {
    if (!out) return;
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        out[i] = _pins[i];
    }
}

/* =========================================================================
 * PERSISTANCE NVS
 * ========================================================================= */

/**
 * @brief Sauvegarde les broches en NVS avec vérification par relecture.
 *
 * Réplique la stratégie des broches I2C (saveBusPins) : écriture puis
 * relecture comparée — une NVS pleine ou en échec est ainsi détectée avant
 * d'annoncer le redémarrage à l'utilisateur.
 */
bool GPIOOutputs::savePins(const uint8_t* pins) {
    if (!pins) return false;

    Preferences prefs;
    if (!prefs.begin("config", false)) {
        return false;
    }
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        prefs.putUChar(GPIO_PIN_KEYS[i], pins[i]);
    }
    bool ok = true;
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        if (prefs.getUChar(GPIO_PIN_KEYS[i], 0xFF) != pins[i]) {
            ok = false;
        }
    }
    prefs.end();
    return ok;
}

/* =========================================================================
 * INTERNES
 * ========================================================================= */

/**
 * @brief Écrit un masque sur les 4 broches (sans relecture).
 * @note Doit être appelée dans la section critique.
 */
void GPIOOutputs::_writeMask(uint8_t mask) {
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        digitalWrite(_pins[i], (mask >> i) & 1);
    }
}

/**
 * @brief Relit le niveau réel des 4 broches (registre d'entrée du pad).
 * @note Doit être appelée dans la section critique.
 */
uint8_t GPIOOutputs::_readMask() const {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        if (digitalRead(_pins[i]) == HIGH) {
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}
