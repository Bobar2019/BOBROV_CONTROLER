/**
 * @file pwm_controller.cpp
 * @brief Implémentation du pilote PCA9685 pour 16 canaux PWM avec mode Témoin.
 *
 * Gère la conversion microsecondes → ticks 12 bits et l'application
 * sécurisée des valeurs PWM avec gestion du mode failsafe et du mode
 * Témoin (Dry-Run) qui neutralise les sorties physiques tout en
 * conservant le buffer de télémétrie pour affichage WebSocket.
 *
 * @author Didier Dero
 * @version 1.1.0
 */

#include "pwm_controller.h"
#include "config.h"
#include <Wire.h>
#include <Preferences.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Instance globale du contrôleur PWM */
PWMController g_pwm;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

/**
 * @brief Constructeur : initialise les valeurs courantes au neutre.
 *
 * Le mutex est créé mais le PCA9685 n'est pas encore initialisé
 * (l'initialisation I2C se fait dans begin()). Le driver est lié au bus
 * I2C n°2 (Wire1, GPIO 6/7), partagé avec le MS5837 de la tâche capteurs.
 */
PWMController::PWMController()
    : _pwm(PCA9685_I2C_ADDR, Wire1)   /* bus I2C n°2 — MS5837 + PCA9685 */
    , _mutex(nullptr)
    , _physicalOutputsEnabled(false)    /* Mode Témoin par défaut au boot */
    , _typeMask(PWM_TYPE_DEFAULT_MASK)  /* typage provisoire avant lecture NVS (begin) */
{
    /* Initialiser tous les canaux au neutre par défaut */
    for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
        _current[i] = PWM_NEUTRAL_US;
    }
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

/**
 * @brief Initialise le PCA9685 : reset, fréquence 50 Hz, position de sécurité.
 *
 * Séquence d'initialisation :
 * 1. Création du mutex FreeRTOS
 * 2. Chargement du typage des canaux 10-15 depuis la NVS
 * 3. Reset logiciel du PCA9685 via la bibliothèque Adafruit
 * 4. Configuration de la fréquence de sortie à 50 Hz
 * 5. Application de la position de sécurité (neutre 1500 µs / 0 %) sur les 16 canaux
 */
void PWMController::begin() {
    /* Création du mutex de protection d'accès */
    _mutex = xSemaphoreCreateMutex();

    /* Chargement du typage des canaux 10-15 (clé NVS « pwm_types ») */
    _loadChannelTypes();

    /* NOTE : le PCA9685 vit sur le bus I2C n°2 (Wire1, GPIO 6/7), partagé
     * avec le MS5837. Le bus est configuré une seule fois par
     * SensorDriver::begin() — exécuté AVANT PWMController::begin() dans
     * main.cpp — de sorte que sa fréquence/timeout ne sont jamais modifiés
     * pendant une transaction ; les accès des deux tâches sont ensuite
     * sérialisés par le verrou interne de Wire1 (une par instance). */

    /* Reset et configuration du PCA9685 */
    if (!_pwm.begin()) {
        Serial.println("[PWM] PCA9685 INTROUVABLE sur le bus I2C n°2 (0x40) — sorties inertes");
        return;
    }
    _pwm.setOscillatorFrequency(25000000UL);    /* Oscillateur interne 25 MHz */
    _pwm.setPWMFreq(PCA9685_FREQ_HZ);          /* 50 Hz pour servos/ESC */

    /* Délai de stabilisation après changement de fréquence */
    delay(10);

    /* Application de la position de sécurité sur tous les canaux (failsafe initial) */
    setAllNeutral();

    Serial.println("[PWM] PCA9685 initialisé à 50 Hz, canaux à la position de sécurité (bus I2C n°2)");
}

/* =========================================================================
 * CONTRÔLE PWM
 * ========================================================================= */

/**
 * @brief Applique une consigne PWM sur un canal unique.
 *
 * La valeur en microsecondes est bornée entre PWM_MIN_US (1000) et
 * PWM_MAX_US (2000) avant conversion en ticks PCA9685.
 *
 * **Mode Témoin** : la valeur est mémorisée dans _current[] (pour la
 * télémétrie WebSocket/trame montante) mais le PCA9685 n'est pas écrit.
 *
 * @param channel      Numéro du canal (0-15).
 * @param microseconds Impulsion souhaitée en µs (1000-2000).
 */
void PWMController::setPWMuS(uint8_t channel, uint16_t microseconds) {
    if (channel >= NUM_PWM_CHANNELS) return;

    /* Bornage de la valeur dans la plage valide */
    if (microseconds < PWM_MIN_US) microseconds = PWM_MIN_US;
    if (microseconds > PWM_MAX_US) microseconds = PWM_MAX_US;

    /* Mise à jour du buffer sous mutex (toujours, même en Dry-Run) */
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _current[channel] = microseconds;

        /* Écriture physique uniquement si les sorties sont activées */
        if (_physicalOutputsEnabled) {
            uint16_t ticks = _usToTicks(microseconds);
            _pwm.setPWM(channel, 0, ticks);
        }
        xSemaphoreGive(_mutex);
    }
}

/**
 * @brief Applique un tableau complet de 16 valeurs PWM en une seule passe.
 *
 * Cette méthode optimise les mises à jour en verrouillant le mutex une
 * seule fois pour l'ensemble des canaux.
 *
 * **Mode Témoin** : les valeurs sont mémorisées dans _current[] mais le
 * PCA9685 n'est pas écrit (sorties physiques neutralisées).
 *
 * @param values Tableau de 16 valeurs en µs (PWM_MIN_US à PWM_MAX_US).
 */
void PWMController::setAllPWM(const uint16_t* values) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
            uint16_t us = values[i];
            /* Bornage individuel de chaque canal */
            if (us < PWM_MIN_US) us = PWM_MIN_US;
            if (us > PWM_MAX_US) us = PWM_MAX_US;
            _current[i] = us;

            /* Écriture physique uniquement si les sorties sont activées */
            if (_physicalOutputsEnabled) {
                uint16_t ticks = _usToTicks(us);
                _pwm.setPWM(i, 0, ticks);
            }
        }
        xSemaphoreGive(_mutex);
    }
}

/**
 * @brief Force les canaux à leur position de sécurité (failsafe).
 *
 * - Canaux bidirectionnels (ESC/servos/outils, 0–9 et 10–15 typés bidir) :
 *   neutre 1500 µs.
 * - Canaux unidirectionnels (gradateurs LED, 10–15 typés unidir) : 0 %
 *   (1000 µs).
 *
 * Les valeurs sont écrites PHYSIQUEMENT sur le PCA9685 — y compris en mode
 * Témoin, la position de sécurité étant le seul état sûr — et mémorisées
 * dans _current[] pour la télémétrie (WebSocket / trame montante).
 */
void PWMController::setAllNeutral() {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
            const uint16_t us = _safeUsForChannel(i);
            _pwm.setPWM(i, 0, _usToTicks(us));
            _current[i] = us;
        }
        xSemaphoreGive(_mutex);
        Serial.println("[PWM] Failsafe : canaux à la position de sécurité (neutre 1500 µs / 0 %)");
    }
}

/**
 * @brief Retourne la valeur PWM courante d'un canal en µs.
 *
 * @param channel Numéro du canal (0-15).
 * @return Valeur en µs, ou PWM_NEUTRAL_US si le canal est invalide.
 */
uint16_t PWMController::getPWMuS(uint8_t channel) const {
    if (channel >= NUM_PWM_CHANNELS) return PWM_NEUTRAL_US;
    return _current[channel];
}

/**
 * @brief Copie les 16 valeurs PWM courantes dans un tableau de sortie.
 *
 * Accès thread-safe via mutex pour une lecture cohérente de l'ensemble.
 *
 * @param out Tableau de sortie de taille >= 16.
 */
void PWMController::getCurrentValues(uint16_t* out) const {
    if (!out) return;
    /* Lecture directe acceptable car uint16_t est atomique sur ESP32 */
    for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
        out[i] = _current[i];
    }
}

/* =========================================================================
 * MODE TÉMOIN (DRY-RUN) — SORTIES PHYSIQUES
 * ========================================================================= */

/**
 * @brief Active ou désactive les sorties physiques du PCA9685.
 *
 * - Passage à `true`  : les prochaines consignes seront écrites sur le PCA9685.
 *   Les valeurs actuelles du buffer sont immédiatement appliquées.
 * - Passage à `false` : le PCA9685 est immédiatement forcé à la position de
 *   sécurité (neutre 1500 µs pour les canaux bidirectionnels, 0 % pour les
 *   canaux unidirectionnels) sans modifier le buffer _current[] qui continue
 *   de refléter les consignes logiques.
 *
 * @param enabled État souhaité des sorties physiques.
 */
void PWMController::setPhysicalOutputsEnabled(bool enabled) {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        bool wasEnabled = _physicalOutputsEnabled;
        _physicalOutputsEnabled = enabled;

        if (enabled && !wasEnabled) {
            /* Passage Dry-Run → Actif : appliquer les valeurs du buffer */
            for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
                uint16_t ticks = _usToTicks(_current[i]);
                _pwm.setPWM(i, 0, ticks);
            }
            Serial.println("[PWM] Sorties physiques ACTIVÉES (propulsion active)");
        } else if (!enabled && wasEnabled) {
            /* Passage Actif → Dry-Run : forcer le neutre sur le PCA9685 */
            _forcePhysicalNeutral();
            Serial.println("[PWM] Sorties physiques NEUTRALISÉES (Mode Témoin / Dry-Run)");
        }
        xSemaphoreGive(_mutex);
    }
}

/**
 * @brief Retourne true si les sorties physiques sont actives.
 */
bool PWMController::isPhysicalOutputsEnabled() const {
    return _physicalOutputsEnabled;
}

/**
 * @brief Force la position de sécurité sur le PCA9685 sans modifier _current[].
 *
 * - Canaux bidirectionnels : 1500 µs (neutre).
 * - Canaux unidirectionnels (gradateurs) : 1000 µs (0 % — LED éteints).
 *
 * @note Doit être appelée avec le mutex déjà acquis.
 */
void PWMController::_forcePhysicalNeutral() {
    for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
        _pwm.setPWM(i, 0, _usToTicks(_safeUsForChannel(i)));
    }
}

/* =========================================================================
 * TYPAGE DES CANAUX 10–15 (BIDIRECTIONNEL / UNIDIRECTIONNEL)
 * ========================================================================= */

/**
 * @brief Charge le masque de typage depuis la NVS (clé « pwm_types »).
 *
 * Valeur absente → PWM_TYPE_DEFAULT_MASK (CH10/CH11 pinces et CH14/CH15
 * auxiliaires bidirectionnels, CH12/CH13 gradateurs unidirectionnels).
 */
void PWMController::_loadChannelTypes() {
    Preferences prefs;
    prefs.begin("config", true);   /* Lecture seule */
    uint8_t mask = prefs.getUChar("pwm_types", PWM_TYPE_DEFAULT_MASK);
    prefs.end();

    _typeMask = (uint8_t)(mask & 0x3F);

    String types;
    for (uint8_t i = 0; i < PWM_TYPE_NUM_CHANNELS; i++) {
        types += " CH" + String(PWM_TYPE_FIRST_CHANNEL + i) + "=";
        types += ((_typeMask >> i) & 1) ? "BIDIR" : "UNI";
    }
    Serial.println("[PWM] Typage canaux 10-15 :" + types);
}

/**
 * @brief Retourne le masque de typage courant (bit n ↔ canal 10+n, 1 = bidir).
 */
uint8_t PWMController::getChannelTypeMask() const {
    return _typeMask;
}

/**
 * @brief Indique si un canal est bidirectionnel (position de sécurité 1500 µs).
 *
 * Les canaux 0–9 (ESC/servos) sont toujours bidirectionnels ; les canaux
 * 10–15 suivent le masque persisté en NVS.
 */
bool PWMController::isBidirectional(uint8_t channel) const {
    if (channel < PWM_TYPE_FIRST_CHANNEL) return true;
    if (channel >= PWM_TYPE_FIRST_CHANNEL + PWM_TYPE_NUM_CHANNELS) return false;
    return ((_typeMask >> (channel - PWM_TYPE_FIRST_CHANNEL)) & 1) != 0;
}

/**
 * @brief Persiste un nouveau masque de typage en NVS (vérifié par relecture)
 *        puis l'applique à chaud — aucun redémarrage nécessaire.
 */
bool PWMController::saveChannelTypes(uint8_t mask) {
    mask &= 0x3F;

    Preferences prefs;
    if (!prefs.begin("config", false)) {
        return false;
    }
    prefs.putUChar("pwm_types", mask);
    const bool ok = (prefs.getUChar("pwm_types", 0xFF) == mask);   /* relecture */
    prefs.end();

    if (ok) {
        _typeMask = mask;
        Serial.printf("[PWM] Typage canaux 10-15 enregistré (NVS) : masque 0x%02X\n", mask);
    }
    return ok;
}

/**
 * @brief Valeur de sécurité d'un canal selon son typage.
 *
 * - Bidirectionnel : PWM_NEUTRAL_US (1500 µs — neutre ESC/servo/outil).
 * - Unidirectionnel : PWM_MIN_US (1000 µs — 0 % gradateur LED).
 */
uint16_t PWMController::_safeUsForChannel(uint8_t channel) const {
    return isBidirectional(channel) ? PWM_NEUTRAL_US : PWM_MIN_US;
}

/* =========================================================================
 * CONVERSION INTERNE
 * ========================================================================= */

/**
 * @brief Convertit des microsecondes en ticks PCA9685 (résolution 12 bits).
 *
 * Le PCA9685 utilise un compteur 12 bits (0-4095) sur une période complète.
 * À 50 Hz, la période est de 20 000 µs, donc :
 *   ticks = µs × 4096 / 20000 = µs × 0.2048
 *
 * @param us Valeur en microsecondes (déjà bornée).
 * @return Ticks PCA9685 (0-4095).
 */
uint16_t PWMController::_usToTicks(uint16_t us) const {
    /*
     * Calcul en entier pour éviter la perte de précision flottante :
     * ticks = us * 4096 / 20000 = us * 256 / 1250
     * Utilise un intermédiaire uint32_t pour éviter le débordement.
     */
    return (uint16_t)((uint32_t)us * PCA9685_RESOLUTION / (1000000UL / PCA9685_FREQ_HZ));
}
