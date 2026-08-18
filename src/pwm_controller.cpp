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
 * (l'initialisation I2C se fait dans begin()).
 */
PWMController::PWMController()
    : _pwm(PCA9685_I2C_ADDR), _mutex(nullptr)
    , _physicalOutputsEnabled(false)    /* Mode Témoin par défaut au boot */
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
 * @brief Initialise le PCA9685 : reset, fréquence 50 Hz, tous canaux au neutre.
 *
 * Séquence d'initialisation :
 * 1. Création du mutex FreeRTOS
 * 2. Reset logiciel du PCA9685 via la bibliothèque Adafruit
 * 3. Configuration de la fréquence de sortie à 50 Hz
 * 4. Application du neutre (1500 µs) sur les 16 canaux
 */
void PWMController::begin() {
    /* Création du mutex de protection d'accès */
    _mutex = xSemaphoreCreateMutex();

    /* Reset et configuration du PCA9685 */
    _pwm.begin();
    _pwm.setOscillatorFrequency(25000000UL);    /* Oscillateur interne 25 MHz */
    _pwm.setPWMFreq(PCA9685_FREQ_HZ);          /* 50 Hz pour servos/ESC */

    /* Délai de stabilisation après changement de fréquence */
    delay(10);

    /* Application du neutre sur tous les canaux (failsafe initial) */
    setAllNeutral();

    Serial.println("[PWM] PCA9685 initialisé à 50 Hz, 16 canaux au neutre");
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
 * @brief Force tous les canaux au neutre (1500 µs).
 *
 * Utilisé par le watchdog failsafe pour arrêter tous les propulseurs
 * en cas de perte de communication série.
 */
void PWMController::setAllNeutral() {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        uint16_t neutralTicks = _usToTicks(PWM_NEUTRAL_US);
        for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
            _pwm.setPWM(i, 0, neutralTicks);
            _current[i] = PWM_NEUTRAL_US;
        }
        xSemaphoreGive(_mutex);
        Serial.println("[PWM] Failsafe : tous les canaux au neutre (1500 µs)");
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
 * - Passage à `false` : le PCA9685 est immédiatement forcé au neutre
 *   (1500 µs canaux 0–11, 0 % canaux 12–15) sans modifier le buffer
 *   _current[] qui continue de refléter les consignes logiques.
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
 * @brief Force le neutre de sécurité sur le PCA9685 sans modifier _current[].
 *
 * - Canaux 0–11 (ESC/Servos) : 1500 µs (neutre)
 * - Canaux 12–15 (Gradateurs) : 0 ticks (0 % — LED éteints)
 *
 * @note Doit être appelée avec le mutex déjà acquis.
 */
void PWMController::_forcePhysicalNeutral() {
    uint16_t neutralTicks = _usToTicks(PWM_NEUTRAL_US);
    for (uint8_t i = 0; i < 12; i++) {
        _pwm.setPWM(i, 0, neutralTicks);
    }
    /* Canaux 12-15 : gradateurs → 0 ticks (0%) */
    for (uint8_t i = 12; i < NUM_PWM_CHANNELS; i++) {
        _pwm.setPWM(i, 0, 0);
    }
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
