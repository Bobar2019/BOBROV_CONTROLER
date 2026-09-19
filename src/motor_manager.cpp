/**
 * @file motor_manager.cpp
 * @brief Implémentation de la gestion de sécurité moteurs (v1.1.0).
 *
 * Check Propulseurs au démarrage (après connexion Wi-Fi non bloquante),
 * surveillance continue des courants ACS770 (INA3221 canaux 2/3),
 * identification + isolement logique du moteur en surintensité, et remontée
 * d'urgence optionnelle avec les verticaux restants.
 *
 * Discipline de sûreté :
 *  - AUCUNE écriture moteur hors des garde-fous existants : les impulsions du
 *    check utilisent g_pwm.setPWMuS() (bornage + buffer + Dry-Run), le retour
 *    au neutre passe par setPWMuS(channel, PWM_NEUTRAL_US) canal par canal —
 *    jamais de setAllNeutral() qui écraserait les servos 8-15 ;
 *  - le watchdog série ne force le neutre qu'UNE fois, à la transition de
 *    timeout, et seulement si une trame RPi 5 a déjà été reçue : au boot
 *    (aucune trame), il ne perturbe jamais la séquence de check ;
 *  - les lectures I2C du check passent par SensorDriver::readACS770CurrentmA()
 *    qui prend le verrou _bus1Lock partagé avec la tâche capteurs ;
 *  - Dry-Run : les impulsions restent dans le buffer (aucun mouvement
 *    physique) mais la logique de mesure s'exécute quand même — utile pour
 *    valider la chaîne de mesure sur établi.
 *
 * @author Didier Dero
 * @version 1.1.0
 */

#include "motor_manager.h"
#include "sensor_driver.h"
#include "pwm_controller.h"
#include "web_server.h"

#include <Preferences.h>
#include <math.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Instance globale du gestionnaire de sécurité moteurs */
MotorManager g_motors;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

MotorManager::MotorManager()
    : _health(MOTOR_HEALTH_UNKNOWN)
    , _checkDone(false)
    , _checkPassed(false)
    , _checkRequested(false)
    , _resetRequested(false)
    , _autoArmed(false)
    , _sequenceRunning(false)
    , _faultActive(false)
    , _emergencyActive(false)
    , _isolatedMask(0)
    , _overcurrentStreakH(0)
    , _overcurrentStreakV(0)
    , _mutex(nullptr)
{
    _cfg.checkEnable       = true;                          /* défauts ON */
    _cfg.detectEnable      = true;
    _cfg.emergencyEnable   = false;
    _cfg.currentThresholdA = MOTOR_CURRENT_THRESH_DEFAULT_A;
    _cfg.vbatLowV          = MOTOR_VBAT_LOW_DEFAULT_V;
    _lastStatus[0]         = '\0';
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

void MotorManager::begin() {
    _mutex = xSemaphoreCreateMutex();

    /* Configuration NVS (namespace « motors ») — chaque lecture retombe sur
     * les défauts config.h si la clé est absente (première utilisation). */
    Preferences prefs;
    prefs.begin("motors", true);
    _cfg.checkEnable       = prefs.getBool("check_enable", true);
    _cfg.detectEnable      = prefs.getBool("detect_enable", true);
    _cfg.emergencyEnable   = prefs.getBool("emergency_enable", false);
    _cfg.currentThresholdA = prefs.getFloat("cur_thresh", MOTOR_CURRENT_THRESH_DEFAULT_A);
    _cfg.vbatLowV          = prefs.getFloat("vbat_low", MOTOR_VBAT_LOW_DEFAULT_V);
    prefs.end();

    /* Bornage défensif : une NVS corrompue ne doit jamais produire un seuil
     * absurde (0 A déclencherait en permanence, 100 A jamais). */
    if (_cfg.currentThresholdA < MOTOR_CURRENT_THRESH_MIN_A) _cfg.currentThresholdA = MOTOR_CURRENT_THRESH_MIN_A;
    if (_cfg.currentThresholdA > MOTOR_CURRENT_THRESH_MAX_A) _cfg.currentThresholdA = MOTOR_CURRENT_THRESH_MAX_A;
    if (_cfg.vbatLowV < MOTOR_VBAT_LOW_MIN_V) _cfg.vbatLowV = MOTOR_VBAT_LOW_MIN_V;
    if (_cfg.vbatLowV > MOTOR_VBAT_LOW_MAX_V) _cfg.vbatLowV = MOTOR_VBAT_LOW_MAX_V;

    /* Armement du check automatique, FIGÉ au boot : activer l'option à chaud
     * depuis l'interface ne déclenche JAMAIS d'impulsions en navigation —
     * seule la relance manuelle (bouton Paramètres) teste à chaud. */
    _autoArmed = _cfg.checkEnable;

    Serial.println("[MOTORS] Gestion moteurs : check="
                   + String(_cfg.checkEnable ? "on" : "off")
                   + ", detection=" + String(_cfg.detectEnable ? "on" : "off")
                   + ", urgence=" + String(_cfg.emergencyEnable ? "on" : "off")
                   + ", seuil I=" + String(_cfg.currentThresholdA, 1) + " A"
                   + ", V_BAT bas=" + String(_cfg.vbatLowV, 1) + " V");

    /* Tâche dédiée sur Core 1 (temps réel, sous la priorité contrôle/PID) :
     * check démarrage + surveillance continue. */
    xTaskCreatePinnedToCore(
        _taskEntry, "Motors", STACK_SIZE_MOTORS,
        this, PRIORITY_MOTORS, nullptr, CORE_RT
    );
}

/* =========================================================================
 * ACCÈS THREAD-SAFE (SERVEUR WEB / TÉLÉMÉTRIE)
 * ========================================================================= */

void MotorManager::getConfig(MotorConfig& out) {
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        out = _cfg;
        xSemaphoreGive(_mutex);
    }
}

bool MotorManager::setConfig(const MotorConfig& cfg) {
    /* Bornage AVANT toute écriture (garde-fou identique au chargement NVS). */
    MotorConfig c = cfg;
    if (c.currentThresholdA < MOTOR_CURRENT_THRESH_MIN_A) c.currentThresholdA = MOTOR_CURRENT_THRESH_MIN_A;
    if (c.currentThresholdA > MOTOR_CURRENT_THRESH_MAX_A) c.currentThresholdA = MOTOR_CURRENT_THRESH_MAX_A;
    if (c.vbatLowV < MOTOR_VBAT_LOW_MIN_V) c.vbatLowV = MOTOR_VBAT_LOW_MIN_V;
    if (c.vbatLowV > MOTOR_VBAT_LOW_MAX_V) c.vbatLowV = MOTOR_VBAT_LOW_MAX_V;

    /* NVS : écriture vérifiée par relecture (convention du projet). */
    Preferences prefs;
    if (!prefs.begin("motors", false)) return false;
    prefs.putBool("check_enable", c.checkEnable);
    prefs.putBool("detect_enable", c.detectEnable);
    prefs.putBool("emergency_enable", c.emergencyEnable);
    prefs.putFloat("cur_thresh", c.currentThresholdA);
    prefs.putFloat("vbat_low", c.vbatLowV);
    const bool ok = (prefs.getFloat("cur_thresh", -1.0f) == c.currentThresholdA) &&
                    (prefs.getFloat("vbat_low", -1.0f) == c.vbatLowV);
    prefs.end();
    if (!ok) {
        Serial.println("[MOTORS] Écriture NVS impossible (configuration non enregistrée)");
        return false;
    }

    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _cfg = c;
        xSemaphoreGive(_mutex);
    }
    Serial.println("[MOTORS] Configuration enregistrée : check="
                   + String(c.checkEnable ? "on" : "off")
                   + ", detection=" + String(c.detectEnable ? "on" : "off")
                   + ", urgence=" + String(c.emergencyEnable ? "on" : "off")
                   + ", seuil I=" + String(c.currentThresholdA, 1) + " A"
                   + ", V_BAT bas=" + String(c.vbatLowV, 1) + " V");
    return true;
}

MotorHealth MotorManager::getStatus() {
    MotorHealth h = MOTOR_HEALTH_UNKNOWN;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        h = _health;
        xSemaphoreGive(_mutex);
    }
    return h;
}

bool MotorManager::isCheckDone() {
    bool v = false;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        v = _checkDone;
        xSemaphoreGive(_mutex);
    }
    return v;
}

bool MotorManager::isCheckPassed() {
    bool v = false;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        v = _checkPassed;
        xSemaphoreGive(_mutex);
    }
    return v;
}

bool MotorManager::isCheckPending() {
    bool v = false;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        v = _autoArmed && !_checkDone;
        xSemaphoreGive(_mutex);
    }
    return v;
}

bool MotorManager::isFaultActive() {
    bool v = false;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        v = _faultActive;
        xSemaphoreGive(_mutex);
    }
    return v;
}

bool MotorManager::isEmergencySurfaceActive() {
    bool v = false;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        v = _emergencyActive;
        xSemaphoreGive(_mutex);
    }
    return v;
}

uint8_t MotorManager::getIsolatedMask() {
    uint8_t m = 0;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        m = _isolatedMask;
        xSemaphoreGive(_mutex);
    }
    return m;
}

bool MotorManager::requestManualCheck() {
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (_sequenceRunning || _faultActive) {
            xSemaphoreGive(_mutex);
            return false;
        }
        _checkRequested = true;
        xSemaphoreGive(_mutex);
        Serial.println("[MOTORS] Check propulseurs manuel demandé (interface Web)");
        return true;
    }
    return false;
}

/**
 * @brief Réarmement « établi » demandé à l'arrêt du simulateur de test.
 *
 * Les défauts observés pendant la simulation (surintensité, isolements de
 * sécurité, check échoué, remontée d'urgence) n'ont plus de cause une fois
 * les valeurs simulées retirées : la tâche Motors applique le réarmement au
 * cycle suivant (voir _applySimulatorReset, hors séquence de sécurité).
 */
void MotorManager::requestSimulatorReset() {
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _resetRequested = true;
        xSemaphoreGive(_mutex);
    }
}

void MotorManager::abortEmergencySurface() {
    bool was = false;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        was = _emergencyActive;
        _emergencyActive = false;
        xSemaphoreGive(_mutex);
    }
    if (was) {
        g_pwm.setEmergencySurface(false);   /* verticaux → neutre (écriture physique inconditionnelle) */
        Serial.println("[MOTORS] Remontée d'urgence INTERROMPUE (arrêt d'urgence humain — "
                       "la surveillance redevient pleinement opérationnelle)");
    }
}

String MotorManager::getLastStatusText() {
    String s;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        s = String(_lastStatus);
        xSemaphoreGive(_mutex);
    }
    return s;
}

/* =========================================================================
 * CHECK PROPULSEURS AU DÉMARRAGE
 * ========================================================================= */

/**
 * @brief Séquence de check : tension batterie PUIS impulsion séquentielle.
 *
 * Retourne true uniquement si la batterie est dans la plage ET les 8
 * impulsions restent sous le seuil. Chaque moteur défaillant est isolé
 * immédiatement (position de sécurité) pour ne pas gêner la suite du test.
 *
 * NOTE Dry-Run : les consignes restent logiques (aucun mouvement), la
 * chaîne de mesure INA3221/ACS770 est quand même exercée — validation
 * complète sur établi sans sortir les hélices.
 */
bool MotorManager::_runStartupCheck() {
    Serial.println("[MOTORS] Check propulseurs : début de séquence");

    /* Remise à zéro du voyant pour CETTE exécution : le verdict final la
     * repositionne (OK/WARNING/FAULT). Un check manuel relancé peut donc
     * reverdir après un WARNING corrigé (batterie rechargée) — en navigation,
     * en revanche, le voyant ne reverdit jamais seul (_setHealth ne monte). */
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _health = MOTOR_HEALTH_UNKNOWN;
        xSemaphoreGive(_mutex);
    }

    /* ---- 1. Tension batterie (canal 1 INA3221, via télémétrie publiée) ---- */
    PowerData pwr;
    g_sensors.getPowerData(pwr);
    const float vbat = (float)pwr.voltage[0] / 1000.0f;
    if (pwr.voltage[0] == 0 || vbat < _cfg.vbatLowV) {
        _setHealth(MOTOR_HEALTH_WARNING);
        _setStatus("Tension batterie basse");
        Serial.println("[MOTORS] Check refusé : V_BAT = " + String(vbat, 2)
                       + " V < seuil " + String(_cfg.vbatLowV, 1) + " V");
        return false;
    }
    Serial.println("[MOTORS] V_BAT = " + String(vbat, 2) + " V — OK, impulsions moteurs");

    /* ---- 2. Impulsion séquentielle M1 → M8 ---- */
    uint8_t failedMask = 0;
    for (uint8_t m = 0; m < MOTOR_NUM_CHANNELS; m++) {
        if (_isolatedMask & (1u << m)) continue;   /* déjà isolé ce boot */

        /* Indice PowerData du courant (config.h) : horizontaux (M1-M4) ou
         * verticaux (M5-M8) — canaux INA3221 2/3 sous-jacents. */
        const uint8_t inaChannel = (m < MOTOR_HORIZ_COUNT)
                                 ? MOTOR_CURRENT_CH_HORIZ : MOTOR_CURRENT_CH_VERT;

        /* Impulsion brève (vérifie un moteur bloqué SANS le laisser chauffer :
         * 400 ms d'impulsion + lecture pendant la fenêtre). */
        g_pwm.setPWMuS(m, MOTOR_CHECK_PULSE_US);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_CHECK_SETTLE_MS));

        /* Fenêtre de mesure : maximum de 3 lectures espacées (la moyenne ×16
         * interne de l'INA3221 lisse déjà, on capture le pic de démarrage). */
        int32_t maxMA = 0;
        for (uint8_t k = 0; k < 3; k++) {
            const int32_t ma = g_sensors.readACS770CurrentmA(inaChannel);
            if (ma > maxMA) maxMA = ma;
            vTaskDelay(pdMS_TO_TICKS(MOTOR_CHECK_PULSE_MS / 4));
        }

        /* Retour immédiat au neutre AVANT toute décision (canal par canal —
         * jamais setAllNeutral(), les servos 8-15 ne sont pas concernés). */
        g_pwm.setPWMuS(m, PWM_NEUTRAL_US);

        if (maxMA < 0) {
            /* Lecture impossible (INA3221 absent/bus occupé) : pas d'impulsion
             * supplémentaire — on ne peut pas conclure, on signale. */
            _setHealth(MOTOR_HEALTH_WARNING);
            _setStatus("Lecture courant impossible");
            Serial.println("[MOTORS] M" + String(m + 1) + " : lecture INA3221 impossible ("
                           + String(maxMA) + ") — séquence interrompue");
            return false;
        }

        const float amps = (float)maxMA / 1000.0f;
        if (maxMA > (int32_t)(_cfg.currentThresholdA * 1000.0f)) {
            failedMask |= (1u << m);
            g_pwm.isolateMotor(m);   /* moteur bloqué → sécurité inconditionnelle */
            Serial.println("[MOTORS] M" + String(m + 1) + " : ÉCHEC — " + String(amps, 2)
                           + " A > seuil " + String(_cfg.currentThresholdA, 1) + " A → isolé");
        } else {
            Serial.println("[MOTORS] M" + String(m + 1) + " : OK — " + String(amps, 2) + " A");
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_CHECK_PAUSE_MS));
    }

    /* Verdict final : tout échec du check a immédiatement isolé son moteur ;
     * un check manuel relancé skip ces canaux — si le moindre isolement
     * subsiste, le check ne peut pas conclure au vert. */
    const uint8_t isolatedNow = g_pwm.getIsolatedMask();
    if (isolatedNow != 0) {
        _setHealth(MOTOR_HEALTH_FAULT);
        _setStatus("Moteur(s) bloqué(s)/isolé(s)");
        Serial.println("[MOTORS] Check TERMINÉ — ÉCHEC (moteurs isolés : masque 0x"
                       + String(isolatedNow, HEX) + ", échecs ce passage 0x"
                       + String(failedMask, HEX) + ")");
        return false;
    }

    _setHealth(MOTOR_HEALTH_OK);
    _setStatus("Check OK");
    Serial.println("[MOTORS] Check TERMINÉ — 8/8 moteurs OK, voyant Statut Moteurs → VERT");
    return true;
}

/* =========================================================================
 * SURVEILLANCE CONTINUE (NAVIGATION)
 * ========================================================================= */

/**
 * @brief Un pas de surveillance (cadence MOTOR_SURVEIL_PERIOD_MS, _taskLoop).
 *
 * Lit les deux courants publiés par la télémétrie (déjà moyennés côté
 * INA3221), exige MOTOR_OVERCURRENT_CONFIRM confirmations consécutives puis
 * déclenche l'identification/isolement — filtrage des transitoires de
 * commande (inversion brutale de poussée) sans jamais rater un blocage.
 */
void MotorManager::_surveyStep() {
    PowerData pwr;
    g_sensors.getPowerData(pwr);
    const int32_t threshMA = (int32_t)(_cfg.currentThresholdA * 1000.0f);

    const int32_t iHorizMA = (int32_t)pwr.current[MOTOR_CURRENT_CH_HORIZ];
    const int32_t iVertMA  = (int32_t)pwr.current[MOTOR_CURRENT_CH_VERT];

    bool overH = (iHorizMA > threshMA);
    bool overV = (iVertMA  > threshMA);

    /* Canal ENTIEREMENT isolé : plus aucun moteur pilotable de ce côté, la
     * mesure résiduelle (bruit/offset ACS770) ne doit rien déclencher. */
    if ((_isolatedMask & 0x0F) == 0x0F) overH = false;
    if ((_isolatedMask & 0xF0) == 0xF0) overV = false;

    /* Remontée d'urgence active : le courant des verticaux reflète la poussée
     * de survie — ré-isoler des verticaux sains sur cette base priverait le
     * ROV de sa remontée. Le canal horizontal reste surveillé. */
    if (_emergencyActive) overV = false;

    _overcurrentStreakH = overH ? (uint8_t)(_overcurrentStreakH + 1) : 0;
    _overcurrentStreakV = overV ? (uint8_t)(_overcurrentStreakV + 1) : 0;

    if (_overcurrentStreakH >= MOTOR_OVERCURRENT_CONFIRM) {
        _overcurrentStreakH = 0;
        _identifyAndIsolate(false);
    } else if (_overcurrentStreakV >= MOTOR_OVERCURRENT_CONFIRM) {
        _overcurrentStreakV = 0;
        _identifyAndIsolate(true);
    }
}

/**
 * @brief Identifie le moteur en surintensité puis l'isole logiquement.
 *
 * Procédure : isole successivement les moteurs actifs du canal fautive (les
 * plus sollicités d'abord — |µs − 1500| décroissant), ré-isole le courant à
 * chaque pas : le moteur dont l'isolement fait retomber le courant SOUS le
 * seuil est le coupable → il reste isolé, les autres sont relâchés au neutre.
 *
 * Si l'identification échoue (courant toujours élevé après tout isoler —
 * défaut en amont : ESC, câblage, batterie), les moteurs du canal restent
 * isolés par prudence et le statut passe en défaut.
 *
 * @param vertical false = canal 2 (horizontaux M1-M4), true = canal 3 (verticaux M5-M8).
 */
void MotorManager::_identifyAndIsolate(bool vertical) {
    Serial.println("[MOTORS] SURINTENSITÉ confirmée — canal "
                   + String(vertical ? "verticaux (3)" : "horizontaux (2)")
                   + " : identification du moteur incriminé…");
    _sequenceRunning = true;

    const uint8_t first = vertical ? MOTOR_VERT_FIRST : 0;
    const uint8_t count = vertical ? MOTOR_VERT_COUNT : MOTOR_HORIZ_COUNT;
    const uint8_t inaChannel = vertical ? MOTOR_CURRENT_CH_VERT : MOTOR_CURRENT_CH_HORIZ;
    const int32_t threshMA = (int32_t)(_cfg.currentThresholdA * 1000.0f);

    /* Ordre de test : moteurs non isolés triés par sollicitation décroissante. */
    uint8_t order[MOTOR_NUM_CHANNELS];
    uint8_t nOrder = 0;
    for (uint8_t k = 0; k < count; k++) {
        const uint8_t m = first + k;
        if (m >= MOTOR_NUM_CHANNELS || (_isolatedMask & (1u << m))) continue;
        order[nOrder++] = m;
    }
    /* Tri par insertion selon |µs − neutre| décroissant. */
    for (uint8_t i = 1; i < nOrder; i++) {
        const uint8_t key = order[i];
        const int32_t keyUs = abs((int32_t)g_pwm.getPWMuS(key) - (int32_t)PWM_NEUTRAL_US);
        uint8_t j = i;
        while (j > 0) {
            const int32_t usJ = abs((int32_t)g_pwm.getPWMuS(order[j - 1]) - (int32_t)PWM_NEUTRAL_US);
            if (usJ >= keyUs) break;
            order[j] = order[j - 1];
            j--;
        }
        order[j] = key;
    }

    int8_t culprit = -1;
    for (uint8_t k = 0; k < nOrder; k++) {
        const uint8_t m = order[k];
        g_pwm.isolateMotor(m);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_IDENTIFY_SETTLE_MS));
        const int32_t ma = g_sensors.readACS770CurrentmA(inaChannel);
        if (ma < 0) break;   /* mesure perdue : prudence → tout reste isolé */
        if (ma <= threshMA) {
            culprit = (int8_t)m;
            break;           /* courant retombé sous le seuil : coupable trouvé */
        }
    }

    if (culprit >= 0) {
        /* Relâcher les moteurs PROVISOIREMENT isolés qui ne sont PAS le
         * coupable : releaseMotor() lève l'isolement (le canal reste au
         * neutre où isolateMotor() l'a placé — la source de consigne
         * suivante, RPi 5/PID, reprend la main dessus normalement). */
        for (uint8_t k = 0; k < nOrder; k++) {
            if (order[k] == (uint8_t)culprit) continue;
            g_pwm.releaseMotor(order[k]);
        }
        _faultActive = true;
        _setHealth(MOTOR_HEALTH_FAULT);
        _setStatus(vertical ? "Surintensité verticaux" : "Surintensité horizontaux");
        Serial.println("[MOTORS] Coupable identifié : M" + String(culprit + 1)
                       + " — isolé logiquement");

        /* Remontée d'urgence (option) : uniquement si le défaut touche un
         * vertical ET que des verticaux restent utilisables — comptage sur
         * le masque PWM à jour (coupable isolé, innocents déjà relâchés). */
        if (_cfg.emergencyEnable && vertical) {
            const uint8_t isolatedV = __builtin_popcount(
                (g_pwm.getIsolatedMask() >> MOTOR_VERT_FIRST) & 0x0Fu);
            const uint8_t remaining = MOTOR_VERT_COUNT - isolatedV;
            if (remaining > 0) {
                _emergencyActive = true;
                g_pwm.setEmergencySurface(true);
                Serial.println("[MOTORS] REMONTÉE D'URGENCE — " + String(remaining)
                               + " propulseur(s) vertical(aux) restant(s) vers la surface");
            } else {
                Serial.println("[MOTORS] Remontée d'urgence IMPOSSIBLE — aucun "
                               "propulseur vertical disponible");
            }
        } else if (_cfg.emergencyEnable && !vertical) {
            Serial.println("[MOTORS] Défaut horizontal — remontée d'urgence non déclenchée "
                           "(les verticaux restent pilotables par le pilote)");
        }
    } else {
        /* Identification impossible : toute la chaîne du canal reste isolée. */
        _faultActive = true;
        _setHealth(MOTOR_HEALTH_FAULT);
        _setStatus("Surintensité non identifiée");
        Serial.println("[MOTORS] Identification impossible — canal entier isolé par prudence");
    }

    _publishIsolatedMask();
    _sequenceRunning = false;
}

/**
 * @brief Applique le réarmement établi (arrêt du simulateur de test).
 *
 * Lève tous les isolements moteurs, coupe une éventuelle remontée d'urgence
 * et remet défaut/check/streaks à zéro — le voyant repasse à l'état initial
 * (gris : à re-vérifier, seul un check réellement réussi reverdit). Sans
 * effet si l'état est déjà propre (un check réussi n'est jamais écrasé).
 *
 * Contexte : tâche Motors UNIQUEMENT, hors séquence de sécurité (le flag est
 * consommé dans _taskLoop quand aucune séquence ne court).
 */
void MotorManager::_applySimulatorReset() {
    /* État déjà propre : ne rien effacer (ex. extinction du simulateur après
     * un check réussi — le voyant VERT doit rester visible). */
    bool need = false;
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        need = _faultActive || _emergencyActive || _isolatedMask != 0 ||
               g_pwm.getIsolatedMask() != 0 || (_checkDone && !_checkPassed);
        xSemaphoreGive(_mutex);
    }
    if (!need) return;

    /* 1. Lever tous les isolements (les canaux restent au neutre — la reprise
     *    se fait par les consignes normales RPi 5/PID). */
    g_pwm.clearIsolations();

    /* 2. Couper une éventuelle remontée d'urgence (déclenchée par un défaut
     *    simulé : inutile et perturbante une fois la simulation arrêtée). */
    g_pwm.setEmergencySurface(false);

    /* 3. Remise à zéro de l'état interne (le voyant redevient GRIS : seul un
     *    check réel pourra le repasser au vert — jamais de reverdissement
     *    silencieux). */
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _faultActive        = false;
        _emergencyActive    = false;
        _isolatedMask       = 0;
        _overcurrentStreakH = 0;
        _overcurrentStreakV = 0;
        _checkDone          = false;
        _checkPassed        = false;
        _autoArmed          = false;   /* pas de re-déclenchement auto d'impulsions */
        _health             = MOTOR_HEALTH_UNKNOWN;
        xSemaphoreGive(_mutex);
    }
    _setStatus("Réarmé — fin de simulation (isolements levés)");

    Serial.println("[MOTORS] Réarmement établi : simulateur arrêté — isolements levés, "
                   "défaut et verdict de check remis à zéro (relance manuelle possible)");
}

/* =========================================================================
 * HELPERS
 * ========================================================================= */

void MotorManager::_setHealth(MotorHealth h) {
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (h > _health) _health = h;   /* le voyant ne reverdit jamais seul */
        xSemaphoreGive(_mutex);
    }
}

void MotorManager::_setStatus(const char* text) {
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        strncpy(_lastStatus, text, sizeof(_lastStatus) - 1);
        _lastStatus[sizeof(_lastStatus) - 1] = '\0';
        xSemaphoreGive(_mutex);
    }
}

void MotorManager::_publishIsolatedMask() {
    if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _isolatedMask = g_pwm.getIsolatedMask();
        xSemaphoreGive(_mutex);
    }
}

/* =========================================================================
 * TÂCHE FREERTOS
 * ========================================================================= */

void MotorManager::_taskEntry(void* param) {
    static_cast<MotorManager*>(param)->_taskLoop();
}

/**
 * @brief Boucle de la tâche de gestion moteurs (Core 1, période
 *        MOTOR_SURVEIL_PERIOD_MS).
 *
 * Phase 1 — check propulseurs :
 *  - automatique : attente NON BLOQUANTE de la connexion Wi-Fi (la connexion
 *    STA est re-testée à chaque cycle, jamais d'attente passive sur
 *    l'événement réseau) puis, après MOTOR_CHECK_POST_WIFI_DELAY_MS de
 *    stabilisation, exécution UNIQUE — l'option est figée au boot
 *    (_autoArmed) : l'activer à chaud ne déclenche aucune impulsion ;
 *  - manuel (requestManualCheck) : relance immédiate, après le boot comme
 *    après un premier check (utile pour reverdir après un WARNING corrigé).
 *
 * Phase 2 — surveillance continue : tant que l'option « Détection
 * surintensité » est active et qu'aucune séquence de sécurité ne tourne,
 * surveille les canaux 2/3 à chaque cycle. La remontée d'urgence n'a PAS
 * besoin d'être réaffirmée ici : le filtrage de PWMController
 * (setPWMuS/setAllPWM/setAllNeutral) remplace toute consigne des verticaux
 * par la poussée de surface — signal propre, sans oscillation.
 */
void MotorManager::_taskLoop() {
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t wifiConnectedAt = 0;
    bool wifiSeen = false;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* ---- Check démarrage (auto) / relance manuelle : déclenchement ---- */
        bool autoPending = false;
        bool manualReq = false;
        bool seqRunning = false;
        bool faultActive = false;
        bool simResetReq = false;
        if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            autoPending  = _autoArmed && !_checkDone;
            manualReq    = _checkRequested;
            seqRunning   = _sequenceRunning;
            faultActive  = _faultActive;
            simResetReq  = _resetRequested && !_sequenceRunning;
            if (simResetReq) _resetRequested = false;   /* requête consommée */
            xSemaphoreGive(_mutex);
        }

        /* ---- Réarmement établi (arrêt du simulateur) : AVANT la surveillance
         *      pour que la levée des isolements précède toute détection ---- */
        if (simResetReq) {
            _applySimulatorReset();
        }

        /* Sur ce cycle, les drapeaux lus plus haut sont périmés si le
         * réarmement vient de s'appliquer (armement/verdict remis à zéro) :
         * un éventuel check (auto/manuel) repartira proprement au cycle
         * suivant. */
        if ((autoPending || manualReq) && !seqRunning && !simResetReq) {
            if (autoPending && faultActive) {
                /* Défaut confirmé AVANT le check auto (surintensité pendant
                 * l'attente Wi-Fi — moteurs au neutre : ESC/câblage en
                 * court-circuit) : le check est annulé et l'état « en
                 * attente » ne doit pas s'éterniser dans l'interface. */
                if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    _checkDone = true;
                    _checkPassed = false;
                    xSemaphoreGive(_mutex);
                }
                Serial.println("[MOTORS] Check auto annulé : défaut détecté avant son lancement");
            } else {
                bool runNow = manualReq;   /* manuel : immédiat (Wi-Fi actif) */
                if (manualReq && _mutex != nullptr &&
                    xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    _checkRequested = false;   /* requête consommée */
                    xSemaphoreGive(_mutex);
                }
                if (!runNow) {
                    /* Auto : Wi-Fi non bloquant + stabilisation */
                    if (!wifiSeen && g_webServer.isSTAConnected()) {
                        wifiSeen = true;
                        wifiConnectedAt = now;
                        Serial.println("[MOTORS] Wi-Fi STA connecté — check propulseurs dans "
                                       + String(MOTOR_CHECK_POST_WIFI_DELAY_MS / 1000) + " s");
                    }
                    if (wifiSeen &&
                        (now - wifiConnectedAt) >= pdMS_TO_TICKS(MOTOR_CHECK_POST_WIFI_DELAY_MS)) {
                        runNow = true;
                    }
                }

                if (runNow) {
                    if (_mutex != nullptr &&
                        xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        _sequenceRunning = true;
                        xSemaphoreGive(_mutex);
                    }
                    const bool ok = _runStartupCheck();
                    if (_mutex != nullptr &&
                        xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        _checkDone = true;
                        _checkPassed = ok;
                        _checkRequested = false;
                        _sequenceRunning = false;
                        _isolatedMask = g_pwm.getIsolatedMask();
                        xSemaphoreGive(_mutex);
                    }
                }
            }
        }

        /* ---- Surveillance continue (un pas par cycle, cadence
         *      MOTOR_SURVEIL_PERIOD_MS) ---- */
        bool detectOn = false;
        bool seqRunning2 = false;
        if (_mutex != nullptr && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            detectOn = _cfg.detectEnable;
            seqRunning2 = _sequenceRunning;
            xSemaphoreGive(_mutex);
        }
        if (detectOn && !seqRunning2) {
            _surveyStep();
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTOR_SURVEIL_PERIOD_MS));
    }
}
