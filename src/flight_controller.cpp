/**
 * @file flight_controller.cpp
 * @brief Implémentation du contrôleur de vol BOB-CONTROL (PID + mixage + priorité).
 *
 * Boucle de contrôle exécutée depuis vTaskControl à 100 Hz sur Core 1.
 * Les assistances (masque MODE_BIT_*, superposables) appliquent leurs
 * corrections PID sur les 8 moteurs de propulsion (M1-M8), avec clamp
 * 1000-2000 µs après somme totale. Le retour surface est prioritaire.
 *
 * @author Didier Dero
 * @version 1.1.0
 * @date Septembre 2026
 */

#include "flight_controller.h"

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

FlightController g_flightCtrl;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

FlightController::FlightController()
    : _config(nullptr)
    , _mutex(nullptr)
    , _activeMode(MODE_PASSIF)
    , _rpiMaster(false)
    , _lastRPi5FrameTime(0)
    , _altTarget(0.0f)
    , _depthHoldValid(false)
    , _yawSetpoint(0.0f)
    , _yawSetpointValid(false)
{
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

void FlightController::begin(AutopilotConfig* cfg, SemaphoreHandle_t mutex) {
    _config = cfg;
    _mutex  = mutex;
    _activeMode = MODE_PASSIF;
    _rpiMaster  = false;
    _lastRPi5FrameTime = 0;
    _altTarget = 0.0f;
    _depthHoldValid = false;
    _yawSetpoint = 0.0f;
    _yawSetpointValid = false;

    resetPID();
    Serial.println("[FLIGHT] Contrôleur de vol initialisé — mode PASSIF");
}

/* =========================================================================
 * GESTION DU MODE ACTIF
 * ========================================================================= */

void FlightController::setMode(uint8_t mode) {
    /* Masque de bits : ignorer les bits inconnus (4-7) */
    mode &= MODE_MASK_ALL;

    if (mode != _activeMode) {
        resetPID();
        _activeMode = mode;

        /* Libellé construit à partir des bits actifs (assistances superposables) */
        String label;
        if (mode == MODE_PASSIF) {
            label = "PASSIF";
        } else {
            if (mode & MODE_BIT_RT)      label += "R/T ";
            if (mode & MODE_BIT_DEPTH)   label += "PROFONDEUR ";
            if (mode & MODE_BIT_CAP)     label += "CAP ";
            if (mode & MODE_BIT_SURFACE) label += "SURFACE ";
            label.trim();
        }
        Serial.println("[FLIGHT] Mode changé → 0x" + String(mode, HEX) + " (" + label + ")");
    }
}

uint8_t FlightController::getActiveMode() const {
    return _activeMode;
}

/* =========================================================================
 * ÉTAT RÉEL DES ASSERVISSEMENTS (bits status de la télémétrie)
 * ========================================================================= */

bool FlightController::isRollPitchActive() const {
    /* Suspendu tant que le retour surface est actif */
    return (_activeMode & MODE_BIT_RT) != 0 && (_activeMode & MODE_BIT_SURFACE) == 0;
}

bool FlightController::isDepthHoldActive() const {
    /* Suspendu tant que le retour surface est actif */
    return (_activeMode & MODE_BIT_DEPTH) != 0 && (_activeMode & MODE_BIT_SURFACE) == 0;
}

bool FlightController::isCapHoldActive() const {
    return (_activeMode & MODE_BIT_CAP) != 0;
}

bool FlightController::isSurfaceReturnActive() const {
    return (_activeMode & MODE_BIT_SURFACE) != 0;
}

/* =========================================================================
 * GESTION DE PRIORITÉ MAÎTRE (RPi 5 vs Web ESP32)
 * ========================================================================= */

bool FlightController::isRPi5Master() const {
    return _rpiMaster;
}

void FlightController::updateFromRPi5(uint8_t rpiMode) {
    _lastRPi5FrameTime = millis();

    /* Passer en maître RPi 5 si ce n'était pas déjà le cas */
    if (!_rpiMaster) {
        _rpiMaster = true;
        Serial.println("[FLIGHT] Maître → RPi 5");
    }

    /* Appliquer le masque d'assistances dicté par le RPi 5 (bits 4-7 ignorés) */
    setMode(rpiMode);
}

bool FlightController::setModeFromWeb(uint8_t mode) {
    _checkRPi5Timeout();

    if (_rpiMaster) {
        Serial.println("[FLIGHT] Commande Web ignorée — RPi 5 est maître");
        return false;
    }

    setMode(mode);
    return true;
}

void FlightController::_checkRPi5Timeout() {
    if (_rpiMaster && _lastRPi5FrameTime > 0) {
        unsigned long now = millis();
        if (now - _lastRPi5FrameTime > RPI5_MASTER_TIMEOUT_MS) {
            _rpiMaster = false;
            Serial.println("[FLIGHT] Maître → Manuel ESP32 (timeout RPi 5)");
        }
    }
}

/* =========================================================================
 * RESET PID
 * ========================================================================= */

void FlightController::resetPID() {
    /* Invalider les captures d'activation : la prochaine itération de update()
     * recapturera la profondeur / le cap alors courants. */
    _depthHoldValid = false;
    _yawSetpointValid = false;

    if (!_config) return;

    if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _config->roll.integral  = 0.0f;  _config->roll.prev_error  = 0.0f;
        _config->pitch.integral = 0.0f;  _config->pitch.prev_error = 0.0f;
        _config->yaw.integral   = 0.0f;  _config->yaw.prev_error   = 0.0f;
        _config->alt.integral = 0.0f;  _config->alt.prev_error = 0.0f;
        xSemaphoreGive(_mutex);
    }
    Serial.println("[FLIGHT] PID reset (intégrales effacées)");
}

/* =========================================================================
 * BOUCLE DE CONTRÔLE PID + MIXAGE
 * ========================================================================= */

/**
 * @brief Applique les corrections PID des assistances actives (superposables).
 *
 * IMPORTANT : le clamp 1000-2000 µs est appliqué UNE SEULE FOIS à la fin,
 * après la somme de TOUTES les corrections (Roll + Pitch + Yaw + Altitude).
 * Cela évite la saturation prématurée d'un axe qui empêcherait les autres
 * corrections de s'exprimer.
 *
 * Priorité : le retour surface (MODE_BIT_SURFACE) suspend Roll/Pitch et la
 * tenue de profondeur, et force les verticaux M5-M8 vers la surface ;
 * l'Auto Cap reste indépendant (voie horizontale M1-M4).
 *
 * Convention des moteurs :
 *   M1 (idx 0) = Av-D horizontal    M5 (idx 4) = Av-D vertical
 *   M2 (idx 1) = Ar-D horizontal    M6 (idx 5) = Ar-D vertical
 *   M3 (idx 2) = Ar-G horizontal    M7 (idx 6) = Ar-G vertical
 *   M4 (idx 3) = Av-G horizontal    M8 (idx 7) = Av-G vertical
 */
void FlightController::update(const IMUData& imu, const PressureData& press,
                               uint16_t* pwm, uint8_t numMotors, float dt) {
    if (!_config || !pwm || numMotors < NUM_MOTORS) return;

    /* Vérifier le timeout RPi 5 à chaque itération */
    _checkRPi5Timeout();

    /* Mode PASSIF : aucune correction */
    if (_activeMode == MODE_PASSIF) return;

    /* Vérifier que le mutex est disponible */
    if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(1)) != pdTRUE) return;

    /* ---- Conversion des valeurs PWM en float pour le mixage ---- */
    float out[NUM_MOTORS];
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        out[i] = (float)pwm[i];
    }

    /* ---- Décomposition du masque d'assistances superposables ---- */
    const uint8_t mode = _activeMode;
    const bool surfaceReturn = (mode & MODE_BIT_SURFACE) != 0;

    /* ---- PID Roll + Pitch (bit MODE_BIT_RT — suspendu par le retour surface) ---- */
    if (!surfaceReturn && (mode & MODE_BIT_RT)) {
        /* PID Roll (cible = 0° → roulis à plat) */
        float rollError = 0.0f - imu.euler[0];
        float pidRoll = pidCompute(_config->roll, rollError, dt);

        /* Mixage Roll sur moteurs verticaux M5-M8 :
         * Roll positif (penche à droite) → augmenter côté droit, diminuer côté gauche */
        out[4] += pidRoll;   /* M5 Av-D + */
        out[5] += pidRoll;   /* M6 Ar-D + */
        out[6] -= pidRoll;   /* M7 Ar-G - */
        out[7] -= pidRoll;   /* M8 Av-G - */

        /* PID Pitch (cible = 0° → assiette horizontale) */
        float pitchError = 0.0f - imu.euler[1];
        float pidPitch = pidCompute(_config->pitch, pitchError, dt);

        /* Mixage Pitch sur moteurs verticaux :
         * Pitch positif (nez en haut) → pousser arrière vers le bas, avant vers le haut */
        out[4] -= pidPitch;   /* M5 Av-D : nez monte → moins de poussée avant */
        out[5] += pidPitch;   /* M6 Ar-D : arrière descend */
        out[6] += pidPitch;   /* M7 Ar-G : arrière descend */
        out[7] -= pidPitch;   /* M8 Av-G : nez monte */
    }

    /* ---- PID Yaw / Auto Cap (bit MODE_BIT_CAP — indépendant du retour surface) ---- */
    if (mode & MODE_BIT_CAP) {
        /* Capture du cap de consigne à l'activation de la boucle (invalidée par resetPID) */
        if (!_yawSetpointValid) {
            _yawSetpoint = imu.euler[2];
            _yawSetpointValid = true;
        }
        float yawError = _yawSetpoint - imu.euler[2];
        /* Gérer le wrap-around ±180° */
        while (yawError > 180.0f)  yawError -= 360.0f;
        while (yawError < -180.0f) yawError += 360.0f;
        float pidYaw = pidCompute(_config->yaw, yawError, dt);

        /* Mixage vectoriel Yaw sur M1-M4 uniquement (configuration X) :
         * Axes physiques : M1/M3 sur 135° (poussée positive vers l'avant-gauche),
         *                  M2/M4 sur 45°  (poussée positive vers l'avant-droite).
         * pidYaw > 0 (cap à augmenter, CW vu du dessus) :
         *   M1/M2 (côté droit) en inverse + M3/M4 (côté gauche) en avant
         *   → l'avant part à droite, l'arrière à gauche → rotation CW.
         * Les verticaux M5-M8 ne gèrent STRICTEMENT que Roll / Pitch / Altitude. */
        out[0] -= pidYaw;   /* M1 Av-D */
        out[1] -= pidYaw;   /* M2 Ar-D */
        out[2] += pidYaw;   /* M3 Ar-G */
        out[3] += pidYaw;   /* M4 Av-G */
    }

    /* ---- PID Altitude / Tenue de profondeur (bit MODE_BIT_DEPTH — suspendu si surface) ---- */
    if (!surfaceReturn && (mode & MODE_BIT_DEPTH)) {
        /* Capture de la profondeur COURANTE à l'activation (invalidée par resetPID) */
        if (!_depthHoldValid) {
            _altTarget = press.altitude_m;
            _depthHoldValid = true;
        }

        /* Erreur > 0 (sous la cible) → poussée verticale vers le haut. */
        float altError = _altTarget - press.altitude_m;
        float pidAlt = pidCompute(_config->alt, altError, dt);

        /* Mixage Altitude : uniforme sur tous les verticaux */
        out[4] += pidAlt;   /* M5 */
        out[5] += pidAlt;   /* M6 */
        out[6] += pidAlt;   /* M7 */
        out[7] += pidAlt;   /* M8 */
    }

    /* ---- Retour surface PRIORITAIRE (bit MODE_BIT_SURFACE) ----
     * Les verticaux sont forcés à leur position de remontée d'urgence ;
     * un éventuel Auto Cap conserve la voie horizontale M1-M4. */
    if (surfaceReturn) {
        for (uint8_t i = VERTICAL_MOTOR_OFFSET; i < NUM_MOTORS; i++) {
            out[i] = (float)MOTOR_EMERGENCY_SURFACE_US;
        }
    }

    /* ---- CLAMP FINAL après somme de toutes les corrections ---- */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (out[i] < (float)PWM_MIN_US) out[i] = (float)PWM_MIN_US;
        if (out[i] > (float)PWM_MAX_US) out[i] = (float)PWM_MAX_US;
        pwm[i] = (uint16_t)out[i];
    }

    xSemaphoreGive(_mutex);
}

/* =========================================================================
 * UTILITAIRES
 * ========================================================================= */

void FlightController::_clampPWM(uint16_t* pwm, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        if (pwm[i] < PWM_MIN_US) pwm[i] = PWM_MIN_US;
        if (pwm[i] > PWM_MAX_US) pwm[i] = PWM_MAX_US;
    }
}
