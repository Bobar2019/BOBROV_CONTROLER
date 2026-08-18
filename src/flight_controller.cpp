/**
 * @file flight_controller.cpp
 * @brief Implémentation du contrôleur de vol BOB-CONTROL (PID + mixage + priorité).
 *
 * Boucle de contrôle exécutée depuis vTaskControl à 100 Hz sur Core 1.
 * Applique les corrections PID selon le mode actif sur les 8 moteurs
 * de propulsion (M1-M8), avec clamp 1000-2000 µs après somme totale.
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Août 2026
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
    , _depthTarget(0.0f)
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
    _depthTarget = 0.0f;

    resetPID();
    Serial.println("[FLIGHT] Contrôleur de vol initialisé — mode PASSIF");
}

/* =========================================================================
 * GESTION DU MODE ACTIF
 * ========================================================================= */

void FlightController::setMode(uint8_t mode) {
    if (mode > MODE_AUTO_FULL) mode = MODE_PASSIF;

    if (mode != _activeMode) {
        resetPID();
        _activeMode = mode;

        const char* label;
        switch (mode) {
            case MODE_AUTO_ROULIS: label = "AUTO_ROULIS"; break;
            case MODE_AUTO_FULL:   label = "AUTO_FULL";   break;
            default:               label = "PASSIF";       break;
        }
        Serial.println("[FLIGHT] Mode changé → " + String(label));
    }
}

uint8_t FlightController::getActiveMode() const {
    return _activeMode;
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

    /* Appliquer le mode dicté par le RPi 5 */
    if (rpiMode <= MODE_AUTO_FULL) {
        setMode(rpiMode);
    }
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
    if (!_config) return;

    if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        _config->roll.integral  = 0.0f;  _config->roll.prev_error  = 0.0f;
        _config->pitch.integral = 0.0f;  _config->pitch.prev_error = 0.0f;
        _config->yaw.integral   = 0.0f;  _config->yaw.prev_error   = 0.0f;
        _config->depth.integral = 0.0f;  _config->depth.prev_error = 0.0f;
        xSemaphoreGive(_mutex);
    }
    Serial.println("[FLIGHT] PID reset (intégrales effacées)");
}

/* =========================================================================
 * BOUCLE DE CONTRÔLE PID + MIXAGE
 * ========================================================================= */

/**
 * @brief Applique les corrections PID sur le buffer PWM selon le mode actif.
 *
 * IMPORTANT : le clamp 1000-2000 µs est appliqué UNE SEULE FOIS à la fin,
 * après la somme de TOUTES les corrections (Roll + Pitch + Yaw + Depth).
 * Cela évite la saturation prématurée d'un axe qui empêcherait les autres
 * corrections de s'exprimer.
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

    /* ---- PID Roll (cible = 0° → roulis à plat) ---- */
    float rollError = 0.0f - imu.euler[0];
    float pidRoll = pidCompute(_config->roll, rollError, dt);

    /* Mixage Roll sur moteurs verticaux M5-M8 :
     * Roll positif (penche à droite) → augmenter côté droit, diminuer côté gauche */
    out[4] += pidRoll;   /* M5 Av-D + */
    out[5] += pidRoll;   /* M6 Ar-D + */
    out[6] -= pidRoll;   /* M7 Ar-G - */
    out[7] -= pidRoll;   /* M8 Av-G - */

    if (_activeMode == MODE_AUTO_FULL) {
        /* ---- PID Pitch (cible = 0° → assiette horizontale) ---- */
        float pitchError = 0.0f - imu.euler[1];
        float pidPitch = pidCompute(_config->pitch, pitchError, dt);

        /* Mixage Pitch sur moteurs verticaux :
         * Pitch positif (nez en haut) → pousser arrière vers le bas, avant vers le haut */
        out[4] -= pidPitch;   /* M5 Av-D : nez monte → moins de poussée avant */
        out[5] += pidPitch;   /* M6 Ar-D : arrière descend */
        out[6] += pidPitch;   /* M7 Ar-G : arrière descend */
        out[7] -= pidPitch;   /* M8 Av-G : nez monte */

        /* ---- PID Yaw (cible = Yaw actuel → maintien du cap) ---- */
        static float _yawSetpoint = 0.0f;
        static bool _yawSetpointInit = false;
        if (!_yawSetpointInit) {
            _yawSetpoint = imu.euler[2];
            _yawSetpointInit = true;
        }
        float yawError = _yawSetpoint - imu.euler[2];
        /* Gérer le wrap-around ±180° */
        while (yawError > 180.0f)  yawError -= 360.0f;
        while (yawError < -180.0f) yawError += 360.0f;
        float pidYaw = pidCompute(_config->yaw, yawError, dt);

        /* Mixage Yaw sur verticaux (torsion diagonale) :
         * Yaw positif (CW vu du dessus) → couple CCW */
        out[4] += pidYaw;   /* M5 Av-D */
        out[5] -= pidYaw;   /* M6 Ar-D */
        out[6] += pidYaw;   /* M7 Ar-G */
        out[7] -= pidYaw;   /* M8 Av-G */

        /* Mixage Yaw sur horizontaux (poussée différentielle) :
         * Avant pousse droite, arrière pousse gauche → rotation CW */
        out[0] += pidYaw;   /* M1 Av-D */
        out[1] -= pidYaw;   /* M2 Ar-D */
        out[2] -= pidYaw;   /* M3 Ar-G */
        out[3] += pidYaw;   /* M4 Av-G */

        /* ---- PID Profondeur (maintien de la profondeur cible) ---- */
        float depthError = _depthTarget - press.depth_m;
        float pidDepth = pidCompute(_config->depth, depthError, dt);

        /* Mixage Depth : uniforme sur tous les verticaux */
        out[4] += pidDepth;   /* M5 */
        out[5] += pidDepth;   /* M6 */
        out[6] += pidDepth;   /* M7 */
        out[7] += pidDepth;   /* M8 */
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
