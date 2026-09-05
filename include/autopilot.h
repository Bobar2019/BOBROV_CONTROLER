/**
 * @file autopilot.h
 * @brief Structures et paramètres des contrôleurs PID pour l'autopilote BOB-ROV.
 *
 * Définit les structures de paramètres PID pour les boucles d'asservissement
 * de roulis, tangage et profondeur. Les paramètres sont stockés en NVS
 * et modifiables via l'interface Web (onglet Paramètres).
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Août 2026
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <stdint.h>

/**
 * @brief Paramètres d'un contrôleur PID simple.
 *
 * Structure utilisée pour les quatre boucles d'asservissement
 * (roll, pitch, yaw, altitude). Les gains sont appliqués dans la tâche
 * vTaskControl lorsque le mode autopilote est actif.
 */
struct PIDParams {
    float kp;           ///< Gain proportionnel
    float ki;           ///< Gain intégral
    float kd;           ///< Gain dérivé
    float integral;     ///< Accumulateur intégral (réinitialisé à chaque désarmement)
    float prev_error;   ///< Erreur précédente pour le calcul de la dérivée
    float output_min;   ///< Borne inférieure de la sortie PID
    float output_max;   ///< Borne supérieure de la sortie PID
    float integral_max; ///< Anti-windup : limite de l'intégrale
};

/**
 * @brief Ensemble complet des paramètres d'asservissement.
 *
 * Regroupe les PID des trois axes et le timeout du watchdog série.
 * Cette structure est sauvegardée/restaurée depuis la NVS.
 */
struct AutopilotConfig {
    PIDParams roll;             ///< PID stabilisation roulis
    PIDParams pitch;            ///< PID stabilisation tangage
    PIDParams yaw;              ///< PID stabilisation cap/lacet
    PIDParams alt;              ///< PID maintien d'altitude (verticaux)
    uint32_t  watchdog_ms;      ///< Timeout watchdog série (ms), défaut 500
};

/**
 * @brief Fournit une configuration autopilote avec des valeurs par défaut.
 *
 * @return AutopilotConfig initialisée avec des gains conservateurs.
 */
inline AutopilotConfig getDefaultAutopilotConfig() {
    AutopilotConfig cfg;
    /* PID Roll : stabilisation latérale */
    cfg.roll = {2.0f, 0.05f, 0.5f, 0.0f, 0.0f, -200.0f, 200.0f, 100.0f};
    /* PID Pitch : stabilisation longitudinale */
    cfg.pitch = {2.0f, 0.05f, 0.5f, 0.0f, 0.0f, -200.0f, 200.0f, 100.0f};
    /* PID Yaw : stabilisation cap/lacet */
    cfg.yaw = {1.5f, 0.02f, 0.3f, 0.0f, 0.0f, -150.0f, 150.0f, 80.0f};
    /* PID Altitude : maintien de l'altitude cible (erreur > 0 → poussée vers le haut) */
    cfg.alt = {3.0f, 0.1f, 1.0f, 0.0f, 0.0f, -300.0f, 300.0f, 150.0f};
    /* Watchdog série : 500 ms par défaut */
    cfg.watchdog_ms = 500;
    return cfg;
}

/**
 * @brief Calcule une itération du contrôleur PID.
 *
 * @param[in,out] pid     Paramètres et état du PID.
 * @param[in]     error   Erreur courante (consigne - mesure).
 * @param[in]     dt      Pas de temps en secondes.
 * @return Sortie bornée du contrôleur.
 */
inline float pidCompute(PIDParams& pid, float error, float dt) {
    if (dt <= 0.0f) return 0.0f;

    /* Terme proportionnel */
    float p_term = pid.kp * error;

    /* Terme intégral avec anti-windup */
    pid.integral += error * dt;
    if (pid.integral > pid.integral_max)  pid.integral = pid.integral_max;
    if (pid.integral < -pid.integral_max) pid.integral = -pid.integral_max;
    float i_term = pid.ki * pid.integral;

    /* Terme dérivé (filtré par la différence d'erreur) */
    float derivative = (error - pid.prev_error) / dt;
    float d_term = pid.kd * derivative;
    pid.prev_error = error;

    /* Somme et bornage de la sortie */
    float output = p_term + i_term + d_term;
    if (output > pid.output_max) output = pid.output_max;
    if (output < pid.output_min) output = pid.output_min;

    return output;
}

#endif /* AUTOPILOT_H */
