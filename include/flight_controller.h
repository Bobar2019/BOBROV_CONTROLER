/**
 * @file flight_controller.h
 * @brief Contrôleur de vol pour le ROV BOB-CONTROL (modes PID et mixage moteurs).
 *
 * Gère un pilotage combinatoire par masque de bits (assistances superposables,
 * protocole v1.4.0) :
 * - MODE_BIT_RT (0x01)      : stabilisation PID roulis + tangage sur les verticaux M5-M8.
 * - MODE_BIT_DEPTH (0x02)   : tenue de profondeur (PID altitude sur M5-M8).
 * - MODE_BIT_CAP (0x04)     : maintien de cap (PID lacet sur les horizontaux M1-M4).
 * - MODE_BIT_SURFACE (0x08) : retour surface PRIORITAIRE (verticaux vers la surface).
 * Exemple : 0x03 = Auto R/T + tenue de profondeur simultanées.
 *
 * La priorité du maître (RPi 5 vs interface Web) est gérée par détection
 * de heartbeat : si aucune trame RPi 5 n'est reçue depuis RPI5_TIMEOUT_MS,
 * le maître bascule sur l'interface Web ESP32.
 *
 * @author Didier Dero
 * @version 1.1.0
 * @date Septembre 2026
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "protocol.h"
#include "autopilot.h"
#include "sensor_driver.h"

/* =========================================================================
 * CONSTANTES
 * ========================================================================= */

/** @brief Délai sans trame RPi 5 avant de basculer en maître Web */
constexpr unsigned long RPI5_MASTER_TIMEOUT_MS = 1000;

/** @brief Nombre de moteurs de propulsion (M0-M7) */
constexpr uint8_t NUM_MOTORS = 8;

/** @brief Nombre de moteurs verticaux (M5-M8 → index 4-7) */
constexpr uint8_t NUM_VERTICAL_MOTORS = 4;

/** @brief Index du premier moteur vertical (M5 = index 4) */
constexpr uint8_t VERTICAL_MOTOR_OFFSET = 4;

/* =========================================================================
 * CLASSE FLIGHTCONTROLLER
 * ========================================================================= */

/**
 * @brief Contrôleur de vol avec modes PID et gestion de priorité maître.
 *
 * Cette classe est conçue pour être appelée depuis la tâche vTaskControl
 * (Core 1, 100 Hz). Elle applique les corrections PID sur les consignes PWM
 * brutes en fonction du mode actif, puis écrit les résultats dans le buffer
 * PWM fourni.
 *
 * Thread-safety : l'accès au mode actif est protégé par un mutex partagé
 * avec la tâche principale.
 */
class FlightController {
public:
    FlightController();

    /**
     * @brief Initialise le contrôleur avec la configuration PID et le mutex.
     * @param cfg    Pointeur vers la structure AutopilotConfig (persistante).
     * @param mutex  Mutex protégeant l'accès à la configuration.
     */
    void begin(AutopilotConfig* cfg, SemaphoreHandle_t mutex);

    /**
     * @brief Définit le masque d'assistances actif (depuis n'importe quelle source).
     * @param mode Masque superposable MODE_BIT_* (0x00 = Passif).
     *             Les bits 4-7 sont ignorés (masqués par MODE_MASK_ALL).
     */
    void setMode(uint8_t mode);

    /** @brief Retourne le masque d'assistances actuellement actif */
    uint8_t getActiveMode() const;

    /** @brief true si la boucle Auto Roulis/Tangage tourne réellement (faux si retour surface) */
    bool isRollPitchActive() const;

    /** @brief true si la tenue de profondeur tourne réellement (faux si retour surface) */
    bool isDepthHoldActive() const;

    /** @brief true si l'Auto Cap est réellement actif */
    bool isCapHoldActive() const;

    /** @brief true si le retour surface prioritaire est actif */
    bool isSurfaceReturnActive() const;

    /** @brief Indique si le RPi 5 est actuellement le maître */
    bool isRPi5Master() const;

    /**
     * @brief Appelé quand une trame valide est reçue du RPi 5.
     *
     * Met à jour le timestamp heartbeat et applique le masque d'assistances reçu.
     * @param rpiMode Masque superposable (champ mode de la trame descendante).
     */
    void updateFromRPi5(uint8_t rpiMode);

    /**
     * @brief Tentative de changement de mode depuis l'interface Web.
     *
     * Si le RPi 5 est maître, la commande est ignorée et un message série
     * est émis. Sinon, le mode est appliqué.
     * @param mode Mode demandé par l'utilisateur Web.
     * @return true si le mode a été appliqué, false si ignoré (RPi maître).
     */
    bool setModeFromWeb(uint8_t mode);

    /**
     * @brief Force (ou relâche) le retour surface d'urgence (voie d'eau).
     *
     * Quand l'override est actif, le bit MODE_BIT_SURFACE est superposé à
     * CHAQUE appel de setMode() (y compris les trames descendantes RPi 5),
     * ce qui garantit que le mode Retour Surface reste prioritaire tant que
     * l'alarme n'est pas relâchée. Le bit STATUS_BIT_SURFACE (0x80) de la
     * trame montante suit automatiquement (isSurfaceReturnActive()).
     * @param active true = forcer le Retour Surface, false = relâcher.
     */
    void setSurfaceOverride(bool active);

    /** @brief true si le retour surface d'urgence (voie d'eau) est forcé */
    bool isSurfaceOverrideActive() const;

    /**
     * @brief Boucle principale de contrôle PID + mixage (appelée à 100 Hz).
     *
     * Applique les corrections PID des assistances actives (superposables)
     * sur le buffer PWM fourni. En mode PASSIF (0x00), le buffer n'est pas
     * modifié.
     *
     * Le clamp 1000-2000 µs est appliqué APRÈS la somme totale de toutes
     * les corrections (Roll + Pitch + Yaw + Profondeur). Le retour surface
     * est prioritaire : il suspend Roll/Pitch et la profondeur, force les
     * verticaux M5-M8 à MOTOR_EMERGENCY_SURFACE_US et laisse l'Auto Cap
     * agir sur M1-M4.
     *
     * @param imu      Données IMU courantes (Euler Roll/Pitch/Yaw).
     * @param press    Données de pression courantes (profondeur).
     * @param pwm      Buffer PWM en entrée (consignes) / sortie (corrigé).
     * @param numMotors Nombre de moteurs à traiter (8 = propulseurs uniquement).
     * @param dt       Pas de temps en secondes (défaut 0.01 = 100 Hz).
     */
    void update(const IMUData& imu, const PressureData& press,
                uint16_t* pwm, uint8_t numMotors, float dt = 0.01f);

    /**
     * @brief Réinitialise les accumulateurs intégraux de tous les PID.
     *
     * Doit être appelé à chaque changement de mode ou désarmement.
     */
    void resetPID();

private:
    AutopilotConfig*  _config;              ///< Pointeur vers la config PID
    SemaphoreHandle_t _mutex;               ///< Mutex configuration partagée

    volatile uint8_t  _activeMode;          ///< Masque d'assistances actif (MODE_BIT_*)
    volatile bool     _rpiMaster;           ///< true = RPi 5 est maître
    unsigned long     _lastRPi5FrameTime;   ///< Timestamp dernière trame RPi 5 valide

    /** @brief Mode demandé par la source (RPi 5 ou Web), sans l'override voie d'eau */
    volatile uint8_t  _requestedMode;

    /** @brief true = Retour Surface forcé par l'alarme voie d'eau (prioritaire) */
    volatile bool     _surfaceOverride;

    /** @brief Cible du PID profondeur (profondeur capturée à l'activation) */
    float _altTarget;

    /** @brief true = profondeur de consigne capturée (invalidé par resetPID) */
    volatile bool _depthHoldValid;

    /** @brief Cap de consigne du PID lacet (capturé à l'activation de l'Auto Cap) */
    float _yawSetpoint;

    /** @brief true = cap de consigne capturé (invalidé par resetPID) */
    volatile bool _yawSetpointValid;

    /** @brief Applique le clamp 1000-2000 µs sur un tableau PWM */
    void _clampPWM(uint16_t* pwm, uint8_t count);

    /** @brief Vérifie le timeout RPi 5 et bascule en maître Web si expiré */
    void _checkRPi5Timeout();
};

/* Instance globale extern */
extern FlightController g_flightCtrl;

#endif /* FLIGHT_CONTROLLER_H */
