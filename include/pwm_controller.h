/**
 * @file pwm_controller.h
 * @brief Pilote PCA9685 pour la génération PWM 16 canaux avec mode Témoin (Dry-Run).
 *
 * Convertit les consignes en microsecondes (1000-2000 µs) en ticks 12 bits
 * pour le contrôleur PWM PCA9685, avec gestion du failsafe neutre et d'un
 * mode Témoin (physicalOutputsEnabled=false) qui neutralise les sorties
 * physiques tout en conservant le buffer de télémétrie pour l'affichage.
 *
 * @author Didier Dero
 * @version 1.1.0
 */

#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

/**
 * @brief Classe de contrôle du PCA9685 avec sécurité intégrée et mode Témoin.
 *
 * Gère l'initialisation, l'application des valeurs PWM et le mode
 * failsafe (tous les canaux au neutre 1500 µs).
 *
 * **Mode Témoin (Dry-Run)** : quand `physicalOutputsEnabled` est false,
 * les consignes sont enregistrées dans le buffer d'état (pour télémétrie
 * WebSocket et trame montante) mais le PCA9685 reste au neutre.
 */
class PWMController {
public:
    PWMController();

    /** @brief Initialise le PCA9685 à 50 Hz et met tous les canaux au neutre */
    void begin();

    /** @brief Applique une consigne PWM en µs sur un canal (borné 1000-2000) */
    void setPWMuS(uint8_t channel, uint16_t microseconds);

    /** @brief Applique un tableau de 16 valeurs PWM (µs) en une seule passe */
    void setAllPWM(const uint16_t* values);

    /** @brief Force tous les canaux au neutre (1500 µs) - mode failsafe */
    void setAllNeutral();

    /** @brief Retourne la valeur PWM actuelle d'un canal en µs */
    uint16_t getPWMuS(uint8_t channel) const;

    /** @brief Copie les 16 valeurs PWM courantes dans un tableau */
    void getCurrentValues(uint16_t* out) const;

    /* -----------------------------------------------------------------
     * MODE TÉMOIN (DRY-RUN) — SORTIES PHYSIQUES
     * ----------------------------------------------------------------- */

    /**
     * @brief Active ou désactive les sorties physiques du PCA9685.
     *
     * - `true`  : les consignes PWM sont écrites sur le PCA9685 (propulsion active).
     * - `false` : les consignes sont mémorisées dans le buffer mais le PCA9685
     *   reste au neutre (1500 µs canaux 0-11, 0% canaux 12-15).
     *
     * @param enabled État souhaité des sorties physiques.
     */
    void setPhysicalOutputsEnabled(bool enabled);

    /** @brief Retourne true si les sorties physiques sont actives */
    bool isPhysicalOutputsEnabled() const;

private:
    Adafruit_PWMServoDriver _pwm;                       ///< Driver PCA9685
    uint16_t _current[NUM_PWM_CHANNELS];                ///< Valeurs courantes (µs)
    SemaphoreHandle_t _mutex;                           ///< Mutex d'accès
    volatile bool _physicalOutputsEnabled;              ///< false = Mode Témoin (Dry-Run)

    /** @brief Applique le neutre de sécurité sur le PCA9685 (sans modifier _current) */
    void _forcePhysicalNeutral();

    /**
     * @brief Convertit des microsecondes en ticks PCA9685 (0-4095).
     *
     * Formule : ticks = µs × (4096 / (1000000 / freq))
     * À 50 Hz : période = 20000 µs, donc ticks = µs × 4096 / 20000
     *
     * @param us Valeur en microsecondes.
     * @return Nombre de ticks (0-4095).
     */
    uint16_t _usToTicks(uint16_t us) const;
};

/* Instance globale extern */
extern PWMController g_pwm;

#endif /* PWM_CONTROLLER_H */
