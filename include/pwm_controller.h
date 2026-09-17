/**
 * @file pwm_controller.h
 * @brief Pilote PCA9685 pour la génération PWM 16 canaux avec mode Témoin (Dry-Run).
 *
 * Convertit les consignes en microsecondes (1000-2000 µs) en ticks 12 bits
 * pour le contrôleur PWM PCA9685, avec gestion du failsafe neutre et d'un
 * mode Témoin (physicalOutputsEnabled=false) qui neutralise les sorties
 * physiques tout en conservant le buffer de télémétrie pour l'affichage.
 *
 * Les canaux 10 à 15 sont typables (bidirectionnel 1500 µs / unidirectionnel
 * pleine échelle 0-100 %) — choix persisté en NVS (clé « pwm_types »).
 *
 * @author Didier Dero
 * @version 1.2.0
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
 * failsafe (position de sécurité par canal : neutre 1500 µs ou 0 %).
 *
 * **Mode Témoin (Dry-Run)** : quand `physicalOutputsEnabled` est false,
 * les consignes sont enregistrées dans le buffer d'état (pour télémétrie
 * WebSocket et trame montante) mais le PCA9685 reste à la position de
 * sécurité.
 *
 * **Typage canaux 10–15** : chaque canal outil est Bidirectionnel (neutre
 * 1500 µs, position de sécurité 1500 µs) ou Unidirectionnel pleine échelle
 * (0-100 %, position de sécurité 0 % — gradateurs LED).
 */
class PWMController {
public:
    PWMController();

    /** @brief Initialise le PCA9685 à 50 Hz et place tous les canaux à leur position de sécurité */
    void begin();

    /** @brief Applique une consigne PWM en µs sur un canal (borné 1000-2000) */
    void setPWMuS(uint8_t channel, uint16_t microseconds);

    /** @brief Applique un tableau de 16 valeurs PWM (µs) en une seule passe */
    void setAllPWM(const uint16_t* values);

    /** @brief Force les canaux à leur position de sécurité (neutre 1500 µs / 0 %) - mode failsafe */
    void setAllNeutral();

    /** @brief Retourne la valeur PWM actuelle d'un canal en µs */
    uint16_t getPWMuS(uint8_t channel) const;

    /** @brief Copie les 16 valeurs PWM courantes dans un tableau */
    void getCurrentValues(uint16_t* out) const;

    /* -----------------------------------------------------------------
     * TYPAGE DES CANAUX 10–15 (BIDIRECTIONNEL / UNIDIRECTIONNEL)
     * ----------------------------------------------------------------- */

    /**
     * @brief Retourne le masque de typage (6 bits : bit n ↔ canal 10+n, 1 = bidirectionnel).
     */
    uint8_t getChannelTypeMask() const;

    /**
     * @brief Indique si un canal est en mode Bidirectionnel (position de sécurité 1500 µs).
     *
     * Canaux 0–9 : toujours bidirectionnels (ESC/servos — neutre 1500 µs).
     * Canaux 10–15 : selon le masque persisté en NVS.
     */
    bool isBidirectional(uint8_t channel) const;

    /**
     * @brief Persiste un nouveau masque de typage en NVS et l'applique à chaud.
     * @param mask Masque 6 bits (bit n ↔ canal 10+n, 1 = bidirectionnel).
     * @return true si l'écriture NVS est confirmée par relecture.
     */
    bool saveChannelTypes(uint8_t mask);

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
    Adafruit_PWMServoDriver _pwm;                       ///< Driver PCA9685 (bus I2C n°2 / Wire1, GPIO 6/7)
    uint16_t _current[NUM_PWM_CHANNELS];                ///< Valeurs courantes (µs)
    SemaphoreHandle_t _mutex;                           ///< Mutex d'accès
    volatile bool _physicalOutputsEnabled;              ///< false = Mode Témoin (Dry-Run)
    volatile uint8_t _typeMask;                         ///< Typage canaux 10-15 (bit n ↔ ch 10+n, 1 = bidir)

    /** @brief Applique la position de sécurité sur le PCA9685 (sans modifier _current) */
    void _forcePhysicalNeutral();

    /** @brief Charge le masque de typage depuis la NVS (appelée par begin()) */
    void _loadChannelTypes();

    /** @brief Valeur de sécurité d'un canal (1500 µs bidir / 1000 µs unidir) */
    uint16_t _safeUsForChannel(uint8_t channel) const;

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
