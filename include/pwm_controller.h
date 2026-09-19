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
 * Le driver PCA9685 est alloué dans begin() sur le bus ET l'adresse I2C
 * ROUTÉS (v1.3.0, g_i2cRouter — &Wire/&Wire1 et adresse NVS) : ni port ni
 * adresse codés en dur, le module est réassignable depuis l'interface Web
 * (Plug & Play).
 *
 * @author Didier Dero
 * @version 1.3.0
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
     * ISOLEMENT MOTEURS & REMONTÉE D'URGENCE (v1.1.0 — INA3221/ACS770)
     * ----------------------------------------------------------------- */

    /**
     * @brief Isole un canal propulseur : la position de sécurité devient
     *        inconditionnelle, TOUTE consigne future sur ce canal est ignorée.
     *
     * Le canal est physiquement forcé au neutre (1500 µs, canaux 0-7 ESC) et
     * le buffer _current[] y est maintenu : une consigne RPi 5 / PID / banc
     * de test / routine de check ne peut PLUS jamais le réactiver (sécurité
     * « désactivation logicielle » — seule une réinitialisation la lève).
     *
     * @param channel Canal à isoler (0-7 = M1-M8 ; les autres ignorés).
     */
    void isolateMotor(uint8_t channel);

    /**
     * @brief Lève l'isolement PROVISOIRE d'un canal (processus d'identification).
     *
     * Réservé au gestionnaire de sécurité moteurs : _identifyAndIsolate()
     * isole successivement les moteurs suspects pour trouver le coupable, puis
     * relâche par cette méthode les moteurs innocents (le canal reste au neutre
     * où isolateMotor() l'a placé — c'est à la source de consigne suivante,
     * RPi 5 / PID, de reprendre la main). Ne doit JAMAIS servir à relâcher un
     * moteur dont la culpabilité est confirmée.
     *
     * @param channel Canal à relâcher (0-7 = M1-M8 ; les autres ignorés).
     */
    void releaseMotor(uint8_t channel);

    /** @brief Retourne true si le canal est isolé (désactivé logiquement) */
    bool isMotorIsolated(uint8_t channel) const;

    /** @brief Retourne le masque 8 bits des canaux isolés (bit n ↔ canal n) */
    uint8_t getIsolatedMask() const;

    /** @brief Lève tous les isolements (réinitialisation / reboot uniquement) */
    void clearIsolations();

    /**
     * @brief Active/désactive la remontée d'urgence (moteur isolé en plongée).
     *
     * Active : les verticaux M5-M8 (canaux 4-7) non isolés sont pilotés à
     * MOTOR_EMERGENCY_SURFACE_US (poussée vers la surface). Toute consigne
     * future sur ces canaux — PID, RPi 5, watchdog, setAllNeutral() — est
     * filtrée et remplacée par la consigne de surface (mode survie : le ROV
     * remonte quoi qu'il arrive). Le mode Témoin (Dry-Run) reste respecté :
     * aucun mouvement physique n'est forcé tant que les sorties ne sont pas
     * activées.
     * Désactive : retour au neutre des verticaux concernés, sortie laissée au
     * pilote.
     */
    void setEmergencySurface(bool active);

    /** @brief Retourne true si la remontée d'urgence est active */
    bool isEmergencySurfaceActive() const;

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
    Adafruit_PWMServoDriver* _pwm;                      ///< Driver PCA9685, alloué dans begin() sur le bus ROUTÉ (g_i2cRouter)
    uint16_t _current[NUM_PWM_CHANNELS];                ///< Valeurs courantes (µs)
    SemaphoreHandle_t _mutex;                           ///< Mutex d'accès
    volatile bool _physicalOutputsEnabled;              ///< false = Mode Témoin (Dry-Run)
    volatile uint8_t _typeMask;                         ///< Typage canaux 10-15 (bit n ↔ ch 10+n, 1 = bidir)
    volatile uint8_t _isolatedMask;                     ///< Canaux M1-M8 isolés (bit n ↔ canal n) — v1.1.0
    volatile bool _emergencySurface;                    ///< Remontée d'urgence active — v1.1.0

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
