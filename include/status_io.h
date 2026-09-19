/**
 * @file status_io.h
 * @brief Entrées numériques de sécurité (armement, voie d'eau) et bandeau
 *        LED d'état WS2812 (v1.0.0).
 *
 * Regroupe les deux nouvelles E/S « terrain » configurables depuis l'onglet
 * Paramètres et persistées en NVS (namespace « statusio ») :
 *
 * 1. **Entrées numériques** (polarité configurable par entrée : contact vers
 *    GND = active, INPUT_PULLUP — défaut ; ou signal actif +3,3 V,
 *    INPUT_PULLDOWN) :
 *    - « Armement / Allumage » (interrupteur magnétique) : état exposé à
 *      l'interface Web et à la télémétrie WebSocket (aucune action sur le
 *      mode de vol — décision volontairement passive) ;
 *    - « Auxiliaire / Détecteur Voie d'eau » : quand elle devient active, le
 *      firmware écrase le mode de vol pour forcer le Retour Surface —
 *      FlightController::setSurfaceOverride() est piloté à 100 Hz par la
 *      tâche de contrôle depuis isWaterAlarm() (le bit STATUS_BIT_SURFACE
 *      0x80 est alors remonté au RPi 5 dans la trame montante).
 *
 * 2. **Bandeau LED d'état WS2812** (Adafruit_NeoPixel, NEO_GRB + NEO_KHZ800) :
 *    matrice de 5 états (effet + couleur RGB, cf. LedStateId) affichée selon
 *    la priorité : Avarie > Retour Surface > Autopilote > RPi5 > Attente.
 *    Effets : Fixe, Pulsation (1,5 s), Stroboscope (3,3 Hz), Éteint.
 *    Une luminosité par état (1-100 %, stockée dans le blob « led_cfg »)
 *    atténue chaque couleur au rendu — économie d'énergie, appliquée à chaud.
 *
 * Une tâche FreeRTOS dédiée (Core 0, sous la priorité Web — le rendu
 * NeoPixel ne doit jamais retarder la boucle de contrôle temps réel du
 * Core 1) échantillonne les entrées à 40 Hz avec anti-rebond (~75 ms) et
 * rafraîchit le bandeau à cadence bornée (LED_ANIM_FRAME_MS) uniquement
 * lorsqu'une couleur doit réellement changer — les états fixes et éteints
 * n'émettent aucun rafraîchissement après leur application.
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Septembre 2026
 */

#ifndef STATUS_IO_H
#define STATUS_IO_H

#include <Arduino.h>
#include "config.h"

/* Forward declaration : la bibliothèque Adafruit_NeoPixel n'est incluse que
 * dans status_io.cpp (l'interface n'expose qu'un pointeur). */
class Adafruit_NeoPixel;

/* =========================================================================
 * EFFETS ET ÉTATS DU BANDEAU LED
 * ========================================================================= */

/**
 * @brief Effet d'animation d'un état du bandeau LED.
 *
 * Persisté en NVS (nibble « effect » de chaque entrée de la matrice blob).
 */
enum LedEffect : uint8_t {
    LED_EFFECT_SOLID  = 0,   ///< Couleur fixe pleine échelle
    LED_EFFECT_PULSE  = 1,   ///< Pulsation douce (respiration LED_PULSE_PERIOD_MS)
    LED_EFFECT_STROBE = 2,   ///< Stroboscope (LED_STROBE_PERIOD_MS, 50 % allumé)
    LED_EFFECT_OFF    = 3    ///< Éteint (le bandeau peut rester branché)
};

/**
 * @brief États signalables du ROV (ordre de la matrice NVS/UI).
 *
 * L'état AFFICHÉ est le premier actif selon la priorité décroissante :
 * FAULT > SURFACE_RETURN > AUTOPILOT > RPI5_CONNECTED > NET_WAIT.
 */
enum LedStateId : uint8_t {
    LED_ST_NET_WAIT       = 0,  ///< 1 · Attente Réseau (aucune liaison maître)
    LED_ST_RPI5_CONNECTED = 1,  ///< 2 · RPi5 Connecté (heartbeat série OK)
    LED_ST_AUTOPILOT      = 2,  ///< 3 · Autopilote actif (R/T, profondeur ou cap)
    LED_ST_SURFACE_RETURN = 3,  ///< 4 · Retour Surface d'urgence (bit 0x08/status 0x80)
    LED_ST_FAULT          = 4   ///< 5 · Avarie (surintensité/isolement, IMU mort)
};

/**
 * @brief Configuration d'un état du bandeau : effet + couleur RGB + luminosité
 *        (5 octets).
 *
 * Les LED_STATE_COUNT entrées forment un blob NVS compact (clé « led_cfg »,
 * namespace « statusio ») — 25 octets au lieu de 15 clés distinctes. L'ancien
 * format 20 octets (effet + couleur seuls) est migré automatiquement au boot.
 */
struct LedStateConfig {
    uint8_t effect;      ///< LedEffect
    uint8_t r;           ///< Composante rouge (0-255)
    uint8_t g;           ///< Composante verte (0-255)
    uint8_t b;           ///< Composante bleue (0-255)
    uint8_t brightness;  ///< Luminosité de l'état en % (LED_WS2812_BRI_MIN..MAX)
};

/**
 * @brief Configuration complète du module (persistée en NVS « statusio »).
 *
 * Clés : in_arm, in_water (UChar, 0xFF = non assigné), in_arm_hi,
 * in_water_hi (UChar, polarité : 0 = actif GND, 1 = actif +3,3 V),
 * led_pin (UChar), led_count (UChar), led_cfg (blob =
 * LedStateConfig[LED_STATE_COUNT] — effet, couleur et luminosité par état).
 */
struct StatusIOConfig {
    int8_t  armPin;                 ///< GPIO de l'entrée « Armement » (-1 = non assigné)
    int8_t  waterPin;               ///< GPIO de l'entrée « Voie d'eau » (-1 = non assigné)
    uint8_t armActiveHigh;          ///< Polarité armement : 0 = actif GND, 1 = actif +3,3 V
    uint8_t waterActiveHigh;        ///< Polarité voie d'eau : 0 = actif GND, 1 = actif +3,3 V
    int8_t  ledPin;                 ///< GPIO de la ligne de données WS2812 (-1 = bandeau inactif)
    uint8_t ledCount;               ///< Nombre de LEDs (LED_WS2812_NUM_MIN..MAX)
    LedStateConfig led[LED_STATE_COUNT];  ///< Matrice des 5 états (effet + couleur + luminosité)
};

/* =========================================================================
 * CLASSE STATUSIO
 * ========================================================================= */

/**
 * @brief Entrées de sécurité debouncées + bandeau LED d'état WS2812.
 *
 * Thread-safety : la configuration est protégée par mutex (lue par la tâche
 * StatusIO à 40 Hz, écrite par les handlers Web) ; les états débouncés sont
 * publiés en volatile et relus sans verrou par la tâche de contrôle (bool
 * unique, lecture atomique sur ESP32-S3).
 */
class StatusIO {
public:
    StatusIO();

    /**
     * @brief Charge la configuration NVS, initialise les entrées (pull-up ou
     *        pull-down selon la polarité), le bandeau NeoPixel et lance la
     *        tâche FreeRTOS dédiée.
     *
     * À appeler depuis setup() APRÈS SensorDriver::begin() et
     * MotorManager::begin() (la tâche lit leurs états pour la matrice LED).
     * Les broches et la polarité ne sont appliquées qu'au boot (pattern des
     * E/S du projet) : une modification depuis le Web déclenche un
     * redémarrage différé.
     */
    void begin();

    /** @brief Copie thread-safe de la configuration courante */
    void getConfig(StatusIOConfig& out);

    /**
     * @brief Applique et persiste une nouvelle configuration (NVS vérifiée
     *        par relecture, bornes clampées). La matrice LED est appliquée à
     *        chaud ; les broches ne sont appliquées qu'au prochain boot
     *        (le handler Web programme le redémarrage si elles changent).
     *
     * @return true si l'écriture NVS est confirmée.
     */
    bool setConfig(const StatusIOConfig& cfg);

    /** @brief true si l'interrupteur d'armement est FERMÉ (entrée active) */
    bool isArmClosed();

    /**
     * @brief true si le détecteur de voie d'eau est en ALARME (entrée active,
     *        GND ou +3,3 V selon la polarité configurée).
     *
     * Lu par la tâche de contrôle (100 Hz) pour écraser le mode de vol en
     * Retour Surface tant que l'alarme est active.
     */
    bool isWaterAlarm();

    /** @brief GPIO de l'entrée « Armement » (-1 = non assigné) */
    int8_t getArmPin() const;

    /** @brief GPIO de l'entrée « Voie d'eau » (-1 = non assigné) */
    int8_t getWaterPin() const;

    /** @brief true si l'entrée « Armement » est active au niveau haut (+3,3 V) */
    bool getArmActiveHigh() const;

    /** @brief true si l'entrée « Voie d'eau » est active au niveau haut (+3,3 V) */
    bool getWaterActiveHigh() const;

    /** @brief GPIO de la ligne de données LED (-1 = bandeau inactif) */
    int8_t getLedPin() const;

    /** @brief Nombre de LEDs réellement appliqué (boot) */
    uint8_t getLedCount() const;

    /**
     * @brief État actuellement affiché par le bandeau (LedStateId),
     *        255 si le bandeau n'est pas configuré. Source : pastille
     *        « État affiché » de l'onglet Paramètres (télémétrie WebSocket).
     */
    uint8_t getActiveLedState() const;

private:
    StatusIOConfig    _cfg;            ///< Configuration active (NVS)
    SemaphoreHandle_t _mutex;          ///< Protection de _cfg

    /* -- Broches appliquées au boot (jamais modifiées ensuite) -- */
    uint8_t _armPin;                   ///< STATUS_IO_PIN_NONE si non assigné
    uint8_t _waterPin;                 ///< STATUS_IO_PIN_NONE si non assigné
    uint8_t _ledPin;                   ///< STATUS_IO_PIN_NONE si bandeau inactif
    uint8_t _ledCountApplied;          ///< Nombre de LEDs de l'instance NeoPixel
    bool    _armActiveHigh;            ///< Polarité armement appliquée au boot
    bool    _waterActiveHigh;          ///< Polarité voie d'eau appliquée au boot

    /* -- Bandeau NeoPixel (instance créée au boot uniquement) -- */
    Adafruit_NeoPixel* _strip;         ///< nullptr = bandeau non configuré

    /* -- États débouncés des entrées (volatile : lus par vTaskControl) -- */
    volatile bool _armStable;          ///< true = interrupteur fermé (entrée active)
    volatile bool _waterStable;        ///< true = alarme voie d'eau (entrée active)
    uint8_t _armDebounce;              ///< Compteur d'échantillons consécutifs
    uint8_t _waterDebounce;            ///< Compteur d'échantillons consécutifs

    /* -- Cache de rendu du bandeau (détection de changement) -- */
    volatile uint8_t _activeLedState;  ///< État affiché (255 = bandeau inactif)
    uint8_t _paintedState;             ///< Dernier état peint
    uint8_t _paintedR;                 ///< Dernière couleur réellement émise
    uint8_t _paintedG;
    uint8_t _paintedB;
    unsigned long _lastAnimMs;         ///< Horodatage de la dernière frame animée

    /* -- Interne (tâche StatusIO uniquement) -- */
    void _sampleInputs();
    void _updateLed(unsigned long now);
    void _fill(uint8_t r, uint8_t g, uint8_t b);

    /* -- Tâche FreeRTOS -- */
    static void _taskEntry(void* param);
    void _taskLoop();
};

/* Instance globale extern (définie dans status_io.cpp) */
extern StatusIO g_statusIo;

#endif /* STATUS_IO_H */
