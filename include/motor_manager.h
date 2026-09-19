/**
 * @file motor_manager.h
 * @brief Gestion de la sécurité moteurs : check au démarrage, surveillance
 *        surintensité, isolement logique et remontée d'urgence (v1.1.0).
 *
 * Architecture (INA3221 + 2× ACS770-100U) :
 *  - Canal 1 INA3221 : tension batterie propulsion (V_BAT) ;
 *  - Canal 2 : courant des propulseurs HORIZONTAUX M1-M4 (ACS770 40 mV/A) ;
 *  - Canal 3 : courant des propulseurs VERTICAUX M5-M8 (ACS770 40 mV/A).
 *
 * Deux mécanismes complémentaires :
 *
 * 1. **Check Propulseurs au démarrage** (option NVS) : juste après la
 *    connexion Wi-Fi (non bloquante), vérifie la tension batterie puis envoie
 *    une courte impulsion séquentielle à chaque moteur (un par un) en lisant
 *    le courant du canal INA3221 correspondant — dépassement du seuil
 *    paramétré = échec, moteur maintenu isolé. Voyant « Statut Moteurs ».
 *
 * 2. **Détection surintensité & isolement** (option NVS) : en navigation,
 *    surveille en continu les canaux 2/3 ; seuil franchi (confirmé
 *    MOTOR_OVERCURRENT_CONFIRM fois) → identifie le moteur incriminé (par
 *    isolement successif et re-lecture du courant) et le désactive
 *    logiquement. Si l'option « Remontée d'urgence » est cochée, les
 *    verticaux restants montent vers la surface (MOTOR_EMERGENCY_SURFACE_US).
 *
 * Toute la configuration est persistée en NVS (namespace « motors ») et
 * éditable depuis l'onglet Paramètres de l'interface Web.
 *
 * @author Didier Dero
 * @version 1.1.0
 */

#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include "config.h"

/* =========================================================================
 * ÉTAT DE SANTÉ MOTEURS (VOYANT « STATUT MOTEURS »)
 * ========================================================================= */

/**
 * @brief État global de la chaîne de propulsion.
 *
 * Ordre croissant de gravité — getStatus() retourne le pire état constaté
 * depuis le boot (le voyant ne « reverdit » qu'après réinitialisation).
 */
enum MotorHealth : uint8_t {
    MOTOR_HEALTH_UNKNOWN   = 0,    ///< Avant check / option désactivée
    MOTOR_HEALTH_OK        = 1,    ///< Check réussi ou actif sans défaut
    MOTOR_HEALTH_WARNING   = 2,    ///< Tension batterie basse (orange)
    MOTOR_HEALTH_FAULT     = 3     ///< Moteur bloqué / isolé (rouge)
};

/* =========================================================================
 * CONFIGURATION PERSISTÉE (NVS « motors »)
 * ========================================================================= */

/**
 * @brief Paramètres utilisateur de la gestion moteurs (onglet Paramètres).
 *
 * Sauvegardés en NVS sous le namespace « motors » (clés check_enable,
 * detect_enable, emergency_enable, cur_thresh, vbat_low) — bornes de
 * validité dans config.h (MOTOR_CURRENT_THRESH_* / MOTOR_VBAT_LOW_*).
 */
struct MotorConfig {
    bool  checkEnable;          ///< « Check Propulseurs au démarrage » (défaut : true)
    bool  detectEnable;         ///< « Détection surintensité & isolation » (défaut : true)
    bool  emergencyEnable;      ///< « Remontée d'urgence si moteur isolé » (défaut : false)
    float currentThresholdA;    ///< « Seuil d'intensité max (moteur bloqué) » en A
    float vbatLowV;             ///< « Seuil de tension basse (batterie propulseurs) » en V
};

/* =========================================================================
 * CLASSE MOTOR MANAGER
 * ========================================================================= */

/**
 * @brief Gestionnaire de sécurité de la propulsion M1-M8.
 *
 * Une tâche FreeRTOS dédiée (Core 1, priorité sous le contrôle/PID) exécute :
 *  - l'attente de la connexion Wi-Fi puis la routine de check au démarrage
 *    (une seule fois par boot — jamais dans la boucle de contrôle, jamais
 *    bloquante pour le reste du système) ;
 *  - la surveillance continue des courants ACS770 (cadence
 *    MOTOR_SURVEIL_PERIOD_MS) et l'escalade isolement → identification →
 *    remontée d'urgence.
 *
 * Thread-safe : l'état est protégé par mutex, les seuils ne sont jamais
 * modifiés à chaud en cours de séquence de sécurité, et les lectures I2C
 * passent par SensorDriver::readACS770CurrentmA (verrou bus n°1).
 */
class MotorManager {
public:
    MotorManager();

    /**
     * @brief Charge la configuration NVS et lance la tâche de gestion.
     *
     * À appeler depuis setup() APRÈS SensorDriver::begin(), PWMController::begin()
     * et BobWebServer::begin() (le check attend la connexion Wi-Fi diffusée
     * par le serveur Web — isSTAConnected() — sans jamais bloquer le boot).
     */
    void begin();

    /** @brief Copie thread-safe de la configuration courante */
    void getConfig(MotorConfig& out);

    /**
     * @brief Applique et persiste une nouvelle configuration (NVS vérifié par
     *        relecture). Les seuils bornés sont clampés avant écriture.
     *
     * @return true si l'écriture NVS est confirmée.
     */
    bool setConfig(const MotorConfig& cfg);

    /** @brief État du voyant « Statut Moteurs » (le pire depuis le boot) */
    MotorHealth getStatus();

    /** @brief true si le check démarrage est terminé (réussi OU échoué) */
    bool isCheckDone();

    /** @brief true si le check a été exécuté et a réussi */
    bool isCheckPassed();

    /** @brief true si le check auto est armé (option active au BOOT) et pas encore lancé */
    bool isCheckPending();

    /** @brief true si la surveillance surintensité a détecté un défaut actif */
    bool isFaultActive();

    /** @brief true si la remontée d'urgence est en cours */
    bool isEmergencySurfaceActive();

    /** @brief Masque 8 bits des moteurs isolés (bit n ↔ M(n+1)) */
    uint8_t getIsolatedMask();

    /**
     * @brief Rejoue manuellement la séquence de check (bouton Paramètres).
     *
     * Refuse si la surveillance a déjà isolé des moteurs ce boot (la seule
     * issue d'un défaut confirmé est une inspection/réinitialisation) ou si
     * une séquence est déjà en cours.
     *
     * @return true si la séquence a été programmée.
     */
    bool requestManualCheck();

    /**
     * @brief Demande un réarmement « établi » (arrêt du simulateur de test).
     *
     * Les défauts observés sous valeurs simulées (surintensité, moteurs
     * isolés, check échoué, remontée d'urgence) ne sont plus justifiés une
     * fois le simulateur coupé : la tâche Motors lève alors tous les
     * isolements et remet le voyant à l'état initial (gris — seul un check
     * réellement réussi peut reverdir). Idempotent ; sans effet si l'état
     * est déjà propre (ex. check réussi, aucun isolement).
     */
    void requestSimulatorReset();

    /**
     * @brief Coupe la remontée d'urgence (arrêt d'urgence humain).
     *
     * La décision humaine (E-Stop du RPi 5, mise à jour OTA) PRIME sur la
     * survie automatique : les verticaux repassent au neutre et la
     * surveillance redevient pleinement opérationnelle (un nouveau défaut
     * pourra être détecté et traité normalement). Idempotent.
     */
    void abortEmergencySurface();

    /** @brief Dernière erreur lisible (diagnostic interface Web) */
    String getLastStatusText();

private:
    MotorConfig      _cfg;                 ///< Configuration active
    MotorHealth      _health;              ///< Pire état constaté depuis le boot
    bool             _checkDone;           ///< Check exécuté (réussi ou non)
    bool             _checkPassed;         ///< Check exécuté avec succès
    bool             _checkRequested;      ///< Check manuel demandé (interface Web)
    bool             _resetRequested;      ///< Réarmement établi demandé (arrêt du simulateur)
    bool             _autoArmed;           ///< Check auto armé (option active AU BOOT uniquement)
    bool             _sequenceRunning;     ///< Séquence de sécurité en cours
    bool             _faultActive;         ///< Défaut confirmé ce boot
    bool             _emergencyActive;     ///< Remontée d'urgence en cours
    uint8_t          _isolatedMask;        ///< Copie du masque d'isolement
    uint8_t          _overcurrentStreakH;  ///< Confirmations consécutives canal horizontaux
    uint8_t          _overcurrentStreakV;  ///< Confirmations consécutives canal verticaux
    char             _lastStatus[96];      ///< Dernier diagnostic (texte)
    SemaphoreHandle_t _mutex;              ///< Protection de l'état

    /* -- Séquence de check au démarrage (tâche dédiée uniquement) -- */
    bool _runStartupCheck();

    /* -- Surveillance continue + identification (tâche dédiée uniquement) -- */
    void _surveyStep();
    void _identifyAndIsolate(bool vertical);

    /* -- Réarmement établi, fin de simulateur (tâche dédiée uniquement) -- */
    void _applySimulatorReset();

    /* -- Helpers -- */
    void _setHealth(MotorHealth h);
    void _setStatus(const char* text);
    void _publishIsolatedMask();

    /* -- Tâche FreeRTOS -- */
    static void _taskEntry(void* param);
    void _taskLoop();
};

/* Instance globale extern (définie dans motor_manager.cpp) */
extern MotorManager g_motors;

#endif /* MOTOR_MANAGER_H */
