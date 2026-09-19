/**
 * @file simulator.h
 * @brief Simulateur de test virtuel — injection de fausses valeurs capteurs.
 *
 * Établi uniquement : quand la simulation est active, le pilote capteurs
 * (sensor_driver.cpp) bypasse les lectures I2C de l'INA3221 et des INA226 et
 * publie les valeurs injectées depuis l'interface Web (panneau latéral
 * « Simulateur de Test Virtuel », routes GET/POST /api/simulator). La machine
 * à états de sécurité moteurs (surveillance surintensité, identification,
 * isolement, check propulseurs) consomme ces valeurs SANS modification :
 * elle ne voit aucune différence avec des lectures réelles.
 *
 * Sémantique des courants ACS770 simulés (par banc de propulseurs M1-M4 /
 * M5-M8, voir SensorDriver::_simBankCurrentMA) :
 *   - aucun moteur bloqué coché    → courant = slider du banc (injection
 *     pure : le seuil de coupure peut être testé sans coupable identifiable) ;
 *   - moteur(s) bloqué(s) coché(s) → courant = slider tant qu'au moins un
 *     moteur bloqué du banc n'est pas isolé, 0 A sinon — l'isolement du
 *     coupable (provisoire pendant l'identification, puis définitif) fait
 *     retomber le courant, exactement comme un vrai moteur bloqué.
 *
 * État VOLATILE (RAM uniquement, jamais NVS) : tout redémarrage repart
 * simulation INACTIVE — un ROV réel ne peut pas hériter d'un état simulé.
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Septembre 2026
 */

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <Arduino.h>
#include <stdint.h>
#include "config.h"

/** @brief Valeurs injectées par le panneau « Simulateur de Test Virtuel » */
struct SimValues {
    bool    active;     ///< Simulation active (bypass I2C INA3221 + INA226)
    float   vbat1V;     ///< Tension simulée batterie 1 propulsion (V)
    float   vbat2V;     ///< Tension simulée batterie 2 électronique (V)
    float   curH_A;     ///< Courant simulé banc horizontal M1-M4 (A, ACS770 H)
    float   curV_A;     ///< Courant simulé banc vertical M5-M8 (A, ACS770 V)
    uint8_t stallMask;  ///< Bit n = moteur Mn+1 « bloqué » (M1..M8 → bits 0..7)
};

/**
 * @brief Store thread-safe des valeurs du simulateur.
 *
 * Écrivain : tâche Web (POST /api/simulator). Lecteurs : tâche capteurs
 * (_readPower) et tâche de gestion moteurs (readACS770CurrentmA). Sections
 * critiques courtes (copie de structure), sans blocage possible.
 */
class SimulatorStore {
public:
    SimulatorStore();

    /** @brief Copie atomique des valeurs courantes */
    SimValues get();

    /** @brief Remplace les valeurs courantes (bornage côté appelant) */
    void set(const SimValues& v);

private:
    SimValues    _v;    ///< Valeurs courantes
    portMUX_TYPE _mux;  ///< Verrou critique (accès court, sans blocage)
};

/* Instance globale (définie dans simulator.cpp) */
extern SimulatorStore g_sim;

#endif /* SIMULATOR_H */
