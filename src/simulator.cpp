/**
 * @file simulator.cpp
 * @brief Implémentation du simulateur de test virtuel (voir simulator.h).
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Septembre 2026
 */

#include "simulator.h"

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */
SimulatorStore g_sim;

/* =========================================================================
 * STORE THREAD-SAFE
 * ========================================================================= */

SimulatorStore::SimulatorStore()
    : _mux(portMUX_INITIALIZER_UNLOCKED)
{
    /* Simulation INACTIVE au boot (RAM seule — jamais persistée en NVS :
     * un redémarrage doit toujours rendre la main aux capteurs réels). */
    _v.active    = false;
    _v.vbat1V    = SIM_VBAT1_DEFAULT_V;
    _v.vbat2V    = SIM_VBAT2_DEFAULT_V;
    _v.curH_A    = 0.0f;
    _v.curV_A    = 0.0f;
    _v.stallMask = 0;
}

SimValues SimulatorStore::get() {
    SimValues out;
    portENTER_CRITICAL(&_mux);
    out = _v;
    portEXIT_CRITICAL(&_mux);
    return out;
}

void SimulatorStore::set(const SimValues& v) {
    portENTER_CRITICAL(&_mux);
    _v = v;
    portEXIT_CRITICAL(&_mux);
}
