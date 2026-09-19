/**
 * @file i2c_router.cpp
 * @brief Implémentation du routeur I2C Plug & Play (NVS + topologie par défaut).
 *
 * La carte de routage vit dans le namespace NVS « config » (même namespace
 * que les broches SDA/SCL : une seule zone « configuration matérielle »,
 * effaçable ensemble via « Réinitialiser les paramètres » de l'interface).
 * Chaque module porte DEUX clés :
 *   - « route_<module> » (1 = Wire, 2 = Wire1) — le bus ;
 *   - « addr_<module> »  (0x01–0x7F)            — l'adresse I2C 7 bits,
 *     choisie dans l'interface parmi les adresses découvertes par le scan.
 *
 * @author Didier Dero
 * @version 1.3.0
 * @date Septembre 2026
 */

#include "i2c_router.h"
#include <Preferences.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Routeur I2C global — chargé par SensorDriver::begin() au boot. */
I2CRouter g_i2cRouter;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

I2CRouter::I2CRouter() {
    /* Topologie de référence jusqu'au load() : jamais de bus ni d'adresse
     * indéterminés, même si une instance accédait au routeur avant
     * SensorDriver::begin(). */
    _setDefaults();
}

void I2CRouter::_setDefaults() {
    /* Topologie de référence (historique) — bus n°1 : capteurs de propulsion
     * (BNO085 + INA3221 + 2× INA226) ; bus n°2 : pression + PWM
     * (MS5803 + PCA9685, partagé avec la tâche de contrôle).
     * Les adresses par défaut proviennent de config.h (seul usage désormais :
     * valeurs de repli du routage). */
    _bus[I2C_DEV_BNO085]  = 1;
    _bus[I2C_DEV_INA3221] = 1;
    _bus[I2C_DEV_INA226_2] = 1;
    _bus[I2C_DEV_INA226_3] = 1;
    _bus[I2C_DEV_MS5803]  = 2;
    _bus[I2C_DEV_PCA9685] = 2;

    _addr[I2C_DEV_BNO085]   = BNO085_I2C_ADDR;
    _addr[I2C_DEV_INA3221]  = INA3221_I2C_ADDR;
    _addr[I2C_DEV_INA226_2] = INA226_2_I2C_ADDR;
    _addr[I2C_DEV_INA226_3] = INA226_3_I2C_ADDR;
    _addr[I2C_DEV_MS5803]   = MS5803_I2C_ADDR;
    _addr[I2C_DEV_PCA9685]  = PCA9685_I2C_ADDR;
}

/* =========================================================================
 * CHARGEMENT / SAUVEGARDE NVS
 * ========================================================================= */

void I2CRouter::load() {
    _setDefaults();   /* base de repli pour toute clé absente ou invalide */

    Preferences prefs;
    prefs.begin("config", true);   /* lecture seule */
    bool tweaked = false;
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;

        /* Bus : valeur corrompue (ni 1 ni 2) → repli sur le défaut,
         * silencieusement : un NVS endommagé ne doit jamais produire un bus
         * hors bornes. */
        const uint8_t storedBus = prefs.getUChar(keyOf(dev), _bus[dev]);
        if (storedBus == 1 || storedBus == 2) {
            if (storedBus != _bus[dev]) tweaked = true;
            _bus[dev] = storedBus;
        }

        /* Adresse : hors 0x01–0x7F (0x00 réservé, > 0x7F invalide, 0 par
         * sentinelle d'absence) → repli sur le défaut. */
        const uint8_t storedAddr = prefs.getUChar(addrKeyOf(dev), _addr[dev]);
        if (storedAddr >= 0x01 && storedAddr <= 0x7F) {
            if (storedAddr != _addr[dev]) tweaked = true;
            _addr[dev] = storedAddr;
        }
    }
    prefs.end();

    /* Journaliser la carte active (une ligne par périphérique routé) */
    Serial.println("[I2C] Routage Plug & Play : "
                   + String(tweaked ? "NVS personnalisé" : "défauts (NVS vierge)"));
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;
        char hex[6];
        snprintf(hex, sizeof(hex), "0x%02X", _addr[dev]);
        Serial.println("[I2C]   " + String(nameOf(dev)) + " (" + hex
                       + ") → bus n°" + String(_bus[dev]));
    }

    /* Conflit (deux modules même bus + même adresse) : ne peut pas provenir
     * du NVS si save() est le seul chemin d'écriture (il refuse ce cas), mais
     * une NVS éditée à la main ou un downgrade de firmware peut le produire —
     * l'avertir plutôt que d'échouer au boot : _identifyI2CDevice arbitrera
     * par le routage pour l'étiquetage du scan. */
    I2CDeviceId ca, cb;
    if (firstConflict(ca, cb)) {
        Serial.println("[I2C] AVERTISSEMENT : " + String(shortNameOf(ca)) + " et "
                       + String(shortNameOf(cb)) + " partagent la même adresse (0x"
                       + String(_addr[ca] < 16 ? "0" : "") + String(_addr[ca], HEX)
                       + ") sur le bus n°" + String(_bus[ca])
                       + " — re-sauvegarder le routage depuis l'interface Web");
    }
}

bool I2CRouter::save() {
    /* Garde-fou structurel : deux modules sur le même bus à la même adresse
     * seraient indistinguables (cas historique : INA3221 et PCA9685, tous
     * deux 0x40 — les laisser sur le même bus casserait le Check Propulseurs). */
    I2CDeviceId ca, cb;
    if (firstConflict(ca, cb)) {
        Serial.println("[I2C] Sauvegarde refusée : " + String(shortNameOf(ca)) + " et "
                       + String(shortNameOf(cb)) + " sur le même bus n°" + String(_bus[ca])
                       + " à la même adresse");
        return false;
    }

    Preferences prefs;
    if (!prefs.begin("config", false)) {
        return false;
    }

    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;
        prefs.putUChar(keyOf(dev), _bus[dev]);
        prefs.putUChar(addrKeyOf(dev), _addr[dev]);
    }

    /* Vérification par relecture : sentinelle 0xFF = clé absente */
    bool ok = true;
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;
        if (prefs.getUChar(keyOf(dev), 0xFF) != _bus[dev] ||
            prefs.getUChar(addrKeyOf(dev), 0xFF) != _addr[dev]) {
            ok = false;
            break;
        }
    }
    prefs.end();

    if (ok) {
        Serial.println("[I2C] Routage enregistré en NVS (6 modules, adresses incluses) — appliqué au prochain boot");
    } else {
        Serial.println("[I2C] Échec de sauvegarde du routage (relecture NVS différente)");
    }
    return ok;
}

/* =========================================================================
 * ACCESSEURS
 * ========================================================================= */

TwoWire* I2CRouter::wireOf(I2CDeviceId dev) const {
    return wireOfBus(_bus[dev]);
}

bool I2CRouter::setBusOf(I2CDeviceId dev, uint8_t bus) {
    if (dev >= I2C_DEV_COUNT || (bus != 1 && bus != 2)) {
        return false;
    }
    _bus[dev] = bus;
    return true;
}

bool I2CRouter::setAddrOf(I2CDeviceId dev, uint8_t addr) {
    if (dev >= I2C_DEV_COUNT || addr < 0x01 || addr > 0x7F) {
        return false;
    }
    _addr[dev] = addr;
    return true;
}

bool I2CRouter::hasAddressConflict() const {
    for (uint8_t i = 0; i < I2C_DEV_COUNT; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < I2C_DEV_COUNT; j++) {
            if (_bus[i] == _bus[j] && _addr[i] == _addr[j]) {
                return true;
            }
        }
    }
    return false;
}

bool I2CRouter::firstConflict(I2CDeviceId& a, I2CDeviceId& b) const {
    for (uint8_t i = 0; i < I2C_DEV_COUNT; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < I2C_DEV_COUNT; j++) {
            if (_bus[i] == _bus[j] && _addr[i] == _addr[j]) {
                a = (I2CDeviceId)i;
                b = (I2CDeviceId)j;
                return true;
            }
        }
    }
    return false;
}

const char* I2CRouter::keyOf(I2CDeviceId dev) {
    switch (dev) {
        case I2C_DEV_BNO085:   return "route_bno085";
        case I2C_DEV_INA3221:  return "route_ina3221";
        case I2C_DEV_INA226_2: return "route_ina226_2";
        case I2C_DEV_INA226_3: return "route_ina226_3";
        case I2C_DEV_MS5803:   return "route_ms5803";
        case I2C_DEV_PCA9685:  return "route_pca9685";
        default:               return "route_unknown";
    }
}

const char* I2CRouter::addrKeyOf(I2CDeviceId dev) {
    switch (dev) {
        case I2C_DEV_BNO085:   return "addr_bno085";
        case I2C_DEV_INA3221:  return "addr_ina3221";
        case I2C_DEV_INA226_2: return "addr_ina226_2";
        case I2C_DEV_INA226_3: return "addr_ina226_3";
        case I2C_DEV_MS5803:   return "addr_ms5803";
        case I2C_DEV_PCA9685:  return "addr_pca9685";
        default:               return "addr_unknown";
    }
}

const char* I2CRouter::nameOf(I2CDeviceId dev) {
    switch (dev) {
        case I2C_DEV_BNO085:   return "BNO085 (IMU 9-DOF)";
        case I2C_DEV_INA3221:  return "INA3221 (V_BAT + 2 ACS770)";
        case I2C_DEV_INA226_2: return "INA226 #2 (Électronique)";
        case I2C_DEV_INA226_3: return "INA226 #3 (Projecteur LED)";
        case I2C_DEV_MS5803:   return "MS5803/MS5837 (Pression)";
        case I2C_DEV_PCA9685:  return "PCA9685 (PWM 16 Ch)";
        default:               return "?";
    }
}

const char* I2CRouter::shortNameOf(I2CDeviceId dev) {
    switch (dev) {
        case I2C_DEV_BNO085:   return "BNO085";
        case I2C_DEV_INA3221:  return "INA3221";
        case I2C_DEV_INA226_2: return "INA226 #2";
        case I2C_DEV_INA226_3: return "INA226 #3";
        case I2C_DEV_MS5803:   return "MS5803/MS5837";
        case I2C_DEV_PCA9685:  return "PCA9685";
        default:               return "?";
    }
}
