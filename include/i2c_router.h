/**
 * @file i2c_router.h
 * @brief Routage dynamique Plug & Play des périphériques I2C (bus + adresse).
 *
 * La topologie « quel capteur vit sur quel bus, à quelle adresse » n'est plus
 * codée en dur : elle est persistée en NVS (namespace « config » — clés
 * « route_* » pour le bus, « addr_* » pour l'adresse 7 bits) et modifiable
 * depuis la page Paramètres de l'interface Web (carte Scanner — « Routage
 * I2C Plug & Play »), chaque adresse étant choisie parmi celles découvertes
 * par le scan I2C. Au boot, SensorDriver::begin() charge la carte de routage
 * AVANT toute initialisation de capteur, puis chaque composant reçoit le
 * pointeur TwoWire de SON bus (&Wire ou &Wire1) et l'adresse routée.
 *
 * Routage par défaut (premier démarrage / NVS vide) = topologie de référence
 * historique, pour un comportement strictement identique aux versions
 * antérieures :
 *   - Bus n°1 (Wire,  GPIO 10/11) : BNO085 (0x4A) + INA3221 (0x40) + 2× INA226 (0x44, 0x45)
 *   - Bus n°2 (Wire1, GPIO 6/7)   : MS5803/MS5837 (0x76) + PCA9685 (0x40)
 *
 * RÈGLE DE SÛRETÉ STRUCTURELLE : le bus du PCA9685 ne doit JAMAIS être
 * bit-bangé ni réinitialisé à chaud (les sorties PWM 100 Hz de la tâche de
 * contrôle y vivent) : pwmBus() expose ce bus protégé à toutes les routines
 * de récupération. L'API de sauvegarde refuse par ailleurs tout routage
 * plaçant deux modules sur le même bus à la même adresse (cas historique :
 * INA3221 ↔ PCA9685, tous deux 0x40 par défaut).
 *
 * @author Didier Dero
 * @version 1.3.0
 * @date Septembre 2026
 */

#ifndef I2C_ROUTER_H
#define I2C_ROUTER_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

/**
 * @brief Identifiants des périphériques routables.
 *
 * L'ordre des valeurs fixe l'API REST /api/i2c/routing (clés « bno085 »,
 * « ina3221 », « ina226_2 », « ina226_3 », « ms5803 », « pca9685 ») — ne
 * jamais réordonner : la compatibilité frontend/firmware en dépend.
 */
enum I2CDeviceId {
    I2C_DEV_BNO085 = 0,   ///< IMU BNO085 (défaut 0x4A) — cap magnétique + attitude
    I2C_DEV_INA3221,      ///< Triple wattmètre INA3221 (défaut 0x40 — V_BAT + 2× ACS770)
    I2C_DEV_INA226_2,     ///< INA226 n°2 — batterie 2, électronique de commande (défaut 0x44)
    I2C_DEV_INA226_3,     ///< INA226 n°3 — batterie 2, projecteur LED (défaut 0x45)
    I2C_DEV_MS5803,       ///< Baromètre MS5803-30BA / MS5837 (défaut 0x76)
    I2C_DEV_PCA9685,      ///< Driver PWM 16 canaux PCA9685 (défaut 0x40)
    I2C_DEV_COUNT         ///< borne supérieure (pas un périphérique)
};

/**
 * @brief Carte de routage périphérique → (bus I2C, adresse 7 bits), NVS.
 *
 * Chargée une seule fois au boot (SensorDriver::begin(), avant Wire/Wire1
 * et avant toute sonde de présence) puis considérée en lecture seule pour
 * tout le cycle de vie du firmware : un changement de routage passe par
 * l'API Web → NVS → redémarrage différé (même discipline que les broches
 * SDA/SCL, jamais de re-routage à chaud).
 */
class I2CRouter {
public:
    I2CRouter();

    /**
     * @brief Charge la carte de routage (bus + adresses) depuis la NVS.
     *
     * Clés absentes ou invalides (bus ni 1 ni 2, adresse hors 0x01–0x7F)
     * → topologie par défaut (config.h). Journalise la carte résultante et
     * tout conflit (deux modules même bus + même adresse).
     */
    void load();

    /**
     * @brief Écrit la carte complète (12 clés) en NVS, vérifiée par relecture.
     *
     * Refuse (retourne false sans écrire) si deux modules aboutissent sur le
     * même bus à la même adresse — cas historique : INA3221 et PCA9685,
     * tous deux 0x40 par défaut.
     *
     * @return true si les 12 clés sont relues avec leurs nouvelles valeurs.
     */
    bool save();

    /** @brief Numéro de bus routé du périphérique (1 = Wire, 2 = Wire1). */
    uint8_t busOf(I2CDeviceId dev) const { return _bus[dev]; }

    /** @brief Adresse I2C (7 bits) routée du périphérique. */
    uint8_t addrOf(I2CDeviceId dev) const { return _addr[dev]; }

    /** @brief Instance TwoWire du bus routé (&Wire ou &Wire1). */
    TwoWire* wireOf(I2CDeviceId dev) const;

    /** @brief Instance TwoWire d'un NUMÉRO de bus (1 = &Wire, 2 = &Wire1). */
    TwoWire* wireOfBus(uint8_t bus) const { return (bus == 2) ? &Wire1 : &Wire; }

    /** @brief Modifie le bus d'un périphérique en mémoire (1 ou 2). */
    bool setBusOf(I2CDeviceId dev, uint8_t bus);

    /** @brief Modifie l'adresse I2C d'un périphérique en mémoire (0x01–0x7F). */
    bool setAddrOf(I2CDeviceId dev, uint8_t addr);

    /** @brief Clé NVS du BUS routé (namespace « config », « route_* »). */
    static const char* keyOf(I2CDeviceId dev);

    /** @brief Clé NVS de l'ADRESSE routée (namespace « config », « addr_* »). */
    static const char* addrKeyOf(I2CDeviceId dev);

    /** @brief Nom affichable du périphérique (interface Web / logs). */
    static const char* nameOf(I2CDeviceId dev);

    /** @brief Nom COURT du périphérique (scan I2C, messages de conflit). */
    static const char* shortNameOf(I2CDeviceId dev);

    /** @brief Bus hébergeant le PCA9685 — JAMAIS bit-bangé ni reseté à chaud. */
    uint8_t pwmBus() const { return _bus[I2C_DEV_PCA9685]; }

    /** @brief true si le bus donné est celui du PCA9685 (bus PWM protégé). */
    bool isPwmBus(uint8_t bus) const { return bus == _bus[I2C_DEV_PCA9685]; }

    /** @brief L'autre bus que celui donné (1 ↔ 2). */
    static uint8_t otherBus(uint8_t bus) { return (bus == 1) ? 2 : 1; }

    /**
     * @brief true si deux modules partagent le MÊME bus ET la MÊME adresse.
     *
     * Généralisation du conflit historique 0x40 INA3221 ↔ PCA9685 : toute
     * paire (bus, adresse) dupliquée rendrait les deux modules indistinguables.
     */
    bool hasAddressConflict() const;

    /**
     * @brief Première paire en conflit (même bus, même adresse).
     *
     * @param a Premier module de la paire (renseigné si conflit)
     * @param b Second module de la paire (renseigné si conflit)
     * @return true si un conflit existe, false sinon.
     */
    bool firstConflict(I2CDeviceId& a, I2CDeviceId& b) const;

private:
    /** @brief Bus routé par périphérique (1 = Wire, 2 = Wire1). */
    uint8_t _bus[I2C_DEV_COUNT];

    /** @brief Adresse I2C routée par périphérique (7 bits, 0x01–0x7F). */
    uint8_t _addr[I2C_DEV_COUNT];

    /** @brief Restaure la topologie de référence (bus + adresses de config.h). */
    void _setDefaults();
};

/* Instance globale (définie dans i2c_router.cpp) — chargée par
 * SensorDriver::begin() avant toute initialisation de capteur. */
extern I2CRouter g_i2cRouter;

#endif /* I2C_ROUTER_H */
