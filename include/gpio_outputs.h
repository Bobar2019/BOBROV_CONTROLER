/**
 * @file gpio_outputs.h
 * @brief Pilotage des 4 sorties tout-ou-rien (GPIO ON/OFF) avec assignation NVS.
 *
 * Les broches par défaut (GPIO 15, 16, 17, 18) sont configurables depuis
 * l'interface Web et persistées en NVS (clés « gpio_p1 »..« gpio_p4 »,
 * namespace « config ») ; elles ne sont appliquées qu'au démarrage. Au boot,
 * les sorties sont initialisées en sortie à l'état BAS (OFF). Le masque
 * gpio_cmd (4 bits) reçu dans la trame descendante est appliqué par la tâche
 * de contrôle ; le watchdog série et l'E-Stop forcent les 4 sorties à OFF.
 *
 * @author Didier Dero
 * @version 1.0.0
 */

#ifndef GPIO_OUTPUTS_H
#define GPIO_OUTPUTS_H

#include <Arduino.h>
#include "config.h"

/**
 * @brief Classe de gestion des 4 sorties numériques ON/OFF.
 *
 * L'état réel est relu après chaque écriture (mesure du niveau pad) et
 * reflété dans la trame montante (gpio_state). Les accès proviennent de
 * tâches FreeRTOS distinctes (contrôle, réception série, Web) : les écritures
 * et relectures des 4 broches sont atomisées par une section critique.
 */
class GPIOOutputs {
public:
    GPIOOutputs();

    /**
     * @brief Charge les broches depuis la NVS puis les initialise en sortie à OFF.
     *
     * Toute valeur NVS invalide (> 48) est remplacée par la broche par défaut
     * correspondante (GPIO 15/16/17/18).
     */
    void begin();

    /**
     * @brief Applique le masque de commande reçu du RPi 5 (bit 0 = sortie 1).
     * @param mask Masque 4 bits (bits 4-7 ignorés).
     */
    void applyCommand(uint8_t mask);

    /** @brief Force les 4 sorties à OFF (failsafe watchdog série / E-Stop) */
    void allOff();

    /** @brief Retourne l'état réel courant (relecture GPIO) sous forme de masque 4 bits */
    uint8_t getState() const;

    /** @brief Copie les broches actuellement assignées (4 entrées) */
    void getPins(uint8_t* out) const;

    /**
     * @brief Persiste une nouvelle assignation de broches en NVS (vérifiée par relecture).
     *
     * Les broches ne sont appliquées qu'au redémarrage suivant (comme les
     * broches I2C) : init OFF au boot, puis pilotage par la trame descendante.
     *
     * @param pins Tableau de GPIO_NUM_OUTPUTS broches (0-48, uniques).
     * @return true si l'écriture NVS est confirmée par relecture.
     */
    bool savePins(const uint8_t* pins);

private:
    uint8_t           _pins[GPIO_NUM_OUTPUTS];  ///< Broches assignées (chargées de la NVS)
    volatile uint8_t  _state;                   ///< Dernier état réel relu (masque 4 bits)
    portMUX_TYPE      _mux;                     ///< Section critique (accès multi-tâches)

    /** @brief Écrit un masque sur les 4 broches (sans relecture) */
    void _writeMask(uint8_t mask);

    /** @brief Relit le niveau réel des 4 broches → masque */
    uint8_t _readMask() const;
};

/* Instance globale extern */
extern GPIOOutputs g_gpio;

#endif /* GPIO_OUTPUTS_H */
