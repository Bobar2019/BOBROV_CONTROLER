/**
 * @file serial_comm.h
 * @brief Gestionnaire de communication série avec protocole binaire (USB-CDC ou UART GPIO).
 *
 * Abstraction de l'interface physique via pointeur Stream* permettant de router
 * dynamiquement les trames binaires vers le port USB natif (Serial) ou vers
 * l'UART matériel (Serial1 sur GPIO 1/2).
 *
 * Machine à états non bloquante pour parser les trames descendantes,
 * watchdog failsafe et émission périodique de la télémétrie montante.
 *
 * @author Didier Dero
 * @version 1.1.0
 */

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <Arduino.h>
#include "protocol.h"
#include "config.h"

/** @brief Données reçues via la trame descendante (consignes RPi → ESP32) */
struct DownlinkData {
    uint16_t pwm[NUM_PWM_CHANNELS]; ///< Consignes PWM en µs
    uint8_t  mode;                  ///< Mode opérationnel
    uint8_t  arm_state;             ///< État d'armement
    uint8_t  gpio_cmd;              ///< Masque 4 bits des sorties ON/OFF (bit 0 = sortie 1)
};

/**
 * @brief Classe de gestion de la liaison série avec machine à états.
 *
 * Parse les trames descendantes de manière non bloquante et émet
 * la télémétrie montante à fréquence configurable. Le routage physique
 * (USB-CDC ou UART GPIO) est sélectionné via begin(). Inclut un watchdog
 * logiciel qui déclenche le failsafe après timeout configurable.
 */
class SerialComm {
public:
    SerialComm();

    /**
     * @brief Initialise l'interface série sélectionnée et lance les tâches FreeRTOS.
     * @param type Interface physique : COMM_INTERFACE_USB_CDC ou COMM_INTERFACE_UART_GPIO
     */
    void begin(SerialInterfaceType type = COMM_INTERFACE_USB_CDC);

    /** @brief Retourne le type d'interface actuellement active */
    SerialInterfaceType getActiveInterface() const;

    /** @brief Retourne un libellé humain de l'interface active (pour WebSocket) */
    const char* getInterfaceLabel() const;

    /** @brief Indique si une nouvelle trame descendante valide a été reçue */
    bool hasNewFrame() const;

    /** @brief Copie thread-safe des dernières données descendantes reçues */
    void getDownlinkData(DownlinkData& out);

    /** @brief Définit le timeout watchdog en millisecondes */
    void setWatchdogTimeout(uint32_t ms);

    /** @brief Retourne le timeout watchdog courant en millisecondes */
    uint32_t getWatchdogTimeout() const;

    /** @brief Indique si le watchdog a été déclenché (timeout dépassé) */
    bool isWatchdogTriggered() const;

    /** @brief Retourne le timestamp de la dernière trame valide reçue */
    unsigned long getLastFrameTime() const;

    /** @brief Met à jour les données de télémétrie pour l'émission montante */
    void updateTelemetry(const uint8_t* uplink_buffer, size_t len);

private:
    /* -- Abstraction de l'interface physique -- */
    Stream*             _commStream;    ///< Pointeur vers le flux actif (Serial ou Serial1)
    SerialInterfaceType _interfaceType; ///< Type d'interface sélectionnée

    /* -- États de la machine à états de parsing -- */
    enum ParseState : uint8_t {
        PARSE_IDLE,           ///< Attente du premier octet d'en-tête
        PARSE_HEADER2,        ///< Attente du second octet d'en-tête
        PARSE_PAYLOAD,        ///< Lecture du payload restant
    };

    /* -- Variables de la machine à états -- */
    ParseState      _parseState;                    ///< État courant du parser
    uint8_t         _rxBuf[DL_FRAME_SIZE];          ///< Buffer de réception
    uint16_t        _rxIdx;                         ///< Index d'écriture dans le buffer
    volatile bool   _newFrame;                      ///< Flag nouvelle trame valide

    /* -- Données partagées -- */
    DownlinkData    _lastData;                      ///< Dernière trame décodée
    uint8_t         _txBuf[UL_FRAME_SIZE];          ///< Buffer d'émission
    SemaphoreHandle_t _mutexRx;                     ///< Mutex données descendantes
    SemaphoreHandle_t _mutexTx;                     ///< Mutex buffer d'émission

    /* -- Watchdog logiciel -- */
    volatile unsigned long _lastFrameTime;          ///< Timestamp dernière trame
    volatile uint32_t _watchdogTimeout;             ///< Timeout en ms
    volatile bool   _watchdogTriggered;             ///< Flag watchdog déclenché

    /* -- Traitement -- */
    void _processByte(uint8_t b);                   ///< Traitement d'un octet reçu
    bool _validateFrame();                          ///< Validation CRC de la trame

    /* -- Tâches FreeRTOS -- */
    static void _taskRxEntry(void* param);
    static void _taskTxEntry(void* param);
    void _taskRxLoop();
    void _taskTxLoop();
};

/* Instance globale extern */
extern SerialComm g_serial;

#endif /* SERIAL_COMM_H */
