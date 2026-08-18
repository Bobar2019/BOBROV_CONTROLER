/**
 * @file serial_comm.cpp
 * @brief Implémentation de la communication série (USB-CDC ou UART GPIO) avec protocole binaire.
 *
 * Machine à états non bloquante pour le parsing des trames descendantes,
 * watchdog logiciel avec failsafe et émission périodique de télémétrie.
 * Le routage physique (Serial ou Serial1) est sélectionné à l'initialisation.
 *
 * @author Didier Dero
 * @version 1.1.0
 */

#include "serial_comm.h"
#include "pwm_controller.h"
#include "config.h"
#include "protocol.h"
#include <Preferences.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Instance globale du gestionnaire série */
SerialComm g_serial;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

/**
 * @brief Constructeur : initialise la machine à états et les variables.
 */
SerialComm::SerialComm()
    : _commStream(nullptr)
    , _interfaceType(COMM_INTERFACE_USB_CDC)
    , _parseState(PARSE_IDLE)
    , _rxIdx(0)
    , _newFrame(false)
    , _mutexRx(nullptr)
    , _mutexTx(nullptr)
    , _lastFrameTime(0)
    , _watchdogTimeout(SERIAL_TIMEOUT_DEFAULT_MS)
    , _watchdogTriggered(false)
{
    memset(&_lastData, 0, sizeof(_lastData));
    memset(_rxBuf, 0, sizeof(_rxBuf));
    memset(_txBuf, 0, sizeof(_txBuf));
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

/**
 * @brief Initialise l'interface série sélectionnée et lance les tâches FreeRTOS.
 *
 * Selon le type d'interface demandé :
 * - COMM_INTERFACE_USB_CDC  : route le protocole binaire sur le port USB natif (Serial).
 *   Le debug moniteur partage alors le même port (trames séparées par en-têtes).
 * - COMM_INTERFACE_UART_GPIO : initialise Serial1 à 921 600 bauds sur GPIO 1 (RX) / GPIO 2 (TX)
 *   et y dirige le protocole binaire. Serial (USB-CDC) reste dédié au debug.
 *
 * @param type Interface physique à utiliser.
 */
void SerialComm::begin(SerialInterfaceType type) {
    /* Enregistrement du type d'interface */
    _interfaceType = type;

    /* Création des mutex de synchronisation */
    _mutexRx = xSemaphoreCreateMutex();
    _mutexTx = xSemaphoreCreateMutex();

    switch (type) {
        case COMM_INTERFACE_UART_GPIO:
            /* ---- Mode UART matériel (GPIO 1 RX / GPIO 2 TX) ---- */
            Serial1.begin(SERIAL_BAUD_RATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
            delay(100); /* Délai de stabilisation UART */
            _commStream = &Serial1;
            Serial.println("[SERIAL] UART1 RPi5 initialisé à " + String(SERIAL_BAUD_RATE) +
                           " bauds (RX=GPIO" + String(UART1_RX_PIN) + ", TX=GPIO" + String(UART1_TX_PIN) + ")");
            break;

        case COMM_INTERFACE_USB_CDC:
        default:
            /* ---- Mode USB-CDC natif (partage debug + protocole) ---- */
            /* Serial (USB-CDC) est déjà initialisé dans setup() pour le debug.
             * On le réutilise pour le protocole binaire RPi5.
             * Le RPi5 verra /dev/ttyACM0 avec les trames binaires. */
            _commStream = &Serial;
            Serial.println("[SERIAL] USB-CDC natif sélectionné pour protocole RPi5 (/dev/ttyACM0)");
            break;
    }

    /* Persistance du choix en NVS */
    {
        Preferences prefs;
        prefs.begin("config", false);
        prefs.putUChar("comm_iface", (uint8_t)type);
        prefs.end();
    }

    /* Lancement de la tâche de réception série sur Core 1 (temps réel) */
    xTaskCreatePinnedToCore(
        _taskRxEntry,           /* Fonction de la tâche */
        "SerialRx",             /* Nom */
        STACK_SIZE_SERIAL,      /* Taille de stack */
        this,                   /* Paramètre (pointeur this) */
        PRIORITY_SERIAL,        /* Priorité */
        nullptr,                /* Handle (non utilisé) */
        CORE_RT                 /* Core 1 */
    );

    /* Lancement de la tâche d'émission télémétrie sur Core 0 (Wi-Fi) */
    xTaskCreatePinnedToCore(
        _taskTxEntry,
        "SerialTx",
        STACK_SIZE_SERIAL,
        this,
        PRIORITY_SERIAL,
        nullptr,
        CORE_WIFI
    );
}

/* =========================================================================
 * ACCESSEURS INTERFACE
 * ========================================================================= */

/**
 * @brief Retourne le type d'interface actuellement active.
 */
SerialInterfaceType SerialComm::getActiveInterface() const {
    return _interfaceType;
}

/**
 * @brief Retourne un libellé humain de l'interface active pour affichage WebSocket.
 */
const char* SerialComm::getInterfaceLabel() const {
    return (_interfaceType == COMM_INTERFACE_UART_GPIO)
        ? "UART (GPIO 1/2)"
        : "USB-CDC";
}

/* =========================================================================
 * ACCÈS AUX DONNÉES
 * ========================================================================= */

/**
 * @brief Vérifie si une nouvelle trame descendante valide est disponible.
 * @return true si une trame non lue est en attente.
 */
bool SerialComm::hasNewFrame() const {
    return _newFrame;
}

/**
 * @brief Copie les dernières données descendantes reçues (thread-safe).
 *
 * Réinitialise le flag _newFrame après lecture pour éviter
 * le traitement en double de la même trame.
 *
 * @param out Structure de sortie remplie avec les données décodées.
 */
void SerialComm::getDownlinkData(DownlinkData& out) {
    if (xSemaphoreTake(_mutexRx, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&out, &_lastData, sizeof(DownlinkData));
        _newFrame = false;  /* Acquitter le flag après lecture */
        xSemaphoreGive(_mutexRx);
    }
}

/**
 * @brief Définit le timeout du watchdog logiciel.
 * @param ms Timeout en millisecondes (minimum 100 ms).
 */
void SerialComm::setWatchdogTimeout(uint32_t ms) {
    if (ms < 100) ms = 100;   /* Minimum de sécurité */
    _watchdogTimeout = ms;
    Serial.println("[SERIAL] Watchdog timeout réglé à " + String(ms) + " ms");
}

/**
 * @brief Retourne le timeout watchdog courant en ms.
 */
uint32_t SerialComm::getWatchdogTimeout() const {
    return _watchdogTimeout;
}

/**
 * @brief Indique si le watchdog a été déclenché (perte de liaison série).
 */
bool SerialComm::isWatchdogTriggered() const {
    return _watchdogTriggered;
}

/**
 * @brief Retourne le timestamp (millis) de la dernière trame valide.
 */
unsigned long SerialComm::getLastFrameTime() const {
    return _lastFrameTime;
}

/**
 * @brief Met à jour le buffer de télémétrie pour l'émission montante.
 *
 * Copie le buffer UplinkFrame_t dans le buffer d'émission interne
 * sous mutex pour garantir la cohérence lors de l'envoi.
 *
 * @param uplink_buffer Pointeur vers la trame UplinkFrame_t sérialisée.
 * @param len           Taille du buffer (doit être UL_FRAME_SIZE).
 */
void SerialComm::updateTelemetry(const uint8_t* uplink_buffer, size_t len) {
    if (len != UL_FRAME_SIZE || !uplink_buffer) return;
    if (xSemaphoreTake(_mutexTx, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(_txBuf, uplink_buffer, UL_FRAME_SIZE);
        xSemaphoreGive(_mutexTx);
    }
}

/* =========================================================================
 * MACHINE À ÉTATS DE PARSING
 * ========================================================================= */

/**
 * @brief Traite un octet reçu sur la liaison série.
 *
 * Machine à états à 3 niveaux :
 * - PARSE_IDLE : attend l'octet d'en-tête 0xAA
 * - PARSE_HEADER2 : attend le second octet 0x55
 * - PARSE_PAYLOAD : accumule les octets restants (type + payload + CRC)
 *
 * En cas d'erreur (mauvais en-tête, CRC invalide), retour à PARSE_IDLE.
 *
 * @param b Octet reçu.
 */
void SerialComm::_processByte(uint8_t b) {
    switch (_parseState) {
        case PARSE_IDLE:
            /* Attente du premier octet d'en-tête 0xAA */
            if (b == DL_HEADER_1) {
                _rxBuf[0] = b;
                _rxIdx = 1;
                _parseState = PARSE_HEADER2;
            }
            break;

        case PARSE_HEADER2:
            /* Vérification du second octet d'en-tête 0x55 */
            if (b == DL_HEADER_2) {
                _rxBuf[1] = b;
                _rxIdx = 2;
                _parseState = PARSE_PAYLOAD;
            } else {
                /* Séquence invalide, retour à l'état initial */
                _parseState = PARSE_IDLE;
            }
            break;

        case PARSE_PAYLOAD:
            /* Accumulation des octets de payload + CRC */
            _rxBuf[_rxIdx++] = b;
            if (_rxIdx >= DL_FRAME_SIZE) {
                /* Trame complète reçue, validation CRC */
                if (_validateFrame()) {
                    _newFrame = true;
                }
                /* Retour à l'état initial quel que soit le résultat */
                _parseState = PARSE_IDLE;
                _rxIdx = 0;
            }
            break;
    }
}

/**
 * @brief Valide la trame reçue par vérification CRC16 et extraction des données.
 *
 * Vérifie le CRC16-CCITT sur l'ensemble de la trame et, si valide,
 * extrait les champs mode, arm_state et les 16 canaux PWM.
 *
 * @return true si la trame est valide et les données extraites.
 */
bool SerialComm::_validateFrame() {
    /* Vérification du CRC16 sur la trame complète */
    if (!crc16_verify(_rxBuf, DL_FRAME_SIZE)) {
        Serial.println("[SERIAL] CRC16 invalide sur trame descendante");
        return false;
    }

    /* Extraction des données sous mutex */
    if (xSemaphoreTake(_mutexRx, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* Cast de la structure packed sur le buffer reçu */
        DownlinkFrame_t* frame = reinterpret_cast<DownlinkFrame_t*>(_rxBuf);

        _lastData.mode      = frame->mode;
        _lastData.arm_state = frame->arm_state;

        /* Copie des 16 canaux PWM (déjà en little-endian sur ESP32) */
        for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
            _lastData.pwm[i] = frame->pwm[i];
        }

        /* Mise à jour du timestamp pour le watchdog */
        _lastFrameTime = millis();
        _watchdogTriggered = false;

        xSemaphoreGive(_mutexRx);
    }

    return true;
}

/* =========================================================================
 * TÂCHES FREERTOS
 * ========================================================================= */

/**
 * @brief Point d'entrée statique de la tâche de réception série.
 */
void SerialComm::_taskRxEntry(void* param) {
    static_cast<SerialComm*>(param)->_taskRxLoop();
}

/**
 * @brief Boucle de la tâche de réception série RPi5 (Core 1).
 *
 * Lit les octets disponibles sur l'interface active (_commStream) et les passe
 * à la machine à états de parsing. Vérifie également le timeout watchdog.
 * Les messages de debug sont envoyés sur Serial (USB-CDC).
 */
void SerialComm::_taskRxLoop() {
    for (;;) {
        /* Lecture non bloquante de tous les octets disponibles sur l'interface active */
        while (_commStream && _commStream->available() > 0) {
            uint8_t b = _commStream->read();
            _processByte(b);
        }

        /* Vérification du watchdog logiciel */
        unsigned long now = millis();
        if ((now - _lastFrameTime) > _watchdogTimeout && _lastFrameTime > 0) {
            if (!_watchdogTriggered) {
                _watchdogTriggered = true;
                Serial.println("[SERIAL] Watchdog déclenché ! Failsafe activé.");
                /* Force tous les propulseurs au neutre immédiatement */
                g_pwm.setAllNeutral();
            }
        }

        /* Pause de 1 ms avant la prochaine itération */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Point d'entrée statique de la tâche d'émission série.
 */
void SerialComm::_taskTxEntry(void* param) {
    static_cast<SerialComm*>(param)->_taskTxLoop();
}

/**
 * @brief Boucle de la tâche d'émission de télémétrie (Core 0).
 *
 * Envoie périodiquement la trame montante (télémétrie) vers le RPi 5
 * via l'interface active (_commStream) à la fréquence définie par TASK_SERIAL_TX_PERIOD_MS.
 */
void SerialComm::_taskTxLoop() {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        /* Copie thread-safe du buffer de télémétrie */
        uint8_t localBuf[UL_FRAME_SIZE];
        if (xSemaphoreTake(_mutexTx, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(localBuf, _txBuf, UL_FRAME_SIZE);
            xSemaphoreGive(_mutexTx);

            /* Émission de la trame binaire sur l'interface active (vers RPi5) */
            if (_commStream) {
                _commStream->write(localBuf, UL_FRAME_SIZE);
            }
        }

        /* Attente périodique précise via vTaskDelayUntil */
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SERIAL_TX_PERIOD_MS));
    }
}
