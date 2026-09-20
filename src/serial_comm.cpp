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
#include "gpio_outputs.h"
#include "config.h"
#include "protocol.h"
#include <Preferences.h>
#include <ArduinoJson.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Instance globale du gestionnaire série */
SerialComm g_serial;

/* =========================================================================
 * HELPERS DE SYNCHRONISATION DE CONFIGURATION (protocole v1.5.0)
 * ========================================================================= */

/** @brief Taille max du JSON des noms en NVS (alignée sur web_server.cpp) */
static constexpr size_t CFG_NAMES_JSON_MAX = 1024;

/**
 * @brief Empaquette une trame de configuration des noms (27 octets).
 *
 * @param[out] out  Buffer de CFG_FRAME_SIZE octets à remplir.
 * @param[in]  type UL_FRAME_TYPE_CFG_PWM (0x03) ou UL_FRAME_TYPE_CFG_GPIO (0x04).
 * @param[in]  id   Numéro de canal PWM (10-15) ou de sortie TOR (1-4).
 * @param[in]  name Nom UTF-8 (NVS, déjà nettoyé/borné par le serveur web) ou nullptr.
 */
static void _fillNameFrame(uint8_t* out, uint8_t type, uint8_t id, const char* name) {
    ConfigFrame_t* f = reinterpret_cast<ConfigFrame_t*>(out);
    f->header1 = UL_HEADER_1;
    f->header2 = UL_HEADER_2;
    f->type    = type;
    f->id      = id;

    size_t len = (name != nullptr) ? strlen(name) : 0;
    if (len > CFG_NAME_MAX_LEN) len = CFG_NAME_MAX_LEN;   /* garde-fou */
    f->name_len = (uint8_t)len;
    memset(f->name, 0, CFG_NAME_MAX_LEN);
    if (len > 0) {
        memcpy(f->name, name, len);
    }

    crc16_fill(out, CFG_FRAME_SIZE);
}

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
    , _cfgSyncPending(false)
    , _cfgBurstCount(0)
    , _cfgBurstIdx(0)
{
    memset(&_lastData, 0, sizeof(_lastData));
    memset(_rxBuf, 0, sizeof(_rxBuf));
    memset(_txBuf, 0, sizeof(_txBuf));
    memset(_cfgBurst, 0, sizeof(_cfgBurst));
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
 * SYNCHRONISATION DE CONFIGURATION MATÉRIELLE (protocole v1.5.0)
 * =========================================================================
 *
 * Source unique de vérité : la NVS de l'ESP32 (namespace « config », clé
 * « names » — même format que le serveur web : {"ch":[…16…],"gpio":[…4…]}).
 * Les noms sont transmis au RPi 5 par trames 0x03 (canaux PWM 10-15, id =
 * numéro de canal) et 0x04 (sorties TOR, id = numéro de sortie 1-4) — une
 * trame de 27 octets par sortie, émise à raison d'une par cycle de 10 ms. */

/**
 * @brief Demande la (re)synchronisation de la configuration vers le RPi 5.
 *
 * Non bloquant : positionne uniquement un drapeau consommé par la tâche
 * SerialTx, qui lit la NVS, construit la rafale et l'émet — la télémétrie
 * 100 Hz reste totalement intacte.
 */
void SerialComm::sendHardwareConfigToRPi() {
    _cfgSyncPending = true;
}

/**
 * @brief Construit la rafale de trames de configuration depuis la NVS.
 *
 * Exécutée par la tâche SerialTx uniquement. Un JSON absent ou corrompu
 * produit des trames à nom vide (name_len = 0 = « nom par défaut » côté
 * RPi 5) : la synchronisation reste toujours possible.
 */
void SerialComm::_loadHardwareConfig() {
    static char json[CFG_NAMES_JSON_MAX + 1];

    /* Lecture NVS (même namespace/clé que le serveur web) */
    Preferences prefs;
    size_t len = 0;
    if (prefs.begin("config", true)) {
        len = prefs.getString("names", json, sizeof(json));
        prefs.end();
    }

    JsonDocument doc;
    const bool ok = (len > 0) && !deserializeJson(doc, json, len);

    uint8_t n = 0;
    /* Canaux PWM auxiliaires 10-15 → type 0x03, id = numéro de canal */
    for (uint8_t ch = PWM_TYPE_FIRST_CHANNEL; ch < NUM_PWM_CHANNELS; ch++) {
        const char* nm = ok ? (doc["ch"][ch] | "") : "";
        _fillNameFrame(_cfgBurst[n++], UL_FRAME_TYPE_CFG_PWM, ch, nm);
    }
    /* 4 sorties tout-ou-rien → type 0x04, id = numéro de sortie (1-4) */
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        const char* nm = ok ? (doc["gpio"][i] | "") : "";
        _fillNameFrame(_cfgBurst[n++], UL_FRAME_TYPE_CFG_GPIO, (uint8_t)(i + 1), nm);
    }

    _cfgBurstCount = n;
    _cfgBurstIdx   = 0;
    Serial.println("[SERIAL] Synchronisation configuration → RPi5 : " +
                   String(_cfgBurstCount) + " trames (noms NVS)");
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
                /* Trame complète reçue : validation CRC + dispatch par type.
                 * Seules les commandes de pilotage (0x01) déclenchent le
                 * traitement par la tâche Control (_newFrame). */
                if (_validateFrame() == FRAME_COMMAND) {
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
 * @brief Valide la trame reçue (CRC16) et la distribue selon son type.
 *
 * - 0x01 (commande) : extraction des champs mode, arm_state, gpio_cmd et des
 *   16 canaux PWM, réarmement du watchdog. Si c'est la première commande
 *   depuis le boot ou depuis un déclenchement du watchdog (liaison qui
 *   reprend), une synchronisation de la configuration est demandée.
 * - 0xFF (requête de synchronisation) : réarme le watchdog (preuve de
 *   liaison) et déclenche le renvoi des noms (trames 0x03/0x04). Aucune
 *   consigne n'est modifiée.
 * - Autres types : ignorés.
 *
 * @return FRAME_COMMAND si consignes extraites, FRAME_SYNCREQ sur requête de
 *         synchronisation, FRAME_IGNORED si type inconnu, FRAME_INVALID si CRC KO.
 */
SerialComm::FrameResult SerialComm::_validateFrame() {
    /* Vérification du CRC16 sur la trame complète */
    if (!crc16_verify(_rxBuf, DL_FRAME_SIZE)) {
        Serial.println("[SERIAL] CRC16 invalide sur trame descendante");
        return FRAME_INVALID;
    }

    const DownlinkFrame_t* frame = reinterpret_cast<const DownlinkFrame_t*>(_rxBuf);

    /* ---- Requête de synchronisation (0xFF) : config renvoyée, pas de consigne ---- */
    if (frame->type == DL_FRAME_TYPE_SYNC) {
        _lastFrameTime = millis();   /* preuve de liaison : réarme le watchdog */
        _watchdogTriggered = false;
        sendHardwareConfigToRPi();
        Serial.println("[SERIAL] Requête de synchronisation 0xFF reçue — configuration renvoyée");
        return FRAME_SYNCREQ;
    }

    /* ---- Type inconnu : ignoré ---- */
    if (frame->type != DL_FRAME_TYPE) {
        Serial.println("[SERIAL] Trame descendante de type inconnu 0x" +
                       String(frame->type, HEX) + " — ignorée");
        return FRAME_IGNORED;
    }

    /* ---- Commande de pilotage (0x01) ----
     * Première trame depuis le boot ou depuis une perte de liaison : le
     * RPi 5 vient de (re)démarrer ou la liaison reprend — resynchroniser. */
    const bool resyncNeeded = (_lastFrameTime == 0) || _watchdogTriggered;

    /* Extraction des données sous mutex */
    if (xSemaphoreTake(_mutexRx, pdMS_TO_TICKS(10)) == pdTRUE) {
        _lastData.mode      = frame->mode;
        _lastData.arm_state = frame->arm_state;
        _lastData.gpio_cmd  = frame->gpio_cmd;

        /* Copie des 16 canaux PWM (déjà en little-endian sur ESP32) */
        for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
            _lastData.pwm[i] = frame->pwm[i];
        }

        /* Mise à jour du timestamp pour le watchdog */
        _lastFrameTime = millis();
        _watchdogTriggered = false;

        xSemaphoreGive(_mutexRx);
    }

    if (resyncNeeded) {
        sendHardwareConfigToRPi();
    }

    return FRAME_COMMAND;
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
                /* Sorties tout-ou-rien forcées à OFF par sécurité */
                g_gpio.allOff();
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
 * @brief Boucle de la tâche d'émission (Core 0) : télémétrie + configuration.
 *
 * Envoie périodiquement la trame montante (télémétrie) vers le RPi 5 via
 * l'interface active (_commStream) à la fréquence définie par
 * TASK_SERIAL_TX_PERIOD_MS, puis — si demandé — prépare et émet la rafale de
 * configuration des noms (trames 0x03/0x04) à raison d'UNE trame de
 * 27 octets par cycle : la cadence de télémétrie 100 Hz reste intacte
 * (~100 octets par cycle contre ~1150 possibles à 921 600 bauds).
 */
void SerialComm::_taskTxLoop() {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        /* ---- 1. Télémétrie : priorité de cadence ---- */
        uint8_t localBuf[UL_FRAME_SIZE];
        if (xSemaphoreTake(_mutexTx, pdMS_TO_TICKS(10)) == pdTRUE) {
            memcpy(localBuf, _txBuf, UL_FRAME_SIZE);
            xSemaphoreGive(_mutexTx);

            /* Émission de la trame binaire sur l'interface active (vers RPi5) */
            if (_commStream) {
                _commStream->write(localBuf, UL_FRAME_SIZE);
            }
        }

        /* ---- 2. Synchronisation de configuration demandée ? ----
         * La lecture NVS et la construction de la rafale s'exécutent ici
         * (jamais dans la tâche appelante : Web, Rx série ou Control). */
        if (_cfgSyncPending) {
            _cfgSyncPending = false;
            _loadHardwareConfig();
        }

        /* ---- 3. Rafale en cours : une trame de configuration par cycle ---- */
        if (_cfgBurstIdx < _cfgBurstCount && _commStream) {
            _commStream->write(_cfgBurst[_cfgBurstIdx], CFG_FRAME_SIZE);
            _cfgBurstIdx++;
        }

        /* Attente périodique précise via vTaskDelayUntil */
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_SERIAL_TX_PERIOD_MS));
    }
}
