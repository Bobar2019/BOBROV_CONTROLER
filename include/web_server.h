/**
 * @file web_server.h
 * @brief Serveur Web embarqué avec DNS captif, REST API et WebSocket.
 *
 * Héberge l'interface Web SPA depuis LittleFS, fournit les endpoints REST
 * pour la configuration Wi-Fi, les paramètres et le diagnostic I2C, et diffuse la télémétrie en
 * temps réel via WebSocket.
 *
 * @author Didier Dero
 * @version 1.0.1
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include "config.h"

/**
 * @brief Classe de gestion du serveur Web embarqué et du portail captif.
 *
 * Combine un serveur HTTP (fichiers statiques LittleFS + API REST),
 * un serveur WebSocket (télémétrie temps réel) et un serveur DNS
 * captif (redirection automatique vers l'IHM).
 */
class BobWebServer {
public:
    BobWebServer();

    /** @brief Initialise le Wi-Fi AP, DNS, HTTP et WebSocket */
    void begin();

    /** @brief Boucle de traitement DNS (à appeler depuis la tâche Web) */
    void processDNS();

    /**
     * @brief Diffuse un message JSON à tous les clients WebSocket connectés.
     * @param json Chaîne JSON à envoyer.
     */
    void broadcastWS(const String& json);

    /** @brief Retourne le nombre de clients WebSocket actuellement connectés */
    uint8_t getWSClientCount() const;

    /** @brief Retourne l'adresse IP courante (AP ou STA) */
    String getIPAddress() const;

    /** @brief Indique si l'ESP est connecté à un réseau Wi-Fi externe (mode STA) */
    bool isSTAConnected() const;

private:
    AsyncWebServer  _server;        ///< Serveur HTTP asynchrone
    AsyncWebSocket  _ws;            ///< Serveur WebSocket
    DNSServer       _dns;           ///< Serveur DNS captif
    bool            _staConnected;  ///< État de connexion STA
    bool            _otaRejected;   ///< Upload OTA courant rejeté (fichier incompatible avec la cible)
    bool            _otaWasFS;      ///< Upload OTA courant ciblant la partition LittleFS

    /* -- Calibration altimétrique QNH (v1.0.1) -- */
    bool            _qnhAutoEnable;         ///< Option NVS : requête Open-Meteo au démarrage
    bool            _qnhBootApplied;        ///< Requête de démarrage déjà lancée ce boot
    volatile bool   _qnhFetchRunning;       ///< Requête QNH (boot ou manuelle) en cours
    volatile bool   _qnhLastFetchOk;        ///< Dernière requête Open-Meteo aboutie

    /* -- Configuration des endpoints -- */
    void _setupRoutes();            ///< Endpoints HTTP et fichiers statiques
    void _setupWebSocket();         ///< Gestionnaire d'événements WebSocket
    void _setupCaptiveDNS();        ///< DNS captif (redirection *)

    /* -- Handlers REST API -- */
    void _handleWifiScan(AsyncWebServerRequest* request);
    void _handleWifiConnect(AsyncWebServerRequest* request);
    void _handleSettingsSave(AsyncWebServerRequest* request);
    void _handleCommConfigGet(AsyncWebServerRequest* request);
    void _handleCommConfigPost(AsyncWebServerRequest* request);
    void _handlePwmOutputEnable(AsyncWebServerRequest* request);
    void _handlePwmConfigGet(AsyncWebServerRequest* request);
    void _handlePwmConfigPost(AsyncWebServerRequest* request);
    void _handleGpioConfigGet(AsyncWebServerRequest* request);
    void _handleGpioConfigPost(AsyncWebServerRequest* request);
    void _handleGpioTestPost(AsyncWebServerRequest* request);

    /* -- Signalisation LED WS2812 + entrées numériques (v1.6.0) -- */
    void _handleGpioInputsConfigGet(AsyncWebServerRequest* request);
    void _handleGpioInputsConfigPost(AsyncWebServerRequest* request);
    void _handleLedConfigGet(AsyncWebServerRequest* request);
    void _handleLedConfigPost(AsyncWebServerRequest* request);

    void _handleNamesGet(AsyncWebServerRequest* request);
    void _handleNamesPost(AsyncWebServerRequest* request);

    /* -- Calibration altimétrique QNH (v1.0.1) : Open-Meteo, tâche détachée -- */
    void _handleQnhConfigGet(AsyncWebServerRequest* request);
    void _handleQnhConfigPost(AsyncWebServerRequest* request);
    void _handleQnhRefreshPost(AsyncWebServerRequest* request);
    float _qnhFetchAndApply();              ///< Requête Open-Meteo + application (tâche QnhFetch uniquement)
    static void _qnhTaskEntry(void* param); ///< Entrée de la tâche détachée QNH

    /* -- Gestion de sécurité moteurs (v1.1.0) : config NVS + relance check -- */
    void _handleMotorsConfigGet(AsyncWebServerRequest* request);
    void _handleMotorsConfigPost(AsyncWebServerRequest* request);
    void _handleMotorsCheckPost(AsyncWebServerRequest* request);

    /* -- Simulateur de test virtuel (établi) : injection capteurs, RAM seule -- */
    void _handleSimulatorGet(AsyncWebServerRequest* request);
    void _handleSimulatorPost(AsyncWebServerRequest* request);

    void _handleI2CScan(AsyncWebServerRequest* request);
    void _handleI2CConfigGet(AsyncWebServerRequest* request);
    void _handleI2CConfigPost(AsyncWebServerRequest* request);

    /* -- Routage I2C Plug & Play (v1.2.0) : carte module → bus en NVS -- */
    void _handleI2CRoutingGet(AsyncWebServerRequest* request);
    void _handleI2CRoutingPost(AsyncWebServerRequest* request);

    /* -- Mise à jour OTA (firmware + fichiers Web) -- */
    void _handleSystemStatus(AsyncWebServerRequest* request);
    void _handleOtaBody(AsyncWebServerRequest* request, uint8_t* data, size_t len,
                        size_t index, size_t total, int partitionType);
    void _handleOtaDone(AsyncWebServerRequest* request);

    /* -- Handler WebSocket -- */
    void _onWSEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                    AwsEventType type, void* arg, uint8_t* data, size_t len);

    /* -- Tâche FreeRTOS -- */
    static void _taskEntry(void* param);
    void _taskLoop();
};

/* Instance globale extern */
extern BobWebServer g_webServer;

#endif /* WEB_SERVER_H */
