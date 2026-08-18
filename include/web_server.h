/**
 * @file web_server.h
 * @brief Serveur Web embarqué avec DNS captif, REST API et WebSocket.
 *
 * Héberge l'interface Web SPA depuis LittleFS, fournit les endpoints REST
 * pour la configuration Wi-Fi et le mode, et diffuse la télémétrie en
 * temps réel via WebSocket.
 *
 * @author Didier Dero
 * @version 1.0.0
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

    /* -- Configuration des endpoints -- */
    void _setupRoutes();            ///< Endpoints HTTP et fichiers statiques
    void _setupWebSocket();         ///< Gestionnaire d'événements WebSocket
    void _setupCaptiveDNS();        ///< DNS captif (redirection *)

    /* -- Handlers REST API -- */
    void _handleWifiScan(AsyncWebServerRequest* request);
    void _handleWifiConnect(AsyncWebServerRequest* request);
    void _handleMode(AsyncWebServerRequest* request);
    void _handleSensorConfigGet(AsyncWebServerRequest* request);
    void _handleSensorConfigPost(AsyncWebServerRequest* request);
    void _handleSettingsSave(AsyncWebServerRequest* request);
    void _handleCommConfigGet(AsyncWebServerRequest* request);
    void _handleCommConfigPost(AsyncWebServerRequest* request);
    void _handlePwmOutputEnable(AsyncWebServerRequest* request);
    void _handleI2CScan(AsyncWebServerRequest* request);
    void _handleI2CConfigGet(AsyncWebServerRequest* request);
    void _handleI2CConfigPost(AsyncWebServerRequest* request);

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
