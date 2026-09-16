/**
 * @file web_server.cpp
 * @brief Implémentation du serveur Web embarqué (HTTP, WebSocket, DNS captif).
 *
 * Gère le point d'accès Wi-Fi, le portail captif DNS, les endpoints REST
 * (/api/wifi/scan, /api/wifi/connect, /api/settings/save, /api/comm/config,
 * /api/i2c/scan, /api/i2c/config, /api/pwm/output-enable, /api/ota/firmware,
 * /api/ota/filesystem, /api/system/status) et le serveur WebSocket
 * diffusant la télémétrie temps réel en JSON.
 *
 * @author Didier Dero
 * @version 1.0.0
 */

#include "web_server.h"
#include "sensor_driver.h"
#include "pwm_controller.h"
#include "serial_comm.h"
#include "config.h"
#include "protocol.h"
#include "flight_controller.h"

#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/**
 * @brief Callback Wi-Fi pour suivre l'état STA de manière asynchrone.
 *
 * Mis à jour automatiquement par l'event loop Wi-Fi de l'ESP32.
 * Gère les connexions/déconnexions du mode STA sans bloquer le serveur.
 */
static volatile bool _wifi_sta_got_ip = false;
static volatile bool _wifi_sta_failed = false;

static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            _wifi_sta_got_ip = true;
            _wifi_sta_failed = false;
            Serial.println("[WEB] STA connecté — IP : " + WiFi.localIP().toString());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            _wifi_sta_got_ip = false;
            _wifi_sta_failed = true;
            Serial.println("[WEB] STA déconnecté");
            break;
        default:
            break;
    }
}

BobWebServer g_webServer;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

/**
 * @brief Constructeur : initialise les serveurs HTTP et WebSocket.
 */
BobWebServer::BobWebServer()
    : _server(80)
    , _ws("/ws")
    , _staConnected(false)
    , _otaRejected(false)
    , _otaWasFS(false)
{
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

/**
 * @brief Initialise le Wi-Fi AP, les serveurs et lance la tâche.
 *
 * Séquence :
 * 1. Configuration du point d'accès Wi-Fi avec IP statique
 * 2. Configuration des routes HTTP et fichiers statiques
 * 3. Démarrage du serveur DNS captif
 * 4. Démarrage du serveur HTTP
 * 5. Lancement de la tâche FreeRTOS sur Core 0
 */
void BobWebServer::begin() {
    /* Configuration du point d'accès Wi-Fi */
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),    /* IP locale */
        IPAddress(192, 168, 4, 1),    /* Gateway */
        IPAddress(255, 255, 255, 0)   /* Masque */
    );
    WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false, AP_MAX_CLIENTS);

    /* Enregistrement du callback d'événements Wi-Fi (STA) */
    WiFi.onEvent(onWiFiEvent);

    Serial.println("[WEB] Point d'accès Wi-Fi démarré : " + String(AP_SSID));
    Serial.println("[WEB] IP : " + WiFi.softAPIP().toString());

    /* Tentative de reconnexion au dernier réseau STA mémorisé */
    Preferences prefs;
    prefs.begin("wifi", true);  /* Lecture seule */
    String savedSSID = prefs.getString("ssid", "");
    String savedPass = prefs.getString("pass", "");
    prefs.end();

    if (savedSSID.length() > 0) {
        Serial.println("[WEB] Tentative de connexion à : " + savedSSID);
        _wifi_sta_got_ip = false;
        _wifi_sta_failed = false;
        WiFi.begin(savedSSID.c_str(), savedPass.c_str());
        /* Connexion non bloquante : les events Wi-Fi mettront à jour le statut */
    }

    /* Configuration des endpoints et du WebSocket */
    _setupRoutes();
    _setupWebSocket();
    _setupCaptiveDNS();

    /* Démarrage du serveur HTTP */
    _server.begin();
    Serial.println("[WEB] Serveur HTTP démarré sur le port 80");

    /* Lancement de la tâche FreeRTOS sur Core 0 (Wi-Fi) */
    xTaskCreatePinnedToCore(
        _taskEntry, "WebServer", STACK_SIZE_WEB,
        this, PRIORITY_WEB, nullptr, CORE_WIFI
    );
}

/* =========================================================================
 * DNS CAPTIF
 * ========================================================================= */

/**
 * @brief Configure le serveur DNS pour rediriger toutes les requêtes.
 *
 * Le DNS captif intercepte toutes les requêtes DNS et répond avec
 * l'adresse IP du point d'accès (192.168.4.1), forçant ainsi la
 * redirection vers l'interface Web.
 */
void BobWebServer::_setupCaptiveDNS() {
    _dns.start(53, "*", IPAddress(192, 168, 4, 1));
    Serial.println("[WEB] DNS captif démarré (port 53, redirection → 192.168.4.1)");
}

/**
 * @brief Traite les requêtes DNS en attente.
 *
 * Doit être appelé régulièrement depuis la tâche Web pour
 * répondre aux résolutions DNS des clients connectés.
 */
void BobWebServer::processDNS() {
    _dns.processNextRequest();
}

/* =========================================================================
 * ROUTES HTTP ET FICHIERS STATIQUES
 * ========================================================================= */

/**
 * @brief Configure toutes les routes HTTP, le portail captif et les fichiers LittleFS.
 *
 * Ordre d'enregistrement critique : les routes de détection de portail captif
 * doivent être enregistrées AVANT le service statique et le handler 404.
 *
 * - Portail captif : redirection 302 vers 192.168.4.1 pour tous les OS
 * - Fichiers statiques : racine "/" sert index.html depuis LittleFS
 * - REST API : /api/wifi/scan, /api/wifi/connect, /api/settings/save,
 *   /api/comm/config, /api/i2c/scan, /api/i2c/config, /api/pwm/output-enable
 * - Handler catchall : redirection pour les hôtes non reconnus
 */
void BobWebServer::_setupRoutes() {

    /* =====================================================================
     * PORTAIL CAPTIF — Détection automatique par les OS
     * =====================================================================
     *
     * Quand un appareil se connecte au Wi-Fi, il interroge des URLs spécifiques
     * pour vérifier s'il a accès à Internet. En renvoyant une redirection 302
     * vers notre IP, l'OS ouvre automatiquement la page captive.
     */

    /* Android / ChromeOS : /generate_204 */
    _server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    /* Android : connectivity check fallback */
    _server.on("/gen_204", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    /* Android (Chrome) : captive portal detection */
    _server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    /* iOS / macOS (Apple captive portal detection) */
    _server.on("/library/test/success.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    /* Windows (Microsoft NCSI — Network Connectivity Status Indicator) */
    _server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });
    _server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    /* Firefox captive portal detection */
    _server.on("/canonical.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    /* Kindle / Fire OS */
    _server.on("/kindle-wifi/wifistub.html", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->redirect("http://192.168.4.1/");
    });

    /* =====================================================================
     * API REST
     * ===================================================================== */

    /* REST : Scan des réseaux Wi-Fi environnants */
    _server.on("/api/wifi/scan", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleWifiScan(request); });

    /* REST : Connexion à un réseau Wi-Fi externe */
    _server.on("/api/wifi/connect", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleWifiConnect(request); });



    /* REST : Sauvegarde paramètres PID + watchdog */
    _server.on("/api/settings/save", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleSettingsSave(request); });

    /* REST : Configuration interface série RPi5 (lecture et écriture) */
    _server.on("/api/comm/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleCommConfigGet(request); });
    _server.on("/api/comm/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleCommConfigPost(request); });

    /* REST : Activation/désactivation des sorties physiques (Dry-Run) */
    _server.on("/api/pwm/output-enable", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handlePwmOutputEnable(request); });

    /* REST : Scan I2C (GET) */
    _server.on("/api/i2c/scan", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleI2CScan(request); });

    /* REST : Configuration broches I2C (GET + POST) */
    _server.on("/api/i2c/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleI2CConfigGet(request); });
    _server.on("/api/i2c/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleI2CConfigPost(request); });

    /* REST : Mise à jour OTA — corps brut d'un .bin vers la partition
     * applicative inactive (firmware) ou LittleFS (fichiers Web). On utilise
     * le handler de corps (onBody) et NON le téléversement multipart : le
     * parseur multipart de la bibliothèque perd silencieusement les corps
     * multi-paquets (petit corps OK, ≥ 8 Ko jamais remis au handler). Le
     * callback reçoit les blocs au fil de l'eau ; le callback de fin conclut
     * (redémarrage différé 3 s si l'écriture est validée). */
    _server.on("/api/ota/firmware", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleOtaDone(request); },
        nullptr,   /* pas d'upload multipart : corps brut via onBody */
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len,
               size_t index, size_t total) {
            _handleOtaBody(request, data, len, index, total, U_FLASH);
        });
    _server.on("/api/ota/filesystem", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleOtaDone(request); },
        nullptr,   /* pas d'upload multipart : corps brut via onBody */
        [this](AsyncWebServerRequest* request, uint8_t* data, size_t len,
               size_t index, size_t total) {
            _handleOtaBody(request, data, len, index, total, U_SPIFFS);
        });

    /* REST : État système (uptime — l'interface OTA y lit la preuve du reboot) */
    _server.on("/api/system/status", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleSystemStatus(request); });

    /* Fichier .glb du modele 3D du ROV : l'extension est inconnue de la table
     * MIME du core Arduino (servie en text/plain) — route explicite avec le bon
     * Content-Type model/gltf-binary, sans jamais de compression (binaire deja
     * compact). Servie AVANT serveStatic pour prendre la priorité. */
    _server.on("/3D/bob_rov_3D.glb", HTTP_GET,
        [](AsyncWebServerRequest* request) {
            if (!LittleFS.exists("/3D/bob_rov_3D.glb")) {
                request->send(404, "text/plain", "Not Found");
                return;
            }
            AsyncWebServerResponse* res = request->beginResponse(
                LittleFS, "/3D/bob_rov_3D.glb", "model/gltf-binary");
            res->addHeader("Cache-Control", "max-age=86400");
            res->addHeader("Content-Encoding", "identity");
            request->send(res);
        });



    /* =====================================================================
     * FICHIERS STATIQUES LITTLEFS (après les routes spécifiques)
     * ===================================================================== */

    _server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    /* =====================================================================
     * HANDLER CATCHALL — Redirection portail captif pour tout hôte inconnu
     * =====================================================================
     *
     * Toute requête dont le Host header n'est pas 192.168.4.1 est redirigée
     * vers l'interface captive. Cela intercepte les requêtes DNS détournées
     * par le DNS captif (ex: www.google.com → 192.168.4.1).
     *
     * Si LittleFS n'a pas été uploadé, une page de secours est servie.
     */
    _server.onNotFound([](AsyncWebServerRequest* request) {
        /* Si le Host n'est pas notre IP, c'est une requête captivée → redirect */
        String host = request->getHeader("Host") ? request->getHeader("Host")->value() : "";
        if (host != "192.168.4.1" && host != "192.168.4.1:80") {
            request->redirect("http://192.168.4.1/");
            return;
        }

        /* Hôte correct → servir une page de secours si LittleFS est vide */
        String path = request->url();
        if (path == "/" || path == "/index.html" || path == "/index.htm") {
            request->send(200, "text/html",
                "<!DOCTYPE html><html lang='fr'><head><meta charset='UTF-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>BOB-CONTROL</title>"
                "<style>"
                "body{background:#0d0d0d;color:#e0e0e0;font-family:monospace;"
                "display:flex;align-items:center;justify-content:center;height:100vh;margin:0}"
                ".box{background:#1a1a1a;border:2px solid #ff6a00;border-radius:8px;"
                "padding:30px;max-width:500px;text-align:center}"
                "h1{color:#ff6a00;margin:0 0 15px}code{background:#242424;padding:4px 10px;"
                "border-radius:4px;color:#00bcd4;font-size:0.95em}"
                "p{line-height:1.6;color:#888}"
                "</style></head><body><div class='box'>"
                "<h1>BOB-CONTROL</h1>"
                "<p>Le firmware fonctionne, mais les fichiers web ne sont pas encore "
                "téléversés dans la mémoire flash.</p>"
                "<p>Exécutez la commande :</p>"
                "<p><code>pio run -t uploadfs</code></p>"
                "<p>Puis rechargez cette page.</p>"
                "</div></body></html>"
            );
        } else {
            /* Autres chemins (CSS, JS, images) → 404 */
            request->send(404, "text/plain", "Not Found");
        }
    });
}

/* =========================================================================
 * HANDLERS REST API
 * ========================================================================= */

/**
 * @brief Handler GET /api/wifi/scan - Retourne la liste des AP détectés.
 *
 * Effectue un scan Wi-Fi asynchrone et retourne un tableau JSON
 * contenant SSID, RSSI (dBm) et type de chiffrement pour chaque AP.
 *
 * @param request Requête HTTP entrante.
 */
void BobWebServer::_handleWifiScan(AsyncWebServerRequest* request) {
    int n = WiFi.scanNetworks(true);  /* Scan asynchrone */

    /* Attendre la fin du scan (max 5 secondes) */
    unsigned long start = millis();
    while (WiFi.scanComplete() == WIFI_SCAN_RUNNING && (millis() - start) < 5000) {
        delay(10);
    }

    n = WiFi.scanComplete();
    if (n < 0) n = 0;

    /* Construction du JSON de réponse */
    JsonDocument doc;
    JsonArray arr = doc["networks"].to<JsonArray>();

    for (int i = 0; i < n; i++) {
        JsonObject net = arr.add<JsonObject>();
        net["ssid"]   = WiFi.SSID(i);
        net["rssi"]   = WiFi.RSSI(i);
        net["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);

    /* Nettoyage des résultats de scan */
    WiFi.scanDelete();
}

/**
 * @brief Handler POST /api/wifi/connect - Connexion à un réseau Wi-Fi.
 *
 * Attend un corps form-data : ssid=...&pass=...
 * Sauvegarde les identifiants en NVS et lance la connexion STA
 * de manière non bloquante (sans tuer le point d'accès).
 *
 * IMPORTANT : Ne pas appeler WiFi.disconnect() en mode AP_STA car cela
 * peut désactiver le SoftAP et couper la connexion avec le client web.
 *
 * @param request Requête HTTP entrante.
 */
void BobWebServer::_handleWifiConnect(AsyncWebServerRequest* request) {
    /* Vérifier la présence du paramètre ssid */
    if (!request->hasParam("ssid", true)) {
        request->send(400, "application/json",
                       "{\"error\":\"Paramètre 'ssid' manquant\"}");
        return;
    }

    String ssid = request->getParam("ssid", true)->value();
    String pass = request->hasParam("pass", true)
                  ? request->getParam("pass", true)->value() : "";

    Serial.println("[WEB] Connexion demandée à : " + ssid);

    /* Sauvegarde des identifiants en NVS (Preferences) */
    Preferences prefs;
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();

    /* Réinitialiser les flags d'état STA */
    _wifi_sta_got_ip = false;
    _wifi_sta_failed = false;

    /*
     * Lancer la connexion STA SANS appeler WiFi.disconnect() d'abord.
     * WiFi.begin() avec de nouveaux identifiants remplace automatiquement
     * la connexion STA précédente sans affecter le SoftAP.
     */
    WiFi.begin(ssid.c_str(), pass.c_str());

    /*
     * Attente non bloquante de la connexion (max 15 secondes).
     * On cède le CPU à chaque itération pour permettre au WiFi event loop
     * et au serveur HTTP de continuer à fonctionner.
     */
    unsigned long start = millis();
    while (!_wifi_sta_got_ip && !_wifi_sta_failed && (millis() - start) < 15000) {
        delay(100);  /* Cède le CPU aux tâches Wi-Fi/HTTP */
    }

    if (_wifi_sta_got_ip && WiFi.status() == WL_CONNECTED) {
        _staConnected = true;
        String ip = WiFi.localIP().toString();
        Serial.println("[WEB] Connecté à " + ssid + " — IP : " + ip);
        request->send(200, "application/json",
                       "{\"status\":\"connected\",\"ip\":\"" + ip + "\"}");
    } else {
        _staConnected = false;
        Serial.println("[WEB] Échec de connexion à " + ssid);
        request->send(200, "application/json",
                       "{\"status\":\"failed\",\"message\":\"Vérifiez le mot de passe\"}");
    }
}


/**
 * @brief Handler POST /api/settings/save — Sauvegarde PID + watchdog en NVS.
 *
 * Reçoit les paramètres PID (roll, pitch, yaw, altitude) et le timeout watchdog
 * via form-data et les sauvegarde en NVS pour persistance.
 */
void BobWebServer::_handleSettingsSave(AsyncWebServerRequest* request) {
    /* Lecture des paramètres depuis le form-data */
    float r_kp = request->hasParam("r_kp", true) ? request->getParam("r_kp", true)->value().toFloat() : 2.0f;
    float r_ki = request->hasParam("r_ki", true) ? request->getParam("r_ki", true)->value().toFloat() : 0.05f;
    float r_kd = request->hasParam("r_kd", true) ? request->getParam("r_kd", true)->value().toFloat() : 0.5f;
    float p_kp = request->hasParam("p_kp", true) ? request->getParam("p_kp", true)->value().toFloat() : 2.0f;
    float p_ki = request->hasParam("p_ki", true) ? request->getParam("p_ki", true)->value().toFloat() : 0.05f;
    float p_kd = request->hasParam("p_kd", true) ? request->getParam("p_kd", true)->value().toFloat() : 0.5f;
    float d_kp = request->hasParam("d_kp", true) ? request->getParam("d_kp", true)->value().toFloat() : 3.0f;
    float d_ki = request->hasParam("d_ki", true) ? request->getParam("d_ki", true)->value().toFloat() : 0.1f;
    float d_kd = request->hasParam("d_kd", true) ? request->getParam("d_kd", true)->value().toFloat() : 1.0f;
    float y_kp = request->hasParam("y_kp", true) ? request->getParam("y_kp", true)->value().toFloat() : 1.5f;
    float y_ki = request->hasParam("y_ki", true) ? request->getParam("y_ki", true)->value().toFloat() : 0.02f;
    float y_kd = request->hasParam("y_kd", true) ? request->getParam("y_kd", true)->value().toFloat() : 0.3f;
    uint32_t wdg = request->hasParam("wdg_ms", true) ? request->getParam("wdg_ms", true)->value().toInt() : 500;

    /* Sauvegarde NVS */
    Preferences prefs;
    prefs.begin("autopilot", false);
    prefs.putFloat("r_kp", r_kp); prefs.putFloat("r_ki", r_ki); prefs.putFloat("r_kd", r_kd);
    prefs.putFloat("p_kp", p_kp); prefs.putFloat("p_ki", p_ki); prefs.putFloat("p_kd", p_kd);
    prefs.putFloat("d_kp", d_kp); prefs.putFloat("d_ki", d_ki); prefs.putFloat("d_kd", d_kd);
    prefs.putFloat("y_kp", y_kp); prefs.putFloat("y_ki", y_ki); prefs.putFloat("y_kd", y_kd);
    prefs.putUInt("wdg_ms", wdg);
    prefs.end();

    /* Appliquer le watchdog immédiatement */
    g_serial.setWatchdogTimeout(wdg);

    Serial.println("[WEB] Paramètres PID + watchdog sauvegardés en NVS");
    request->send(200, "application/json", "{\"status\":\"ok\"}");
}

/* =========================================================================
 * HANDLERS REST — CONFIGURATION INTERFACE SÉRIE RPi5
 * ========================================================================= */

/**
 * @brief GET /api/comm/config — Retourne l'interface série active.
 *
 * Réponse JSON : {"interface": 0} (USB-CDC) ou {"interface": 1} (UART GPIO 1/2).
 */
void BobWebServer::_handleCommConfigGet(AsyncWebServerRequest* request) {
    JsonDocument doc;
    doc["interface"] = (uint8_t)g_serial.getActiveInterface();
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/**
 * @brief POST /api/comm/config — Change l'interface série active.
 *
 * Paramètre attendu (form-data) : "iface" = 0 (USB-CDC) ou 1 (UART GPIO).
 * Sauvegarde en NVS et bascule à chaud du pointeur _commStream.
 * Un redémarrage est recommandé pour garantir la cohérence des tâches FreeRTOS.
 */
void BobWebServer::_handleCommConfigPost(AsyncWebServerRequest* request) {
    uint8_t iface = request->hasParam("iface", true)
                    ? request->getParam("iface", true)->value().toInt()
                    : 0;

    if (iface > 1) iface = 0; /* Valeur par défaut sécurisée */

    /* Sauvegarde NVS du choix */
    Preferences prefs;
    prefs.begin("config", false);
    prefs.putUChar("comm_iface", iface);
    prefs.end();

    Serial.println("[WEB] Interface série RPi5 réglée sur : " + String(iface));

    /* Réponse avec indication de redémarrage */
    request->send(200, "application/json",
                  "{\"status\":\"ok\",\"iface\":" + String(iface) +
                  ",\"msg\":\"Redémarrage requis pour appliquer\"}");
}

/**
 * @brief Handler POST /api/pwm/output-enable — Active/désactive les sorties physiques.
 *
 * Accepte un JSON brut : `{"enabled": true}` ou `{"enabled": false}`.
 * En mode Dry-Run (false), le PCA9685 reste au neutre mais les consignes
 * PWM continuent d'être mémorisées pour la télémétrie WebSocket.
 */
void BobWebServer::_handlePwmOutputEnable(AsyncWebServerRequest* request) {
    if (!request->hasParam("plain", true)) {
        /* Accepte aussi un paramètre de query simple ?enabled=1/0 */
        bool enabled = request->hasParam("enabled")
                       ? request->getParam("enabled")->value().toInt() != 0
                       : false;
        g_pwm.setPhysicalOutputsEnabled(enabled);
        request->send(200, "application/json",
                      "{\"status\":\"ok\",\"enabled\":" + String(enabled ? "true" : "false") + "}");
        return;
    }

    /* Parsing du body JSON */
    const AsyncWebParameter* body = request->getParam("plain", true);
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, body->value());
    if (err) {
        request->send(400, "application/json", "{\"error\":\"JSON invalide\"}");
        return;
    }

    bool enabled = doc["enabled"] | false;
    g_pwm.setPhysicalOutputsEnabled(enabled);

    Serial.println("[WEB] Sorties physiques : " +
                   String(enabled ? "ACTIVES" : "NEUTRALISÉES (Dry-Run)"));

    String response;
    JsonDocument resp;
    resp["status"] = "ok";
    resp["enabled"] = enabled;
    serializeJson(resp, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler GET /api/i2c/scan — Scan complet du bus I2C.
 *
 * Exécute le scan via g_sensors.scanI2CBus() et retourne un tableau JSON
 * des périphériques détectés avec adresse (hex/dec) et nom du composant.
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleI2CScan(AsyncWebServerRequest* request) {
    std::vector<I2CDeviceInfo> devices = g_sensors.scanI2CBus();

    String response;
    JsonDocument doc;
    JsonArray arr = doc["devices"].to<JsonArray>();
    for (const auto& d : devices) {
        JsonObject obj = arr.add<JsonObject>();
        obj["address"] = String(d.hexStr);
        obj["dec"]     = d.address;
        obj["bus"]     = d.bus;    /* 1 = capteurs (Wire) | 2 = MS5837 + PCA9685 (Wire1) */
        obj["name"]    = String(d.name);
    }
    doc["sda"] = g_sensors.getCurrentSDA();
    doc["scl"] = g_sensors.getCurrentSCL();
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler GET /api/i2c/config — Retourne les broches SDA/SCL actuelles.
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleI2CConfigGet(AsyncWebServerRequest* request) {
    String response;
    JsonDocument doc;
    doc["sda"]  = g_sensors.getCurrentSDA();
    doc["scl"]  = g_sensors.getCurrentSCL();
    doc["sda2"] = g_sensors.getCurrent2SDA();
    doc["scl2"] = g_sensors.getCurrent2SCL();
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Redémarrage différé du contrôleur (3 s)
 *
 * Utilisé après sauvegarde NVS des broches d'un bus I2C : elles ne sont
 * appliquées qu'au boot. Une micro-tâche dédiée — plus sûre qu'un timer
 * FreeRTOS, dont la commande de démarrage (transmise avec un délai de
 * blocage nul) peut être silencieusement perdue — laisse partir la réponse
 * HTTP puis déclenche ESP.restart(). La garde statique neutralise un
 * double-clic sur le bouton.
 * ------------------------------------------------------------------------- */
static void _scheduleRestart(uint32_t delayMs) {
    static bool sScheduled = false;
    if (sScheduled) return;
    sScheduled = true;
    if (xTaskCreate([](void* arg) {
            vTaskDelay(pdMS_TO_TICKS((uint32_t)(uintptr_t)arg));
            ESP.restart();
        }, "web_rst", 2048, (void*)(uintptr_t)delayMs, 1, nullptr) != pdPASS) {
        sScheduled = false;   /* création impossible : permet un nouvel essai */
        Serial.println("[WEB] Redémarrage différé : échec de création de la tâche");
    }
}

/* -------------------------------------------------------------------------
 * Handler POST /api/i2c/config — Enregistre les nouvelles broches I2C.
 *
 * Bus n°1 (défaut) et bus n°2 (paramètre bus=2) : même traitement —
 * sauvegarde NVS vérifiée par relecture puis redémarrage différé de 3 s
 * (les broches ne sont appliquées qu'au boot : le bus n°2 est partagé avec
 * les sorties PWM ; le bus n°1 suit la même procédure pour une interface
 * uniforme).
 *
 * Form : sda=10&scl=11[&bus=2]  |  JSON : {"sda":6,"scl":7,"bus":2}
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleI2CConfigPost(AsyncWebServerRequest* request) {
    uint8_t sda, scl;
    uint8_t bus = 1;   /* 1 = capteurs (défaut) | 2 = MS5837 + PCA9685 (Wire1) */

    /* Accepte form-urlencoded (sda=X&scl=Y[&bus=N]) ou query params */
    if (request->hasParam("sda", true) && request->hasParam("scl", true)) {
        sda = request->getParam("sda", true)->value().toInt();
        scl = request->getParam("scl", true)->value().toInt();
        if (request->hasParam("bus", true)) bus = request->getParam("bus", true)->value().toInt();
    } else if (request->hasParam("sda") && request->hasParam("scl")) {
        sda = request->getParam("sda")->value().toInt();
        scl = request->getParam("scl")->value().toInt();
        if (request->hasParam("bus")) bus = request->getParam("bus")->value().toInt();
    } else if (request->hasParam("plain", true)) {
        /* Fallback JSON body */
        const AsyncWebParameter* body = request->getParam("plain", true);
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, body->value());
        if (err || !doc["sda"].is<uint8_t>() || !doc["scl"].is<uint8_t>()) {
            request->send(400, "application/json", "{\"error\":\"JSON invalide\"}");
            return;
        }
        sda = doc["sda"].as<uint8_t>();
        scl = doc["scl"].as<uint8_t>();
        if (doc["bus"].is<uint8_t>()) bus = doc["bus"].as<uint8_t>();
    } else {
        request->send(400, "application/json", "{\"error\":\"Paramètres sda/scl requis\"}");
        return;
    }

    /* Validation basique des GPIO valides sur ESP32-S3 */
    if (sda > 48 || scl > 48) {
        request->send(400, "application/json", "{\"error\":\"GPIO invalide (0-48)\"}");
        return;
    }

    if (bus != 1 && bus != 2) {
        request->send(400, "application/json", "{\"error\":\"Bus invalide (1 ou 2)\"}");
        return;
    }

    /* Les DEUX bus appliquent leurs nouvelles broches au boot : sauvegarde NVS
     * (vérifiée par relecture côté driver) puis redémarrage différé de 3 s.
     * Uniformité des deux cartes de l'interface — et sur le bus n°2, partagé
     * avec la tâche de contrôle (PCA9685 @ 100 Hz), un reset à chaud couperait
     * de toute façon les sorties PWM en pleine navigation. */
    if (!g_sensors.saveBusPins(bus, sda, scl)) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (broches non enregistrees)\"}");
        return;
    }
    Serial.println("[WEB] Config I2C bus n°" + String(bus) + " : SDA=" + String(sda)
                   + ", SCL=" + String(scl) + " — redémarrage dans 3 s");
    _scheduleRestart(3000);

    String response;
    JsonDocument resp;
    resp["status"]     = "ok";
    resp["bus"]        = bus;
    resp["sda"]        = sda;
    resp["scl"]        = scl;
    resp["restart_ms"] = 3000;
    serializeJson(resp, response);
    request->send(200, "application/json", response);
}

/* =========================================================================
 * MISE À JOUR OTA — TÉLÉVERSEMENT FIRMWARE & FICHIERS WEB (LittleFS)
 * =========================================================================
 *
 * Le schéma de partitions (default_8MB.csv) contient deux partitions
 * applicatives (ota_0/ota_1) : le téléversement écrit dans la partition
 * INACTIVE, l'ancien firmware continue de tourner jusqu'au redémarrage —
 * une coupure en cours d'écriture ne brike jamais le contrôleur.
 * ========================================================================= */

/**
 * @brief Handler GET /api/system/status — Uptime du contrôleur.
 *
 * Utilisé par l'interface OTA comme preuve de redémarrage : après un flash,
 * l'uptime repart de zéro (un simple code 200 peut venir de l'ancien firmware
 * encore vivant pendant le délai de redémarrage de 3 s).
 */
void BobWebServer::_handleSystemStatus(AsyncWebServerRequest* request) {
    String response;
    JsonDocument doc;
    doc["status"]    = "ok";
    doc["uptime_ms"] = (uint32_t)millis();
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/**
 * @brief Handler corps brut POST /api/ota/firmware | /api/ota/filesystem.
 *
 * Reçoit un .bin par blocs (corps brut, hors parseur multipart) et l'écrit
 * directement en flash :
 *  - U_FLASH   → partition applicative inactive (firmware.bin de `pio run`)
 *  - U_SPIFFS  → partition LittleFS (littlefs.bin de `pio run -t buildfs`),
 *               démontée au préalable pour éviter toute écriture concurrente.
 *
 * Sécurité : la propulsion est neutralisée (Dry-Run + neutre) AVANT la
 * première écriture — les écritures flash suspendent brièvement les deux
 * cœurs, aucune sortie ne doit rester active pendant la mise à jour.
 *
 * @param partitionType U_FLASH (firmware) ou U_SPIFFS (fichiers Web)
 */
void BobWebServer::_handleOtaBody(AsyncWebServerRequest* request, uint8_t* data,
                                  size_t len, size_t index, size_t total,
                                  int partitionType) {
    if (index == 0) {
        _otaRejected = false;
        _otaWasFS = (partitionType == U_SPIFFS);
        Serial.println(String("[WEB] OTA : réception du corps (") + String(total)
                       + " octets — " + String(_otaWasFS ? "fichiers Web" : "firmware") + ")");

        /* Neutraliser la propulsion AVANT de toucher la flash */
        g_pwm.setPhysicalOutputsEnabled(false);
        g_pwm.setAllNeutral();

        /* Octet magie ESP : un firmware commence TOUJOURS par 0xE9, jamais une
         * image LittleFS — détecte l'inversion des deux fichiers AVANT
         * d'écrire (une image FS écrite en partition firmware serait de toute
         * façon rejetée, mais l'inverse corromprait LittleFS silencieusement). */
        if (len > 0) {
            const bool magicOk = (partitionType == U_FLASH)
                               ? (data[0] == 0xE9) : (data[0] != 0xE9);
            if (!magicOk) {
                _otaRejected = true;
                Serial.println("[WEB] OTA : fichier refusé — octet magie 0xE9 incompatible avec la cible");
                return;
            }
        }

        /* LittleFS doit être démonté avant la réécriture de SA partition
         * (LittleFSFS::end() retourne void — pas de statut de démontage). */
        if (_otaWasFS) {
            LittleFS.end();
            Serial.println("[WEB] OTA : LittleFS démonté avant réécriture");
        }

        if (!Update.begin(UPDATE_SIZE_UNKNOWN, partitionType)) {
            _otaRejected = true;
            Update.printError(Serial);
            return;
        }
    }

    if (!_otaRejected && Update.isRunning() && !Update.hasError() && len) {
        if (Update.write(data, len) != len) {
            Update.printError(Serial);
        }
    }

    /* Dernier bloc reçu (corps complet) : valider l'écriture en flash */
    if (index + len == total && !_otaRejected) {
        if (Update.end(true)) {
            Serial.println("[WEB] OTA : partition écrite (" + String(Update.size())
                           + " octets) — redémarrage requis");
        } else {
            Update.printError(Serial);
        }
    }
}

/**
 * @brief Handler de fin POST /api/ota/* — statut final + redémarrage différé.
 *
 * Appelé une fois le body entièrement reçu. Succès uniquement si l'écriture
 * est complète et validée (Update terminé, sans erreur, taille non nulle) :
 * le redémarrage est programmé à 3 s pour laisser partir la réponse HTTP.
 * En cas d'échec sur la cible fichiers Web, LittleFS est remonté si possible
 * pour conserver l'interface (sinon redémarrage pour revenir à un état propre).
 */
void BobWebServer::_handleOtaDone(AsyncWebServerRequest* request) {
    const bool ok = !_otaRejected && !Update.isRunning() && !Update.hasError()
                 && Update.size() > 0;

    if (ok) {
        Serial.println("[WEB] OTA : succès — redémarrage dans 3 s");
        _scheduleRestart(3000);
        String response;
        JsonDocument doc;
        doc["status"]     = "ok";
        doc["restart_ms"] = 3000;
        serializeJson(doc, response);
        request->send(200, "application/json", response);
        return;
    }

    /* Échec : identifier la cause, libérer la partition pour un nouvel essai */
    const char* reason;
    if (_otaRejected) {
        reason = "fichier incompatible avec la cible (octet magie 0xE9)";
    } else if (Update.size() == 0) {
        reason = "transfert interrompu ou fichier vide";
    } else {
        reason = Update.errorString();
    }
    Update.abort();
    Serial.println(String("[WEB] OTA : échec — ") + reason);

    /* Cible fichiers Web : LittleFS a été démonté avant la réécriture — le
     * remonter si la partition est saine ; sinon redémarrer (état propre). */
    if (_otaWasFS) {
        if (LittleFS.begin()) {
            Serial.println("[WEB] OTA : LittleFS remonté après échec");
        } else {
            Serial.println("[WEB] OTA : LittleFS irrécupérable — redémarrage");
            _scheduleRestart(3000);
        }
    }

    request->send(500, "application/json", String("{\"error\":\"") + reason + "\"}");
}

/* =========================================================================
 * WEBSOCKET
 * ========================================================================= */

/**
 * @brief Configure le gestionnaire d'événements WebSocket.
 *
 * Enregistre le handler d'événements pour les connexions, déconnexions
 * et messages reçus sur le endpoint /ws.
 */
void BobWebServer::_setupWebSocket() {
    _ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
        _onWSEvent(server, client, type, arg, data, len);
    });
    _server.addHandler(&_ws);
    Serial.println("[WEB] WebSocket initialisé sur /ws");
}

/**
 * @brief Handler d'événements WebSocket.
 *
 * Gère les connexions/déconnexions et les messages de commande reçus
 * depuis les clients Web (mode banc de test).
 *
 * @param server Pointeur vers le serveur WebSocket.
 * @param client Pointeur vers le client concerné.
 * @param type   Type d'événement (CONNECT, DISCONNECT, MESSAGE, ERROR).
 * @param arg    Argument spécifique au type d'événement.
 * @param data   Données du message reçu.
 * @param len    Taille des données.
 */
void BobWebServer::_onWSEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                               AwsEventType type, void* arg, uint8_t* data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.println("[WS] Client connecté : " + String(client->id()));
            break;

        case WS_EVT_DISCONNECT:
            Serial.println("[WS] Client déconnecté : " + String(client->id()));
            break;

        case WS_EVT_DATA: {
            /* Parsing du message JSON reçu du client */
            if (len > 0 && data != nullptr) {
                JsonDocument doc;
                DeserializationError err = deserializeJson(doc, data, len);
                if (!err) {
                    /* Commande de test manuel depuis le banc de test */
                    if (doc["pwm"].is<JsonArray>()) {
                        JsonArray pwmArr = doc["pwm"].as<JsonArray>();
                        uint16_t values[NUM_PWM_CHANNELS];
                        for (uint8_t i = 0; i < NUM_PWM_CHANNELS && i < pwmArr.size(); i++) {
                            values[i] = pwmArr[i].as<uint16_t>();
                        }
                        /* Application des valeurs PWM manuelles */
                        g_pwm.setAllPWM(values);
                    }


                    /* Commande d'activation des sorties physiques (Dry-Run) */
                    if (doc["output_enable"].is<bool>()) {
                        bool enabled = doc["output_enable"].as<bool>();
                        g_pwm.setPhysicalOutputsEnabled(enabled);
                        Serial.println("[WS] Sorties physiques : " +
                                       String(enabled ? "ACTIVES" : "NEUTRALISÉES (Dry-Run)"));
                    }

                    /* Commande de changement de mode de pilotage */
                    if (doc["set_mode"].is<uint8_t>()) {
                        uint8_t mode = doc["set_mode"].as<uint8_t>();
                        bool ok = g_flightCtrl.setModeFromWeb(mode);
                        Serial.println("[WS] Mode pilotage : " + String(mode) +
                                       (ok ? " appliqué" : " ignoré (RPi 5 maître)"));
                    }

                    /* Commande d'étalonnage de la pression de surface (tare MS5803) :
                     * à déclencher capteur HORS de l'eau pour mémoriser P_surface. */
                    if (doc["tare_surface"].is<bool>() && doc["tare_surface"].as<bool>()) {
                        g_sensors.requestSurfaceTare();
                        Serial.println("[WS] Tare surface demandée (P_surface à recapturer)");
                    }

                    /* Commande de tare Nord (cap IMU) : ROV immobile, nez pointé vers le
                     * Nord réel → le cap courant devient 0°. Offset appliqué à la source
                     * (télémétrie, HSI, modèle 3D et PID de cap). À faire en mode PASSIF. */
                    if (doc["tare_north"].is<bool>() && doc["tare_north"].as<bool>()) {
                        g_sensors.requestNorthTare();
                        Serial.println("[WS] Tare Nord demandée (cap remis à zéro sur le Nord réel)");
                    }
                }
            }
            break;
        }

        case WS_EVT_ERROR:
            Serial.println("[WS] Erreur sur client : " + String(client->id()));
            break;
    }
}

/**
 * @brief Diffuse un message JSON à tous les clients WebSocket.
 * @param json Chaîne JSON à envoyer.
 */
void BobWebServer::broadcastWS(const String& json) {
    _ws.textAll(json);
}

/**
 * @brief Retourne le nombre de clients WebSocket connectés.
 */
uint8_t BobWebServer::getWSClientCount() const {
    return _ws.count();
}

/**
 * @brief Retourne l'adresse IP courante (AP ou STA selon le mode actif).
 */
String BobWebServer::getIPAddress() const {
    if (_staConnected && WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP().toString();
    }
    return WiFi.softAPIP().toString();
}

/**
 * @brief Indique si l'ESP est connecté en mode STA à un réseau externe.
 */
bool BobWebServer::isSTAConnected() const {
    return _staConnected && (WiFi.status() == WL_CONNECTED);
}

/* =========================================================================
 * TÂCHE FREERTOS
 * ========================================================================= */

/**
 * @brief Point d'entrée statique de la tâche Web.
 */
void BobWebServer::_taskEntry(void* param) {
    static_cast<BobWebServer*>(param)->_taskLoop();
}

/**
 * @brief Boucle principale de la tâche Web (Core 0).
 *
 * Traite les requêtes DNS captif et diffuse périodiquement
 * la télémétrie aux clients WebSocket sous forme de JSON.
 */
void BobWebServer::_taskLoop() {
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastBroadcast = xTaskGetTickCount();

    for (;;) {
        /* Traitement du DNS captif (non bloquant) */
        _dns.processNextRequest();

        /* Nettoyage des clients WebSocket déconnectés */
        _ws.cleanupClients();

        /* Broadcast périodique de la télémétrie via WebSocket */
        TickType_t now = xTaskGetTickCount();
        if ((now - lastBroadcast) >= pdMS_TO_TICKS(TASK_WS_BROADCAST_MS)) {
            lastBroadcast = now;

            /* Construire le JSON de télémétrie seulement si des clients sont connectés */
            if (_ws.count() > 0) {
                JsonDocument doc;

                doc["wdg"] = g_serial.isWatchdogTriggered();
                doc["physical_outputs_enabled"] = g_pwm.isPhysicalOutputsEnabled();

                /* Informations de connexion Wi-Fi (STA + AP) */
                {
                    JsonObject wifi = doc["wifi_info"].to<JsonObject>();
                    /* Mode STA (connexion à un réseau externe) */
                    bool staUp = (WiFi.status() == WL_CONNECTED);
                    wifi["sta_connected"] = staUp;
                    if (staUp) {
                        wifi["sta_ssid"]    = WiFi.SSID();
                        wifi["sta_ip"]      = WiFi.localIP().toString();
                        wifi["sta_gateway"] = WiFi.gatewayIP().toString();   /* Serveur DHCP */
                        wifi["sta_mask"]    = WiFi.subnetMask().toString();
                        wifi["sta_rssi"]    = WiFi.RSSI();
                        wifi["sta_mac"]     = WiFi.macAddress();
                    } else {
                        wifi["sta_ssid"]    = "";
                        wifi["sta_ip"]      = "—";
                        wifi["sta_gateway"] = "—";
                        wifi["sta_mask"]    = "—";
                        wifi["sta_rssi"]    = 0;
                        wifi["sta_mac"]     = WiFi.macAddress();
                    }
                    /* Mode AP (point d'accès local) */
                    wifi["ap_ssid"] = String(AP_SSID);
                    wifi["ap_ip"]   = WiFi.softAPIP().toString();
                    wifi["ap_clients"] = WiFi.softAPgetStationNum();
                }

                /* 16 valeurs PWM actuelles */
                uint16_t pwmVals[NUM_PWM_CHANNELS];
                g_pwm.getCurrentValues(pwmVals);
                JsonArray pwmArr = doc["pwm"].to<JsonArray>();
                for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
                    pwmArr.add(pwmVals[i]);
                }

                /* Données IMU (Euler) */
                IMUData imu;
                g_sensors.getIMUData(imu);
                JsonObject imuObj = doc["imu"].to<JsonObject>();
                imuObj["roll"]  = roundf(imu.euler[0] * 10.0f) / 10.0f;
                imuObj["pitch"] = roundf(imu.euler[1] * 10.0f) / 10.0f;
                imuObj["yaw"]   = roundf(imu.euler[2] * 10.0f) / 10.0f;
                imuObj["north_offset"] = roundf(imu.heading_offset * 10.0f) / 10.0f;  /* tare Nord */
                imuObj["mag_active"]   = imu.mag_active;   /* 1 = cap magnétique (Rotation Vector 9-DOF) */
                imuObj["mag_cal"]      = imu.mag_cal;      /* calibration magnétomètre SH-2 (0–3) */
                imuObj["mag_cfg_ok"]   = imu.mag_cfg_ok;   /* diagnostic : Set Feature RV transmis (SH-2 sans accusé) */
                imuObj["rv_reports"]   = (uint32_t)imu.rv_reports;   /* diagnostic : rapports RV reçus */
                imuObj["grv_reports"]  = (uint32_t)imu.grv_reports;  /* diagnostic : rapports GRV reçus */

                /* Pression, profondeur et altitude (MS5803 avec tare surface) */
                PressureData press;
                g_sensors.getPressureData(press);
                JsonObject pressObj = doc["press"].to<JsonObject>();
                pressObj["mbar"]     = press.pressure_mbar / 10.0f;
                pressObj["temp"]     = press.temperature / 100.0f;
                pressObj["surface"]  = roundf(press.surface_mbar * 100.0f) / 100.0f;  /* tare P_surface */
                pressObj["depth"]    = roundf(press.depth_m * 100.0f) / 100.0f;       /* profondeur eau douce */
                pressObj["baro_alt"] = roundf(press.baro_alt_m * 100.0f) / 100.0f;    /* altitude barométrique */
                pressObj["immersed"] = press.immersed;                                 /* statut immersion */
                pressObj["alt"]      = roundf(press.altitude_m * 100.0f) / 100.0f;     /* repère PID (compat) */

                /* Puissance (3 wattmètres) */
                PowerData pwr;
                g_sensors.getPowerData(pwr);
                JsonArray pwrArr = doc["power"].to<JsonArray>();
                for (uint8_t i = 0; i < 3; i++) {
                    JsonObject ch = pwrArr.add<JsonObject>();
                    ch["v"] = pwr.voltage[i];
                    ch["i"] = pwr.current[i];
                }

                /* Statut capteurs (badges diagnostic) */
                SensorBankStatus st;
                g_sensors.getSensorStatus(st);
                JsonObject sens = doc["sensors"].to<JsonObject>();
                sens["bno085"]  = (uint8_t)st.bno085;
                sens["bno085_err"] = g_sensors.getLastBNO085Error();   /* diagnostic sans câble USB */
                sens["ms5803"]  = (uint8_t)st.ms5803;
                sens["ina0"] = (uint8_t)st.ina226[0];
                sens["ina1"] = (uint8_t)st.ina226[1];
                sens["ina2"] = (uint8_t)st.ina226[2];
                sens["pca"]  = (uint8_t)st.pca9685;

                /* Statut liaison série RPi5 (interface active + heartbeat) */
                JsonObject rpi = doc["rpi_link"].to<JsonObject>();
                rpi["interface"] = g_serial.getInterfaceLabel();
                rpi["connected"] = !g_serial.isWatchdogTriggered();

                /* Contrôleur de vol (mode + maître) */
                doc["flight_mode"] = g_flightCtrl.getActiveMode();
                doc["rpi_master"]  = g_flightCtrl.isRPi5Master();

                /* Sérialisation et envoi */
                String json;
                serializeJson(doc, json);
                _ws.textAll(json);
            }
        }

        /* Pause courte pour laisser le CPU respirer */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
