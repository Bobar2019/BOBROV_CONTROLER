/**
 * @file web_server.cpp
 * @brief Implémentation du serveur Web embarqué (HTTP, WebSocket, DNS captif).
 *
 * Gère le point d'accès Wi-Fi, le portail captif DNS, les endpoints REST
 * (/api/wifi/scan, /api/wifi/connect, /api/settings/save, /api/comm/config,
 * /api/pwm/output-enable, /api/pwm/config, /api/gpio/config, /api/gpio/test,
 * /api/gpio-inputs/config, /api/led/config, /api/names, /api/i2c/scan,
 * /api/i2c/config, /api/i2c/routing, /api/ota/firmware, /api/ota/filesystem,
 * /api/system/status, /api/qnh/config, /api/qnh/refresh, /api/calibrate_qnh,
 * /api/motors/config, /api/motors/check)
 * et le serveur WebSocket diffusant la télémétrie temps réel en JSON.
 *
 * @author Didier Dero
 * @version 1.0.1
 */

#include "web_server.h"
#include "sensor_driver.h"
#include "pwm_controller.h"
#include "gpio_outputs.h"
#include "serial_comm.h"
#include "config.h"
#include "protocol.h"
#include "flight_controller.h"
#include "motor_manager.h"      /* g_motors : config NVS + check + télémétrie (v1.1.0) */
#include "status_io.h"          /* g_statusIo : entrées sécurité + bandeau LED WS2812 (v1.6.0) */
#include "audio_manager.h"      /* g_audio : mélodie de démarrage buzzer LEDC (v1.8.0) */
#include "simulator.h"          /* g_sim : simulateur de test virtuel (établi) */
#include "i2c_router.h"         /* g_i2cRouter : carte de routage Plug & Play (v1.2.0) */

#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

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
    , _qnhAutoEnable(false)
    , _qnhBootApplied(false)
    , _qnhFetchRunning(false)
    , _qnhLastFetchOk(false)
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

    /* QNH (v1.0.1) : option et dernière valeur en NVS — la valeur est
     * appliquée immédiatement au capteur (altitude cohérente même sans
     * Internet) ; la requête Open-Meteo de démarrage sera lancée par la
     * tâche Web (différée de QNH_BOOT_FETCH_DELAY_MS) si l'option est activée. */
    {
        Preferences prefs;
        prefs.begin("config", true);
        _qnhAutoEnable = prefs.getBool("qnh_auto_enable", false);
        float savedQnh = prefs.getFloat("current_qnh", QNH_DEFAULT_MBAR);
        float savedHwOffset = prefs.getFloat("hw_offset", 0.0f);
        prefs.end();
        if (savedQnh >= QNH_MIN_MBAR && savedQnh <= QNH_MAX_MBAR) {
            g_sensors.setQnh(savedQnh);
        }
        if (fabsf(savedHwOffset) <= MS5803_HW_OFFSET_MAX_MBAR) {
            g_sensors.setHwOffset(savedHwOffset);
        }
        Serial.println("[WEB] QNH : calibration auto "
                       + String(_qnhAutoEnable ? "activee" : "desactivee")
                       + " — valeur appliquee : " + String(savedQnh, 2) + " mbar"
                       + " — offset materiel : " + String(savedHwOffset, 2) + " mbar");
    }

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

    /* REST : Typage des canaux PWM 10-15 (GET + POST) — appliqué à chaud */
    _server.on("/api/pwm/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handlePwmConfigGet(request); });
    _server.on("/api/pwm/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handlePwmConfigPost(request); });

    /* REST : Assignation des 4 sorties GPIO ON/OFF (GET + POST) — appliquée au boot */
    _server.on("/api/gpio/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleGpioConfigGet(request); });
    _server.on("/api/gpio/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleGpioConfigPost(request); });

    /* REST : Test manuel des 4 sorties GPIO ON/OFF (masque 4 bits, établi) */
    _server.on("/api/gpio/test", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleGpioTestPost(request); });

    /* REST : Entrées numériques (v1.6.0) — Armement / Voie d'eau (GET + POST).
     * Broches et polarités (actif GND / actif +3,3 V) appliquées au boot →
     * redémarrage différé 3 s si changement ; l'alarme voie d'eau force le
     * Retour Surface côté firmware. */
    _server.on("/api/gpio-inputs/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleGpioInputsConfigGet(request); });
    _server.on("/api/gpio-inputs/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleGpioInputsConfigPost(request); });

    /* REST : Signalisation LED WS2812 (v1.6.0) — ligne de données et nombre
     * de LEDs (boot) + matrice 5 états (effet, couleur, luminosité par état)
     * appliquée à chaud, GET + POST. */
    _server.on("/api/led/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleLedConfigGet(request); });
    _server.on("/api/led/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleLedConfigPost(request); });

    /* REST : Acoustique / Buzzer (v1.8.0) — GPIO du buzzer et activation de
     * la mélodie de démarrage (GET + POST). Paramètres appliqués au prochain
     * démarrage, sans redémarrage immédiat : la mélodie ne joue qu'au boot
     * et relit la NVS à ce moment. */
    _server.on("/api/audio/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleAudioConfigGet(request); });
    _server.on("/api/audio/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleAudioConfigPost(request); });

    /* REST : Noms personnalisés des sorties (16 PWM + 4 GPIO, NVS — GET + POST) */
    _server.on("/api/names", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleNamesGet(request); });
    _server.on("/api/names", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleNamesPost(request); });

    /* REST : Calibration altimétrique QNH (v1.0.1) — Open-Meteo, tâche détachée.
     * GET/POST config = option NVS ; POST refresh = requête manuelle forcée. */
    _server.on("/api/qnh/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleQnhConfigGet(request); });
    _server.on("/api/qnh/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleQnhConfigPost(request); });
    _server.on("/api/qnh/refresh", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleQnhRefreshPost(request); });
    /* Alias historique demandé par la spec : calibration QNH + offset matériel. */
    _server.on("/api/calibrate_qnh", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleQnhRefreshPost(request); });

    /* REST : Gestion de sécurité moteurs (v1.1.0) — GET/POST config = 5
     * paramètres NVS (check démarrage, détection surintensité, remontée
     * d'urgence, seuil I, seuil V_BAT) ; POST check = relance manuelle. */
    _server.on("/api/motors/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleMotorsConfigGet(request); });
    _server.on("/api/motors/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleMotorsConfigPost(request); });
    _server.on("/api/motors/check", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleMotorsCheckPost(request); });

    /* REST : Simulateur de test virtuel (établi) — GET = valeurs courantes +
     * bornes des sliders ; POST = injection (toggle, tensions B1/B2, courants
     * H/V, blocages M1-M8). État RAM uniquement, jamais NVS. */
    _server.on("/api/simulator", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleSimulatorGet(request); });
    _server.on("/api/simulator", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleSimulatorPost(request); });

    /* REST : Scan I2C (GET) */
    _server.on("/api/i2c/scan", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleI2CScan(request); });

    /* REST : Configuration broches I2C (GET + POST) */
    _server.on("/api/i2c/config", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleI2CConfigGet(request); });
    _server.on("/api/i2c/config", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleI2CConfigPost(request); });

    /* REST : Routage I2C Plug & Play (v1.2.0) — GET = carte courante, POST =
     * nouvelle carte (une clé par module, 1 = Wire / 2 = Wire1) → NVS +
     * redémarrage différé 3 s : le routage ne s'applique qu'au boot (le bus
     * du PCA9685 porte les sorties PWM 100 Hz — jamais de re-routage à chaud). */
    _server.on("/api/i2c/routing", HTTP_GET,
        [this](AsyncWebServerRequest* request) { _handleI2CRoutingGet(request); });
    _server.on("/api/i2c/routing", HTTP_POST,
        [this](AsyncWebServerRequest* request) { _handleI2CRoutingPost(request); });

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
        obj["bus"]     = d.bus;    /* 1 = Wire | 2 = Wire1 — modules répartis par le routage Plug & Play */
        obj["name"]    = String(d.name);
    }
    doc["sda"]  = g_sensors.getCurrentSDA();
    doc["scl"]  = g_sensors.getCurrentSCL();
    doc["sda2"] = g_sensors.getCurrent2SDA();
    doc["scl2"] = g_sensors.getCurrent2SCL();
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

/* -------------------------------------------------------------------------
 * Routage I2C Plug & Play (v1.3.0) — clés API de l'endpoint /api/i2c/routing.
 *
 * L'ORDRE suit exactement l'enum I2CDeviceId (i2c_router.h) : ne jamais
 * réordonner, la compatibilité frontend/firmware en dépend.
 * Chaque module porte DEUX paramètres : la clé ci-dessous (bus) et la même
 * clé suffixée « _addr » (adresse I2C, décimale ou 0x hexadécimale).
 * ------------------------------------------------------------------------- */
static const char* const kRoutingKeys[I2C_DEV_COUNT] = {
    "bno085", "ina3221", "ina226_2", "ina226_3", "ms5803", "pca9685"
};

/* Parse une adresse I2C (0x01–0x7F) depuis une valeur API : décimale
 * (ex. « 74 ») ou hexadécimale (ex. « 0x4A »). Retourne 0 si invalide. */
static uint8_t _parseI2CAddr(const String& value) {
    const char* s = value.c_str();
    long v;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        v = strtol(s + 2, nullptr, 16);
    } else {
        v = strtol(s, nullptr, 10);
    }
    if (v < 0x01 || v > 0x7F) return 0;
    return (uint8_t)v;
}

/* -------------------------------------------------------------------------
 * Handler GET /api/i2c/routing — Carte de routage courante (v1.3.0).
 *
 * Retourne le couple (bus, adresse) assigné de chaque module (avec nom et
 * adresse pour le rendu de la table « Routage I2C Plug & Play »), les
 * broches actuelles des deux bus, le bus protégé du PCA9685 (pwm_bus),
 * l'état de conflit généralisé (deux modules même bus + même adresse —
 * possible seulement via une NVS éditée à la main, l'API de sauvegarde
 * refuse ce cas) ET le dernier scan I2C mémorisé (champ « scan ») : les
 * selects d'adresses de l'interface proposent ces adresses découvertes
 * sans relancer de scan à chaque chargement de page.
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleI2CRoutingGet(AsyncWebServerRequest* request) {
    String response;
    JsonDocument doc;
    doc["pwm_bus"] = g_i2cRouter.pwmBus();
    doc["conflict"] = g_i2cRouter.hasAddressConflict();
    doc["sda"]  = g_sensors.getCurrentSDA();
    doc["scl"]  = g_sensors.getCurrentSCL();
    doc["sda2"] = g_sensors.getCurrent2SDA();
    doc["scl2"] = g_sensors.getCurrent2SCL();

    JsonArray arr = doc["devices"].to<JsonArray>();
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;
        const uint8_t addr = g_i2cRouter.addrOf(dev);
        JsonObject obj = arr.add<JsonObject>();
        char hex[6];
        snprintf(hex, sizeof(hex), "0x%02X", addr);
        obj["key"]      = kRoutingKeys[d];
        obj["name"]     = I2CRouter::nameOf(dev);
        obj["addr"]     = hex;
        obj["addr_dec"] = addr;
        obj["bus"]      = g_i2cRouter.busOf(dev);
    }

    /* Dernier scan I2C (boot ou scan manuel) — même format que /api/i2c/scan */
    JsonArray scanArr = doc["scan"].to<JsonArray>();
    const std::vector<I2CDeviceInfo> lastScan = g_sensors.getLastScan();
    for (const I2CDeviceInfo& dev : lastScan) {
        JsonObject so = scanArr.add<JsonObject>();
        so["address"] = String(dev.hexStr);
        so["dec"]     = dev.address;
        so["bus"]     = dev.bus;
        so["name"]    = String(dev.name);
    }
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler POST /api/i2c/routing — Enregistre la carte de routage complète.
 *
 * Les DOUZE clés sont requises (bus ET adresse des six modules, v1.3.0) :
 *   Form : bno085=1&bno085_addr=74&ina3221=1&ina3221_addr=64&...
 *   JSON : {"bno085":1,"bno085_addr":74,...}
 *   - bus : 1 = Wire ou 2 = Wire1 ;
 *   - adresse : 0x01–0x7F, décimale (74) ou hexadécimale (0x4A).
 *
 * Refus 400 si deux modules aboutissent sur le MÊME bus à la MÊME adresse
 * (conflit généralisé v1.3.0 — cas historique INA3221/PCA9685 à 0x40 ;
 * save() refuse aussi structurellement ce cas). La carte en mémoire est
 * restaurée si la NVS refuse l'écriture : pwmBus() dirige les récupérations
 * watchdog, il doit refléter la topologie RÉELLEMENT active jusqu'au
 * redémarrage. Application au boot suivant uniquement — le re-routage à
 * chaud est interdit (le bus du PCA9685 porte les PWM 100 Hz de la tâche
 * de contrôle) : sauvegarde NVS vérifiée par relecture puis redémarrage
 * différé de 3 s, même discipline que les broches SDA/SCL.
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleI2CRoutingPost(AsyncWebServerRequest* request) {
    uint8_t wantedBus[I2C_DEV_COUNT];
    uint8_t wantedAddr[I2C_DEV_COUNT];
    bool got[I2C_DEV_COUNT] = {false};

    /* Accepte form-urlencoded / query params : une paire de clés par module */
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const char* keyBus = kRoutingKeys[d];
        const String keyAddr = String(keyBus) + "_addr";
        const AsyncWebParameter* pBus = nullptr;
        const AsyncWebParameter* pAddr = nullptr;
        if (request->hasParam(keyBus, true)) pBus = request->getParam(keyBus, true);
        else if (request->hasParam(keyBus)) pBus = request->getParam(keyBus);
        if (request->hasParam(keyAddr.c_str(), true)) pAddr = request->getParam(keyAddr.c_str(), true);
        else if (request->hasParam(keyAddr.c_str())) pAddr = request->getParam(keyAddr.c_str());
        if (pBus != nullptr && pAddr != nullptr) {
            wantedBus[d]  = (uint8_t)pBus->value().toInt();
            wantedAddr[d] = _parseI2CAddr(pAddr->value());
            got[d] = true;
        }
    }

    /* Fallback JSON body : {"bno085":1,"bno085_addr":74,...} */
    bool any = false;
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        if (got[d]) any = true;
    }
    if (!any && request->hasParam("plain", true)) {
        const AsyncWebParameter* body = request->getParam("plain", true);
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, body->value());
        if (!err) {
            for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
                const String keyAddr = String(kRoutingKeys[d]) + "_addr";
                if (doc[kRoutingKeys[d]].is<uint8_t>() && doc[keyAddr].is<uint8_t>()) {
                    wantedBus[d]  = doc[kRoutingKeys[d]].as<uint8_t>();
                    wantedAddr[d] = doc[keyAddr].as<uint8_t>();
                    got[d] = true;
                }
            }
        }
    }

    /* Les six modules doivent être assignés (bus ET adresse) : un POST
     * partiel ne doit jamais pouvoir replier silencieusement les modules
     * omis sur leurs défauts. */
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        if (!got[d]) {
            request->send(400, "application/json",
                          "{\"error\":\"Parametres requis : bus ET adresse des 6 modules "
                          "(<module> et <module>_addr)\"}");
            return;
        }
    }
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        if (wantedBus[d] != 1 && wantedBus[d] != 2) {
            request->send(400, "application/json", "{\"error\":\"Bus invalide (1 ou 2)\"}");
            return;
        }
        if (wantedAddr[d] < 0x01 || wantedAddr[d] > 0x7F) {
            request->send(400, "application/json",
                          "{\"error\":\"Adresse invalide (0x01-0x7F)\"}");
            return;
        }
    }

    /* Appliquer la nouvelle carte en mémoire, en gardant une copie de
     * secours de la carte active (restaurée sur tout échec en aval). */
    uint8_t backupBus[I2C_DEV_COUNT];
    uint8_t backupAddr[I2C_DEV_COUNT];
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;
        backupBus[d]  = g_i2cRouter.busOf(dev);
        backupAddr[d] = g_i2cRouter.addrOf(dev);
        g_i2cRouter.setBusOf(dev, wantedBus[d]);
        g_i2cRouter.setAddrOf(dev, wantedAddr[d]);
    }

    /* Garde-fou structurel : deux modules jamais sur le même bus à la même
     * adresse (historique : INA3221 et PCA9685, tous deux 0x40). */
    I2CDeviceId ca, cb;
    if (g_i2cRouter.firstConflict(ca, cb)) {
        const uint8_t confBus  = g_i2cRouter.busOf(ca);
        const uint8_t confAddr = g_i2cRouter.addrOf(ca);
        for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
            g_i2cRouter.setBusOf((I2CDeviceId)d, backupBus[d]);
            g_i2cRouter.setAddrOf((I2CDeviceId)d, backupAddr[d]);
        }
        char errMsg[192];
        snprintf(errMsg, sizeof(errMsg),
                 "{\"error\":\"Conflit adresse : %s et %s sur le bus %u a la meme adresse 0x%02X\"}",
                 I2CRouter::shortNameOf(ca), I2CRouter::shortNameOf(cb),
                 (unsigned)confBus, (unsigned)confAddr);
        request->send(400, "application/json", errMsg);
        return;
    }

    /* Sauvegarde NVS (vérifiée par relecture côté routeur). En cas d'échec,
     * restaurer la carte en mémoire — pwmBus() dirige les récupérations
     * watchdog et ne doit pas dévier de la topologie réellement active. */
    if (!g_i2cRouter.save()) {
        for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
            g_i2cRouter.setBusOf((I2CDeviceId)d, backupBus[d]);
            g_i2cRouter.setAddrOf((I2CDeviceId)d, backupAddr[d]);
        }
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (routage non enregistre)\"}");
        return;
    }

    Serial.println("[WEB] Routage I2C Plug & Play enregistré (NVS, bus + adresses) — redémarrage dans 3 s");
    _scheduleRestart(3000);

    String response;
    JsonDocument resp;
    resp["status"]     = "ok";
    resp["restart_ms"] = 3000;
    JsonArray arr = resp["devices"].to<JsonArray>();
    for (uint8_t d = 0; d < I2C_DEV_COUNT; d++) {
        const I2CDeviceId dev = (I2CDeviceId)d;
        JsonObject obj = arr.add<JsonObject>();
        char hex[6];
        snprintf(hex, sizeof(hex), "0x%02X", g_i2cRouter.addrOf(dev));
        obj["key"]      = kRoutingKeys[d];
        obj["bus"]      = g_i2cRouter.busOf(dev);
        obj["addr"]     = hex;
        obj["addr_dec"] = g_i2cRouter.addrOf(dev);
    }
    serializeJson(resp, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler GET /api/pwm/config — Typage actuel des canaux 10–15.
 *
 * Retourne le masque 6 bits (bit n ↔ canal 10+n, 1 = bidirectionnel) et les
 * bornes des canaux typables — utilisé par l'onglet Paramètres et le PWM
 * Monitor de l'interface.
 * ------------------------------------------------------------------------- */
void BobWebServer::_handlePwmConfigGet(AsyncWebServerRequest* request) {
    String response;
    JsonDocument doc;
    doc["type_mask"]     = g_pwm.getChannelTypeMask();
    doc["first_channel"] = PWM_TYPE_FIRST_CHANNEL;
    doc["num_channels"]  = PWM_TYPE_NUM_CHANNELS;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler POST /api/pwm/config — Enregistre le typage des canaux 10–15.
 *
 * Le masque 6 bits (bit n ↔ canal 10+n, 1 = bidirectionnel) est persisté en
 * NVS (clé « pwm_types ») puis appliqué à chaud : seule la position de
 * sécurité des canaux concernés change (1500 µs bidir / 0 % unidir), aucun
 * redémarrage n'est nécessaire.
 *
 * Form : mask=51  |  JSON : {"mask":51}
 * ------------------------------------------------------------------------- */
void BobWebServer::_handlePwmConfigPost(AsyncWebServerRequest* request) {
    int maskIn = -1;

    /* Accepte form-urlencoded (mask=X) ou query params, avec fallback JSON body */
    if (request->hasParam("mask", true)) {
        maskIn = request->getParam("mask", true)->value().toInt();
    } else if (request->hasParam("mask")) {
        maskIn = request->getParam("mask")->value().toInt();
    } else if (request->hasParam("plain", true)) {
        const AsyncWebParameter* body = request->getParam("plain", true);
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, body->value());
        if (err || !doc["mask"].is<int>()) {
            request->send(400, "application/json", "{\"error\":\"JSON invalide\"}");
            return;
        }
        maskIn = doc["mask"].as<int>();
    } else {
        request->send(400, "application/json", "{\"error\":\"Paramètre mask requis\"}");
        return;
    }

    /* Masque 6 bits : bit n ↔ canal 10+n (1 = bidirectionnel) */
    if (maskIn < 0 || maskIn > (int)((1u << PWM_TYPE_NUM_CHANNELS) - 1)) {
        request->send(400, "application/json", "{\"error\":\"Masque invalide (0-63)\"}");
        return;
    }

    if (!g_pwm.saveChannelTypes((uint8_t)maskIn)) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (masque non enregistre)\"}");
        return;
    }
    Serial.println("[WEB] Typage PWM canaux 10-15 : masque=0x" + String(maskIn, HEX)
                   + " — appliqué à chaud");

    String response;
    JsonDocument resp;
    resp["status"]    = "ok";
    resp["type_mask"] = g_pwm.getChannelTypeMask();
    serializeJson(resp, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler GET /api/gpio/config — Broches et état des 4 sorties ON/OFF.
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleGpioConfigGet(AsyncWebServerRequest* request) {
    uint8_t pins[GPIO_NUM_OUTPUTS];
    g_gpio.getPins(pins);

    String response;
    JsonDocument doc;
    JsonArray arr = doc["pins"].to<JsonArray>();
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) arr.add(pins[i]);
    doc["state"] = g_gpio.getState();
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler POST /api/gpio/config — Enregistre l'assignation des 4 sorties.
 *
 * Validation : 4 broches 0-48, uniques, hors GPIO 19/20 (USB natif), hors
 * broches des deux bus I2C actifs et de l'UART1 (liaison RPi 5). Sauvegarde
 * NVS vérifiée par relecture puis redémarrage différé de 3 s : les broches
 * ne sont appliquées qu'au boot (init OFF, puis pilotage par gpio_cmd).
 *
 * Form : p1=15&p2=16&p3=17&p4=18  |  JSON : {"p1":15,"p2":16,"p3":17,"p4":18}
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleGpioConfigPost(AsyncWebServerRequest* request) {
    static const char* const KEYS[GPIO_NUM_OUTPUTS] = { "p1", "p2", "p3", "p4" };
    int pinIn[GPIO_NUM_OUTPUTS] = { -1, -1, -1, -1 };

    /* Accepte form-urlencoded (p1..p4) ou query params, avec fallback JSON body */
    if (request->hasParam(KEYS[0], true) || request->hasParam(KEYS[0])) {
        for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
            const AsyncWebParameter* p = request->hasParam(KEYS[i], true)
                ? request->getParam(KEYS[i], true) : request->getParam(KEYS[i]);
            if (!p) {
                request->send(400, "application/json", "{\"error\":\"Paramètres p1-p4 requis\"}");
                return;
            }
            pinIn[i] = p->value().toInt();
        }
    } else if (request->hasParam("plain", true)) {
        const AsyncWebParameter* body = request->getParam("plain", true);
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, body->value());
        if (err) {
            request->send(400, "application/json", "{\"error\":\"JSON invalide\"}");
            return;
        }
        for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
            if (!doc[KEYS[i]].is<int>()) {
                request->send(400, "application/json", "{\"error\":\"JSON invalide (p1-p4 requis)\"}");
                return;
            }
            pinIn[i] = doc[KEYS[i]].as<int>();
        }
    } else {
        request->send(400, "application/json", "{\"error\":\"Paramètres p1-p4 requis\"}");
        return;
    }

    /* Broches réservées : bus I2C actifs + UART1 (liaison RPi 5) + entrées
     * numériques, ligne de données du bandeau LED et GPIO du buzzer déjà
     * assignés (v1.6.0/v1.8.0 — exclusions croisées ; la sentinelle 0xFF
     * « non assigné » ne peut jamais égaler un GPIO valide). */
    const uint8_t reserved[] = {
        g_sensors.getCurrentSDA(),  g_sensors.getCurrentSCL(),
        g_sensors.getCurrent2SDA(), g_sensors.getCurrent2SCL(),
        UART1_RX_PIN, UART1_TX_PIN,
        (uint8_t)g_statusIo.getArmPin(),
        (uint8_t)g_statusIo.getWaterPin(),
        (uint8_t)g_statusIo.getLedPin(),
        (uint8_t)g_audio.getBuzzerPin()
    };

    uint8_t pins[GPIO_NUM_OUTPUTS];
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        int p = pinIn[i];
        if (p < 0 || p > 48 || p == 19 || p == 20) {
            request->send(400, "application/json",
                          "{\"error\":\"GPIO invalide (0-48, hors 19/20 USB)\"}");
            return;
        }
        for (uint8_t k = 0; k < sizeof(reserved); k++) {
            if ((uint8_t)p == reserved[k]) {
                request->send(400, "application/json",
                              "{\"error\":\"GPIO reserve (I2C actif ou UART1)\"}");
                return;
            }
        }
        for (uint8_t j = 0; j < i; j++) {
            if ((uint8_t)p == pins[j]) {
                request->send(400, "application/json", "{\"error\":\"GPIO en double\"}");
                return;
            }
        }
        pins[i] = (uint8_t)p;
    }

    if (!g_gpio.savePins(pins)) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (broches non enregistrees)\"}");
        return;
    }
    Serial.println("[WEB] Config GPIO ON/OFF : p1=" + String(pins[0]) + ", p2=" + String(pins[1])
                   + ", p3=" + String(pins[2]) + ", p4=" + String(pins[3])
                   + " — redémarrage dans 3 s");
    _scheduleRestart(3000);

    String response;
    JsonDocument resp;
    resp["status"] = "ok";
    JsonArray arr = resp["pins"].to<JsonArray>();
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) arr.add(pins[i]);
    resp["restart_ms"] = 3000;
    serializeJson(resp, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler POST /api/gpio/test — Test manuel des 4 sorties ON/OFF.
 *
 * Applique directement un masque 4 bits (bit 0 = sortie 1) sur les sorties,
 * sans passer par la trame descendante — utilisé par les boutons de test de
 * l'onglet Paramètres (établi, sans RPi 5). L'état réel est relu et renvoyé
 * dans « gpio_state ». Si le RPi 5 est maître, la prochaine trame écrasera le
 * masque : le champ « rpi5_master » permet à l'interface d'avertir.
 *
 * Form : mask=5  |  JSON : {"mask":5}
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleGpioTestPost(AsyncWebServerRequest* request) {
    int maskIn = -1;

    /* Accepte form-urlencoded (mask=X) ou query params, avec fallback JSON body */
    if (request->hasParam("mask", true)) {
        maskIn = request->getParam("mask", true)->value().toInt();
    } else if (request->hasParam("mask")) {
        maskIn = request->getParam("mask")->value().toInt();
    } else if (request->hasParam("plain", true)) {
        const AsyncWebParameter* body = request->getParam("plain", true);
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, body->value());
        if (err || !doc["mask"].is<int>()) {
            request->send(400, "application/json", "{\"error\":\"JSON invalide\"}");
            return;
        }
        maskIn = doc["mask"].as<int>();
    } else {
        request->send(400, "application/json", "{\"error\":\"Parametre mask requis\"}");
        return;
    }

    /* Masque 4 bits (bit 0 = sortie 1) */
    if (maskIn < 0 || maskIn > 0x0F) {
        request->send(400, "application/json", "{\"error\":\"Masque invalide (0-15)\"}");
        return;
    }

    g_gpio.applyCommand((uint8_t)maskIn);
    Serial.println("[WEB] Test sorties ON/OFF : masque=0x" + String(maskIn, HEX)
                   + " — état réel relu=0x" + String(g_gpio.getState(), HEX));

    String response;
    JsonDocument resp;
    resp["status"]      = "ok";
    resp["gpio_state"]  = g_gpio.getState();
    resp["rpi5_master"] = g_flightCtrl.isRPi5Master();
    serializeJson(resp, response);
    request->send(200, "application/json", response);
}

/* =========================================================================
 * ENTRÉES NUMÉRIQUES DE SÉCURITÉ + BANDEAU LED WS2812 (v1.6.0)
 * =========================================================================
 *
 * Endpoints des deux blocs de l'onglet Paramètres :
 * - /api/gpio-inputs/config : entrées « Armement » (interrupteur magnétique)
 *   et « Détecteur Voie d'eau », à polarité configurable par entrée (contact
 *   vers GND = active, INPUT_PULLUP — défaut ; signal actif +3,3 V,
 *   INPUT_PULLDOWN) ; l'alarme voie d'eau force le Retour Surface (override
 *   FlightController).
 * - /api/led/config : ligne de données WS2812, nombre de LEDs et matrice
 *   des 5 états (effet + couleur RGB + luminosité par état), consommée par
 *   la tâche StatusIO.
 *
 * Persistance NVS dans le module StatusIO (namespace « statusio »). Les
 * BROCHES et la POLARITÉ des entrées sont appliquées au boot (pull-up ou
 * pull-down / instance NeoPixel) : tout changement programme un redémarrage
 * différé de 3 s. La MATRICE LED (effets, couleurs et luminosités par état)
 * est appliquée à chaud, sans redémarrage.
 *
 * Exclusions croisées : une broche déjà utilisée par les sorties ON/OFF,
 * l'autre entrée ou le bandeau est refusée, comme les broches des bus I2C
 * actifs, l'UART1, les GPIO 19/20 (USB natif) et les GPIO 35-37 (bus SPI
 * flash/PSRAM interne des modules octaux). Valeur -1 = « non assigné ».
 * ========================================================================= */

/**
 * @brief Valide une broche GPIO assignable par l'utilisateur (v1.6.0).
 *
 * Refuse hors 0-48, les GPIO 19/20 (USB natif), les GPIO 35-37 (bus SPI
 * flash/PSRAM interne des modules octaux), les broches des bus I2C actifs
 * et de l'UART1 (liaison RPi 5), plus toute broche présente dans extra[]
 * (exclusions croisées entre modules). Les valeurs -1 et STATUS_IO_PIN_NONE
 * (0xFF) = « non assigné » sont acceptées.
 *
 * @return nullptr si la broche est valide, sinon le motif d'erreur.
 */
static const char* _userPinCheck(int pin, const uint8_t* extra, uint8_t numExtra) {
    if (pin == -1 || pin == (int)STATUS_IO_PIN_NONE) return nullptr;   /* non assigné */

    const uint8_t reserved[] = {
        g_sensors.getCurrentSDA(),  g_sensors.getCurrentSCL(),
        g_sensors.getCurrent2SDA(), g_sensors.getCurrent2SCL(),
        UART1_RX_PIN, UART1_TX_PIN
    };
    if (pin < 0 || pin > 48 || pin == 19 || pin == 20) {
        return "GPIO invalide (0-48, hors 19/20 USB)";
    }
    if (isMemoryBusPin((uint8_t)pin)) {
        return "GPIO reserve (bus flash/PSRAM interne 35-37)";
    }
    for (uint8_t k = 0; k < sizeof(reserved); k++) {
        if ((uint8_t)pin == reserved[k]) {
            return "GPIO reserve (I2C actif ou UART1)";
        }
    }
    for (uint8_t k = 0; k < numExtra; k++) {
        if (extra[k] != STATUS_IO_PIN_NONE && (uint8_t)pin == extra[k]) {
            return "GPIO deja utilise (entree/sortie ou bandeau LED)";
        }
    }
    return nullptr;
}

/* Handler GET /api/gpio-inputs/config — broches assignées + états live des
 * deux entrées (interrupteur d'armement fermé, alarme voie d'eau). */
void BobWebServer::_handleGpioInputsConfigGet(AsyncWebServerRequest* request) {
    JsonDocument doc;
    doc["arm_pin"]           = g_statusIo.getArmPin();
    doc["water_pin"]         = g_statusIo.getWaterPin();
    doc["arm_active_high"]   = g_statusIo.getArmActiveHigh();
    doc["water_active_high"] = g_statusIo.getWaterActiveHigh();
    doc["arm_closed"]        = g_statusIo.isArmClosed();
    doc["water_alarm"]       = g_statusIo.isWaterAlarm();
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* Handler POST /api/gpio-inputs/config — enregistre les deux broches
 * d'entrée et leurs polarités (NVS vérifiée par relecture côté StatusIO).
 * Champs absents = valeur courante. Redémarrage différé de 3 s UNIQUEMENT
 * si une broche ou une polarité change (appliquées au boot).
 *
 * Form : arm_pin=4&water_pin=5&arm_active_high=0&water_active_high=1
 * JSON : {"arm_pin":4,"water_pin":5,"arm_active_high":0} */
void BobWebServer::_handleGpioInputsConfigPost(AsyncWebServerRequest* request) {
    StatusIOConfig cfg;
    g_statusIo.getConfig(cfg);   /* base : champs absents conservent la valeur courante */

    auto readPin = [&request](const char* name, int8_t& out) {
        if (request->hasParam(name, true)) {
            out = (int8_t)request->getParam(name, true)->value().toInt();
        } else if (request->hasParam(name)) {
            out = (int8_t)request->getParam(name)->value().toInt();
        }
    };
    readPin("arm_pin",   cfg.armPin);
    readPin("water_pin", cfg.waterPin);

    /* Polarités : 0 = actif GND (défaut), 1 = actif +3,3 V */
    auto readFlag = [&request](const char* name, uint8_t& out) {
        int v = out;
        if (request->hasParam(name, true)) {
            v = request->getParam(name, true)->value().toInt();
        } else if (request->hasParam(name)) {
            v = request->getParam(name)->value().toInt();
        }
        out = (v != 0) ? 1 : 0;
    };
    readFlag("arm_active_high",   cfg.armActiveHigh);
    readFlag("water_active_high", cfg.waterActiveHigh);

    /* Exclusions croisées : sorties ON/OFF + bandeau LED + buzzer + l'autre entrée */
    uint8_t outPins[GPIO_NUM_OUTPUTS];
    g_gpio.getPins(outPins);
    const uint8_t buzzPin = (uint8_t)g_audio.getBuzzerPin();

    const char* err = nullptr;
    {
        uint8_t extra[GPIO_NUM_OUTPUTS + 3];
        memcpy(extra, outPins, GPIO_NUM_OUTPUTS);
        extra[GPIO_NUM_OUTPUTS]     = (uint8_t)(cfg.ledPin   < 0 ? STATUS_IO_PIN_NONE : cfg.ledPin);
        extra[GPIO_NUM_OUTPUTS + 1] = (uint8_t)(cfg.waterPin < 0 ? STATUS_IO_PIN_NONE : cfg.waterPin);
        extra[GPIO_NUM_OUTPUTS + 2] = buzzPin;
        err = _userPinCheck(cfg.armPin, extra, sizeof(extra));
    }
    if (err == nullptr) {
        uint8_t extra[GPIO_NUM_OUTPUTS + 3];
        memcpy(extra, outPins, GPIO_NUM_OUTPUTS);
        extra[GPIO_NUM_OUTPUTS]     = (uint8_t)(cfg.ledPin < 0 ? STATUS_IO_PIN_NONE : cfg.ledPin);
        extra[GPIO_NUM_OUTPUTS + 1] = (uint8_t)(cfg.armPin < 0 ? STATUS_IO_PIN_NONE : cfg.armPin);
        extra[GPIO_NUM_OUTPUTS + 2] = buzzPin;
        err = _userPinCheck(cfg.waterPin, extra, sizeof(extra));
    }
    if (err != nullptr) {
        String body = "{\"error\":\"" + String(err) + "\"}";
        request->send(400, "application/json", body);
        return;
    }

    /* Changement de broche ou de polarité = redémarrage différé (appliqués au boot) */
    const bool pinsChanged = (cfg.armPin != g_statusIo.getArmPin())
                          || (cfg.waterPin != g_statusIo.getWaterPin())
                          || ((cfg.armActiveHigh   != 0) != g_statusIo.getArmActiveHigh())
                          || ((cfg.waterActiveHigh != 0) != g_statusIo.getWaterActiveHigh());

    if (!g_statusIo.setConfig(cfg)) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (configuration non enregistree)\"}");
        return;
    }

    String response;
    JsonDocument resp;
    resp["status"]            = "ok";
    resp["arm_pin"]           = cfg.armPin;
    resp["water_pin"]         = cfg.waterPin;
    resp["arm_active_high"]   = cfg.armActiveHigh;
    resp["water_active_high"] = cfg.waterActiveHigh;
    resp["restart_ms"]        = pinsChanged ? 3000 : 0;
    serializeJson(resp, response);

    if (pinsChanged) {
        Serial.println("[WEB] Entrées numériques : arm=" + String(cfg.armPin)
                       + (cfg.armActiveHigh ? " (actif +3,3 V)" : " (actif GND)")
                       + ", water=" + String(cfg.waterPin)
                       + (cfg.waterActiveHigh ? " (actif +3,3 V)" : " (actif GND)")
                       + " — redémarrage dans 3 s");
        _scheduleRestart(3000);
    } else {
        Serial.println("[WEB] Entrées numériques : broches et polarités inchangées (NVS à jour, sans redémarrage)");
    }
    request->send(200, "application/json", response);
}

/* Handler GET /api/led/config — broche, nombre de LEDs, bornes des champs
 * et matrice des 5 états (effet + couleur RGB + luminosité par état).
 * « active_state » = état actuellement affiché (255 = bandeau inactif). */
void BobWebServer::_handleLedConfigGet(AsyncWebServerRequest* request) {
    StatusIOConfig cfg;
    g_statusIo.getConfig(cfg);

    JsonDocument doc;
    doc["led_pin"]      = cfg.ledPin;
    doc["led_count"]    = cfg.ledCount;
    doc["count_min"]    = LED_WS2812_NUM_MIN;
    doc["count_max"]    = LED_WS2812_NUM_MAX;
    doc["active_state"] = g_statusIo.getActiveLedState();
    JsonArray arr = doc["states"].to<JsonArray>();
    for (uint8_t i = 0; i < LED_STATE_COUNT; i++) {
        JsonObject st = arr.add<JsonObject>();
        st["effect"]     = cfg.led[i].effect;
        st["r"]          = cfg.led[i].r;
        st["g"]          = cfg.led[i].g;
        st["b"]          = cfg.led[i].b;
        st["brightness"] = cfg.led[i].brightness;
    }
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* Handler POST /api/led/config — enregistre la configuration du bandeau
 * (NVS vérifiée par relecture côté StatusIO). Champs absents = valeur
 * courante. Exclusions croisées identiques aux entrées. Redémarrage différé
 * de 3 s si la broche ou le nombre de LEDs change (instance NeoPixel créée
 * au boot) ; la matrice effets/couleurs/luminosités est appliquée à chaud.
 *
 * Form : led_pin=38&led_count=8&s0_effect=1&s0_color=0080ff&s0_bri=100&... */
void BobWebServer::_handleLedConfigPost(AsyncWebServerRequest* request) {
    StatusIOConfig cfg;
    g_statusIo.getConfig(cfg);   /* base : champs absents conservent la valeur courante */

    auto readStr = [&request](const char* name) -> String {
        if (request->hasParam(name, true)) return request->getParam(name, true)->value();
        if (request->hasParam(name))       return request->getParam(name)->value();
        return String();
    };
    auto readInt = [&request](const char* name, int fallback) -> int {
        if (request->hasParam(name, true)) return request->getParam(name, true)->value().toInt();
        if (request->hasParam(name))       return request->getParam(name)->value().toInt();
        return fallback;
    };

    /* Ligne de données (-1 = non assigné) */
    cfg.ledPin = (int8_t)readInt("led_pin", cfg.ledPin);

    /* Nombre de LEDs : bornes 1-100 */
    {
        int n = readInt("led_count", cfg.ledCount);
        if (n < LED_WS2812_NUM_MIN || n > LED_WS2812_NUM_MAX) {
            request->send(400, "application/json",
                          "{\"error\":\"Nombre de LEDs invalide (1-100)\"}");
            return;
        }
        cfg.ledCount = (uint8_t)n;
    }

    /* Validation de la broche (exclusions croisées : sorties + entrées + buzzer) */
    {
        uint8_t outPins[GPIO_NUM_OUTPUTS];
        g_gpio.getPins(outPins);
        uint8_t extra[GPIO_NUM_OUTPUTS + 3];
        memcpy(extra, outPins, GPIO_NUM_OUTPUTS);
        extra[GPIO_NUM_OUTPUTS]     = (uint8_t)(cfg.armPin   < 0 ? STATUS_IO_PIN_NONE : cfg.armPin);
        extra[GPIO_NUM_OUTPUTS + 1] = (uint8_t)(cfg.waterPin < 0 ? STATUS_IO_PIN_NONE : cfg.waterPin);
        extra[GPIO_NUM_OUTPUTS + 2] = (uint8_t)g_audio.getBuzzerPin();
        const char* err = _userPinCheck(cfg.ledPin, extra, sizeof(extra));
        if (err != nullptr) {
            String body = "{\"error\":\"" + String(err) + "\"}";
            request->send(400, "application/json", body);
            return;
        }
    }

    /* Matrice des 5 états : effet (0-3) + couleur hex « rrggbb » + luminosité % */
    static const char* const FX_KEYS[LED_STATE_COUNT] = {
        "s0_effect", "s1_effect", "s2_effect", "s3_effect", "s4_effect"
    };
    static const char* const COLOR_KEYS[LED_STATE_COUNT] = {
        "s0_color", "s1_color", "s2_color", "s3_color", "s4_color"
    };
    static const char* const BRI_KEYS[LED_STATE_COUNT] = {
        "s0_bri", "s1_bri", "s2_bri", "s3_bri", "s4_bri"
    };
    for (uint8_t i = 0; i < LED_STATE_COUNT; i++) {
        int fx = readInt(FX_KEYS[i], (int)cfg.led[i].effect);
        if (fx < LED_EFFECT_SOLID || fx > LED_EFFECT_OFF) {
            request->send(400, "application/json",
                          "{\"error\":\"Effet invalide (0-3)\"}");
            return;
        }
        cfg.led[i].effect = (uint8_t)fx;

        int bri = readInt(BRI_KEYS[i], (int)cfg.led[i].brightness);
        if (bri < LED_WS2812_BRI_MIN || bri > LED_WS2812_BRI_MAX) {
            request->send(400, "application/json",
                          "{\"error\":\"Luminosite invalide (1-100)\"}");
            return;
        }
        cfg.led[i].brightness = (uint8_t)bri;

        String col = readStr(COLOR_KEYS[i]);
        if (col.length() > 0) {
            if (col[0] == '#') col.remove(0, 1);
            if (col.length() != 6) {
                request->send(400, "application/json",
                              "{\"error\":\"Couleur invalide (format rrggbb)\"}");
                return;
            }
            char* end = nullptr;
            long v = strtol(col.c_str(), &end, 16);
            if (end == col.c_str() || *end != '\0' || v < 0 || v > 0xFFFFFF) {
                request->send(400, "application/json",
                              "{\"error\":\"Couleur invalide (format rrggbb)\"}");
                return;
            }
            cfg.led[i].r = (uint8_t)((v >> 16) & 0xFF);
            cfg.led[i].g = (uint8_t)((v >> 8)  & 0xFF);
            cfg.led[i].b = (uint8_t)( v        & 0xFF);
        }
    }

    /* Changement de broche ou de nombre de LEDs = redémarrage différé
     * (instance NeoPixel créée au boot) ; la matrice s'applique à chaud. */
    const bool pinsChanged = (cfg.ledPin != g_statusIo.getLedPin())
                          || (cfg.ledCount != g_statusIo.getLedCount());

    if (!g_statusIo.setConfig(cfg)) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (configuration non enregistree)\"}");
        return;
    }

    String response;
    JsonDocument resp;
    resp["status"]       = "ok";
    resp["led_pin"]      = cfg.ledPin;
    resp["led_count"]    = cfg.ledCount;
    resp["active_state"] = g_statusIo.getActiveLedState();
    resp["restart_ms"]   = pinsChanged ? 3000 : 0;
    serializeJson(resp, response);

    if (pinsChanged) {
        Serial.println("[WEB] Bandeau LED : pin=" + String(cfg.ledPin)
                       + ", count=" + String(cfg.ledCount) + " — redémarrage dans 3 s");
        _scheduleRestart(3000);
    } else {
        Serial.println("[WEB] Bandeau LED : matrice appliquée à chaud (broches inchangées)");
    }
    request->send(200, "application/json", response);
}

/* =========================================================================
 * ACOUSTIQUE / BUZZER — SIGNATURE SONORE DE DÉMARRAGE (v1.8.0)
 * =========================================================================
 *
 * Buzzer passif piloté par le timer LEDC : mélodie du thème « Rencontres du
 * 3ème type » (5 notes) jouée une seule fois au démarrage dans une tâche
 * éphémère (cf. audio_manager.h). GPIO et activation persistés en NVS
 * (namespace « audio », clés buzz_pin / melody_en), écriture vérifiée par
 * relecture côté AudioManager. Contrairement aux autres E/S, aucun
 * redémarrage n'est programmé après sauvegarde : la broche n'est sollicitée
 * qu'au moment de jouer la mélodie (prochain démarrage), la configuration
 * étant relue au boot.
 * ========================================================================= */

/* Handler GET /api/audio/config — GPIO du buzzer, activation et volume de
 * la mélodie de démarrage (relus de la NVS au boot). */
void BobWebServer::_handleAudioConfigGet(AsyncWebServerRequest* request) {
    AudioConfig cfg;
    g_audio.getConfig(cfg);
    JsonDocument doc;
    doc["buzzer_pin"]     = cfg.buzzerPin;
    doc["melody_enabled"] = (cfg.melodyEnabled != 0);
    doc["melody_volume"]  = cfg.melodyVolume;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* Handler POST /api/audio/config — enregistre le GPIO du buzzer, l'activation
 * et le volume de la mélodie de démarrage (NVS vérifiée par relecture).
 * Champs absents = valeur courante. Aucun redémarrage : appliqué au
 * prochain démarrage (la mélodie relit la NVS au boot).
 *
 * Form : buzzer_pin=25&melody_enabled=1&melody_volume=50  |  JSON : {...} */
void BobWebServer::_handleAudioConfigPost(AsyncWebServerRequest* request) {
    AudioConfig cfg;
    g_audio.getConfig(cfg);   /* base : champs absents conservent la valeur courante */

    auto readPin = [&request](const char* name, int8_t& out) {
        if (request->hasParam(name, true)) {
            out = (int8_t)request->getParam(name, true)->value().toInt();
        } else if (request->hasParam(name)) {
            out = (int8_t)request->getParam(name)->value().toInt();
        }
    };
    readPin("buzzer_pin", cfg.buzzerPin);

    /* Activation de la mélodie : 0 = désactivée, 1 = activée */
    {
        int v = cfg.melodyEnabled;
        if (request->hasParam("melody_enabled", true)) {
            v = request->getParam("melody_enabled", true)->value().toInt();
        } else if (request->hasParam("melody_enabled")) {
            v = request->getParam("melody_enabled")->value().toInt();
        }
        cfg.melodyEnabled = (v != 0) ? 1 : 0;
    }

    /* Volume de la mélodie : bornes 1-100 % (valeur invalide refusée) */
    {
        int v = cfg.melodyVolume;
        if (request->hasParam("melody_volume", true)) {
            v = request->getParam("melody_volume", true)->value().toInt();
        } else if (request->hasParam("melody_volume")) {
            v = request->getParam("melody_volume")->value().toInt();
        }
        if (v < AUDIO_VOLUME_MIN || v > AUDIO_VOLUME_MAX) {
            request->send(400, "application/json",
                          "{\"error\":\"Volume invalide (1-100)\"}");
            return;
        }
        cfg.melodyVolume = (uint8_t)v;
    }

    /* Exclusions croisées : sorties ON/OFF + entrées + bandeau LED */
    uint8_t outPins[GPIO_NUM_OUTPUTS];
    g_gpio.getPins(outPins);
    StatusIOConfig ioCfg;
    g_statusIo.getConfig(ioCfg);

    uint8_t extra[GPIO_NUM_OUTPUTS + 3];
    memcpy(extra, outPins, GPIO_NUM_OUTPUTS);
    extra[GPIO_NUM_OUTPUTS]     = (uint8_t)(ioCfg.armPin   < 0 ? STATUS_IO_PIN_NONE : ioCfg.armPin);
    extra[GPIO_NUM_OUTPUTS + 1] = (uint8_t)(ioCfg.waterPin < 0 ? STATUS_IO_PIN_NONE : ioCfg.waterPin);
    extra[GPIO_NUM_OUTPUTS + 2] = (uint8_t)(ioCfg.ledPin   < 0 ? STATUS_IO_PIN_NONE : ioCfg.ledPin);

    const char* err = _userPinCheck(cfg.buzzerPin, extra, sizeof(extra));
    if (err != nullptr) {
        String body = "{\"error\":\"" + String(err) + "\"}";
        request->send(400, "application/json", body);
        return;
    }

    if (!g_audio.setConfig(cfg)) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (configuration non enregistree)\"}");
        return;
    }

    String response;
    JsonDocument resp;
    resp["status"]         = "ok";
    resp["buzzer_pin"]     = cfg.buzzerPin;
    resp["melody_enabled"] = (cfg.melodyEnabled != 0);
    resp["melody_volume"]  = cfg.melodyVolume;
    serializeJson(resp, response);

    const String pinStr = (cfg.buzzerPin < 0) ? String("non assigné")
                                              : "GPIO " + String(cfg.buzzerPin);
    Serial.println("[WEB] Acoustique / Buzzer : buzzer=" + pinStr
                   + ", mélodie de démarrage=" + (cfg.melodyEnabled ? "activée" : "désactivée")
                   + ", volume=" + String(cfg.melodyVolume) + " %"
                   + " — appliqué au prochain démarrage");
    request->send(200, "application/json", response);
}

/* =========================================================================
 * NOMS PERSONNALISÉS DES SORTIES (16 PWM + 4 GPIO) — PERSISTANCE NVS
 * =========================================================================
 *
 * Purement présentationnels : les noms sont stockés en NVS (namespace
 * « config », clé « names », un seul JSON) et servis à l'interface Web pour
 * le Banc de Test, le PWM Monitor et les états GPIO. Une chaîne vide signifie
 * « nom par défaut » (substitué côté interface). La chaîne stockée est
 * nettoyée côté firmware (caractères de contrôle/HTML retirés, longueur
 * bornée) — la couche d'affichage reste responsable de son échappement.
 * ========================================================================= */

static constexpr size_t NAMES_MAX_LEN  = 20;    /* longueur max d'un nom (octets UTF-8) */
static constexpr size_t NAMES_JSON_MAX = 1024;  /* taille max du corps JSON accepté */

/**
 * @brief Nettoie un nom d'affichage avant persistance.
 *
 * Retire les caractères de contrôle et les caractères HTML sensibles
 * (< > " ' & ` \), normalise les espaces, borne la longueur à NAMES_MAX_LEN
 * sans couper un caractère UTF-8 multi-octets (accents).
 */
static String _sanitizeName(const char* raw) {
    String s;
    if (raw == nullptr) return s;
    bool lastSpace = false;
    for (const char* p = raw; *p != '\0'; p++) {
        uint8_t c = (uint8_t)*p;
        if (c == ' ') {
            if (s.length() == 0 || lastSpace) continue;
            lastSpace = true;
            s += ' ';
        } else if (c < 0x20 || c == 0x7F) {
            continue;
        } else if (c == '<' || c == '>' || c == '"' || c == '\'' ||
                   c == '&' || c == '`' || c == '\\') {
            continue;
        } else {
            lastSpace = false;
            s += (char)c;
        }
    }
    if (s.length() > NAMES_MAX_LEN) {
        s = s.substring(0, NAMES_MAX_LEN);
        /* Ne pas laisser un caractère UTF-8 coupé en deux */
        while (s.length() > 0 && ((uint8_t)s[s.length() - 1] & 0xC0) == 0x80) {
            s.remove(s.length() - 1);
        }
        if (s.length() > 0 && ((uint8_t)s[s.length() - 1] & 0xC0) == 0xC0) {
            s.remove(s.length() - 1);
        }
    }
    s.trim();
    return s;
}

/* -------------------------------------------------------------------------
 * Handler GET /api/names — Noms personnalisés des sorties.
 *
 * Retourne deux tableaux : « ch » (16 noms PWM, index = canal) et « gpio »
 * (4 noms, index = sortie 1-4). Une entrée vide = nom par défaut (côté UI).
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleNamesGet(AsyncWebServerRequest* request) {
    String stored;
    {
        Preferences prefs;
        prefs.begin("config", true);
        stored = prefs.getString("names", "");
        prefs.end();
    }

    JsonDocument saved;
    if (stored.length() == 0 || deserializeJson(saved, stored)) {
        saved.clear();   /* absent ou corrompu → tout par défaut */
    }

    JsonDocument doc;
    JsonArray savedCh   = saved["ch"].as<JsonArray>();
    JsonArray savedGpio = saved["gpio"].as<JsonArray>();
    JsonArray ch   = doc["ch"].to<JsonArray>();
    JsonArray gpio = doc["gpio"].to<JsonArray>();
    for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
        const char* v = (i < savedCh.size()) ? savedCh[i].as<const char*>() : nullptr;
        ch.add(v ? v : "");
    }
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        const char* v = (i < savedGpio.size()) ? savedGpio[i].as<const char*>() : nullptr;
        gpio.add(v ? v : "");
    }
    doc["max_len"] = NAMES_MAX_LEN;

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* -------------------------------------------------------------------------
 * Handler POST /api/names — Enregistre les noms personnalisés (NVS).
 *
 * Corps : form-urlencoded « names={"ch":[...],"gpio":[...]} » ou JSON brut.
 * Chaque nom est nettoyé (caractères de contrôle/HTML retirés, espaces
 * normalisés, longueur bornée) puis les deux tableaux complets (16 + 4
 * entrées, vide = défaut) sont persistés en NVS sous la clé « names » —
 * relecture de vérification, aucun redémarrage nécessaire.
 * ------------------------------------------------------------------------- */
void BobWebServer::_handleNamesPost(AsyncWebServerRequest* request) {
    String body;
    if (request->hasParam("names", true)) {
        body = request->getParam("names", true)->value();
    } else if (request->hasParam("names")) {
        body = request->getParam("names")->value();
    } else if (request->hasParam("plain", true)) {
        body = request->getParam("plain", true)->value();
    } else {
        request->send(400, "application/json", "{\"error\":\"Parametre names requis\"}");
        return;
    }

    if (body.length() == 0 || body.length() > NAMES_JSON_MAX) {
        request->send(400, "application/json", "{\"error\":\"Corps names invalide\"}");
        return;
    }

    JsonDocument in;
    if (deserializeJson(in, body) || !in.is<JsonObject>()) {
        request->send(400, "application/json", "{\"error\":\"JSON invalide\"}");
        return;
    }

    JsonArray inCh   = in["ch"].as<JsonArray>();
    JsonArray inGpio = in["gpio"].as<JsonArray>();
    JsonDocument out;
    JsonArray outCh   = out["ch"].to<JsonArray>();
    JsonArray outGpio = out["gpio"].to<JsonArray>();
    uint8_t custom = 0;

    for (uint8_t i = 0; i < NUM_PWM_CHANNELS; i++) {
        String n = _sanitizeName((i < inCh.size()) ? inCh[i].as<const char*>() : nullptr);
        if (n.length() > 0) custom++;
        outCh.add(n);
    }
    for (uint8_t i = 0; i < GPIO_NUM_OUTPUTS; i++) {
        String n = _sanitizeName((i < inGpio.size()) ? inGpio[i].as<const char*>() : nullptr);
        if (n.length() > 0) custom++;
        outGpio.add(n);
    }

    String json;
    serializeJson(out, json);

    Preferences prefs;
    prefs.begin("config", false);
    size_t written = prefs.putString("names", json);
    String check = prefs.getString("names", "");
    prefs.end();
    if (written == 0 || check != json) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (noms non enregistres)\"}");
        return;
    }
    Serial.println("[WEB] Noms des sorties : " + String(custom)
                   + " nom(s) personnalise(s) — enregistres en NVS");

    /* Source unique de vérité (protocole v1.5.0) : pousser les nouveaux noms
     * vers le RPi 5 (trames 0x03/0x04) pour que le cockpit s'adapte en temps
     * réel. Non bloquant : pose un simple drapeau lu par la tâche SerialTx. */
    g_serial.sendHardwareConfigToRPi();

    out["status"]  = "ok";
    out["max_len"] = NAMES_MAX_LEN;
    String response;
    serializeJson(out, response);
    request->send(200, "application/json", response);
}

/* =========================================================================
 * CALIBRATION ALTIMÉTRIQUE QNH (v1.0.1) — OPEN-METEO (CHAMOSON, VS)
 * ========================================================================= *
 *
 * Le QNH ne modifie QUE la formule d'altitude barométrique hors de l'eau :
 * la profondeur immergée reste strictement fondée sur la tare de surface
 * locale (ΔP) et la boucle de contrôle n'est jamais impactée. La requête
 * HTTP (HTTPS, timeout court) s'exécute uniquement dans la tâche détachée
 * « QnhFetch » — jamais dans la boucle Web ni dans un handler async.
 * Fallback transparent sur la valeur standard si l'option est désactivée,
 * si Internet est absent ou si la réponse est invalide/hors bornes.
 * ------------------------------------------------------------------------- */

/* Extrait le QNH (mbar) ET l'élévation (m) d'une réponse Open-Meteo :
 * {"elevation":484.0,"current":{"time":"...","pressure_msl":1021.3,...}}
 * — l'élévation est un champ racine (altitude topographique réelle du lieu
 * demandé). Retourne false si le JSON est inutilisable ; qnh/elev restent
 * alors à 0.0f (→ hors plages de validité → ignorés par les garde-fous). */
static bool _qnhFromOpenMeteo(const String& body, float& outQnhMbar, float& outElevM) {
    outQnhMbar = 0.0f;
    outElevM   = 0.0f;
    JsonDocument doc;
    if (deserializeJson(doc, body)) return false;
    outQnhMbar = doc["current"]["pressure_msl"] | 0.0f;   /* absent → 0 → hors plage */
    outElevM   = doc["elevation"] | 0.0f;                 /* champ racine — 0 = absent */
    return true;
}

/**
 * @brief Requête Open-Meteo + application du QNH (tâche QnhFetch uniquement).
 *
 * Connexion HTTPS (certificat non vérifié : pas d'horloge RTC fiable sur
 * l'ESP32) avec timeout court QNH_HTTP_TIMEOUT_MS. Une valeur plausible
 * (bornes QNH_MIN/MAX_MBAR) est appliquée au capteur et persistée en NVS
 * (clé current_qnh) ; tout autre cas retourne 0.0f sans rien modifier.
 */
float BobWebServer::_qnhFetchAndApply() {
    WiFiClientSecure client;
    client.setInsecure();   /* pas de validation du certificat (pas d'horloge RTC fiable) */
    HTTPClient http;
    http.setConnectTimeout(QNH_HTTP_TIMEOUT_MS);
    http.setTimeout(QNH_HTTP_TIMEOUT_MS);
    if (!http.begin(client, OPEN_METEO_QNH_URL)) return 0.0f;

    int code = http.GET();
    float qnh = 0.0f;
    float elevation = 0.0f;
    if (code == 200) {
        _qnhFromOpenMeteo(http.getString(), qnh, elevation);
    } else {
        Serial.println("[WEB] QNH : requete Open-Meteo en echec (HTTP " + String(code) + ")");
    }
    http.end();

    if (qnh < QNH_MIN_MBAR || qnh > QNH_MAX_MBAR) {
        Serial.println("[WEB] QNH : reponse inutilisable — QNH courant conserve (fallback)");
        return 0.0f;
    }

    g_sensors.setQnh(qnh);
    Preferences prefs;
    prefs.begin("config", false);
    prefs.putFloat("current_qnh", qnh);
    prefs.end();
    Serial.println("[WEB] QNH Open-Meteo : " + String(qnh, 2) + " mbar — applique et enregistre");

    /* ---- Offset matériel MS5803-30BA (v1.0.1) : P_theo vs pression brute ----
     *
     * Pression atmosphérique théorique locale par la formule barométrique
     * standard (ISA) : P_theo = QNH·(1 − 0.0065·elev/288.15)^5.255.
     * La différence avec la pression absolue brute instantanée du capteur
     * mesure son erreur d'usine (~25 mbar sur ce 30 Bars : insignifiante
     * sous 300 m d'eau, ~200 m d'erreur d'altitude dans l'air).
     *
     * Garde-fous : calcul uniquement capteur HORS de l'eau (immergé, la
     * colonne d'eau fausserait tout), pression plausible (capteur présent)
     * et élévation fournie. L'offset borné (±MS5803_HW_OFFSET_MAX_MBAR)
     * est appliqué à chaud ET persisté en NVS (clé hw_offset) — il ne
     * compensera QUE la formule d'altitude, jamais la tare ni la profondeur. */
    PressureData press;
    g_sensors.getPressureData(press);
    const float pBruteMbar = static_cast<float>(press.pressure_mbar) / 10.0f;
    if (!press.immersed && pBruteMbar > 300.0f && elevation != 0.0f) {
        const float pTheoMbar = qnh * powf(1.0f - (0.0065f * elevation) / 288.15f, 5.255f);
        const float hwOffset  = pBruteMbar - pTheoMbar;
        if (fabsf(hwOffset) <= MS5803_HW_OFFSET_MAX_MBAR) {
            g_sensors.setHwOffset(hwOffset);
            prefs.begin("config", false);
            prefs.putFloat("hw_offset", hwOffset);
            prefs.end();
            Serial.println("[WEB] QNH : offset materiel MS5803 = " + String(hwOffset, 2)
                           + " mbar (P_brute " + String(pBruteMbar, 2)
                           + " - P_theo " + String(pTheoMbar, 2)
                           + ", elev " + String(elevation, 1)
                           + " m) — compense et enregistre");
        } else {
            Serial.println("[WEB] QNH : offset materiel aberrant (" + String(hwOffset, 2)
                           + " mbar) — ignore, offset courant conserve");
        }
    } else {
        Serial.println("[WEB] QNH : offset materiel non calcule (capteur "
                       + String(press.immersed ? "immerge" : "absent")
                       + " ou elevation inconnue) — offset courant conserve");
    }
    return qnh;
}

/* Tâche détachée one-shot : requête + application + auto-suppression. */
void BobWebServer::_qnhTaskEntry(void* param) {
    BobWebServer* self = static_cast<BobWebServer*>(param);
    self->_qnhLastFetchOk = (self->_qnhFetchAndApply() > 0.0f);
    self->_qnhFetchRunning = false;
    vTaskDelete(nullptr);
}

/* Handler GET /api/qnh/config — état de l'option + QNH appliqué + diagnostic. */
void BobWebServer::_handleQnhConfigGet(AsyncWebServerRequest* request) {
    Preferences prefs;
    prefs.begin("config", true);
    float qnh = prefs.getFloat("current_qnh", QNH_DEFAULT_MBAR);
    float hwOffset = prefs.getFloat("hw_offset", 0.0f);
    prefs.end();
    if (qnh < QNH_MIN_MBAR || qnh > QNH_MAX_MBAR) qnh = QNH_DEFAULT_MBAR;
    if (fabsf(hwOffset) > MS5803_HW_OFFSET_MAX_MBAR) hwOffset = 0.0f;

    JsonDocument doc;
    doc["auto_enable"]   = _qnhAutoEnable;
    doc["qnh"]           = roundf(qnh * 100.0f) / 100.0f;
    doc["hw_offset"]     = roundf(hwOffset * 100.0f) / 100.0f;
    doc["default_qnh"]   = QNH_DEFAULT_MBAR;
    doc["fetching"]      = _qnhFetchRunning;
    doc["last_fetch_ok"] = _qnhLastFetchOk;
    doc["sta_connected"] = isSTAConnected();
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* Handler POST /api/qnh/config — active/désactive l'option (NVS, à chaud).
 * Corps : form-urlencoded « enable=true|false » (paramètre de corps, d'URL
 * ou JSON). La désactivation réapplique immédiatement le QNH standard. */
void BobWebServer::_handleQnhConfigPost(AsyncWebServerRequest* request) {
    bool enable = false;
    if (request->hasParam("enable", true)) {
        enable = request->getParam("enable", true)->value().equalsIgnoreCase("true");
    } else if (request->hasParam("enable")) {
        enable = request->getParam("enable")->value().equalsIgnoreCase("true");
    } else {
        request->send(400, "application/json", "{\"error\":\"Parametre enable requis\"}");
        return;
    }

    Preferences prefs;
    prefs.begin("config", false);
    size_t written = prefs.putBool("qnh_auto_enable", enable);
    bool check = prefs.getBool("qnh_auto_enable", !enable);
    if (!enable) prefs.putFloat("current_qnh", QNH_DEFAULT_MBAR);  /* retour au standard */
    prefs.end();
    if (written == 0 || check != enable) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (option non enregistree)\"}");
        return;
    }
    _qnhAutoEnable = enable;
    if (!enable) g_sensors.setQnh(QNH_DEFAULT_MBAR);
    Serial.println(enable
        ? "[WEB] QNH : calibration automatique ACTIVEE (Open-Meteo au demarrage)"
        : "[WEB] QNH : calibration automatique DESACTIVEE (retour au QNH standard)");
    request->send(200, "application/json", "{\"status\":\"ok\"}");
}

/* Handler POST /api/qnh/refresh — force une requête Open-Meteo immédiate.
 * Non bloquant : la requête part en tâche détachée ; l'interface suit le
 * résultat en interrogeant GET /api/qnh/config (champs « fetching » puis
 * « last_fetch_ok » / « qnh »). */
void BobWebServer::_handleQnhRefreshPost(AsyncWebServerRequest* request) {
    if (!isSTAConnected()) {
        request->send(503, "application/json",
                      "{\"error\":\"Pas de connexion Internet (rejoignez un reseau externe en mode STA)\"}");
        return;
    }
    if (_qnhFetchRunning) {
        request->send(200, "application/json", "{\"status\":\"busy\"}");
        return;
    }
    _qnhFetchRunning = true;
    if (xTaskCreate(_qnhTaskEntry, "QnhFetch", STACK_SIZE_QNH_FETCH,
                    this, PRIORITY_WEB, nullptr) != pdPASS) {
        _qnhFetchRunning = false;
        request->send(500, "application/json", "{\"error\":\"Creation de la tache QNH impossible\"}");
        return;
    }
    Serial.println("[WEB] QNH : requete manuelle Open-Meteo lancee (tache detachee)");
    request->send(200, "application/json", "{\"status\":\"started\"}");
}

/* =========================================================================
 * GESTION DE SÉCURITÉ MOTEURS (v1.1.0) — CHECK, SURVEILLANCE, ISOLEMENT
 * ========================================================================= */

/* Handler GET /api/motors/config — 5 paramètres NVS + état courant du
 * gestionnaire (voyant, check, défaut, isolements). L'onglet Paramètres
 * s'y réfère à l'ouverture ; la carte Moteurs y lit les bornes de validité. */
void BobWebServer::_handleMotorsConfigGet(AsyncWebServerRequest* request) {
    MotorConfig cfg;
    g_motors.getConfig(cfg);

    JsonDocument doc;
    doc["check_enable"]     = cfg.checkEnable;
    doc["detect_enable"]    = cfg.detectEnable;
    doc["emergency_enable"] = cfg.emergencyEnable;
    doc["cur_thresh"]       = roundf(cfg.currentThresholdA * 10.0f) / 10.0f;
    doc["vbat_low"]         = roundf(cfg.vbatLowV * 10.0f) / 10.0f;
    doc["cur_thresh_min"]   = MOTOR_CURRENT_THRESH_MIN_A;
    doc["cur_thresh_max"]   = MOTOR_CURRENT_THRESH_MAX_A;
    doc["vbat_low_min"]     = MOTOR_VBAT_LOW_MIN_V;
    doc["vbat_low_max"]     = MOTOR_VBAT_LOW_MAX_V;
    doc["health"]           = (uint8_t)g_motors.getStatus();
    doc["status_text"]      = g_motors.getLastStatusText();
    doc["check_done"]       = g_motors.isCheckDone();
    doc["check_passed"]     = g_motors.isCheckPassed();
    doc["check_pending"]    = g_motors.isCheckPending();
    doc["fault"]            = g_motors.isFaultActive();
    doc["emergency"]        = g_motors.isEmergencySurfaceActive();
    doc["isolated_mask"]    = g_motors.getIsolatedMask();
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* Handler POST /api/motors/config — enregistre les 5 paramètres en NVS
 * (écriture vérifiée par relecture dans MotorManager::setConfig).
 * Corps form-urlencoded : check_enable, detect_enable, emergency_enable,
 * cur_thresh (A), vbat_low (V) — champs de corps, d'URL ou JSON ; les champs
 * absents conservent la valeur courante. */
void BobWebServer::_handleMotorsConfigPost(AsyncWebServerRequest* request) {
    MotorConfig cfg;
    g_motors.getConfig(cfg);   /* valeurs courantes par défaut (champs absents) */

    auto readBool = [&request](const char* name, bool& out) {
        if (request->hasParam(name, true)) {
            out = request->getParam(name, true)->value().equalsIgnoreCase("true");
        } else if (request->hasParam(name)) {
            out = request->getParam(name)->value().equalsIgnoreCase("true");
        }
    };
    auto readFloat = [&request](const char* name, float& out) {
        if (request->hasParam(name, true)) {
            out = request->getParam(name, true)->value().toFloat();
        } else if (request->hasParam(name)) {
            out = request->getParam(name)->value().toFloat();
        }
    };
    readBool("check_enable",     cfg.checkEnable);
    readBool("detect_enable",    cfg.detectEnable);
    readBool("emergency_enable", cfg.emergencyEnable);
    readFloat("cur_thresh",      cfg.currentThresholdA);
    readFloat("vbat_low",        cfg.vbatLowV);

    if (!g_motors.setConfig(cfg)) {
        request->send(500, "application/json",
                      "{\"error\":\"Ecriture NVS impossible (configuration non enregistree)\"}");
        return;
    }
    request->send(200, "application/json", "{\"status\":\"ok\"}");
}

/* Handler POST /api/motors/check — relance manuelle de la séquence de check
 * propulseurs. Non bloquant : la tâche de gestion moteurs exécute les
 * impulsions séquentielles ; l'interface suit le résultat via GET
 * /api/motors/config et l'objet « motors » de la télémétrie WebSocket. */
void BobWebServer::_handleMotorsCheckPost(AsyncWebServerRequest* request) {
    if (!g_motors.requestManualCheck()) {
        request->send(409, "application/json",
                      "{\"error\":\"Check impossible : sequence en cours ou defaut actif (isolation)\"}");
        return;
    }
    request->send(200, "application/json", "{\"status\":\"started\"}");
}

/* =========================================================================
 * SIMULATEUR DE TEST VIRTUEL (établi) — INJECTION CAPTEURS
 * =========================================================================
 *
 * Routes /api/simulator (état RAM uniquement, jamais NVS) : le panneau
 * « Simulateur de Test Virtuel » (Paramètres, grands écrans) pilote le bypass
 * des lectures INA3221/INA226 (voir SensorDriver et simulator.h). Dès que la
 * simulation repasse inactive, la machine à états de sécurité est réalimentée
 * par les capteurs réels au cycle capteurs suivant (10 ms) et un réarmement
 * « établi » est demandé à la tâche Motors : isolements, défaut et check
 * échoué induits par la simulation sont levés au cycle de surveillance
 * suivant (100 ms).
 */

/* Handler GET /api/simulator — valeurs courantes + bornes de bornage. */
void BobWebServer::_handleSimulatorGet(AsyncWebServerRequest* request) {
    const SimValues sim = g_sim.get();
    JsonDocument doc;
    doc["active"]        = sim.active;
    doc["vbat1"]         = roundf(sim.vbat1V * 100.0f) / 100.0f;
    doc["vbat2"]         = roundf(sim.vbat2V * 100.0f) / 100.0f;
    doc["curh"]          = roundf(sim.curH_A * 100.0f) / 100.0f;
    doc["curv"]          = roundf(sim.curV_A * 100.0f) / 100.0f;
    doc["stall_mask"]    = sim.stallMask;
    doc["vbat1_default"] = SIM_VBAT1_DEFAULT_V;
    doc["vbat1_max"]     = SIM_VBAT1_MAX_V;
    doc["vbat2_default"] = SIM_VBAT2_DEFAULT_V;
    doc["vbat2_max"]     = SIM_VBAT2_MAX_V;
    doc["cur_max"]       = SIM_CURRENT_MAX_A;
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
}

/* Handler POST /api/simulator — remplace les valeurs injectées.
 * Corps form-urlencoded : active, vbat1, vbat2, curh, curv, stall_mask —
 * champs de corps ou d'URL ; les champs absents conservent la valeur
 * courante ; toutes les valeurs sont bornées défensivement (config.h SIM_*). */
void BobWebServer::_handleSimulatorPost(AsyncWebServerRequest* request) {
    const SimValues prev = g_sim.get();   /* base : valeurs courantes (champs absents) */
    SimValues sim = prev;

    auto readBool = [&request](const char* name, bool& out) {
        if (request->hasParam(name, true)) {
            out = request->getParam(name, true)->value().equalsIgnoreCase("true");
        } else if (request->hasParam(name)) {
            out = request->getParam(name)->value().equalsIgnoreCase("true");
        }
    };
    auto readFloat = [&request](const char* name, float& out) {
        if (request->hasParam(name, true)) {
            out = request->getParam(name, true)->value().toFloat();
        } else if (request->hasParam(name)) {
            out = request->getParam(name)->value().toFloat();
        }
    };
    auto readStallMask = [&request](uint8_t& out) {
        const AsyncWebParameter* p = request->hasParam("stall_mask", true)
            ? request->getParam("stall_mask", true)
            : (request->hasParam("stall_mask") ? request->getParam("stall_mask") : nullptr);
        if (p == nullptr) return;   /* champ absent : masque courant conservé */
        long v = p->value().toInt();
        if (v < 0) v = 0;
        if (v > 0xFF) v = 0xFF;     /* 8 moteurs simulables : bits 0-7 */
        out = (uint8_t)v;
    };

    readBool("active", sim.active);
    readFloat("vbat1", sim.vbat1V);
    readFloat("vbat2", sim.vbat2V);
    readFloat("curh", sim.curH_A);
    readFloat("curv", sim.curV_A);
    readStallMask(sim.stallMask);

    /* Bornage défensif (config.h) — l'API ne peut pas injecter hors plage. */
    if (sim.vbat1V < 0.0f) sim.vbat1V = 0.0f;
    if (sim.vbat1V > SIM_VBAT1_MAX_V) sim.vbat1V = SIM_VBAT1_MAX_V;
    if (sim.vbat2V < 0.0f) sim.vbat2V = 0.0f;
    if (sim.vbat2V > SIM_VBAT2_MAX_V) sim.vbat2V = SIM_VBAT2_MAX_V;
    if (sim.curH_A < 0.0f) sim.curH_A = 0.0f;
    if (sim.curH_A > SIM_CURRENT_MAX_A) sim.curH_A = SIM_CURRENT_MAX_A;
    if (sim.curV_A < 0.0f) sim.curV_A = 0.0f;
    if (sim.curV_A > SIM_CURRENT_MAX_A) sim.curV_A = SIM_CURRENT_MAX_A;

    g_sim.set(sim);

    /* Arrêt du simulateur (actif → inactif) : les défauts observés sous
     * valeurs simulées (surintensité, moteurs isolés, check échoué, remontée
     * d'urgence) ne sont plus justifiés par aucune mesure réelle — réarmement
     * établi demandé à la tâche Motors (appliqué au cycle suivant, 100 ms ;
     * sans effet si l'état était déjà propre). */
    if (prev.active && !sim.active) {
        g_motors.requestSimulatorReset();
    }

    /* Trace série sur transition uniquement (pas de spam à chaque slider). */
    if (sim.active != prev.active) {
        Serial.println(sim.active
            ? "[SIM] Simulation capteurs ACTIVE — lectures INA3221/INA226 bypassées, valeurs injectées"
            : "[SIM] Simulation capteurs INACTIVE — retour aux capteurs réels, réarmement moteurs demandé");
    }
    if (sim.active && sim.stallMask != prev.stallMask) {
        Serial.println("[SIM] Blocages moteurs simulés : masque 0x" + String(sim.stallMask, HEX));
    }

    request->send(200, "application/json", "{\"status\":\"ok\"}");
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
 * @brief Handler GET /api/system/status — Uptime + version du contrôleur.
 *
 * Utilisé par l'interface OTA comme preuve de redémarrage : après un flash,
 * l'uptime repart de zéro (un simple code 200 peut venir de l'ancien firmware
 * encore vivant pendant le délai de redémarrage de 3 s).
 *
 * Le champ "version" expose la macro GIT_VERSION (git describe --tags
 * --always --dirty), définie dans l'en-tête généré include/git_version.h
 * (scripts/git_version.py, extra_scripts pre:), au pied de page de
 * l'interface. Fallback "dev" si compilé hors dépôt Git.
 */
#include "git_version.h"
void BobWebServer::_handleSystemStatus(AsyncWebServerRequest* request) {
    String response;
    JsonDocument doc;
    doc["status"]    = "ok";
    doc["uptime_ms"] = (uint32_t)millis();
    doc["version"]   = GIT_VERSION;
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

        /* Neutraliser la propulsion et les sorties ON/OFF AVANT de toucher la flash.
         * La remontée d'urgence est d'abord coupée : une mise à jour volontaire
         * est une décision humaine qui prime sur la survie automatique. */
        g_motors.abortEmergencySurface();
        g_pwm.setPhysicalOutputsEnabled(false);
        g_pwm.setAllNeutral();
        g_gpio.allOff();

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
    if (WiFi.status() == WL_CONNECTED) {
        return WiFi.localIP().toString();
    }
    return WiFi.softAPIP().toString();
}

/**
 * @brief Indique si l'ESP est connecté en mode STA à un réseau externe.
 *
 * Vérité matérielle (correctif v1.0.1) : le flag membre _staConnected n'était
 * mis à jour QUE par le formulaire de connexion manuelle (/api/wifi/connect).
 * Après l'auto-reconnexion du boot (identifiants NVS, WiFi.begin() au début de
 * begin()), il restait donc faux alors que le STA était bien connecté — d'où
 * un 503 erroné sur /api/qnh/refresh et un fetch QNH de démarrage jamais lancé.
 * WiFi.status() est la même source de vérité que le broadcast temps réel
 * (wifi_info.sta_connected) : un seul comportement partout.
 */
bool BobWebServer::isSTAConnected() const {
    return (WiFi.status() == WL_CONNECTED);
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
    const TickType_t taskStart = lastWake;   /* repère du démarrage (requête QNH boot, v1.0.1) */

    for (;;) {
        /* Traitement du DNS captif (non bloquant) */
        _dns.processNextRequest();

        /* Nettoyage des clients WebSocket déconnectés */
        _ws.cleanupClients();

        /* Broadcast périodique de la télémétrie via WebSocket */
        TickType_t now = xTaskGetTickCount();

        /* QNH (v1.0.1) : requête Open-Meteo unique par boot, différée de
         * QNH_BOOT_FETCH_DELAY_MS pour laisser la station Wi-Fi joindre
         * Internet — en tâche détachée, jamais dans cette boucle (Core 0
         * libre pour le DNS et le broadcast pendant le fetch HTTPS).
         * Lancée UNIQUEMENT si l'option est activée ET que le STA est
         * connecté à Internet (re-testé à chaque cycle : si le réseau
         * externe arrive tard — ou jamais —, la requête part dès qu'il
         * est là, ou ne part pas du tout en mode AP seul). */
        if (_qnhAutoEnable && !_qnhBootApplied &&
            (now - taskStart) >= pdMS_TO_TICKS(QNH_BOOT_FETCH_DELAY_MS) &&
            isSTAConnected()) {
            _qnhBootApplied = true;
            _qnhFetchRunning = true;
            if (xTaskCreate(_qnhTaskEntry, "QnhFetch", STACK_SIZE_QNH_FETCH,
                            this, PRIORITY_WEB, nullptr) != pdPASS) {
                _qnhFetchRunning = false;
                Serial.println("[WEB] QNH : creation de la tache Open-Meteo impossible");
            }
        }

        if ((now - lastBroadcast) >= pdMS_TO_TICKS(TASK_WS_BROADCAST_MS)) {
            lastBroadcast = now;

            /* Construire le JSON de télémétrie seulement si des clients sont connectés */
            if (_ws.count() > 0) {
                JsonDocument doc;

                doc["wdg"] = g_serial.isWatchdogTriggered();
                doc["physical_outputs_enabled"] = g_pwm.isPhysicalOutputsEnabled();
                doc["pwm_type_mask"] = g_pwm.getChannelTypeMask();  /* typage canaux 10-15 (bit=1 bidir) */
                doc["gpio_state"]    = g_gpio.getState();           /* état réel des 4 sorties ON/OFF */
                doc["sim_active"]    = g_sim.get().active;          /* simulateur établi (badge topbar) */

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

                /* Puissance (5 canaux : 3 canaux INA3221 + 2 INA226 résiduels) */
                PowerData pwr;
                g_sensors.getPowerData(pwr);
                JsonArray pwrArr = doc["power"].to<JsonArray>();
                for (uint8_t i = 0; i < 5; i++) {
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
                sens["ina3221"] = (uint8_t)st.ina3221;   /* triple wattmètre V_BAT + 2× ACS770 (v1.1.0) */
                sens["ina1"]    = (uint8_t)st.ina226[0]; /* INA226 n°2 (électronique de commande) */
                sens["ina2"]    = (uint8_t)st.ina226[1]; /* INA226 n°3 (projecteur LED) */
                sens["pca"]     = (uint8_t)st.pca9685;

                /* Gestion moteurs (v1.1.0) : voyant « Statut Moteurs », état du
                 * check, défaut/isolation en cours et remontée d'urgence. */
                {
                    JsonObject motors = doc["motors"].to<JsonObject>();
                    motors["health"]        = (uint8_t)g_motors.getStatus();
                    motors["status"]        = g_motors.getLastStatusText();
                    motors["check_done"]    = g_motors.isCheckDone();
                    motors["check_passed"]  = g_motors.isCheckPassed();
                    motors["check_pending"] = g_motors.isCheckPending();
                    motors["fault"]         = g_motors.isFaultActive();
                    motors["emergency"]     = g_motors.isEmergencySurfaceActive();
                    motors["isolated_mask"] = g_motors.getIsolatedMask();
                }

                /* Statut liaison série RPi5 (interface active + heartbeat) */
                JsonObject rpi = doc["rpi_link"].to<JsonObject>();
                rpi["interface"] = g_serial.getInterfaceLabel();
                rpi["connected"] = !g_serial.isWatchdogTriggered();

                /* Contrôleur de vol (mode + maître) */
                doc["flight_mode"] = g_flightCtrl.getActiveMode();
                doc["rpi_master"]  = g_flightCtrl.isRPi5Master();

                /* Signalisation LED + entrées numériques (v1.6.0) : états
                 * débouncés des deux entrées, index de l'état affiché par le
                 * bandeau (255 = bandeau non configuré) et override actif. */
                {
                    JsonObject sio = doc["status_io"].to<JsonObject>();
                    sio["arm"]       = g_statusIo.isArmClosed();
                    sio["water"]     = g_statusIo.isWaterAlarm();
                    sio["led_state"] = g_statusIo.getActiveLedState();
                    sio["override"]  = g_flightCtrl.isSurfaceOverrideActive();
                }

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
