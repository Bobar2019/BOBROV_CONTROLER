/**
 * @file main.cpp
 * @brief Point d'entrée du firmware BOB-CONTROL pour ESP32-S3.
 *
 * Initialise l'ensemble des périphériques (LittleFS, I2C, Serial, PCA9685),
 * instancie les mutex FreeRTOS et crée les tâches réparties sur les deux cœurs :
 *
 * - Core 0 (Wi-Fi)  : vTaskWeb (serveur HTTP, WebSocket, DNS captif)
 * - Core 1 (RT)     : vTaskControl (boucle de contrôle PWM + failsafe)
 *                     vTaskSensors (acquisition capteurs / simulation)
 *
 * La tâche SerialRx (réception trame) est lancée par SerialComm::begin()
 * et la tâche SerialTx (émission télémétrie) également.
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Août 2026
 */

#include <Arduino.h>
#include <LittleFS.h>
#include <Preferences.h>

#include "config.h"
#include "protocol.h"
#include "autopilot.h"
#include "sensor_driver.h"
#include "pwm_controller.h"
#include "serial_comm.h"
#include "web_server.h"
#include "flight_controller.h"

/* =========================================================================
 * VARIABLES GLOBALES PARTAGÉES
 * ========================================================================= */

/**
 * @brief Mutex protégeant l'accès à la configuration autopilote.
 *
 * La structure AutopilotConfig est lue par la tâche Control
 * et modifiée par le serveur Web via l'onglet Paramètres.
 */
static SemaphoreHandle_t g_mutexConfig = nullptr;

/**
 * @brief Configuration autopilote active (PID + watchdog).
 *
 * Initialisée avec les valeurs par défaut au démarrage, puis
 * restaurée depuis la NVS si des paramètres sauvegardés existent.
 */
static AutopilotConfig g_apConfig;

/* =========================================================================
 * PERSISTANCE NVS (PARAMÈTRES AUTOPILOTE)
 * ========================================================================= */

/**
 * @brief Sauvegarde la configuration autopilote en NVS.
 *
 * Utilise la bibliothèque Preferences (NVS) pour stocker les gains PID
 * et le timeout du watchdog de manière persistante.
 */
static void saveAutopilotConfig() {
    Preferences prefs;
    prefs.begin("autopilot", false);

    /* PID Roll */
    prefs.putFloat("r_kp", g_apConfig.roll.kp);
    prefs.putFloat("r_ki", g_apConfig.roll.ki);
    prefs.putFloat("r_kd", g_apConfig.roll.kd);

    /* PID Pitch */
    prefs.putFloat("p_kp", g_apConfig.pitch.kp);
    prefs.putFloat("p_ki", g_apConfig.pitch.ki);
    prefs.putFloat("p_kd", g_apConfig.pitch.kd);

    /* PID Yaw */
    prefs.putFloat("y_kp", g_apConfig.yaw.kp);
    prefs.putFloat("y_ki", g_apConfig.yaw.ki);
    prefs.putFloat("y_kd", g_apConfig.yaw.kd);

    /* PID Altitude */
    prefs.putFloat("d_kp", g_apConfig.alt.kp);
    prefs.putFloat("d_ki", g_apConfig.alt.ki);
    prefs.putFloat("d_kd", g_apConfig.alt.kd);

    /* Watchdog timeout */
    prefs.putUInt("wdg_ms", g_apConfig.watchdog_ms);

    prefs.end();
    Serial.println("[MAIN] Configuration autopilote sauvegardée en NVS");
}

/**
 * @brief Restaure la configuration autopilote depuis la NVS.
 *
 * Si aucune donnée n'est trouvée, les valeurs par défaut sont conservées.
 */
static void loadAutopilotConfig() {
    Preferences prefs;
    prefs.begin("autopilot", true);  /* Lecture seule */

    /* Vérifier si des données existent (clef "r_kp" comme marqueur) */
    if (!prefs.isKey("r_kp")) {
        prefs.end();
        Serial.println("[MAIN] Aucune config NVS trouvée → valeurs par défaut");
        return;
    }

    /* PID Roll */
    g_apConfig.roll.kp = prefs.getFloat("r_kp", g_apConfig.roll.kp);
    g_apConfig.roll.ki = prefs.getFloat("r_ki", g_apConfig.roll.ki);
    g_apConfig.roll.kd = prefs.getFloat("r_kd", g_apConfig.roll.kd);

    /* PID Pitch */
    g_apConfig.pitch.kp = prefs.getFloat("p_kp", g_apConfig.pitch.kp);
    g_apConfig.pitch.ki = prefs.getFloat("p_ki", g_apConfig.pitch.ki);
    g_apConfig.pitch.kd = prefs.getFloat("p_kd", g_apConfig.pitch.kd);

    /* PID Yaw */
    g_apConfig.yaw.kp = prefs.getFloat("y_kp", g_apConfig.yaw.kp);
    g_apConfig.yaw.ki = prefs.getFloat("y_ki", g_apConfig.yaw.ki);
    g_apConfig.yaw.kd = prefs.getFloat("y_kd", g_apConfig.yaw.kd);

    /* PID Altitude */
    g_apConfig.alt.kp = prefs.getFloat("d_kp", g_apConfig.alt.kp);
    g_apConfig.alt.ki = prefs.getFloat("d_ki", g_apConfig.alt.ki);
    g_apConfig.alt.kd = prefs.getFloat("d_kd", g_apConfig.alt.kd);

    /* Watchdog */
    g_apConfig.watchdog_ms = prefs.getUInt("wdg_ms", SERIAL_TIMEOUT_DEFAULT_MS);

    prefs.end();
    Serial.println("[MAIN] Configuration autopilote restaurée depuis NVS");
}

/* =========================================================================
 * TÂCHE FREERTOS : vTaskControl
 * ========================================================================= */

/**
 * @brief Tâche de contrôle principale (Core 1 - Temps Réel).
 *
 * Boucle à 100 Hz qui :
 * 1. Vérifie le watchdog série (failsafe neutre si timeout)
 * 2. Lit les trames descendantes reçues du RPi 5
 * 3. Applique les consignes PWM aux actionneurs
 * 4. Construit la trame montante de télémétrie
 * 5. Met à jour le buffer d'émission série
 *
 * @param param Non utilisé.
 */
static void vTaskControl(void* param) {
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        /* ---- 1. Vérification du watchdog série ---- */
        if (g_serial.isWatchdogTriggered()) {
            /* Le watchdog a déjà forcé le neutre dans serial_comm.cpp */
            /* On continue de boucler pour maintenir la télémétrie */
        }

        /* ---- 2. Traitement d'une nouvelle trame descendante ---- */
        if (g_serial.hasNewFrame()) {
            DownlinkData dl;
            g_serial.getDownlinkData(dl);

            /* Vérification de l'état d'armement */
            if (dl.arm_state == ARM_ESTOP) {
                /* Arrêt d'urgence : tous les canaux au neutre */
                g_pwm.setAllNeutral();
                g_flightCtrl.resetPID();
            } else if (dl.arm_state == ARM_ARMED) {
                /* Armé : appliquer les consignes PWM brutes du RPi 5 */
                g_pwm.setAllPWM(dl.pwm);
                /* Notifier le contrôleur de vol du mode RPi 5 */
                g_flightCtrl.updateFromRPi5(dl.mode);
            } else {
                /* Désarmé : neutre sur tous les propulseurs (0-7) */
                for (uint8_t i = 0; i < 8; i++) {
                    g_pwm.setPWMuS(i, PWM_NEUTRAL_US);
                }
                /* Servos et auxiliaires acceptent les commandes même désarmés */
                for (uint8_t i = 8; i < NUM_PWM_CHANNELS; i++) {
                    g_pwm.setPWMuS(i, dl.pwm[i]);
                }
                g_flightCtrl.resetPID();
            }
        }

        /* ---- 2b. Corrections PID du contrôleur de vol ---- */
        {
            uint8_t mode = g_flightCtrl.getActiveMode();
            if (mode != MODE_PASSIF) {
                /* Récupérer les consignes PWM actuelles */
                uint16_t pwmOut[NUM_PWM_CHANNELS];
                g_pwm.getCurrentValues(pwmOut);

                /* Si aucun RPi 5 maître, forcer les propulseurs à 1500µs
                 * (validation sur établi sans trame descendante) */
                if (!g_flightCtrl.isRPi5Master()) {
                    for (uint8_t i = 0; i < 8; i++) {
                        pwmOut[i] = PWM_NEUTRAL_US;
                    }
                }

                /* Appliquer les corrections PID + mixage */
                IMUData imuCtl;
                g_sensors.getIMUData(imuCtl);
                PressureData pressCtl;
                g_sensors.getPressureData(pressCtl);
                float dt = (float)TASK_CONTROL_PERIOD_MS / 1000.0f;
                g_flightCtrl.update(imuCtl, pressCtl, pwmOut, 8, dt);
                g_pwm.setAllPWM(pwmOut);
            }
        }

        /* ---- 3. Construction de la trame montante de télémétrie ---- */
        UplinkFrame_t ul;
        ul.header1 = UL_HEADER_1;    /* 0x55 */
        ul.header2 = UL_HEADER_2;    /* 0xAA */
        ul.type    = UL_FRAME_TYPE;  /* 0x02 */

        /* Bits de statut */
        ul.status = 0;
        if (g_serial.getLastFrameTime() > 0)     ul.status |= STATUS_BIT_ARMED;
        if (g_serial.isWatchdogTriggered())      ul.status |= STATUS_BIT_WDG;
        if (!g_pwm.isPhysicalOutputsEnabled())   ul.status |= STATUS_BIT_DRYRUN;

        /* Données IMU (quaternions et gyroscope) */
        IMUData imu;
        g_sensors.getIMUData(imu);
        memcpy(ul.quat, imu.quat, sizeof(ul.quat));
        memcpy(ul.gyro, imu.gyro, sizeof(ul.gyro));

        /* Pression et température */
        PressureData press;
        g_sensors.getPressureData(press);
        ul.pressure    = press.pressure_mbar;
        ul.temperature = press.temperature;

        /* Télémétrie puissance (3 wattmètres) */
        PowerData pwr;
        g_sensors.getPowerData(pwr);
        for (uint8_t i = 0; i < 3; i++) {
            ul.power[i * 2]     = pwr.voltage[i];  /* Tension en mV */
            ul.power[i * 2 + 1] = pwr.current[i];  /* Courant en mA */
        }

        /* PWM effectifs (valeurs réellement envoyées) */
        g_pwm.getCurrentValues(ul.pwm_actual);

        /* Calcul et insertion du CRC16 */
        uint8_t* frameBytes = reinterpret_cast<uint8_t*>(&ul);
        crc16_fill(frameBytes, UL_FRAME_SIZE);

        /* Mise à jour du buffer d'émission série */
        g_serial.updateTelemetry(frameBytes, UL_FRAME_SIZE);

        /* Attente périodique précise (100 Hz) */
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));
    }
}

/* =========================================================================
 * SETUP (INITIALISATION GLOBALE)
 * ========================================================================= */

/**
 * @brief Initialisation globale appelée une seule fois au démarrage.
 *
 * Séquence d'initialisation :
 * 1. Liaison série USB-CDC
 * 2. Filesystem LittleFS (interface Web statique)
 * 3. Configuration autopilote (NVS)
 * 4. Bus I2C et capteurs
 * 5. Contrôleur PWM PCA9685
 * 6. Communication série (machine à états)
 * 7. Serveur Web embarqué
 * 8. Tâche de contrôle FreeRTOS
 */
void setup() {
    /* ---- 1. Liaison série USB-CDC pour le debug ---- */
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n==============================================");
    Serial.println("  BOB-CONTROL v1.0.0 - ROV Firmware");
    Serial.println("  ESP32-S3 PlatformIO / FreeRTOS");
    Serial.println("==============================================\n");

    /* ---- 2. Initialisation du filesystem LittleFS ---- */
    if (!LittleFS.begin(true)) {  /* true = formater si nécessaire */
        Serial.println("[MAIN] ERREUR : échec de montage LittleFS !");
    } else {
        Serial.println("[MAIN] LittleFS monté avec succès");
        /* Afficher l'espace disponible */
        Serial.println("[MAIN] LittleFS - Total : " + String(LittleFS.totalBytes())
                       + " octets, Utilisé : " + String(LittleFS.usedBytes()) + " octets");
    }

    /* ---- 3. Configuration autopilote (valeurs par défaut + NVS) ---- */
    g_apConfig = getDefaultAutopilotConfig();
    loadAutopilotConfig();

    /* Création du mutex de protection de la configuration */
    g_mutexConfig = xSemaphoreCreateMutex();

    /* ---- 4. Initialisation des capteurs I2C ---- */
    /* Les broches des DEUX bus (clés NVS i2c_sda/i2c_scl et i2c2_sda/i2c2_scl,
     * défauts GPIO 10/11 et 6/7) sont chargées par SensorDriver::begin() —
     * source unique, qui journalise et configure Wire (bus n°1) et Wire1
     * (bus n°2) d'un seul endroit. */
    g_sensors.begin();

    /* ---- 5. Initialisation du contrôleur PWM PCA9685 ---- */
    g_pwm.begin();

    /* ---- 6. Initialisation de la communication série (USB-CDC ou UART GPIO) ---- */
    /* Chargement du choix d'interface depuis la NVS */
    SerialInterfaceType commIface = COMM_INTERFACE_USB_CDC;
    {
        Preferences cprefs;
        cprefs.begin("config", true);
        if (cprefs.isKey("comm_iface")) {
            commIface = (SerialInterfaceType)cprefs.getUChar("comm_iface", 0);
        }
        cprefs.end();
        Serial.println("[MAIN] Interface série RPi5 : " + String((uint8_t)commIface) +
                       (commIface == COMM_INTERFACE_UART_GPIO ? " (UART GPIO 1/2)" : " (USB-CDC)"));
    }
    g_serial.begin(commIface);

    /* Appliquer le timeout watchdog depuis la configuration */
    g_serial.setWatchdogTimeout(g_apConfig.watchdog_ms);

    /* ---- 7. Initialisation du serveur Web et portail captif ---- */
    g_webServer.begin();

    /* ---- 7b. Initialisation du contrôleur de vol ---- */
    g_flightCtrl.begin(&g_apConfig, g_mutexConfig);

    /* ---- 8. Lancement de la tâche de contrôle sur Core 1 ---- */
    xTaskCreatePinnedToCore(
        vTaskControl,           /* Fonction de la tâche */
        "Control",              /* Nom */
        STACK_SIZE_CONTROL,     /* Taille de stack (4096 octets) */
        nullptr,                /* Paramètre */
        PRIORITY_CONTROL,       /* Priorité maximale (5) */
        nullptr,                /* Handle */
        CORE_RT                 /* Core 1 (temps réel) */
    );

    Serial.println("[MAIN] Initialisation complète - système opérationnel\n");
}

/* =========================================================================
 * LOOP (INACTIF - TOUT EST GÉRÉ PAR FREERTOS)
 * ========================================================================= */

/**
 * @brief Boucle Arduino principale (inutilisée).
 *
 * Toute la logique est déléguée aux tâches FreeRTOS.
 * La boucle loop() reste vide pour éviter la contention CPU.
 */
void loop() {
    vTaskDelay(pdMS_TO_TICKS(10000));  /* Suspension longue durée */
}
