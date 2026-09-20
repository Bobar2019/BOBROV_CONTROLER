/**
 * @file audio_manager.cpp
 * @brief Implémentation de la signature sonore de démarrage (v1.0.0).
 *
 * Chaîne matérielle : timer LEDC → PWM à la fréquence de la note, rapport
 * cyclique modulé par le volume réglé (1-100 %) → buzzer passif. La tâche
 * « Melody » (éphémère, Core Wi-Fi, priorité
 * PRIORITY_AUDIO) est créée par playStartupMelodyIfEnabled() à la fin de
 * setup() et se détruit après la dernière note ; elle n'attache la broche
 * qu'au moment de jouer et la relâche ensuite (GPIO laissé au repos bas),
 * le buzzer ne consommant donc que pendant la mélodie (~2,9 s).
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Septembre 2026
 */

#include "audio_manager.h"

#include <Preferences.h>

/* =========================================================================
 * INSTANCE GLOBALE
 * ========================================================================= */

/** @brief Instance globale du module acoustique (buzzer) */
AudioManager g_audio;

/* =========================================================================
 * CONSTRUCTEUR
 * ========================================================================= */

AudioManager::AudioManager()
    : _busy(false)
{
    /* Défauts : buzzer non assigné, mélodie de démarrage désactivée,
     * volume médian */
    _cfg.buzzerPin     = -1;
    _cfg.melodyEnabled = 0;
    _cfg.melodyVolume  = AUDIO_VOLUME_DEFAULT;
}

/* =========================================================================
 * INITIALISATION
 * ========================================================================= */

void AudioManager::begin() {
    /* ---- Chargement de la configuration NVS (namespace « audio ») ---- */
    {
        Preferences prefs;
        prefs.begin("audio", true);
        const uint8_t pin = prefs.getUChar("buzz_pin", STATUS_IO_PIN_NONE);
        _cfg.melodyEnabled = prefs.getUChar("melody_en", 0) ? 1 : 0;
        _cfg.melodyVolume  = prefs.getUChar("buzz_vol", AUDIO_VOLUME_DEFAULT);
        prefs.end();

        /* Bornage défensif : une NVS corrompue ne doit jamais produire une
         * broche inexistante (même garde-fou que les autres modules d'E/S). */
        _cfg.buzzerPin = (pin != STATUS_IO_PIN_NONE && pin <= 48
                          && !isMemoryBusPin(pin)) ? (int8_t)pin : -1;
        if (pin != STATUS_IO_PIN_NONE && pin <= 48 && isMemoryBusPin(pin)) {
            Serial.println("[AUDIO] GPIO 35-37 réservés au bus flash/PSRAM (module octal) — affectation ignorée");
        }

        /* Bornage défensif du volume (1-100 %) */
        if (_cfg.melodyVolume < AUDIO_VOLUME_MIN) _cfg.melodyVolume = AUDIO_VOLUME_MIN;
        if (_cfg.melodyVolume > AUDIO_VOLUME_MAX) _cfg.melodyVolume = AUDIO_VOLUME_MAX;
    }

    if (_cfg.buzzerPin >= 0) {
        Serial.println("[AUDIO] Buzzer passif : GPIO " + String(_cfg.buzzerPin)
                       + " — mélodie de démarrage "
                       + (_cfg.melodyEnabled ? "activée" : "désactivée")
                       + " (volume " + String(_cfg.melodyVolume)
                       + " %, timer LEDC matériel)");
    } else {
        Serial.println("[AUDIO] Buzzer : non assigné (aucune mélodie)");
    }
}

/* =========================================================================
 * ACCÈS (SERVEUR WEB)
 * ========================================================================= */

void AudioManager::getConfig(AudioConfig& out) const {
    out = _cfg;
}

bool AudioManager::setConfig(const AudioConfig& cfg) {
    /* Bornage AVANT toute écriture (garde-fou identique au chargement NVS). */
    AudioConfig c = cfg;
    if (c.buzzerPin < -1 || c.buzzerPin > 48
        || (c.buzzerPin >= 0 && isMemoryBusPin((uint8_t)c.buzzerPin))) {
        c.buzzerPin = -1;
    }
    c.melodyEnabled = (c.melodyEnabled != 0) ? 1 : 0;
    if (c.melodyVolume < AUDIO_VOLUME_MIN) c.melodyVolume = AUDIO_VOLUME_MIN;
    if (c.melodyVolume > AUDIO_VOLUME_MAX) c.melodyVolume = AUDIO_VOLUME_MAX;

    /* NVS : écriture vérifiée par relecture (convention du projet). */
    Preferences prefs;
    if (!prefs.begin("audio", false)) return false;
    prefs.putUChar("buzz_pin", (uint8_t)(c.buzzerPin < 0 ? STATUS_IO_PIN_NONE : c.buzzerPin));
    prefs.putUChar("melody_en", c.melodyEnabled);
    prefs.putUChar("buzz_vol",  c.melodyVolume);

    bool ok = true;
    ok = ok && (prefs.getUChar("buzz_pin", 0xFE)
                == (uint8_t)(c.buzzerPin < 0 ? STATUS_IO_PIN_NONE : c.buzzerPin));
    ok = ok && (prefs.getUChar("melody_en", 0xFE) == c.melodyEnabled);
    ok = ok && (prefs.getUChar("buzz_vol",  0xFE) == c.melodyVolume);
    prefs.end();
    if (!ok) {
        Serial.println("[AUDIO] Écriture NVS impossible (configuration non enregistrée)");
        return false;
    }

    _cfg = c;
    return true;
}

int8_t AudioManager::getBuzzerPin() const {
    return _cfg.buzzerPin;
}

bool AudioManager::isMelodyEnabled() const {
    return _cfg.melodyEnabled != 0;
}

/* =========================================================================
 * SIGNATURE SONORE — TÂCHE ÉPHÉMÈRE
 * ========================================================================= */

void AudioManager::playStartupMelodyIfEnabled() {
    /* Condition d'exécution : mélodie cochée en NVS ET GPIO valide (-1 = non) */
    if (!isMelodyEnabled() || _cfg.buzzerPin < 0) {
        Serial.println("[AUDIO] Mélodie de démarrage : désactivée ou GPIO non assigné");
        return;
    }
    if (_busy) return;   /* une mélodie est déjà en cours */

    _busy = true;
    /* Tâche éphémère non bloquante (Core Wi-Fi, priorité basse) : elle joue
     * la mélodie puis se détruit — setup() n'attend jamais. */
    if (xTaskCreatePinnedToCore(_melodyTaskEntry, "Melody", STACK_SIZE_AUDIO,
                                this, PRIORITY_AUDIO, nullptr, CORE_WIFI) != pdPASS) {
        _busy = false;
        Serial.println("[AUDIO] Impossible de créer la tâche mélodie (mémoire insuffisante)");
        return;
    }
    Serial.println("[AUDIO] Mélodie de démarrage : lecture en tâche de fond ("
                   + String(AUDIO_MELODY_NOTES) + " notes, volume "
                   + String(_cfg.melodyVolume) + " %, thème Rencontres du 3ème type)");
}

void AudioManager::_melodyTaskEntry(void* param) {
    static_cast<AudioManager*>(param)->_playMelody();
    /* Jamais atteint : _playMelody se termine par vTaskDelete(NULL) */
}

void AudioManager::_playMelody() {
    const uint8_t pin = (uint8_t)_cfg.buzzerPin;

    /* LEDC (API core 3.x, pilotage par broche — canal alloué
     * automatiquement) : ledcWriteTone fixe la fréquence de la note mais
     * force le rapport cyclique à 50 % ; le volume est donc appliqué juste
     * après par ledcWrite (duty proportionnel au pourcentage réglé). */
    ledcAttach(pin, AUDIO_MELODY_FREQ_HZ[0], BUZZER_LEDC_RESOLUTION);

    /* Rapport cyclique correspondant au volume réglé (1-100 %) : du très
     * faible (son doux) au maximum autorisé par la résolution 10 bits. */
    const uint32_t dutyMax = (1u << BUZZER_LEDC_RESOLUTION) - 1u;
    const uint32_t duty    = (uint32_t)_cfg.melodyVolume * dutyMax / 100u;

    for (uint8_t i = 0; i < AUDIO_MELODY_NOTES; i++) {
        ledcWriteTone(pin, AUDIO_MELODY_FREQ_HZ[i]);   /* fréquence de la note */
        ledcWrite(pin, duty);                          /* volume (duty recalculé après le tone) */
        vTaskDelay(pdMS_TO_TICKS(AUDIO_MELODY_DUR_MS[i]));
        /* Silence entre deux notes : rapport cyclique à 0 */
        ledcWrite(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(BUZZER_SILENCE_MS));
    }

    /* Fin de mélodie : canal silencieux, broche relâchée au repos bas. */
    ledcWrite(pin, 0);
    ledcDetach(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    _busy = false;
    vTaskDelete(NULL);   /* tâche éphémère : auto-destruction */
}
