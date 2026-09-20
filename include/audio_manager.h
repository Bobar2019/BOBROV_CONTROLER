/**
 * @file audio_manager.h
 * @brief Signature sonore de démarrage sur buzzer passif (LEDC, v1.0.0).
 *
 * Mélodie du thème « Rencontres du 3ème type » (5 notes : Ré5, Mi5, Do5,
 * Do4, Sol4) générée par le timer matériel LEDC : fréquences fixées par
 * ledcWriteTone puis rapport cyclique modulé par ledcWrite selon le volume
 * réglé (1-100 %), silence de BUZZER_SILENCE_MS entre chaque note (rapport
 * cyclique ramené à 0 pour bien détacher les notes). GPIO, activation et
 * volume configurables depuis l'onglet Paramètres et persistés en NVS
 * (namespace « audio », clés buzz_pin / melody_en / buzz_vol ; 0xFF = non
 * assigné).
 *
 * La lecture est encapsulée dans une tâche FreeRTOS éphémère « Melody »
 * (Core Wi-Fi, priorité basse) créée à la fin de setup() et auto-détruite
 * après la dernière note (vTaskDelete(NULL)) : le démarrage du système et
 * la télémétrie 100 Hz du Core 1 ne sont jamais bloqués. La broche n'est
 * attachée au canal LEDC qu'au moment de jouer puis relâchée au repos bas
 * — le buzzer ne consomme que pendant la mélodie (~2,9 s).
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Septembre 2026
 */

#ifndef AUDIO_MANAGER_H
#define AUDIO_MANAGER_H

#include <Arduino.h>
#include "config.h"

/**
 * @brief Configuration du module acoustique (persistée en NVS « audio »).
 *
 * Clés : buzz_pin (UChar, STATUS_IO_PIN_NONE = non assigné), melody_en
 * (UChar : 0 = mélodie de démarrage désactivée, 1 = activée), buzz_vol
 * (UChar : volume de la mélodie en %, 1-100).
 */
struct AudioConfig {
    int8_t  buzzerPin;      ///< GPIO du buzzer passif (-1 = non assigné)
    uint8_t melodyEnabled;  ///< 1 = jouer la mélodie au démarrage, 0 = non
    uint8_t melodyVolume;   ///< Volume 1-100 % (duty = % du maximum de résolution)
};

/**
 * @brief Buzzer passif : mélodie de démarrage non bloquante (LEDC + tâche
 *        éphémère).
 *
 * Thread-safety : la configuration se résume à trois scalaires de 8 bits,
 * lus/écrits atomiquement (handlers Web) et relus au lancement de la tâche
 * mélodie — qui ne joue qu'au démarrage, bien avant la première requête
 * Web. Aucun verrou nécessaire.
 */
class AudioManager {
public:
    AudioManager();

    /**
     * @brief Charge la configuration NVS (namespace « audio ») et journalise
     *        l'état du module. À appeler depuis setup() — n'émet aucun son.
     */
    void begin();

    /** @brief Copie de la configuration courante (handlers Web) */
    void getConfig(AudioConfig& out) const;

    /**
     * @brief Applique et persiste une nouvelle configuration (NVS vérifiée
     *        par relecture, bornes clampées). Le GPIO n'étant sollicité
     *        qu'au moment de jouer la mélodie, la modification s'applique
     *        au prochain démarrage — sans redémarrage immédiat.
     *
     * @return true si l'écriture NVS est confirmée.
     */
    bool setConfig(const AudioConfig& cfg);

    /** @brief GPIO du buzzer (-1 = non assigné) */
    int8_t getBuzzerPin() const;

    /** @brief true si la mélodie de démarrage est activée en NVS */
    bool isMelodyEnabled() const;

    /**
     * @brief Lance la mélodie de démarrage si elle est activée en NVS ET
     *        qu'un GPIO valide est configuré.
     *
     * Crée la tâche éphémère « Melody » (Core Wi-Fi, priorité basse) qui
     * joue les 5 notes puis se détruit elle-même — retour immédiat, aucune
     * attente dans setup().
     */
    void playStartupMelodyIfEnabled();

private:
    AudioConfig   _cfg;    ///< Configuration active (NVS)
    volatile bool _busy;   ///< true = tâche mélodie en cours (anti-réentrance)

    /** @brief Entrée de la tâche éphémère (joue la mélodie puis se détruit) */
    static void _melodyTaskEntry(void* param);
    void _playMelody();
};

/* Instance globale extern (définie dans audio_manager.cpp) */
extern AudioManager g_audio;

#endif /* AUDIO_MANAGER_H */
