/**
 * @file protocol.h
 * @brief Définition du protocole série binaire RPi5 ↔ ESP32-S3.
 *
 * Structures packed des trames descendante (40 octets) et montante (73 octets),
 * trames de configuration des noms (0x03/0x04 — 27 octets) et requête de
 * synchronisation (0xFF), constantes de protocole, et fonctions de
 * calcul/vérification CRC16-CCITT.
 *
 * Polynôme CRC : 0x1021 (CCITT), valeur initiale : 0xFFFF.
 *
 * @author Didier Dero
 * @version 1.5.0
 * @date Septembre 2026
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * CONSTANTES D'EN-TÊTE DE TRAME
 * ========================================================================= */

/** @brief Octets d'en-tête de la trame descendante (RPi → ESP32) */
#define DL_HEADER_1     0xAA
#define DL_HEADER_2     0x55

/** @brief Octets d'en-tête de la trame montante (ESP32 → RPi) */
#define UL_HEADER_1     0x55
#define UL_HEADER_2     0xAA

/** @brief Identifiant de type pour la trame descendante */
#define DL_FRAME_TYPE   0x01

/** @brief Identifiant de type pour la trame montante */
#define UL_FRAME_TYPE   0x02

/* =========================================================================
 * TRAMES DE SYNCHRONISATION DE CONFIGURATION (v1.5.0)
 * =========================================================================
 *
 * Source unique de vérité : l'ESP32 détient la configuration matérielle
 * (noms personnalisés des sorties, NVS) et la transmet au RPi 5, qui adapte
 * son cockpit sans aucune valeur codée en dur.
 *
 * - Trame descendante 0xFF « requête de synchronisation » : envoyée par le
 *   RPi 5 (à son démarrage notamment) — trame de 40 octets, contenu ignoré,
 *   elle déclenche le renvoi de la configuration.
 * - Trames montantes 0x03/0x04 « configuration des noms » : une trame de
 *   27 octets par sortie (une par canal PWM 10-15, une par sortie TOR 1-4).
 *
 * Déclencheurs d'envoi (identiques côté firmware) : requête 0xFF, première
 * commande reçue après (re)connexion, ou modification des noms depuis
 * l'interface Web (push à chaud). */

/** @brief Type de trame descendante « requête de synchronisation » (contenu ignoré) */
#define DL_FRAME_TYPE_SYNC  0xFF

/** @brief Type de trame montante : noms des canaux PWM auxiliaires (10-15) */
#define UL_FRAME_TYPE_CFG_PWM   0x03

/** @brief Type de trame montante : noms des 4 sorties tout-ou-rien (TOR) */
#define UL_FRAME_TYPE_CFG_GPIO  0x04

/** @brief Longueur max du nom embarqué dans une trame de configuration (octets UTF-8) */
#define CFG_NAME_MAX_LEN    20

/** @brief Taille totale d'une trame de configuration des noms (octets) */
#define CFG_FRAME_SIZE      27

/* =========================================================================
 * TAILLES DES TRAMES
 * ========================================================================= */

/** @brief Taille totale de la trame descendante en octets (inclut gpio_cmd) */
#define DL_FRAME_SIZE   40

/** @brief Taille totale de la trame montante en octets (inclut gpio_state) */
#define UL_FRAME_SIZE   73

/** @brief Nombre de canaux PWM dans une trame */
#define PROTO_NUM_PWM   16

/* =========================================================================
 * MODE DE PILOTAGE — MASQUE DE BITS SUPERPOSABLES (TRAME DESCENDANTE)
 * =========================================================================
 *
 * Le champ `mode` n'est plus une valeur énumérée : chaque bit active une
 * assistance indépendante et plusieurs bits se COMBINENT (ex. 0x03 = Auto
 * R/T + Tenue de profondeur). 0x00 = Passif (manuel direct). Les bits 4-7
 * sont ignorés (masqués à la réception). */

#define MODE_BIT_RT       0x01  ///< Bit 0 : Auto Roulis + Tangage (PID sur les verticaux M5-M8)
#define MODE_BIT_DEPTH    0x02  ///< Bit 1 : Tenue de profondeur (PID altitude sur M5-M8)
#define MODE_BIT_CAP      0x04  ///< Bit 2 : Auto Cap (PID lacet sur les horizontaux M1-M4)
#define MODE_BIT_SURFACE  0x08  ///< Bit 3 : Retour surface (PRIORITAIRE — verticaux vers la surface)
#define MODE_MASK_ALL     0x0F  ///< Masque des bits valides (bits 4-7 ignorés)

#define MODE_PASSIF       0x00  ///< Aucune assistance (manuel direct — PWM_in = PWM_out)

/* =========================================================================
 * CONSTANTES D'ÉTAT D'ARMEMENT
 * ========================================================================= */

#define ARM_DISARMED        0   ///< Propulseurs désarmés
#define ARM_ARMED           1   ///< Propulseurs armés et actifs
#define ARM_ESTOP           2   ///< Arrêt d'urgence immédiat

/* =========================================================================
 * BITS DE STATUT (TRAME MONTANTE)
 * ========================================================================= */

#define STATUS_BIT_ARMED    0x02    ///< Bit 1 : Liaison établie (≥ 1 trame descendante reçue)
#define STATUS_BIT_RT       0x04    ///< Bit 2 : Auto Roulis + Tangage réellement actif
#define STATUS_BIT_WDG      0x08    ///< Bit 3 : Watchdog série déclenché
#define STATUS_BIT_DRYRUN   0x10    ///< Bit 4 : Sorties physiques neutralisées (Dry-run / Mode Témoin)
#define STATUS_BIT_DEPTH    0x20    ///< Bit 5 : Tenue de profondeur réellement active
#define STATUS_BIT_CAP      0x40    ///< Bit 6 : Auto Cap réellement actif
#define STATUS_BIT_SURFACE  0x80    ///< Bit 7 : Retour surface actif (commande prioritaire)

/* =========================================================================
 * STRUCTURES BINAIRES PACKED
 * ========================================================================= */

#pragma pack(push, 1)

/**
 * @brief Trame descendante : RPi 5 → ESP32-S3 (40 octets).
 *
 * Contient les consignes de commande envoyées par le cockpit Raspberry Pi :
 * mode opérationnel, état d'armement, valeurs PWM pour 16 canaux et masque
 * de commande des 4 sorties tout-ou-rien (GPIO ON/OFF).
 *
 * Format : [0xAA][0x55][TYPE][MODE][ARM][PWM_0..PWM_15][GPIO_CMD][CRC16_HI][CRC16_LO]
 */
typedef struct {
    uint8_t  header1;                       ///< 0xAA
    uint8_t  header2;                       ///< 0x55
    uint8_t  type;                          ///< 0x01 (commande)
    uint8_t  mode;                          ///< Masque d'assistances superposables (MODE_BIT_* — 0x00 = Passif)
    uint8_t  arm_state;                     ///< 0=Désarmé, 1=Armé, 2=E-Stop
    uint16_t pwm[PROTO_NUM_PWM];            ///< Consignes PWM en µs (1000-2000) — offsets 5..36
    uint8_t  gpio_cmd;                      ///< Masque 4 bits sorties ON/OFF (bit 0 = sortie 1 … bit 3 = sortie 4) — offset 37
    uint16_t crc16;                         ///< CRC16-CCITT sur octets [2..37] — offsets 38 (hi) / 39 (lo)
} DownlinkFrame_t;

/**
 * @brief Trame montante : ESP32-S3 → RPi 5 (73 octets).
 *
 * Télémétrie complète incluant attitude IMU, pression, température,
 * mesures de puissance (3 wattmètres), valeurs PWM effectives et état
 * réel des 4 sorties tout-ou-rien (GPIO ON/OFF).
 *
 * Format : [0x55][0xAA][TYPE][STATUS][QUAT_W,X,Y,Z][GYRO_X,Y,Z]
 *          [PRESS][TEMP][INA_V1,I1,V2,I2,V3,I3][PWM_0..15][GPIO_STATE][CRC16]
 */
typedef struct {
    uint8_t  header1;                       ///< 0x55
    uint8_t  header2;                       ///< 0xAA
    uint8_t  type;                          ///< 0x02 (télémétrie)
    uint8_t  status;                        ///< Bits de statut (voir STATUS_BIT_*)
    int16_t  quat[4];                       ///< Quaternions W,X,Y,Z (×10000) — repère VÉHICULE (remappage montage BNO085_MOUNT_YAW_DEG à la source)
    int16_t  gyro[3];                       ///< Gyroscope X,Y,Z (×100, °/s) — repère VÉHICULE (avant/droite/bas, comme le quaternion)
    int32_t  pressure;                      ///< Pression en 0.1 mbar
    int32_t  temperature;                   ///< Température en 0.01 °C
    uint16_t power[6];                      ///< [V1,I1, V2,I2, V3,I3] en mV/mA
    uint16_t pwm_actual[PROTO_NUM_PWM];     ///< PWM effectifs envoyés (µs) — offsets 38..69
    uint8_t  gpio_state;                    ///< État réel des 4 sorties ON/OFF (bit 0 = sortie 1 … bit 3 = sortie 4) — offset 70
    uint16_t crc16;                         ///< CRC16-CCITT sur octets [2..70] — offsets 71 (hi) / 72 (lo)
} UplinkFrame_t;

/**
 * @brief Trame montante de configuration des noms : ESP32-S3 → RPi 5 (27 octets).
 *
 * Émise une fois par sortie nommée (rafale lissée : une trame par cycle de
 * 10 ms, sans perturber la télémétrie 100 Hz) en réponse à : requête de
 * synchronisation 0xFF du RPi 5, première commande reçue après
 * (re)connexion, ou modification des noms via l'interface Web.
 * Une chaîne vide (name_len = 0) signifie « nom par défaut » : le RPi 5
 * applique alors son libellé par défaut pour la sortie concernée.
 *
 * Format : [0x55][0xAA][TYPE][ID][LEN][NAME ×20][CRC16_HI][CRC16_LO]
 */
typedef struct {
    uint8_t  header1;                   ///< 0x55
    uint8_t  header2;                   ///< 0xAA
    uint8_t  type;                      ///< 0x03 (config PWM) / 0x04 (config sorties TOR)
    uint8_t  id;                        ///< 0x03 : numéro de canal PWM (10-15) ; 0x04 : numéro de sortie TOR (1-4)
    uint8_t  name_len;                  ///< Longueur utile du nom en octets (0-20) — offset 4
    char     name[CFG_NAME_MAX_LEN];    ///< Nom UTF-8, complété par des 0x00 — offsets 5..24
    uint16_t crc16;                     ///< CRC16-CCITT sur octets [2..24] — offsets 25 (hi) / 26 (lo)
} ConfigFrame_t;

#pragma pack(pop)

/* =========================================================================
 * GARDE-FOUS DE COMPATIBILITÉ BINAIRE (VÉRIFIÉS À LA COMPILATION)
 * =========================================================================
 *
 * Les offsets de ce protocole sont contractuels avec le RPi 5 : toute
 * dérive silencieuse (champ oublié, padding, réordonnancement) casserait la
 * communication binaire. Ces vérifications échouent à la COMPILATION et dans
 * l'outil hôte .pio/proto_size_test.cpp. */

static_assert(sizeof(DownlinkFrame_t) == DL_FRAME_SIZE,
              "DownlinkFrame_t doit faire exactement DL_FRAME_SIZE octets");
static_assert(sizeof(UplinkFrame_t) == UL_FRAME_SIZE,
              "UplinkFrame_t doit faire exactement UL_FRAME_SIZE octets");
static_assert(offsetof(DownlinkFrame_t, gpio_cmd) == 37,
              "gpio_cmd doit etre a l'offset 37 (trame descendante)");
static_assert(offsetof(DownlinkFrame_t, crc16) == 38,
              "CRC16 descendant doit demarrer a l'offset 38");
static_assert(offsetof(UplinkFrame_t, gpio_state) == 70,
              "gpio_state doit etre a l'offset 70 (trame montante)");
static_assert(offsetof(UplinkFrame_t, crc16) == 71,
              "CRC16 montant doit demarrer a l'offset 71");
static_assert(sizeof(ConfigFrame_t) == CFG_FRAME_SIZE,
              "ConfigFrame_t doit faire exactement CFG_FRAME_SIZE octets");
static_assert(offsetof(ConfigFrame_t, name) == 5,
              "name doit etre a l'offset 5 (trame de configuration)");
static_assert(offsetof(ConfigFrame_t, crc16) == 25,
              "CRC16 de configuration doit demarrer a l'offset 25");

/* =========================================================================
 * FONCTIONS CRC16-CCITT
 * ========================================================================= */

/**
 * @brief Calcule le CRC16-CCITT sur un bloc de données.
 *
 * Algorithme : polynôme 0x1021, valeur initiale 0xFFFF,
 * traitement bit-par-bit sans table précalculée.
 *
 * @param[in] data  Pointeur vers les données source.
 * @param[in] len   Nombre d'octets à traiter.
 * @return Valeur CRC16 sur 16 bits.
 *
 * @note Utilisé pour calculer le CRC des trames montantes et vérifier
 *       l'intégrité des trames descendantes.
 */
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Vérifie l'intégrité CRC16 d'une trame complète.
 *
 * Calcule le CRC sur les octets de données (du champ type jusqu'avant
 * le CRC stocké) et le compare avec le CRC contenu dans la trame.
 *
 * @param[in] frame       Pointeur vers la trame complète.
 * @param[in] frame_size  Taille totale de la trame (incluant CRC).
 * @return true si le CRC est valide, false sinon.
 */
inline bool crc16_verify(const uint8_t* frame, size_t frame_size) {
    if (frame_size < 3) return false;
    /* Le CRC couvre les octets de l'index 2 jusqu'à frame_size-3 (inclus) */
    uint16_t computed = crc16_ccitt(frame + 2, frame_size - 4);
    uint16_t stored   = (uint16_t)(frame[frame_size - 2] << 8)
                      | (uint16_t)(frame[frame_size - 1]);
    return computed == stored;
}

/**
 * @brief Remplit le champ CRC16 d'une trame avant émission.
 *
 * Calcule le CRC sur les octets [2..size-3] et écrit le résultat
 * dans les 2 derniers octets de la trame (big-endian).
 *
 * @param[in,out] frame       Pointeur vers la trame à compléter.
 * @param[in]     frame_size  Taille totale de la trame (incluant CRC).
 */
inline void crc16_fill(uint8_t* frame, size_t frame_size) {
    if (frame_size < 3) return;
    uint16_t crc = crc16_ccitt(frame + 2, frame_size - 4);
    frame[frame_size - 2] = (uint8_t)(crc >> 8);   /* Octet haut */
    frame[frame_size - 1] = (uint8_t)(crc & 0xFF);  /* Octet bas  */
}

#endif /* PROTOCOL_H */
