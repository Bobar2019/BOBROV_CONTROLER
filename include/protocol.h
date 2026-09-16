/**
 * @file protocol.h
 * @brief Définition du protocole série binaire RPi5 ↔ ESP32-S3.
 *
 * Structures packed des trames descendante (39 octets) et montante (71 octets),
 * constantes de protocole, et fonctions de calcul/vérification CRC16-CCITT.
 *
 * Polynôme CRC : 0x1021 (CCITT), valeur initiale : 0xFFFF.
 *
 * @author Didier Dero
 * @version 1.0.0
 * @date Août 2026
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
 * TAILLES DES TRAMES
 * ========================================================================= */

/** @brief Taille totale de la trame descendante en octets */
#define DL_FRAME_SIZE   39

/** @brief Taille totale de la trame montante en octets */
#define UL_FRAME_SIZE   71

/** @brief Nombre de canaux PWM dans une trame */
#define PROTO_NUM_PWM   16

/* =========================================================================
 * CONSTANTES DE MODE DE PILOTAGE
 * ========================================================================= */

#define MODE_PASSIF         0   ///< Mode Passif (Manuel direct — PWM_in = PWM_out)
#define MODE_AUTO_ROULIS    1   ///< Stabilisation Roll + Pitch automatique via PID (M5-M8)
#define MODE_AUTO_FULL      2   ///< Stabilisation complète Roll+Pitch+Yaw+Altitude

/* =========================================================================
 * CONSTANTES D'ÉTAT D'ARMEMENT
 * ========================================================================= */

#define ARM_DISARMED        0   ///< Propulseurs désarmés
#define ARM_ARMED           1   ///< Propulseurs armés et actifs
#define ARM_ESTOP           2   ///< Arrêt d'urgence immédiat

/* =========================================================================
 * BITS DE STATUT (TRAME MONTANTE)
 * ========================================================================= */

#define STATUS_BIT_ARMED    0x02    ///< Bit 1 : Système armé (bit 0 réservé)
#define STATUS_BIT_AUTO     0x04    ///< Bit 2 : Autopilote actif
#define STATUS_BIT_WDG      0x08    ///< Bit 3 : Watchdog série déclenché
#define STATUS_BIT_DRYRUN   0x10    ///< Bit 4 : Sorties physiques neutralisées (Dry-run / Mode Témoin)

/* =========================================================================
 * STRUCTURES BINAIRES PACKED
 * ========================================================================= */

#pragma pack(push, 1)

/**
 * @brief Trame descendante : RPi 5 → ESP32-S3 (39 octets).
 *
 * Contient les consignes de commande envoyées par le cockpit Raspberry Pi :
 * mode opérationnel, état d'armement et valeurs PWM pour 16 canaux.
 *
 * Format : [0xAA][0x55][TYPE][MODE][ARM][PWM_0..PWM_15][CRC16_HI][CRC16_LO]
 */
typedef struct {
    uint8_t  header1;                       ///< 0xAA
    uint8_t  header2;                       ///< 0x55
    uint8_t  type;                          ///< 0x01 (commande)
    uint8_t  mode;                          ///< Mode : 0=Manuel, 1=AutoRoll, 2=Depth, 3=Full
    uint8_t  arm_state;                     ///< 0=Désarmé, 1=Armé, 2=E-Stop
    uint16_t pwm[PROTO_NUM_PWM];            ///< Consignes PWM en µs (1000-2000)
    uint16_t crc16;                         ///< CRC16-CCITT calculé sur octets [2..36]
} DownlinkFrame_t;

/**
 * @brief Trame montante : ESP32-S3 → RPi 5 (71 octets).
 *
 * Télémétrie complète incluant attitude IMU, pression, température,
 * mesures de puissance (3 wattmètres) et valeurs PWM effectives.
 *
 * Format : [0x55][0xAA][TYPE][STATUS][QUAT_W,X,Y,Z][GYRO_X,Y,Z]
 *          [PRESS][TEMP][INA_V1,I1,V2,I2,V3,I3][PWM_0..15][CRC16]
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
    uint16_t pwm_actual[PROTO_NUM_PWM];     ///< PWM effectifs envoyés (µs)
    uint16_t crc16;                         ///< CRC16-CCITT sur octets [2..68]
} UplinkFrame_t;

#pragma pack(pop)

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
 * Calcule le CRC sur les octets [2..size-5] et écrit le résultat
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
