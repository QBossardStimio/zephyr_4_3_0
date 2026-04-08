/*
 * uart_relay.h
 *
 * Protocole SOTP (STIMIO Object Transfer Protocol) sur UART entre
 * le NINA-B301 (nRF52840) et l'iMX6 Linux (/dev/ttymxc7).
 *
 * Format de trame :
 *   [0xAA][0x55][TYPE][LEN_LE_4B][PAYLOAD...][CRC16_LE_2B]
 *
 * CRC16 : CRC16-CCITT (poly 0x1021, init 0xFFFF) sur TYPE + LEN + PAYLOAD.
 *
 * Types de trames :
 *   0x01 OBJ_FROM_BLE : objet reçu de BLE → iMX6
 *   0x02 OBJ_TO_BLE   : objet à envoyer vers BLE ← iMX6
 *   0x03 ACK          : accusé de réception
 *   0x04 NACK         : erreur (1 octet payload = code erreur)
 *   0x05 STATUS       : statut périodique NINA
 */

#ifndef UART_RELAY_H_
#define UART_RELAY_H_

#include <stdint.h>
#include <stddef.h>

/** Types de trames SOTP */
#define SOTP_TYPE_OBJ_FROM_BLE  0x01
#define SOTP_TYPE_OBJ_TO_BLE    0x02
#define SOTP_TYPE_ACK           0x03
#define SOTP_TYPE_NACK          0x04
#define SOTP_TYPE_STATUS        0x05

/** Codes d'erreur NACK */
#define SOTP_ERR_BUFFER_FULL    0x01
#define SOTP_ERR_TIMEOUT        0x02
#define SOTP_ERR_CRC            0x03

/**
 * @brief Initialise l'UART et le thread de réception SOTP.
 * @return 0 si succès, code d'erreur négatif sinon.
 */
int uart_relay_init(void);

/**
 * @brief Envoie un objet reçu de BLE vers l'iMX6 (trame OBJ_FROM_BLE).
 * @param data    Pointeur vers les données de l'objet.
 * @param len     Longueur des données en octets.
 * @return 0 si succès, code d'erreur négatif sinon.
 */
int uart_relay_send_object(const uint8_t *data, size_t len);

#endif /* UART_RELAY_H_ */
