/*
 * uart_relay.c
 *
 * Protocole SOTP (STIMIO Object Transfer Protocol) sur UART entre
 * NINA-B301 (nRF52840) et iMX6 Linux (/dev/ttymxc7).
 *
 * Ce module remplace le transport HCI H4 du firmware hci_uart.
 * L'UART0 (même pins physiques) transporte maintenant des trames SOTP
 * au lieu du protocole HCI.
 *
 * Format de trame SOTP :
 *   [SYNC_0=0xAA][SYNC_1=0x55][TYPE][LEN_B0][LEN_B1][LEN_B2][LEN_B3][PAYLOAD...][CRC16_LO][CRC16_HI]
 *   Total overhead : 9 octets (2 sync + 1 type + 4 len + 2 crc)
 *
 * CRC16 : CRC16-CCITT (poly 0x1021, seed 0xFFFF) calculé sur TYPE + LEN + PAYLOAD.
 *
 * Architecture des threads :
 *   TX : polling synchrone (uart_poll_out), appelé depuis les callbacks OTS (thread BT RX).
 *        Simple et fiable pour une première implémentation.
 *        TODO : passer à IRQ-driven si la latence TX devient problématique.
 *
 *   RX : thread dédié "sotp_rx" (K_PRIO_PREEMPT(8)), polling uart_poll_in.
 *        State machine à 7 états : WAIT_SYNC_0 → WAIT_SYNC_1 → TYPE →
 *        LEN → PAYLOAD → CRC_LO → CRC_HI.
 *        TODO : passer à uart_irq_callback_set() pour réduire la charge CPU.
 *
 * Note device UART :
 *   On utilise le chosen node `zephyr,sotp-uart` défini dans l'overlay DTS
 *   (boards/railnet200_nina_b301_nrf52840.overlay). Ce chosen pointe vers uart0,
 *   le même UART physique que hci_uart mais avec un rôle sémantique distinct.
 *
 * Auteur : STIMIO / équipe développement embarqué
 */

#include "uart_relay.h"
#include "ots_handler.h"

#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

LOG_MODULE_REGISTER(uart_relay, LOG_LEVEL_DBG);

/* -------------------------------------------------------------------------
 * Device UART
 *
 * DT_CHOSEN(zephyr_sotp_uart) résout le chosen node `zephyr,sotp-uart`
 * défini dans boards/railnet200_nina_b301_nrf52840.overlay → &uart0.
 *
 * La convention Zephyr convertit les tirets en underscores dans DT_CHOSEN :
 *   `zephyr,sotp-uart`  →  DT_CHOSEN(zephyr_sotp_uart)
 * ------------------------------------------------------------------------- */
static const struct device *const uart_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_sotp_uart));

/* -------------------------------------------------------------------------
 * Constantes SOTP
 * ------------------------------------------------------------------------- */
#define SOTP_SYNC_0      0xAAU
#define SOTP_SYNC_1      0x55U
#define SOTP_CRC_SEED    0xFFFFU

/** Taille maximale d'une trame SOTP RX (payload OBJ_TO_BLE = objet entier depuis iMX6) */
#define SOTP_RX_BUF_SIZE  (OTS_HANDLER_OBJ_MAX_SIZE + 16U)

/* -------------------------------------------------------------------------
 * Statistiques atomiques
 * Utiles pour le thread stats dans main.c.
 * ------------------------------------------------------------------------- */
static atomic_t stat_tx_bytes;
static atomic_t stat_rx_frames;
static atomic_t stat_rx_errors;

/* -------------------------------------------------------------------------
 * ACK/NACK synchronisation pour retry
 *
 * uart_relay_send_object() envoie la trame SOTP puis attend un ACK/NACK
 * du sotp-bridge iMX6. Le thread sotp_rx poste le résultat via ce semaphore.
 * ------------------------------------------------------------------------- */
#define SOTP_TX_MAX_RETRIES  3
#define SOTP_TX_ACK_TIMEOUT  K_SECONDS(5)

static K_SEM_DEFINE(tx_ack_sem, 0, 1);
static atomic_t tx_ack_result;  /* 0 = ACK, >0 = NACK code */

/* -------------------------------------------------------------------------
 * Fonctions CRC
 *
 * CRC16-CCITT calculé sur les champs TYPE(1) + LEN(4) + PAYLOAD(N).
 * On utilise crc16_ccitt() de Zephyr (poly=0x1021, seed configurable).
 * ------------------------------------------------------------------------- */
static uint16_t sotp_crc(uint8_t type, uint32_t len, const uint8_t *payload)
{
	uint16_t crc = SOTP_CRC_SEED;

	/* Inclure TYPE dans le CRC */
	crc = crc16_ccitt(crc, &type, 1);

	/* Inclure LEN (4 octets little-endian) dans le CRC */
	uint8_t len_bytes[4] = {
		(uint8_t)(len & 0xFFU),
		(uint8_t)((len >> 8) & 0xFFU),
		(uint8_t)((len >> 16) & 0xFFU),
		(uint8_t)((len >> 24) & 0xFFU),
	};
	crc = crc16_ccitt(crc, len_bytes, sizeof(len_bytes));

	/* Inclure PAYLOAD dans le CRC (si présent) */
	if (payload && len > 0U) {
		crc = crc16_ccitt(crc, payload, (size_t)len);
	}

	return crc;
}

/* -------------------------------------------------------------------------
 * TX : envoi d'une trame SOTP
 *
 * Utilise uart_poll_out() — bloquant par octet, simple et fiable.
 * Appelé depuis les callbacks OTS (thread BT RX) — pas de ISR context.
 * ------------------------------------------------------------------------- */

/**
 * @brief Envoie un octet sur l'UART en mode polling.
 */
static inline void sotp_tx_byte(uint8_t byte)
{
	uart_poll_out(uart_dev, byte);
	atomic_inc(&stat_tx_bytes);
}

/**
 * @brief Encode et envoie une trame SOTP complète.
 *
 * Structure : [0xAA][0x55][TYPE][LEN_LE32][PAYLOAD][CRC16_LE16]
 *
 * @param type    Type de trame (SOTP_TYPE_*)
 * @param payload Données payload (peut être NULL si len == 0)
 * @param len     Longueur du payload en octets
 */
static void sotp_send_frame(uint8_t type, const uint8_t *payload, uint32_t len)
{
	uint16_t crc = sotp_crc(type, len, payload);

	/* SYNC */
	sotp_tx_byte(SOTP_SYNC_0);
	sotp_tx_byte(SOTP_SYNC_1);

	/* TYPE */
	sotp_tx_byte(type);

	/* LEN (little-endian 32 bits) */
	sotp_tx_byte((uint8_t)(len & 0xFFU));
	sotp_tx_byte((uint8_t)((len >> 8) & 0xFFU));
	sotp_tx_byte((uint8_t)((len >> 16) & 0xFFU));
	sotp_tx_byte((uint8_t)((len >> 24) & 0xFFU));

	/* PAYLOAD */
	for (uint32_t i = 0U; i < len; i++) {
		sotp_tx_byte(payload[i]);
	}

	/* CRC16 (little-endian 16 bits) */
	sotp_tx_byte((uint8_t)(crc & 0xFFU));
	sotp_tx_byte((uint8_t)((crc >> 8) & 0xFFU));

	LOG_DBG("sotp_send_frame: type=0x%02x len=%u crc=0x%04x", type, len, crc);
}

/* -------------------------------------------------------------------------
 * API publique TX
 * ------------------------------------------------------------------------- */

int uart_relay_send_object(const uint8_t *data, size_t len)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("uart_relay_send_object: UART not ready");
		return -ENODEV;
	}

	if (len > OTS_HANDLER_OBJ_MAX_SIZE) {
		LOG_ERR("uart_relay_send_object: len %zu > max %d",
			len, OTS_HANDLER_OBJ_MAX_SIZE);
		return -ENOMEM;
	}

	LOG_INF("uart_relay_send_object: %zu bytes → iMX6", len);
	sotp_send_frame(SOTP_TYPE_OBJ_FROM_BLE, data, (uint32_t)len);

	/* Ne pas bloquer ici — obj_write est appelé depuis le thread BT RX.
	 * Bloquer ici gèlerait le stack BLE (GATT Unlikely Error sur le chunk suivant).
	 * Le ACK/NACK du sotp-bridge est loggé par sotp_dispatch mais pas attendu.
	 * Le retry sera géré dans une future version via un work queue dédié.
	 */

	return 0;
}

/* -------------------------------------------------------------------------
 * RX : state machine de réception SOTP
 * ------------------------------------------------------------------------- */

/**
 * @brief États de la state machine de réception.
 *
 * WAIT_SYNC_0 → WAIT_SYNC_1 → TYPE → LEN → PAYLOAD → CRC_LO → CRC_HI
 *                                                               ↓
 *                                                         vérification CRC
 *                                                         → WAIT_SYNC_0
 */
enum sotp_rx_state {
	SOTP_RX_WAIT_SYNC_0,
	SOTP_RX_WAIT_SYNC_1,
	SOTP_RX_TYPE,
	SOTP_RX_LEN,
	SOTP_RX_PAYLOAD,
	SOTP_RX_CRC_LO,
	SOTP_RX_CRC_HI,
};

/** Contexte de réception — état de la state machine entre les octets */
static struct {
	enum sotp_rx_state state;
	uint8_t  type;
	uint8_t  len_buf[4];   /* accumule les 4 octets LEN */
	uint8_t  len_idx;      /* 0..3 pendant la réception de LEN */
	uint32_t payload_len;  /* LEN décodé */
	uint32_t payload_rcvd; /* octets payload déjà reçus */
	uint8_t  payload[SOTP_RX_BUF_SIZE]; /* buffer payload complet */
	uint8_t  crc_lo;       /* premier octet CRC */
} rx_ctx;

/**
 * @brief Traite une trame SOTP complète et validée reçue depuis l'iMX6.
 */
static void sotp_dispatch(uint8_t type, const uint8_t *payload, uint32_t len)
{
	atomic_inc(&stat_rx_frames);

	switch (type) {
	case SOTP_TYPE_OBJ_TO_BLE:
		/*
		 * L'iMX6 envoie un objet à destination d'un client BLE.
		 * On l'ajoute dans le pool OTS — un client connecté pourra le lire.
		 *
		 * Nom généré automatiquement : "imx6_obj_N" (N = compteur).
		 */
		LOG_INF("sotp_dispatch: OBJ_TO_BLE len=%u → OTS pool", len);
		{
			static uint32_t rx_obj_counter;
			char name[32];

			snprintf(name, sizeof(name), "imx6_obj_%u",
				 rx_obj_counter++);

			int err = ots_handler_add_object_from_uart(
					payload, (size_t)len, name);
			if (err) {
				LOG_ERR("sotp_dispatch: ots_handler_add_object_from_uart "
					"failed: %d", err);
				sotp_send_frame(SOTP_TYPE_NACK,
						(uint8_t[]){SOTP_ERR_BUFFER_FULL},
						1U);
			} else {
				sotp_send_frame(SOTP_TYPE_ACK, NULL, 0U);
			}
		}
		break;

	case SOTP_TYPE_ACK:
		LOG_DBG("sotp_dispatch: ACK from iMX6");
		atomic_set(&tx_ack_result, 0);
		k_sem_give(&tx_ack_sem);
		break;

	case SOTP_TYPE_NACK:
		if (len >= 1U) {
			LOG_WRN("sotp_dispatch: NACK from iMX6, code=0x%02x",
				payload[0]);
			atomic_set(&tx_ack_result, (atomic_val_t)payload[0]);
		} else {
			atomic_set(&tx_ack_result, 0xFF);
		}
		k_sem_give(&tx_ack_sem);
		break;

	case SOTP_TYPE_STATUS:
		/* Statut périodique de l'iMX6 — log informatif */
		LOG_INF("sotp_dispatch: STATUS from iMX6 (len=%u)", len);
		break;

	default:
		LOG_WRN("sotp_dispatch: unknown type 0x%02x", type);
		break;
	}
}

/**
 * @brief Traite un octet entrant dans la state machine SOTP.
 *
 * Appelé pour chaque octet reçu depuis uart_poll_in() dans le thread sotp_rx.
 * La state machine avance d'un état à la fois — pas de buffer circulaire nécessaire.
 */
static void sotp_rx_byte(uint8_t byte)
{
	switch (rx_ctx.state) {
	case SOTP_RX_WAIT_SYNC_0:
		if (byte == SOTP_SYNC_0) {
			rx_ctx.state = SOTP_RX_WAIT_SYNC_1;
		}
		/* Sinon : bruit sur l'UART, on attend le prochain 0xAA */
		break;

	case SOTP_RX_WAIT_SYNC_1:
		if (byte == SOTP_SYNC_1) {
			rx_ctx.state = SOTP_RX_TYPE;
		} else {
			/*
			 * Faux SYNC_0 (ex: payload contenant 0xAA suivi d'autre chose).
			 * Retour en WAIT_SYNC_0, mais si l'octet courant est 0xAA
			 * il pourrait être le début d'un nouveau SYNC — on teste.
			 */
			atomic_inc(&stat_rx_errors);
			rx_ctx.state = (byte == SOTP_SYNC_0) ?
				SOTP_RX_WAIT_SYNC_1 : SOTP_RX_WAIT_SYNC_0;
		}
		break;

	case SOTP_RX_TYPE:
		rx_ctx.type    = byte;
		rx_ctx.len_idx = 0U;
		rx_ctx.state   = SOTP_RX_LEN;
		break;

	case SOTP_RX_LEN:
		rx_ctx.len_buf[rx_ctx.len_idx++] = byte;
		if (rx_ctx.len_idx == 4U) {
			/* Décoder LEN little-endian 32 bits */
			rx_ctx.payload_len =
				(uint32_t)rx_ctx.len_buf[0]          |
				((uint32_t)rx_ctx.len_buf[1] << 8)   |
				((uint32_t)rx_ctx.len_buf[2] << 16)  |
				((uint32_t)rx_ctx.len_buf[3] << 24);

			rx_ctx.payload_rcvd = 0U;

			if (rx_ctx.payload_len > SOTP_RX_BUF_SIZE) {
				LOG_ERR("sotp_rx: payload too large (%u > %u), resync",
					rx_ctx.payload_len,
					(uint32_t)SOTP_RX_BUF_SIZE);
				atomic_inc(&stat_rx_errors);
				rx_ctx.state = SOTP_RX_WAIT_SYNC_0;
			} else if (rx_ctx.payload_len == 0U) {
				/* Pas de payload → aller directement au CRC */
				rx_ctx.state = SOTP_RX_CRC_LO;
			} else {
				rx_ctx.state = SOTP_RX_PAYLOAD;
			}
		}
		break;

	case SOTP_RX_PAYLOAD:
		rx_ctx.payload[rx_ctx.payload_rcvd++] = byte;
		if (rx_ctx.payload_rcvd == rx_ctx.payload_len) {
			rx_ctx.state = SOTP_RX_CRC_LO;
		}
		break;

	case SOTP_RX_CRC_LO:
		rx_ctx.crc_lo = byte;
		rx_ctx.state  = SOTP_RX_CRC_HI;
		break;

	case SOTP_RX_CRC_HI: {
		uint16_t recv_crc = (uint16_t)rx_ctx.crc_lo |
				    ((uint16_t)byte << 8);
		uint16_t calc_crc = sotp_crc(rx_ctx.type,
					     rx_ctx.payload_len,
					     rx_ctx.payload);

		if (recv_crc != calc_crc) {
			LOG_ERR("sotp_rx: CRC mismatch recv=0x%04x calc=0x%04x",
				recv_crc, calc_crc);
			sotp_send_frame(SOTP_TYPE_NACK,
					(uint8_t[]){SOTP_ERR_CRC}, 1U);
			atomic_inc(&stat_rx_errors);
		} else {
			sotp_dispatch(rx_ctx.type, rx_ctx.payload,
				      rx_ctx.payload_len);
		}

		/* Retour à l'attente du prochain SYNC dans tous les cas */
		rx_ctx.state = SOTP_RX_WAIT_SYNC_0;
		break;
	}

	default:
		/* Ne devrait jamais arriver — reset défensif */
		LOG_ERR("sotp_rx: invalid state %d, resync", rx_ctx.state);
		rx_ctx.state = SOTP_RX_WAIT_SYNC_0;
		break;
	}
}

/* -------------------------------------------------------------------------
 * Thread RX UART
 * ------------------------------------------------------------------------- */

/*
 * Stack du thread sotp_rx.
 * 1024 octets :
 *   - sotp_rx_byte() : state machine légère (~100 octets stack frame)
 *   - sotp_dispatch() → ots_handler_add_object_from_uart() : ~256 octets
 *   - LOG_INF() avec RTT : ~128 octets
 *   Total estimé : ~500 octets → 1024 avec marge.
 */
#define SOTP_RX_STACK_SIZE 1024

static K_THREAD_STACK_DEFINE(sotp_rx_stack, SOTP_RX_STACK_SIZE);
static struct k_thread sotp_rx_thread_data;

static void sotp_rx_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t byte;

	LOG_INF("sotp_rx thread started");

	while (1) {
		/*
		 * uart_poll_in() : retourne 0 si un octet est disponible, -1 sinon.
		 * On cède le CPU (k_sleep 1ms) quand il n'y a rien — évite le
		 * busy-wait qui monopoliserait le CPU.
		 *
		 * Alternative plus efficace : uart_irq_callback_set() avec FIFO.
		 * Laissé en polling pour simplifier le debug initial.
		 * TODO : passer IRQ-driven si le 1ms de latence pose problème.
		 */
		if (uart_poll_in(uart_dev, &byte) == 0) {
			sotp_rx_byte(byte);
		} else {
			k_sleep(K_MSEC(1));
		}
	}
}

/* -------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

int uart_relay_init(void)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("uart_relay_init: UART device '%s' not ready",
			uart_dev->name);
		return -ENODEV;
	}

	/* Initialiser la state machine RX à l'état initial */
	memset(&rx_ctx, 0, sizeof(rx_ctx));
	rx_ctx.state = SOTP_RX_WAIT_SYNC_0;

	/* Lancer le thread de réception SOTP */
	k_thread_create(&sotp_rx_thread_data,
			sotp_rx_stack,
			K_THREAD_STACK_SIZEOF(sotp_rx_stack),
			sotp_rx_thread_fn,
			NULL, NULL, NULL,
			K_PRIO_PREEMPT(8),
			0,
			K_NO_WAIT);
	k_thread_name_set(&sotp_rx_thread_data, "sotp_rx");

	LOG_INF("uart_relay_init: UART '%s' ready, sotp_rx thread started",
		uart_dev->name);

	return 0;
}
