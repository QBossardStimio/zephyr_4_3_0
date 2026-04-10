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
#include "config_service.h"

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
 * CRC16-CCITT reflected calculé sur les champs TYPE(1) + LEN(4) + PAYLOAD(N).
 * On utilise crc16_reflect() de Zephyr (poly=0x8408, seed=0xFFFF).
 *
 * IMPORTANT : sotp-bridge.c et sotp-send.c côté iMX6 Linux utilisent cette
 * même variante reflected (poly 0x8408 = bit-reversed de 0x1021).
 * crc16_ccitt() de Zephyr est une variante CCITT-FALSE différente qui
 * produit des résultats incompatibles → CRC mismatch systématique.
 * ------------------------------------------------------------------------- */
#define SOTP_CRC_POLY 0x8408U

static uint16_t sotp_crc(uint8_t type, uint32_t len, const uint8_t *payload)
{
	uint16_t crc = SOTP_CRC_SEED;

	/* Inclure TYPE dans le CRC */
	crc = crc16_reflect(SOTP_CRC_POLY, crc, &type, 1);

	/* Inclure LEN (4 octets little-endian) dans le CRC */
	uint8_t len_bytes[4] = {
		(uint8_t)(len & 0xFFU),
		(uint8_t)((len >> 8) & 0xFFU),
		(uint8_t)((len >> 16) & 0xFFU),
		(uint8_t)((len >> 24) & 0xFFU),
	};
	crc = crc16_reflect(SOTP_CRC_POLY, crc, len_bytes, sizeof(len_bytes));

	/* Inclure PAYLOAD dans le CRC (si présent) */
	if (payload && len > 0U) {
		crc = crc16_reflect(SOTP_CRC_POLY, crc, payload, (size_t)len);
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

int uart_relay_send_file_request(const uint8_t *path, size_t len)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("uart_relay_send_file_request: UART not ready");
		return -ENODEV;
	}

	LOG_INF("uart_relay_send_file_request: '%.*s' (%zu bytes) → iMX6",
		(int)len, (const char *)path, len);
	sotp_send_frame(SOTP_TYPE_FILE_REQUEST, path, (uint32_t)len);

	return 0;
}

void uart_relay_send_delete_ack(void)
{
	if (!device_is_ready(uart_dev)) {
		return;
	}

	LOG_DBG("uart_relay_send_delete_ack: DELETE_ACK → iMX6");
	sotp_send_frame(SOTP_TYPE_DELETE_ACK, NULL, 0U);
}

int uart_relay_send_config_read(uint8_t file_id, uint8_t field_id)
{
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	uint8_t payload[2] = { file_id, field_id };
	LOG_INF("config_read_req: file=0x%02x field=0x%02x → iMX6",
		file_id, field_id);
	sotp_send_frame(SOTP_TYPE_CONFIG_READ_REQ, payload, 2U);
	return 0;
}

int uart_relay_send_config_write(uint8_t file_id, uint8_t field_id,
				 const void *value, uint16_t value_len)
{
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	uint8_t payload[2 + 248];
	payload[0] = file_id;
	payload[1] = field_id;

	uint16_t vlen = (value_len > 248) ? 248 : value_len;
	memcpy(payload + 2, value, vlen);

	LOG_INF("config_write: file=0x%02x field=0x%02x vlen=%u → iMX6",
		file_id, field_id, vlen);
	sotp_send_frame(SOTP_TYPE_CONFIG_WRITE, payload, (uint32_t)(2 + vlen));
	return 0;
}

int uart_relay_send_config_commit(uint8_t file_id)
{
	if (!device_is_ready(uart_dev)) {
		return -ENODEV;
	}

	uint8_t payload[1] = { file_id };
	LOG_INF("config_commit: file=0x%02x → iMX6", file_id);
	sotp_send_frame(SOTP_TYPE_CONFIG_COMMIT, payload, 1U);
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
		 * Format payload : [NAME_LEN uint8][NAME][DATA]
		 * On l'ajoute dans le pool OTS — un client connecté pourra le lire.
		 */
		LOG_INF("sotp_dispatch: OBJ_TO_BLE len=%u → OTS pool", len);
		{
			char name[CONFIG_BT_OTS_OBJ_MAX_NAME_LEN + 1];
			const uint8_t *obj_data = payload;
			uint32_t obj_len = len;

			if (len >= 1) {
				uint8_t name_len = payload[0];

				if (name_len > 0 && name_len <= CONFIG_BT_OTS_OBJ_MAX_NAME_LEN
				    && len >= (uint32_t)(1 + name_len)) {
					memcpy(name, payload + 1, name_len);
					name[name_len] = '\0';
					obj_data = payload + 1 + name_len;
					obj_len = len - 1 - name_len;
				} else {
					static uint32_t rx_obj_counter;
					snprintf(name, sizeof(name), "imx6_obj_%u",
						 rx_obj_counter++);
				}
			}

			int err = ots_handler_add_object_from_uart(
					obj_data, (size_t)obj_len, name);
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

	/* Config service responses from iMX6 */
	case SOTP_TYPE_CONFIG_READ_RSP:
	case SOTP_TYPE_CONFIG_WRITE_RSP:
	case SOTP_TYPE_CONFIG_COMMIT_RSP:
		LOG_DBG("sotp_dispatch: config RSP type=0x%02x len=%u",
			type, len);
		config_service_handle_response(type, payload, len);
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
 * RX UART — mode interrupt-driven avec ring buffer
 *
 * Le mode polling (uart_poll_in + k_sleep) perdait des octets à 115200 baud :
 * le nRF52840 UARTE en mode poll ne bufferise qu'un seul octet (EVENTS_RXDRDY).
 * Si le thread sotp_rx ne lit pas avant l'arrivée du suivant → perte.
 *
 * Le mode interrupt-driven utilise uart_irq_callback_set() :
 *   - L'ISR UART lit tous les octets disponibles dans un ring buffer
 *   - Le thread sotp_rx consomme le ring buffer et alimente la state machine
 *   - Aucun octet perdu tant que le ring buffer n'est pas plein
 *
 * Ring buffer dimensionné à 1024 octets :
 *   Trame SOTP max = 7 (header) + ~32 KB (payload) + 2 (CRC) = ~32 KB
 *   Mais le thread consomme en continu → 1024 suffit largement pour
 *   absorber les bursts à 115200 baud (~14 octets/ms).
 * ------------------------------------------------------------------------- */

#define SOTP_RX_RING_SIZE 1024

static uint8_t rx_ring_buf[SOTP_RX_RING_SIZE];
static volatile uint32_t rx_ring_head; /* écrit par ISR */
static volatile uint32_t rx_ring_tail; /* lu par thread */

/**
 * @brief ISR callback UART — lit tous les octets disponibles dans le ring buffer.
 *
 * Appelé en contexte ISR par le driver UART nRF quand des données sont disponibles.
 * On lit en boucle avec uart_fifo_read() pour vider le FIFO hardware.
 */
static void uart_irq_rx_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	uint8_t tmp[32];
	int len;

	while ((len = uart_fifo_read(dev, tmp, sizeof(tmp))) > 0) {
		for (int i = 0; i < len; i++) {
			uint32_t next = (rx_ring_head + 1) % SOTP_RX_RING_SIZE;
			if (next == rx_ring_tail) {
				/* Ring buffer plein — octet perdu */
				atomic_inc(&stat_rx_errors);
				continue;
			}
			rx_ring_buf[rx_ring_head] = tmp[i];
			rx_ring_head = next;
		}
	}
}

#define SOTP_RX_STACK_SIZE 1024

static K_THREAD_STACK_DEFINE(sotp_rx_stack, SOTP_RX_STACK_SIZE);
static struct k_thread sotp_rx_thread_data;

/**
 * @brief Thread sotp_rx — consomme le ring buffer et alimente la state machine.
 *
 * Attend un sémaphore quand le ring buffer est vide → pas de busy-wait.
 * L'ISR poste le sémaphore à chaque réception.
 */
static K_SEM_DEFINE(rx_data_sem, 0, 1);

static void sotp_rx_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	LOG_INF("sotp_rx thread started (IRQ-driven)");

	while (1) {
		/* Consommer tout ce qui est dans le ring buffer */
		while (rx_ring_tail != rx_ring_head) {
			uint8_t byte = rx_ring_buf[rx_ring_tail];
			rx_ring_tail = (rx_ring_tail + 1) % SOTP_RX_RING_SIZE;
			sotp_rx_byte(byte);
		}

		/* Attendre que l'ISR signale de nouvelles données */
		k_sleep(K_MSEC(1));
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

	/* Configurer le mode interrupt-driven RX :
	 * L'ISR uart_irq_rx_handler lit les octets dans le ring buffer.
	 * Le thread sotp_rx consomme le ring buffer. */
	uart_irq_callback_set(uart_dev, uart_irq_rx_handler);
	uart_irq_rx_enable(uart_dev);

	/* Lancer le thread de consommation SOTP */
	k_thread_create(&sotp_rx_thread_data,
			sotp_rx_stack,
			K_THREAD_STACK_SIZEOF(sotp_rx_stack),
			sotp_rx_thread_fn,
			NULL, NULL, NULL,
			K_PRIO_PREEMPT(8),
			0,
			K_NO_WAIT);
	k_thread_name_set(&sotp_rx_thread_data, "sotp_rx");

	LOG_INF("uart_relay_init: UART '%s' ready, IRQ RX + sotp_rx thread started",
		uart_dev->name);

	return 0;
}
