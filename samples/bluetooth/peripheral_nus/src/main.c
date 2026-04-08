/*
 * Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(peripheral_nus, LOG_LEVEL_INF);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

/* UART bridge vers iMX6 */
#define UART_DEVICE_NODE DT_NODELABEL(uart0)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* Buffer RX UART — taille MTU NUS max */
#define UART_RX_BUF_SIZE 256
static uint8_t uart_rx_buf[UART_RX_BUF_SIZE];

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
};

/* NUS RX (Android → NINA) : relaye vers UART iMX6 */
static void nus_received(struct bt_conn *conn, const void *data, uint16_t len, void *ctx)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(ctx);

	LOG_INF("NUS RX %d bytes → UART", len);

	const uint8_t *buf = data;
	for (uint16_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

static void notif_enabled(bool enabled, void *ctx)
{
	ARG_UNUSED(ctx);
	LOG_INF("NUS notify %s", enabled ? "enabled" : "disabled");
}

struct bt_nus_cb nus_listener = {
	.notif_enabled = notif_enabled,
	.received = nus_received,
};

int main(void)
{
	int err;

	printk("RailNet200 NUS Bridge\n");

	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}
	LOG_INF("UART bridge ready");

	/* BD Address statique : DE:CA:F1:5B:AD:00 ("DECAF15BAD") */
	static const bt_addr_t bd_addr = { { 0x00, 0xAD, 0x5B, 0xF1, 0xCA, 0xDE } };
	bt_ctlr_set_public_addr(bd_addr.val);
	LOG_INF("BD Address: DE:CA:F1:5B:AD:00");

	err = bt_nus_cb_register(&nus_listener, NULL);
	if (err) {
		LOG_ERR("Failed to register NUS callback: %d", err);
		return err;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Failed to enable bluetooth: %d", err);
		return err;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Failed to start advertising: %d", err);
		return err;
	}

	LOG_INF("Advertising as \"%s\"", DEVICE_NAME);

	/* Boucle principale : UART RX (iMX6) → NUS TX (Android)
	 *
	 * Le driver ttymxc (iMX6) écrit par chunks de 7 bytes max par appel
	 * write(). On accumule plusieurs chunks consécutifs avant d'envoyer
	 * via NUS. Stratégie : lire tant qu'il y a des données, avec une
	 * fenêtre d'accumulation de 50ms entre les bursts.
	 */
	while (true) {
		uint8_t c;

		/* Attente du premier octet */
		if (uart_poll_in(uart_dev, &c) < 0) {
			k_sleep(K_MSEC(5));
			continue;
		}

		uart_rx_buf[0] = c;
		int rx_len = 1;

		/* Accumulation : fenêtre de 50ms pour recevoir tous les chunks */
		int64_t deadline = k_uptime_get() + 50;
		while (k_uptime_get() < deadline && rx_len < UART_RX_BUF_SIZE) {
			if (uart_poll_in(uart_dev, &c) == 0) {
				uart_rx_buf[rx_len++] = c;
				/* Reset deadline à chaque nouvel octet */
				deadline = k_uptime_get() + 50;
			} else {
				k_sleep(K_MSEC(1));
			}
		}

		LOG_INF("UART RX %d bytes → NUS", rx_len);

		/* Envoie par chunks de 20 bytes (MTU BLE minimal garanti) */
		int offset = 0;
		while (offset < rx_len) {
			int chunk = MIN(20, rx_len - offset);
			err = bt_nus_send(NULL, uart_rx_buf + offset, chunk);
			if (err < 0 && err != -EAGAIN && err != -ENOTCONN) {
				LOG_WRN("NUS send error: %d", err);
				break;
			}
			offset += chunk;
			k_sleep(K_MSEC(10));
		}
	}

	return 0;
}
