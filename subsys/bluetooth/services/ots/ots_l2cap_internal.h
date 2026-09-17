/*
 * Copyright (c) 2020 - 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_GATT_OTS_L2CAP_H_
#define BT_GATT_OTS_L2CAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <zephyr/sys/slist.h>
#include <sys/types.h>

#include <zephyr/bluetooth/l2cap.h>

struct bt_gatt_ots_l2cap_tx {
	uint8_t *data;
	uint32_t len;
	uint32_t len_sent;
};

struct bt_gatt_ots_l2cap {
	sys_snode_t node;
	struct bt_l2cap_le_chan ot_chan;
	struct bt_gatt_ots_l2cap_tx tx;
	void (*tx_done)(struct bt_gatt_ots_l2cap *l2cap_ctx,
			struct bt_conn *conn);
	ssize_t (*rx_done)(struct bt_gatt_ots_l2cap *l2cap_ctx,
			   struct bt_conn *conn, struct net_buf *buf);
#if defined(CONFIG_BT_L2CAP_SEG_RECV)
	/* Appelé pour chaque segment reçu (avec CONFIG_BT_L2CAP_SEG_RECV).
	 * seg_offset : offset de ce segment dans le SDU complet.
	 * sdu_len    : taille totale du SDU (= taille de l'objet à écrire).
	 */
	ssize_t (*seg_rx_done)(struct bt_gatt_ots_l2cap *l2cap_ctx,
			       struct bt_conn *conn, size_t sdu_len,
			       off_t seg_offset, struct net_buf_simple *seg);
#endif
	void (*closed)(struct bt_gatt_ots_l2cap *l2cap_ctx,
			struct bt_conn *conn);
};

bool bt_gatt_ots_l2cap_is_open(struct bt_gatt_ots_l2cap *l2cap_ctx,
				   struct bt_conn *conn);

int bt_gatt_ots_l2cap_send(struct bt_gatt_ots_l2cap *l2cap_ctx,
			       uint8_t *data,
			       uint32_t len);

int bt_gatt_ots_l2cap_register(struct bt_gatt_ots_l2cap *l2cap_ctx);

int bt_gatt_ots_l2cap_unregister(struct bt_gatt_ots_l2cap *l2cap_ctx);

/** @brief Connect OTS L2CAP channel
 *
 *  This function is for the OTS client to make an L2CAP connection to
 *  the OTS server.  One of the available registered L2CAP contexts
 *  will be used for the connection.
 *
 * @param[in]  conn       Connection pointer
 * @param[out] l2cap_ctx  The context that was connected
 *
 * @return     0 in case of success or negative value in case of error
 */
int bt_gatt_ots_l2cap_connect(struct bt_conn *conn,
			      struct bt_gatt_ots_l2cap **l2cap_ctx);

int bt_gatt_ots_l2cap_disconnect(struct bt_gatt_ots_l2cap *l2cap_ctx);


#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_GATT_OTS_L2CAP_H_ */
