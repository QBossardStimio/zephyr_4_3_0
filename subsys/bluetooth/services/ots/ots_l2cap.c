/*
 * Copyright (c) 2020 - 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/net_buf.h>

#include "ots_l2cap_internal.h"

#include <zephyr/logging/log.h>

/* This l2cap is the only OTS-file in use for OTC.
 * If only OTC is used, the OTS log module must be registered here.
 */
#if defined(CONFIG_BT_OTS)
LOG_MODULE_DECLARE(bt_ots, CONFIG_BT_OTS_LOG_LEVEL);
#elif defined(CONFIG_BT_OTS_CLIENT)
LOG_MODULE_REGISTER(bt_ots, CONFIG_BT_OTS_CLIENT_LOG_LEVEL);
#endif

/* According to Bluetooth specification Assigned Numbers that are used in the
 * Logical Link Control for protocol/service multiplexers.
 */
#define BT_GATT_OTS_L2CAP_PSM	0x0025

NET_BUF_POOL_FIXED_DEFINE(ot_chan_tx_pool, 1,
			  BT_L2CAP_SDU_BUF_SIZE(CONFIG_BT_OTS_L2CAP_CHAN_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

/* Avec CONFIG_BT_L2CAP_SEG_RECV, les segments sont reçus directement depuis
 * le pool ACL global — pas besoin d'un pool RX dédié pour l'OTS.
 */
#if !defined(CONFIG_BT_L2CAP_SEG_RECV)
#if (CONFIG_BT_OTS_L2CAP_CHAN_RX_MTU > BT_L2CAP_SDU_RX_MTU)
NET_BUF_POOL_FIXED_DEFINE(ot_chan_rx_pool, 1,
			  CONFIG_BT_OTS_L2CAP_CHAN_RX_MTU, 8, NULL);
#endif
#endif /* !CONFIG_BT_L2CAP_SEG_RECV */

/* List of Object Transfer Channels. */
static sys_slist_t channels;

static int ots_l2cap_send(struct bt_gatt_ots_l2cap *l2cap_ctx)
{
	int ret;
	struct net_buf *buf;
	uint32_t len;

	/* Calculate maximum length of data chunk. */
	len = MIN(l2cap_ctx->ot_chan.tx.mtu, CONFIG_BT_OTS_L2CAP_CHAN_TX_MTU);
	len = MIN(len, l2cap_ctx->tx.len - l2cap_ctx->tx.len_sent);

	/* Prepare buffer for sending. */
	buf = net_buf_alloc(&ot_chan_tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_L2CAP_SDU_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, &l2cap_ctx->tx.data[l2cap_ctx->tx.len_sent], len);

	ret = bt_l2cap_chan_send(&l2cap_ctx->ot_chan.chan, buf);
	if (ret < 0) {
		LOG_ERR("Unable to send data over CoC: %d", ret);
		net_buf_unref(buf);

		return -ENOEXEC;
	}

	/* Mark that L2CAP TX was accepted. */
	l2cap_ctx->tx.len_sent += len;

	LOG_DBG("Sending TX chunk with %d bytes on L2CAP CoC", len);

	return 0;
}

#if !defined(CONFIG_BT_L2CAP_SEG_RECV)
#if (CONFIG_BT_OTS_L2CAP_CHAN_RX_MTU > BT_L2CAP_SDU_RX_MTU)
static struct net_buf *l2cap_alloc_buf(struct bt_l2cap_chan *chan)
{
	LOG_DBG("Channel %p allocating buffer", chan);

	return net_buf_alloc(&ot_chan_rx_pool, K_FOREVER);
}
#endif
#endif /* !CONFIG_BT_L2CAP_SEG_RECV */

#if defined(CONFIG_BT_L2CAP_SEG_RECV)
/* Appelé par Zephyr L2CAP pour chaque segment PDU reçu.
 * Pas de buffer complet en RAM — les données sont passées directement
 * au callback seg_rx_done (→ oacp_write_proc_cb par segments).
 */
static void l2cap_seg_recv(struct bt_l2cap_chan *chan, size_t sdu_len,
			   off_t seg_offset, struct net_buf_simple *seg)
{
	struct bt_l2cap_le_chan *l2chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);
	struct bt_gatt_ots_l2cap *l2cap_ctx;

	l2cap_ctx = CONTAINER_OF(l2chan, struct bt_gatt_ots_l2cap, ot_chan);

	LOG_DBG("seg_recv chan %p sdu_len %zu offset %ld seg_len %u",
		chan, sdu_len, (long)seg_offset, seg->len);

	if (!l2cap_ctx->seg_rx_done) {
		return;
	}

	l2cap_ctx->seg_rx_done(l2cap_ctx, chan->conn, sdu_len, seg_offset, seg);

	/* Réémettre un crédit pour chaque PDU traité.
	 * Sans cela le transfert s'arrête après les crédits initiaux.
	 */
	int err = bt_l2cap_chan_give_credits(chan, 1);

	if (err) {
		LOG_WRN("Failed to give L2CAP credit: %d", err);
	}
}
#endif /* CONFIG_BT_L2CAP_SEG_RECV */


static void l2cap_sent(struct bt_l2cap_chan *chan)
{
	struct bt_l2cap_le_chan *l2chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);
	struct bt_gatt_ots_l2cap *l2cap_ctx;

	LOG_DBG("Outgoing data channel %p transmitted", chan);

	l2cap_ctx = CONTAINER_OF(l2chan, struct bt_gatt_ots_l2cap, ot_chan);

	/* Ongoing TX - sending next chunk. */
	if (l2cap_ctx->tx.len != l2cap_ctx->tx.len_sent) {
		ots_l2cap_send(l2cap_ctx);

		return;
	}

	/* TX completed - notify upper layers and clean up. */
	memset(&l2cap_ctx->tx, 0, sizeof(l2cap_ctx->tx));

	LOG_DBG("Scheduled TX on L2CAP CoC is complete");

	if (l2cap_ctx->tx_done) {
		l2cap_ctx->tx_done(l2cap_ctx, chan->conn);
	}
}

static int l2cap_recv(struct bt_l2cap_chan *chan, struct net_buf *buf)
{
	struct bt_l2cap_le_chan *l2chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);
	struct bt_gatt_ots_l2cap *l2cap_ctx;

	LOG_DBG("Incoming data channel %p received", chan);

	l2cap_ctx = CONTAINER_OF(l2chan, struct bt_gatt_ots_l2cap, ot_chan);

	if (!l2cap_ctx->rx_done) {
		return -ENODEV;
	}

	return l2cap_ctx->rx_done(l2cap_ctx, chan->conn, buf);
}

static void l2cap_status(struct bt_l2cap_chan *chan, atomic_t *status)
{
	LOG_DBG("Channel %p status %lu", chan, atomic_get(status));
}

static void l2cap_connected(struct bt_l2cap_chan *chan)
{
	LOG_DBG("Channel %p connected", chan);

#if defined(CONFIG_BT_L2CAP_SEG_RECV)
	/* Accorder des crédits initiaux supplémentaires pour permettre à Ubuntu
	 * d'envoyer plusieurs PDUs en rafale sans attendre un crédit à la fois.
	 * bt_l2cap_chan_give_credits n'est disponible qu'avec CONFIG_BT_L2CAP_SEG_RECV.
	 */
	int err = bt_l2cap_chan_give_credits(chan, CONFIG_BT_BUF_ACL_RX_COUNT_EXTRA);
	if (err) {
		LOG_WRN("Failed to give L2CAP credits: %d", err);
	} else {
		LOG_DBG("Gave %d extra L2CAP RX credits", CONFIG_BT_BUF_ACL_RX_COUNT_EXTRA);
	}
#endif
}

static void l2cap_disconnected(struct bt_l2cap_chan *chan)
{
	struct bt_l2cap_le_chan *l2chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);
	struct bt_gatt_ots_l2cap *l2cap_ctx;

	LOG_DBG("Channel %p disconnected", chan);

	l2cap_ctx = CONTAINER_OF(l2chan, struct bt_gatt_ots_l2cap, ot_chan);

	if (l2cap_ctx->closed) {
		l2cap_ctx->closed(l2cap_ctx, chan->conn);
	}
}

static const struct bt_l2cap_chan_ops l2cap_ops = {
#if defined(CONFIG_BT_L2CAP_SEG_RECV)
	/* Avec SEG_RECV : pas d'alloc_buf, pas de recv — seulement seg_recv. */
	.seg_recv	= l2cap_seg_recv,
#else
#if (CONFIG_BT_OTS_L2CAP_CHAN_RX_MTU > BT_L2CAP_SDU_RX_MTU)
	.alloc_buf	= l2cap_alloc_buf,
#endif
	.recv		= l2cap_recv,
#endif /* CONFIG_BT_L2CAP_SEG_RECV */
	.sent		= l2cap_sent,
	.status		= l2cap_status,
	.connected	= l2cap_connected,
	.disconnected	= l2cap_disconnected,
};

static inline void l2cap_chan_init(struct bt_l2cap_le_chan *chan)
{
	chan->rx.mtu = CONFIG_BT_OTS_L2CAP_CHAN_RX_MTU;
	chan->chan.ops = &l2cap_ops;

#if defined(CONFIG_BT_L2CAP_SEG_RECV)
	/* rx.mps = DLE_max(251) - L2CAP_HDR(4) - SDU_LEN_HDR(2) = 245.
	 *
	 * Structure d'un PDU L2CAP CoC complet :
	 *   [L2CAP_HDR 4B][SDU_LEN 2B (premier PDU seulement)][data rx.mps B]
	 *
	 * Le SDU_LEN (2 bytes) est présent UNIQUEMENT dans le premier PDU du SDU.
	 * Pour tenir dans un seul LL PDU DLE (251 bytes max) :
	 *   4 + 2 + rx.mps ≤ 251  →  rx.mps ≤ 245.
	 *
	 * ATTENTION : Zephyr l2cap.c écrase rx.mps si rx.mps > BT_L2CAP_RX_MTU
	 * (= CONFIG_BT_BUF_ACL_RX_SIZE - 4). Il faut donc que BT_L2CAP_RX_MTU ≥ 245,
	 * soit ACL_RX_SIZE ≥ 249. Avec CONFIG_BT_BUF_ACL_RX_SIZE=251 :
	 * BT_L2CAP_RX_MTU=247 ≥ 245 → rx.mps=245 conservé.
	 *
	 * Règle : rx.mps = CONFIG_BT_CTLR_DATA_LENGTH_MAX - BT_L2CAP_HDR_SIZE
	 *                  - BT_L2CAP_SDU_HDR_SIZE
	 *               = 251 - 4 - 2 = 245.
	 */
	chan->rx.mps = CONFIG_BT_CTLR_DATA_LENGTH_MAX - BT_L2CAP_HDR_SIZE
		       - BT_L2CAP_SDU_HDR_SIZE;
#endif

	LOG_DBG("RX MTU set to %u, MPS set to %u", chan->rx.mtu, chan->rx.mps);
}

static struct bt_gatt_ots_l2cap *find_free_l2cap_ctx(void)
{
	struct bt_gatt_ots_l2cap *l2cap_ctx;

	SYS_SLIST_FOR_EACH_CONTAINER(&channels, l2cap_ctx, node) {
		if (l2cap_ctx->ot_chan.chan.conn) {
			continue;
		}

		return l2cap_ctx;
	}

	return NULL;
}

static int l2cap_accept(struct bt_conn *conn, struct bt_l2cap_server *server,
			struct bt_l2cap_chan **chan)
{
	struct bt_gatt_ots_l2cap *l2cap_ctx;

	LOG_DBG("Incoming conn %p", (void *)conn);

	l2cap_ctx = find_free_l2cap_ctx();
	if (l2cap_ctx) {
		l2cap_chan_init(&l2cap_ctx->ot_chan);
		memset(&l2cap_ctx->tx, 0, sizeof(l2cap_ctx->tx));

		*chan = &l2cap_ctx->ot_chan.chan;

		return 0;
	}

	return -ENOMEM;
}

static struct bt_l2cap_server l2cap_server = {
	.psm = BT_GATT_OTS_L2CAP_PSM,
	.accept	= l2cap_accept,
};

static int bt_gatt_ots_l2cap_init(void)
{
	int err;

	sys_slist_init(&channels);

	err = bt_l2cap_server_register(&l2cap_server);
	if (err) {
		LOG_ERR("Unable to register OTS PSM");
		return err;
	}

	LOG_DBG("Initialized OTS L2CAP");

	return 0;
}

bool bt_gatt_ots_l2cap_is_open(struct bt_gatt_ots_l2cap *l2cap_ctx,
				   struct bt_conn *conn)
{
	return (l2cap_ctx->ot_chan.chan.conn == conn);
}

int bt_gatt_ots_l2cap_send(struct bt_gatt_ots_l2cap *l2cap_ctx,
			       uint8_t *data, uint32_t len)
{
	int err;

	if (l2cap_ctx->tx.len != 0) {
		LOG_ERR("L2CAP TX in progress");

		return -EAGAIN;
	}

	l2cap_ctx->tx.data = data;
	l2cap_ctx->tx.len = len;

	LOG_DBG("Starting TX on L2CAP CoC with %d byte packet", len);

	err = ots_l2cap_send(l2cap_ctx);
	if (err) {
		LOG_ERR("Unable to send data over CoC: %d", err);

		return err;
	}

	return 0;
}

int bt_gatt_ots_l2cap_register(struct bt_gatt_ots_l2cap *l2cap_ctx)
{
	sys_slist_append(&channels, &l2cap_ctx->node);

	return 0;
}

int bt_gatt_ots_l2cap_unregister(struct bt_gatt_ots_l2cap *l2cap_ctx)
{
	sys_slist_find_and_remove(&channels, &l2cap_ctx->node);

	return 0;
}

/* Similar to l2cap_accept(), but for the client side */
int bt_gatt_ots_l2cap_connect(struct bt_conn *conn,
			      struct bt_gatt_ots_l2cap **l2cap_ctx)
{
	int err;
	struct bt_gatt_ots_l2cap *ctx;

	if (!conn) {
		LOG_WRN("Invalid Connection");
		return -ENOTCONN;
	}

	if (!l2cap_ctx) {
		LOG_WRN("Invalid context");
		return -EINVAL;
	}

	*l2cap_ctx = NULL;

	ctx = find_free_l2cap_ctx();
	if (!ctx) {
		return -ENOMEM;
	}

	l2cap_chan_init(&ctx->ot_chan);
	(void)memset(&ctx->tx, 0, sizeof(ctx->tx));

	LOG_DBG("Connecting L2CAP CoC");
	err = bt_l2cap_chan_connect(conn, &ctx->ot_chan.chan, BT_GATT_OTS_L2CAP_PSM);
	if (err) {
		LOG_WRN("Unable to connect to psm %u (err %d)", BT_GATT_OTS_L2CAP_PSM, err);
	} else {
		LOG_DBG("L2CAP connection pending");
		*l2cap_ctx = ctx;
	}

	return err;
}

int bt_gatt_ots_l2cap_disconnect(struct bt_gatt_ots_l2cap *l2cap_ctx)
{
	return bt_l2cap_chan_disconnect(&l2cap_ctx->ot_chan.chan);
}

SYS_INIT(bt_gatt_ots_l2cap_init, APPLICATION,
	 CONFIG_APPLICATION_INIT_PRIORITY);
