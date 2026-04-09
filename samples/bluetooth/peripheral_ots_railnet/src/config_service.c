/*
 * config_service.c
 *
 * GATT configuration service — transparent SOTP relay.
 * One characteristic per config field, all UTF-8 strings.
 */

#include "config_service.h"
#include "uart_relay.h"

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

#include <string.h>

LOG_MODULE_REGISTER(config_service, LOG_LEVEL_INF);

/* -------------------------------------------------------------------------
 * UUIDs — 128-bit custom STIMIO UUIDs
 *
 * Base:  5354494D-xxxx-5354-494D-535449494F44
 *        S T I M        S T   I M   S T I I O D
 *
 * Service:       xxxx = 0001
 * Control Point: xxxx = 0002
 * Status:        xxxx = 0003
 * Fields:        xxxx = 01XX (XX = field_id)
 * ------------------------------------------------------------------------- */

#define STIM_UUID_INIT(val) BT_UUID_INIT_128( \
	0x44, 0x4F, 0x49, 0x49, 0x54, 0x53, \
	0x4D, 0x49, \
	0x54, 0x53, \
	0x4D, 0x49, \
	0x54, 0x53, \
	(uint8_t)((val) & 0xFF), (uint8_t)(((val) >> 8) & 0xFF))

/* Service UUID */
static struct bt_uuid_128 svc_uuid = STIM_UUID_INIT(0x0001);

/* Control Point and Status UUIDs */
static struct bt_uuid_128 cp_uuid     = STIM_UUID_INIT(0x0002);
static struct bt_uuid_128 status_uuid = STIM_UUID_INIT(0x0003);

/* Field UUIDs: 0x0100 + field_id */
static struct bt_uuid_128 field_uuids[] = {
	STIM_UUID_INIT(0x0100), /* system.host */
	STIM_UUID_INIT(0x0101), /* system.port */
	STIM_UUID_INIT(0x0102), /* system.topic_prefix */
	STIM_UUID_INIT(0x0103), /* system.dns_primary */
	STIM_UUID_INIT(0x0104), /* system.dns_secondary */
	STIM_UUID_INIT(0x0105), /* system.enable_tls */
	STIM_UUID_INIT(0x0106), /* system.cert_reqs */
	STIM_UUID_INIT(0x0107), /* system.ca_certs */
	STIM_UUID_INIT(0x0108), /* system.certfile */
	STIM_UUID_INIT(0x0109), /* system.keyfile */
	STIM_UUID_INIT(0x010A), /* system.apn */
	STIM_UUID_INIT(0x010B), /* system.ntp_server */
	STIM_UUID_INIT(0x010C), /* system.ntp_port */
	STIM_UUID_INIT(0x010D), /* system.syslog_ip */
	STIM_UUID_INIT(0x010E), /* system.syslog_port */
	STIM_UUID_INIT(0x010F), /* system.can_bitrate */
	STIM_UUID_INIT(0x0110), /* buffer.storage_path */
	STIM_UUID_INIT(0x0111), /* buffer.max_batch_size */
};

#define CONFIG_FILE_ID   0x00   /* /etc/stimio/config.json */
#define FIELD_COUNT      18
#define MAX_VALUE_LEN    248    /* ATT_MTU(251) - 3 */
#define SOTP_TIMEOUT_MS  500

/* -------------------------------------------------------------------------
 * Shared response state (single semaphore + buffer)
 *
 * Safe because:
 * - Only one BLE connection at a time (peripheral)
 * - GATT operations are serialized by Zephyr BLE stack
 * ------------------------------------------------------------------------- */
static K_SEM_DEFINE(rsp_sem, 0, 1);

static struct {
	uint8_t  data[2 + MAX_VALUE_LEN];
	uint32_t len;
	uint8_t  type;
} rsp_buf;

/* -------------------------------------------------------------------------
 * GATT read callback — triggered for each field characteristic
 * ------------------------------------------------------------------------- */

static uint8_t field_id_from_uuid(const struct bt_uuid *uuid)
{
	const struct bt_uuid_128 *u128 = (const struct bt_uuid_128 *)uuid;
	/* Bytes 14-15 contain our value in LE */
	uint16_t val = (uint16_t)u128->val[14] | ((uint16_t)u128->val[15] << 8);
	return (uint8_t)(val - 0x0100);
}

static ssize_t field_read_cb(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	uint8_t fid = field_id_from_uuid(attr->uuid);

	LOG_INF("GATT read field_id=0x%02x", fid);

	/* Send CONFIG_READ_REQ to iMX6 */
	int ret = uart_relay_send_config_read(CONFIG_FILE_ID, fid);
	if (ret) {
		LOG_ERR("field_read_cb: send failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Wait for response from iMX6 */
	ret = k_sem_take(&rsp_sem, K_MSEC(SOTP_TIMEOUT_MS));
	if (ret) {
		LOG_ERR("field_read_cb: timeout waiting for iMX6 response");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Check response */
	if (rsp_buf.type != SOTP_TYPE_CONFIG_READ_RSP ||
	    rsp_buf.len < 2 ||
	    rsp_buf.data[0] != CONFIG_FILE_ID ||
	    rsp_buf.data[1] != fid) {
		LOG_ERR("field_read_cb: unexpected response type=0x%02x",
			rsp_buf.type);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* If only 3 bytes and 3rd byte is non-zero, it's an error status */
	if (rsp_buf.len == 3 && rsp_buf.data[2] != 0) {
		LOG_WRN("field_read_cb: iMX6 returned error 0x%02x",
			rsp_buf.data[2]);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Value is in rsp_buf.data[2..len-1] */
	const uint8_t *value = rsp_buf.data + 2;
	uint32_t vlen = rsp_buf.len - 2;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, vlen);
}

/* -------------------------------------------------------------------------
 * GATT write callback — triggered for each field characteristic
 * ------------------------------------------------------------------------- */
static ssize_t field_write_cb(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      const void *buf, uint16_t len,
			      uint16_t offset, uint8_t flags)
{
	uint8_t fid = field_id_from_uuid(attr->uuid);

	if (offset > 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len > MAX_VALUE_LEN) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	LOG_INF("GATT write field_id=0x%02x len=%u", fid, len);

	/* Send CONFIG_WRITE to iMX6 */
	int ret = uart_relay_send_config_write(CONFIG_FILE_ID, fid, buf, len);
	if (ret) {
		LOG_ERR("field_write_cb: send failed: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	/* Wait for CONFIG_WRITE_RSP */
	ret = k_sem_take(&rsp_sem, K_MSEC(SOTP_TIMEOUT_MS));
	if (ret) {
		LOG_ERR("field_write_cb: timeout waiting for iMX6 response");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	if (rsp_buf.type != SOTP_TYPE_CONFIG_WRITE_RSP ||
	    rsp_buf.len < 3 ||
	    rsp_buf.data[2] != 0) {
		LOG_ERR("field_write_cb: error from iMX6");
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

/* -------------------------------------------------------------------------
 * Control Point write callback — "commit" command
 * ------------------------------------------------------------------------- */

static uint8_t status_value[32];
static uint16_t status_len;

static ssize_t cp_write_cb(struct bt_conn *conn,
			   const struct bt_gatt_attr *attr,
			   const void *buf, uint16_t len,
			   uint16_t offset, uint8_t flags)
{
	if (offset > 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len == 6 && memcmp(buf, "commit", 6) == 0) {
		LOG_INF("Control Point: commit");

		int ret = uart_relay_send_config_commit(CONFIG_FILE_ID);
		if (ret) {
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}

		ret = k_sem_take(&rsp_sem, K_MSEC(SOTP_TIMEOUT_MS));
		if (ret) {
			LOG_ERR("cp_write_cb: timeout");
			return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
		}

		if (rsp_buf.type == SOTP_TYPE_CONFIG_COMMIT_RSP &&
		    rsp_buf.len >= 2 && rsp_buf.data[1] == 0) {
			memcpy(status_value, "OK", 2);
			status_len = 2;
		} else {
			memcpy(status_value, "ERR", 3);
			status_len = 3;
		}

		/* Notify status — best effort */
		bt_gatt_notify(conn, attr - 2, status_value, status_len);

		return len;
	}

	LOG_WRN("Control Point: unknown command (len=%u)", len);
	return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
}

/* -------------------------------------------------------------------------
 * Status characteristic read callback
 * ------------------------------------------------------------------------- */
static ssize_t status_read_cb(struct bt_conn *conn,
			      const struct bt_gatt_attr *attr,
			      void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 status_value, status_len);
}

/* -------------------------------------------------------------------------
 * GATT Service definition
 * ------------------------------------------------------------------------- */

#define FIELD_CHAR(idx) \
	BT_GATT_CHARACTERISTIC(&field_uuids[idx].uuid, \
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE, \
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, \
		field_read_cb, field_write_cb, NULL)

BT_GATT_SERVICE_DEFINE(config_svc,
	BT_GATT_PRIMARY_SERVICE(&svc_uuid),

	/* Control Point — write only */
	BT_GATT_CHARACTERISTIC(&cp_uuid.uuid,
		BT_GATT_CHRC_WRITE,
		BT_GATT_PERM_WRITE,
		NULL, cp_write_cb, NULL),

	/* Status — read + notify */
	BT_GATT_CHARACTERISTIC(&status_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		status_read_cb, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	/* 18 field characteristics */
	FIELD_CHAR(0x00),  /* system.host */
	FIELD_CHAR(0x01),  /* system.port */
	FIELD_CHAR(0x02),  /* system.topic_prefix */
	FIELD_CHAR(0x03),  /* system.dns_primary */
	FIELD_CHAR(0x04),  /* system.dns_secondary */
	FIELD_CHAR(0x05),  /* system.enable_tls */
	FIELD_CHAR(0x06),  /* system.cert_reqs */
	FIELD_CHAR(0x07),  /* system.ca_certs */
	FIELD_CHAR(0x08),  /* system.certfile */
	FIELD_CHAR(0x09),  /* system.keyfile */
	FIELD_CHAR(0x0A),  /* system.apn */
	FIELD_CHAR(0x0B),  /* system.ntp_server */
	FIELD_CHAR(0x0C),  /* system.ntp_port */
	FIELD_CHAR(0x0D),  /* system.syslog_ip */
	FIELD_CHAR(0x0E),  /* system.syslog_port */
	FIELD_CHAR(0x0F),  /* system.can_bitrate */
	FIELD_CHAR(0x10),  /* buffer.storage_path */
	FIELD_CHAR(0x11),  /* buffer.max_batch_size */
);

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

int config_service_init(void)
{
	memcpy(status_value, "INIT", 4);
	status_len = 4;

	LOG_INF("Config service initialized (%d fields)", FIELD_COUNT);
	return 0;
}

void config_service_handle_response(uint8_t type, const uint8_t *payload,
				    uint32_t len)
{
	if (len > sizeof(rsp_buf.data)) {
		len = sizeof(rsp_buf.data);
	}

	rsp_buf.type = type;
	memcpy(rsp_buf.data, payload, len);
	rsp_buf.len = len;

	k_sem_give(&rsp_sem);
}
