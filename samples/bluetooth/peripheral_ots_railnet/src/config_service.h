/*
 * config_service.h
 *
 * BLE GATT configuration service for RailNet200.
 *
 * Exposes iMX6 config file fields as individual GATT characteristics.
 * The NINA acts as a transparent relay — all JSON logic is on the iMX6.
 *
 * Each GATT read/write triggers a synchronous SOTP exchange with iMX6
 * (CONFIG_READ_REQ / CONFIG_WRITE / CONFIG_COMMIT).
 */

#ifndef CONFIG_SERVICE_H_
#define CONFIG_SERVICE_H_

#include <stdint.h>

/**
 * @brief Initialize the GATT config service.
 *
 * The GATT service is declared statically via BT_GATT_SERVICE_DEFINE,
 * so this function only initializes internal state (semaphores, buffers).
 *
 * Must be called from bt_ready_cb(), after bt_enable().
 *
 * @return 0 on success, negative error code on failure.
 */
int config_service_init(void);

/**
 * @brief Handle a CONFIG_*_RSP received from iMX6 via SOTP.
 *
 * Called by uart_relay.c::sotp_dispatch() when a response frame arrives.
 * Copies the payload into the shared response buffer and signals the
 * waiting GATT callback via semaphore.
 *
 * @param type     SOTP type (CONFIG_READ_RSP, CONFIG_WRITE_RSP, CONFIG_COMMIT_RSP)
 * @param payload  Response payload
 * @param len      Payload length
 */
void config_service_handle_response(uint8_t type, const uint8_t *payload,
				    uint32_t len);

#endif /* CONFIG_SERVICE_H_ */
