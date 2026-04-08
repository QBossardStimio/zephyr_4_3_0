/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		printk("RX: 0x%02x\n", c);
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		} else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

int main(void)
{
	char tx_buf[MSG_SIZE];

	printk("echo_bot main() started\n");

	if (!device_is_ready(uart_dev)) {
		printk("UART device not ready!\n");
		return 0;
	}

	printk("UART device ready, starting echo\n");

	struct uart_config cfg;
	if (uart_config_get(uart_dev, &cfg) == 0) {
		printk("UART config:\n");
		printk("  baudrate:  %d\n", cfg.baudrate);
		printk("  parity:    %d\n", cfg.parity);
		printk("  stop_bits: %d\n", cfg.stop_bits);
		printk("  data_bits: %d\n", cfg.data_bits);
		printk("  flow_ctrl: %d\n", cfg.flow_ctrl);
	} else {
		printk("uart_config_get failed\n");
	}

	/* configure interrupt and callback to receive data */
	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}
	uart_irq_rx_enable(uart_dev);

	/*
	 * Debug : lire les deux pins de flow control du nRF52840
	 *
	 * D'après le schéma RailNet200 (STO06 sheet 5 + 19) :
	 *   ball 55 (ENET2_TX_CLK) = BLE_UART_CTS → NINA pin 20 = P0.31
	 *   ball 58 (ENET2_RX_ER)  = BLE_UART_RTS → NINA pin 21 = P1.12
	 *
	 * GPIO P0 IN register = 0x50000510, P0.31 = bit 31
	 * GPIO P1 IN register = 0x50000810, P1.12 = bit 12
	 */
	volatile uint32_t *gpio0_in = (volatile uint32_t *)0x50000510;
	volatile uint32_t *gpio1_in = (volatile uint32_t *)0x50000810;

	while (1) {
		uint32_t p0 = *gpio0_in;
		uint32_t p1 = *gpio1_in;
		uint32_t pin_p0_31 = (p0 >> 31) & 1;  /* ball 55 = BLE_UART_CTS */
		uint32_t pin_p1_12 = (p1 >> 12) & 1;  /* ball 58 = BLE_UART_RTS */

		printk("P0.31(ball55/CTS)=%u  P1.12(ball58/RTS)=%u\n",
		       pin_p0_31, pin_p1_12);

		/* Envoyer un test byte si au moins un signal est LOW */
		if (pin_p0_31 == 0 || pin_p1_12 == 0) {
			printk("  -> Signal LOW! Sending 0xAA...\n");
			uart_poll_out(uart_dev, 0xAA);
			printk("  -> Sent OK\n");
		}

		k_sleep(K_MSEC(1000));
	}
	return 0;
}
