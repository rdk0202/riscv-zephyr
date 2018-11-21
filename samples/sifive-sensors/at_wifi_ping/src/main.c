/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <spi.h>
#include <gpio.h>
#include <misc/printk.h>
#include <string.h>

#define AT_SPI_BAUD		100000
#define AT_CSID			2
#define AT_READY_PIN	10

/* Create spi_buf_sets for the SPI interface */
#define AT_BUF_LEN	1024
u8_t tx_buf[AT_BUF_LEN] = {0};
u8_t rx_buf[AT_BUF_LEN] = {0};
struct spi_buf tx_spi_buf = {
	.buf = &tx_buf,
	.len = sizeof(tx_buf),
};
struct spi_buf rx_spi_buf = {
	.buf = &rx_buf,
	.len = sizeof(rx_buf),
};
struct spi_buf_set tx_spi_bufs = {
	.buffers = &tx_spi_buf,
	.count = 1,
};
struct spi_buf_set rx_spi_bufs = {
	.buffers = &rx_spi_buf,
	.count = 1,
};

/* Enum for send/receive flag in AT commands */
enum at_flag {
	AT_SEND = 2,
	AT_RECV = 1,
};

/* Semaphore to indicate when the AT interface is ready.
 *
 * The initial count is set to 1 because we assume that the AT
 * interface is ready at boot time. */
K_SEM_DEFINE(sem_at_ready, 1, 1);

/* AT Ready Pin Callback Handler */
static struct gpio_callback at_ready_cb;
void at_ready_callback(struct device *dev,
		struct gpio_callback *cb,
		u32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Signal that the AT interface is ready */
	k_sem_give(&sem_at_ready);
}

int at_sendflag(struct device *spi_dev,
		struct spi_config *config,
		enum at_flag flag)
{
	int rc = 0;

	tx_buf[0] = (u8_t) flag;
	tx_buf[1] = 0;
	tx_buf[2] = 0;
	tx_buf[3] = 0;
	tx_spi_buf.len = 4;
	rx_spi_buf.len = 4;

	/* Send flag */
	rc = spi_transceive(spi_dev, config, &tx_spi_bufs, &rx_spi_bufs);
	if(rc != 0) {
		printk("Sending command length failed\n");
		return rc;
	}

	if(rx_buf[0] != 'C') {
		printk("AT was not ready for flag\n");
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 0, rx_buf[0], rx_buf[0]);
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 1, rx_buf[1], rx_buf[1]);
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 2, rx_buf[2], rx_buf[2]);
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 3, rx_buf[3], rx_buf[3]);
		return 1;
	}

	return 0;
}

int at_send(struct device *spi_dev, struct spi_config *config, char * at_cmd) {
	int rc = 0;

	/* Send flag */
	k_sem_take(&sem_at_ready, K_FOREVER);
	rc = at_sendflag(spi_dev, config, AT_SEND);
	if(rc != 0) {
		printk("Sending flag failed\n");
		return rc;
	}

	/* Calculate command length */
	u16_t cmd_len = (u16_t) strlen(at_cmd);

	/* Create command length buffers */
	tx_buf[0] = cmd_len & 127;
	tx_buf[1] = cmd_len >> 7;
	tx_buf[2] = 0;
	tx_buf[3] = 'A';
	tx_spi_buf.len = 4;
	rx_spi_buf.len = 4;

	k_busy_wait(200);

	/* Send command length */
	k_sem_take(&sem_at_ready, K_FOREVER);
	rc = spi_transceive(spi_dev, config, &tx_spi_bufs, &rx_spi_bufs);
	if(rc != 0) {
		printk("Sending command length failed\n");
		return rc;
	}

	/* Check command length rx buffer */
	if(rx_buf[0] != 'b') {
		printk("AT was not ready for command length\n");
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 0, rx_buf[0], rx_buf[0]);
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 1, rx_buf[1], rx_buf[1]);
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 2, rx_buf[2], rx_buf[2]);
		printk("\trx_buf[%d] = '%c' 0x%02X\n", 3, rx_buf[3], rx_buf[3]);
		return 1;
	}

	/* Create command buffes */
	strncpy(tx_buf, at_cmd, cmd_len);
	tx_spi_buf.len = cmd_len;
	rx_spi_buf.len = AT_BUF_LEN;

	/* Send command */
	k_sem_take(&sem_at_ready, K_FOREVER);
	rc = spi_transceive(spi_dev, config, &tx_spi_bufs, &rx_spi_bufs);
	if(rc != 0) {
		printk("Sending command length failed\n");
		return rc;
	}

	/* The AT responds 'C' to the flag. If we see it here it means that the
	 * AT has reset to the top of the command flow and we've lost
	 * protocol synchrony */
	if(rx_buf[0] == 'C') {
		printk("AT was not ready for command\n");
		return 1;
	}

	return 0;
}

int at_recv_one(struct device *spi_dev, struct spi_config *config) {
	int rc = 0;

	/* Send flag */
	k_sem_take(&sem_at_ready, K_FOREVER);
	rc = at_sendflag(spi_dev, config, AT_RECV);
	if(rc != 0) {
		printk("Sending flag failed\n");
		return rc;
	}

	/* Receive command length */
	for(int i = 0; i < 4; i++)
		tx_buf[i] = 0;
	tx_spi_buf.len = 4;
	rx_spi_buf.len = 4;

	k_sem_take(&sem_at_ready, K_FOREVER);
	rc = spi_transceive(spi_dev, config, &tx_spi_bufs, &rx_spi_bufs);
	if(rc != 0) {
		printk("Receiving command length failed\n");
		return rc;
	}

	if(rx_buf[3] != 'B') {
		printk("AT length response malformed\n");
		return 1;
	}

	/* Calculate response length */
	u16_t resp_len = (rx_buf[1] << 7) + rx_buf[0];

	/* Create response buffers */
	for(int i = 0; i < resp_len; i++)
		tx_buf[0] = 0;
	tx_spi_buf.len = resp_len;
	rx_spi_buf.len = resp_len;

	/* Receive response */
	k_sem_take(&sem_at_ready, K_FOREVER);
	rc = spi_transceive(spi_dev, config, &tx_spi_bufs, &rx_spi_bufs);
	if(rc != 0) {
		printk("Receiving response failed\n");
		return rc;
	}

	/* Make sure response is null-terminated */
	rx_buf[resp_len] = 0;

	return 0;
}

#define AT_TERM_READY	"\r\nready\r\n"
#define AT_TERM_OK		"\r\nOK\r\n"
#define AT_TERM_ERR		"\r\nERROR\r\n"

int at_recv(struct device *spi_dev, struct spi_config *config) {
	int rc = 0;

	while(1) {
		rc = at_recv_one(spi_dev, config);
		if(rc != 0) {
			printk("Failed to receive response\n");
			return rc;
		}

		printk("%s", rx_buf);

		if(strncmp(rx_buf, AT_TERM_OK, sizeof(AT_TERM_OK) -1) == 0) {
			break;
		} else if(strncmp(rx_buf, AT_TERM_READY, sizeof(AT_TERM_READY) - 1) == 0) {
			break;
		} else if(strncmp(rx_buf, AT_TERM_ERR, sizeof(AT_TERM_ERR) - 1) == 0) {
			break;
		}
	}

	return 0;
}

int at_sendrecv(struct device *spi_dev,
		struct spi_config *config,
		char * at_cmd)
{
	int rc = 0;

	rc = at_send(spi_dev, config, at_cmd);
	if(rc != 0) {
		printk("Sending AT command failed\n");
		return rc;
	}

	rc = at_recv(spi_dev, config);
	if(rc != 0) {
		printk("Receiving AT response failed\n");
		return rc;
	}

	return 0;
}

void main(void)
{
	int rc;

	/* Get the SPI device */
	struct device *spi_dev = device_get_binding(CONFIG_SIFIVE_SPI_1_LABEL);
	if(!spi_dev) {
		printk("Cannot find device %s\n", CONFIG_SIFIVE_SPI_1_LABEL);
		return;
	}

	/* Configuration for the SPI interface */
	struct spi_config at_spi_conf = {
		.frequency = AT_SPI_BAUD,
		.operation = SPI_TRANSFER_MSB | SPI_LINES_SINGLE | (8 << 5),
		.slave = AT_CSID,
		.cs = NULL, /* Use hardware control of the CS line */
	};

	/* Get the GPIO device */
	struct device *gpio_dev = device_get_binding(CONFIG_GPIO_SIFIVE_GPIO_NAME);
	if(!gpio_dev) {
		printk("Cannot find device %s\n", CONFIG_GPIO_SIFIVE_GPIO_NAME);
		return;
	}

	/* Configure the callback for the AT Ready pin */
	gpio_init_callback(&at_ready_cb, at_ready_callback, (1 << AT_READY_PIN));
	rc = gpio_add_callback(gpio_dev, &at_ready_cb);
	if(rc) {
		printk("Error creating callback\n");
	}
	rc = gpio_pin_enable_callback(gpio_dev, AT_READY_PIN);
	if(rc) {
		printk("Unable to enable callback\n");
	}

	/* Configure AT Ready pin for interrupt */
	rc = gpio_pin_configure(gpio_dev, AT_READY_PIN, 
			(GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH));
	if(rc) {
		printk("Error configuring GPIO AT_READY_PIN\n");
		return;
	}

	printk("Finished init\n");
	
	while(1) {
		/* Confirm communication with AT interface */
		rc = at_sendrecv(spi_dev, &at_spi_conf, "AT\r\n");
		if(rc != 0) {
			printk("AT command failed\n");
		}

		k_sleep(1000);
		k_sem_give(&sem_at_ready);
	}
}

