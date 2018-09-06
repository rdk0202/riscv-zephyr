/*
 * Copyright (c) 2018 SiFive Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SPI_LEVEL
#include <logging/sys_log.h>

#include "spi_context.h"
#include "spi_sifive.h"

#include <device.h>
#include <spi.h>
#include <board.h>

#define MIN(x, y) ((x < y) ? x : y)

/* Structure Declarations */

struct spi_sifive_data {
	struct spi_context ctx;
};

struct spi_sifive_cfg {
	u32_t base;
	u32_t f_sys;
};

/* Helper Functions */

int spi_config(struct device * dev, u32_t frequency, u16_t operation) {
	if(SPI_OP_MODE_GET(operation) != SPI_OP_MODE_MASTER)
		return -ENOTSUP;

	if(operation & SPI_MODE_LOOP)
		return -ENOTSUP;

	/* Set the SPI frequency */
	u32_t div = (SPI_CFG(dev)->f_sys / (2 * frequency)) - 1;
	SPI_REG(dev, REG_SCKDIV) = (SF_SCKDIV_DIV_MASK & div);

	/* Set the polarity */
	if(operation & SPI_MODE_CPOL) {
		/* If CPOL is set, then SCK idles at logical 1 */
		SET_BIT(SPI_REG(dev, REG_SCKMODE), SF_SCKMODE_POL);
	} else {
		/* SCK idles at logical 0 */
		CLR_BIT(SPI_REG(dev, REG_SCKMODE), SF_SCKMODE_POL);
	}

	/* Set the phase */
	if(operation & SPI_MODE_CPHA) {
		/* If CPHA is set, then data is sampled on the trailing SCK edge */
		SET_BIT(SPI_REG(dev, REG_SCKMODE), SF_SCKMODE_PHA);
	} else {
		/* Data is sampled on the leading SCK edge */
		CLR_BIT(SPI_REG(dev, REG_SCKMODE), SF_SCKMODE_PHA);
	}

	/* Get the frame length */
	u32_t fmt_len = SPI_WORD_SIZE_GET(operation);
	if(fmt_len > SF_FMT_LEN_MASK) {
		return -ENOTSUP;
	}

	/* Set the frame length */
	fmt_len = fmt_len << SF_FMT_LEN;
	fmt_len &= SF_FMT_LEN_MASK;
	CLR_MASK(SPI_REG(dev, REG_FMT), SF_FMT_LEN_MASK);
	SPI_REG(dev, REG_FMT) |= fmt_len;

	if((operation & SPI_LINES_MASK) != SPI_LINES_SINGLE)
		return -ENOTSUP;
	/* Set single line operation */
	CLR_MASK(SPI_REG(dev, REG_FMT), SF_FMT_PROTO_MASK);

	/* Set the endianness */
	if(operation & SPI_TRANSFER_LSB)
		SET_BIT(SPI_REG(dev, REG_FMT), SF_FMT_ENDIAN);
	else
		CLR_BIT(SPI_REG(dev, REG_FMT), SF_FMT_ENDIAN);	

	/* Set CS hold */
	if(operation & SPI_HOLD_ON_CS)
		SPI_REG(dev, REG_CSMODE) = SF_CSMODE_HOLD;
	else
		SPI_REG(dev, REG_CSMODE) = SF_CSMODE_AUTO;

	return 0;
}

void spi_sifive_send(struct device *dev, u16_t frame) {
	while(SPI_REG(dev, REG_TXDATA) & SF_TXDATA_FULL) ;

	SPI_REG(dev, REG_TXDATA) = (u32_t) frame;
}

u16_t spi_sifive_recv(struct device * dev) {
	u32_t val;
	while((val = SPI_REG(dev, REG_RXDATA)) & SF_RXDATA_EMPTY) ;

	return (u16_t) val;
}

int spi_sifive_transceive_one(struct device *dev,
		const struct spi_buf *tx_buf,
		const struct spi_buf *rx_buf)
{
	u32_t send_len = 0;
	if(tx_buf->buf == NULL && rx_buf->buf == NULL) {
		return -EINVAL;
	} else if(tx_buf->buf == NULL) {
		send_len = rx_buf->len;
	} else if(rx_buf->buf == NULL) {
		send_len = tx_buf->len;
	} else {
		send_len = MIN(tx_buf->len, rx_buf->len);
	}

	for(int i = 0; i < send_len; i++) {

		/* Send a frame */
		if(tx_buf->buf != NULL) {
			spi_sifive_send(dev, ((u8_t *) tx_buf->buf)[i]);
		} else {
			/* Send dummy bytes */
			spi_sifive_send(dev, 0);
		}

		/* Receive a frame */
		if(rx_buf->buf != NULL) {
			/* TODO: what happens with frame lengths > 8? */
			((u8_t *) rx_buf->buf)[i] = (u8_t) spi_sifive_recv(dev);
		} else {
			/* Discard returned value */
			spi_sifive_recv(dev);
		}
	}

	return 0;
}

/* API Functions */

int spi_sifive_init(struct device *dev) {
	/* Disable SPI Flash mode */
	CLR_BIT(SPI_REG(dev, REG_FCTRL), SF_FCTRL_EN);

	/* Make sure the context is unlocked */
	spi_context_unlock_unconditionally(&SPI_DATA(dev)->ctx);
	return 0;
}

int spi_sifive_transceive(struct device *dev,
		const struct spi_config *config,
		const struct spi_buf_set *tx_bufs,
		const struct spi_buf_set *rx_bufs)
{
	int rc = 0;

	/* Lock the SPI Context */
	spi_context_lock(&SPI_DATA(dev)->ctx, false, NULL);

	/* Configure the SPI bus */
	SPI_DATA(dev)->ctx.config = config;

	spi_context_cs_configure(&SPI_DATA(dev)->ctx);
	spi_context_cs_control(&SPI_DATA(dev)->ctx, true);

	rc = spi_config(dev, config->frequency, config->operation);
	if(rc < 0) {
		spi_context_release(&SPI_DATA(dev)->ctx, rc);
		return rc;
	}

	/* Determine the number of transfers to perform */
	int num_xfers = 0;
	if(tx_bufs == NULL && rx_bufs == NULL) {
		/* Nothing to do */
		spi_context_release(&SPI_DATA(dev)->ctx, 0);
		return 0;
	} else if(tx_bufs == NULL) {
		num_xfers = rx_bufs->count;
	} else if(rx_bufs == NULL) {
		num_xfers = tx_bufs->count;
	} else {
		num_xfers = MIN(tx_bufs->count, rx_bufs->count);
	}

	/* Default to transferring dummy bytes/discarding received bytes */
	const struct spi_buf empty_buf = {
		.buf = NULL,
		.len = 0,
	};
	const struct spi_buf * tx_buf = &empty_buf;
	const struct spi_buf * rx_buf = &empty_buf;

	/* Perform transfers */
	for(int i = 0; i < num_xfers; i++) {
		if(tx_bufs != NULL)
			tx_buf = &(tx_bufs->buffers[i]);
		if(rx_bufs != NULL)
			rx_buf = &(rx_bufs->buffers[i]);

		rc = spi_sifive_transceive_one(dev, tx_buf, rx_buf);

		if(rc != 0) {
			spi_context_release(&SPI_DATA(dev)->ctx, rc);
			return rc;
		}
	}

	spi_context_cs_control(&SPI_DATA(dev)->ctx, false);

	spi_context_release(&SPI_DATA(dev)->ctx, 0);
	return 0;
}

int spi_sifive_release(struct device *dev, const struct spi_config *config) {
	spi_context_unlock_unconditionally(&SPI_DATA(dev)->ctx);
	return 0;
}

/* Device Instantiation */

static struct spi_driver_api spi_sifive_api = {
	.transceive = spi_sifive_transceive,
	.release = spi_sifive_release,
};

#define SPI_INIT(n)	\
	static struct spi_sifive_data spi_sifive_data_##n = { \
		SPI_CONTEXT_INIT_LOCK(spi_sifive_data_##n, ctx), \
	}; \
	static struct spi_sifive_cfg spi_sifive_cfg_##n = { \
		.base = CONFIG_SIFIVE_SPI_##n##_BASE_ADDR, \
		.f_sys = spi_sifive_port_##n##_clk_freq, \
	}; \
	DEVICE_AND_API_INIT(spi_##n, \
			CONFIG_SIFIVE_SPI_##n##_LABEL, \
			spi_sifive_init, \
			&spi_sifive_data_##n, \
			&spi_sifive_cfg_##n, \
			POST_KERNEL, \
			CONFIG_SPI_INIT_PRIORITY, \
			&spi_sifive_api)

#ifndef CONFIG_SIFIVE_SPI_0_ROM
#ifdef CONFIG_SIFIVE_SPI_0_LABEL

SPI_INIT(0);

#endif /* CONFIG_SIFIVE_SPI_0_LABEL */
#endif /* CONFIG_SIFIVE_SPI_0_ROM */

#ifdef CONFIG_SIFIVE_SPI_1_LABEL

SPI_INIT(1);

#endif /* CONFIG_SIFIVE_SPI_1_LABEL */

#ifdef CONFIG_SIFIVE_SPI_2_LABEL

SPI_INIT(2);

#endif /* CONFIG_SIFIVE_SPI_2_LABEL */

