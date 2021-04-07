// SPDX-License-Identifier: LGPL-2.1-or-later
/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "../debug.h"
#include "ops.h"
#include "thread-pool.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <termios.h>

struct serial_pdata {
	struct iio_context *ctx;
	bool debug;
	int fd;
	const void *xml_zstd;
	size_t xml_zstd_len;
};

static void serial_main(struct thread_pool *pool, void *d)
{
	struct serial_pdata *pdata = d;

	do {
		interpreter(pdata->ctx, pdata->fd, pdata->fd, pdata->debug,
			    false, false, pool,
			    pdata->xml_zstd, pdata->xml_zstd_len);
	} while (!thread_pool_is_stopped(pool));

	close(pdata->fd);
	free(pdata);
}

static int serial_configure(int fd, unsigned int uart_bps, unsigned int uart_bits)
{
	struct termios tty_attrs;
	int err;

	err = tcgetattr(fd, &tty_attrs);
	if (err == -1) {
		IIO_ERROR("tcgetattr failed\n");
		return -errno;
	}

	tty_attrs.c_lflag &= ~ECHO;
	tty_attrs.c_oflag &= ~OPOST;

	tty_attrs.c_cflag |= CLOCAL | CREAD;
	tty_attrs.c_cflag &= ~(CSIZE | CBAUD);

#define CASE_BPS(bps, attr) case bps: (attr)->c_cflag |= B##bps; break
	switch (uart_bps) {
	CASE_BPS(50, &tty_attrs);
	CASE_BPS(75, &tty_attrs);
	CASE_BPS(110, &tty_attrs);
	CASE_BPS(134, &tty_attrs);
	CASE_BPS(150, &tty_attrs);
	CASE_BPS(200, &tty_attrs);
	CASE_BPS(300, &tty_attrs);
	CASE_BPS(600, &tty_attrs);
	CASE_BPS(1200, &tty_attrs);
	CASE_BPS(1800, &tty_attrs);
	CASE_BPS(2400, &tty_attrs);
	CASE_BPS(4800, &tty_attrs);
	CASE_BPS(9600, &tty_attrs);
	CASE_BPS(19200, &tty_attrs);
	CASE_BPS(38400, &tty_attrs);
	CASE_BPS(57600, &tty_attrs);
	CASE_BPS(115200, &tty_attrs);
	CASE_BPS(230400, &tty_attrs);
	CASE_BPS(460800, &tty_attrs);
	CASE_BPS(500000, &tty_attrs);
	CASE_BPS(576000, &tty_attrs);
	CASE_BPS(921600, &tty_attrs);
	CASE_BPS(1000000, &tty_attrs);
	CASE_BPS(1152000, &tty_attrs);
	CASE_BPS(1500000, &tty_attrs);
	CASE_BPS(2000000, &tty_attrs);
	CASE_BPS(2500000, &tty_attrs);
	CASE_BPS(3000000, &tty_attrs);
	CASE_BPS(3500000, &tty_attrs);
	CASE_BPS(4000000, &tty_attrs);
	default:
		IIO_ERROR("Invalid baud rate\n");
		return -EINVAL;
	}

	switch (uart_bits) {
	case 5:
		tty_attrs.c_cflag |= CS5;
		break;
	case 6:
		tty_attrs.c_cflag |= CS6;
		break;
	case 7:
		tty_attrs.c_cflag |= CS7;
		break;
	case 8:
		tty_attrs.c_cflag |= CS8;
		break;
	default:
		IIO_ERROR("Invalid serial configuration\n");
		return -EINVAL;
	}

	err = tcsetattr(fd, TCSANOW, &tty_attrs);
	if (err == -1) {
		IIO_ERROR("Unable to apply serial settings\n");
		return -errno;
	}

	return 0;
}

int start_serial_daemon(struct iio_context *ctx, const char *dev,
			unsigned int uart_bps, unsigned int uart_bits,
			bool debug, struct thread_pool *pool,
			const void *xml_zstd, size_t xml_zstd_len)
{
	struct serial_pdata *pdata;
	int fd, err;

	pdata = zalloc(sizeof(*pdata));
	if (!pdata)
		return -ENOMEM;

	fd = open(dev, O_RDWR | O_CLOEXEC);
	if (fd == -1) {
		err = -errno;
		goto err_free_pdata;
	}

	err = serial_configure(fd, uart_bps, uart_bits);
	if (err)
		goto err_close_fd;

	pdata->ctx = ctx;
	pdata->debug = debug;
	pdata->fd = fd;
	pdata->xml_zstd = xml_zstd;
	pdata->xml_zstd_len = xml_zstd_len;

	IIO_DEBUG("Serving over UART on %s at %u bps, %u bits\n",
		  dev, uart_bps, uart_bits);

	return thread_pool_add_thread(pool, serial_main, pdata, "iiod_serial_thd");

err_close_fd:
	close(fd);
err_free_pdata:
	free(pdata);
	return err;
}
