/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2020 Analog Devices, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * */

#ifndef __IIO_BACKEND_H__
#define __IIO_BACKEND_H__

#include <stdbool.h>

struct iio_device;
struct iio_context;

enum iio_attr_type {
	IIO_ATTR_TYPE_DEVICE = 0,
	IIO_ATTR_TYPE_DEBUG,
	IIO_ATTR_TYPE_BUFFER,
};

struct iio_backend_ops {
	struct iio_context * (*clone)(const struct iio_context *ctx);
	ssize_t (*read)(const struct iio_device *dev, void *dst, size_t len,
			uint32_t *mask, size_t words);
	ssize_t (*write)(const struct iio_device *dev,
			const void *src, size_t len);
	int (*open)(const struct iio_device *dev,
			size_t samples_count, bool cyclic);
	int (*close)(const struct iio_device *dev);
	int (*get_fd)(const struct iio_device *dev);
	int (*set_blocking_mode)(const struct iio_device *dev, bool blocking);

	void (*cancel)(const struct iio_device *dev);

	int (*set_kernel_buffers_count)(const struct iio_device *dev,
			unsigned int nb_blocks);
	ssize_t (*get_buffer)(const struct iio_device *dev,
			void **addr_ptr, size_t bytes_used,
			uint32_t *mask, size_t words);

	ssize_t (*read_device_attr)(const struct iio_device *dev,
			const char *attr, char *dst, size_t len, enum iio_attr_type);
	ssize_t (*write_device_attr)(const struct iio_device *dev,
			const char *attr, const char *src,
			size_t len, enum iio_attr_type);
	ssize_t (*read_channel_attr)(const struct iio_channel *chn,
			const char *attr, char *dst, size_t len);
	ssize_t (*write_channel_attr)(const struct iio_channel *chn,
			const char *attr, const char *src, size_t len);

	int (*get_trigger)(const struct iio_device *dev,
			const struct iio_device **trigger);
	int (*set_trigger)(const struct iio_device *dev,
			const struct iio_device *trigger);

	void (*shutdown)(struct iio_context *ctx);

	int (*get_version)(const struct iio_context *ctx, unsigned int *major,
			unsigned int *minor, char git_tag[8]);

	int (*set_timeout)(struct iio_context *ctx, unsigned int timeout);
};

#endif /* __IIO_BACKEND_H__ */
