// SPDX-License-Identifier: GPL-2.0-only
/*
 * Main ctrl for Omron D6T series thermal sensors
 *
 * Copyright (C) 2025-26 by Duy Bach Nguyen <duybach2808@gmail.com>
 *
 */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/printk.h>
#include <linux/string.h>
#include "d6t_core.h"

static inline struct d6t_t *d6t_init(const char *name)
{
	struct d6t_t *d6t;

	d6t = kmalloc(sizeof(struct d6t_t), GFP_KERNEL);
	if (!d6t)
		return NULL;

	if (strcmp(name, "d6t01a") == 0) {
		pr_info("D6T: Initialize D6T - Model: d6t01a\n");
		d6t->name = "d6t01a";
		d6t->row = 1;
		d6t->col = 1;
		d6t->iir = NOT_SUPPORT;
		d6t->avg = NOT_SUPPORT;
		d6t->cycle = NOT_SUPPORT;
		d6t->command = D6T_01A_COMMAND;

		d6t->n_read = 2 * (1 + d6t->row * d6t->col) + 1;
		d6t->buffer = kmalloc(d6t->n_read * sizeof(u8), GFP_KERNEL);

		d6t->n_raw_data = 1 + d6t->row * d6t->col;
		d6t->raw_data =
			kmalloc(d6t->n_raw_data * sizeof(u16), GFP_KERNEL);

		return d6t;
	}

	if (strcmp(name, "d6t32l01a") == 0) {
		pr_info("D6T: Initialize D6T - Model: d6t32l01a\n");
		d6t->name = "d6t32l01a";
		d6t->row = 32;
		d6t->col = 32;
		d6t->iir = 5;
		d6t->avg = 5;
		d6t->cycle = 200;
		d6t->command = D6T_32L_01A_COMMAND;

		d6t->n_read = 2 * (1 + d6t->row * d6t->col) + 1;
		d6t->buffer = kmalloc(d6t->n_read * sizeof(u8), GFP_KERNEL);

		d6t->n_raw_data = 1 + d6t->row * d6t->col;
		d6t->raw_data =
			kmalloc(d6t->n_raw_data * sizeof(u16), GFP_KERNEL);

		return d6t;
	}

	pr_info("D6T: Model can't be recognized \n");
	kfree(d6t);
	return NULL;
}

static inline u32 d6t_clear(struct d6t_t **d6t_ptr)
{
	struct d6t_t *d6t;

	if (!d6t_ptr || !*d6t_ptr) {
		pr_warn("D6T: Cannot clear, device structure is NULL\n");
		return 1;
	}

	d6t = *d6t_ptr;

	if (d6t->buffer) {
		kfree(d6t->buffer);
		d6t->buffer = NULL;
	}

	if (d6t->raw_data) {
		kfree(d6t->raw_data);
		d6t->raw_data = NULL;
	}

	kfree(d6t);
	*d6t_ptr = NULL;

	return 0;
}

static inline u32 d6t_read_data(struct i2c_client *client, struct d6t_t *d6t)
{
	if (client == NULL) {
		pr_info("D6T: I2C client has not initialized yet \n");
		return 1;
	}

	if (d6t == NULL) {
		pr_info("D6T: D6T has not initialized yet, please run d6t_init()\n");
		return 2;
	}

	struct i2c_msg msgs[2];
	int ret;

	/* Gửi địa chỉ thanh ghi cần đọc */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0; // Write
	msgs[0].len = 1;
	msgs[0].buf = d6t->command;

	/* Đọc dữ liệu */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD; // Read
	msgs[1].len = d6t->n_read;
	msgs[1].buf = d6t->buffer;

	memset(d6t->buffer, 0, d6t->n_read);

	//try 10 lần
	for (int i = 0; i < 10; i++) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret != 2) {
			pr_err("I2c_transfer failed: %d\n", ret);
			if (i == 9)
				return 3;
			msleep(100);
		} else {
			break;
		}
	}

	ret = D6T_checkPEC(client, d6t);
	if (ret != 1) {
		return 4;
	}

	u16 n = d6t->n_raw_data;
	for (int i = 0; i < n; i++) {
		d6t->raw_data[i] = conv8us_s16_le(d6t->buffer, 2 * i);
	}

	return 0; // OK
}

static inline u32 calc_crc(uint8_t data)
{
	u32 index;
	u8 temp;
	for (index = 0; index < 8; index++) {
		temp = data;
		data <<= 1;
		if (temp & 0x80) {
			data ^= 0x07;
		}
	}
	return data;
}

static inline u32 d6t_write(struct i2c_client *client, struct d6t_t *d6t,
			    u8 reg, u8 value)
{
	uint8_t data[2];
	data[0] = reg;
	data[2] = value;
	int ret = i2c_master_send(client, data, 2);

	if (ret != 2)
		return 1;
}

static inline u32 d6t_checkPEC(struct i2c_client *client, struct d6t_t *d6t)
{
	u32 i;
	u32 n = d6t->n_read - 1;
	u8 crc = calc_crc(((client->addr) << 1) | 1); // I2C Read address (8bit)
	for (i = 0; i < n; i++) {
		crc = calc_crc(d6t->buffer[i] ^ crc);
	}
	u32 ret = crc != d6t->buffer[n];
	if (ret) {
		pr_info("PEC check failed: %02X(cal)-%02X(get)\n", crc,
			d6t->buffer[n]);
	}
	return ret;
}

static inline s16 conv8us_s16_le(uint8_t *buf, int n)
{
	s16 ret;
	ret = (s16)buf[n];
	ret += ((s16)buf[n + 1]) << 8;
	return (s16)ret; // and convert negative.
}