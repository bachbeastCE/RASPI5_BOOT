
// SPDX-License-Identifier: GPL-2.0
/*
 * Omron D6T series thermal sensors driver
 *
 * Copyright (C) 2025-26 by Duy Bach Nguyen 
 * 
 * 
 * 
*/

//LIBRARIES
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/ioctl.h>

// ================ DEFINES ========================
#define DRIVER_NAME "d6t"

#define N_PIXELS(row, col) ((row) * (col))
#define N_READ(row, col) \
	(2 * (1 + N_PIXELS(row, col)) + 1) // 2 bytes per pixel and PTAT + 1 byte for CRC

#define NOT_SUPPORT 0xFF

//IOCTL COMMANDS
#define D6T_IOC_MAGIC 'x'
#define D6T_IOC_INIT _IOW(D6T_IOC_MAGIC, 0, char *) //(copy_from_user)
#define D6T_IOC_CLEAR _IO(D6T_IOC_MAGIC, 1)

// ================ STRUCTURES ========================
struct d6t_info;
struct d6t_data {
	//Manage d6t operation
	struct d6t_info *d6t_info;
	struct mutex lock;
	u8 *buf;
	u16 *raw;
	u16 n_read; // Number of bytes to read
	u16 n_raw_data; // Number of raw data points
};

enum {
	D6T_01A,
	D6T_32L_01A,
};

struct d6t_info {
	const char *model_name;
	u8 command;
	u8 row;
	u8 col;
	s8 status_reg;
	s8 iir_avg_reg;
	s8 cycle_reg;
};


struct d6t_info d6t_info_tbl[] = {
	[D6T_01A] = { "d6t01a", 0x4C, 1, 1, NOT_SUPPORT, NOT_SUPPORT,
		      NOT_SUPPORT },
	[D6T_32L_01A] = { "d6t32l01a", 0x4D, 32, 32, 0x00, 0x01, 0x02 },
	/* Add more models here if needed */
};

static struct i2c_client *d6t_client;
static dev_t d6t_dev_num;
static struct cdev d6t_cdev;
static struct class *d6t_class;
static struct d6t_data* d6t_data;

/* ================ FUNCTION DECLARATIONS ======================== */
static int ioctl_d6t_init(struct d6t_data* d6t_data,const char *name);
static int ioctl_d6t_clear(struct d6t_data* d6t_data);
static long d6t_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int d6t_read(struct file *file, char __user *buf, size_t count,
		    loff_t *ppos);
static int d6t_write(struct file *file, const char __user *buf, size_t count,
		     loff_t *ppos);
static int d6t_open(struct inode *inode, struct file *file);
static int d6t_release(struct inode *inode, struct file *file);
static int d6t_probe(struct i2c_client *client);
static void d6t_remove(struct i2c_client *client);

/* ===================== FILE OPS ======================== */
static struct file_operations d6t_fops = {
	.owner = THIS_MODULE,
	.read = d6t_read,
	.write = d6t_write,
	.unlocked_ioctl = d6t_ioctl,
	.open = d6t_open,
	.release = d6t_release,
};

/* ===================== FILE OPS TABLE ======================== */
static struct file_operations d6t_fops = {
	.owner = THIS_MODULE,
	.read = d6t_read,
	.write = d6t_write,
	.open = d6t_open,
	.release = d6t_release,
	.unlocked_ioctl = d6t_ioctl,
};

/* ===================== MATCHING ======================== */
#ifdef CONFIG_OF
static const struct of_device_id d6t_of_match[] = { { .compatible =
							      "omron,d6t" },
						    {} };
MODULE_DEVICE_TABLE(of, d6t_of_match);
#endif

static const struct i2c_device_id d6t_id[] = { { "d6t01a", D6T_01A },
					       { "d6t32l01a", D6T_32L_01A } };
MODULE_DEVICE_TABLE(i2c, d6t_id);

static struct i2c_driver d6t_driver = {
    .driver = {
        .name = "d6t",
        .of_match_table = of_match_ptr(d6t_of_match),
    },
    .probe_new = d6t_probe,
    .remove = d6t_remove,
    .id_table = d6t_id,
};

module_i2c_driver(d6t_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NGUYEN DUY BACH");
MODULE_DESCRIPTION("Omron D6T series thermal sensors driver");
MODULE_VERSION("1.0");

// ================ FUNCTIONS IMPLEMENTATION ========================
// IOCTL FUNCTIONS
static int ioctl_d6t_init(struct d6t_data* d6t_data, const char *name)
{
	if (strcmp(name, "d6t01a") == 0)
		d6t_data->d6t_info = &d6t_info_tbl[D6T_01A];
	else if (strcmp(name, "d6t32l01a") == 0)
		d6t_data->d6t_info = &d6t_info_tbl[D6T_32L_01A];
	else
	{
		pr_err("D6T: Unsupported model %s\n", name);
		return -EINVAL;
	}

	d6t_data->n_read = N_READ(d6t_data->d6t_info->row, d6t_data->d6t_info->col);
	d6t_data->n_raw_data = N_PIXELS(d6t_data->d6t_info->row, d6t_data->d6t_info->col) + 1; // +1 for PTAT
		
	d6t_data->buf = kmalloc(d6t_data->n_read * sizeof(u8), GFP_KERNEL);
	if (!d6t_data->buf) {
		pr_err("D6T: Failed to allocate buffer\n");
		return -ENOMEM;
	}

	d6t_data->raw = kmalloc(d6t_data->n_raw_data * sizeof(u16), GFP_KERNEL);
	if (!d6t_data->raw) {
		kfree(d6t_data->buf);
		pr_err("D6T: Failed to allocate raw data buffer\n");
		return -ENOMEM;
	}

	mutex_init(&d6t_data->lock);
	pr_info("D6T: Initialized with model %s\n",
		d6t_data->d6t_info->model_name);
	return 0;
}

static int ioctl_d6t_clear(struct d6t_data* d6t_data)
{
	if (!d6t_data->d6t_info) {
		pr_warn("D6T: Device not initialized\n");
		return -EINVAL;
	}

	kfree(d6t_data->buf);
	kfree(d6t_data->raw);
	d6t_data->d6t_info = NULL;
	d6t_data->buf = NULL;
	d6t_data->raw = NULL;
	d6t_data->n_read = 0;
	d6t_data->n_raw_data = 0;

	pr_info("D6T: Cleared device data\n");
	return 0;
}

static long d6t_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case D6T_IOC_INIT: {
		char name[32];
		if (copy_from_user(&name, (int __user *)arg, sizeof(name)))
			return -EFAULT;
		pr_info("Received from user: %s\n", name);
		if (ioctl_d6t_init(d6t_data,name))
			return -EFAULT;
		//pr_info("Init D6T model %s sucessfully\n", name);
		break;
	}

	case D6T_IOC_CLEAR: {
		pr_info("D6T_IOC_CLEAR called\n");
		if (ioctl_d6t_clear(d6t_data))
			return -EFAULT;
		break;
	}

	default:
		return -ENOTTY;
	}
	return 0;
}

// READ / WRITE FUNCTIONS
static u8 d6t_crc8(u8 crc, u8 data)
{
    crc ^= data;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0x07;
        else
            crc <<= 1;
    }
    return crc;
}

static bool d6t_checkPEC(struct i2c_client *client, struct d6t_data * d6t_data)
{
    u32 n =  d6t_data->n_read - 1; // Last byte is CRC
    u8 crc = 0; // initial value
    u8 addr = (client->addr << 1) | 1; // I2C Read address (8 bit)

    crc = d6t_crc8(crc, addr); // Start with address

    for (u32 i = 0; i < n; i++) {
        crc = d6t_crc8(crc, d6t_data->buf[i]);
    }

    if (crc !=  d6t_data->buf[n]) {
        pr_info("PEC check failed: calc=%02X get=%02X\n", crc, d6t_data->buf[n]);
        return true; // ERROR
    }
    return false; // OK
}

static int d6t_get_frame(struct i2c_client *d6t_client, struct d6t_data *d6t_data){
	int ret;
	struct i2c_msg msgs[2];
	u8 command = d6t_data->d6t_info->command;

	if (!d6t_client || !d6t_data || !d6t_data->buf) {
		pr_err("D6T: Invalid client or data structure\n");
		return -EINVAL;
	}
	
	msgs[0].addr = d6t_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &command;
	
	msgs[1].addr = d6t_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = d6t_data->n_read;;
	msgs[1].buf = d6t_data->buf;

	memset(d6t_data->buf, 0, msgs[1].len);

	ret = i2c_transfer(d6t_client->adapter, msgs, 2);
	if (ret < 0) {
		pr_err("D6T: I2C transfer failed: %d\n", ret);
		return ret;
	}
	
	if (ret != 2) {
		pr_err("D6T: I2C transfer returned %d messages, expected 2\n", ret);
		return -EIO;
	}
	return 0;
}

static inline int d6t_convert_u8_to_s16(struct d6t_data *d6t_data){
	u32 n = d6t_data->n_raw_data;
	for (u32 i = 0; i < n; i++) {
		d6t_data->raw[i] = (d6t_data->buf[2 * i + 1] << 8) | d6t_data->buf[2 * i];
	}
	return 0;
}

static int d6t_read(struct file *file, char __user *buf, size_t count,
		    loff_t *ppos)
{
	//struct d6t_data *d6t_data = file->private_data;
	if (!d6t_data || !d6t_data->d6t_info || !d6t_data->buf || !d6t_data->raw) {
		pr_err("D6T: Device not initialized or memory not allocated\n");
		return -EINVAL;
	}

	if (*ppos > 0) {
		return 0; // EOF
	}

	mutex_lock(&d6t_data->lock);

	if (d6t_get_frame(d6t_client, d6t_data) < 0) {
		mutex_unlock(&d6t_data->lock);
		return -EIO;
	}

	if (d6t_checkPEC(d6t_client, d6t_data)) {
		mutex_unlock(&d6t_data->lock);
		return -EIO;
	}

	d6t_convert_u8_to_s16(d6t_data);

	int ret = copy_to_user(buf, d6t_data->raw, d6t_data->n_raw_data * sizeof(u16));
	mutex_unlock(&d6t_data->lock);
	if (ret) {
		pr_err("D6T: Failed to copy data to user space\n");
		return -EFAULT;
	}
	*ppos += d6t_data->n_raw_data * sizeof(u16);
	pr_info("D6T: Read %zu bytes from device\n", d6t_data->n_raw_data * sizeof(u16));
	return d6t_data->n_raw_data * sizeof(u16);
	}

static int d6t_write(struct file *file, const char __user *buf, size_t count,
		     loff_t *ppos)
{
	if (!d6t_data || !d6t_data->d6t_info) {
		pr_err("D6T: Device not initialized\n");
		return -EINVAL;
	}

	if (*ppos > 0) {
		return 0; // EOF
	}

	if (count != sizeof(u16)) {
    pr_err("D6T: Invalid write size %zu\n", count);
    return -EINVAL;
}

	u16 msg;
	if (copy_from_user(&msg, buf, sizeof(u16))) {
    	pr_err("D6T: Failed to copy data from user\n");
    	return -EFAULT;
	}	

	u8 command = msg >> 8; // High byte is register address
	u8 value = msg & 0xFF; // Low byte is value

	if (command != d6t_data->d6t_info->iir_avg_reg &&
    	command != d6t_data->d6t_info->cycle_reg) {
    	pr_err("D6T: Unsupported register address 0x%02X\n", command);
    	return -EINVAL;
	}

	struct i2c_msg msg = {
		.addr = d6t_client->addr,
		.flags = 0,
		.len = 2,
		.buf = (u8[]){ command, value }
	};

	int ret = i2c_transfer(d6t_client->adapter, &msg, 1);
	if (ret < 0) {
		pr_err("D6T: I2C transfer failed: %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		pr_err("D6T: I2C transfer returned %d messages, expected 1\n", ret);
		return -EIO;
	}

	pr_info("D6T: Wrote value 0x%02X to register 0x%02X\n", value, command);
	return count; // Return number of bytes written
}

static int d6t_open(struct inode *inode, struct file *file){
	pr_info("D6T: Device opened\n");
	return 0;
}

static int d6t_release(struct inode *inode, struct file *file){
	pr_info("D6T: Device released\n");
	return 0;
}


/* ===================== PROBE / REMOVE ======================== */
static int d6t_probe(struct i2c_client *client)
{
	int ret;
	d6t_client = client;

	ret = alloc_chrdev_region(&d6t_dev_num, 0, 1, DRIVER_NAME);
	if (ret < 0)
		return ret;

	// Initialize cdev
	cdev_init(&d6t_cdev, &d6t_fops);
	ret = cdev_add(&d6t_cdev, d6t_dev_num, 1);
	if (ret < 0)
		goto unregister_region;

	// Create device class
	d6t_class = class_create(THIS_MODULE, "d6t_class");
	if (IS_ERR(d6t_class)) {
		ret = PTR_ERR(d6t_class);
		goto del_cdev;
	}
	
	// Create device node
	if (!device_create(d6t_class, NULL, d6t_dev_num, NULL, DRIVER_NAME)) {
		ret = -ENOMEM;
		goto destroy_class;
	}

	// Allocate memory for d6t_data
	d6t_data = kzalloc(sizeof(struct d6t_data), GFP_KERNEL);
	if (!d6t_data) {
		pr_err("D6T: Failed to allocate memory for d6t_data\n");
		ret = -ENOMEM;
		goto destroy_class;
	}

	//Get device model name from device tree or i2c id
	if (client->dev.of_node) {
		const char *model_name;
		if (of_property_read_string(client->dev.of_node, "model", &model_name) == 0) {
			ret = ioctl_d6t_init(d6t_data, model_name);
			if (ret < 0) {
				kfree(d6t_data);
				goto destroy_class;
			}
		}
	}

	pr_info("D6T: %s probed successfully\n", client->name);
	return 0;

destroy_class:
	class_destroy(d6t_class);
del_cdev:
	cdev_del(&d6t_cdev);
unregister_region:
	unregister_chrdev_region(d6t_dev_num, 1);
	kfree(d6t_data);
	return ret;
}


static void d6t_remove(struct i2c_client *client)
{
	ioctl_d6t_clear(d6t_data);
	kfree(d6t_data);
	device_destroy(d6t_class, d6t_dev_num);
	class_destroy(d6t_class);
	cdev_del(&d6t_cdev);
	unregister_chrdev_region(d6t_dev_num, 1);
	dev_info(&client->dev, "%s removed\n", DRIVER_NAME);
}
