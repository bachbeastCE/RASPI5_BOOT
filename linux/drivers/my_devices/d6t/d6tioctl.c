#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <linux/mutex.h>
#include <linux/i2c.h>

#define DEVICE_NAME "d6t"
#define CLASS_NAME  "d6t_class"

#define N_PIXELS(row, col) ((row) * (col))
#define N_READ(row, col) \
	(2 * (1 + N_PIXELS(row, col)) + 1) // 2 bytes per pixel and PTAT + 1 byte for CRC

#define NOT_SUPPORT 0xFF

// IOCTL
#define D6T_IOC_MAGIC  'x'
#define D6T_IOC_READ_RAW _IOR(D6T_IOC_MAGIC, 1, uint16_t *)
#define D6T_IOC_INIT  _IOW(D6T_IOC_MAGIC, 2, char *)  
#define D6T_IOC_CLEAR _IO(D6T_IOC_MAGIC, 3)  

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
static struct d6t_data *d6t_data;
struct device * d6t_dev;


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

static int d6t_init(struct d6t_data* d6t_data, const char *name)
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

static int d6t_clear(struct d6t_data* d6t_data)
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




/* ================= FILE OPERATIONS ================== */
static int d6t_open(struct inode *inode, struct file *file)
{
    d6t_init(d6t_data,"d6t32l01a");
    pr_info("d6t: Device opened\n");
    return 0;
}

static int d6t_release(struct inode *inode, struct file *file)
{
    d6t_clear(d6t_data);
    pr_info("d6t: Device closed\n");
    return 0;
}

static long d6t_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    if (_IOC_TYPE(cmd) != D6T_IOC_MAGIC)
        return -ENOTTY;

    switch (cmd) {
    case D6T_IOC_READ_RAW:
    {
        int ret;
        //struct d6t_data *d6t_data = file->private_data;
        if (!d6t_data || !d6t_data->d6t_info || !d6t_data->buf || !d6t_data->raw) {
            pr_err("D6T: Device not initialized or memory not allocated\n");
            return -EINVAL;
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
        ret = copy_to_user((uint16_t __user *)arg, d6t_data->raw, d6t_data->n_raw_data * sizeof(u16));
	    mutex_unlock(&d6t_data->lock);
        if (ret) {
            pr_err("D6T: Failed to copy data to user space\n");
            return -EFAULT;
        }
        
        
        //size_t bytes = RAW_SIZE * sizeof(uint16_t);
        // uint16_t *tmp = kmalloc(bytes, GFP_KERNEL);
        //if (!tmp) return -ENOMEM;
    
        // // Fake d? li?u h?p lý
        // tmp[0] = 25; // PTAT
        // int i;
        // for (i = 1; i < RAW_SIZE; i++)
        //     tmp[i] = i % 50; // pixel 0–49
    
        // if (copy_to_user((uint16_t __user *)arg, tmp, bytes)) {
        //     kfree(tmp);
        //     return -EFAULT;
        // }

        // pr_info("d6t: PTAT=%d, Pixel[0]=%d ... Pixel[1023]=%d\n",
        //         tmp[0], tmp[1], tmp[RAW_SIZE-1]);
    
        // kfree(tmp);
        break;
    }
    default:
        return -ENOTTY;
    }

    return 0;
}

static const struct file_operations d6t_fops = {
    .owner = THIS_MODULE,
    .open = d6t_open,
    .release = d6t_release,
    .unlocked_ioctl = d6t_ioctl,
};


/* ================= DEVICE MATCH ================== */
static int d6t_probe(struct i2c_client *client)
{
    int ret;

    d6t_client = client;

    ret = alloc_chrdev_region(&d6t_dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0)
        return ret;

    cdev_init(&d6t_cdev, &d6t_fops);
    d6t_cdev.owner = THIS_MODULE;
    ret = cdev_add(&d6t_cdev, d6t_dev_num, 1);
    if (ret < 0)
        goto unregister_region;

    d6t_class = class_create(CLASS_NAME);
    if (IS_ERR(d6t_class)) {
        ret = PTR_ERR(d6t_class);
        goto del_cdev;
    }

    d6t_dev = device_create(d6t_class, NULL, d6t_dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(d6t_dev)) {
        ret = PTR_ERR(d6t_dev);
        goto destroy_class;
    }

    d6t_data = kzalloc(sizeof(*d6t_data), GFP_KERNEL);
    if (!d6t_data) {
        ret = -ENOMEM;
        goto destroy_device;
    }

    mutex_init(&d6t_data->lock);

    pr_info("d6t: %s probed successfully\n", client->name);
    return 0;

destroy_device:
    device_destroy(d6t_class, d6t_dev_num);
destroy_class:
    class_destroy(d6t_class);
del_cdev:
    cdev_del(&d6t_cdev);
unregister_region:
    unregister_chrdev_region(d6t_dev_num, 1);
    return ret;
}

static void d6t_remove(struct i2c_client *client)
{
    if (d6t_data) {
        kfree(d6t_data);
        d6t_data = NULL;
    }
    if (d6t_dev)
        device_destroy(d6t_class, d6t_dev_num);
    if (!IS_ERR_OR_NULL(d6t_class))
        class_destroy(d6t_class);

    cdev_del(&d6t_cdev);
    unregister_chrdev_region(d6t_dev_num, 1);

    dev_info(&client->dev, "d6t removed\n");
}

#ifdef CONFIG_OF
static const struct of_device_id d6t_of_match[] = {
    { .compatible = "omron,d6t" },
    {},
};
MODULE_DEVICE_TABLE(of, d6t_of_match);
#endif

static const struct i2c_device_id d6t_id[] = {
    { "d6t01a", 0 },
    { "d6t32l01a", 1 },
    {}
};
MODULE_DEVICE_TABLE(i2c, d6t_id);

static struct i2c_driver d6t_driver = {
    .driver = {
        .name = "d6t",
        .of_match_table = of_match_ptr(d6t_of_match),
    },
    .probe = d6t_probe,
    .remove = d6t_remove,
    .id_table = d6t_id,
};

module_i2c_driver(d6t_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NGUYEN DUY BACH");
MODULE_DESCRIPTION("Omron D6T series thermal sensor debug driver");
MODULE_VERSION("1.0");
