#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/types.h>
//#include "d6t_core.h"

#define DRIVER_NAME "D6T"

static struct i2c_client *d6t_client;
static dev_t d6t_dev_num;
static struct cdev d6t_cdev;
static struct class *d6t_class;


//static struct d6t_t d6t;

//static u8 buf[5];
//static uint16_t raw_global[2]; /* consistent type */

/* helper reads into provided uint16_t raw[2] */
static int d6t_read_helper(uint16_t *raw)
{
    uint8_t *buf;
    int ret;
    int retry;

    buf = kmalloc(2051, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    for (retry = 0; retry < 10; retry++) {
        msleep(200); // delay trước mỗi lần đọc

        ret = i2c_smbus_write_byte(d6t_client, 0x4D);
        if (ret < 0)
            continue; // thử lại

        int offset = 0;
        bool error = false;

        while (offset < 2051) {
            int to_read = min(256, 2051 - offset);
            ret = i2c_master_recv(d6t_client, buf + offset, to_read);
            if (ret < 0) {
                error = true;
                break; // lỗi, thoát vòng đọc
            }
            offset += to_read;
        }

        if (!error) {
            // chuyển dữ liệu từ buf sang raw
            for (int i = 0; i < 2050; i += 2)
                raw[i / 2] = ((buf[i + 1] << 8) | buf[i]);

            ret = 0; // đọc thành công
            goto out;
        }

        // nếu lỗi thì tiếp tục thử lại
        msleep(20); // delay nhỏ trước lần retry tiếp theo
    }

    // nếu chạy đến đây là lỗi sau 10 lần
    ret = -EIO;

out:
    kfree(buf);
    return ret;
}

/* ===================== FILE OPS ======================== */
static int d6t_open(struct inode *inode, struct file *file)
{
	pr_info("device: opened\n");

	/* khởi tạo struct d6t (sửa cú pháp, thêm dấu ; ) */
//	d6t = (struct d6t_t){
//		.name = "d6t01a",
//		.row = 1,
//		.col = 1,
//		.iir = NOT_SUPPORT,
//		.avg = NOT_SUPPORT,
//		.cycle = NOT_SUPPORT,
//		.command = 0x4C,
//		.buffer = buf,
//		.n_read = 5,
//		.raw_data = raw_global,
//		.n_raw_data = 2,
//	};

	return 0;
}

static int d6t_release(struct inode *inode, struct file *file)
{
	pr_info("device: released\n");


	return 0;
}

static ssize_t d6t_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
   uint16_t *raw = kmalloc_array(1025, sizeof(uint16_t), GFP_KERNEL);
char *raw_str = kmalloc(6200, GFP_KERNEL); 
    int len = 0, ret;

    if (!raw || !raw_str) { ret = -ENOMEM; goto out_free; }

    if (*ppos > 0) { ret = 0; goto out_free; }

    if (d6t_read_helper(raw) < 0) { ret = -EIO; goto out_free; }

    for (int i = 0; i < 1025; i++)
        len += scnprintf(raw_str + len, 6200 - len, "%u ", raw[i]);

    if (copy_to_user(ubuf, raw_str, len))
        ret = -EFAULT;
    else {
        *ppos += len;
        ret = len;
    }

out_free:
  kfree(raw);
  kfree(raw_str);
    
    return ret;
}

static ssize_t d6t_write(struct file *file, const char __user *ubuf,
			    size_t count, loff_t *ppos)
{
	char kbuf[16];
	uint16_t val;

	if (count > sizeof(kbuf) - 1)
		return -EINVAL;

	if (copy_from_user(kbuf, ubuf, count))
		return -EFAULT;

	kbuf[count] = '\0';
	if (kstrtou16(kbuf, 10, &val))
		return -EINVAL;

	/* TODO: handle val */

	return count;
}

static long d6t_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case 1:
		pr_info("device: ioctl command 1 received\n");
		return 0;
	case 2:
		pr_info("device: ioctl command 2 received\n");
		return 0;
	default:
		return -ENOTTY;
	}
}

/* ===================== FILE OPS TABLE ======================== */
static const struct file_operations d6t_fops = {
	.owner = THIS_MODULE,
	.read = d6t_read,
	.write = d6t_write,
	.open = d6t_open,
	.release = d6t_release,
	.unlocked_ioctl = d6t_ioctl,
};

/* ===================== PROBE / REMOVE ======================== */
static int d6t_probe(struct i2c_client *client)
{
	int ret;
	struct device *dev_ret;

	d6t_client = client;

	ret = alloc_chrdev_region(&d6t_dev_num, 0, 1, DRIVER_NAME);
	if (ret < 0)
		return ret;

	cdev_init(&d6t_cdev, &d6t_fops);
	ret = cdev_add(&d6t_cdev, d6t_dev_num, 1);
	if (ret < 0)
		goto unregister_region;

	d6t_class = class_create("d6t_class");
	if (IS_ERR(d6t_class)) {
		ret = PTR_ERR(d6t_class);
		goto del_cdev;
	}

	dev_ret = device_create(d6t_class, NULL, d6t_dev_num, NULL, DRIVER_NAME);
	if (IS_ERR(dev_ret)) {
		ret = PTR_ERR(dev_ret);
		goto destroy_class;
	}

	dev_info(&client->dev, "%s probed successfully\n", DRIVER_NAME);
	return 0;

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
	device_destroy(d6t_class, d6t_dev_num);
	class_destroy(d6t_class);
	cdev_del(&d6t_cdev);
	unregister_chrdev_region(d6t_dev_num, 1);
	dev_info(&client->dev, "%s removed\n", DRIVER_NAME);
}

/* ===================== MATCHING ======================== */
#ifdef CONFIG_OF
static const struct of_device_id d6t_of_match[] = {
	{ .compatible = "omron,d6t" },
	{ }
};
MODULE_DEVICE_TABLE(of, d6t_of_match);
#endif

static const struct i2c_device_id d6t_id[] = { { DRIVER_NAME, 0 }, { } };
MODULE_DEVICE_TABLE(i2c, d6t_id);

static struct i2c_driver device_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = of_match_ptr(d6t_of_match),
	},
	.probe = d6t_probe,
	.remove = d6t_remove,
	.id_table = d6t_id,
};

module_i2c_driver(device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ABC");
MODULE_DESCRIPTION("I2C char driver with ioctl support"); 

