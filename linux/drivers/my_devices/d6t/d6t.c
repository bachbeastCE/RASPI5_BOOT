#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>

#define DRIVER_NAME "d6t"

static struct i2c_client *d6t_client; 
static dev_t d6t_dev_num;
static struct cdev d6t_cdev;
static struct class *d6t_class;
static struct d6t_t d6t;

/* helper reads into provided uint16_t raw[2] */
static int d6t_read_helper(uint16_t *raw)
{
	int ret;
	uint8_t buf_local[5];

	/* gửi lệnh đo */
	ret = i2c_smbus_write_byte(d6t_client, 0x4C);
	if (ret < 0)
		return ret;

	msleep(200);

	ret = i2c_master_recv(d6t_client, buf_local, 5);
	if (ret < 0)
		return ret;

	/* ghép little-endian (low byte first) */
	raw[0] = ((uint16_t)buf_local[1] << 8) | buf_local[0];
	raw[1] = ((uint16_t)buf_local[3] << 8) | buf_local[2];

	return 0;
}

/* ===================== FILE OPS ======================== */
static int d6t_open(struct inode *inode, struct file *file)
{
//	pr_info("device: opened\n");
//
//	/* khởi tạo struct d6t (sửa cú pháp, thêm dấu ; ) */
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
//
//	return 0;
}

static int d6t_release(struct inode *inode, struct file *file)
{
	pr_info("device: released\n");
	return 0;
}

static ssize_t d6t_read(struct file *file, char __user *ubuf, size_t count,
			loff_t *ppos)
{
	uint16_t raw[2];
	char raw_str[32]; /* đủ lớn để chứa 2 số text */
	int len = 0;
	int ret;

	if (*ppos > 0)
		return 0;

	ret = d6t_read_helper(raw);
	if (ret < 0)
		return -EIO;

	/* nối từng phần tử vào buffer chuỗi */
	len += snprintf(raw_str + len, sizeof(raw_str) - len, "%d ", raw[0]);
	len += snprintf(raw_str + len, sizeof(raw_str) - len, "%d\n", raw[1]);

	/* đảm bảo không vượt quá giới hạn count request của userspace */
	if ((size_t)len > count)
		return -EINVAL;

	if (copy_to_user(ubuf, raw_str, len))
		return -EFAULT;

	*ppos += len;
	return len;
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
	.probe_new = d6t_probe,
	.remove = d6t_remove,
	.id_table = d6t_id,
};

module_i2c_driver(device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ABC");
MODULE_DESCRIPTION("I2C char driver with ioctl support");
