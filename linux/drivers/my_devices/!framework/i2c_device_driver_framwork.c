#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>

#define DRIVER_NAME "device"
#defind I2C_ADDRESS = 0x3F //example 

#define COMMAND1 0x01 //example
#define COMMAND1 0x02 //example
#define COMMAND1 0x03 //example

#define REGISTER1 0x01 //example
#define REGISTER1 0x02 //example


static struct i2c_client *device_client;
static dev_t dev_num;
static struct cdev device_cdev;
static struct class *device_class;

/* ================= LOW-LEVEL I2C ACCESS ================= */
static int device_read_helper(uint16_t *buffer)
{
    int ret;
    uint8_t data[2];

    ret = i2c_smbus_write_byte(device_client, 0x10); // BH1750 command
    if (ret < 0)
        return ret;

    msleep(180);

    ret = i2c_smbus_read_i2c_block_data(device_client, 0x00, 2, data);
    if (ret < 0)
        return ret;

    *buffer = (data[0] << 8) | data[1];
    return 0;
}

static int device_write_helper(uint16_t value)
{
    uint8_t buf[2] = {value >> 8, value & 0xFF};
    return i2c_master_send(device_client, buf, 2);
}
/* ===================== FILE OPS ======================== */
static int device_open(struct inode *inode, struct file *file)
{
    pr_info("device: opened\n");
    return 0;
}

static int device_release(struct inode *inode, struct file *file)
{
    pr_info("device: released\n");
    return 0;
}

static ssize_t device_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    uint16_t val;
    char str[16];
    int len;

    if (*ppos > 0)
        return 0;

    if (device_read_helper(&val) < 0)
        return -EIO;

    len = snprintf(str, sizeof(str), "%u\n", val);

    if (copy_to_user(buf, str, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static ssize_t device_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    char kbuf[16];
    uint16_t val;

    if (count > sizeof(kbuf) - 1)
        return -EINVAL;

    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;

    kbuf[count] = '\0';
    if (kstrtou16(kbuf, 10, &val))
        return -EINVAL;

    if (device_write_helper(val) < 0)
        return -EIO;

    return count;
}

static long device_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    uint16_t value;

    switch (cmd)
    {
    case 1:
        // DO SOMETHING
        pr_info("device: ioctl command 1 received\n");
        return 0;

    case 2:
        // DO SOMETHING
        pr_info("device: ioctl command 2 received\n");
        return 0;

    default:
        return -ENOTTY;
    }
}

/* ===================== FILE OPS TABLE ======================== */
static struct file_operations device_fops = {
    .owner = THIS_MODULE,
    .read = device_read,
    .write = device_write,
    .open = device_open,
    .release = device_release,
    .unlocked_ioctl = device_ioctl,
};

/* ===================== PROBE / REMOVE ======================== */
static int device_probe(struct i2c_client *client)
{
    int ret;

    device_client = client;

    ret = alloc_chrdev_region(&dev_num, 0, 1, DRIVER_NAME);
    if (ret < 0)
        return ret;

    cdev_init(&device_cdev, &device_fops);
    ret = cdev_add(&device_cdev, dev_num, 1);
    if (ret < 0)
        goto unregister_region;

    device_class = class_create(THIS_MODULE, "device_class");
    if (IS_ERR(device_class))
    {
        ret = PTR_ERR(device_class);
        goto del_cdev;
    }

    if (!device_create(device_class, NULL, dev_num, NULL, DRIVER_NAME))
    {
        ret = -ENOMEM;
        goto destroy_class;
    }

    dev_info(&client->dev, "%s probed successfully\n", DRIVER_NAME);
    return 0;

destroy_class:
    class_destroy(device_class);
del_cdev:
    cdev_del(&device_cdev);
unregister_region:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void device_remove(struct i2c_client *client)
{
    device_destroy(device_class, dev_num);
    class_destroy(device_class);
    cdev_del(&device_cdev);
    unregister_chrdev_region(dev_num, 1);
    dev_info(&client->dev, "%s removed\n", DRIVER_NAME);
}

/* ===================== MATCHING ======================== */
#ifdef CONFIG_OF
static const struct of_device_id device_of_match[] = {
    {.compatible = "abc"},
    {}};
MODULE_DEVICE_TABLE(of, device_of_match);
#endif

static const struct i2c_device_id device_id[] = {
    {DRIVER_NAME, 0},
    {}};
MODULE_DEVICE_TABLE(i2c, device_id);

static struct i2c_driver device_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = of_match_ptr(device_of_match),
    },
    .probe_new = device_probe,
    .remove = device_remove,
    .id_table = device_id,
};
module_i2c_driver(device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("ABC");
MODULE_DESCRIPTION("I2C char driver with ioctl support");
