#include <linux/module.h>  // Macro cho module kernel (module_init, module_exit, ...)
#include <linux/init.h>	   // Macro init/exit
#include <linux/i2c.h>	   // Cấu trúc và API I2C trong kernel
#include <linux/fs.h>	   // Character device: alloc_chrdev_region, struct file_operations, ...
#include <linux/uaccess.h> // copy_to_user, copy_from_user: giao tiếp user-kernel
#include <linux/cdev.h>	   // Cấu trúc và hàm cdev cho character device
#include <linux/device.h>  // class_create, device_create
#include <linux/delay.h>   // msleep, udelay...
#include <linux/types.h>

#define DRIVER_NAME "bh1750" // Tên driver
#define BH1750_I2C_ADDR 0x23 // Địa chỉ mặc định của cảm biến BH1750
#define BH1750_CMD_CONT_HRES \
	0x10 // Lệnh đo liên tục, độ phân giải cao (datasheet)

static struct i2c_client
	*bh1750_client;				   // Con trỏ đến struct đại diện cho thiết bị I2C
static dev_t dev_num;			   // Mã thiết bị (major:minor)
static struct cdev bh1750_cdev;	   // Character device cấu trúc chính
static struct class *bh1750_class; // Lớp thiết bị dùng để tạo /dev/bh1750

/* ==== Hàm đọc ánh sáng từ BH1750 qua I2C ==== */
static int bh1750_read_lux(uint16_t *raw_lux)
{
	int ret;
	int8_t buf[2];

	// Gửi lệnh đo liên tục (CONTINUOUS HIGH RESOLUTION MODE)
	ret = i2c_smbus_write_byte(bh1750_client, BH1750_CMD_CONT_HRES);
	if (ret < 0)
		return ret;

	msleep(180); // Đợi cảm biến thực hiện đo (theo datasheet cần ~180ms)

	// Đọc 2 byte dữ liệu ánh sáng từ cảm biến
	ret = i2c_master_recv(bh1750_client, buf, 2);
	if (ret < 0)
		return ret;

	// Chuyển 2 byte thành giá trị lux (theo datasheet: lux = raw / 1.2)
	*raw_lux = ((buf[0] << 8) | buf[1]);

	return 0;
}

/* ==== Hàm read() của file /dev/bh1750 ==== */
static ssize_t bh1750_read(struct file *file, char __user *buf, size_t count,
						   loff_t *ppos)
{
	uint16_t lux;
	char lux_str[16]; // buffer lưu chuỗi kết quả lux
	int len;

	// Tránh đọc lặp lại cùng dữ liệu
	if (*ppos > 0)
		return 0;

	// Đọc lux từ cảm biến
	if (bh1750_read_lux(&lux) < 0)
		return -EIO;

	// Chuyển giá trị lux sang chuỗi
	len = snprintf(lux_str, sizeof(lux_str), "%u\n", lux);

	// Gửi dữ liệu từ kernel space -> user space
	if (copy_to_user(buf, lux_str, len))
		return -EFAULT;

	*ppos += len; // Cập nhật vị trí đọc để tránh lặp
	return len;
}

/* ==== Định nghĩa file_operations cho character device ==== */
static struct file_operations bh1750_fops = {
	.owner = THIS_MODULE,
	.read = bh1750_read,
};

/* ==== Hàm probe() - gọi khi kernel phát hiện thiết bị I2C tương thích ==== */
static int bh1750_probe(struct i2c_client *client)
{
	int ret;

	bh1750_client = client; // Lưu thông tin thiết bị I2C

	// Cấp phát major/minor number cho character device
	ret = alloc_chrdev_region(&dev_num, 0, 1, DRIVER_NAME);
	if (ret < 0)
		return ret;

	// Khởi tạo và thêm character device vào kernel
	cdev_init(&bh1750_cdev, &bh1750_fops);
	ret = cdev_add(&bh1750_cdev, dev_num, 1);
	if (ret)
		goto unregister_region;

	// Tạo class để tạo thiết bị trong /dev/
	bh1750_class = class_create("bh1750_class");
	if (IS_ERR(bh1750_class))
	{
		ret = PTR_ERR(bh1750_class);
		goto del_cdev;
	}

	// Tạo thiết bị /dev/bh1750
	device_create(bh1750_class, NULL, dev_num, NULL, "bh1750");

	dev_info(&client->dev, "BH1750 character driver probed\n");
	return 0;

del_cdev:
	cdev_del(&bh1750_cdev);
unregister_region:
	unregister_chrdev_region(dev_num, 1);
	return ret;
}

/* ==== Hàm remove() - gọi khi thiết bị bị ngắt kết nối hoặc rút module ==== */
static void bh1750_remove(struct i2c_client *client)
{
	device_destroy(bh1750_class, dev_num); // Xoá /dev/bh1750
	class_destroy(bh1750_class);		   // Xoá class
	cdev_del(&bh1750_cdev);				   // Gỡ character device
	unregister_chrdev_region(dev_num, 1);  // Giải phóng số hiệu thiết bị
	dev_info(&client->dev, "BH1750 driver removed\n");
}

/* ==== Bảng ID dùng cho I2C subsystem (không dùng DT) ==== */
static const struct i2c_device_id bh1750_id[] = {
	{"bh1750", 0},
	{}};
MODULE_DEVICE_TABLE(i2c, bh1750_id); // Tạo bảng ánh xạ cho i2c

/* ==== Bảng ánh xạ Device Tree (DTS) ==== */
static const struct of_device_id bh1750_of_match[] = {
	{.compatible = "rohm,bh1750"}, // Phù hợp với dts: compatible = "rohm,bh1750"
	{}};
MODULE_DEVICE_TABLE(of, bh1750_of_match);

/* ==== Đăng ký driver với kernel I2C subsystem ==== */
static struct i2c_driver bh1750_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = bh1750_of_match, // Hỗ trợ device tree
	},
	.probe = bh1750_probe,	 // Hàm được gọi khi tìm thấy thiết bị
	.remove = bh1750_remove, // Hàm được gọi khi tháo thiết bị
	.id_table = bh1750_id,	 // Hỗ trợ non-DT
};

// Macro tạo module init/exit cho i2c driver
module_i2c_driver(bh1750_driver);

/* ==== Metadata cho kernel ==== */
MODULE_LICENSE("GPL"); // Cần cho phép module
MODULE_AUTHOR("ABC");  // Tên tác gia
MODULE_DESCRIPTION("Character device driver cho BH1750 trên Raspberry Pi");