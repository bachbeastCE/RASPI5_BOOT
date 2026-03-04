#include <linux/module.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/acpi.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>

#define DEVICE_NAME "sx1278"
#define CLASS_NAME  "sx1278_class"

//--------- MODES ---------//
#define SLEEP_MODE 0
#define STDBY_MODE 1
#define FREQUENCY_SYSTHESIS_TX_MODE 2
#define TRANSMIT_MODE 3
#define FREQUENCY_SYSTHESIS_RX_MODE 4
#define RXCONTINUOUS_MODE 5
#define RXSINGLE_MODE 6
#define CAD_MODE 7

//------- BANDWIDTH -------//
#define BW_7_8KHz			0
#define BW_10_4KHz			1
#define BW_15_6KHz			2
#define BW_20_8KHz			3
#define BW_31_25KHz			4
#define BW_41_7KHz			5
#define BW_62_5KHz			6
#define BW_125KHz			7
#define BW_250KHz			8
#define BW_500KHz			9

//------ CODING RATE ------//
#define CR_4_5				1
#define CR_4_6				2
#define CR_4_7				3
#define CR_4_8				4

//--- SPREADING FACTORS ---//
#define SF_7				7
#define SF_8				8
#define SF_9				9
#define SF_10				10
#define SF_11  				11
#define SF_12				12

//------ POWER GAIN ------//
#define POWER_11db			0xF6
#define POWER_14db			0xF9
#define POWER_17db			0xFC
#define POWER_20db			0xFF

//------- REGISTERS -------//
#define RegFiFo					0x00
#define RegOpMode				0x01
// Reversed 				0x02 - 0x05
#define RegFrMsb				0x06
#define RegFrMid				0x07
#define RegFrLsb				0x08
#define RegPaConfig				0x09
#define RegPaRamp				0x0A
#define RegOcp					0x0B
#define RegLna					0x0C
#define RegFiFoAddPtr			0x0D
#define RegFiFoTxBaseAddr		0x0E
#define RegFiFoRxBaseAddr		0x0F
#define RegFiFoRxCurrentAddr	0x10
#define RegIrqFlagsMask			0x11
#define RegIrqFlags				0x12
#define RegRxNbBytes			0x13
#define RegPktRssiValue			0x1A
#define RegRssiValue 			0x1B
#define RegHopChannel 			0x1C
#define	RegModemConfig1			0x1D
#define RegModemConfig2			0x1E
#define RegSymbTimeoutLsb		0x1F
#define RegPreambleMsb			0x20
#define RegPreambleLsb			0x21
#define RegPayloadLength		0x22
#define RegMaxPayloadLength 	0x23
#define RegHopPeriod			0x24
#define RegFifoRxByteAddr 		0x25
#define RegModemConfig3			0x26
#define RegPpmCorrection 		0x27
#define RegFeiMsb 				0x28
#define RegFeiMid 				0x29
#define RegFeiLsb 				0x2A
// Reversed 				0x2B
#define RegRssiWideband 		0x2C
// Reversed 				0x2D - 0x2E
#define RegIfFreq2 				0x2F
#define RegIfFreq1 				0x30
#define RegDetectOptimize 		0x31
// Reversed 				0x32
#define RegInvertIQ				0x33
// Reversed 				0x34 - 0x35
#define	RegHighBWOptimize1		0x36
#define RegDetectionThreshold	0x37
// Reversed 				0x38
#define RegSyncWord				0x39
#define RegDioMapping1			0x40
#define RegDioMapping2			0x41
#define RegVersion				0x42

//------ LORA STATUS ------//
#define LORA_OK				200
#define LORA_NOT_FOUND			404
#define LORA_LARGE_PAYLOAD		413
#define LORA_UNAVAILABLE		503


static const struct regmap_config sx1278_regmap_config = {
    .reg_bits = 8,              // Address register 8-bit
    .val_bits = 8,              // Value register  8-bit
    .write_flag_mask = 0x80,    // MSB = 1 when write reg
    .max_register = 0x70,       // Max address of module
    .cache_type = REGCACHE_NONE, // Turn of cache to read directly
};

//HAHAHAH
typedef struct LoRa_setting {
    // Hardware settings 
    struct spi_device *spi;      
    struct regmap     *regmap;   
    struct gpio_desc  *cs_pin;    
    struct gpio_desc  *reset_pin;

    // Character Device components
    dev_t dev_num;              // device order
    struct cdev lora_cdev;      // char dev
    struct class *lora_class;    
    struct device *lora_device; 

    // Module settings:
    int      current_mode;
    uint32_t frequency;          
    uint8_t  spredingFactor;
    uint8_t  bandWidth;
    uint8_t  crcRate;
    uint16_t preamble;
    uint8_t  power;
    uint8_t  overCurrentProtection;
} LoRa;

static void LoRa_reset(LoRa* _LoRa){

}

uint8_t LoRa_isvalid(LoRa* _LoRa);
static uint8_t LoRa_read(LoRa* _LoRa, uint8_t address){

}


void LoRa_write(LoRa* _LoRa, uint8_t address, uint8_t value);
void LoRa_burstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length);
void LoRa_burstRead(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length);
void LoRa_gotoMode(LoRa* _LoRa, int mode);
void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value);
void LoRa_setAutoLDO(LoRa* _LoRa);
void LoRa_setFrequency(LoRa* _LoRa, int freq);
void LoRa_setSpreadingFactor(LoRa* _LoRa, int SP);
void LoRa_setPower(LoRa* _LoRa, uint8_t power);
void LoRa_setOCP(LoRa* _LoRa, uint8_t current);
void LoRa_setTOMsb_setCRCon(LoRa* _LoRa);
void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword);
int LoRa_getRSSI(LoRa* _LoRa);
uint16_t LoRa_init(LoRa* _LoRa);
uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout);
void LoRa_startReceiving(LoRa* _LoRa);
uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);









static int sx1278_open(struct inode *inode, struct file *file)
{
    pr_info("%s: Device opened\n",DEVICE_NAME);
    return 0;
}

static int sx1278_release(struct inode *inode, struct file *file){
    pr_info("%s: Device closed\n",DEVICE_NAME);
    return 0;
}

static int sx1278_ioctl(struct inode *inode, struct file *file){
    pr_info("%s: Device called IOCTL\n",DEVICE_NAME);
    return 0;
}

static const struct file_operations sx1278_fops = {
    .owner = THIS_MODULE,
    .open = sx1278_open,
    .release = sx1278_release,
    .unlocked_ioctl = sx1278_ioctl,
};


static void sx1278_probe(struct spi_device *spi) 
{
    int ret;
    LoRa* lora;

    lora = devm_kzalloc(&spi->dev, sizeof(*lora), GFP_KERNEL);
    if (!lora) return -ENOMEM;

    spi_set_drvdata(spi, lora);
    lora->spi = spi;

    // Set RST and CS Pin:
    lora->reset_pin = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(lora->reset_pin)) {
        return dev_err_probe(&spi->dev, PTR_ERR(lora->reset_pin), "Failed to get reset GPIO\n");
    }

    lora->cs_pin = devm_gpiod_get(&spi->dev, "manual-cs", GPIOD_OUT_HIGH);
    if (IS_ERR(lora->cs_pin)) {
        return dev_err_probe(&spi->dev, PTR_ERR(lora->cs_pin), "Failed to get CS GPIO\n");
    }

    lora->regmap = devm_regmap_init_spi(spi, &sx1278_regmap_config);

    //Default Configuration
	lora->frequency             	= 433       		; //Defaut Fre 433(Asia)
	lora->spredingFactor        	= SF_7      		; //Defaut 7kHZ
	lora->bandWidth			   	    = BW_125KHz 		; //Defaut 125kHZ
	lora->crcRate               	= CR_4_5    		; //Defaut 4/5
	lora->power				   	    = POWER_20db        ; //Defaut max 20db
	lora->overCurrentProtection 	= 100       		; //Defaut Imax 100 --> OcpTrim = 0x0b
	lora->preamble			   	    = 8         		;

    //Chardev registration
    ret = alloc_chrdev_region(lora->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0)
        return ret;

    cdev_init(&lora->lora_cdev, &sx1278_fops);
    lora->lora_cdev.owner = THIS_MODULE;

    ret = cdev_add(&lora->lora_cdev, lora->dev_num, 1);
    if (ret < 0)
        goto unregister_region;

    lora->lora_class = class_create(CLASS_NAME);
    if (IS_ERR(lora->lora_class)) {
        ret = PTR_ERR(lora->lora_class);
        goto del_cdev;
    }

    lora->lora_device = device_create(lora->lora_class, NULL, lora->dev_num, NULL, DEVICE_NAME);
    if (IS_ERR(lora->lora_device)) {
        ret = PTR_ERR(lora->lora_device);
        goto destroy_class;
    }

    pr_info("%s: SPI bus %d probed successfully\n", DEVICE_NAME, spi->controller->bus_num);
    return 0;

destroy_device:
    device_destroy(lora->lora_class, lora->dev_num);
destroy_class:
    class_destroy(lora->lora_class);
del_cdev:
    cdev_del(&lora->lora_cdev);
unregister_region:
    unregister_chrdev_region(lora->dev_num, 1);
    return ret;
}

static void sx1278_remove(struct spi_device *spi) {
    LoRa *lora = spi_get_drvdata(spi);

    if (!lora) return;

    if (lora->lora_device) {
        device_destroy(lora->lora_class, lora->dev_num);
    }

    if (!IS_ERR_OR_NULL(lora->lora_class)) {
        class_destroy(lora->lora_class);
    }

    cdev_del(&lora->lora_cdev);
    unregister_chrdev_region(lora->dev_num, 1);

    // Ghi log (Dùng &spi->dev để đúng chuẩn dev_info)
    dev_info(&spi->dev, "SX1278 LoRa driver removed successfully\n");
}

static const struct of_device_id sx1278_dt_ids[] = {
    { .compatible = "semtech,sx1278", },
    { }
};

MODULE_DEVICE_TABLE(of, sx1278_dt_ids);

static struct spi_driver sx1278_driver = {
    .driver = {
        .name = "sx1278_lora",
        .of_match_table = sx1278_dt_ids,
    },
    .probe = sx1278_probe,
    .remove = sx1278_remove,
};

module_spi_driver(sx1278_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Duy Bach - HCMUT");
MODULE_DESCRIPTION("SX1278 LoRa SPI Driver for RPi 5");