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
#include "sx1278_ioctl.h"

#define DEVICE_NAME "sx1278"
#define CLASS_NAME  "sx1278_class"

#define MAX_FIFO_SIZE 256

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
    
    //Buffer & synchronization
    uint8_t *buffer; 
    struct mutex lock;

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

static void LoRa_reset(LoRa *lora) {
    gpiod_set_value(lora->reset_pin, 1); 
    msleep(10); 

    gpiod_set_value(lora->reset_pin, 0); 
    msleep(10);
}

static uint8_t LoRa_isvalid(struct LoRa_setting *lora) {
    
    /* 1. Frequency Validation: SX1278 supports 137MHz to 525MHz range */
    if (lora->frequency < 137000000 || lora->frequency > 525000000) {
        dev_err(&lora->spi->dev, "Invalid Frequency: %u Hz (Range: 137M-525M)\n", lora->frequency);
        return 0;
    }

    /* 2. Spreading Factor (SF): LoRa supports SF6 to SF12 */
    if (lora->spredingFactor < 6 || lora->spredingFactor > 12) {
        dev_err(&lora->spi->dev, "Invalid SF: %u (Valid range: 6-12)\n", lora->spredingFactor);
        return 0;
    }

    /* 3. Bandwidth (BW): Index must be 0-9 corresponding to 7.8kHz-500kHz */
    if (lora->bandWidth > 9) {
        dev_err(&lora->spi->dev, "Invalid BW Index: %u (Valid range: 0-9)\n", lora->bandWidth);
        return 0;
    }

    /* 4. Coding Rate (CR): Usually 1 (4/5) to 4 (4/8) */
    if (lora->crcRate < 1 || lora->crcRate > 4) {
        dev_err(&lora->spi->dev, "Invalid Coding Rate: %u (Valid range: 1-4)\n", lora->crcRate);
        return 0;
    }

    /* 5. Output Power: 2dBm to 20dBm (using PA_BOOST for higher power) */
    if (lora->power < 2 || lora->power > 20) {
        dev_err(&lora->spi->dev, "Invalid Power: %u dBm (Valid range: 2-20)\n", lora->power);
        return 0;
    }

    /* 6. Over Current Protection (OCP): Safety limit for current draw */
    if (lora->overCurrentProtection < 45 || lora->overCurrentProtection > 240) {
        dev_err(&lora->spi->dev, "Invalid OCP: %u mA (Valid range: 45-240)\n", lora->overCurrentProtection);
        return 0;
    }

    /* 7. Preamble Length: Minimum 6 programmed symbols required */
    if (lora->preamble < 6) {
        dev_err(&lora->spi->dev, "Preamble too short: %u (Min: 6)\n", lora->preamble);
        return 0;
    }

    return 1; // All parameters are valid
}

static uint8_t LoRa_read(LoRa *lora, uint8_t address) {
    unsigned int val; // Regmap require 32bits value to receive data
    int ret;
    ret = regmap_read(lora->regmap, address, &val);
    
    if (ret < 0) {
        dev_err(&lora->spi->dev, "Error reading register at 0x%02x: %d\n", address, ret);
        return 0xFF;
    }
    return (uint8_t)val;
}

static void LoRa_write(struct LoRa_setting *_LoRa, uint8_t address, uint8_t value) {
    int ret = regmap_write(_LoRa->regmap, (unsigned int)address, (unsigned int)value);
    if (ret < 0) {
        dev_err(&_LoRa->spi->dev, "Error writing register at 0x%02x: %d\n", address, ret);
        return;
    }
   return;
}

static void LoRa_burstWrite(struct LoRa_setting *_LoRa, uint8_t address, uint8_t *value, uint8_t length) {
    int ret;
    ret = regmap_bulk_write(_LoRa->regmap, (unsigned int)address, value, length);
    if (ret < 0) {
        dev_err(&_LoRa->spi->dev, "Burst write failed at 0x%02x, len %d: %d\n", address, length, ret);
        return;
    }
}

static void LoRa_burstRead(struct LoRa_setting *_LoRa, uint8_t address, uint8_t *value, uint8_t length) {
    int ret;
    ret = regmap_bulk_read(_LoRa->regmap, (unsigned int)address, value, length);
    if (ret < 0) {
        dev_err(&_LoRa->spi->dev, "Burst read failed at 0x%02x, len %d: %d\n", address, length, ret);
        return;
    }
}

static void LoRa_gotoMode(LoRa* _LoRa, int mode){
    uint8_t  read;
	uint8_t  data;

	//Read the current value of RegOpMode
	read = LoRa_read(_LoRa, RegOpMode);
	data = read;

	//Read & F8: Mask (7:3) bits
    switch (mode) {
        case SLEEP_MODE:
            data = (read & 0xF8) | 0x00;
		    _LoRa->current_mode = SLEEP_MODE;
            break;
        case STDBY_MODE:
            data = (read & 0xF8) | 0x01;
		    _LoRa->current_mode = STDBY_MODE;
            break;
        case FREQUENCY_SYSTHESIS_TX_MODE:
            data = (read & 0xF8) | 0x02;
			_LoRa->current_mode = FREQUENCY_SYSTHESIS_TX_MODE;
            break;
        case TRANSMIT_MODE:
            data = (read & 0xF8) | 0x03;
		    _LoRa->current_mode = TRANSMIT_MODE;
            break;
        case FREQUENCY_SYSTHESIS_RX_MODE:
            data = (read & 0xF8) | 0x04;
			_LoRa->current_mode = FREQUENCY_SYSTHESIS_RX_MODE;
            break;
        case RXCONTINUOUS_MODE:
            data = (read & 0xF8) | 0x05;
            _LoRa->current_mode = RXCONTINUOUS_MODE;
            break;
        case RXSINGLE_MODE:
            data = (read & 0xF8) | 0x06;
		    _LoRa->current_mode = RXSINGLE_MODE;
            break;
        case CAD_MODE:
            data = (read & 0xF8) | 0x07;
		    _LoRa->current_mode = CAD_MODE;
            break;
        default:
            dev_warn(&_LoRa->spi->dev, "Chế độ không hợp lệ: %d\n", mode);
            return;
    }
	LoRa_write(_LoRa, RegOpMode, data);
    usleep(1000);
	return;
}

static void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value){
    uint8_t	data;
	uint8_t	read;
	read = LoRa_read(_LoRa, RegModemConfig3);
	if(value) data = read | 0x08;
	else data = read & 0xF7;
	LoRa_write(_LoRa, RegModemConfig3, data);
    usleep(1000);
	return;
}

static void LoRa_setAutoLDO(LoRa* _LoRa){
    double BW[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125.0, 250.0, 500.0};
	LoRa_setLowDaraRateOptimization(_LoRa, (long)((1 << _LoRa->spredingFactor) / ((double)BW[_LoRa->bandWidth])) > 16.0);
}

static void LoRa_setFrequency(LoRa* _LoRa, int freq){
    uint8_t  data;
	uint32_t F;
	F = (freq * 524288)>>5;

	// write Msb:
	data = F >> 16;
	LoRa_write(_LoRa, RegFrMsb, data);
	usleep(1000);

	// write Mid:
	data = F >> 8;
	LoRa_write(_LoRa, RegFrMid, data);
	usleep(1000);

	// write Lsb:
	data = F >> 0;
	LoRa_write(_LoRa, RegFrLsb, data);
	usleep(1000);
}

static void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF){
	uint8_t	data;
	uint8_t	read;

	if(SF>12)
		SF = 12;
	if(SF<7)
		SF = 7;

	read = LoRa_read(_LoRa, RegModemConfig2);
	HAL_Delay(10);

	data = (SF << 4) + (read & 0x0F);
	LoRa_write(_LoRa, RegModemConfig2, data);
	HAL_Delay(10);

	LoRa_setAutoLDO(_LoRa);
}

static void LoRa_setPower(LoRa* _LoRa, uint8_t power){
    LoRa_write(_LoRa, RegPaConfig, power);
    usleep(1000);
}

static void LoRa_setOCP(LoRa* _LoRa, uint8_t ocp_current){
    uint8_t	OcpTrim = 0;

	if(ocp_current < 45) ocp_current = 45;
	if(ocp_current>240) ocp_current = 240;

	if(ocp_current <= 120)
		OcpTrim = (ocp_current - 45)/5;
	else if(ocp_current <= 240)
		OcpTrim = (ocp_current + 30)/10;

	OcpTrim = OcpTrim + (1 << 5);
	LoRa_write(_LoRa, RegOcp, OcpTrim);
    usleep(1000);
}

static void LoRa_setTOMsb_setCRCon(LoRa* _LoRa){
    uint8_t read, data;
	read = LoRa_read(_LoRa, RegModemConfig2);
	data = read | 0x07;
	LoRa_write(_LoRa, RegModemConfig2, data);
    usleep(1000);
}

static void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword){
    LoRa_write(_LoRa, RegSyncWord, syncword);
    usleep(1000);
}

static int LoRa_getRSSI(LoRa* _LoRa){
    uint8_t read;
	read = LoRa_read(_LoRa, RegPktRssiValue);
    usleep(1000);
	return -164 + read;
}

uint16_t LoRa_init(LoRa* _LoRa){
    uint8_t    data;
	uint8_t    read;

	if(LoRa_isvalid(_LoRa)){
		// goto sleep mode:
			LoRa_gotoMode(_LoRa, SLEEP_MODE);
			usleep(1000);

		// turn on LoRa mode:
			read = LoRa_read(_LoRa, RegOpMode);
			usleep(1000);
			data = read | 0x80;
			LoRa_write(_LoRa, RegOpMode, data);
			usleep(1000);

		// set frequency:
			LoRa_setFrequency(_LoRa, _LoRa->frequency);

		// set output power gain:
			LoRa_setPower(_LoRa, _LoRa->power);

		// set over current protection:
			LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);

		// set LNA gain:
			LoRa_write(_LoRa, RegLna, 0x23);

		// set spreading factor, CRC on, and Timeout Msb:
			LoRa_setTOMsb_setCRCon(_LoRa);
			LoRa_setSpreadingFactor(_LoRa, _LoRa->spredingFactor);

		// set Timeout Lsb:
			LoRa_write(_LoRa, RegSymbTimeoutLsb, 0xFF);

		// set bandwidth, coding rate and expilicit mode:
			// 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
			//       bits represent --> |   bandwidth   |     CR    |I/E|
			data = 0;
			data = (_LoRa->bandWidth << 4) + (_LoRa->crcRate << 1);
			LoRa_write(_LoRa, RegModemConfig1, data);
			LoRa_setAutoLDO(_LoRa);

		// set preamble:
			LoRa_write(_LoRa, RegPreambleMsb, _LoRa->preamble >> 8);
			LoRa_write(_LoRa, RegPreambleLsb, _LoRa->preamble >> 0);

		// DIO mapping:   --> DIO: RxDone
			read = LoRa_read(_LoRa, RegDioMapping1);
			data = read | 0x3F;
			LoRa_write(_LoRa, RegDioMapping1, data);

		// goto standby mode:
			LoRa_gotoMode(_LoRa, STDBY_MODE);
			_LoRa->current_mode = STDBY_MODE;
			HAL_Delay(10);

			read = LoRa_read(_LoRa, RegVersion);
			if(read == 0x12)
				return LORA_OK;
			else
				return LORA_NOT_FOUND;
	}
	else {
        return LORA_UNAVAILABLE;
	}
}

static uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout){
    uint8_t read;
	int mode = _LoRa->current_mode;
	LoRa_gotoMode(_LoRa, STDBY_MODE);
	read = LoRa_read(_LoRa, RegFiFoTxBaseAddr);
	LoRa_write(_LoRa, RegFiFoAddPtr, read);
	if (length > MAX_FIFO_SIZE) length = MAX_FIFO_SIZE;
	LoRa_write(_LoRa, RegPayloadLength, length);
	LoRa_burstWrite(_LoRa, RegFiFo, data, length);
	LoRa_gotoMode(_LoRa, TRANSMIT_MODE);

    while(1){
        read = LoRa_read(_LoRa, RegIrqFlags);
        if((read & 0x08)!=0){
            LoRa_write(_LoRa, RegIrqFlags, 0xFF);
            LoRa_gotoMode(_LoRa, mode);
            return 0;
        }
        else{
            if(--timeout==0){
                LoRa_gotoMode(_LoRa, mode);
                return 1;
            }
        }
        usleep(1000);
    }
}

void LoRa_startReceiving(LoRa* _LoRa){
    LoRa_gotoMode(_LoRa, RXCONTINUOUS_MODE);
}

uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length){
    uint8_t read;
	uint8_t number_of_bytes;
	uint8_t min = 0;

	for(int i=0; i<length; i++)
		data[i]=0;

	LoRa_gotoMode(_LoRa, STDBY_MODE);
	read = LoRa_read(_LoRa, RegIrqFlags);
	if((read & 0x40) != 0){
		LoRa_write(_LoRa, RegIrqFlags, 0xFF);
		number_of_bytes = LoRa_read(_LoRa, RegRxNbBytes);
		read = LoRa_read(_LoRa, RegFiFoRxCurrentAddr);
		LoRa_write(_LoRa, RegFiFoAddPtr, read);
		min = length >= number_of_bytes ? number_of_bytes : length;
		for(int i=0; i<min; i++)
			data[i] = LoRa_read(_LoRa, RegFiFo);
	}
	LoRa_gotoMode(_LoRa, RXCONTINUOUS_MODE);
    return min;
}

static long sx1278_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    LoRa *lora = file->private_data;
    struct lora_config conf;
    struct lora_packet pkt;

    switch (cmd) {
        case LORA_IOC_RESET:
            /* Trigger the hardware reset sequence */
            LoRa_reset(lora);
            break;

        case LORA_IOC_CONFIG:
            /* Copy configuration from user-space */
            if (copy_from_user(&conf, (struct lora_config __user *)arg, sizeof(conf)))
                return -EFAULT;
            
            /* Apply settings to SX1278 registers */
            lora->frequency = conf.frequency;
            // LoRa_init(lora); 
            break;

        case LORA_IOC_TRANSMIT:
            /* Copy data to be sent from user-space */
            if (copy_from_user(&pkt, (struct lora_packet __user *)arg, sizeof(pkt)))
                return -EFAULT;
            
            /* Send data via LoRa */
            // LoRa_transmit(lora, pkt.payload, pkt.size);
            break;

        case LORA_IOC_RECEIVE:
            /* Get received data from driver buffer */
            // pkt.size = LoRa_receive(lora, pkt.payload);
            
            /* Copy received data back to user-space */
            if (copy_to_user((struct lora_packet __user *)arg, &pkt, sizeof(pkt)))
                return -EFAULT;
            break;

        default:
            return -ENOTTY; /* Command not supported */
    }
    return 0;
}


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

    lora->buffer = devm_kzalloc(&spi->dev, MAX_FIFO_SIZE, GFP_KERNEL);
    if (!lora->buffer) return -ENOMEM;

    mutex_init(&lora->lock);

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

    dev_info(&spi->dev, "SX1278 LoRa driver probed successfully\n");
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