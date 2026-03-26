#include <linux/module.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/acpi.h>
#include <linux/of_device.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>   
#include <linux/sched.h> 
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>


#include "sx1278_ioctl.h"

#define DEVICE_NAME "sx1278"
#define CLASS_NAME  "sx1278_class"

#define MAX_FIFO_SIZE 255
 
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

typedef struct LoRa_setting {
    // Hardware settings 
    struct spi_device *spi;      
    struct gpio_desc  *dio0_pin;    
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
    uint8_t  spreadingFactor;
    uint8_t  bandWidth;
    uint8_t  crcRate;
    uint16_t preamble;
    uint8_t  power;
    uint8_t  overCurrentProtection;

    /* Cơ chế Blocking Read */
    wait_queue_head_t rx_wait;    // Hàng đợi để App "đi ngủ"
    bool data_ready;              // Cờ báo: True = Có hàng, False = Đang đợi
    int irq;                      // Số hiệu ngắt hệ thống
} LoRa;

static void LoRa_reset(LoRa *lora);
static uint8_t LoRa_isvalid(LoRa *lora);
static uint8_t LoRa_read(LoRa *lora, uint8_t address);
static int LoRa_write(LoRa *_LoRa, uint8_t address, uint8_t value);
static int LoRa_burstWrite(LoRa *_LoRa, uint8_t address, uint8_t *value, uint8_t length);
static int LoRa_burstRead(LoRa *_LoRa, uint8_t address, uint8_t *value, uint8_t length);
static void LoRa_gotoMode(LoRa* _LoRa, int mode);
static void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value);
static void LoRa_setAutoLDO(LoRa* _LoRa);
static void LoRa_setFrequency(LoRa* _LoRa, int freq);
static void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF);
static void LoRa_setPower(LoRa* _LoRa, uint8_t power);
static void LoRa_setOCP(LoRa* _LoRa, uint8_t ocp_current);
static void LoRa_setTOMsb_setCRCon(LoRa* _LoRa);
static void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword);
static int LoRa_getRSSI(LoRa* _LoRa);
static uint16_t LoRa_init(LoRa* _LoRa);
static void LoRa_startReceiving(LoRa* _LoRa);


static void LoRa_reset(LoRa *lora) {
    gpiod_set_value(lora->reset_pin, 1); 
    mdelay(10); 

    gpiod_set_value(lora->reset_pin, 0); 
    mdelay(10);
}

static uint8_t LoRa_isvalid(LoRa *lora)
{
    /* 1. Frequency Validation (Numerical Range)
     * Supporting 137MHz to 525MHz for SX1278. 
     * Ensure frequency is in KHz (e.g., 433).
     */
    if (lora->frequency < 137 || lora->frequency > 525) {
        dev_err(&lora->spi->dev, "Invalid Frequency: %u Hz\n", lora->frequency);
        return 0;
    }

    /* 2. Bandwidth Validation (Strict Macro Check) */
    switch (lora->bandWidth) {
        case BW_7_8KHz:  case BW_10_4KHz: case BW_15_6KHz:
        case BW_20_8KHz: case BW_31_25KHz: case BW_41_7KHz:
        case BW_62_5KHz: case BW_125KHz:  case BW_250KHz:
        case BW_500KHz:
            break; // Valid
        default:
            dev_err(&lora->spi->dev, "Invalid BW constant: %u\n", lora->bandWidth);
            return 0;
    }

    /* 3. Coding Rate Validation (Strict Macro Check) */
    switch (lora->crcRate) {
        case CR_4_5: case CR_4_6: case CR_4_7: case CR_4_8:
            break; // Valid
        default:
            dev_err(&lora->spi->dev, "Invalid CR constant: %u\n", lora->crcRate);
            return 0;
    }

    /* 4. Spreading Factor Validation (Strict Macro Check) */
    switch (lora->spreadingFactor) {
        case SF_7: case SF_8: case SF_9:
        case SF_10: case SF_11: case SF_12:
            break; // Valid
        default:
            dev_err(&lora->spi->dev, "Invalid SF constant: %u\n", lora->spreadingFactor);
            return 0;
    }

    /* 5. Power Gain Validation (Strict Macro Check) 
     * Matching hex values: 0xF6, 0xF9, 0xFC, 0xFF
     */
    switch (lora->power) {
        case POWER_11db: case POWER_14db: 
        case POWER_17db: case POWER_20db:
            break; // Valid
        default:
            dev_err(&lora->spi->dev, "Invalid Power Gain constant: 0x%02X\n", lora->power);
            return 0;
    }

    /* 6. OCP & Preamble (Numerical Range) */
    if (lora->overCurrentProtection < 45 || lora->overCurrentProtection > 240) {
        dev_err(&lora->spi->dev, "Invalid OCP: %u mA\n", lora->overCurrentProtection);
        return 0;
    }

    if (lora->preamble < 6) {
        dev_err(&lora->spi->dev, "Preamble too short: %u\n", lora->preamble);
        return 0;
    }

    return 1; // Success
}

static uint8_t LoRa_read(LoRa *lora, uint8_t addr)
{
    uint8_t tx = addr & 0x7F; 
    uint8_t rx = 0xFF;
    int ret;
    ret = spi_write_then_read(lora->spi, &tx, 1, &rx, 1);
    if (ret < 0) {
        dev_err(&lora->spi->dev, "SPI read failed at 0x%02X: %d\n", addr, ret);
        return 0xFF;
    }

    return rx;
}

static int LoRa_write(LoRa *lora, uint8_t addr, uint8_t val)
{
    uint8_t tx[2];

    tx[0] = addr | 0x80; // Bit 7 = 1 d? ghi
    tx[1] = val;

    // Ghi 2 byte: [Address | Data]
    return spi_write(lora->spi, tx, 2);
}

static int LoRa_burstWrite(LoRa *lora, uint8_t addr, uint8_t *buf, uint8_t len)
{
    int ret;

    // Ki?m tra gi?i h?n d? không tràn buffer dã c?p phát trong struct LoRa
    if (len > MAX_FIFO_SIZE - 1) len = MAX_FIFO_SIZE - 1;

    lora->buffer[0] = addr | 0x80;
    memcpy(&lora->buffer[1], buf, len);

    ret = spi_write(lora->spi, lora->buffer, len + 1);
    return ret;
}

static int LoRa_burstRead(LoRa *lora, uint8_t addr, uint8_t *buf, uint8_t len)
{
    uint8_t tx = addr & 0x7F;
    return spi_write_then_read(lora->spi, &tx, 1, buf, len);
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
            dev_warn(&_LoRa->spi->dev, "Ch? d? không h?p l?: %d\n", mode);
            return;
    }
	LoRa_write(_LoRa, RegOpMode, data);
    udelay(1000);
	return;
}

static void LoRa_setLowDaraRateOptimization(LoRa* _LoRa, uint8_t value){
    uint8_t	data;
	uint8_t	read;
	read = LoRa_read(_LoRa, RegModemConfig3);
	if(value) data = read | 0x08;
	else data = read & 0xF7;
	LoRa_write(_LoRa, RegModemConfig3, data);
    udelay(1000);
	return;
}

static void LoRa_setAutoLDO(LoRa* _LoRa){
    uint32_t BW[] = {
        7800,    // 7.8 kHz
        10400,   // 10.4 kHz
        15600,   // 15.6 kHz
        20800,   // 20.8 kHz
        31250,   // 31.25 kHz
        41700,   // 41.7 kHz
        62500,   // 62.5 kHz
        125000,  // 125 kHz
        250000,  // 250 kHz
        500000   // 500 kHz
    };
    uint32_t symbol = (1 << _LoRa->spreadingFactor);
    // (2^SF / BW) > 16  <=>  2^SF > 16 * BW
    uint8_t ldo_on = symbol > (16 * BW[_LoRa->bandWidth]);
    LoRa_setLowDaraRateOptimization(_LoRa, ldo_on);
}

static void LoRa_setFrequency(LoRa* _LoRa, int freq){
    uint8_t  data;
	uint32_t F;
	F = (freq * 524288)>>5;

	// write Msb:
	data = F >> 16;
	LoRa_write(_LoRa, RegFrMsb, data);
	// write Mid:
	data = F >> 8;
	LoRa_write(_LoRa, RegFrMid, data);
	// write Lsb:
	data = F >> 0;
	LoRa_write(_LoRa, RegFrLsb, data);

}

static void LoRa_setSpreadingFactor(LoRa* _LoRa, int SF){
	uint8_t	data;
	uint8_t	read;

	if(SF>12)
		SF = 12;
	if(SF<7)
		SF = 7;

	read = LoRa_read(_LoRa, RegModemConfig2);

	data = (SF << 4) + (read & 0x0F);
	LoRa_write(_LoRa, RegModemConfig2, data);

	LoRa_setAutoLDO(_LoRa);
}

static void LoRa_setPower(LoRa* _LoRa, uint8_t power){
    LoRa_write(_LoRa, RegPaConfig, power);
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
}

static void LoRa_setTOMsb_setCRCon(LoRa* _LoRa){
    uint8_t read, data;
	read = LoRa_read(_LoRa, RegModemConfig2);
	data = read | 0x07;
	LoRa_write(_LoRa, RegModemConfig2, data);
}

static void LoRa_setSyncWord(LoRa* _LoRa, uint8_t syncword){
    LoRa_write(_LoRa, RegSyncWord, syncword);
}

static int LoRa_getRSSI(LoRa* _LoRa){
    uint8_t read;
	read = LoRa_read(_LoRa, RegRssiValue);
	return -164 + read;
}

static void LoRa_enable_rx_crc(LoRa *lora) {
    uint8_t config2;
    config2 = LoRa_read(lora, RegModemConfig2);
    config2 |= 0x04;
    LoRa_write(lora, RegModemConfig2, config2);
}


static uint16_t LoRa_init(LoRa* _LoRa){
    uint8_t data, read;
    uint8_t v_freq_msb, v_freq_mid, v_freq_lsb;
    uint16_t v_preamble;
    uint8_t v_config1, v_config2, v_sync, v_dio;

    if(LoRa_isvalid(_LoRa)){
        // 1. Reset & Mode Setup
        LoRa_gotoMode(_LoRa, SLEEP_MODE);
        read = LoRa_read(_LoRa, RegOpMode);
        LoRa_write(_LoRa, RegOpMode, read | 0x80); // Chế độ LoRa

        // 2. Set các thông số chính
        LoRa_setFrequency(_LoRa, _LoRa->frequency);
        LoRa_setPower(_LoRa, _LoRa->power);
        LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);
        LoRa_write(_LoRa, RegLna, 0x23);

        // 3. Modem Configuration (SF, CRC, Timeout, BW, CR)
        LoRa_setTOMsb_setCRCon(_LoRa);
        LoRa_setSpreadingFactor(_LoRa, _LoRa->spreadingFactor);
        LoRa_write(_LoRa, RegSymbTimeoutLsb, 0xFF);

        data = (_LoRa->bandWidth << 4) + (_LoRa->crcRate << 1);
        LoRa_write(_LoRa, RegModemConfig1, data);
        LoRa_setAutoLDO(_LoRa);

        // 4. Preamble & Sync Word
        LoRa_write(_LoRa, RegPreambleMsb, _LoRa->preamble >> 8);
        LoRa_write(_LoRa, RegPreambleLsb, _LoRa->preamble & 0xFF);
        LoRa_write(_LoRa, 0x39, 0xF1); // Force Sync Word 0xF1

        // 5. DIO Mapping & Clean Up
        read = LoRa_read(_LoRa, RegDioMapping1);
        LoRa_write(_LoRa, RegDioMapping1, 0x00); 
        LoRa_write(_LoRa, RegIrqFlags, 0xFF);
        LoRa_enable_rx_crc(_LoRa);

        LoRa_gotoMode(_LoRa, STDBY_MODE);

        // =========================================================
        // PHẦN ĐỌC LẠI ĐỂ KIỂM TRA (VERIFICATION)
        // =========================================================
        dev_info(&_LoRa->spi->dev, "--- SX1278 REGISTER VERIFICATION ---\n");

        // Đọc lại Frequency (RegFrMsb 0x06, Mid 0x07, Lsb 0x08)
        v_freq_msb = LoRa_read(_LoRa, 0x06);
        v_freq_mid = LoRa_read(_LoRa, 0x07);
        v_freq_lsb = LoRa_read(_LoRa, 0x08);
        dev_info(&_LoRa->spi->dev, "[CHECK] Freq Regs: 0x%02X 0x%02X 0x%02X\n", v_freq_msb, v_freq_mid, v_freq_lsb);

        // Đọc lại Preamble (RegPreambleMsb 0x20, Lsb 0x21)
        v_preamble = (LoRa_read(_LoRa, 0x20) << 8) | LoRa_read(_LoRa, 0x21);
        dev_info(&_LoRa->spi->dev, "[CHECK] Preamble: %d (Target: %d)\n", v_preamble, _LoRa->preamble);

        // Đọc lại Modem Config 1 & 2 (BW, CR, SF, CRC)
        v_config1 = LoRa_read(_LoRa, 0x1D);
        v_config2 = LoRa_read(_LoRa, 0x1E);
        dev_info(&_LoRa->spi->dev, "[CHECK] Config1(0x1D): 0x%02X | Config2(0x1E): 0x%02X\n", v_config1, v_config2);

        // Đọc lại Sync Word (0x39)
        v_sync = LoRa_read(_LoRa, 0x39);
        dev_info(&_LoRa->spi->dev, "[CHECK] Sync Word: 0x%02X (Target: 0xF1)\n", v_sync);

        // Đọc lại DIO Mapping
        v_dio = LoRa_read(_LoRa, RegDioMapping1);
        dev_info(&_LoRa->spi->dev, "[CHECK] DIO0 Mapping: %s\n", (v_dio >> 6 == 0) ? "RxDone (OK)" : "WRONG");

        // Cuối cùng check Version
        read = LoRa_read(_LoRa, RegVersion);
        dev_info(&_LoRa->spi->dev, "[CHECK] Silicon Version: 0x%02X\n", read);
        dev_info(&_LoRa->spi->dev, "------------------------------------\n");

        return (read == 0x12) ? LORA_OK : LORA_NOT_FOUND;
    }
    return LORA_UNAVAILABLE;
}

// static uint16_t LoRa_init(LoRa* _LoRa){
//     uint8_t    data;
// 	uint8_t    read;

// 	if(LoRa_isvalid(_LoRa)){
// 		// goto sleep mode:
// 			LoRa_gotoMode(_LoRa, SLEEP_MODE);

// 		// turn on LoRa mode:
// 			read = LoRa_read(_LoRa, RegOpMode);
// 			data = read | 0x80;
// 			LoRa_write(_LoRa, RegOpMode, data);

// 		// set frequency:
// 			LoRa_setFrequency(_LoRa, _LoRa->frequency);

// 		// set output power gain:
// 			LoRa_setPower(_LoRa, _LoRa->power);

// 		// set over current protection:
// 			LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);

// 		// set LNA gain:
// 			LoRa_write(_LoRa, RegLna, 0x23);

// 		// set spreading factor, CRC on, and Timeout Msb:
// 			LoRa_setTOMsb_setCRCon(_LoRa);
// 			LoRa_setSpreadingFactor(_LoRa, _LoRa->spreadingFactor);

// 		// set Timeout Lsb:
// 			LoRa_write(_LoRa, RegSymbTimeoutLsb, 0xFF);

// 		// set bandwidth, coding rate and expilicit mode:
// 			// 8 bit RegModemConfig --> | X | X | X | X | X | X | X | X |
// 			//       bits represent --> |   bandwidth   |     CR    |I/E|
// 			data = 0;
// 			data = (_LoRa->bandWidth << 4) + (_LoRa->crcRate << 1);
// 			LoRa_write(_LoRa, RegModemConfig1, data);
// 			LoRa_setAutoLDO(_LoRa);

// 		// set preamble:
//             LoRa_write(_LoRa, RegPreambleMsb, _LoRa->preamble >> 8);
//             LoRa_write(_LoRa, RegPreambleLsb, _LoRa->preamble >> 0);

//         // DIO mapping: DIO0 -> RxDone (00)
//             read = LoRa_read(_LoRa, RegDioMapping1);
//             data = (read & 0x3F); // Ép DIO0 về 00 (RxDone)
//             LoRa_write(_LoRa, RegDioMapping1, data);
//             LoRa_write(_LoRa, RegIrqFlags, 0xFF);

//             LoRa_enable_rx_crc(_LoRa);

//         // goto standby mode:
//             LoRa_gotoMode(_LoRa, STDBY_MODE);
//             _LoRa->current_mode = STDBY_MODE;

//         // Check Version cuối cùng
//             read = LoRa_read(_LoRa, RegVersion);
//             return (read == 0x12) ? LORA_OK : LORA_NOT_FOUND;
// 	}
// 	else {
//         return LORA_UNAVAILABLE;
// 	}
// }

static uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint16_t length, uint16_t timeout){
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
    }
}


static irqreturn_t sx1278_irq_thread_fn(int irq, void *dev_id)
{
    LoRa *lora = dev_id;

    // 1. Đánh dấu có dữ liệu mới
    lora->data_ready = true;

    // 2. Đánh thức App đang đợi ở wait_event_interruptible
    wake_up_interruptible(&lora->rx_wait);
    
    dev_info(&lora->spi->dev, "Interrupt DIO0 triggered!\n");

    return IRQ_HANDLED;
}

static int LoRa_receive_continue(LoRa *lora, struct lora_packet *pkt)
{
    uint8_t irq_flags, num_bytes, fifo_addr;
    int i;

    // 1. Reset cờ phần mềm TRƯỚC khi ngủ để đảm bảo không bị thức giấc giả
    lora->data_ready = false;

    // 2. Kích hoạt chế độ nghe
    mutex_lock(&lora->lock);
    LoRa_gotoMode(lora, RXCONTINUOUS_MODE);
    // Xóa sạch cờ cũ trên chip để DIO0 về mức Thấp (chuẩn bị cho cạnh lên mới)
    LoRa_write(lora, RegIrqFlags, 0xFF); 
    mutex_unlock(&lora->lock);

    // 3. ĐI NGỦ (Wait for DIO0 Rising Edge)
    if (wait_event_interruptible(lora->rx_wait, lora->data_ready)) {
        // Nếu bị Ctrl+C thì đưa chip về Sleep cho tiết kiệm điện
        mutex_lock(&lora->lock);
        LoRa_gotoMode(lora, SLEEP_MODE);
        mutex_unlock(&lora->lock);
        return -ERESTARTSYS;
    }

    // 4. TỈNH DẬY: Xử lý dữ liệu
    mutex_lock(&lora->lock);
    
    // Đọc cờ ngắt TRƯỚC khi chuyển sang mode khác
    irq_flags = LoRa_read(lora, RegIrqFlags);
    
    // Log để debug xem chip báo gì (RxDone=0x40, PayloadCrcError=0x20)
    dev_info(&lora->spi->dev, "Wake up! IRQ Flags: 0x%02x\n", irq_flags);

    if (irq_flags & 0x40) { // RxDone bit
        // Kiểm tra lỗi CRC (Nếu bị nhiễu sóng)
        if (irq_flags & 0x20) {
            dev_warn(&lora->spi->dev, "CRC Error detected! Packet might be corrupted.\n");
        }

        num_bytes = LoRa_read(lora, RegRxNbBytes);
        pkt->length = (num_bytes > 256) ? 256 : num_bytes;
        
        fifo_addr = LoRa_read(lora, RegFiFoRxCurrentAddr);
        LoRa_write(lora, RegFiFoAddPtr, fifo_addr);
        
        for (i = 0; i < pkt->length; i++) {
            pkt->data[i] = LoRa_read(lora, RegFiFo);
        }

        // Xóa cờ ngắt ngay sau khi đọc xong để chân DIO0 hạ xuống mức Thấp
        LoRa_write(lora, RegIrqFlags, 0xFF);
    } else {
        pkt->length = 0;
        dev_info(&lora->spi->dev, "Spurious wakeup: No RxDone flag found.\n");
    }

    // Đưa về Standby để chuẩn bị cho chu kỳ ioctl tiếp theo
    LoRa_gotoMode(lora, RXCONTINUOUS_MODE);
    mutex_unlock(&lora->lock);

    return 0;
}

static long sx1278_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
    LoRa *lora = file->private_data;
    struct lora_packet pkt;
    int val_int;
    uint8_t val8;
    int ret = 0;

    if (!lora) return -EINVAL;

    if (mutex_lock_interruptible(&lora->lock))
        return -ERESTARTSYS;

    switch (cmd) {
        case LORA_IOC_RESET:
            LoRa_reset(lora);
            dev_info(&lora->spi->dev, "IOCTL: Hardware Reset performed\n");
            break;

        case LORA_IOC_SET_FREQ:
            if (copy_from_user(&val_int, (int __user *)arg, sizeof(int))) {
                ret = -EFAULT; goto out;
            }
            LoRa_setFrequency(lora, val_int);
            dev_info(&lora->spi->dev, "IOCTL: Frequency set to %d Hz\n", val_int);
            break;

        case LORA_IOC_SET_SF:
            if (copy_from_user(&val_int, (int __user *)arg, sizeof(int))) {
                ret = -EFAULT; goto out;
            }
            LoRa_setSpreadingFactor(lora, val_int);
            dev_info(&lora->spi->dev, "IOCTL: Spreading Factor set to %d\n", val_int);
            break;

        case LORA_IOC_SET_POWER:
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(uint8_t))) {
                ret = -EFAULT; goto out;
            }
            LoRa_setPower(lora, val8);
            dev_info(&lora->spi->dev, "IOCTL: Transmit Power set to %d dBm\n", val8);
            break;

        case LORA_IOC_SET_OCP:
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(uint8_t))) {
                ret = -EFAULT; goto out;
            }
            LoRa_setOCP(lora, val8);
            dev_info(&lora->spi->dev, "IOCTL: Over Current Protection set to %d mA\n", val8);
            break;

        case LORA_IOC_TRANSMIT:
            if (copy_from_user(&pkt, (struct lora_packet __user *)arg, sizeof(pkt))) {
                ret = -EFAULT; goto out;
            }
            if (pkt.length > 256) pkt.length = 256; 
            LoRa_transmit(lora, pkt.data, pkt.length, 1000);
            dev_info(&lora->spi->dev, "IOCTL: Transmitted %d bytes\n", pkt.length);
            break;

        case LORA_IOC_GET_RSSI: {
            int rssi_val = LoRa_getRSSI(lora);
            if (copy_to_user((int __user *)arg, &rssi_val, sizeof(int))) {
                ret = -EFAULT; goto out;
            }
            // Log này có thể hơi nhiễu nếu App gọi liên tục, nhưng để debug thì rất tốt
            dev_info(&lora->spi->dev, "IOCTL: Current RSSI is %d dBm\n", rssi_val);
            break;
        }

        case LORA_IOC_RECEIVE_CONTINUE: {
            mutex_unlock(&lora->lock);
            ret = LoRa_receive_continue(lora, &pkt);
            
            if (mutex_lock_interruptible(&lora->lock)) {
                return -ERESTARTSYS; 
            }

            if (ret) {
                // Chỉ log khi thực sự có lỗi hoặc bị ngắt, tránh spam log "Sleep" liên tục
                if (ret != -ERESTARTSYS)
                    dev_err(&lora->spi->dev, "IOCTL: Receive Continuous failed with error %d\n", ret);
                goto out;
            }

            if (copy_to_user((struct lora_packet __user *)arg, &pkt, sizeof(struct lora_packet))) {
                ret = -EFAULT;
                goto out;
            }
            
            dev_info(&lora->spi->dev, "IOCTL: Received packet, length: %d bytes\n", pkt.length);
            break;
        }

        case LORA_IOC_SET_SYNC_WORD:{
            // 1. Copy giá trị từ người dùng
            if (copy_from_user(&val8, (uint8_t __user *)arg, sizeof(uint8_t))) {
                ret = -EFAULT;
                goto out;
            }
            LoRa_write(lora, RegSyncWord, val8); 
            
            dev_info(&lora->spi->dev, "IOCTL: Sync Word set to 0x%02X\n", val8);
            break;
        }

        default:
            dev_warn(&lora->spi->dev, "IOCTL: Unknown command 0x%x\n", cmd);
            ret = -ENOTTY;
            break;
    }

out:
    mutex_unlock(&lora->lock);
    return ret;
}

static int sx1278_open(struct inode *inode, struct file *file)
{
    LoRa *lora = container_of(inode->i_cdev, LoRa, lora_cdev);
    int ret;

    file->private_data = lora;

    dev_info(&lora->spi->dev, "Device opened successfully\n");
    return 0;
}

static int sx1278_release(struct inode *inode, struct file *file)
{
    LoRa *lora = file->private_data;
    LoRa_gotoMode(lora, STDBY_MODE);
    dev_info(&lora->spi->dev, "Device closed\n");
    return 0;
}

static const struct file_operations sx1278_fops = {
    .owner          = THIS_MODULE,
    .open           = sx1278_open,
    .release        = sx1278_release,
    .unlocked_ioctl = sx1278_ioctl,
};

// DRIVER DECLERATION
static int sx1278_probe(struct spi_device *spi) 
{
    int ret;
    uint8_t version;
    LoRa* lora;

    // 1. SPI Config
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret) {
        return dev_err_probe(&spi->dev, ret, "Setup SPI failed!\n");
    }

    // 2. Kh?i t?o c?u trúc
    lora = devm_kzalloc(&spi->dev, sizeof(*lora), GFP_KERNEL);
    if (!lora) return -ENOMEM;

    lora->spi = spi;
    mutex_init(&lora->lock);
    spi_set_drvdata(spi, lora);

    // 3. L?y chân Reset và Interrupt (DIO0) t? Device Tree
    lora->reset_pin = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH); // Ð? High d? không b? reset liên t?c
    if (IS_ERR(lora->reset_pin)) {
        return dev_err_probe(&spi->dev, PTR_ERR(lora->reset_pin), "Failed to get reset GPIO\n");
    }

    lora->dio0_pin = devm_gpiod_get(&spi->dev, "dio0", GPIOD_IN); 
    if (IS_ERR(lora->dio0_pin)) {
        return dev_err_probe(&spi->dev, PTR_ERR(lora->dio0_pin), "Failed to get DIO0 GPIO\n");
    }

    lora->irq = gpiod_to_irq(lora->dio0_pin);
    if (lora->irq < 0) {
        dev_err(&spi->dev, "Failed to register IRQ: %d\n", ret);
        return lora->irq;
    }

    // NULL: Không dùng Top-half (vì mình xử lý hết trong Thread-fn)
    // IRQF_TRIGGER_RISING: Kích hoạt khi chân DIO0 nhảy từ 0 lên 1 (Rising Edge)
    // IRQF_ONESHOT: Quan trọng! Kernel sẽ giữ ngắt này lại cho đến khi thread_fn chạy xong
    // ret = devm_request_threaded_irq(&spi->dev, 
    //                                 lora->irq, 
    //                                 NULL, 
    //                                 sx1278_irq_thread_fn, 
    //                                 IRQF_TRIGGER_RISING | IRQF_ONESHOT, 
    //                                 "sx1278_irq", 
    //                                 lora);
                                

    ret = devm_request_threaded_irq(&spi->dev, 
                                    lora->irq, 
                                    NULL, 
                                    sx1278_irq_thread_fn, 
                                    IRQF_ONESHOT, // Không ép kiểu trigger ở đây nữa
                                    "sx1278_irq", 
                                    lora); 

    if (ret) {
        dev_err(&spi->dev, "Failed to register IRQ: %d\n", ret);
        return ret;
    }

    // 3. Khởi tạo hàng đợi chờ
    init_waitqueue_head(&lora->rx_wait);
    lora->data_ready = false;

    pr_info("sx1278: IRQ %d (GPIO 27) has successfully registered.!\n", lora->irq);

    // 4. Reset cứng module truơc khi Init 
    gpiod_set_value(lora->reset_pin, 0);
    msleep(10);
    gpiod_set_value(lora->reset_pin, 1);
    msleep(10);

    // 5. Ki?m tra(Version Check)
    version = LoRa_read(lora, RegVersion);
    if (version != 0x12) {
        dev_err(&spi->dev, "Failed to detected SX1278  (Version: 0x%02x, mong d?i: 0x12)\n", version);
        return -ENODEV;
    }
    dev_info(&spi->dev, "Detected SX1278 successfully, Version: 0x%02x\n", version);

    // 6. Hardware Init
    lora->frequency             = 434; // 433 MHz
    lora->spreadingFactor       = SF_7;
    lora->bandWidth             = BW_125KHz;
    lora->crcRate               = CR_4_5;
    lora->power                 = POWER_20db;
    lora->overCurrentProtection = 120;
    lora->preamble              = 10;

    ret = LoRa_init(lora);
    if (ret != LORA_OK) {
        return -EIO;
    }

    // 7. Ðang ký Character Device
    ret = alloc_chrdev_region(&lora->dev_num, 0, 1, DEVICE_NAME);
    if (ret < 0) return ret;

    cdev_init(&lora->lora_cdev, &sx1278_fops);
    lora->lora_cdev.owner = THIS_MODULE;
    ret = cdev_add(&lora->lora_cdev, lora->dev_num, 1);
    if (ret < 0) goto unregister_region;

    lora->lora_class = class_create(CLASS_NAME); 
    if (IS_ERR(lora->lora_class)) {
        ret = PTR_ERR(lora->lora_class);
        goto del_cdev;
    }

    lora->lora_device = device_create(lora->lora_class, NULL, lora->dev_num, lora, DEVICE_NAME);
    if (IS_ERR(lora->lora_device)) {
        ret = PTR_ERR(lora->lora_device);
        goto destroy_class;
    }

    dev_info(&spi->dev, "SX1278 LoRa driver probed successfully\n");

    return 0;

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

    // 1. Ðưa chip về Sleep (Cần lock SPI)
    mutex_lock(&lora->lock);
    LoRa_gotoMode(lora, SLEEP_MODE); 
    mutex_unlock(&lora->lock);

    // 2. Hủy Device trước Class (Nguyên tắc: Con trước cha sau)
    if (lora->lora_device) {
        device_destroy(lora->lora_class, lora->dev_num);
    }
    if (!IS_ERR_OR_NULL(lora->lora_class)) {
        class_destroy(lora->lora_class);
    }

    // 3. Hủy Character Device
    cdev_del(&lora->lora_cdev);
    unregister_chrdev_region(lora->dev_num, 1);

    // 4. Hủy Mutex cuối cùng
    mutex_destroy(&lora->lock);

    dev_info(&spi->dev, "SX1278 LoRa driver removed successfully\n");
}

/* 1. B?ng cho Device Tree */
static const struct of_device_id sx1278_dt_ids[] = {
    { .compatible = "semtech,sx1278", },
    { }
};
MODULE_DEVICE_TABLE(of, sx1278_dt_ids);

/* 2. B?ng ID cho SPI subsystem (D? phòng & Autoload) */
static const struct spi_device_id sx1278_ids[] = {
    { "sx1278", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, sx1278_ids);

static struct spi_driver sx1278_driver = {
    .driver = {
        .name = "sx1278",               
        .of_match_table = sx1278_dt_ids,
    },
    .id_table = sx1278_ids, // G?n b?ng ID vào dây
    .probe    = sx1278_probe,
    .remove   = sx1278_remove,           
};
module_spi_driver(sx1278_driver);

MODULE_DESCRIPTION("SX1278 LoRa SPI Driver");
MODULE_AUTHOR("Duy Bach - HCMUT");
MODULE_LICENSE("GPL");
