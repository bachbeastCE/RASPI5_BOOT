// Include
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

// Registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_OCP                  0x0b
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_RSSI_VALUE           0x1b
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_FREQ_ERROR_MSB       0x28
#define REG_FREQ_ERROR_MID       0x29
#define REG_FREQ_ERROR_LSB       0x2a
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_INVERTIQ2            0x3b
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42
#define REG_PA_DAC               0x4d

// Modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define RF_MID_BAND_THRESHOLD    525000000
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255

// Addition
#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1

// Device name
#define DEVICE_NAME "sx1278"
#define CLASS_NAME  "sx1278_class"

//Macro
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))


// LoRa Controller Struct
typedef struct LoRa{
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
    uint16_t length;

    struct mutex lock;

    uint32_t frequency;
    uint8_t txPower;

    int packetIndex;
    int implicitHeaderMode;

    void (*onReceive)(int);
    void (*onTxDone)(void);

    /* Cơ chế Blocking Read */
    wait_queue_head_t rx_wait;    // Hàng đợi để App "đi ngủ"
    bool data_ready;              // Cờ báo: True = Có hàng, False = Đang đợi
    int irq;                      // Số hiệu ngắt hệ thống
} LoRa_t;

typedef struct lora_packet {
    uint8_t  data[256]; 
    uint16_t length;           
} LoRa_packet_t;

typedef struct lora_config {
    uint8_t  current_mode;
    uint32_t frequency;       
    uint8_t  spreadingFactor;
    uint32_t bandWidth;       
    uint8_t  crcRate;
    uint16_t preamble;
    uint8_t  power;
    uint8_t  enableCrc;
    uint8_t  syncWord;          
} LoRa_config_t;

//IOCTL Define
#define LORA_IOC_MAGIC 'L'
#define LORA_IOC_RESET              _IO(LORA_IOC_MAGIC, 0)
#define LORA_IOC_GET_SNR          _IOR(LORA_IOC_MAGIC, 1, int)
#define LORA_IOC_GET_PKT_RSSI     _IOR(LORA_IOC_MAGIC, 2, int)
#define LORA_IOC_GET_CONFIG       _IOR(LORA_IOC_MAGIC, 3, LoRa_config_t)
#define LORA_IOC_SET_CONFIG       _IOW(LORA_IOC_MAGIC, 4, LoRa_config_t)

// Funtion predefine
/* low-level */
static uint8_t LoRa_readRegister(LoRa_t *lora, uint8_t address);
static void LoRa_writeRegister(LoRa_t* lora, uint8_t address, uint8_t value);

/* mode */
static void LoRa_reset(LoRa_t* lora);
static void LoRa_sleep(LoRa_t* lora);
static void LoRa_idle(LoRa_t* lora);

/* config */
static void LoRa_setFrequency(LoRa_t *lora, uint32_t frequency);
static void LoRa_setTxPower(LoRa_t *lora, int level, int outputPin);
static void LoRa_setOCP(LoRa_t *lora, uint8_t mA);
static void LoRa_setGain(LoRa_t *lora, uint8_t gain);
static void LoRa_disableInvertIQ(LoRa_t *lora);
static void LoRa_enableInvertIQ(LoRa_t *lora);
static void LoRa_disableCrc(LoRa_t *lora);
static void LoRa_enableCrc(LoRa_t *lora);
static void LoRa_setSyncWord(LoRa_t *lora, int sw);
static void LoRa_setPreambleLength(LoRa_t *lora, long length);
static void LoRa_setCodingRate4(LoRa_t *lora, int denominator);
static void LoRa_setLdoFlag(LoRa_t *lora);
static void LoRa_setSignalBandwidth(LoRa_t *lora, long sbw);
static void LoRa_setSpreadingFactor(LoRa_t *lora, int sf);

/* get */
static int LoRa_getSpreadingFactor(LoRa_t *lora);
static long LoRa_getSignalBandwidth(LoRa_t *lora);
static int LoRa_getSpreadingFactor(LoRa_t *lora);

/* control */
static bool LoRa_isTransmitting(LoRa_t *lora);
static int LoRa_beginPacket(LoRa_t *lora, int implicitHeader);
static void LoRa_handleDio0Rise(LoRa_t *lora);
static void LoRa_onDio0Rise(LoRa_t *lora);
static void LoRa_dumpRegisters(LoRa_t *lora);
static uint8_t LoRa_random(LoRa_t *lora);
static uint8_t LoRa_peek(LoRa_t *lora);
static int LoRa_available(LoRa_t *lora);
static int LoRa_rssi(LoRa_t *lora);
static long LoRa_packetFrequencyError(LoRa_t *lora);
static int LoRa_packetSnr(LoRa_t *lora);
static int LoRa_packetRssi(LoRa_t *lora);
static int LoRa_parsePacket(LoRa_t *lora, int size);
static int LoRa_read_fifo(LoRa_t *lora);
static size_t LoRa_write_fifo(LoRa_t *lora, const uint8_t *buffer, size_t size);
static inline int LoRa_timedRead(LoRa_t *lora, uint32_t timeout_ms);
static size_t LoRa_readBytes(LoRa_t *lora, uint8_t *buffer, size_t length);

/* driver */
static irqreturn_t LoRa_irq_thread_fn(int irq, void *dev_id);
static int LoRa_probe(struct spi_device *spi);
static void LoRa_remove(struct spi_device *spi);
static int LoRa_release_file(struct inode *inode, struct file *file);
static int LoRa_open_file(struct inode *inode, struct file *file);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t LoRa_readRegister(LoRa_t *lora, uint8_t address)
{
    uint8_t tx = address & 0x7F;
    uint8_t rx;
    spi_write_then_read(lora->spi, &tx, 1, &rx, 1);
    return rx;
}

static void LoRa_writeRegister(LoRa_t *lora, uint8_t address, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = address | 0x80;
    tx[1] = value;
    spi_write(lora->spi, tx, 2); // ignore error
}

static void LoRa_reset(LoRa_t* lora){
    gpiod_set_value(lora->reset_pin, 0);
    msleep(100);
    gpiod_set_value(lora->reset_pin, 1);
    msleep(100);
}

static void LoRa_sleep(LoRa_t* lora)
{
  LoRa_writeRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

static void LoRa_idle(LoRa_t* lora)
{
  LoRa_writeRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

static void LoRa_setFrequency(LoRa_t *lora, uint32_t frequency)
{
    uint64_t frf;

    lora->frequency = frequency;

    frf = ((uint64_t)frequency << 19) / 32000000;
    LoRa_writeRegister(lora, REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_writeRegister(lora, REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_writeRegister(lora, REG_FRF_LSB, (uint8_t)(frf >> 0));
}

static void LoRa_setOCP(LoRa_t *lora, uint8_t mA)
{
  uint8_t ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

  LoRa_writeRegister(lora, REG_OCP, 0x20 | (0x1F & ocpTrim));
}

static void LoRa_setTxPower(LoRa_t *lora, int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    LoRa_writeRegister(lora, REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level > 17) {
      if (level > 20) {
        level = 20;
      }

      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;

      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      LoRa_writeRegister(lora, REG_PA_DAC, 0x87);
      LoRa_setOCP(lora, 140);
    } else {
      if (level < 2) {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      LoRa_writeRegister(lora, REG_PA_DAC, 0x84);
      LoRa_setOCP(lora, 100);
    }

    LoRa_writeRegister(lora, REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

static void LoRa_setGain(LoRa_t *lora, uint8_t gain)
{
  // check allowed range
  if (gain > 6) {
    gain = 6;
  }
  
  // set to standby
  LoRa_idle(lora);
  
  // set gain
  if (gain == 0) {
    // if gain = 0, enable AGC
    LoRa_writeRegister(lora, REG_MODEM_CONFIG_3, 0x04);
  } else {
    // disable AGC
    LoRa_writeRegister(lora, REG_MODEM_CONFIG_3, 0x00);
	
    // clear Gain and set LNA boost
    LoRa_writeRegister(lora, REG_LNA, 0x03);
	
    // set gain
    LoRa_writeRegister(lora, REG_LNA, LoRa_readRegister(lora, REG_LNA) | (gain << 5));
  }
}

static int LoRa_getSpreadingFactor(LoRa_t *lora)
{
  return LoRa_readRegister(lora,REG_MODEM_CONFIG_2) >> 4;
}

static bool LoRa_isTransmitting(LoRa_t *lora)
{
  if ((LoRa_readRegister(lora, REG_OP_MODE) & MODE_TX) == MODE_TX) {
    return true;
  }

  if (LoRa_readRegister(lora, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
    // clear IRQ's
    LoRa_writeRegister(lora, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
  }

  return false;
}

static void LoRa_explicitHeaderMode(LoRa_t *lora)
{
  lora->implicitHeaderMode = 0;

  LoRa_writeRegister(lora, REG_MODEM_CONFIG_1, LoRa_readRegister(lora, REG_MODEM_CONFIG_1) & 0xfe);
}

static void LoRa_implicitHeaderMode(LoRa_t *lora)
{
  lora->implicitHeaderMode = 1;

  LoRa_writeRegister(lora, REG_MODEM_CONFIG_1, LoRa_readRegister(lora, REG_MODEM_CONFIG_1) | 0x01);
}

static int LoRa_beginPacket(LoRa_t *lora, int implicitHeader)
{
  if (LoRa_isTransmitting(lora)) {
    return 0;
  }

  // put in standby mode
  LoRa_idle(lora);

  if (implicitHeader) {
    LoRa_implicitHeaderMode(lora);
  } else {
    LoRa_explicitHeaderMode(lora);
  }

  // reset FIFO address and paload length
  LoRa_writeRegister(lora, REG_FIFO_ADDR_PTR, 0);
  LoRa_writeRegister(lora, REG_PAYLOAD_LENGTH, 0);

  return 1;
}

static void LoRa_handleDio0Rise(LoRa_t *lora)
{
  int irqFlags = LoRa_readRegister(lora, REG_IRQ_FLAGS);

  // clear IRQ's
  LoRa_writeRegister(lora, REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {

    if ((irqFlags & IRQ_RX_DONE_MASK) != 0) {
      // received a packet
      lora->packetIndex = 0;

      // read packet length
      int packetLength = lora->implicitHeaderMode ? LoRa_readRegister(lora, REG_PAYLOAD_LENGTH) : LoRa_readRegister(lora, REG_RX_NB_BYTES);

      // set FIFO address to current RX address
      LoRa_writeRegister(lora, REG_FIFO_ADDR_PTR, LoRa_readRegister(lora, REG_FIFO_RX_CURRENT_ADDR));

      if (lora->onReceive) {
        lora->onReceive(packetLength);
      }
    }
    else if ((irqFlags & IRQ_TX_DONE_MASK) != 0) {
      if (lora->onTxDone) {
        lora->onTxDone();
      }
    }
  }
}

static void LoRa_onDio0Rise(LoRa_t *lora)
{
  LoRa_handleDio0Rise(lora);
}

static void LoRa_dumpRegisters(LoRa_t *lora)
{
    int i;
    uint8_t val;

    for (i = 0; i < 128; i++) {
        val = LoRa_readRegister(lora, i);
        dev_info(&lora->spi->dev, "Reg[0x%02X] = 0x%02X\n", i, val);
    }
}

static uint8_t LoRa_random(LoRa_t *lora)
{
  return LoRa_readRegister(lora, REG_RSSI_WIDEBAND);
}

static void LoRa_disableInvertIQ(LoRa_t *lora)
{
  LoRa_writeRegister(lora, REG_INVERTIQ,  0x27);
  LoRa_writeRegister(lora, REG_INVERTIQ2, 0x1d);
}

static void LoRa_enableInvertIQ(LoRa_t *lora)
{
  LoRa_writeRegister(lora, REG_INVERTIQ,  0x66);
  LoRa_writeRegister(lora, REG_INVERTIQ2, 0x19);
}

static void LoRa_enableCrc(LoRa_t *lora)
{
  LoRa_writeRegister(lora, REG_MODEM_CONFIG_2, LoRa_readRegister(lora, REG_MODEM_CONFIG_2) | 0x04);
}

static void LoRa_disableCrc(LoRa_t *lora)
{
  LoRa_writeRegister(lora, REG_MODEM_CONFIG_2, LoRa_readRegister(lora, REG_MODEM_CONFIG_2) & 0xfb);
}

static void LoRa_setSyncWord(LoRa_t *lora, int sw)
{
  LoRa_writeRegister(lora, REG_SYNC_WORD, sw);
}

static void LoRa_setPreambleLength(LoRa_t *lora, long length)
{
  LoRa_writeRegister(lora, REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  LoRa_writeRegister(lora, REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

static void LoRa_setCodingRate4(LoRa_t *lora, int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  LoRa_writeRegister(lora, REG_MODEM_CONFIG_1, (LoRa_readRegister(lora,REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

static void LoRa_setSignalBandwidth(LoRa_t *lora, long sbw)
{
    int bw;

    if (sbw <= 7800) {
        bw = 0;
    } else if (sbw <= 10400) {
        bw = 1;
    } else if (sbw <= 15600) {
        bw = 2;
    } else if (sbw <= 20800) {
        bw = 3;
    } else if (sbw <= 31250) {
        bw = 4;
    } else if (sbw <= 41700) {
        bw = 5;
    } else if (sbw <= 62500) {
        bw = 6;
    } else if (sbw <= 125000) {
        bw = 7;
    } else if (sbw <= 250000) {
        bw = 8;
    } else {
        bw = 9; // 500 kHz
    }

    LoRa_writeRegister(lora, REG_MODEM_CONFIG_1, (LoRa_readRegister(lora,REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
    LoRa_setLdoFlag(lora);
}

static void LoRa_setLdoFlag(LoRa_t *lora)
{
    // Section 4.1.1.5
    long bw = LoRa_getSignalBandwidth(lora);
    int sf = LoRa_getSpreadingFactor(lora);

    // symbolDuration = (2^SF) / BW  (seconds)
    // convert sang ms: *1000
    // => symbolDuration_ms = (1000 * 2^SF) / BW

    long symbolDuration = (1000L << sf) / bw;

    // Section 4.1.1.6
    bool ldoOn = symbolDuration > 16;

    uint8_t config3 = LoRa_readRegister(lora, REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, ldoOn);
    LoRa_writeRegister(lora, REG_MODEM_CONFIG_3, config3);
}

static long LoRa_getSignalBandwidth(LoRa_t *lora)
{
  uint8_t bw = (LoRa_readRegister(lora, REG_MODEM_CONFIG_1) >> 4);

  switch (bw) {
        case 0: return 7800;
        case 1: return 10400;
        case 2: return 15600;
        case 3: return 20800;
        case 4: return 31250;
        case 5: return 41700;
        case 6: return 62500;
        case 7: return 125000;
        case 8: return 250000;
        case 9: return 500000;
        default: return -1;
    }

  return -1;
}

static void LoRa_setSpreadingFactor(LoRa_t *lora, int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    LoRa_writeRegister(lora, REG_DETECTION_OPTIMIZE, 0xc5);
    LoRa_writeRegister(lora, REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    LoRa_writeRegister(lora, REG_DETECTION_OPTIMIZE, 0xc3);
    LoRa_writeRegister(lora, REG_DETECTION_THRESHOLD, 0x0a);
  }

  LoRa_writeRegister(lora, REG_MODEM_CONFIG_2, (LoRa_readRegister(lora,REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  LoRa_setLdoFlag(lora);
}

static void LoRa_receive(LoRa_t *lora, int size)
{
  LoRa_writeRegister(lora, REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

  if (size > 0) {
    LoRa_implicitHeaderMode(lora);

    LoRa_writeRegister(lora, REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    LoRa_explicitHeaderMode(lora);
  }

  LoRa_writeRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

static void LoRa_onTxDone(LoRa_t *lora, void (*callback)(void))
{
    // Linux không dùng callback kiểu này
    lora->onTxDone = callback;
}

static void LoRa_onReceive(LoRa_t *lora, void (*callback)(int))
{
  // Linux không dùng callback kiểu này  
  lora->onReceive = callback;
}

static int LoRa_available(LoRa_t *lora)
{
  return (LoRa_readRegister(lora, REG_RX_NB_BYTES) - lora->packetIndex);
}

static uint8_t LoRa_peek(LoRa_t *lora)
{
  if (!LoRa_available(lora)) {
    return -1;
  }

  // store current FIFO address
  int currentAddress = LoRa_readRegister(lora, REG_FIFO_ADDR_PTR);

  // read
  uint8_t b = LoRa_readRegister(lora, REG_FIFO);

  // restore FIFO address
  LoRa_writeRegister(lora, REG_FIFO_ADDR_PTR, currentAddress);

  return b;
}

static size_t LoRa_write_fifo(LoRa_t *lora, const uint8_t *buffer, size_t size)
{
  int currentLength = LoRa_readRegister(lora, REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  // write data
  for (size_t i = 0; i < size; i++) {
    LoRa_writeRegister(lora, REG_FIFO, buffer[i]);
  }

  // update length
  LoRa_writeRegister(lora, REG_PAYLOAD_LENGTH, currentLength + size);

  return size;
}

static int LoRa_read_fifo(LoRa_t *lora)
{
  if (!LoRa_available(lora)) {
    return -1;
  }

  lora->packetIndex++;

  return LoRa_readRegister(lora, REG_FIFO);
}

static int LoRa_rssi(LoRa_t *lora)
{
  return (LoRa_readRegister(lora, REG_RSSI_VALUE) - (lora->frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

static int LoRa_packetRssi(LoRa_t *lora)
{
  return (LoRa_readRegister(lora, REG_PKT_RSSI_VALUE) - (lora->frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

static int LoRa_packetSnr(LoRa_t *lora)
{
    // giá trị SNR
    return (int8_t)(LoRa_readRegister(lora, REG_PKT_SNR_VALUE) / 4);
}

static long LoRa_packetFrequencyError(LoRa_t *lora)
{
    int32_t freqError = 0;
    int64_t temp;

    freqError = (LoRa_readRegister(lora, REG_FREQ_ERROR_MSB) & 0x07);
    freqError <<= 8;
    freqError |= LoRa_readRegister(lora, REG_FREQ_ERROR_MID);
    freqError <<= 8;
    freqError |= LoRa_readRegister(lora, REG_FREQ_ERROR_LSB);

    // sign bit
    if (LoRa_readRegister(lora, REG_FREQ_ERROR_MSB) & 0x08)
        freqError -= 524288; // 2^19

    // temp = freqError * 2^24
    temp = (int64_t)freqError << 24;

    // chia cho FXOSC (32MHz)
    temp = temp / 32000000;

    // nhân bandwidth
    temp = temp * LoRa_getSignalBandwidth(lora);

    // chia 500k
    temp = temp / 500000;

    return (long)temp;
}

static int LoRa_parsePacket(LoRa_t *lora, int size)
{
  int packetLength = 0;
  int irqFlags = LoRa_readRegister(lora, REG_IRQ_FLAGS);

  if (size > 0) {
    LoRa_implicitHeaderMode(lora);
    LoRa_writeRegister(lora, REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    LoRa_explicitHeaderMode(lora);
  }

  // clear IRQ's
  LoRa_writeRegister(lora, REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && !(irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)) {
    // received a packet
    lora->packetIndex = 0;

    // read packet length
    if (lora->implicitHeaderMode) {
      packetLength = LoRa_readRegister(lora, REG_PAYLOAD_LENGTH);
    } else {
      packetLength = LoRa_readRegister(lora, REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    LoRa_writeRegister(lora, REG_FIFO_ADDR_PTR, LoRa_readRegister(lora,REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    LoRa_idle(lora);
  } 
  else if (LoRa_readRegister(lora, REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    LoRa_writeRegister(lora, REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    LoRa_writeRegister(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

static inline int LoRa_timedRead(LoRa_t *lora, uint32_t timeout_ms)
{
    int c;
    unsigned long start = jiffies;

    do {
        c = LoRa_read_fifo(lora);   // hàm read của m
        if (c >= 0) {
            return c;
        }

        // tránh ăn CPU
        udelay(1000);
    } while (time_before(jiffies, start + msecs_to_jiffies(timeout_ms)));

    return -1; // timeout
}

static size_t LoRa_readBytes(LoRa_t *lora, uint8_t *buffer, size_t length)
{
  size_t count = 0;
    while(count < length) {
        int c = LoRa_timedRead(lora, 100);
        if(c < 0) {
            break;
        }
        *buffer++ = (char) c;
        count++;
    }
    return count;
}

static int LoRa_receive_continue(LoRa_t *lora, LoRa_packet_t *pkt)
{
    uint8_t irq_flags, num_bytes, fifo_addr;
    int i;

    // 1. reset flag software
    lora->data_ready = false;

    // 2. clear IRQ
    LoRa_writeRegister(lora, REG_IRQ_FLAGS, 0xFF);

    // 3. set RX mode
    LoRa_receive(lora,0);
    // 4. wait interrupt
    if (wait_event_interruptible(lora->rx_wait, lora->data_ready)) {
        return -ERESTARTSYS;
    }

    // 5.Read IRQ after waking up
    irq_flags = LoRa_readRegister(lora, REG_IRQ_FLAGS);

    dev_info(&lora->spi->dev, "IRQ=0x%02X\n", irq_flags);

    // 6. check RX_DONE
    if (irq_flags & 0x40)
    {
        // check CRC
        if (irq_flags & 0x20) {
            dev_warn(&lora->spi->dev, "CRC ERROR\n");
            pkt->length = 0;
            goto out;
        }

        // 7. read payload
        num_bytes = LoRa_readRegister(lora, REG_RX_NB_BYTES);
        pkt->length = (num_bytes > 256) ? 256 : num_bytes;

        fifo_addr = LoRa_readRegister(lora, REG_FIFO_RX_CURRENT_ADDR);
        LoRa_writeRegister(lora, REG_FIFO_ADDR_PTR, fifo_addr);

        for (i = 0; i < pkt->length; i++) {
            pkt->data[i] = LoRa_readRegister(lora, REG_FIFO);
        }

        // debug RSSI/SNR
        int rssi = LoRa_packetRssi(lora);
        int snr  = LoRa_packetSnr(lora);

        dev_info(&lora->spi->dev,
                 "[RX] len=%d RSSI=%d SNR=%d\n",
                 pkt->length, rssi, snr);
    }
    else
    {
        pkt->length = 0;
        dev_warn(&lora->spi->dev, "Spurious IRQ\n");
    }

out:
    // 8. clear IRQ & set RX_CONTIMODE
    LoRa_writeRegister(lora, REG_IRQ_FLAGS, 0xFF);
    LoRa_receive(lora,0);

    return 0;
}

static ssize_t LoRa_read_file(struct file *file, char __user *buf, size_t len, loff_t *offset)
{
  int rssi;
  int snr ;
  LoRa_t *lora = file->private_data;

  LoRa_receive(lora,0);

  uint8_t mode = LoRa_readRegister(lora, REG_OP_MODE);
  dev_info(&lora->spi->dev, "MODE=0x%02X\n", mode);

  dev_info(&lora->spi->dev, "IRQ=0x%02X\n",
    LoRa_readRegister(lora, REG_IRQ_FLAGS));

    if (!lora)
        return -EINVAL;
    int packetSize = LoRa_parsePacket(lora,0);

    if (packetSize > 0) {
        rssi = LoRa_packetRssi(lora);
        snr  = LoRa_packetSnr(lora);
    }

    dev_info(&lora->spi->dev,
              "[LORA] RX: size=%d RSSI=%d SNR=%d\n",
              packetSize,
              rssi,
              snr);
    return packetSize;
}

static ssize_t LoRa_read_file_continue(struct file *file, char __user *buf, size_t len, loff_t *offset)
{
    struct lora_packet pkt;
    LoRa_t *lora = file->private_data;

    // call blocking to wait for interrupt
    LoRa_receive_continue(lora, &pkt);

    // copy data ra user
    copy_to_user(buf, pkt.data, pkt.length);

    //print_hex_dump(KERN_INFO, "LoRa RX Payload: ", DUMP_PREFIX_OFFSET, 16, 1, pkt.data, pkt.length, true);

    return pkt.length;
}


static int LoRa_open_file(struct inode *inode, struct file *file)
{
    LoRa_t *lora = container_of(inode->i_cdev, LoRa_t, lora_cdev);
    int ret;

    file->private_data = lora;

    LoRa_receive(lora,0);

    dev_info(&lora->spi->dev, "Device opened successfully\n");
    return 0;
}

static int LoRa_release_file(struct inode *inode, struct file *file)
{
    LoRa_t *lora = container_of(inode->i_cdev, LoRa_t, lora_cdev);
    int ret;

    file->private_data = lora;

    LoRa_sleep(lora);

    dev_info(&lora->spi->dev, "Device close successfully\n");
    return 0;
}

static long LoRa_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    LoRa_t *lora = file->private_data;
    int tmp_val;
    LoRa_config_t config;

    switch (cmd) {
        case LORA_IOC_RESET:
            LoRa_reset(lora); 
            pr_info("LoRa: Hardware reset triggered via IOCTL\n");
            break;

        case LORA_IOC_GET_SNR:
            {
                tmp_val = LoRa_packetSnr(lora);
                
                if (copy_to_user((int __user *)arg, &tmp_val, sizeof(tmp_val)))
                    return -EFAULT;
            }
            break;

        case LORA_IOC_GET_PKT_RSSI:
            {
                tmp_val = LoRa_packetRssi(lora);
                
                if (copy_to_user((int __user *)arg, &tmp_val, sizeof(tmp_val)))
                    return -EFAULT;
            }
            break;

        case LORA_IOC_GET_CONFIG:
            {
                uint8_t frf_msb, frf_mid, frf_lsb;
                uint8_t mc1, mc2;
                uint64_t frf;

                // 1. Current Mode
                config.current_mode = LoRa_readRegister(lora, REG_OP_MODE);

                // 2. Frequency
                frf_msb = LoRa_readRegister(lora, REG_FRF_MSB);
                frf_mid = LoRa_readRegister(lora, REG_FRF_MID);
                frf_lsb = LoRa_readRegister(lora, REG_FRF_LSB);
                frf = ((uint64_t)frf_msb << 16) | ((uint64_t)frf_mid << 8) | frf_lsb;
                // Freq = (Frf * 32MHz) / (2^19)
                config.frequency = (int)((frf * 32000000ULL) >> 19);

                // 3. Spreading Factor, Bandwidth, CR, CRC Enable
                mc1 = LoRa_readRegister(lora, REG_MODEM_CONFIG_1);
                mc2 = LoRa_readRegister(lora, REG_MODEM_CONFIG_2);

                config.bandWidth = mc1 >> 4;                 // Bits 7-4
                config.crcRate = ((mc1 >> 1) & 0x07) + 4;    // Bits 3-1
                config.spreadingFactor = mc2 >> 4;           // Bits 7-4
                config.enableCrc = (mc2 >> 2) & 0x01;        // Bit 2

                // 4. Preamble Length
                config.preamble = (LoRa_readRegister(lora, REG_PREAMBLE_MSB) << 8) | 
                                   LoRa_readRegister(lora, REG_PREAMBLE_LSB);

                // 5. TX Power & Sync Word

                config.power = lora->txPower;
                config.syncWord = LoRa_readRegister(lora, REG_SYNC_WORD);

                // Copy struct to User-space
                if (copy_to_user((LoRa_config_t __user *)arg, &config, sizeof(config)))
                    return -EFAULT;
            }
            break;

        case LORA_IOC_SET_CONFIG:
            {
                if (copy_from_user(&config, (LoRa_config_t __user *)arg, sizeof(config)))
                    return -EFAULT;
                
                tmp_val = LoRa_readRegister(lora, REG_OP_MODE);
                lora->txPower = config.power;

                LoRa_idle(lora); // Đưa về Standby
                
                LoRa_setFrequency(lora, config.frequency);
                LoRa_setSpreadingFactor(lora, config.spreadingFactor);
                LoRa_setSignalBandwidth(lora, config.bandWidth); 
                LoRa_setCodingRate4(lora, config.crcRate);
                LoRa_setPreambleLength(lora, config.preamble);     
                LoRa_setSyncWord(lora, config.syncWord);                       
                LoRa_setTxPower(lora, config.power, 1);
                config.enableCrc ? LoRa_enableCrc(lora) : LoRa_disableCrc(lora) ;

                LoRa_writeRegister(lora, REG_OP_MODE, (uint8_t)tmp_val );
            }
            break;

        default:
            return -ENOTTY; 
    }

    return 0;
}

static irqreturn_t LoRa_irq_thread_fn(int irq, void *dev_id)
{
    LoRa_t *lora = dev_id;

    // 1. Mark as containing new data
    lora->data_ready = true;

    // 2. The bookmarking application is being awaited at wait_event_interruptible
    wake_up_interruptible(&lora->rx_wait);
    
    dev_info(&lora->spi->dev, "Interrupt DIO0 triggered!\n");

    return IRQ_HANDLED;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const struct file_operations sx1278_fops = {
    .owner          = THIS_MODULE,
    .open           = LoRa_open_file,
    .release        = LoRa_release_file,
    .unlocked_ioctl = LoRa_ioctl,
    .read = LoRa_read_file_continue,
};

static int LoRa_probe(struct spi_device *spi)
{
    int ret;
    uint8_t version;
    LoRa_t* lora;

    // 1. SPI Config
    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;
    ret = spi_setup(spi);
    if (ret) {
        return dev_err_probe(&spi->dev, ret, "Setup SPI failed!\n");
    }

    lora = devm_kzalloc(&spi->dev, sizeof(*lora), GFP_KERNEL);
    if (!lora) return -ENOMEM;

    lora->spi = spi;
    mutex_init(&lora->lock);
    spi_set_drvdata(spi, lora);

    // 3. Get Reset và Interrupt (DIO0) from Device Tree
    lora->reset_pin = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);
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
                               
    ret = devm_request_threaded_irq(&spi->dev, 
                                    lora->irq, 
                                    NULL, 
                                    LoRa_irq_thread_fn, 
                                    IRQF_ONESHOT, 
                                    "LoRa_irq", 
                                    lora); 

    if (ret) {
        dev_err(&spi->dev, "Failed to register IRQ: %d\n", ret);
        return ret;
    }

    // 3. Khởi tạo hàng đợi chờ
    init_waitqueue_head(&lora->rx_wait);
    lora->data_ready = false;

    pr_info("sx1278: IRQ %d (GPIO 27) has successfully registered.!\n", lora->irq);

    // 4. Reset module before Init 
    gpiod_set_value(lora->reset_pin, 0);
    msleep(100);
    gpiod_set_value(lora->reset_pin, 1);
    msleep(100);

    // 5. Conection check (Version Check)
    version = LoRa_readRegister(lora, REG_VERSION);
    if (version != 0x12) {
        dev_err(&spi->dev, "Failed to detected SX1278  (Version: 0x%02x, mong d?i: 0x12)\n", version);
        return -ENODEV;
    }
    dev_info(&spi->dev, "Detected SX1278 successfully, Version: 0x%02x\n", version);

    // put in sleep mode
      LoRa_sleep(lora);

    // set frequency
      LoRa_setFrequency(lora, 433000000);

    // set base addresses
        LoRa_writeRegister(lora, REG_FIFO_TX_BASE_ADDR, 0);
        LoRa_writeRegister(lora, REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
        LoRa_writeRegister(lora, REG_LNA, LoRa_readRegister(lora, REG_LNA) | 0x03);

    // set auto AGC
        LoRa_writeRegister(lora, REG_MODEM_CONFIG_3, 0x04);

        
    // set output power to 14 dBm
        lora->txPower = 14;
        LoRa_setTxPower(lora, lora->txPower, 1);

    // LoRa_setSpreadingFactor(lora, 7);     // SF7
    // LoRa_setSignalBandwidth(lora, 125000); // 125 kHz
    // LoRa_setCodingRate4(lora, 5);         // 4/5
    // LoRa_setPreambleLength(lora, 10);     // Preamble = 10
    // LoRa_setSyncWord(lora, 0xF1);         // Sync Word giống STM32
    // LoRa_enableCrc(lora);               // bật CRC (nếu STM32 bật)

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

static void LoRa_remove(struct spi_device *spi) 
{
    LoRa_t *lora = spi_get_drvdata(spi);
    if (!lora) return;

    LoRa_sleep(lora);

    if (lora->lora_device) {
        device_destroy(lora->lora_class, lora->dev_num);
    }
    if (!IS_ERR_OR_NULL(lora->lora_class)) {
        class_destroy(lora->lora_class);
    }

    cdev_del(&lora->lora_cdev);
    unregister_chrdev_region(lora->dev_num, 1);

    mutex_destroy(&lora->lock);

    dev_info(&spi->dev, "SX1278 LoRa driver removed successfully\n");
}



static const struct of_device_id sx1278_dt_ids[] = {
    { .compatible = "semtech,sx1278", },
    { }
};
MODULE_DEVICE_TABLE(of, sx1278_dt_ids);

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
    .id_table = sx1278_ids,
    .probe    = LoRa_probe,
    .remove   = LoRa_remove,           
};
module_spi_driver(sx1278_driver);

MODULE_DESCRIPTION("SX1278 LoRa SPI Driver");
MODULE_AUTHOR("Duy Bach - HCMUT");
MODULE_LICENSE("GPL");