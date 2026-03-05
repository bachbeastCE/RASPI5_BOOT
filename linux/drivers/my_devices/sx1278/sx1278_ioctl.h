#ifndef SX1278_IOCTL_H
#define SX1278_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

/* Magic Number for LoRa driver */
#define LORA_IOC_MAGIC 'L'

/* Data structure for Configuration */
struct lora_config {
    uint32_t frequency;     /* in Hz */
    uint8_t  sf;            /* Spreading Factor (6-12) */
    uint8_t  bw;            /* Bandwidth Index (0-9) */
    uint8_t  cr;            /* Coding Rate (1-4) */
    uint8_t  power;         /* Output power (2-20dBm) */
};

/* Data structure for Data Transmission/Reception */
struct lora_packet {
    uint8_t  payload[256];  /* Data buffer (SX1278 FIFO is 256 bytes) */
    uint16_t size;          /* Actual size of data */
};

/* --- IOCTL Command Definitions --- */

/* 1. Reset: No data transfer, just triggers Hardware Reset */
#define LORA_IOC_RESET      _IO(LORA_IOC_MAGIC, 1)

/* 2. Config: Sends configuration structure from User to Kernel */
#define LORA_IOC_CONFIG     _IOW(LORA_IOC_MAGIC, 2, struct lora_config)

/* 3. Transmit: Sends data packet from User to Kernel */
#define LORA_IOC_TRANSMIT   _IOW(LORA_IOC_MAGIC, 3, struct lora_packet)

/* 4. Receive: Reads data packet from Kernel to User */
#define LORA_IOC_RECEIVE    _IOR(LORA_IOC_MAGIC, 4, struct lora_packet)

#endif