#ifndef SX1278_IOCTL_H
#define SX1278_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define LORA_IOC_MAGIC 'L'

struct lora_packet {
    uint8_t  payload[256]; 
    uint16_t size;           
};

#define LORA_IOC_RESET      _IO(LORA_IOC_MAGIC, 0)
#define LORA_IOC_TRANSMIT   _IOW(LORA_IOC_MAGIC, 1, struct lora_packet)
#define LORA_IOC_RECEIVE    _IOR(LORA_IOC_MAGIC, 2, struct lora_packet)
#define LORA_IOC_GET_RSSI   _IOR(LORA_IOC_MAGIC, 3, int)

#define LORA_IOC_SET_FREQ        _IOW(LORA_IOC_MAGIC, 4, int)
#define LORA_IOC_SET_SF          _IOW(LORA_IOC_MAGIC, 5, int)
#define LORA_IOC_SET_POWER       _IOW(LORA_IOC_MAGIC, 6, uint8_t)
#define LORA_IOC_SET_SYNC_WORD   _IOW(LORA_IOC_MAGIC, 7, uint8_t)
#define LORA_IOC_SET_OCP         _IOW(LORA_IOC_MAGIC, 8, uint8_t)

#endif