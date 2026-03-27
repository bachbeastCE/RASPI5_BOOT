#include "lora_api.h"

int lora_open(const char *dev_path) {
    int fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        perror("Failed to open LoRa device");
    }
    return fd;
}

void lora_close(int fd) {
    if (fd >= 0) close(fd);
}

int lora_reset(int fd) {
    if (ioctl(fd, LORA_IOC_RESET) < 0) {
        perror("IOCTL Reset failed");
        return -1;
    }
    return 0;
}

int lora_get_snr(int fd, int *snr) {
    if (ioctl(fd, LORA_IOC_GET_SNR, snr) < 0) {
        perror("IOCTL Get SNR failed");
        return -1;
    }
    return 0;
}

int lora_get_pkt_rssi(int fd, int *rssi) {
    if (ioctl(fd, LORA_IOC_GET_PKT_RSSI, rssi) < 0) {
        perror("IOCTL Get RSSI failed");
        return -1;
    }
    return 0;
}

int lora_get_config(int fd, LoRa_config_t *config) {
    if (ioctl(fd, LORA_IOC_GET_CONFIG, config) < 0) {
        perror("IOCTL Get Config failed");
        return -1;
    }
    return 0;
}

int lora_set_config(int fd, LoRa_config_t *config) {
    if (ioctl(fd, LORA_IOC_SET_CONFIG, config) < 0) {
        perror("IOCTL Set Config failed");
        return -1;
    }
    return 0;
}

int lora_receive(int fd, uint8_t *buffer, size_t max_len) {
    int bytes_read = read(fd, buffer, max_len);
    if (bytes_read < 0) {
        perror("Failed to read from LoRa device");
    }
    return bytes_read;
}