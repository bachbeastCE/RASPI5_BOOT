#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

// =========================================================================
// 1. STRUCT & IOCTL
// =========================================================================

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

#define LORA_IOC_MAGIC 'L'
#define LORA_IOC_RESET            _IO(LORA_IOC_MAGIC, 0)
#define LORA_IOC_GET_SNR          _IOR(LORA_IOC_MAGIC, 1, int)
#define LORA_IOC_GET_PKT_RSSI     _IOR(LORA_IOC_MAGIC, 2, int)
#define LORA_IOC_GET_CONFIG       _IOR(LORA_IOC_MAGIC, 3, LoRa_config_t)
#define LORA_IOC_SET_CONFIG       _IOW(LORA_IOC_MAGIC, 4, LoRa_config_t)

#define BUFFER_MAX_SIZE 256

// =========================================================================
// 2. API GIAO TIẾP VỚI DRIVER
// =========================================================================

// Mở thiết bị
int lora_open(const char *dev_path);

// Đóng thiết bị
void lora_close(int fd);

// API: Reset module
int lora_reset(int fd);

// API: Lấy SNR của gói tin vừa nhận
int lora_get_snr(int fd, int *snr);

// API: Lấy RSSI của gói tin vừa nhận
int lora_get_pkt_rssi(int fd, int *rssi);

// API: Đọc toàn bộ cấu hình hiện tại từ thanh ghi
int lora_get_config(int fd, LoRa_config_t *config);

// API: Ghi cấu hình mới xuống module
int lora_set_config(int fd, LoRa_config_t *config);

// API: Nhận dữ liệu 
int lora_receive(int fd, uint8_t *buffer, size_t max_len);