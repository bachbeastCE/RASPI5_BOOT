#include "lora_api.h"

int main() {
    int fd;
    LoRa_config_t config;
    int snr, rssi;
    uint8_t rx_buffer[BUFFER_MAX_SIZE]; // Buffer chứa payload
    
    printf("--- LoRa User-Space Test App ---\n");

    // 1. Mở device file
    fd = lora_open("/dev/sx1278");
    if (fd < 0) return EXIT_FAILURE;

    // 2. Test IOCTL: Lấy cấu hình hiện tại và in ra màn hình
    printf("\n[+] Reading current hardware config...\n");
    if (lora_get_config(fd, &config) == 0) {
        printf("    Frequency : %d Hz\n", config.frequency);
        printf("    SF        : %d\n", config.spreadingFactor);
        printf("    Bandwidth : %d\n", config.bandWidth);
        printf("    Sync Word : 0x%02X\n", config.syncWord);
        printf("    CRC Enable: %s\n", config.enableCrc ? "Yes" : "No");
    }

    // ==========================================================
    // TEST IOCTL: SET VÀ ĐỌC LẠI CẤU HÌNH
    // ==========================================================
    printf("\n[+] Setting new LoRa config...\n");

    LoRa_config_t set_cfg;
    memset(&set_cfg, 0, sizeof(set_cfg)); // Khởi tạo sạch struct

    // Ghi đè cấu hình theo yêu cầu
    set_cfg.frequency       = 434000000; 
    set_cfg.spreadingFactor = 7;         
    set_cfg.bandWidth       = 125000;    
    set_cfg.crcRate         = 5;         
    set_cfg.preamble        = 10;        
    set_cfg.syncWord        = 0xF1;      
    set_cfg.enableCrc       = 1;         
    set_cfg.power           = 20;        

    // Gọi IOCTL Set
    if (lora_set_config(fd, &set_cfg) == 0) {
        printf("    -> SET command sent successfully!\n");
    } else {
        printf("    -> Failed to set config!\n");
    }

    // ==========================================================
    // ĐỌC NGƯỢC LẠI NGAY LẬP TỨC ĐỂ XÁC MINH
    // ==========================================================
    printf("\n[+] Verifying config directly from Hardware Registers...\n");
    
    LoRa_config_t read_cfg;
    if (lora_get_config(fd, &read_cfg) == 0) {
        printf("    Frequency : %d Hz\n", read_cfg.frequency);
        printf("    SF        : %d\n", read_cfg.spreadingFactor);
        printf("    Bandwidth : %ld Hz\n", read_cfg.bandWidth); // Sẽ in ra đúng 125000 Hz
        printf("    CodingRate: 4/%d\n", read_cfg.crcRate);
        printf("    Preamble  : %d\n", read_cfg.preamble);
        printf("    Sync Word : 0x%02X\n", read_cfg.syncWord);
        printf("    CRC Enable: %s\n", read_cfg.enableCrc ? "Yes" : "No");
        printf("    Power(Raw): %d\n", read_cfg.power);
    } else {
        printf("    -> Failed to read back config!\n");
    }

    // 3. Vòng lặp nhận dữ liệu
    printf("\n[+] Waiting for incoming data (RxContinuous mode)...\n");
    while (1) {
        memset(rx_buffer, 0, sizeof(rx_buffer));
        
        // Gọi API receive (Nó sẽ nằm im ở đây chờ ngắt từ Kernel)
        int rx_len = lora_receive(fd, rx_buffer, sizeof(rx_buffer));
        
        if (rx_len > 0) {
            // Đọc thêm thông số tín hiệu của gói vừa nhận
            lora_get_pkt_rssi(fd, &rssi);
            lora_get_snr(fd, &snr);

            printf("\n------------------------------------------------\n");
            printf("Received %d bytes | RSSI: %d dBm | SNR: %d dB\n", rx_len, rssi, snr);
            
            // In raw data ra màn hình (Format giống hexdump)
            printf("Payload Data:\n");
            for (int i = 0; i < rx_len; i++) {
                printf("%02X ", rx_buffer[i]);
                if ((i + 1) % 16 == 0) printf("\n");
            }
            printf("\n");
        }
    }

    lora_close(fd);
    return EXIT_SUCCESS;
}