#include "lora_api.h"
#include "micro_aes.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>

#define PACKET_SIZE 88
#define PLAIN_TEXT_SIZE 64

typedef struct __attribute__((packed)) LOCATION_DATA_HEADER {
	uint64_t seq_num; // 8 bytes
}  LOC_DATA_HEADER;

typedef struct __attribute__((packed)) LOCATION_DATA {
	uint32_t device_id; // 4 bytes
	float gps_hdop;  // 4 bytes
    double loc_gps_lon; // 8 bytes
    double loc_gps_lat; // 8 bytes
    double loc_gps_alt; // 8 bytes
    double tag_gps_lon; // 8 bytes
    double tag_gps_lat; // 8 bytes
    double tag_gps_alt; // 8 bytes
    double tag_distance; // 8 bytes
}  LOC_DATA_PAYLOAD;

typedef struct __attribute__((packed)) LOC_RECEIVE_PACKET {
  LOC_DATA_HEADER loc_data_header;
  LOC_DATA_PAYLOAD loc_data_payload;
  uint8_t security_tag[16] ;
} LOC_RECEIVE_PACKET ;

LOC_RECEIVE_PACKET receive_packet;
uint8_t key[] = {0x2b, 0x7e, 0x15, 0x16,0x28, 0xae, 0xd2, 0xa6,0xab, 0xf7, 0x15, 0x88,0x09, 0xcf, 0x4f, 0x3c};
uint8_t gcm_nonce[GCM_NONCE_LEN];

LOC_DATA_PAYLOAD plain_text;
uint8_t decrypt_status;

// Hàm này nhận vào Sequence Number và con trỏ chứa Payload đã giải mã (plain_text)
void print_location_data(uint64_t seq_num, const LOC_DATA_PAYLOAD *payload) {
    // Dùng PRIu64 để in chuẩn xác uint64_t trên mọi hệ điều hành (32/64 bit)
    printf("------ After decrypt: -------------\n");
    printf("Packet order: %" PRIu64 "\n", seq_num);
    printf("------ RECEIVED DATA PACKET ---\n");
    
    // In Device ID (Dạng Hex 8 chữ số, có số 0 ở đầu)
    printf("Device ID:    0x%08X\n", payload->device_id);

    // In tọa độ GPS Local (%.6f để lấy 6 số thập phân, %.2f lấy 2 số)
    printf("Local GPS:    Lat: %.6f | Lon: %.6f | Alt: %.2f\n", 
           payload->loc_gps_lat, payload->loc_gps_lon, payload->loc_gps_alt);

    // In tọa độ GPS Tag
    printf("Tag GPS:      Lat: %.6f | Lon: %.6f | Alt: %.2f\n", 
           payload->tag_gps_lat, payload->tag_gps_lon, payload->tag_gps_alt);

    // In khoảng cách và sai số
    printf("Distance:     %.2f meters\n", payload->tag_distance);
    printf("GPS HDOP:     %.2f\n", payload->gps_hdop);
    
    printf("\n------------------------------------------------\n");
}

int main() {
    int fd;
    LoRa_config_t config;
    int snr, rssi;
    //uint8_t rx_buffer[BUFFER_MAX_SIZE]; // Buffer chứa payload
    
    printf("--- LoRa User-Space Test App ---\n");

    // 1. Mở device file
    fd = lora_open("/dev/sx1278");
    if (fd < 0) return EXIT_FAILURE;

    // 2. Test IOCTL: 
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
        memset((uint8_t*)&receive_packet, 0, sizeof(receive_packet));
        
        // Gọi API receive (Nó sẽ nằm im ở đây chờ ngắt từ Kernel)
        int rx_len = lora_receive(fd, (uint8_t*)&receive_packet, sizeof(receive_packet));
        
        if (rx_len > 0) {
            // Đọc thêm thông số tín hiệu của gói vừa nhận
            lora_get_pkt_rssi(fd, &rssi);
            lora_get_snr(fd, &snr);

            printf("\n------------------------------------------------\n");
            printf("Received %d bytes | RSSI: %d dBm | SNR: %d dB\n", rx_len, rssi, snr);
            
            // In raw data ra màn hình (Format giống hexdump)
            uint8_t* byte_ptr = (uint8_t*)&receive_packet; // Ép kiểu sang con trỏ byte
            printf("Payload Data:\n");
            for (int i = 0; i < rx_len; i++) {
                printf("%02X ", byte_ptr[i]);
                if ((i + 1) % 16 == 0) printf("\n");
            }
            printf("\n");

            // Giải mã tín hiệu
            memset(gcm_nonce, 0,  GCM_NONCE_LEN);
	        memcpy(gcm_nonce, &receive_packet.loc_data_header, sizeof(LOC_DATA_HEADER));//Copy 8 bytes from header to gcm_nonce to create specific nonce of one time tranfer
            decrypt_status = AES_GCM_decrypt(key, gcm_nonce,
                            &(receive_packet.loc_data_header.seq_num), sizeof(receive_packet.loc_data_header.seq_num),
                            &(receive_packet.loc_data_payload), sizeof(receive_packet.loc_data_payload), &plain_text );

            print_location_data(receive_packet.loc_data_header.seq_num, &plain_text);
        }
    }

    lora_close(fd);
    return EXIT_SUCCESS;
}