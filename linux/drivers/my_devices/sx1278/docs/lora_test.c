#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdint.h>
#include "sx1278_ioctl.h" // Đảm bảo file này nằm cùng thư mục

#define DEVICE_PATH "/dev/lora0" // Thay đổi theo tên thiết bị bạn đăng ký

int main() {
    int fd;
    int freq = 433000000; // 433 MHz
    int sf = 7;           // Spreading Factor 7
    int rssi;
    uint8_t sync_word = 0x12;
    struct lora_packet tx_pkt;
    struct lora_packet rx_pkt;

    // 1. Mở thiết bị
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Lỗi: Không thể mở file thiết bị");
        return -1;
    }
    printf("--- Đã mở thiết bị LoRa SX1278 ---\n");

    // 2. Reset phần cứng
    if (ioctl(fd, LORA_IOC_RESET) < 0) {
        perror("Lỗi IOCTL Reset");
    } else {
        printf("1. Đã Reset phần cứng SX1278 thành công.\n");
    }

    // 3. Cấu hình các tham số RF
    if (ioctl(fd, LORA_IOC_SET_FREQ, &freq) < 0) perror("Lỗi Set Freq");
    else printf("2. Đã đặt tần số: %d Hz\n", freq);

    if (ioctl(fd, LORA_IOC_SET_SF, &sf) < 0) perror("Lỗi Set SF");
    else printf("3. Đã đặt Spreading Factor: %d\n", sf);

    if (ioctl(fd, LORA_IOC_SET_SYNC_WORD, &sync_word) < 0) perror("Lỗi Set Sync Word");
    else printf("4. Đã đặt Sync Word: 0x%02X\n", sync_word);

    // 4. Test gửi dữ liệu (Transmit)
    memset(&tx_pkt, 0, sizeof(struct lora_packet));
    sprintf((char*)tx_pkt.payload, "Hello from HCMUT Student!");
    tx_pkt.size = strlen((char*)tx_pkt.payload);

    printf("5. Đang gửi dữ liệu: '%s' (%d bytes)...\n", tx_pkt.payload, tx_pkt.size);
    if (ioctl(fd, LORA_IOC_TRANSMIT, &tx_pkt) < 0) {
        perror("Lỗi IOCTL Transmit");
    } else {
        printf("   => Gửi thành công!\n");
    }

    // 5. Đọc RSSI
    if (ioctl(fd, LORA_IOC_GET_RSSI, &rssi) < 0) {
        perror("Lỗi IOCTL Get RSSI");
    } else {
        printf("6. Cường độ tín hiệu hiện tại (RSSI): %d dBm\n", rssi);
    }

    // 6. Test nhận dữ liệu (Receive)
    printf("7. Đang chờ nhận dữ liệu...\n");
    memset(&rx_pkt, 0, sizeof(struct lora_packet));
    if (ioctl(fd, LORA_IOC_RECEIVE, &rx_pkt) < 0) {
        perror("Lỗi IOCTL Receive");
    } else {
        printf("   => Đã nhận: '%s' (Size: %d bytes)\n", rx_pkt.payload, rx_pkt.size);
    }

    // 7. Đóng thiết bị
    close(fd);
    printf("--- Kết thúc bài test ---\n");

    return 0;
}