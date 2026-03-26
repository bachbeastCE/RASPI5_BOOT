#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdint.h>
#include <signal.h> 
#include <errno.h>
#include "sx1278_ioctl.h"

#define DEVICE_PATH "/dev/sx1278"

volatile sig_atomic_t keep_running = 1;

void handle_sigint(int sig) {
    keep_running = 0;
}

int main() {
    int fd, rssi;
    uint8_t sync = 0xF1; // Match your sender
    struct lora_packet rx_pkt;
    struct sigaction sa;

    // 1. Setup Signal Handling (No SA_RESTART)
    sa.sa_handler = handle_sigint;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; 
    sigaction(SIGINT, &sa, NULL);

    // 2. Open Device
    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("[-] Failed to open device");
        return -1;
    }

    // 3. Hardware Config
    ioctl(fd, LORA_IOC_RESET);
    if (ioctl(fd, LORA_IOC_SET_SYNC_WORD, &sync) < 0) { // Passing pointer &sync
        perror("[-] Failed to set Sync Word");
    }

    printf("SX1278 Station Online | Sync: 0x%02X | Press Ctrl+C to Exit\n", sync);

    // 4. Main Rx Loop
    while (keep_running) {
        if (ioctl(fd, LORA_IOC_RECEIVE_CONTINUE, &rx_pkt) == 0) {
            if (rx_pkt.length > 0) {
                ioctl(fd, LORA_IOC_GET_RSSI, &rssi);
                rx_pkt.data[rx_pkt.length] = '\0'; // Null-terminate string
                printf("[RCV] Data: \"%s\" | RSSI: %d dBm\n", rx_pkt.data, rssi);
            }
        } else {
            if (errno == EINTR) {
                printf("[!] Interrupted by user. Exiting...\n");
                break; 
            }
            perror("[-] IOCTL Error");
            usleep(100000); // 100ms delay to prevent CPU hogging on error
        }
    }

    close(fd);
    return 0;
}



