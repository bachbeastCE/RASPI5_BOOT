#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define DEVICE_NAME "/dev/d6t"
#define PIXEL_COUNT 1024
#define RAW_SIZE (PIXEL_COUNT + 1) // 1 PTAT + 1024 pixel

#define D6T_IOC_MAGIC 'x'
#define D6T_IOC_READ_RAW _IOR(D6T_IOC_MAGIC, 1, uint16_t *)

// ANSI Color Codes
#define RESET   "\033[0m"
#define PURPLE  "\033[35m"
#define BLUE    "\033[34m"
#define CYAN    "\033[36m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define ORANGE  "\033[91m"
#define RED     "\033[31m"

// Hàm lấy màu theo nhiệt độ
const char* get_color(float t) {
    if (t < 20)       return PURPLE;
    else if (t < 25)  return BLUE;
    else if (t < 30)  return CYAN;
    else if (t < 35)  return GREEN;
    else if (t < 40)  return YELLOW;
    else if (t < 45)  return ORANGE;
    else              return RED;
}

int main(int argc, char *argv[])
{
    int mode = 1; // mặc định là in ô vuông
    if (argc > 1) {
        mode = atoi(argv[1]); // ./program 0 hoặc ./program 1
    }

    int fd = open(DEVICE_NAME, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    uint16_t raw_buf[RAW_SIZE];

    // Clear screen
    printf("\033[2J\033[H");

    while (1) {
        if (ioctl(fd, D6T_IOC_READ_RAW, raw_buf) < 0) {
            perror("ioctl");
            break;
        }

        // Cursor top-left
        printf("\033[H");

        printf("PTAT = %3.1f [*C]\n", raw_buf[0] / 10.0);

        for (int row = 0; row < 32; row++) {
            for (int col = 0; col < 32; col++) {
                float t = raw_buf[1 + row * 32 + col] / 10.0;
                const char *color = get_color(t);

                if (mode == 0) {
                    // In số có màu
                    printf("%s%4.1f%s ", color, t, RESET);
                } else {
                    // In ô vuông màu
                    printf("%s██%s ", color, RESET);
                }
            }
            printf("\n");
        }

        printf("-------------------------\n");

        fflush(stdout);
        usleep(400000);
    }

    close(fd);
    return 0;
}