//  g++ imu_speed_master.cpp -lpigpio -lrt -lpthread -o imu_speed_master
#include <pigpio.h>
#include <cstdint>
#include <cstdio>
#include <csignal>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
constexpr unsigned BUS = 1;        // /dev/i2c-1
constexpr unsigned ADR = 0x09;

volatile bool running = true;
void sigint(int){ running = false; }

/* helpers ----------------------------------------------------------- */
int writeSpeeds(int h, int8_t l, int8_t r)
{
    int8_t out[2] = { l, r };
    return i2cWriteDevice(h, (char*)out, 2);
}

int readIMU(int h, int16_t &roll, int16_t &pitch, int16_t &yaw)
{
    uint8_t buf[6];
    int rc = i2cReadDevice(h, (char*)buf, 6);
    if (rc == 6) {
        roll  = (int16_t)(buf[0] | (buf[1] << 8));
        pitch = (int16_t)(buf[2] | (buf[3] << 8));
        yaw   = (int16_t)(buf[4] | (buf[5] << 8));
    }
    return rc;
}

static inline int8_t parseSpeed(const char* s)
{
    long v = strtol(s, nullptr, 10);
    if (v < -127 || v > 127) {
        fprintf(stderr, "Speed %ld out of range (-127…127)\n", v);
        exit(1);
    }
    return static_cast<int8_t>(v);
}

/* main -------------------------------------------------------------- */
int main(int argc, char** argv)
{
    /* default speeds when no args given */
    int8_t left  = 50;
    int8_t right = -50;
    int multiplier = 1;
    if (argc == 2) {                     // one arg → apply to both
        left = right = parseSpeed(argv[1]);
    } else if (argc == 3) {              // two args → left right
        left  = parseSpeed(argv[1]);
        right = parseSpeed(argv[2]);
    } else if (argc > 3) {
        printf("Usage: %s [left] [right]\n", argv[0]);
        printf("       Speeds are signed bytes (-127…127)\n");
        left  = parseSpeed(argv[1]);
        right = parseSpeed(argv[2]);
        multiplier = atoi(argv[3]);
    }
    


    if (gpioInitialise() < 0) { perror("pigpio"); return 1; }
    int h = i2cOpen(BUS, ADR, 0);
    if (h < 0) { perror("i2cOpen"); gpioTerminate(); return 1; }

    signal(SIGINT, sigint);
    printf("Ctrl-C to quit\n");

    int t = 0; 
    while (running) {
        /* --- read 6-byte IMU frame --- */
        int16_t r, p, y;
        if (readIMU(h, r, p, y) == 6) {
            printf("Roll=%6.2f  Pitch=%6.2f  Yaw=%6.2f\n",
                   r / 100.0, p / 100.0, y / 100.0);
        }

        /* --- send 2-byte speed command (demo pattern) --- */
        
        writeSpeeds(h, left, right);

        gpioDelay(10 * 1000);          // 100 Hz cycle
        t++;
        if(t > 100 * multiplier){
            writeSpeeds(h, 0, 0);
            break;
        }
            
    }
    writeSpeeds(h, 0, 0);
    i2cClose(h);
    gpioTerminate();
}
