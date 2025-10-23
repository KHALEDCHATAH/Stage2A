// g++ speed_master_bytes.cpp -lpigpio -lrt -lpthread -o speed_master_bytes
#include <pigpio.h>
#include <cstdint>
#include <cstdio>
#include <csignal>

constexpr unsigned BUS = 1;       // /dev/i2c-1
constexpr unsigned ADR = 0x09;    // must match Arduino

volatile bool running = true;
void sigint(int){ running = false; }

/* helper: send two int8_t values */
int sendSpeeds(int handle, int8_t left, int8_t right)
{
    int8_t out[2] = { left, right };
    return i2cWriteDevice(handle, (char*)out, 2);   // returns 2 on success
}

int main()
{
    if (gpioInitialise() < 0) { perror("pigpio"); return 1; }
    int h = i2cOpen(BUS, ADR, 0);
    if (h < 0) { perror("i2cOpen"); gpioTerminate(); return 1; }

    signal(SIGINT, sigint);
    printf("Sending byte speeds…  (Ctrl-C to stop)\n");

    int8_t l = 50, r = 50;        // demo pattern
    while (running) {
        /* Example: toggle right thruster between +50 and –30 */
        r = (r == 50) ? -30 : 50;

        int rc = sendSpeeds(h, l, r);
        if (rc < 0) fprintf(stderr, "I2C write error (rc=%d)\n", rc);

        gpioDelay(10000);     // 100 Hz update rate
    }

    i2cClose(h);
    gpioTerminate();
}
