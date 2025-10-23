#include <Wire.h>
#include <RazorIMU_9DOF.h>
#include <Servo.h>
#define SLAVE_ADDR 0x09


const uint32_t IMU_BAUD_RATE    = 57600;   // Razor IMU default
const uint32_t SERIAL_BAUD_RATE = 115200;  // USB debug

const float MAXSPEED   = 127.0;
const float MAXPWM     = 2000.0;
const float MINPWM     = 1000.0;
const float NEUTRALPWM = 1500.0;
const float MINSENSPWM =   3.0;
const float COEF       = (MAXPWM - NEUTRALPWM) / MAXSPEED;

/* ---------- Setup ---------- */
RazorIMU_9DOF IMU;
Servo leftThruster;
Servo rightThruster;

const uint8_t leftThPin  = 9;
const uint8_t rightThPin = 6;

// -------------------- helpers --------------------------
static inline int mapToPWM(int speed)
{
    if (speed < -MAXSPEED) return MINPWM;
    if (speed >  MAXSPEED) return MAXPWM;
    if (speed > -MINSENSPWM && speed < MINSENSPWM)
        return NEUTRALPWM;
    return static_cast<int>(speed * COEF + NEUTRALPWM);
}

static void setThrusterSpeeds(int left, int right)
{
    leftThruster.writeMicroseconds(mapToPWM(left));
    rightThruster.writeMicroseconds(mapToPWM(right));
}

static void armThrusters()
{
    leftThruster.attach(leftThPin);
    rightThruster.attach(rightThPin);
    leftThruster.writeMicroseconds(NEUTRALPWM);
    rightThruster.writeMicroseconds(NEUTRALPWM);
    delay(2000);                             // give ESCs time to arm
}

/* ------------ IMU → Pi frame (6 bytes) ------------ */
struct __attribute__((packed)) IMUFrame {
  int16_t roll;   
  int16_t pitch;
  int16_t yaw;
};

volatile IMUFrame imuTx;          

/* ------------ Pi → Mega command (2 bytes) ---------- */
volatile int8_t leftSpeed  = 0;
volatile int8_t rightSpeed = 0;
volatile bool   newCmd     = false;

/* ---------- Wire callbacks ---------- */
void onRequest()                           // Pi does a READ
{
  Wire.write((const uint8_t*)&imuTx, sizeof(imuTx));   // 6 bytes
}

void onReceive(int count)                  // Pi does a WRITE
{
  if (count >= 2) {                        // we expect exactly 2
    leftSpeed  = (int8_t)Wire.read();
    rightSpeed = (int8_t)Wire.read();
    newCmd = true;
    while (count-- > 2) Wire.read();       // discard extras, if any
  }
}



void setup()
{
    armThrusters();
    Serial.begin(SERIAL_BAUD_RATE);          // debug
    Serial2.begin(IMU_BAUD_RATE);            // Razor IMU
    IMU.AttachIMUSerial(&Serial2);
    Serial.println(F("IMU-speed slave ready"));

    Wire.begin(SLAVE_ADDR);
    Wire.onRequest(onRequest);
    Wire.onReceive(onReceive);
}

/* ---------- Loop ---------- */
void loop()
{
    /* 1. Update IMU and prepare 6-byte frame */
    IMU.UpdateData();
    float r = IMU.GetRoll();
    float p = IMU.GetPitch();
    float y = IMU.GetYaw();

    IMUFrame local {
        (int16_t)(r * 100.0f),       
        (int16_t)(p * 100.0f),
        (int16_t)(y * 100.0f)
    };

    /* publish atomically for the ISR */
    memcpy((void*)&imuTx, &local, sizeof(local));

    /* 2. If a new speed command arrived, act on it */
    if (newCmd) {
        int8_t l = leftSpeed;         // 8-bit reads are atomic on AVR
        int8_t r = rightSpeed;
        newCmd = false;

        // --- put your thruster-drive call here ---------------
        setThrusterSpeeds(l, r);
        Serial.print(F("Cmd: L=")); Serial.print(l);
        Serial.print(F(" R="));     Serial.println(r);
    }

  /* loop pace is dominated by IMU UART latency (~3 ms) */
}
