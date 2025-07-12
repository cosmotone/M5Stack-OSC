/**
 * @file SendGyroPro.ino
 * @brief Send M5 IMU Pro Mini data via OSC messages.
 *
 * @Hardwares:M5StickCPlus2 with M5 IMU Pro Mini
 * @Dependent Library:
 * M5_IMU_PRO: https://github.com/m5stack/M5Unit-IMU-Pro-Mini
 * M5StickCPlus2 library: https://github.com/m5stack/M5StickCPlus2
 * OSC library: https://github.com/CNMAT/OSC
 * Adafruit AHRS: https://github.com/adafruit/Adafruit_AHRS
 */

#include <M5StickCPlus2.h>
#include "M5_IMU_PRO.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Adafruit_AHRS_Mahony.h>

#include "wifi_config.h" // config with WiFi network's information

// Mahony filter
Adafruit_Mahony filter;

// IMU Pro
#define BIM270_SENSOR_ADDR 0x68

BMI270::BMI270 bmi270;

// Set interval at which the data will be gathered and sent (in ms)
const int interval = 100;

// Variables for the timer interrupt
volatile int interruptCounter;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// WiFi setup
const char *ssid = SSID;            // WiFi network's name
const char *password = PASS;        // WiFi password
const IPAddress outIp = OUT_IP;     // IP address of the receiving device
const unsigned int outPort = 8000;  // receiving device input port

// Object for OSC/UDP
WiFiUDP udp;

// Network port for incoming messages
const unsigned int inPort = 8000; // This is not strictly needed for sending messages

// Variables for storing the different bits of IMU data
float accX = 0.f;
float accY = 0.f;
float accZ = 0.f;

float gyroX = 0.f;
float gyroY = 0.f;
float gyroZ = 0.f;

float magX = 0;
float magY = 0;
float magZ = 0;

float pitch = 0.f;
float roll = 0.f;
float yaw = 0.f;

float w = 0.f;
float x = 0.f;
float y = 0.f;
float z = 0.f;

float offsetAccX = 0.f;
float offsetAccY = 0.f;
float offsetAccZ = 0.f;

float compAccX = 0.f;
float compAccY = 0.f;
float compAccZ = 0.f;

//=============================================================
// FUNCTIONS

/// Connect to WiFi
/// - Function for connecting to WiFi network
/// @return status (bool) true = successful, false = failed
bool connectToWiFi()
{
    StickCP2.Display.print("Connecting");

    // initialise - WIFI_STA = Station Mode
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // start timer
    unsigned long startAttemptTime = millis();

    // while not connected to WiFi AND before timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
        StickCP2.Display.print(".");
        delay(400);
    }

    // Print status to LCD
    if (WiFi.status() != WL_CONNECTED) {
        StickCP2.Display.println("\nErr: Failed to connect");
        delay(2000);
        return false;
    } else {
        StickCP2.Display.println("\nConnected to:");
        StickCP2.Display.println(ssid);
        StickCP2.Display.println(WiFi.localIP());

        delay(2000);
        return true;
    }
}

/// Send float OSC Message
/// - function for sending float messages via OSC
/// @param address (const char*) message tag
/// @param message (float) message to send
void sendFloatOscMessage(const char *address, float message)
{
    // init message
    OSCMessage oscMsg(address);
    oscMsg.add(message);

    // send message
    udp.beginPacket(outIp, outPort);
    oscMsg.send(udp);

    // clear message
    udp.endPacket();
    oscMsg.empty();
}

void sendVec3OscMessage(const char *address, float y, float p, float r)
{
    OSCMessage oscMsg(address);
    oscMsg.add(y);
    oscMsg.add(p);
    oscMsg.add(r);

    udp.beginPacket(outIp, outPort);
    oscMsg.send(udp);

    udp.endPacket();
    oscMsg.empty();
}

void sendQuaternionsOscMessage(const char *address, float qw, float qx, float qy, float qz)
{
    OSCMessage oscMsg(address);
    oscMsg.add(qw);
    oscMsg.add(qx);
    oscMsg.add(qy);
    oscMsg.add(qz);

    udp.beginPacket(outIp, outPort);
    oscMsg.send(udp);

    udp.endPacket();
    oscMsg.empty();
}


void calibratePosition()
{
    StickCP2.Speaker.tone(8000, 20);
    StickCP2.Display.clear();
    StickCP2.Display.setCursor(40, 30);
    StickCP2.Display.println("Calibrating position");
    filter.setQuaternion(1.0f, 0.0f, 0.0f, 0.0f); // Assuming that the M5StickCPlus2 is positioned with its screen facing up
    StickCP2.Display.setCursor(0, 40);
    // Measures accelerometer values within (100 * 10) ms window
    int calibrationCount = 100;
    for (int i = 0; i < calibrationCount ; i++) {
        offsetAccX += accX;
        offsetAccY += accY;
        offsetAccZ += (accZ-1.0f); // Z=1 when M5StickCPlus2 is positioned with its screen facing up
        StickCP2.Display.print("-");
        delay(10);
    }
    // Returns average accelerometer values
    offsetAccX /= calibrationCount;
    offsetAccY /= calibrationCount;
    offsetAccZ /= calibrationCount;
    StickCP2.Display.setCursor(40, 70);
    StickCP2.Display.printf("%6.2f  %6.2f  %6.2f      ", offsetAccX, offsetAccY, offsetAccZ);
    delay(1000);
    StickCP2.Display.clear();
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    auto cfg = M5.config();
    StickCP2.begin(cfg);

    StickCP2.Ex_I2C.begin();

    bmi270.init(I2C_NUM_0, BIM270_SENSOR_ADDR);

    filter.begin(1000/interval); // Set filter update frequency (Hz)

    // Connect to WiFi network
    connectToWiFi();
    udp.begin(inPort);

    delay(1000);
    StickCP2.Display.fillScreen(BLACK);

    // Display setup
    StickCP2.Display.setRotation(3);
    StickCP2.Display.fillScreen(BLACK);
    StickCP2.Display.setTextSize(1);

    // Use first (0) hardware timer on ESP32
    // Counts every microsecond 
    // true: count up 
    timer = timerBegin(0 ,getApbFrequency() / 1000000 ,true );

    // Set timer interrupt 
    timerAttachInterrupt(timer, &onTimer, true );

    // Set timer in microseconds 
    timerAlarmWrite(timer, 1000 * interval, true );

    // Start the timer
    timerAlarmEnable(timer);
}

//=============================================================
// LOOP
void loop()
{
    if (interruptCounter > 0) {
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);
        
        // 1. GET IMU DATA
        if (bmi270.accelerationAvailable() && bmi270.gyroscopeAvailable() && bmi270.magneticFieldAvailable()) {
            // Acceleration data from BMI270
            bmi270.readAcceleration(accX, accY, accZ);
            accX = accX * -1.0f; // Invert acc X axis
            accY = accY * -1.0f; // Invert acc Y axis

            // Gyroscope data from BMI270
            bmi270.readGyroscope(gyroX, gyroY, gyroZ);
            gyroX = gyroX * -1.0f; // Invert gyro X axis
            gyroY = gyroY * -1.0f; // Invert gyro Y axis

            // Magnetometer data from BMI270(BMM150)
            bmi270.readMagneticField(magX, magY, magZ);
            magY = magY * -1.0f; // Invert mag Y axis

            // Compensate accelerometer values with the average offset measured during calibration
            compAccX = accX - offsetAccX;
            compAccY = accY - offsetAccY;
            compAccZ = accZ - offsetAccZ;

            // Calculate pitch, roll and yaw based on accelerometer, gyroscope and magnetometer readings using Mahony's AHRS algorithm
            filter.update(gyroX, gyroY, gyroZ, compAccX, compAccY, compAccZ, magX, magY, magZ);

            pitch = filter.getPitch();
            roll = filter.getRoll();
            yaw = filter.getYaw() - 180.0f; // move yaw range to -180/+180

            // Get quaternions representing the rotation
            filter.getQuaternion(&w, &x, &y, &z);
        }

        StickCP2.update();
        // Trigger calibration routine when button A is pressed
        if (StickCP2.BtnA.wasPressed()) {
            calibratePosition();
        }

        // 2. PRINT DATA TO M5 LCD (optional)
        StickCP2.Display.setCursor(80, 15);
        StickCP2.Display.println("SEND GYRO PRO");

        StickCP2.Display.setCursor(30, 30);
        StickCP2.Display.println("  X       Y       Z");

        // Gyroscope data
        StickCP2.Display.setCursor(30, 40);
        StickCP2.Display.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
        StickCP2.Display.setCursor(170, 40);
        StickCP2.Display.print("o/s");

        // Accelerometer data
        StickCP2.Display.setCursor(30, 50);
        StickCP2.Display.printf(" %5.2f   %5.2f   %5.2f   ", accX, accY, accZ);
        StickCP2.Display.setCursor(170, 50);
        StickCP2.Display.print("G");

        // Magnetometer data
        StickCP2.Display.setCursor(30, 60);
        StickCP2.Display.printf(" %5.2f   %5.2f   %5.2f   ", magX, magY, magZ);
        StickCP2.Display.setCursor(170, 60);
        StickCP2.Display.print("uT");

        // Calculated AHRS
        StickCP2.Display.setCursor(30, 80);
        StickCP2.Display.println("  Pitch   Roll    Yaw");
        StickCP2.Display.setCursor(30, 90);
        StickCP2.Display.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);

        // 3. SEND DATA VIA OSC
        // Gyroscope data
        sendVec3OscMessage("/gyro", gyroX, gyroY, gyroZ);

        // Accelerometer data
        sendVec3OscMessage("/acc", accX, accY, accZ);

        // Magnetometer data
        sendVec3OscMessage("/mag", magX, magY, magZ);

        // Calculated AHRS
        sendVec3OscMessage("/ypr", yaw, pitch, roll);

        // Calculated quaternions
        sendQuaternionsOscMessage("/SceneRotator/quaternions", w, x, y, z);
    }
}
