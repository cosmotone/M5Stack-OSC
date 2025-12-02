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

#include <M5Unified.h>
//#include "M5_IMU_PRO.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Adafruit_AHRS_Mahony.h>

#include "wifi_config.h" // config with WiFi network's information

// Mahony filter
Adafruit_Mahony filter;

// IMU Pro
//#define BIM270_SENSOR_ADDR 0x68

//BMI270::BMI270 bmi270;

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

// Magnetometer data range
float minMag = -50.f;
float maxMag = 50.f;

// Variables for storing the different bits of IMU data
float accX = 0.f;
float accY = 0.f;
float accZ = 0.f;

float gyroX = 0.f;
float gyroY = 0.f;
float gyroZ = 0.f;

float magX = 0.f;
float magY = 0.f;
float magZ = 0.f;

float minMagX = minMag;
float maxMagX = maxMag;
float minMagY = minMag;
float maxMagY = maxMag;
float minMagZ = minMag;
float maxMagZ = maxMag;

float normMagX = 0.f;
float normMagY = 0.f;
float normMagZ = 0.f;

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
    M5.Display.print("Connecting");

    // initialise - WIFI_STA = Station Mode
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // start timer
    unsigned long startAttemptTime = millis();

    // while not connected to WiFi AND before timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
        M5.Display.print(".");
        delay(400);
    }

    // Print status to LCD
    if (WiFi.status() != WL_CONNECTED) {
        M5.Display.println("\nErr: Failed to connect");
        delay(2000);
        return false;
    } else {
        M5.Display.println("\nConnected to:");
        M5.Display.println(ssid);
        M5.Display.println(WiFi.localIP());

        delay(2000);
        return true;
    }
}

/// Send float OSC Message
/// - function for sending float messages via OSC
/// @param address (const char*) message tag
/// @param messX (float) x component of the message to send
/// @param messY (float) y component of the message to send
/// @param messZ (float) z component of the message to send
void sendVec3OscMessage(const char *address, float messX, float messY, float messZ)
{
    // init message
    OSCMessage oscMsg(address);
    oscMsg.add(messX);
    oscMsg.add(messY);
    oscMsg.add(messZ);

    // send message
    udp.beginPacket(outIp, outPort);
    oscMsg.send(udp);

    // clear message
    udp.endPacket();
    oscMsg.empty();
}

void sendVec4OscMessage(const char *address, float messX, float messY, float messZ, float messW)
{
    OSCMessage oscMsg(address);
    oscMsg.add(messX);
    oscMsg.add(messY);
    oscMsg.add(messZ);
    oscMsg.add(messW);

    udp.beginPacket(outIp, outPort);
    oscMsg.send(udp);

    udp.endPacket();
    oscMsg.empty();
}

void getImuData()
{
   // Obtain data on the current value of the IMU.
    auto data = M5.Imu.getImuData();

    // Acceleration data from BMI270
    accX = data.accel.x * -1.0f; // Invert acc X axis
    accY = data.accel.y * -1.0f; // Invert acc Y axis

    // Gyroscope data from BMI270
    gyroX = data.gyro.x * -1.0f; // Invert gyro X axis
    gyroY = data.gyro.y * -1.0f; // Invert gyro Y axis

    // Magnetometer data from BMI270(BMM150)
    magY = data.mag.y * -1.0f; // Invert mag Y axis
}

void calibratePosition()
{
    M5.Speaker.tone(8000, 20);
    M5.Display.clear();
    M5.Display.setCursor(40, 30);
    M5.Display.println("Calibrating position");
    filter.setQuaternion(1.0f, 0.0f, 0.0f, 0.0f); // Assuming that the M5StickCPlus2 is positioned with its screen facing up
    M5.Display.setCursor(0, 40);
    // Measures accelerometer values within (100 * 10) ms window
    int calibrationCount = 100;
    for (int i = 0; i < calibrationCount ; i++) {
        if (M5.Imu.accelerationAvailable() && M5.Imu.gyroscopeAvailable() && M5.Imu.magneticFieldAvailable()) {
            getImuData();
            offsetAccX += accX;
            offsetAccY += accY;
            offsetAccZ += (accZ-1.0f); // Z=1 when M5StickCPlus2 is positioned with its screen facing up
        }
        M5.Display.print("-");
        delay(10);
    }
    // Returns average accelerometer values
    offsetAccX /= calibrationCount;
    offsetAccY /= calibrationCount;
    offsetAccZ /= calibrationCount;
    M5.Display.setCursor(40, 70);
    M5.Display.printf("%6.2f  %6.2f  %6.2f      ", offsetAccX, offsetAccY, offsetAccZ);
    delay(1000);
    M5.Display.clear();
}

void calibrateMagnetometer()
{
    M5.Speaker.tone(8000, 20);
    delay(200);
    M5.Speaker.tone(8000, 20);
    M5.Display.clear();
    M5.Display.setCursor(40, 30);
    M5.Display.println("Calibrating magnetometer");
    M5.Display.setCursor(0, 40);
    // Initialize min and max values
    minMagX = magX;
    maxMagX = magX;
    minMagY = magY;
    maxMagY = magY;
    minMagZ = magZ;
    maxMagZ = magZ;
    // Measures magnetometer values within (300 * 100) ms window
    int calibrationCount = 300;
    for (int i = 0; i < calibrationCount ; ++i) {
        if (M5.Imu.accelerationAvailable() && M5.Imu.gyroscopeAvailable() && M5.Imu.magneticFieldAvailable()) {
            getImuData();
            if (magX < minMagX) minMagX = magX;
            if (magX > maxMagX) maxMagX = magX;
            if (magY < minMagY) minMagY = magY;
            if (magY > maxMagY) maxMagY = magY;
            if (magZ < minMagZ) minMagZ = magZ;
            if (magZ > maxMagZ) maxMagZ = magZ;
        }
        M5.Display.print(".");
        delay(100);
    }
    M5.Speaker.tone(8000, 20);
    delay(200);
    M5.Speaker.tone(8000, 20);
    M5.Display.clear();
    M5.Display.setCursor(40, 30);
    M5.Display.println("Calibrating magnetometer");
    M5.Display.setCursor(40, 50);
    M5.Display.printf("%6.2f  %6.2f  %6.2f      ", minMagX, minMagY, minMagZ);
    M5.Display.setCursor(40, 60);
    M5.Display.printf("%6.2f  %6.2f  %6.2f      ", maxMagX, maxMagY, maxMagZ);
    delay(2000);
    M5.Display.clear();
}

float normalizeMag(float value, float inMin, float inMax)
{
    float scaled = (value - inMin) / (inMax - inMin);
    return minMag + scaled * (maxMag - minMag);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    auto cfg = M5.config();

    //Activate external IMU
    cfg.external_imu = true;

    M5.begin(cfg);

    M5.Imu.begin();

    //M5.Imu.init(I2C_NUM_0, BIM270_SENSOR_ADDR);

    filter.begin(1000/interval); // Set filter update frequency (Hz)

    // Connect to WiFi network
    connectToWiFi();
    udp.begin(inPort);

    delay(1000);
    M5.Display.fillScreen(BLACK);

    // Display setup
    M5.Display.setRotation(3);
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(1);

    // Use first (0) hardware timer on ESP32
    // Counts every microsecond 
    // true: count up 
    timer = timerBegin(1000000 );

    // Set timer interrupt 
    timerAttachInterrupt(timer, &onTimer );

    // Set timer in microseconds 
    timerAlarm(timer, 1000 * interval, true, 0 );
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
        if (M5.Imu.accelerationAvailable() && M5.Imu.gyroscopeAvailable() && M5.Imu.magneticFieldAvailable()) {
            getImuData();

            // Compensate accelerometer values with the average offset measured during calibration
            compAccX = accX - offsetAccX;
            compAccY = accY - offsetAccY;
            compAccZ = accZ - offsetAccZ;

            // Normalize magnetometer values
            normMagX = normalizeMag(magX, minMagX, maxMagX);
            normMagY = normalizeMag(magY, minMagY, maxMagY);
            normMagZ = normalizeMag(magZ, minMagZ, maxMagZ);

            // Calculate pitch, roll and yaw based on accelerometer, gyroscope and magnetometer readings using Mahony's AHRS algorithm
            filter.update(gyroX, gyroY, gyroZ, compAccX, compAccY, compAccZ, normMagX, normMagY, normMagZ);

            pitch = filter.getPitch();
            roll = filter.getRoll();
            yaw = filter.getYaw() - 180.0f; // move yaw range to -180/+180

            // Get quaternions representing the rotation
            filter.getQuaternion(&w, &x, &y, &z);
        }

        M5.update();
        // Trigger accelerometer calibration routine when button A is pressed
        if (M5.BtnA.wasPressed()) {
            calibratePosition();
        }

        // Trigger magnetometer calibration routine when button B is pressed
        if (M5.BtnB.wasPressed()) {
            calibrateMagnetometer();
        }

        // 2. PRINT DATA TO M5 LCD (optional)
        M5.Display.setCursor(80, 15);
        M5.Display.println("SEND GYRO PRO");

        M5.Display.setCursor(30, 30);
        M5.Display.println("  X       Y       Z");

        // Gyroscope data
        M5.Display.setCursor(30, 40);
        M5.Display.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
        M5.Display.setCursor(170, 40);
        M5.Display.print("o/s");

        // Accelerometer data
        M5.Display.setCursor(30, 50);
        M5.Display.printf(" %5.2f   %5.2f   %5.2f   ", compAccX, compAccY, compAccZ);
        M5.Display.setCursor(170, 50);
        M5.Display.print("G");

        // Magnetometer data
        M5.Display.setCursor(30, 60);
        M5.Display.printf(" %5.2f   %5.2f   %5.2f   ", normMagX, normMagY, normMagZ);
        M5.Display.setCursor(170, 60);
        M5.Display.print("uT");

        // Calculated AHRS
        M5.Display.setCursor(30, 80);
        M5.Display.println("  Pitch   Roll    Yaw");
        M5.Display.setCursor(30, 90);
        M5.Display.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);

        // 3. SEND DATA VIA OSC
        // Gyroscope data
        sendVec3OscMessage("/gyro", gyroX, gyroY, gyroZ);

        // Accelerometer data
        sendVec3OscMessage("/acc", compAccX, compAccY, compAccZ);

        // Magnetometer data
        sendVec3OscMessage("/mag", normMagX, normMagY, normMagZ);

        // Calculated AHRS
        sendVec3OscMessage("/ypr", yaw, pitch, roll);

        // Calculated quaternions
        sendVec4OscMessage("/SceneRotator/quaternions", w, x, y, z);
    }
}
