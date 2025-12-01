//=============================================================
// File:    SendGyro.ino
//
// Dependencies:
//     - M5StickCPlus2 library (M5Stack)
//     - OSC library (Adrian Freed)
//     - Adafruit AHRS (Adafruit)
//
// Description:
//     - Send M5StickCPlus2 IMU data via OSC messages
//=============================================================

#include <M5Unified.h>
#include <M5GFX.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Adafruit_AHRS_Mahony.h>

#include "wifi_config.h" // config with WiFi network's information

M5GFX display;

// Mahony filter
Adafruit_Mahony filter;

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
    display.print("Connecting");

    // initialise - WIFI_STA = Station Mode
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // start timer
    unsigned long startAttemptTime = millis();

    // while not connected to WiFi AND before timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
        Display.print(".");
        delay(400);
    }

    // Print status to LCD
    if (WiFi.status() != WL_CONNECTED) {
        Display.println("\nErr: Failed to connect");
        delay(2000);
        return false;
    } else {
        Display.println("\nConnected to:");
        Display.println(ssid);
        Display.println(WiFi.localIP());

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


void calibratePosition()
{
    Speaker.tone(8000, 20);
    Display.clear();
    Display.setCursor(40, 30);
    Display.println("Calibrating position");
    filter.setQuaternion(1.0f, 0.0f, 0.0f, 0.0f); // Assuming that the M5StickCPlus2 is positioned with its screen facing up
    Display.setCursor(0, 40);
    // Measures accelerometer values within (100 * 10) ms window
    int calibrationCount = 100;
    for (int i = 0; i < calibrationCount ; i++) {
        offsetAccX += accX;
        offsetAccY += accY;
        offsetAccZ += (accZ-1.0f); // Z=1 when M5StickCPlus2 is positioned with its screen facing up
        Display.print("-");
        delay(10);
    }
    // Returns average accelerometer values
    offsetAccX /= calibrationCount;
    offsetAccY /= calibrationCount;
    offsetAccZ /= calibrationCount;
    Display.setCursor(40, 70);
    Display.printf("%6.2f  %6.2f  %6.2f      ", offsetAccX, offsetAccY, offsetAccZ);
    delay(1000);
    Display.clear();
}

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

//=============================================================
// SETUP
void setup()
{
    auto cfg = M5.config();
    begin(cfg);

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
    timer = timerBegin(1000000 );

    // Set timer interrupt 
    timerAttachInterrupt(timer, &onTimer, true );

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
        auto imu_update = Imu.update();
        if (imu_update) {
            auto data = Imu.getImuData();

            // The data obtained by getImuData can be used as follows.
            accX = data.accel.x; // accel x-axis value.
            accY = data.accel.y; // accel y-axis value.
            accZ = data.accel.z; // accel z-axis value.
            // data.accel.value; // accel 3values array [0]=x / [1]=y / [2]=z.

            gyroX = data.gyro.x; // gyro x-axis value.
            gyroY = data.gyro.y; // gyro y-axis value.
            gyroZ = data.gyro.z; // gyro z-axis value.
            // data.gyro.value;  // gyro 3values array [0]=x / [1]=y / [2]=z.

            // pitch = data.mag.x;  // mag x-axis value.
            // roll = data.mag.y;   // mag y-axis value.
            // yaw = data.mag.z;    // mag z-axis value.
            // data.mag.value;   // mag 3values array [0]=x / [1]=y / [2]=z.

            // data.value;       // all sensor 9values array [0~2]=accel / [3~5]=gyro / [6~8]=mag

            // Compensate accelerometer values with the average offset measured during calibration
            compAccX = accX - offsetAccX;
            compAccY = accY - offsetAccY;
            compAccZ = accZ - offsetAccZ;

            // Calculate pitch, roll and yaw based on accelerometer and gyroscope readings using Mahony's AHRS algorithm
            filter.updateIMU(gyroX, gyroY, gyroZ, compAccX, compAccY, compAccZ);

            pitch = filter.getPitch();
            roll = filter.getRoll();
            yaw = filter.getYaw() - 180.0f; // move yaw range to -180/+180

            // Get quaternions representing the rotation
            filter.getQuaternion(&w, &x, &y, &z);
        }

        update();
        // Trigger calibration routine when button A is pressed
        if (BtnA.wasPressed()) {
            calibratePosition();
        }

        // 2. PRINT DATA TO M5 LCD (optional)
        Display.setCursor(80, 15);
        Display.println("SEND GYRO");

        Display.setCursor(30, 30);
        Display.println("  X       Y       Z");

        // Gyroscope data
        Display.setCursor(30, 40);
        Display.printf("%6.2f  %6.2f  %6.2f      ", gyroX, gyroY, gyroZ);
        Display.setCursor(170, 40);
        Display.print("o/s");

        // Accelerometer data
        Display.setCursor(30, 50);
        Display.printf(" %5.2f   %5.2f   %5.2f   ", compAccX, compAccY, compAccZ);
        Display.setCursor(170, 50);
        Display.print("G");

        Display.setCursor(30, 70);
        Display.println("  Pitch   Roll    Yaw");

        // Calculated AHRS
        Display.setCursor(30, 80);
        Display.printf(" %5.2f   %5.2f   %5.2f   ", pitch, roll, yaw);

        // 3. SEND DATA VIA OSC
        // Gyroscope data
        sendVec3OscMessage("/gyro", gyroX, gyroY, gyroZ);

        // Accelerometer data
        sendVec3OscMessage("/acc", compAccX, compAccY, compAccZ);

        // Calculated AHRS
        sendVec3OscMessage("/ypr", yaw, pitch, roll);

        // Calculated quaternions
        sendVec4OscMessage("/SceneRotator/quaternions", w, x, y, z);
    }
}
