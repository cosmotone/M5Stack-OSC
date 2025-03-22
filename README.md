# M5Stack-OSC
Sending OSC data from M5StickCPlus2 over local network

### Structure

Projects have Arduino IDE sketches structure

### Prerequisites
To make a project work you will need to create `wifi_config.h` file in this projects folder.  
The file must have the following structure:

```cpp
#define SSID "your-wifi-network-name"
#define PASS "your-wifi-network-password"
#define OUT_IP { 127, 0, 0, 1 }
```
Where `SSID` and `PASS` reflect your WiFi's name and password and `OUT_IP` matches IPv4 address of the device that should receive the OSC data.

## Projects

### SendGyro
Script that will send M5StickCPlus2 onboard IMU (MPU6886) data via OSC messages.  
Keep in mind that the MPU6886 IMU does not have geomagnetic sensor so the AHRS data and quaternions are calculated using [Mahony's AHRS algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)  

Created based on: [Edinburgh-College-of-Art/m5stickc-plus-introduction/.../SendFloat_IMU.ino](https://github.com/Edinburgh-College-of-Art/m5stickc-plus-introduction/blob/main/examples/Wireless-Communication/OSC/SendFloat_IMU/SendFloat_IMU.ino)

The following data is transmitted:  
 - `/gyroX` `/gyroY` `/gyroZ` - gyroscope data  
 - `/accX` `/accY` `/accZ` - accelerometer data  
 - `/pitch` `/roll` `/yaw` - attitude and heading reference system (AHRS) data
 - `/w/` `/x` `/y` `/z` - quaternions representing rotation


## Dependencies

 - [M5StickCPlus2 library (M5Stack)](https://github.com/m5stack/M5StickCPlus2)
 - [OSC library (Adrian Freed)](https://github.com/CNMAT/OSC)
 - [Adafruit AHRS (Adafruit)](https://github.com/adafruit/Adafruit_AHRS)
