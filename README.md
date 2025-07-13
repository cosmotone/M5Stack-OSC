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
Receiving device is expected to listen for incoming data on port `8000`

## Projects

### SendGyro
Script that will send M5StickCPlus2 onboard IMU (MPU6886) data via OSC messages.  
Keep in mind that the MPU6886 IMU does not have geomagnetic sensor so the AHRS data and quaternions are calculated using [Mahony's AHRS algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)  

Created based on: [Edinburgh-College-of-Art/m5stickc-plus-introduction/.../SendFloat_IMU.ino](https://github.com/Edinburgh-College-of-Art/m5stickc-plus-introduction/blob/main/examples/Wireless-Communication/OSC/SendFloat_IMU/SendFloat_IMU.ino)

#### Calibration

There is a calibration routine for setting the 'zero' (no rotation) positon - it can be started by pressing the A button.  
During this calibration (1 second) the M5StickCPlus2 should be **stationary**

#### Data

The following data is transmitted:  
 - `/ypr` - calculated attitude and heading reference system (AHRS) - formatted for [SPARTA Rotator](https://leomccormack.github.io/sparta-site/docs/plugins/sparta-suite/#rotator)  
 - `/SceneRotator/quaternions` - calculated quaternions representing rotation - formatted for [IEM Scene Rotator](https://plugins.iem.at/docs/osc/#scenerotator)  

It's also possible to transmit:  
 - `/gyro` - gyroscope data  
 - `/acc` - accelerometer data

### SendGyroPro
Script that will send M5 IMU Pro (BMI270, BMM150 and BMP280) data via OSC messages.  
The AHRS data and quaternions are calculated using [Mahony's AHRS algorithm](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)  
This sketch has one more dependecy:
 - [IMU Pro (M5Stack)](https://github.com/m5stack/M5Unit-IMU-Pro-Mini)

Once you install the above library you should [make a change in it so that the magnetometer data will have a float type](https://github.com/9PEO0xNd/M5Unit-IMU-Pro-Mini/commit/acf6e0a9b6a6189297270687db21260394aac212)

#### Calibration

There is a calibration routine for setting the 'zero' (no rotation) positon - it can be started by pressing the A button.  
During this calibration (1 second) the M5StickCPlus2 should be **stationary**  

There is also a magnetometer calibration routine. It is used to normalize the data comming from magnetometer. It can be started by pressing the B button.  
During this calibration (30 seconds) the M5 IMU Pro sensor should be **gradually rotated and turned** so that each side of the sensor is facing 'up' at least once.  
*Keed in mind that there is a __magnet__ inside of the M5StickCPlus2, so try to move the sensor away from the M5StickCPlus2*

#### Data

The following data is transmitted:  
 - `/ypr` - calculated attitude and heading reference system (AHRS) - formatted for [SPARTA Rotator](https://leomccormack.github.io/sparta-site/docs/plugins/sparta-suite/#rotator)  
 - `/SceneRotator/quaternions` - calculated quaternions representing rotation - formatted for [IEM Scene Rotator](https://plugins.iem.at/docs/osc/#scenerotator)  

It's also possible to transmit:  
 - `/gyro` - gyroscope data  
 - `/acc` - accelerometer data
 - `/mag` - magnetometer data  


## Dependencies

 - [M5StickCPlus2 library (M5Stack)](https://github.com/m5stack/M5StickCPlus2)
 - [OSC library (Adrian Freed)](https://github.com/CNMAT/OSC)
 - [Adafruit AHRS (Adafruit)](https://github.com/adafruit/Adafruit_AHRS)
