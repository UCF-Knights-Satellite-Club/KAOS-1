# KAOS-1 Project
KAOS (Knight Air Observer Satellite)-1 is a 4U CubeSat that will be launched on a weather balloon to study atmospheric chemistry.

## Hardware
TODO: add hardware description

## Software
This project utilizes the Arduino ecosystem on a **DOIT ESP32 DEVKIT V1**. Unless stated otherwise, all libraries can be found in the Arduino IDE Library Manager.

### Prerequisites
- **Arduino ESP32 Core** ([installation instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html))
- **Adafruit BMP3XX Library** by Adafruit
- **Adafruit SCD-30** by Adafruit
- **RTClib** by Adafruit
- **Arducam_Mega** by Arducam

### Explanation
The flight software has five states. The first state is `CALIBRATION` which averages several altitude readings to determine a launch altitude. Next is `PREFLIGHT` which runs when KAOS-1 is on the ground and waiting to be released on the balloon. High acceleration or breaking an altitude threshold will move to the `ASCENT` state. In this state, KAOS-1 monitors acceleration to determine when it is in free fall. In free fall, the accelerometer will read close to zero. Once this condition is met, the state moves to `FREEFALL` where altitude is compared to the deployment altitude. Once deployment altitude is reached, the a buzzer is enabled and the state switches to `LANDING`.
