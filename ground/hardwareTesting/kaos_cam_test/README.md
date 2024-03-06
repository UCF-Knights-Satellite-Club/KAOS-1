# KAOS Cam Test
Takes images using an ArduCam Mega SPI camera and writes them to an SD card.

## Required Libraries
- **ESP-32** by Espressif ([Install Instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html))
- **Arducam_Mega** by Arducam

## Wiring
|Chip                  |ESP32     |
|----------------------|----------|
|ArduCam SCK (White)   | 18 (D18) |
|ArduCam MISO (Brown)  | 19 (D18) |
|ArduCam MOSI (Yellow) | 23 (D23) |
|ArduCam CS (Orange)   | 17 (TX2) |
|SD CLK                | 18 (D18) |
|SD DI                 | 23 (D23) |
|SD DO                 | 19 (D18) |
|SD CS                 | 15 (D15) |
