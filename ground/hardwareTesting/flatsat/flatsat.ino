#include <Adafruit_SCD30.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "RTClib.h"

#define SEALEVELPRESSURE_HPA (1013.25)

//#define CAM_A_CS 4
//#define CAM_B_CS 5
#define SD_CS 15
#define SCD_RDY 27

// TODO:
// Add analog read of the battery and thermistor
// Create folder for each run
// Add camera code

Adafruit_SCD30  scd30;
Adafruit_BMP3XX bmp;
RTC_DS1307 rtc;

//SPIClass *vspi;
SPIClass *hspi;

bool data_ready = false;

void IRAM_ATTR scd30_ready() {
  data_ready = true;
}

void setup() {
  //vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);

  Wire.begin();



  Serial.begin(115200);
  while (!Serial);
  Serial.println("UCF KSC Test Logger");

  // BMP388 SETUP
  Serial.println("Setting up BMP sensor");
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor.");
    return;
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // SCD30 SETUP
  Serial.println("Setting up SCD30");
  if (!scd30.begin()) {
    Serial.println("Failed to find SCD30 chip");
    return;
  }

  // RTC SETUP
  Serial.println("Setting up RTC");
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    return;
  }

  // SD CARD SETUP
  Serial.println("Setting up SD card");
  if(!SD.begin(SD_CS, *hspi)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }

  // Configure SCD30 RDY interrupt
  pinMode(SCD_RDY, INPUT_PULLDOWN);
  attachInterrupt(SCD_RDY, scd30_ready, RISING);
}

void loop() {
  while(!data_ready) { delay(10); }
  if (! scd30.read()) { Serial.println("Error reading SCD30 data"); return; }
  if (! bmp.performReading()) { Serial.println("Error reading BMP388 data"); return; }
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting date/time to 1-1-23/00:00:00");
    rtc.adjust(DateTime(2023, 1, 1, 0, 0, 0));
  }

  File file = SD.open("/datalog.csv", FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }

  // date
  DateTime now = rtc.now();
  file.print(now.month());
  file.print("/");
  file.print(now.day());
  file.print("/");
  file.print(now.year());
  file.print(",");
  
  // time
  file.print(now.hour());
  file.print(":");
  file.print(now.minute());
  file.print(":");
  file.print(now.second());
  file.print(",");

  // temp
  file.print(bmp.temperature);
  file.print(",");
  Serial.print("Temp_(C):");
  Serial.print(bmp.temperature);
  Serial.print(",");

  // pressure
  file.print(bmp.pressure / 100);
  file.print(",");
  Serial.print("Pressure_(hPa):");
  Serial.print(bmp.pressure / 100);
  Serial.print(",");

  // altitude
  file.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  file.print(",");
  Serial.print("Altitude_(m):");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.print(",");

  // humidity
  file.print(scd30.relative_humidity);
  file.print(",");
  Serial.print("Relative_Humidity:");
  Serial.print(scd30.relative_humidity);
  Serial.print(",");

  // co2
  file.print(scd30.CO2, 3);
  file.println();
  Serial.print("CO2_(ppm):");
  Serial.print(scd30.CO2, 3);
  Serial.println();

  file.close();
  
  data_ready = false;
}