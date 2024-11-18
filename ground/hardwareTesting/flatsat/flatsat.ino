#include <Adafruit_SCD30.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "RTClib.h"
#include "Arducam_Mega.h"
#include "Arducam/Platform.h"

#define SEALEVELPRESSURE_HPA (1013.25)
#define PIC_BUFFER_SIZE  0xff

#define CAM_A_CS 4
#define CAM_B_CS 5
#define SD_CS 15
#define SCD_RDY 27
#define VBAT_ADC 34
#define TEMP_ADC 33

// TODO:
// Add analog read of the battery and thermistor
// Create folder for each run
// Add camera code

Adafruit_SCD30 scd30;
Adafruit_BMP3XX bmp;
RTC_DS1307 rtc;

Arducam_Mega camA(CAM_A_CS);
Arducam_Mega camB(CAM_B_CS);

//SPIClass *vspi;
SPIClass *hspi;

bool data_ready = false;
int pic_num = 0;
char base_dir[20];


void IRAM_ATTR scd30_ready() {
  data_ready = true;
}

void write_pic(Arducam_Mega &cam, File dest) {

  uint8_t count = 1;
  uint8_t prev_byte = 0;
  uint8_t cur_byte = 0;
  uint8_t head_flag = 0;
  unsigned int i = 0;
  uint8_t image_buf[PIC_BUFFER_SIZE] = {0};

  while (cam.getReceivedLength())
  {
    // Store current and previous byte
    prev_byte = cur_byte;
    cur_byte = cam.readByte();

    // Write data to buffer
    if (head_flag == 1)
    {
      image_buf[i++]=cur_byte;
      // When buffer is full, write to file
      if (i >= PIC_BUFFER_SIZE)
      {
        dest.write(image_buf, i);
        i = 0;
      }
    }
    // Initialize file on JPEG file start (0xFFD8)
    if (prev_byte == 0xff && cur_byte == 0xd8)
    {
      head_flag = 1;
      //sprintf(name,"/%d.jpg", count);
      count++;
      //Serial.print(F("Saving image..."));
      image_buf[i++]=prev_byte;
      image_buf[i++]=cur_byte;
    }
    // Close file on JPEG file ending (0xFFD9)
    if (prev_byte == 0xff && cur_byte == 0xd9)
    {
      //headFlag = 0;
      dest.write(image_buf, i);
      //i = 0;
      dest.close();
      Serial.println(F("Done"));
      break;
    }
  }
}

void setup() {
  // Disable all CS lines
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  pinMode(CAM_A_CS, OUTPUT);
  digitalWrite(CAM_A_CS, HIGH);
  pinMode(CAM_B_CS, OUTPUT);
  digitalWrite(CAM_B_CS, HIGH);


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

  // Camera setup
  Serial.println("Initializing ArduCam A");
  camA.begin();
  Serial.println("Initializing ArduCam B");
  camB.begin();

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

  // Pick base dir
  int i = 0;
  do {
    sprintf(base_dir, "/kaoslog%d", i);
    i++;
  } while (SD.exists(base_dir));
  SD.mkdir(base_dir);
  Serial.print("Data from this run stored in ");
  Serial.println(base_dir);

  // Configure SCD30 RDY interrupt
  pinMode(SCD_RDY, INPUT_PULLDOWN);
  attachInterrupt(SCD_RDY, scd30_ready, RISING);
}

void loop() {
  // Wait for RDY interrupt then reset flag
  while(!data_ready) { delay(10); }
  data_ready = false;

  if (! scd30.read()) { Serial.println("Error reading SCD30 data"); return; }
  if (! bmp.performReading()) { Serial.println("Error reading BMP388 data"); return; }
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, setting date/time to 1-1-23/00:00:00");
    rtc.adjust(DateTime(2023, 1, 1, 0, 0, 0));
  }

  char fp[35];
  sprintf(fp, "%s/flight_log.csv", base_dir);
  File file = SD.open(fp, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }

  // take pictures
  camA.takePicture(CAM_IMAGE_MODE_WQXGA2,CAM_IMAGE_PIX_FMT_JPG);
  camB.takePicture(CAM_IMAGE_MODE_WQXGA2,CAM_IMAGE_PIX_FMT_JPG);

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

  // bmp temp (internal)
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
  Serial.print(",");

  // thermistor temp (external)
  float thermistor_raw = analogRead(TEMP_ADC);
  thermistor_raw = 10000 / ((4095 / thermistor_raw) - 1);
  Serial.print("Thermistor_(Ohms):");
  Serial.print(thermistor_raw, 3);
  Serial.print(",");

  // battery voltage
  float vbat_raw = analogRead(VBAT_ADC);
  float vbat = 3 * vbat_raw * 2.450 / 4095;
  Serial.print("Battery_(V):");
  Serial.print(vbat);
  Serial.println(); 

  file.close();

  // Save cam A pic
  sprintf(fp, "%s/a%d.jpg", base_dir, pic_num);
  file = SD.open(fp, FILE_WRITE);
  write_pic(camA, file);

  // Save cam B pic
  sprintf(fp, "%s/b%d.jpg", base_dir, pic_num);
  file = SD.open(fp, FILE_WRITE);
  write_pic(camB, file);

  pic_num++;
}