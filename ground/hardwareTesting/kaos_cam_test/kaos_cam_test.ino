#include "Arducam_Mega.h"
#include "Arducam/Platform.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define CAM_CS 17
#define SCK  14
#define MISO  12
#define MOSI  13
#define CS  15

#define  BUFFER_SIZE  0xff

Arducam_Mega myCAM(CAM_CS);

uint8_t count = 1;
char name[20] = {0};
File outFile;
uint8_t imageDataPrev = 0;
uint8_t imageData = 0;
uint8_t headFlag = 0;
unsigned int i = 0;
uint8_t imageBuff[BUFFER_SIZE] = {0};

void setup() {
  // Initialize serial
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("KAOS Image Testbed - KSC 2023"));

  // Initialize camera
  Serial.print(F("Initializing ArduCam..."));
  myCAM.begin();
  Serial.println(F("Done"));

  // Initialize SD
  Serial.print(F("Initializing SD..."));
  pinMode(CS, OUTPUT);
  if (!SD.begin(CS)) {
    Serial.println(F("Error"));
    Serial.println(F("Card mount failed"));
    while (1);
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println(F("Error"));
    Serial.println(F("No SD card attached"));
    while (1);
  }
  Serial.println(F("Done"));

  // I had to reformat twice already...
  Serial.println();
  Serial.println(F("Warning: Removing SD card before completion may cause corruption."));
  Serial.println();
  
  // Lower SPI speed for stability
  //SPI.setClockDivider(SPI_CLOCK_DIV2);

  // Delay to stop first image from being green
  arducamDelayMs(500);
}

void loop() {
  // Stop after 5 images
  if (count > 5) {
    Serial.println(F("SD card safe to remove"));
    while (1);
  }
  
  // Countdown
  Serial.print(F("Taking picture "));
  Serial.print(count);
  Serial.println(F("/5"));
  Serial.print(F("3..."));
  arducamDelayMs(500);
  Serial.print(F("2..."));
  arducamDelayMs(500);
  Serial.print(F("1..."));
  arducamDelayMs(500);
  Serial.println(F("Cheese!"));
  arducamDelayMs(500);

  // Take picture
  myCAM.takePicture(CAM_IMAGE_MODE_FHD,CAM_IMAGE_PIX_FMT_JPG);

  // Save image
  while (myCAM.getReceivedLength())
  {
    // Store current and previous byte
    imageDataPrev = imageData;
    imageData = myCAM.readByte();

    // Write data to buffer
    if (headFlag == 1)
    {
      imageBuff[i++]=imageData;
      // When buffer is full, write to file
      if (i >= BUFFER_SIZE)
      {
        outFile.write(imageBuff, i);
        i = 0;
      }
    }
    // Initialize file on JPEG file start (0xFFD8)
    if (imageDataPrev == 0xff && imageData == 0xd8)
    {
      headFlag = 1;
      sprintf(name,"/%d.jpg", count);
      count++;
      Serial.print(F("Saving image..."));
      outFile = SD.open(name, "w");
      if (!outFile)
      {
        Serial.println(F("Error"));
        Serial.println(F("File open failed"));
        while (1);
      }
      imageBuff[i++]=imageDataPrev;
      imageBuff[i++]=imageData;
    }
    // Close file on JPEG file ending (0xFFD9)
    if (imageDataPrev == 0xff && imageData == 0xd9)
    {
      headFlag = 0;
      outFile.write(imageBuff, i);
      i = 0;
      outFile.close();
      Serial.println(F("Done"));
      break;
    }
  }
}
