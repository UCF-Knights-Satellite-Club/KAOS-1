#include <Adafruit_SCD30.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SD.h>
//#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>


#define SEALEVELPRESSURE_HPA (1013.25) //Calibrate with California please

#define GPSECHO true

Adafruit_BMP3XX bmp;
Adafruit_SCD30 scd30;

File file;

//SoftwareSerial serial(8, 7);
//Adafruit_GPS GPS(&serial);

const int chipSelect = 10;
//uint32_t timer = millis();

void setup(void) {
  Serial.begin(9600);
  while (!Serial) delay(10);

  // SPI SMT SD Card SETUP
  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Initialization failed!"));
    while(1);
  }
  Serial.println(F("Initialization done."));

  if (SD.exists("launch.txt")) {
    Serial.println(F("File exists."));
    if (SD.remove("launch.txt") == true) {
      Serial.println(F("Successfully removed file."));
    } else {
      Serial.println(F("Could not remove file."));
    }
  }

  // SCD30 SETUP
  Serial.println(F("Adafruit SCD30 Test!"));
  
  // Try to initialize!
  if (!scd30.begin()) {
    Serial.println(F("Failed to find SCD30 chip"));
    while (1) { delay(10); }
  }
  Serial.println(F("SCD30 Found!"));
  Serial.print(F("Measurement Interval: ")); 
  Serial.print(scd30.getMeasurementInterval()); 
  Serial.println(F(" seconds"));
  delay(100);

  // BMP380 SETUP
  Serial.println(F("Adafruit BMP388 / BMP390 test"));
  
  if (!bmp.begin_I2C()) {
    Serial.println(F("Could not find a valid BMP3 sensor, check wiring!"));
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
/*
  // Ultimate GPS SETUP
  Serial.println(F("Adafruit GPS library basic parsing test!"));

  GPS.begin(9600);
  // Turn on RMC and GGA including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate to every 1Hz
  // Not recommended to set anything higher than 1Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  serial.println(F(PMTK_Q_RELEASE));
  */
}

void loop() {
  //readSCD();
  //readBMP();
  //readGPS();
  delay(200);
  writeFile();
}

void readSCD() {
  // SCD30 LOOP
  if (scd30.dataReady()){
      Serial.println(F("Data available!"));
  
      if (!scd30.read()){ Serial.println(F("Error reading sensor data")); return; }
  
      Serial.print("Temperature: ");
      Serial.print(scd30.temperature);
      Serial.println(" degrees C");
      
      Serial.print("Relative Humidity: ");
      Serial.print(scd30.relative_humidity);
      Serial.println(" %");
      
      Serial.print("CO2: ");
      Serial.print(scd30.CO2, 3);
      Serial.println(" ppm");
      Serial.println("");
    } else {
      //Serial.println("No data");
    }
  delay(100);
}

void readBMP() {
  // BMP388 LOOP
  if (! bmp.performReading()) {
    Serial.println(F("Failed to perform reading :("));
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}

/*
void readGPS() {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
}
*/

void writeFile() {
  // Opening file to write into. MAKE SURE TO CLOSE IT!
  file = SD.open("launch.txt", FILE_WRITE);

  // If the file opened OK, write to it
  if (file) {
    Serial.print(F("Writing to launch.txt..."));
    // INCLUDE ALL SENSORS DATA HERE
    // SCD30 Sensor
    file.print("Temperature: ");
    file.print(scd30.temperature);
    file.println(" degrees C");
    
    file.print("Relative Humidity: ");
    file.print(scd30.relative_humidity);
    file.println(" %");
    
    file.print("CO2: ");
    file.print(scd30.CO2, 3);
    file.println(" ppm");
    file.println("");
    
    delay(100);

    // BMP3xx Sensor
    file.print("Temperature = ");
    file.print(bmp.temperature);
    file.println(" *C");
  
    file.print("Pressure = ");
    file.print(bmp.pressure / 100.0);
    file.println(" hPa");
  
    file.print("Approx. Altitude = ");
    file.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    file.println(" m");
  
    file.println();
    delay(100);
    
    // Then close the file
    file.close();
    Serial.println(F("Done."));
  } else {
    Serial.println(F("Error opening launch.txt"));
  }
}
