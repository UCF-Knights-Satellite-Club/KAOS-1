#include <SD.h>
#include <SPI.h>

File file;

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  file = SD.open("launch.txt");
  if (file) {
    Serial.println("Reading launch.txt:");

    // Read from file until there's nothing
    while (file.available()) {
      Serial.write(file.read());
    }
    // Close the file
    file.close();
  } else {
    // If the file didn't open, print error
    Serial.println("Error opening launch.txt");
  }
}

void loop() {
  // Nothing happens after setup

}
