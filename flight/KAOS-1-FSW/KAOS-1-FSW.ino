//Including the libraries that we will be using in the code
#include <Wire.h>  //Allows communication between Arduino and the other devices

//Libraries for sensor use and data collection
#include <Adafruit_Sensor.h>  //Gives a unified interface for a bunch of adafruit sensors
#include <Adafruit_BMP3XX.h>  //Library for the barometer

//Libraries for the camera and interface with the camera
#include <Arducam_Mega.h>  //Library for interface with the Camera
#include <Arducam/Platform.h>

//Libraries for data storage and transfer
#include <SPI.h>  //Library for data transfer
#include <FS.h>   //Library that provides file operations for storage on the SD card
#include <SD.h>   //Library used for interfacing with SD cards

#include <Adafruit_SCD30.h>
#include "RTClib.h"
#include "driver/i2c.h"

// Pin constants
#define CAM_CS_A 4
#define CAM_CS_B 5
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define HSPI_CLK 14
#define SD_CS 15
#define VSPI_CLK 18
#define VSPI_MISO 19
#define I2C_SDA 21
#define I2C_SCL 22
#define VSPI_MOSI 23
#define BUZZER_PIN 25
#define SCD30_RDY 27
#define BMP388_INT 32
#define THERMISTOR 33
#define BATT_VOLTAGE 34


// Depending on the selected board, may or may not be defined already
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// Constants
#define SEALEVELPRESSURE_HPA (1013.25)        // standard pressure at sea level
#define PIC_BUFFER_SIZE 254                   // buffer size to store images, must be less than 255
#define CAM_IMAGE_MODE CAM_IMAGE_MODE_WQXGA2  // ArduCam image mode
#define SPI_CLOCK_DIV SPI_CLOCK_DIV16         // SPI clock divider, lowers SPI clock speeds to prevent image corruption
#define LOG_QUEUE_SIZE 50                     // how many entries to store before dropping data
#define SPI_MUTEX_WAIT 20                     // how many ms to wait for SPI mutex lock before giving up
#define LOG_DELAY 100                         // how many ms to wait between logging to SD
#define CALIBRATION_COUNT 20                  // how many measurements to average when determining initial altitude
#define ASCENT_ACCEL_THRESHOLD 20             // acceleration above which PREFLIGHT moves to ASCENT
#define MAX_PREFLIGHT_ALTITUDE 5              // altitude where PREFLIGHT automatically switches to ASCENT
#define ALTITUDE_CHECK_DELAY 50               // ms between altitude checks
#define FREEFALL_ACCEL_THRESHOLD 2            // acceleration below which to switch from ASCENT to FREEFALL
#define FREEFALL_VELOCITY_THRESHOLD 3         // decent speed above which to swtich from ASCENT to FREEFALL
#define PARACHUTE_DEPLOY_ALTITUDE 80          // altutude to switch from FREEFALL to LANDING
#define ALTITUDE_DELTA_FILTER_GAIN 0.95       // between 0 and 1, higher number means each measurement has lower impact on estimate
#define ACCEL_FILTER_GAIN 0.5                 // same as altitude
#define THERMISTORNOMINAL 7300                // resistance at 25 degrees C of thermistor
#define TEMPERATURENOMINAL 25                 // temp. for nominal resistance of thermistor (almost always 25 C)
#define BCOEFFICIENT 3950                     // The beta coefficient of the thermistor (usually 3000-4000)
#define THERM_TEMP_FILTER_GAIN 0.95
#define SERIESRESISTOR 10000  // the value of the 'other' resistor connected to the thermistor
#define BUZZER_INTERVAL 500

// Peripheral globals
Adafruit_BMP3XX bmp;
Arducam_Mega camA(CAM_CS_A);
Arducam_Mega camB(CAM_CS_B);


// Function definitions
void write_pic(Arducam_Mega &cam, File dest);
void calibrationRun();
void preflightRun();
void ascentRun();
void freefallRun();
void landingRun();
void checkAltitude(void *parameter);
void cameraCapture(void *parameter);
void logData(void *parameter);
void Tempreading(float *temp, float *resistance);
float Batt_read();

// Globals
static TaskHandle_t check_altitude;
static TaskHandle_t log_data;
static TaskHandle_t camera_capture;
static SemaphoreHandle_t spi_mutex;
static QueueHandle_t log_queue;
bool sd_fail = false;
int pic_num = 0;
char base_dir[20];
float start_altitude = 0;
int calibration_count = 0;
float altitude_delta_estimate = 0;
float prev_altitude_delta_estimate = 0;
float accel_x_estimate = 0;
float prev_accel_x_estimate = 0;
float accel_y_estimate = 0;
float prev_accel_y_estimate = 0;
float accel_z_estimate = 0;
float prev_accel_z_estimate = 0;
float accel_magnitude = 0;
float absolute_altitude = 0;
float ground_altitude = 0;
float altitude_delta = 0;
float prev_altitude = 0;
float therm_temp_estimate = 0;
float prev_therm_temp_estimate = 0;
char logpath[35];
int logindex = 0;

// Describes current flight state
typedef enum {
  CALIBRATION,  // establishing altitude baseline
  PREFLIGHT,    // waiting for launch
  ASCENT,       // rising on balloon
  FREEFALL,     // detached from balloon
  LANDING,      // parachute deployed
} FlightState;

// All the data that gets logged
typedef struct {
  int index;
  double temperature;
  double pressure;
  float altitude;
  float accel_x;
  float accel_y;
  float accel_z;
  float accel_filtered;
  float ascent_velocity_filtered;
  float therm_resistance;
  float therm_temp;
  float batt_voltage;
  FlightState flight_state;
} DataPoint;

FlightState flight_state = LANDING;
Adafruit_SCD30 scd30;
RTC_DS1307 rtc;

//SPIClass *vspi;
SPIClass *hspi;

bool data_ready = false;

void IRAM_ATTR scd30_ready() {
  data_ready = true;
}

void setup() {
  // LED on for entire setup process
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // BUZZER PINMODE
  pinMode(BUZZER_PIN, OUTPUT);

  // init serial
  Serial.begin(115200);

  hspi = new SPIClass(HSPI);

  Wire.begin();
  //i2c_set_timeout((i2c_port_t)I2C_NUM_0, 0xFFFFF);



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
  if (!SD.begin(SD_CS, *hspi)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  // Configure SCD30 RDY interrupt
  pinMode(SCD30_RDY, INPUT_PULLDOWN);
  attachInterrupt(SCD30_RDY, scd30_ready, RISING);

  // Queues data to be logged to SD
  log_queue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(DataPoint));
  if (log_queue == NULL) {
    Serial.print("Unable to create log queue");
    while (1) {}
  }

  // Used to prevent simultaneous SPI access from causing problems
  spi_mutex = xSemaphoreCreateMutex();
  if (spi_mutex == NULL) {
    Serial.print("Unable to create SPI mutex");
    while (1) {}
  }

  // Must be called after setting up SD and ArduCam since they will reset this
  SPI.setClockDivider(SPI_CLOCK_DIV);

  if (!sd_fail) {
    // Pick base directory for this flight
    int i = 0;
    do {
      sprintf(base_dir, "/KAOS-log%d", i);
      i++;
    } while (SD.exists(base_dir));
    SD.mkdir(base_dir);
    Serial.print("Data from this run stored in ");
    Serial.println(base_dir);
    sprintf(logpath, "%s/data.csv", base_dir);

    // Write CSV header
    File log_file = SD.open(logpath, FILE_APPEND);
    if (log_file) {
      log_file.println("index,bmp_temp,pressure,altitude,accelx,accely,accelz,accel_filtered,ascent_velocity,therm_resistance,therm_temp,batt_voltage,state");
      log_file.close();
    }
  }

  // Create task for sensor checks
  xTaskCreatePinnedToCore(
    checkAltitude,     // Function to call
    "Check Altitude",  // Task name
    4096,              // Memory allocated
    NULL,              // Parameters to pass to the function
    2,                 // Priority (higher number = higher priority)
    &check_altitude,   // Task handle
    0                  // Core
  );

  // Create task for taking pictures
  xTaskCreatePinnedToCore(
    cameraCapture,
    "Camera Capture",
    4096,
    NULL,
    tskIDLE_PRIORITY,  // Lowest possible priority, see https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/01-Tasks-and-co-routines/15-Idle-task
    &camera_capture,
    0);

  // Create task for logging data to SD
  xTaskCreatePinnedToCore(
    logData,
    "Log Data",
    4096,
    NULL,
    1,
    &log_data,
    0);

  // Delay to stop first image from being green
  vTaskDelay(500 / portTICK_PERIOD_MS);

  // Unsuspend tasks
  vTaskResume(check_altitude);
  if (!sd_fail) {
    vTaskResume(camera_capture);
    vTaskResume(log_data);
  }

  // LED off once setup complete
  digitalWrite(LED_BUILTIN, LOW);
}

// Empty loop function, all functionality in custom tasks
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// Periodically monitors sensor data and performs state switches
void checkAltitude(void *parameter) {
  vTaskSuspend(NULL);  // Initially suspend task

  TickType_t last_wake = xTaskGetTickCount();



  while (1) {
    /*
    Serial.print("absolute alt: ");
    Serial.println(absolute_altitude);
    Serial.print("start alt: ");
    Serial.println(start_altitude);
    Serial.print("alt: ");
    Serial.println(ground_altitude);
    */

    // Get altitude data
    bmp.performReading();
    absolute_altitude = calculateAltitude(bmp.pressure);
    ground_altitude = absolute_altitude - start_altitude;
    altitude_delta = absolute_altitude - prev_altitude;
    prev_altitude = absolute_altitude;

    // Get imu data
    //mma.read();
    float accel_x = 0;
    float accel_y = 0;
    float accel_z = 0;

    // Lowpass filter, smoothes out noisy data
    altitude_delta_estimate = (ALTITUDE_DELTA_FILTER_GAIN * prev_altitude_delta_estimate) + (1 - ALTITUDE_DELTA_FILTER_GAIN) * altitude_delta;
    prev_altitude_delta_estimate = altitude_delta_estimate;

    accel_x_estimate = (ACCEL_FILTER_GAIN * prev_accel_x_estimate) + (1 - ACCEL_FILTER_GAIN) * accel_x;
    prev_accel_x_estimate = accel_x_estimate;

    accel_y_estimate = (ACCEL_FILTER_GAIN * prev_accel_y_estimate) + (1 - ACCEL_FILTER_GAIN) * accel_y;
    prev_accel_y_estimate = accel_y_estimate;

    accel_z_estimate = (ACCEL_FILTER_GAIN * prev_accel_z_estimate) + (1 - ACCEL_FILTER_GAIN) * accel_z;
    prev_accel_z_estimate = accel_z_estimate;


    accel_magnitude = sqrtf(accel_x_estimate * accel_x_estimate + accel_y_estimate * accel_y_estimate + accel_z_estimate * accel_z_estimate);

    float temp, resistance;
    Tempreading(&temp, &resistance);

    float batt_voltage = Batt_read();


    therm_temp_estimate = (THERM_TEMP_FILTER_GAIN * prev_therm_temp_estimate) + (1 - THERM_TEMP_FILTER_GAIN) * temp;
    prev_therm_temp_estimate = therm_temp_estimate;

    /*
    Serial.print("accel:");
    Serial.println(accel_estimate);
    */

    // Core state machine
    switch (flight_state) {
      case CALIBRATION:
        /* CALIBRATE: averages measurements to
       * determine a launch altitude. */
        calibrationRun();
        break;
      case PREFLIGHT:
        /* PRE-LAUNCH: waiting on the ground to be released. Detects
       * launch by breaking an altitude rate of change threshold
       * or passing a certain altitude.       - > ASCENT         */
        preflightRun();
        break;
      case ASCENT:
        /* ASCENT: Rising up to the extent of the tether and waiting
       * for the burn wire to trigger and free fall to begin.
       * Detects this by breaking an altitude rate of change
       * threshold (or accelerometer info)    - > FREEFALL       */
        ascentRun();
        break;
      case FREEFALL:
        /* FREE FALL: falling through the air and waiting to
       * hit the parachute deployment altitude.  - > LANDING      */
        freefallRun();
        break;
      case LANDING:
        /* LANDING: Deploy parachute. Final State. */
        landingRun();
        break;
    }

    if (!sd_fail) {
      if (uxQueueSpacesAvailable(log_queue) >= 1) {
        DataPoint data_point = {
          logindex,
          bmp.temperature,
          bmp.pressure,
          ground_altitude,
          accel_x,
          accel_y,
          accel_z,
          accel_magnitude,
          altitude_delta_estimate * 1000 / ALTITUDE_CHECK_DELAY,
          resistance,
          therm_temp_estimate,
          batt_voltage,
          flight_state
        };
        if (xQueueSendToBack(log_queue, &data_point, 0) == pdFALSE) {
          Serial.println("Failed to add data to queue");
        }
      } else {
        Serial.println("WARNING: Log queue is full, some data is getting dropped");
      }

      logindex++;
    }
    vTaskDelayUntil(&last_wake, ALTITUDE_CHECK_DELAY / portTICK_PERIOD_MS);
  }
}


void cameraCapture(void *parameter) {
  vTaskSuspend(NULL);  // Initially suspend task
  while (1) {
    Serial.println("Taking picture");
    camA.takePicture(CAM_IMAGE_MODE, CAM_IMAGE_PIX_FMT_JPG);
    camB.takePicture(CAM_IMAGE_MODE, CAM_IMAGE_PIX_FMT_JPG);
    char fpa[35];
    char fpb[35];
    sprintf(fpa, "%s/pic%da.jpg", base_dir, pic_num);
    sprintf(fpb, "%s/pic%db.jpg", base_dir, pic_num);
    pic_num++;
    Serial.println("Saving picture");
    if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
      File file = SD.open(fpa, FILE_WRITE);
      xSemaphoreGive(spi_mutex);
      write_pic(camA, file);
      Serial.print("Picutre saved to ");
      Serial.println(fpa);
    }
    if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
      File file = SD.open(fpb, FILE_WRITE);
      xSemaphoreGive(spi_mutex);
      write_pic(camB, file);
      Serial.print("Picutre saved to ");
      Serial.println(fpb);
    }
  }
}

void logData(void *parameter) {
  vTaskSuspend(NULL);  // Initially suspend task
  while (1) {
    if (uxQueueMessagesWaiting(log_queue) > 0) {
      if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
        File log_file = SD.open(logpath, FILE_APPEND);
        if (log_file) {
          DataPoint data_point;
          while (xQueueReceive(log_queue, &data_point, 0) == pdTRUE) {
            log_file.print(data_point.index);
            log_file.print(",");
            log_file.print(data_point.temperature);
            log_file.print(",");
            log_file.print(data_point.pressure);
            log_file.print(",");
            log_file.print(data_point.altitude);
            log_file.print(",");
            log_file.print(data_point.accel_x);
            log_file.print(",");
            log_file.print(data_point.accel_y);
            log_file.print(",");
            log_file.print(data_point.accel_z);
            log_file.print(",");
            log_file.print(data_point.accel_filtered);
            log_file.print(",");
            log_file.print(data_point.ascent_velocity_filtered);
            log_file.print(",");
            log_file.print(data_point.therm_resistance);
            log_file.print(",");
            log_file.print(data_point.therm_temp);
            log_file.print(",");
            log_file.print(data_point.batt_voltage);
            log_file.print(",");
            log_file.println(data_point.flight_state);
          }
          log_file.close();
        } else {
          Serial.println("Error opening file");
        }
        xSemaphoreGive(spi_mutex);
      }
    }
    vTaskDelay(LOG_DELAY / portTICK_PERIOD_MS);
  }
}

/* ==== UTILITY FUNCTIONS ==== */

void write_pic(Arducam_Mega &cam, File dest) {

  uint8_t prev_byte = 0;
  uint8_t cur_byte = 0;
  uint8_t head_flag = 0;
  unsigned int i = 0;
  uint8_t image_buf[PIC_BUFFER_SIZE] = { 0 };
  uint32_t read_len = 0;
  uint8_t start_offset = 0;

  while (cam.getReceivedLength()) {
    start_offset = 0;
    if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
      read_len = cam.readBuff(image_buf, PIC_BUFFER_SIZE);
      xSemaphoreGive(spi_mutex);
    } else {
      Serial.println("WARNING: SPI lock timeout exceeded");
      return;
    }
    // Search through buffer for start and end flags
    for (i = 0; i < read_len; i++) {
      // Store current and previous byte
      prev_byte = cur_byte;
      cur_byte = image_buf[i];

      // Start writing at JPEG file start (0xFFD8)
      if (prev_byte == 0xff && cur_byte == 0xd8) {
        Serial.print("Found jpeg start at i=");
        Serial.println(i);
        head_flag = 1;
        start_offset = i + 1;
        if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
          dest.write(0xff);
          dest.write(0xd8);
          xSemaphoreGive(spi_mutex);
        } else {
          Serial.println("WARNING: SPI lock timeout exceeded");
          return;
        }
      }

      // Close file on JPEG file ending (0xFFD9)
      if (head_flag && prev_byte == 0xff && cur_byte == 0xd9) {
        Serial.print("Found jpeg end at i=");
        Serial.println(i);
        Serial.print("Writing buffer from ");
        Serial.print(start_offset);
        Serial.print(" with len ");
        Serial.println(i + 1 - start_offset);
        if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
          dest.write(image_buf + start_offset, i + 1 - start_offset);
          dest.close();
          xSemaphoreGive(spi_mutex);
        } else {
          Serial.println("WARNING: SPI lock timeout exceeded");
        }
        return;
      }
    }

    // No end flag found, write the full buffer
    if (head_flag) {
      /*Serial.print("Writing buffer from ");
      Serial.print(start_offset);
      Serial.print(" with len ");
      Serial.println(PIC_BUFFER_SIZE - start_offset);*/
      if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
        dest.write(image_buf + start_offset, read_len - start_offset);
        xSemaphoreGive(spi_mutex);
      } else {
        Serial.println("WARNING: SPI lock timeout exceeded");
        return;
      }
    }
  }

  // Did not find EOF, corrupted?
  Serial.println("WARNING: Did not find JPEG file ending in image data, possible corruption");
  if (xSemaphoreTake(spi_mutex, SPI_MUTEX_WAIT) == pdTRUE) {
    dest.close();
    xSemaphoreGive(spi_mutex);
  } else {
    Serial.println("WARNING: SPI lock timeout exceeded");
  }
}

float calculateAltitude(float atmospheric) {
  atmospheric = atmospheric / 100.0;
  return 44330.0 * (1.0 - pow(atmospheric / SEALEVELPRESSURE_HPA, 0.1903));
}

/* ================================= */

/* ==== STATE MACHINE FUNCTIONS ==== */

void calibrationRun() {
  // average several readings for baseline then move to PRELAUNCH
  start_altitude += absolute_altitude;
  calibration_count++;

  Serial.print("Calibrating altitude ");
  Serial.print(calibration_count);
  Serial.print("/");
  Serial.println(CALIBRATION_COUNT);

  if (calibration_count >= CALIBRATION_COUNT) {
    start_altitude /= CALIBRATION_COUNT;
    flight_state = PREFLIGHT;

    Serial.print("Starting altitude: ");
    Serial.println(start_altitude);
    Serial.println("Calibration complete, moving to PREFLIGHT");
  }
}

void preflightRun() {
  // move to ASCENT if acceleration is or altitude above threshold

  if (ground_altitude >= MAX_PREFLIGHT_ALTITUDE) {
    flight_state = ASCENT;
    Serial.println("Max preflight altitude exceeded, moving to ASCENT");

  } else if (accel_magnitude >= ASCENT_ACCEL_THRESHOLD) {
    flight_state = ASCENT;
    Serial.println("Acceleration threshold met, moving to ASCENT");
  }
}

void ascentRun() {
  // move to FREEFALL if acceleration is close to 0
  if (accel_magnitude <= FREEFALL_ACCEL_THRESHOLD) {
    flight_state = FREEFALL;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Acceleration threshold met, moving to FREEFALL");
  }
  // move to FREEFALL if altitude is changing faast enough
  if (altitude_delta_estimate * 1000 / ALTITUDE_CHECK_DELAY < -FREEFALL_VELOCITY_THRESHOLD) {
    flight_state = FREEFALL;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Decent velocity threshold met, moving to FREEFALL");
  }
}

void freefallRun() {
  // move to LANDING if below deployment altitude
  if (ground_altitude < PARACHUTE_DEPLOY_ALTITUDE) {
    flight_state = LANDING;

    Serial.println("Parachute deployment altitude reached, moving to LANDING");
  }
}

void landingRun() {
  // Deploy parachute, buzzer loop, last stage of flight
  static unsigned long previousmillis = 0;
  unsigned long currentmillis = millis();  // checks and updates time value for the upcoming calculations

  if (currentmillis - previousmillis >= BUZZER_INTERVAL) {
    previousmillis = currentmillis;  // updates previousmillis for the next run

    static bool buzzer_on = false;

    // below if statement will turn the buzzer on if buzzer_on is false.

    if (buzzer_on) {
      noTone(BUZZER_PIN);
    } else {
      tone(BUZZER_PIN, 3000);
    }

    buzzer_on = !buzzer_on;  // changes state for next run
  }
}


void Tempreading(float *temp, float *resistance) {
  float reading;

  reading = analogRead(THERMISTOR);

  // Serial.print("analog reading ");
  // Serial.println(reading);

  // convert the value to resistance
  reading = 4095 / reading - 1;
  reading = SERIESRESISTOR / reading;
  *resistance = reading;

  float steinhart;
  steinhart = reading / THERMISTORNOMINAL;           // (R/Ro)
  steinhart = log(steinhart);                        // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                         // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                       // Invert
  steinhart -= 273.15;                               // convert absolute temp to C
  *temp = steinhart;
  // Serial.print("Temperature ");
  // Serial.print(steinhart);
  // Serial.println(" *C");
}


float Batt_read() {
  float reading;

  reading = analogRead(BATT_VOLTAGE);

  // Voltage conversion with 3x divider
  reading *= 9.9 / 4095;

  return reading;
}

/* ================================= */