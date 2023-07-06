#include <Arduino.h>
#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "ble_gatt.h"
#include "adc_vbat.h"

#define DEVICE_NAME "RearLeftSensor" // Change as needed

// Definitions needed for measuring suspension travel using a potentiometer as input.
// Ride height is defined as 0 travek. Droop will report negative values, bump positive values.
// Mapping is done using droop and ride height points since it is easier to get
// these measurements. All values in mm.
#define RIDE_HEIGHT_TRAVEL 0
#define MIN_TRAVEL -100
#define MAX_TRAVEL 100
#define DISTANCE_SENSOR_PIN A0`
#define DROOP_TRAVEL -33
#define SENSOR_RIDE_HEIGHT 398 // the value of the sensor at ride height
#define SENSOR_DROOP 312 // the sensor value at full droop

// Used to debug using serial monitor.
#define HAS_DEBUG
#ifdef HAS_DEBUG
#define debugln Serial.println
#define debug Serial.print
#else
void dummy_debug(...) {
}
#define debug dummy_debug
#define debugln dummy_debug
#endif

// MLX90640
const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640

#define TA_SHIFT 8 //Default shift for MLX90640 in open air

#define TOTAL_PIXELS 768 // 32x24
static float mlx90640To[TOTAL_PIXELS];
paramsMLX90640 mlx90640;

// Sensor should be placed with the reference tab pointing to the back of the car.
// We always need to have Temp1 => inner most temp, Temp8 => outer most temp
// For right hand side tyres uncomment to keep the above consistent for all tyres.                  
// #define RIGHT_SIDE_TYRE     
                                              
#define PROTOCOL 0x02
#define TEMPSCALING 1.00    // Default = 1.00
#define TEMPOFFSET 0        // Default = 0      NOTE: in TENTHS of degrees Celsius --> TEMPOFFSET 10 --> 1 degree

typedef struct {
  uint8_t  protocol;         // version of protocol used
  uint8_t  unused;
  int16_t  distance;         // millimeters
  int16_t  temps[8];         // all even numbered temp spots (degrees Celsius x 10)
} one_t;


typedef struct {
  uint8_t  protocol;         // version of protocol used
  uint8_t  charge;           // percent: 0-100
  uint16_t voltage;          // millivolts (normally circa 3500-4200)
  int16_t  temps[8];         // all uneven numbered temp spots (degrees Celsius x 10)
} two_t;


typedef struct {
  uint8_t  protocol;         // version of protocol used
  uint8_t  unused;
  int16_t distance;          // millimeters
  int16_t  temps[8];         // All 16 temp spots used as pairs of two each, max from each pair goes into these 8 temp values (degrees Celsius x 10)
} thr_t;


one_t datapackOne;
two_t datapackTwo;
thr_t datapackThr;
// The raw value of the potentiometer measuring the ride height
int potentiometerValue;

boolean isTempSensorConnected();
void tempSensorSetup();
void measureSuspensionTravel();
void measureTemps();
void measureBattery();
void printStatus(void);

void setup() {
    uint32_t startTimeMs = millis();
#ifdef HAS_DEBUG
  Serial.begin(115200);
#endif
  while (!Serial && millis() - startTimeMs < 1000) {
  }
  debugln("\nBegin startup");

  Bluefruit.autoConnLed(false); // DISABLE BLUE BLINK ON CONNECT STATUS

  tempSensorSetup();

  // SET SOME DEFAULT VALUES IN THE DATA PACKETS
  datapackOne.distance = 0;
  datapackOne.protocol = PROTOCOL;
  datapackTwo.protocol = PROTOCOL;
  datapackThr.distance = 0;
  datapackThr.protocol = PROTOCOL;

  // START UP BLUETOOTH
  debug("Starting bluetooth with MAC address ");
  Bluefruit.begin();
  debugln();
  debug("Device name: ");
  debugln(DEVICE_NAME);
  Bluefruit.setName(DEVICE_NAME); 

  // RUN BLUETOOTH GATT
  setupMainService();
  startAdvertising(); 
  debugln("Running!");
}

void loop() {
  
  measureSuspensionTravel();
  measureTemps();
  measureBattery();
  
  if (Bluefruit.connected()) {
    GATTone.notify(&datapackOne, sizeof(datapackOne));
    GATTtwo.notify(&datapackTwo, sizeof(datapackTwo));
    GATTthr.notify(&datapackThr, sizeof(datapackThr));
  }

  printStatus();
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isTempSensorConnected() {
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0) {
    return (false); //Sensor did not ACK
  }
  return (true);
}

// Setup MLX90640
void tempSensorSetup() {
  debugln("Initializing temp sensor...");
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (!isTempSensorConnected()) {
    debugln("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  debugln("MLX90640 connected!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) {
    debugln("Failed to load system parameters");
  }

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) {
    debugln("Parameter extraction failed");
  }

  // Readig as fast as possibe
  MLX90640_SetRefreshRate(MLX90640_address, 0x04);
}

void measureTemps() {
  for (byte x = 0 ; x < 2 ; x++) {//Read both subpages
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0) {
      debugln("GetFrame Error: ");
      debugln(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  // Placement of the sensor should be with reference tab of the sensor pointing to the back of the car.
  // We always make Temp1 the inner most side of the tyre and Temp8 the outer most side
  // see RIGHT_SIDE_TYRE
  for (byte tIdx = 0; tIdx < 8; tIdx++) {
  #ifdef RIGHT_SIDE_TYRE
    const byte valueIdx = 7 - tIdx;
  #else
    const byte valueIdx = tIdx;
  #endif
    const byte sensorArrayStartColumn = tIdx * 4;
    float avgT = 0; // -1000 degress celcius seems ok to start :)
    for(byte r=0; r<24; r++) {
        byte pixelIdx = sensorArrayStartColumn + r * 32;
        avgT += mlx90640To[pixelIdx] + mlx90640To[pixelIdx+1];
    }
    datapackOne.temps[valueIdx] = (int16_t) TEMPOFFSET + TEMPSCALING * 10 * (avgT / 48);

    avgT = 0;
    for(byte r=0; r<24; r++) {
        byte pixelIdx = sensorArrayStartColumn + 2 + r * 32;
        avgT += mlx90640To[pixelIdx] + mlx90640To[pixelIdx+1];
    }
    datapackTwo.temps[valueIdx] = (int16_t) TEMPOFFSET + TEMPSCALING * 10 * (avgT / 48);
    datapackThr.temps[valueIdx] = (int16_t) max(datapackOne.temps[valueIdx], datapackTwo.temps[valueIdx]);
  }
}

/**
 * Measures the suspension travel using a potentiometer.
 */
void measureSuspensionTravel() {
  potentiometerValue = analogRead(DISTANCE_SENSOR_PIN);
  // map from potentiometer reading to suspension travel
  datapackOne.distance = min(
    MAX_TRAVEL,
    max(
      MIN_TRAVEL,
      map(potentiometerValue, SENSOR_RIDE_HEIGHT, SENSOR_DROOP, RIDE_HEIGHT_TRAVEL, DROOP_TRAVEL)
    )
  );
}

void measureBattery() {
  unsigned long now = millis();
  static unsigned long timer = 60000;  // check every 60 seconds
  if (now - timer >= 60000) {
    timer = now;
    datapackTwo.voltage = getVbat();
    datapackTwo.charge = lipoPercent(datapackTwo.voltage);
  }
}

void printStatus(void) {
#ifdef HAS_DEBUG
  static unsigned long then;
  unsigned long now = millis();
  debug(1000/(float)(now - then),1); // Print loop speed in Hz
  debug("Hz\t");
  then = now;

  debug(datapackTwo.voltage);
  debug("mV\t");
  debug(datapackTwo.charge);
  debug("%\t");

  debug(datapackOne.distance);
  debug("mm(");
  debug(potentiometerValue);
  debug(")\t");

  for (uint8_t i=0; i<8; i++) {
    debug(datapackOne.temps[i]);
    debug("\t");
    debug(datapackTwo.temps[i]);
    debug("\t");
  }

  debugln();
#endif
}
