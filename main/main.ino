#include <bluefruit.h>

#define DEVICE_NAME "FR_S_11"
#define DATA_REPORT_INTERVAL_MS 100
#define SUSPENSION_TRAVEL_PID 1

#define SENSOR_RIDE_HEIGHT 345
#define SENSOR_DROOP 294

// Since we cannot report negative values in BLE, droop is defined as 0.
// Mapping is done using droop and ride height points since it is easier to get
// these measurements. All values in mm.
#define RIDE_HEIGHT_TRAVEL 33
#define DROOP_TRAVEL 0
#define MIN_TRAVEL 0
#define MAX_TRAVEL 100

#define CAN_IDX 0
#define SENSOR_PIN A0

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

unsigned long time_now = 0;
uint8_t canData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int sensorValue;
int suspensionTravel;

// BLEService docs: https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/bleservice
BLEService bleService = BLEService(0x1ff8);

// BLECharacteristic docs: https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/blecharacteristic
// RaceChrono uses two BLE characteristics:

// 0x02 to request which PIDs to send, and how frequently
BLECharacteristic pidRequestsCharacteristic = BLECharacteristic(0x02);
// 0x01 to be notified of data received for those PIDs
BLECharacteristic canBusDataCharacteristic = BLECharacteristic(0x01);

void initBLE() {
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);

  bleService.begin();

  pidRequestsCharacteristic.setProperties(CHR_PROPS_WRITE);
  pidRequestsCharacteristic.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  // pidRequestsCharacteristic.setWriteCallback(handle_racechrono_filter_request);
  pidRequestsCharacteristic.begin();

  canBusDataCharacteristic.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_READ);
  canBusDataCharacteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  canBusDataCharacteristic.begin();

  Bluefruit.setTxPower(+4);
}

void startBLEAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);

  // Fast mode interval: 20 ms, slow mode interval: 152.5 ms.
  // The numbers specified on the call are multiplied by (0.625 ms) units.
  Bluefruit.Advertising.setInterval(/* fast= */ 32, /* slow= */ 244);

  // Timeout for fast mode is 30 seconds.
  Bluefruit.Advertising.setFastTimeout(30);

  // Start advertising forever.
  Bluefruit.Advertising.start(/* timeout= */ 0);
}

void initSensors() {
  pinMode(SENSOR_PIN, INPUT);
}

void setup() {
  uint32_t startTimeMs = millis();
#ifdef HAS_DEBUG
  Serial.begin(115200);
#endif
  while (!Serial && millis() - startTimeMs < 1000) {
  }

  debugln("Initializing BLE...");
  initBLE();
  startBLEAdvertising();

  debugln("BLE initialized, waiting for connection.");
  waitForConnection();
}

void waitForConnection() {
  uint32_t printIteration = 0;
  while (!Bluefruit.connected()) {
    if (printIteration < 10) {
      debug(".");
      printIteration++;
    } else {
      debugln();
      printIteration = 0;
    }
    delay(500);
  }
  debugln("Connected.");
}

void sendData(uint32_t pid, const uint8_t *data) {
  unsigned char buffer[20];
  buffer[0] = pid & 0xFF;
  buffer[1] = (pid >> 8) & 0xFF;
  buffer[2] = (pid >> 16) & 0xFF;
  buffer[3] = (pid >> 24) & 0xFF;
  memcpy(buffer + 4, data, 8);
  // 4 for PID and 8 for payload
  canBusDataCharacteristic.notify(buffer, 12);
}

void reportData() {
 if(millis() >= time_now + DATA_REPORT_INTERVAL_MS){
     time_now += DATA_REPORT_INTERVAL_MS;

     sensorValue = analogRead(SENSOR_PIN);
     // map from potentiometer reading to suspension travel
     suspensionTravel = min(
       MAX_TRAVEL,
      max(
        MIN_TRAVEL,
        map(sensorValue, SENSOR_RIDE_HEIGHT, SENSOR_DROOP, RIDE_HEIGHT_TRAVEL, DROOP_TRAVEL)
      )
     );
     debugln(String(sensorValue, DEC) + "->" + String(suspensionTravel, DEC));
     canData[CAN_IDX] = suspensionTravel;

     sendData(SUSPENSION_TRAVEL_PID, canData);
   }
}

void loop() {
  if (!Bluefruit.connected()) {
    debugln("RaceChrono disconnected! Waiting for a new connection.");
    waitForConnection();
  }
  reportData();
}
