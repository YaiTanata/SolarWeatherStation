#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoHA.h>
#include <PMS.h>
#include <HardwareSerial.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include "SHTSensor.h"
#include "secrets.h"

// Solar Weather Station - optimized for solar + Home Assistant MQTT
// Target: about 1 week no-sun runtime, depending on real battery capacity and deep sleep current.
// Hardware assumption: 2x 18650 are connected in PARALLEL as 1S2P, not series.

#define MQTT_NAME     "Solar_weather_station"
#define STA_ID        "WeatherStation_1"

// Wake strategy for 1-week target
#define DEEP_SLEEP_NORMAL_MINUTES 10     // Normal wake interval
#define DEEP_SLEEP_LOW_MINUTES    30     // Low battery wake interval
#define DEEP_SLEEP_EMPTY_MINUTES  120    // Very low battery wake interval

// PMS strategy
// 10 min x 3 = read PMS every 30 minutes in normal mode.
#define PMS_EVERY_N_WAKE          3

// Thresholds for adaptive power saving
#define BATT_LOW_PERCENT          25
#define BATT_SKIP_PMS_PERCENT     20
#define BATT_EMPTY_PERCENT        5

// Runtime limits
#define WIFI_TIMEOUT_SECONDS      30
#define MQTT_SETTLE_MS            8000
#define WDT_TIMEOUT_SECONDS       120

// Pins
#define LED_BUILTIN   5
#define BATT_PIN      35
#define SENSOR_EN_PIN 13
#define TX2_PIN       25
#define RX2_PIN       26
#define CHARGE_EN     27

// Set to 1 only if GPIO27 is really wired to a charger/module enable pin.
#define USE_CHARGE_EN_CONTROL 0

// Battery ADC calibration
// Adjust BATT_CALIBRATION after comparing Serial output with a multimeter reading.
#define ADC_REF_VOLTAGE     3.30f
#define ADC_MAX_COUNTS      4095.0f
#define BATT_DIVIDER_RATIO  2.00f
#define BATT_CALIBRATION    1.00f

WiFiClient client;

// Home Assistant MQTT objects
HADevice *haDevice;
HAMqtt *haMQTT;

// Use HASensorNumber for numeric values.
HASensorNumber *haSensor_vBatt;
HASensorNumber *haSensor_SoC;
HASensorNumber *haSensor_PM1_0;
HASensorNumber *haSensor_PM2_5;
HASensorNumber *haSensor_PM10_0;
HASensorNumber *haSensor_temp;
HASensorNumber *haSensor_humid;

// Sensors
PMS pms(Serial2);
PMS::DATA data;
SHTSensor sht(SHTSensor::SHT3X);

// Persisted across deep sleep
RTC_DATA_ATTR uint32_t wakeCount = 0;

// Keep latest PMS values across deep sleep.
// This lets Home Assistant show PM entities even on skipped PMS wake cycles.
RTC_DATA_ATTR int lastPM1_0 = 0;
RTC_DATA_ATTR int lastPM2_5 = 0;
RTC_DATA_ATTR int lastPM10_0 = 0;
RTC_DATA_ATTR bool hasLastPMS = false;

// Sensor values
float vBatt = 0.0f;
int SoC = 100;

bool pmsOK = false;
bool pmsWasRead = false;
int PM1_0 = 0;
int PM2_5 = 0;
int PM10_0 = 0;

bool shtOK = false;
float temp = 0.0f;
float humid = 0.0f;

enum BatteryCondition {
  BATT_NORMAL,
  BATT_LOW,
  BATT_EMPTY
};

void shutdownRadios() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();
}

void setSensorPower(bool enabled) {
  digitalWrite(SENSOR_EN_PIN, enabled ? HIGH : LOW);
  delay(enabled ? 300 : 50);
}

void shutdownPmsPins() {
  pms.sleep();
  delay(100);

  Serial2.end();

  pinMode(RX2_PIN, INPUT);
  pinMode(TX2_PIN, INPUT);
}

void deepSleep(BatteryCondition batteryCondition) {
  digitalWrite(LED_BUILTIN, LOW);
  setSensorPower(false);
  shutdownPmsPins();
  shutdownRadios();

  uint32_t minutes = DEEP_SLEEP_NORMAL_MINUTES;

  switch (batteryCondition) {
    case BATT_NORMAL:
      minutes = DEEP_SLEEP_NORMAL_MINUTES;
      Serial.println("Start deep sleep: normal");
      break;

    case BATT_LOW:
      minutes = DEEP_SLEEP_LOW_MINUTES;
      Serial.println("Start deep sleep: battery low");
      break;

    case BATT_EMPTY:
      minutes = DEEP_SLEEP_EMPTY_MINUTES;
      Serial.println("Start deep sleep: battery empty");
      break;
  }

  Serial.printf("Sleep for %lu minutes\n", (unsigned long)minutes);
  Serial.flush();

  esp_sleep_enable_timer_wakeup((uint64_t)minutes * 60ULL * 1000000ULL);
  esp_deep_sleep_start();
}

int calculateSoC(float voltage) {
  // Simple 1S Li-ion open-circuit lookup table.
  // For better accuracy use a MAX17043/MAX17048 fuel gauge.
  if (voltage >= 4.20f) return 100;
  if (voltage >= 4.10f) return 90;
  if (voltage >= 4.00f) return 80;
  if (voltage >= 3.92f) return 70;
  if (voltage >= 3.85f) return 60;
  if (voltage >= 3.79f) return 50;
  if (voltage >= 3.75f) return 40;
  if (voltage >= 3.70f) return 30;
  if (voltage >= 3.60f) return 20;
  if (voltage >= 3.45f) return 10;
  if (voltage >= 3.30f) return 5;
  return 0;
}

float getBattVoltage() {
  uint32_t sum = 0;

  for (int i = 0; i < 64; i++) {
    sum += analogRead(BATT_PIN);
    delay(2);
  }

  float raw = sum / 64.0f;
  float voltage = raw * ADC_REF_VOLTAGE / ADC_MAX_COUNTS;
  voltage *= BATT_DIVIDER_RATIO;
  voltage *= BATT_CALIBRATION;

  if (voltage > 6.0f || voltage < 0.0f) {
    return 0.0f;
  }

  return voltage;
}

bool shouldReadPMS() {
  if (SoC <= BATT_SKIP_PMS_PERCENT) {
    Serial.println("PMS skipped because battery is low");
    return false;
  }

  return (wakeCount % PMS_EVERY_N_WAKE) == 0;
}

bool fetchTempHumid() {
  shtOK = false;

  if (!sht.init()) {
    Serial.println("SHT init failed");
    return false;
  }

  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);

  if (sht.readSample()) {
    temp = sht.getTemperature();
    humid = sht.getHumidity();
    shtOK = true;
  } else {
    Serial.println("SHT read failed");
  }

  return shtOK;
}

bool fetchPMS() {
  pmsOK = false;
  pmsWasRead = true;

  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(200);

  pms.wakeUp();
  pms.passiveMode();

  Serial.print("Warming up PMS sensor");

  // PMS sensors need airflow stabilization. Keep this long enough for usable values.
  for (int i = 0; i < 30; i++) {
    Serial.print(".");
    pms.requestRead();

    if (pms.readUntil(data, 1000)) {
      PM1_0 = data.PM_AE_UG_1_0;
      PM2_5 = data.PM_AE_UG_2_5;
      PM10_0 = data.PM_AE_UG_10_0;

      lastPM1_0 = PM1_0;
      lastPM2_5 = PM2_5;
      lastPM10_0 = PM10_0;
      hasLastPMS = true;

      pmsOK = true;
    }

    esp_task_wdt_reset();
    delay(1000);
  }

  Serial.println();

  pms.sleep();
  return pmsOK;
}

void fetchSensorValues() {
  Wire.begin();
  setSensorPower(true);

  if (shouldReadPMS()) {
    if (!fetchPMS()) {
      Serial.println("PMS sensor error");
    }
  } else {
    pmsWasRead = false;
    Serial.println("PMS read skipped to save power");
  }

  if (!fetchTempHumid()) {
    Serial.println("SHT sensor error");
  }

  setSensorPower(false);
  shutdownPmsPins();

  Serial.println();
  Serial.printf("Wake count = %lu\n", (unsigned long)wakeCount);
  Serial.printf("vBatt     = %.2f V\n", vBatt);
  Serial.printf("SoC       = %d%%\n", SoC);

  if (pmsWasRead && pmsOK) {
    Serial.println("PMS fresh values:");
    Serial.printf("PM1       = %d ug/m3\n", PM1_0);
    Serial.printf("PM2.5     = %d ug/m3\n", PM2_5);
    Serial.printf("PM10      = %d ug/m3\n", PM10_0);
  } else if (hasLastPMS) {
    Serial.println("PMS last known values:");
    Serial.printf("PM1       = %d ug/m3\n", lastPM1_0);
    Serial.printf("PM2.5     = %d ug/m3\n", lastPM2_5);
    Serial.printf("PM10      = %d ug/m3\n", lastPM10_0);
  } else {
    Serial.println("PMS has no value yet");
  }

  if (shtOK) {
    Serial.printf("Temp      = %.2f C\n", temp);
    Serial.printf("Humid     = %.2f%%\n", humid);
  }
}

bool connectWiFi() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_STA_NAME);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS);

  uint32_t startedAt = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    esp_task_wdt_reset();

    if (millis() - startedAt > WIFI_TIMEOUT_SECONDS * 1000UL) {
      Serial.println("\nWiFi timeout");
      return false;
    }
  }

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  return true;
}

bool connectAndSend() {
  if (!connectWiFi()) {
    shutdownRadios();
    return false;
  }

  if (!haMQTT->begin(MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println("MQTT connection failed");
    shutdownRadios();
    return false;
  }

  Serial.println("MQTT connected");

  // Give ArduinoHA time to connect and publish discovery.
  uint32_t connectStartedAt = millis();
  while (millis() - connectStartedAt < 3000) {
    haMQTT->loop();
    esp_task_wdt_reset();
    delay(50);
  }

  // Always announce all sensors so Home Assistant creates all entities.
  haSensor_vBatt->setAvailability(true);
  haSensor_SoC->setAvailability(true);

  // Keep Temperature/Humidity visible.
  haSensor_temp->setAvailability(true);
  haSensor_humid->setAvailability(true);

  // Keep PM sensors visible in Home Assistant.
  // If PMS was skipped this wake cycle, publish latest retained value.
  haSensor_PM1_0->setAvailability(true);
  haSensor_PM2_5->setAvailability(true);
  haSensor_PM10_0->setAvailability(true);

  haSensor_vBatt->setValue(vBatt, true);
  haSensor_SoC->setValue(SoC, true);

  if (shtOK) {
    haSensor_temp->setValue(temp, true);
    haSensor_humid->setValue(humid, true);
  } else {
    // Fallback values so entities still appear.
    haSensor_temp->setValue(0.0f, true);
    haSensor_humid->setValue(0.0f, true);
  }

  if (pmsOK) {
    // Current wake cycle has fresh PMS values.
    haSensor_PM1_0->setValue(PM1_0, true);
    haSensor_PM2_5->setValue(PM2_5, true);
    haSensor_PM10_0->setValue(PM10_0, true);
  } else if (hasLastPMS) {
    // PMS was skipped or failed, use last known values.
    haSensor_PM1_0->setValue(lastPM1_0, true);
    haSensor_PM2_5->setValue(lastPM2_5, true);
    haSensor_PM10_0->setValue(lastPM10_0, true);
  } else {
    // First boot before PMS has ever been read.
    haSensor_PM1_0->setValue(0, true);
    haSensor_PM2_5->setValue(0, true);
    haSensor_PM10_0->setValue(0, true);
  }

  uint32_t startedAt = millis();

  while (millis() - startedAt < MQTT_SETTLE_MS) {
    haMQTT->loop();
    esp_task_wdt_reset();
    delay(50);
  }

  Serial.println("MQTT publish done");

  shutdownRadios();
  return true;
}

void setupHomeAssistant() {
  byte uniqueId[] = {
    0x57, 0x53, 0x54, 0x41, 0x30, 0x30
  };

  haDevice = new HADevice(uniqueId, sizeof(uniqueId));
  haMQTT = new HAMqtt(client, *haDevice);

  haSensor_vBatt = new HASensorNumber("vBatt", HASensorNumber::PrecisionP2);
  haSensor_SoC = new HASensorNumber("SoC", HASensorNumber::PrecisionP0);
  haSensor_PM1_0 = new HASensorNumber("PM1_0", HASensorNumber::PrecisionP0);
  haSensor_PM2_5 = new HASensorNumber("PM2_5", HASensorNumber::PrecisionP0);
  haSensor_PM10_0 = new HASensorNumber("PM10_0", HASensorNumber::PrecisionP0);
  haSensor_temp = new HASensorNumber("TEMP", HASensorNumber::PrecisionP2);
  haSensor_humid = new HASensorNumber("HUMID", HASensorNumber::PrecisionP2);

  haDevice->setName("SolarWeatherStation");
  haDevice->setManufacturer("Tanat");
  haDevice->setModel("ESP32 WROVER B");
  haDevice->setSoftwareVersion("2.1.1");

  haSensor_vBatt->setUnitOfMeasurement("V");
  haSensor_vBatt->setDeviceClass("voltage");
  haSensor_vBatt->setStateClass("measurement");
  haSensor_vBatt->setIcon("mdi:alpha-v-circle-outline");
  haSensor_vBatt->setName("Battery voltage");

  haSensor_SoC->setUnitOfMeasurement("%");
  haSensor_SoC->setDeviceClass("battery");
  haSensor_SoC->setStateClass("measurement");
  haSensor_SoC->setIcon("mdi:battery");
  haSensor_SoC->setName("Battery SoC");

  haSensor_PM1_0->setUnitOfMeasurement("µg/m³");
  haSensor_PM1_0->setDeviceClass("pm1");
  haSensor_PM1_0->setStateClass("measurement");
  haSensor_PM1_0->setIcon("mdi:weather-fog");
  haSensor_PM1_0->setName("PM1");

  haSensor_PM2_5->setUnitOfMeasurement("µg/m³");
  haSensor_PM2_5->setDeviceClass("pm25");
  haSensor_PM2_5->setStateClass("measurement");
  haSensor_PM2_5->setIcon("mdi:weather-fog");
  haSensor_PM2_5->setName("PM2.5");

  haSensor_PM10_0->setUnitOfMeasurement("µg/m³");
  haSensor_PM10_0->setDeviceClass("pm10");
  haSensor_PM10_0->setStateClass("measurement");
  haSensor_PM10_0->setIcon("mdi:weather-fog");
  haSensor_PM10_0->setName("PM10");

  haSensor_temp->setUnitOfMeasurement("°C");
  haSensor_temp->setDeviceClass("temperature");
  haSensor_temp->setStateClass("measurement");
  haSensor_temp->setIcon("mdi:thermometer");
  haSensor_temp->setName("Temperature");

  haSensor_humid->setUnitOfMeasurement("%");
  haSensor_humid->setDeviceClass("humidity");
  haSensor_humid->setStateClass("measurement");
  haSensor_humid->setIcon("mdi:water-percent");
  haSensor_humid->setName("Humidity");
}

void setup() {
  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  esp_task_wdt_add(NULL);

  Serial.begin(115200);
  delay(200);

  wakeCount++;

  Serial.println();
  Serial.println("Starting Solar Weather Station");
  Serial.println("Firmware version: 2.1.1");
  Serial.println("Target runtime: about 1 week without sun");

  analogReadResolution(12);
  analogSetPinAttenuation(BATT_PIN, ADC_11db);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BATT_PIN, INPUT);
  pinMode(SENSOR_EN_PIN, OUTPUT);

  setSensorPower(false);

#if USE_CHARGE_EN_CONTROL
  pinMode(CHARGE_EN, OUTPUT);
  digitalWrite(CHARGE_EN, HIGH);
#else
  pinMode(CHARGE_EN, INPUT);
#endif

  digitalWrite(LED_BUILTIN, HIGH);

  vBatt = getBattVoltage();
  SoC = calculateSoC(vBatt);

  if (SoC <= BATT_EMPTY_PERCENT) {
    deepSleep(BATT_EMPTY);
  }

  setupHomeAssistant();
  fetchSensorValues();
  connectAndSend();

  if (SoC <= BATT_LOW_PERCENT) {
    deepSleep(BATT_LOW);
  }

  deepSleep(BATT_NORMAL);
}

void loop() {
}