#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BluetoothSerial.h"
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <EEPROM.h>

// --- UUIDs ---
#define SERVICE_UUID_LNS "00001819-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LN_FEATURE "00002a6a-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_LOCATION_SPEED "00002a67-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_POSITION_QUALITY "00002a69-0000-1000-8000-00805f9b34fb"
#define SERVICE_UUID_CONFIG "c48e6067-5295-48d3-8d5c-0395f61792b1"
#define CHARACTERISTIC_UUID_SMA_WINDOW "c48e6068-5295-48d3-8d5c-0395f61792b1"
#define CHARACTERISTIC_UUID_GNSS_RATE "c48e6069-5295-48d3-8d5c-0395f61792b1"

// --- Custom BLE (CANBUS-GPS device style) UUIDs ---
#define SERVICE_UUID_CUSTOM_GPS "00000001-0000-00fd-8933-990d6f411ff8"
#define CHARACTERISTIC_UUID_CUSTOM_GPS_MAIN "00000002-0000-00fd-8933-990d6f411ff8"
#define CHARACTERISTIC_UUID_CUSTOM_GPS_TIME "00000003-0000-00fd-8933-990d6f411ff8"

// --- EEPROM Settings ---
#define EEPROM_SIZE 16
#define ADDR_SMA_SIZE 0
#define ADDR_GNSS_RATE 1

int smaWindowSize = 5;
const uint32_t gpsBaud = 115200;
int currentGnssRateHz = 10;
unsigned long bleNotifyIntervalMs = 100;

double lastLat = 0.0, lastLng = 0.0;
unsigned long lastSpeedCalcMillis = 0;
float smoothedSpeedKph = 0.0;
float *speedSamples = nullptr;
int currentSampleIndex = 0;
int filledSamples = 0;
const int MAX_SMA_WINDOW_SIZE = 50;

// --- BLE ---
BLECharacteristic *pLocationSpeedCharacteristic;
BLECharacteristic *pPositionQualityCharacteristic;
BLECharacteristic *pSmaWindowCharacteristic;
BLECharacteristic *pGnssRateCharacteristic;
bool bleConnected = false;
unsigned long lastGpsCheck = 0;

// --- Custom BLE (CANBUS-GPS device style) ---
BLEService *pCustomGpsService;
BLECharacteristic *pCustomGpsMainCharacteristic;
BLECharacteristic *pCustomGpsTimeCharacteristic;
unsigned long lastCustomGpsNotifyMs = 0;

// --- BT classic ---
BluetoothSerial SerialBT;
bool sppConnected = false;

#pragma pack(push, 1)
struct LocationAndSpeed_19Byte {
  uint16_t flags;
  uint16_t instantaneousSpeed;
  int32_t latitude;
  int32_t longitude;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct PositionQuality_15Byte {
  uint16_t flags;
  uint16_t hpe;
  uint16_t vpe;
  uint16_t hdop;
  uint16_t vdop;
  uint16_t pdop;
  uint8_t fixType;
  uint8_t satellitesInView;
};
#pragma pack(pop)

// --- Custom GPS packet (CANBUS-GPS device style, 20 bytes) ---
#pragma pack(push, 1)
struct CustomGpsMainPacket {
  uint8_t data[20];
};
#pragma pack(pop)

String nameBLE = "KawaiiMyGNSSiOS";
String nameSPP = "KawaiiMyGNSSAndroid";
#define RX_PIN 23
#define TX_PIN 22
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

BLEAdvertising *pAdvertising;
BLEServer *pServer;
bool wasSppConnected = false;
bool wasBleConnected = false;

void setSmaWindowSize(int newSize);
void sendUb(byte *ubloxCommand, size_t len);
bool setGnssRate(int newRateHz);
void saveGnssConfig();
void sendLocationOverSpp(String nmea);
void sendLocationOverBle();
void sendPositionQualityOverBle();
void updateSmoothedSpeed(float rawSpeedKph);
byte calculateNmeaChecksum(String nmea);

// --- Custom BLE (CANBUS-GPS device style) GPS packet送信 ---
void sendCustomGpsOverBle() {
  if (!bleConnected)
    return;
  if (millis() - lastCustomGpsNotifyMs < bleNotifyIntervalMs)
    return;
  lastCustomGpsNotifyMs = millis();

  if (gps.location.isValid() && gps.time.isValid()) {
    CustomGpsMainPacket packet;
    memset(&packet, 0, sizeof(packet));

    // --- CANBUS-GPS style: パケット詳細 ---
    // tempData[0] = ((gpsSyncBits & 0x7) << 5) | ((timeSinceHourStart >> 16) & 0x1F);
    // tempData[1] = timeSinceHourStart >> 8;
    // tempData[2] = timeSinceHourStart;
    // tempData[3] = ((min(0x3, gps->fixquality) & 0x3) << 6) | ((min(0x3F, gps->satellites)) & 0x3F);
    // tempData[4-7] = latitude
    // tempData[8-11] = longitude
    // tempData[12-13] = altitude
    // tempData[14-15] = speed
    // tempData[16-17] = bearing
    // tempData[18] = HDOP
    // tempData[19] = 0xFF

    int gpsSyncBits = 0;  // 簡易版: 毎回0に
    int dateAndHour = (gps.date.year() * 8928) + ((gps.date.month() - 1) * 744) + ((gps.date.day() - 1) * 24) + gps.time.hour();
    int timeSinceHourStart = (gps.time.minute() * 30000) + (gps.time.second() * 500);  // msは未使用

    int latitude = (int)(gps.location.lat() * 10000000);
    int longitude = (int)(gps.location.lng() * 10000000);

    float alt = gps.altitude.meters();
    float altCalc = (float)round(alt + 500.0f);
    int altitude;
    if (alt > 6000.0f) {
      int tmp = (altCalc > 0.0f ? (int)altCalc : 0);
      altitude = (tmp & 0x7FFF) | 0x8000;
    } else {
      int tmp = ((altCalc * 10.0f) > 0.0f ? (int)(altCalc * 10.0f) : 0);
      altitude = tmp & 0x7FFF;
    }

    int speedKmph = (int)gps.speed.kmph();
    int speed;
    if (speedKmph > 600.0f) {
      int tmp = ((speedKmph * 10.0f) > 0.0f ? (int)round(speedKmph * 10.0f) : 0);
      speed = (tmp & 0x7FFF) | 0x8000;
    } else {
      int tmp = ((speedKmph * 100.0f) > 0.0f ? (int)round(speedKmph * 100.0f) : 0);
      speed = tmp & 0x7FFF;
    }

    int bearing = max((double)0.0, round(gps.course.deg() * 100.0f));
    int hdop = gps.hdop.isValid() ? round(gps.hdop.hdop() * 10.0f) : 0;

    // 衛星数は最大63
    uint8_t satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    if (satellites > 0x3F) satellites = 0x3F;

    packet.data[0] = ((gpsSyncBits & 0x7) << 5) | ((timeSinceHourStart >> 16) & 0x1F);
    packet.data[1] = (timeSinceHourStart >> 8) & 0xFF;
    packet.data[2] = (timeSinceHourStart)&0xFF;
    packet.data[3] = (((0x1 & 0x3) << 6) | (satellites & 0x3F));

    packet.data[4] = (latitude >> 24) & 0xFF;
    packet.data[5] = (latitude >> 16) & 0xFF;
    packet.data[6] = (latitude >> 8) & 0xFF;
    packet.data[7] = (latitude >> 0) & 0xFF;
    packet.data[8] = (longitude >> 24) & 0xFF;
    packet.data[9] = (longitude >> 16) & 0xFF;
    packet.data[10] = (longitude >> 8) & 0xFF;
    packet.data[11] = (longitude >> 0) & 0xFF;
    packet.data[12] = (altitude >> 8) & 0xFF;
    packet.data[13] = (altitude)&0xFF;
    packet.data[14] = (speed >> 8) & 0xFF;
    packet.data[15] = (speed)&0xFF;
    packet.data[16] = (bearing >> 8) & 0xFF;
    packet.data[17] = (bearing)&0xFF;
    packet.data[18] = hdop & 0xFF;
    packet.data[19] = 0xFF;  // reserved

    pCustomGpsMainCharacteristic->setValue(packet.data, 20);
    pCustomGpsMainCharacteristic->notify();

    // time characteristic
    uint8_t timePacket[3];
    timePacket[0] = ((gpsSyncBits & 0x7) << 5) | ((dateAndHour >> 16) & 0x1F);
    timePacket[1] = (dateAndHour >> 8) & 0xFF;
    timePacket[2] = (dateAndHour)&0xFF;
    pCustomGpsTimeCharacteristic->setValue(timePacket, 3);
    pCustomGpsTimeCharacteristic->notify();
  }
}

// --- BLE Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    bleConnected = true;
    Serial.println("\n✅ Central device connected!");
  }
  void onDisconnect(BLEServer *pServer) {
    bleConnected = false;
    Serial.println("❌ Central device disconnected.");
    pAdvertising->start();
  }
};

class SmaConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    if (value.length() == 1) {
      int newSize = (int)value[0];
      Serial.printf("Received new SMA size via BLE (raw byte): %d\n", newSize);
      setSmaWindowSize(newSize);
      pCharacteristic->setValue(String(smaWindowSize).c_str());
    }
  }
};

class GnssConfigCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    uint32_t receivedValue = 0;

    if (value.length() > 0 && value.length() <= 4) {
      for (int i = 0; i < value.length(); i++) {
        receivedValue = (receivedValue << 8) | (uint8_t)value[i];
      }
    } else {
      Serial.println("Invalid data length received via BLE.");
      return;
    }

    if (pCharacteristic == pGnssRateCharacteristic) {
      if (setGnssRate((int)receivedValue)) {
        pGnssRateCharacteristic->setValue(String(currentGnssRateHz).c_str());
      }
    }
  }
};

void updateSmoothedSpeed(float rawSpeedKph) {
  if (smaWindowSize == 0) {
    smoothedSpeedKph = rawSpeedKph;
  }

  if (speedSamples != nullptr && smaWindowSize > 0) {
    speedSamples[currentSampleIndex] = rawSpeedKph;
    currentSampleIndex = (currentSampleIndex + 1) % smaWindowSize;

    if (filledSamples < smaWindowSize) filledSamples++;
    float sum = 0.0;
    for (int i = 0; i < filledSamples; i++) {
      sum += speedSamples[i];
    }
    smoothedSpeedKph = sum / filledSamples;
  }
}

void setSmaWindowSize(int newSize) {
  if (newSize < 0) return;
  if (newSize > MAX_SMA_WINDOW_SIZE) newSize = MAX_SMA_WINDOW_SIZE;

  if (newSize == 0) {
    if (speedSamples != nullptr) {
      delete[] speedSamples;
      speedSamples = nullptr;
    }
    Serial.println("Speed smoothing DISABLED.");
  } else {
    if (speedSamples != nullptr) delete[] speedSamples;

    speedSamples = new float[newSize];
    for (int i = 0; i < newSize; i++) speedSamples[i] = 0.0;

    currentSampleIndex = 0;
    filledSamples = 0;
    smoothedSpeedKph = 0.0;

    Serial.printf("Speed smoothing ENABLED with window size: %d\n", newSize);
  }

  smaWindowSize = newSize;
  EEPROM.write(ADDR_SMA_SIZE, smaWindowSize);
  EEPROM.commit();
  Serial.println("Saved new SMA setting to EEPROM.");
}

void sendUb(byte *ubloxCommand, size_t len) {
  byte ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < len - 2; i++) {
    ck_a = ck_a + ubloxCommand[i];
    ck_b = ck_b + ck_a;
  }
  ubloxCommand[len - 2] = ck_a;
  ubloxCommand[len - 1] = ck_b;
  Serial.print("Sending UBX Command: ");
  for (size_t i = 0; i < len; i++) {
    gpsSerial.write(ubloxCommand[i]);
    Serial.printf("%02X ", ubloxCommand[i]);
  }
  Serial.println();
}

bool setGnssRate(int newRateHz) {
  if (newRateHz < 1 || newRateHz > 25) {
    Serial.printf("Invalid rate: %d Hz. Must be between 1 and 25.\n", newRateHz);
    return false;
  }

  bleNotifyIntervalMs = 1000 / newRateHz;
  Serial.printf("Setting measurement rate to %d Hz (%d ms)...\n", newRateHz, bleNotifyIntervalMs);

  currentGnssRateHz = newRateHz;
  EEPROM.write(ADDR_GNSS_RATE, (uint8_t)currentGnssRateHz);
  EEPROM.commit();

  byte ubxCfgRate[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00 };
  ubxCfgRate[6] = (uint16_t)bleNotifyIntervalMs & 0xFF;
  ubxCfgRate[7] = ((uint16_t)bleNotifyIntervalMs >> 8) & 0xFF;
  sendUb(ubxCfgRate, sizeof(ubxCfgRate));
  delay(150);
  saveGnssConfig();
  Serial.printf("BLE notify interval updated to %lu ms.\n", bleNotifyIntervalMs);
  Serial.println("Saved new Rate setting to EEPROM.");

  return true;
}

void saveGnssConfig() {
  Serial.println("Saving configuration to GNSS receiver's BBR/Flash...");
  byte ubxCfgCfg[] = {
    0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF,
    0x00, 0x00,
    0x03, 0x00, 0x00,
    0x17, 0x00, 0x00
  };
  sendUb(ubxCfgCfg, sizeof(ubxCfgCfg));
  delay(100);
}

byte calculateNmeaChecksum(String nmea) {
  byte checksum = 0;
  int start = nmea.indexOf('$') + 1;
  int end = nmea.indexOf('*');
  if (start > 0 && end > 0) {
    for (int i = start; i < end; i++) {
      checksum ^= nmea.charAt(i);
    }
  }
  return checksum;
}

void sendLocationOverSpp(String nmea) {
  if (!sppConnected) return;

  if (!nmea.startsWith("$") || !nmea.substring(3, 6).equals("RMC")) {
    return;
  }

  int commaCount = 0;
  int speedStartIndex = -1;
  int speedEndIndex = -1;

  for (int i = 0; i < nmea.length(); i++) {
    if (nmea.charAt(i) == ',') {
      commaCount++;
      if (commaCount == 7) {
        speedStartIndex = i + 1;
      } else if (commaCount == 8) {
        speedEndIndex = i;
        break;
      }
    }
  }

  if (speedStartIndex == -1 || speedEndIndex == -1) {
    return;
  }

  String beforeSpeed = nmea.substring(0, speedStartIndex);
  String afterSpeed = nmea.substring(speedEndIndex);

  float speedKnots = smoothedSpeedKph / 1.852;
  String newSpeedStr = String(speedKnots, 2);

  String modified = beforeSpeed + newSpeedStr + afterSpeed;

  byte newChecksum = calculateNmeaChecksum(modified);

  modified.remove(modified.indexOf('*'));
  modified += "*";
  if (newChecksum < 16) {
    modified += "0";
  }
  modified += String(newChecksum, HEX);
  modified.toUpperCase();

  SerialBT.print(modified);
}

void sendLocationOverBle() {
  static int count = 0;
  static int prevSec = 0;

  if (millis() - lastGpsCheck > bleNotifyIntervalMs && bleConnected) {
    lastGpsCheck = millis();

    if (gps.location.isValid() && gps.time.isValid()) {
      LocationAndSpeed_19Byte loc_data;
      memset(&loc_data, 0, sizeof(loc_data));

      loc_data.flags = 0b0000000001000101;

      float speedKmph = 0.0;
      if (smaWindowSize > 0) {
        speedKmph = smoothedSpeedKph;
      } else {
        speedKmph = gps.speed.kmph();
      }

      loc_data.instantaneousSpeed = (uint16_t)(speedKmph * 100.0);
      loc_data.latitude = (int32_t)(gps.location.lat() * 10000000);
      loc_data.longitude = (int32_t)(gps.location.lng() * 10000000);

      loc_data.year = gps.date.year();
      loc_data.month = gps.date.month();
      loc_data.day = gps.date.day();
      loc_data.hour = gps.time.hour();
      loc_data.minute = gps.time.minute();
      loc_data.second = gps.time.second();

      pLocationSpeedCharacteristic->setValue((uint8_t *)&loc_data, sizeof(loc_data));
      pLocationSpeedCharacteristic->notify();

      if (prevSec == loc_data.second) {
        count++;
      } else {
        Serial.printf("%dHz\n", count);
        prevSec = loc_data.second;
        count = 0;
      }
    }
  }
}

void sendPositionQualityOverBle() {
  if (bleConnected && gps.hdop.isValid() && gps.satellites.isValid()) {
    PositionQuality_15Byte quality_data;
    memset(&quality_data, 0, sizeof(quality_data));

    quality_data.flags = 0b0000000000001000;
    quality_data.hpe = 0;
    quality_data.vpe = 0;
    quality_data.hdop = (uint16_t)(gps.hdop.hdop() * 100);
    quality_data.vdop = 0;
    quality_data.pdop = 0;
    quality_data.satellitesInView = gps.satellites.value();

    int fixType = 0;
    if (gps.location.isUpdated() && gps.location.isValid()) {
      fixType = 3;
    }
    quality_data.fixType = (uint8_t)fixType;

    pPositionQualityCharacteristic->setValue((uint8_t *)&quality_data, sizeof(quality_data));
    pPositionQualityCharacteristic->notify();
  }
}

// --- setup() ---
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  delay(250);

  int storedSize = EEPROM.read(ADDR_SMA_SIZE);
  smaWindowSize = (storedSize < 0 || storedSize > MAX_SMA_WINDOW_SIZE) ? 7 : storedSize;
  Serial.printf("Loaded SMA size from EEPROM: %d\n", smaWindowSize);
  setSmaWindowSize(smaWindowSize);

  int storedRate = EEPROM.read(ADDR_GNSS_RATE);
  currentGnssRateHz = (storedRate < 1 || storedRate > 25) ? 10 : storedRate;
  Serial.printf("Loaded GNSS Rate from EEPROM: %d Hz\n", currentGnssRateHz);
  bleNotifyIntervalMs = 1000 / currentGnssRateHz;

  Serial.println("\n🚀 Starting BLE GNSS module v2...");
  gpsSerial.begin(gpsBaud, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.printf("🛰️  GPS Serial started at %u baud on RX:%d, TX:%d.\n", gpsBaud, RX_PIN, TX_PIN);

  btStart();

  BLEDevice::init(nameBLE.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // --- LNS Service ---
  BLEService *pLnsService = pServer->createService(SERVICE_UUID_LNS);
  BLECharacteristic *pLnFeatureCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LN_FEATURE, BLECharacteristic::PROPERTY_READ);
  const uint32_t ln_feature = 0b0000000001000101;
  pLnFeatureCharacteristic->setValue((uint8_t *)&ln_feature, 4);
  pLocationSpeedCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_LOCATION_SPEED, BLECharacteristic::PROPERTY_NOTIFY);
  pLocationSpeedCharacteristic->addDescriptor(new BLE2902());
  pPositionQualityCharacteristic = pLnsService->createCharacteristic(CHARACTERISTIC_UUID_POSITION_QUALITY, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pPositionQualityCharacteristic->addDescriptor(new BLE2902());
  pLnsService->start();

  // --- Config Service ---
  BLEService *pConfigService = pServer->createService(SERVICE_UUID_CONFIG);
  pSmaWindowCharacteristic = pConfigService->createCharacteristic(CHARACTERISTIC_UUID_SMA_WINDOW, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pSmaWindowCharacteristic->setCallbacks(new SmaConfigCallbacks());
  pSmaWindowCharacteristic->setValue(String(smaWindowSize).c_str());
  pGnssRateCharacteristic = pConfigService->createCharacteristic(CHARACTERISTIC_UUID_GNSS_RATE, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pGnssRateCharacteristic->setCallbacks(new GnssConfigCallbacks());
  pGnssRateCharacteristic->setValue(String(currentGnssRateHz).c_str());
  pConfigService->start();

  // --- Custom GPS BLE Service (CANBUS-GPS device style) ---
  pCustomGpsService = pServer->createService(SERVICE_UUID_CUSTOM_GPS);
  pCustomGpsMainCharacteristic = pCustomGpsService->createCharacteristic(CHARACTERISTIC_UUID_CUSTOM_GPS_MAIN, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pCustomGpsMainCharacteristic->addDescriptor(new BLE2902());
  pCustomGpsTimeCharacteristic = pCustomGpsService->createCharacteristic(CHARACTERISTIC_UUID_CUSTOM_GPS_TIME, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  pCustomGpsTimeCharacteristic->addDescriptor(new BLE2902());
  pCustomGpsService->start();

  pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_LNS);
  pAdvertising->addServiceUUID(SERVICE_UUID_CONFIG);
  pAdvertising->addServiceUUID(SERVICE_UUID_CUSTOM_GPS);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x0C);
  pAdvertising->start();

  SerialBT.begin(nameSPP.c_str(), false, gpsBaud);

  Serial.printf("📡 BLE advertising started as '%s'.\n", nameBLE.c_str());
}

void loop() {
  static String nmeaMsg = "";

  sppConnected = SerialBT.connected();

  if (bleConnected && !wasBleConnected) {
    if (SerialBT.connected()) {
      Serial.println("BLE connected. Ending SPP session.");
      SerialBT.end();
    }
    wasBleConnected = true;
    wasSppConnected = false;
  } else if (sppConnected && !wasSppConnected) {
    pAdvertising->stop();
    Serial.println("SPP connected. Stopping BLE advertising.");
    wasSppConnected = true;
    wasBleConnected = false;
  } else if (!bleConnected && !sppConnected && (wasBleConnected || wasSppConnected)) {
    SerialBT.begin(nameSPP.c_str());
    pAdvertising->start();
    Serial.println("Disconnected. Restarting BLE advertising and SPP.");
    wasBleConnected = false;
    wasSppConnected = false;
  }

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.endsWith("Hz")) {
      int newRate = input.substring(0, input.length() - 2).toInt();
      if (setGnssRate(newRate)) {
        pGnssRateCharacteristic->setValue(String(currentGnssRateHz).c_str());
      }
    } else {
      int newSize = input.toInt();
      if (String(newSize) == input) {
        setSmaWindowSize(newSize);
        pSmaWindowCharacteristic->setValue(String(smaWindowSize).c_str());
      }
    }
  }

  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();

    if (sppConnected) {
      SerialBT.write(c);
      Serial.write(c);
    } else if (gps.encode(c)) {
      if (gps.speed.isValid()) {
        float rawSpeedKph = gps.speed.kmph();
        updateSmoothedSpeed(rawSpeedKph);
      }
    }
  }

  if (bleConnected) {
    sendLocationOverBle();
    sendPositionQualityOverBle();
    sendCustomGpsOverBle();  // --- 追加: CANBUS-GPS device style BLE送信
  }
}