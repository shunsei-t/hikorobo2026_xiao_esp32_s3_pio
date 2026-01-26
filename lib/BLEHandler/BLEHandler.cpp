#include "BLEHandler.h"

bool bleDeviceConnected = false;
NimBLECharacteristic *bleTxCharacteristic = nullptr;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) {
    bleDeviceConnected = true;
    Serial.println("BLE Connected");
  }
  void onDisconnect(NimBLEServer* s) {
    bleDeviceConnected = false;
    Serial.println("BLE Disconnected");
    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c) {
    std::string rx = c->getValue();
    Serial.print("[BLE RX] ");
    Serial.println(rx.c_str());
  }
};

void initBLE() {
  NimBLEDevice::init("ESP32-S3 Logger");

  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  NimBLEService *service = server->createService(SERVICE_UUID);

  // TX Notify
  bleTxCharacteristic = service->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      NIMBLE_PROPERTY::NOTIFY
  );

  // RX Write
  NimBLECharacteristic *rxChar = service->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      NIMBLE_PROPERTY::WRITE
  );
  rxChar->setCallbacks(new RxCallbacks());

  service->start();
  server->getAdvertising()->start();

  Serial.println("BLE Ready. Waiting for connection...");
}
