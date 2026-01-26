#include <NimBLEDevice.h>

void setup() {
    Serial.begin(115200);
    NimBLEDevice::init("");

    const ble_addr_t* addr = NimBLEDevice::getAddress().getBase();

    Serial.print("BLE MAC Address: ");
    for (int i = 5; i >= 0; i--) {  // val[0]〜val[5] を逆順に表示
        Serial.printf("%02X", addr->val[i]);
        if (i > 0) Serial.print(":");
    }
    Serial.println();
}

void loop() {}
