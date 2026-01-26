#pragma once
#include <NimBLEDevice.h>

extern bool bleDeviceConnected;
extern NimBLECharacteristic *bleTxCharacteristic;

void initBLE();
