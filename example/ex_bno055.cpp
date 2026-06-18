#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BNO055_SAMPLERATE_DELAY_MS (20)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO055 Euler Angle Example");

  if (!bno.begin()) {
    Serial.println("No BNO055 detected ... Check wiring!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop() {
  // オイラー角データを取得（単位は度）
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // 角度を表示
  Serial.print("Yaw:");
  Serial.print(orientationData.orientation.x);
  Serial.print(",");
  Serial.print("Pitch:");
  Serial.print(orientationData.orientation.y);
  Serial.print(",");
  Serial.print("Roll:");
  Serial.print(orientationData.orientation.z);
  Serial.println();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
