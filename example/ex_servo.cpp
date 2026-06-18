#include <ESP32Servo.h>

Servo sv;

void setup() {
  Serial.begin(115200);
  sv.setPeriodHertz(50); // 50Hz
  sv.attach(D0); // D9ピンに接続
}

void loop() {
  sv.writeMicroseconds(1500); // サーボを中立位置に設定
  delay(1000);
  sv.writeMicroseconds(1000); // サーボを最小位置に設定
  delay(1000);
  sv.writeMicroseconds(2000); // サーボを最大位置に設定
  delay(1000);
}