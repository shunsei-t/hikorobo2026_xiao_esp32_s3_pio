#include "SBUSReceiver.h"

#define SBUS_SERIAL  Serial1   // 使用するUART

SBUSReceiver sbus(SBUS_SERIAL, 44, -1);  // RX=12, TX=13

unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  sbus.begin();
  Serial.println("S.BUS Receiver Ready");
}

void loop() {
  if (sbus.readFrame()) {
    unsigned long now = micros();
    Serial.print("DT:");
    Serial.print(now - lastTime);
    for (int i=0; i < 16; i++) {
      Serial.print("CH");
      Serial.print(i+1);
      Serial.print(":");
      Serial.print(sbus.getChannel(i));
      Serial.print(" ");
    }
    Serial.print("  CH17:");
    Serial.print(sbus.getCh17());
    Serial.print("  CH18:");
    Serial.print(sbus.getCh18());
    Serial.print("  FrameLost:");
    Serial.print(sbus.isFrameLost());
    Serial.print("  Failsafe:");
    Serial.print(sbus.isFailsafe());
    Serial.print("  LostConnection:");
    Serial.println(sbus.isLostConnection());
    lastTime = now;
  }
}
