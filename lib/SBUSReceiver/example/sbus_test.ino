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
    Serial.print("  CH1:");
    Serial.print(sbus.getChannel(0));
    Serial.print("  CH2:");
    Serial.print(sbus.getChannel(1));
    Serial.print("  CH3:");
    Serial.print(sbus.getChannel(2));
    Serial.print("  CH4:");
    Serial.print(sbus.getChannel(3));
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
