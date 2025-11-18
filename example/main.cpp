#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "SBUSReceiver.h"
#include "Util.h"

// --- BNO055 ---
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_SENSOR_ID, BNO055_I2C_ADDRESS);
// --- SBUS ---
SBUSReceiver sbus(SBUS_SERIAL, PIN_SERIAL1_RX, PIN_SERIAL1_TX);

// --- 構造体定義 ---
struct SBUSData {
  uint32_t stamp_us;
  uint32_t last_stamp_us;
  int ch[16];
  bool ch17;
  bool ch18;
  bool failsafe;
  bool frameLost;
};

struct BNOData {
  uint32_t stamp_us;
  uint32_t last_stamp_us;
  float roll, pitch, yaw;
  float rx, ry, rz;
  float ax, ay, az;
};

struct PIDData {
  uint16_t stamp_us;
  uint16_t last_stamp_us;
  float target;
  float control;
  float error;
  float integral;
  float derivative;

  float kp, ki, kd;
};

struct RPYData {
  PIDData roll, pitch, yaw;
};


SBUSData sbusData;
BNOData bnoData;
RPYData rpyData;

// --- Mutex ---
SemaphoreHandle_t sbusDataMutex;
SemaphoreHandle_t bnoDataMutex;
SemaphoreHandle_t rpyDataMutex;
SemaphoreHandle_t i2cMutex;

// --- プロトタイプ ---
void taskBNO(void *pvParameters);
void taskSBUS(void *pvParameters);
void taskPID(void *pvParameters);
void taskLOG(void *pvParameters);
void initDataStamp();
void printStat();//TODO
void printSBUSData();//TODO
void printBNOData();//TODO
void printRPYData();//TODO
void printSimple();//TODO

void setup() {
  Serial.begin(115200);

  // Dataのタイムスタンプ初期化
  initDataStamp();

  // ミューテックス
  sbusDataMutex = xSemaphoreCreateMutex();
  bnoDataMutex = xSemaphoreCreateMutex();
  rpyDataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();

  // --- BNO055 初期化 ---
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000))) {
    if (!bno.begin()) {
      Serial.println("No BNO055 detected!");
      while (1);
    }
    bno.setExtCrystalUse(true);
    xSemaphoreGive(i2cMutex);
  }

  // --- SBUS 初期化 ---
  sbus.begin();

  // --- タスク生成 ---
  xTaskCreate(taskBNO, "BNO", 4096, NULL, 2, NULL);
  xTaskCreate(taskSBUS, "SBUS", 4096, NULL, 1, NULL);
  xTaskCreate(taskPID, "PID", 4096, NULL, 3, NULL);
  xTaskCreate(taskLOG, "LOG", 4096, NULL, 4, NULL);

  Serial.println("RTOS Tasks started!");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// --- BNO055タスク ---
void taskBNO(void *pvParameters) {
  sensors_event_t orientationData, angVelocityData, linearAccelData;
  for (;;) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50))) {
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      xSemaphoreGive(i2cMutex);
    }

    // 値を共有構造体に書き込み
    if (xSemaphoreTake(bnoDataMutex, pdMS_TO_TICKS(10))) {
      bnoData.last_stamp_us = bnoData.stamp_us;
      bnoData.stamp_us = micros();
      bnoData.yaw = orientationData.orientation.x;
      bnoData.pitch = orientationData.orientation.y;
      bnoData.roll = orientationData.orientation.z;
      bnoData.rx = angVelocityData.gyro.x;
      bnoData.ry = angVelocityData.gyro.y;
      bnoData.rz = angVelocityData.gyro.z;
      bnoData.ax = linearAccelData.acceleration.x;
      bnoData.ay = linearAccelData.acceleration.y;
      bnoData.az = linearAccelData.acceleration.z;
      xSemaphoreGive(bnoDataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_BNO055_DELAY_MS));
  }
}

// --- SBUSタスク ---
void taskSBUS(void *pvParameters) {
  for (;;) {
    if (sbus.readFrame()) {
      if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(10))) {
        sbusData.last_stamp_us = sbusData.stamp_us;
        sbusData.stamp_us = micros();
        for (int i = 0; i < 16; i++)
          sbusData.ch[i] = sbus.getChannel(i);
        sbusData.ch17 = sbus.getCh17();
        sbusData.ch18 = sbus.getCh18();
        sbusData.failsafe = sbus.isFailsafe();
        sbusData.frameLost = sbus.isFrameLost();
        xSemaphoreGive(sbusDataMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(TASK_SBUS_DELAY_MS));
  }
}

// --- PID制御タスク ---
void taskPID(void *pvParameters) {
  const TickType_t xFrequency = pdMS_TO_TICKS(TASK_PID_UDELAY_MS);
  TickType_t xLastWakeTime = xTaskGetTickCount();   // 初期化

  for (;;) {
    // --- 1️⃣ 最新のセンサ値をコピー ---
    SBUSData sbusCopy;
    BNOData bnoCopy;
    if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(5))) {
      sbusCopy = sbusData;
      xSemaphoreGive(sbusDataMutex);
    }
    if (xSemaphoreTake(bnoDataMutex, pdMS_TO_TICKS(5))) {
      bnoCopy = bnoData;
      xSemaphoreGive(bnoDataMutex);
    }

    // --- 2️⃣ PID演算 ---

    // --- 周期を正確に保って次回まで待機 ---
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void taskLOG(void *pvParameters) {
  for (;;) {
    printStat();
    printSBUSData();
    printBNOData();
    vTaskDelay(pdMS_TO_TICKS(TASK_LOG_DELAY_MS));
  }
}

void initDataStamp() {
  sbusData.stamp_us = micros();
  sbusData.last_stamp_us = micros();
  bnoData.stamp_us = micros();
  bnoData.last_stamp_us = micros();
  rpyData.roll.stamp_us = micros();
  rpyData.roll.last_stamp_us = micros();
  rpyData.pitch.stamp_us = micros();
  rpyData.pitch.last_stamp_us = micros();
  rpyData.yaw.stamp_us = micros();
  rpyData.yaw.last_stamp_us = micros();
}

void printStat() {
  Serial.print("Time stamp durations: ");
  if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(10))) {
    float dt_ms = (sbusData.stamp_us - sbusData.last_stamp_us) / 1000.0f;

    Serial.print("SBUS [");
    Serial.print(dt_ms, 2);  // ← 小数2桁
    Serial.print(" ms] ");
    xSemaphoreGive(sbusDataMutex);
  }
  if (xSemaphoreTake(bnoDataMutex, pdMS_TO_TICKS(10))) {
    float dt_ms = (bnoData.stamp_us - bnoData.last_stamp_us) / 1000.0f;

    Serial.print("BNO [");
    Serial.print(dt_ms, 2);  // ← 小数2桁
    Serial.print(" ms] ");
    xSemaphoreGive(bnoDataMutex);
  }
  Serial.println();
}

void printSBUSData() {
  if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(10))) {
    Serial.print("SBUS Data: ");
    for (int i = 0; i < 16; i++) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(sbusData.ch[i]);
      Serial.print(" ");
    }
    Serial.print("CH17: ");
    Serial.print(sbusData.ch17);
    Serial.print(" CH18: ");
    Serial.print(sbusData.ch18);
    Serial.print(" Failsafe: ");
    Serial.print(sbusData.failsafe);
    Serial.print(" FrameLost: ");
    Serial.println(sbusData.frameLost);
    xSemaphoreGive(sbusDataMutex);
  }
}

void printBNOData() {
  if (xSemaphoreTake(bnoDataMutex, pdMS_TO_TICKS(10))) {
    Serial.print("BNO Data: ");
    Serial.print("Yaw: ");
    Serial.print(bnoData.yaw);
    Serial.print(" Pitch: ");
    Serial.print(bnoData.pitch);
    Serial.print(" Roll: ");
    Serial.print(bnoData.roll);
    Serial.print(" Rx: ");
    Serial.print(bnoData.rx);
    Serial.print(" Ry: ");
    Serial.print(bnoData.ry);
    Serial.print(" Rz: ");
    Serial.print(bnoData.rz);
    Serial.print(" Ax: ");
    Serial.print(bnoData.ax);
    Serial.print(" Ay: ");
    Serial.print(bnoData.ay);
    Serial.print(" Az: ");
    Serial.println(bnoData.az);
    xSemaphoreGive(bnoDataMutex);
  }
}