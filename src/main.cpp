#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ESP32Servo.h>
#include <AsyncUDP.h>
#include <Wifi.h>
#include "SBUSReceiver.h"
#include "Util.h"
#include "passwd.h"

// --- 外部クラス関連 ---
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_SENSOR_ID, BNO055_I2C_ADDRESS);
SBUSReceiver sbus(SBUS_SERIAL, PIN_SERIAL1_RX, PIN_SERIAL1_TX);
Servo servoTHR, servoELE, servoRUD, servoAIL_L, servoAIL_R, servoGER;
AsyncUDP udp_client;
AsyncUDP udp_server;
IPAddress hostIP(HOST_IP);
IPAddress localIP(LOCAL_IP);
IPAddress subnet(SUBNET_MASK);
IPAddress gatewayIP(GATEWAY_IP);
IPAddress dnsIP(DNS_IP);

// --- 内部状態 ---
FlightState flightState_ = STATE_INIT;

// -- 内部データ構造体 ---
SBUSData sbusData;
BNOData bnoData;
RPYData rpyData;
ServoData servoData;

// --- Mutex ---
SemaphoreHandle_t sbusDataMutex;
SemaphoreHandle_t bnoDataMutex;
SemaphoreHandle_t rpyDataMutex;
SemaphoreHandle_t servoDataMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t serialMutex;

// --- プロトタイプ ---
void taskBNO(void *pvParameters);
void taskSBUS(void *pvParameters);
void taskPID(void *pvParameters);
void taskLOG(void *pvParameters);
void taskServo(void *pvParameters);
void taskLED(void *pvParameters);
void taskUDP(void *pvParameters);
void taskFSM(void *pvParameters);
void initDataStamp();
void initRPYData();
void initPins();
void printStat();
void printSBUSData();
void printBNOData();
void printRPYData();//TODO
void printSimple();//TODO
void recUDPDataCB(AsyncUDPPacket packet);

// --- グローバル変数 ---
static bool wifi_connected_ = false;
static bool on_stream_ = true;

void setup() {
  Serial.begin(115200);

  // Dataのタイムスタンプ初期化
  initDataStamp();

  rpyData.roll.kp = DEFAULT_ROLL_KP;
  rpyData.roll.ki = DEFAULT_ROLL_KI;
  rpyData.roll.kd = DEFAULT_ROLL_KD;
  rpyData.roll.integral_min = DEFAULT_ROLL_INTEGRAL_MIN;
  rpyData.roll.integral_max = DEFAULT_ROLL_INTEGRAL_MAX;
  rpyData.pitch.kp = DEFAULT_PITCH_KP;
  rpyData.pitch.ki = DEFAULT_PITCH_KI;
  rpyData.pitch.kd = DEFAULT_PITCH_KD;
  rpyData.pitch.integral_min = DEFAULT_PITCH_INTEGRAL_MIN;
  rpyData.pitch.integral_max = DEFAULT_PITCH_INTEGRAL_MAX;

  // Wifi系の初期化
  WiFi.config(localIP, gatewayIP, subnet, dnsIP);
  WiFi.begin(SSID, PASSWD);
  wifi_connected_ = WiFi.waitForConnectResult() == WL_CONNECTED;

  if (wifi_connected_) {
    udp_client.connect(hostIP, HOST_PORT);
    if (udp_server.listen(LOCAL_PORT)) {
      udp_server.onPacket(recUDPDataCB);
    }
  }

  // ミューテックス
  sbusDataMutex = xSemaphoreCreateMutex();
  bnoDataMutex = xSemaphoreCreateMutex();
  rpyDataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  servoDataMutex = xSemaphoreCreateMutex();

  // --- Servo 初期化 ---
  servoAIL_L.setPeriodHertz(50); // 50Hz
  servoAIL_L.attach(PIN_SERVO_AIL_L);
  servoAIL_R.setPeriodHertz(50); // 50Hz
  servoAIL_R.attach(PIN_SERVO_AIL_R);
  servoELE.setPeriodHertz(50); // 50Hz
  servoELE.attach(PIN_SERVO_ELE);
  servoRUD.setPeriodHertz(50); // 50Hz
  servoRUD.attach(PIN_SERVO_RUD);
  servoTHR.setPeriodHertz(50); // 50Hz
  servoTHR.attach(PIN_SERVO_THR);
  servoGER.setPeriodHertz(50); // 50Hz
  servoGER.attach(PIN_SERVO_GEA);

  // --- BNO055 初期化 ---
  if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(1000))) {
    if (!bno.begin()) {
      while (1) {
        Serial.println("No BNO055 detected!");
        delay(1000);
      };
    }
    // bno.setExtCrystalUse(true); // Don't work https://forums.adafruit.com/viewtopic.php?t=180919
    // bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P0); //読み取りが不安定になる
    // bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);
    xSemaphoreGive(i2cMutex);
  }

  // --- SBUS 初期化 ---
  sbus.begin();

  // --- タスク生成 --- // 数字が大きいほど優先度高い
  xTaskCreate(taskBNO,   "BNO",   6144, NULL, 5, NULL);
  xTaskCreate(taskSBUS,  "SBUS",  6144, NULL, 7, NULL);
  xTaskCreate(taskPID,   "PID",   2048, NULL, 4, NULL);
  // xTaskCreate(taskLOG,   "LOG",   4096, NULL, 1, NULL);
  xTaskCreate(taskServo, "SERVO", 6144, NULL, 8, NULL);
  xTaskCreate(taskLED,   "LED",   1024, NULL, 3, NULL);
  xTaskCreate(taskUDP,   "UDP",   6144, NULL, 2, NULL);
  xTaskCreate(taskFSM,   "FSM",   1024, NULL, 6, NULL);
  pinMode(LED_BUILTIN , OUTPUT);

  flightState_ = STATE_MANUAL;
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void recUDPDataCB(AsyncUDPPacket packet) {
  UDPReceiveData udpReceiveData;
  if (packet.length() == sizeof(UDPReceiveDataStruct)) {
    memcpy(udpReceiveData.bytes, packet.data(), sizeof(UDPReceiveDataStruct));

    on_stream_ = udpReceiveData.data.enable_stream != 0;
    // PIDパラメータ更新
    if (xSemaphoreTake(rpyDataMutex, pdMS_TO_TICKS(10))) {
      rpyData.roll.kp = udpReceiveData.data.roll_kp;
      rpyData.roll.ki = udpReceiveData.data.roll_ki;
      rpyData.roll.kd = udpReceiveData.data.roll_kd;
      rpyData.pitch.kp = udpReceiveData.data.pitch_kp;
      rpyData.pitch.ki = udpReceiveData.data.pitch_ki;
      rpyData.pitch.kd = udpReceiveData.data.pitch_kd;
      xSemaphoreGive(rpyDataMutex);
    }
  }

  digitalWriteInv(LED_BUILTIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10));
  digitalWriteInv(LED_BUILTIN, LOW);
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
      bnoData.yaw = pideg2pideg(orientationData.orientation.x);
      bnoData.pitch = -orientationData.orientation.z;
      bnoData.roll = -orientationData.orientation.y;
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
        sbusData.lostConnection = sbus.isLostConnection();

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
  static SBUSData sbusCopy;
  static BNOData bnoCopy;
  static RPYData rpyCopy;

  for (;;) {
    // --- 1️⃣ 最新のセンサ値をコピー ---
    if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(2))) {
      sbusCopy = sbusData;
      xSemaphoreGive(sbusDataMutex);
    }
    if (xSemaphoreTake(bnoDataMutex, pdMS_TO_TICKS(2))) {
      bnoCopy = bnoData;
      xSemaphoreGive(bnoDataMutex);
    }
    if (xSemaphoreTake(rpyDataMutex, pdMS_TO_TICKS(2))) {
      // ターゲット値計算
      rpyCopy = rpyData;
      xSemaphoreGive(rpyDataMutex);
    }

    // --- 2️⃣ PID演算 ---
    float dt_roll = (rpyCopy.roll.stamp_us - rpyCopy.roll.last_stamp_us) / 1000000.0f;
    float dt_pitch = (rpyCopy.pitch.stamp_us - rpyCopy.pitch.last_stamp_us) / 1000000.0f;
    float error_roll = bnoCopy.roll - (mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_AIL]) - 90.0);
    error_roll = -error_roll; // 逆転
    float error_pitch = bnoCopy.pitch - (mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_ELE]) - 90.0);
    float integral_roll = rpyCopy.roll.integral + error_roll * dt_roll;
    integral_roll = constrain(integral_roll, rpyCopy.roll.integral_min, rpyCopy.roll.integral_max);
    float integral_pitch = rpyCopy.pitch.integral + error_pitch * dt_pitch;
    integral_pitch = constrain(integral_pitch, rpyCopy.pitch.integral_min, rpyCopy.pitch.integral_max);
    float derivative_roll = (error_roll - rpyCopy.roll.error) / dt_roll;
    float derivative_pitch = (error_pitch - rpyCopy.pitch.error) / dt_pitch;
    float control_roll = rpyCopy.roll.kp*error_roll + rpyCopy.roll.ki*integral_roll + rpyCopy.roll.kd*derivative_roll;
    float control_pitch = rpyCopy.pitch.kp*error_pitch + rpyCopy.pitch.ki*integral_pitch + rpyCopy.pitch.kd*derivative_pitch;

    // --- 3️⃣ 結果を共有構造体に保存 ---
    if (xSemaphoreTake(rpyDataMutex, pdMS_TO_TICKS(5))) {
      // Roll
      rpyData.roll.last_stamp_us = rpyData.roll.stamp_us;
      rpyData.roll.stamp_us = micros();
      // rpyData.roll.target = target_roll;
      rpyData.roll.control = control_roll;
      rpyData.roll.error = error_roll;
      rpyData.roll.integral = integral_roll;
      rpyData.roll.derivative = derivative_roll;

      // Pitch
      rpyData.pitch.last_stamp_us = rpyData.pitch.stamp_us;
      rpyData.pitch.stamp_us = micros();
      // rpyData.pitch.target = target_pitch;
      rpyData.pitch.control = control_pitch;
      rpyData.pitch.error = error_pitch;
      rpyData.pitch.integral = integral_pitch;
      rpyData.pitch.derivative = derivative_pitch;
      xSemaphoreGive(rpyDataMutex);
    }

    // --- 周期を正確に保って次回まで待機 ---
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void taskLOG(void *pvParameters) {
  for (;;) {
    // printStat();
    // printSBUSData();
    // printBNOData();
    vTaskDelay(pdMS_TO_TICKS(TASK_LOG_DELAY_MS));
  }
}

void taskServo(void *pvParameters) {
  static SBUSData sbusCopy;

  for (;;) {
    if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(5))) {
      sbusCopy = sbusData;
      xSemaphoreGive(sbusDataMutex);
    }

    int16_t servo_data_ail = mapSbus2ServoDeg(SBUS_NUTRAL);
    int16_t servo_data_ele = mapSbus2ServoDeg(SBUS_NUTRAL);
    int16_t servo_data_rud = mapSbus2ServoDeg(SBUS_NUTRAL);
    int16_t servo_data_thr = mapSbus2ServoDeg(EMERGENCY_THROTTLE_DEFAULT);
    int16_t servo_data_ger = mapSbus2ServoDeg(SBUS_NUTRAL);

    if (flightState_ == STATE_SBUS_LOST || flightState_ == STATE_INIT) {
      // SBUS信号が失われた場合、スロットルを最低に設定
      servo_data_thr = mapSbus2ServoDeg(EMERGENCY_THROTTLE_DEFAULT);
    }
    else if (flightState_ == STATE_MANUAL) {
      servo_data_ail = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_AIL]);
      servo_data_ele = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_ELE]);
      servo_data_rud = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_RUD]);
      servo_data_thr = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_THR]);
      servo_data_ger = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_GEA]);
    }
    else if (flightState_ == STATE_SEMIAUTO) {
      // PID制御を適用
      if (xSemaphoreTake(rpyDataMutex, pdMS_TO_TICKS(5))) {
        float roll_control = rpyData.roll.control;
        float pitch_control = rpyData.pitch.control;
        xSemaphoreGive(rpyDataMutex);

        servo_data_ail = constrain(roll_control + 90, 0, 360);
        servo_data_ele = constrain(pitch_control + 90, 0, 360);
        servo_data_rud = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_RUD]);
        servo_data_thr = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_THR]);
        servo_data_ger = mapSbus2ServoDeg(sbusCopy.ch[SBUS_CH_GEA]);
      }
    }

    servoAIL_L.write(servo_data_ail);
    servoAIL_R.write(servo_data_ail);
    servoELE.write(servo_data_ele);
    servoRUD.write(servo_data_rud);
    servoTHR.write(servo_data_thr);
    servoGER.write(servo_data_ger);

    if (xSemaphoreTake(servoDataMutex, pdMS_TO_TICKS(5))) {
      servoData.aileron = servo_data_ail;
      servoData.elevator = servo_data_ele;
      servoData.rudder = servo_data_rud;
      servoData.throttle = servo_data_thr;
      servoData.gear = servo_data_ger;
      xSemaphoreGive(servoDataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_SERVO_DELAY_MS));
  }
}

void taskLED(void *pvParameters) {
  for (;;) {
    if (flightState_ == STATE_INIT) {
      ledControl(500, 500, 1, 0);
    }
    else if (flightState_ == STATE_ERROR) {
      ledControl(100, 100, 1, 0);
    }
    else if (flightState_ == STATE_AUTO) {
      ledControl(1, 0, 0, 1000);
    }
    else if (flightState_ == STATE_SEMIAUTO) {
      ledControl(800, 100, 1, 0);
    }
    else if (flightState_ == STATE_MANUAL) {
      ledControl(0, 1, 0, 1000);
    }
    else if (flightState_ == STATE_SBUS_LOST) {
      ledControl(100, 100, 2, 800);
    }
  }
}

void taskUDP(void *pvParameters) {
  static UDPSendData udpSendData;
  static SBUSData sbusCopy;
  static BNOData bnoCopy;
  static ServoData servoCopy;

  for (;;) {
    if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(5))) {
      sbusCopy = sbusData;
      xSemaphoreGive(sbusDataMutex);
    }
    if (xSemaphoreTake(bnoDataMutex, pdMS_TO_TICKS(5))) {
      bnoCopy = bnoData;
      xSemaphoreGive(bnoDataMutex);
    }
    if (xSemaphoreTake(servoDataMutex, pdMS_TO_TICKS(5))) {
      servoCopy = servoData;
      xSemaphoreGive(servoDataMutex);
    }

    udpSendData.data.stamp_ms = millis();
    for (int i = 0; i < 8; i++) {
      udpSendData.data.sbus_data[i] = sbusCopy.ch[i];
    }
    udpSendData.data.roll = bnoCopy.roll;
    udpSendData.data.pitch = bnoCopy.pitch;
    udpSendData.data.yaw = bnoCopy.yaw;
    udpSendData.data.ax = bnoCopy.ax;
    udpSendData.data.ay = bnoCopy.ay;
    udpSendData.data.az = bnoCopy.az;
    udpSendData.data.flight_state = static_cast<uint8_t>(flightState_);
    udpSendData.data.servo_aileron = servoCopy.aileron;
    udpSendData.data.servo_elevator = servoCopy.elevator;
    udpSendData.data.servo_rudder = servoCopy.rudder;
    udpSendData.data.servo_throttle = servoCopy.throttle;
    udpSendData.data.servo_gear = servoCopy.gear;

    if (wifi_connected_ && on_stream_) {
      udp_client.write(udpSendData.bytes, sizeof(udpSendData.data));
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_UDP_DELAY_MS));
  }
}

void taskFSM(void *pvParameters) {
  static SBUSData sbusCopy;
  static BNOData bnoCopy;

  for (;;) {
    if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(5))) {
      sbusCopy = sbusData;
      xSemaphoreGive(sbusDataMutex);
    }
    if (xSemaphoreTake(bnoDataMutex, pdMS_TO_TICKS(5))) {
      bnoCopy = bnoData;
      xSemaphoreGive(bnoDataMutex);
    }

    if (micros() - sbusCopy.stamp_us > SBUS_TIMEOUT_MS * 1000 || sbusCopy.lostConnection) {
      flightState_ = STATE_SBUS_LOST;
      vTaskDelay(pdMS_TO_TICKS(TASK_FSM_DELAY_MS));
      continue;
    }

    if (sbusCopy.ch[SBUS_CH_SE] == PROPO_SW_UP) {
      flightState_ = STATE_AUTO;
    } else if (sbusCopy.ch[SBUS_CH_SE] == PROPO_SW_MID) {
      flightState_ = STATE_SEMIAUTO;
    } else {
      flightState_ = STATE_MANUAL;
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_FSM_DELAY_MS));
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
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50))) {
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

    xSemaphoreGive(serialMutex);
  }
}

void printSBUSData() {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50))) {
    if (xSemaphoreTake(sbusDataMutex, pdMS_TO_TICKS(10))) {
      Serial.print("SBUS Data: ");
      Serial.print("CH1:");
      Serial.print(sbusData.ch[0]);
      Serial.print("CH2:");
      Serial.print(sbusData.ch[1]);
      Serial.print("CH3:");
      Serial.print(sbusData.ch[2]);
      Serial.print("CH4:");
      Serial.print(sbusData.lostConnection ? " LOST" : " OK");
      Serial.println();

      xSemaphoreGive(sbusDataMutex);
    }

    xSemaphoreGive(serialMutex);
  }
}

void printBNOData() {
  if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50))) {
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

    xSemaphoreGive(serialMutex);
  }
}
