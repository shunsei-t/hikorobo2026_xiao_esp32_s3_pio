#define TASK_BNO055_DELAY_MS (10)
#define TASK_SBUS_DELAY_MS (1)
#define TASK_PID_UDELAY_MS (50) //時間厳格な実装
#define TASK_LOG_DELAY_MS (200)
#define TASK_SERVO_DELAY_MS (10)
#define TASK_UDP_DELAY_MS (50)
#define TASK_FSM_DELAY_MS (30)

#define BNO055_I2C_ADDRESS (0x28)
#define BNO055_SENSOR_ID (55)

#define SBUS_SERIAL Serial1
#define SBUS_TIMEOUT_MS 50

#define PIN_SERIAL1_RX (44)
#define PIN_SERIAL1_TX (-1)

#define PIN_SERVO_AIL_L (D0)
#define PIN_SERVO_AIL_R (D8)
#define PIN_SERVO_ELE (D1)
#define PIN_SERVO_RUD (D3)
#define PIN_SERVO_THR (D2)
#define PIN_SERVO_GEA (D6)
#define PIN_LED (LED_BUILTIN)

#define SBUS_CH_AIL (0)
#define SBUS_CH_ELE (1)
#define SBUS_CH_THR (2)
#define SBUS_CH_RUD (3)
#define SBUS_CH_GEA (4)
#define SBUS_CH_SE (7)

#define EMERGENCY_THROTTLE_DEFAULT (352) // プロポでsbusのスロットル最小値を変更した場合、ここも合わせて変更すること
#define PROPO_SW_UP (1696)
#define PROPO_SW_MID (1024)
#define PROPO_SW_DOWN (352)

#define DEFAULT_ROLL_KP (1.0f)
#define DEFAULT_ROLL_KI (0.0f)
#define DEFAULT_ROLL_KD (0.0f)
#define DEFAULT_ROLL_INTEGRAL_MIN (-10.0f)
#define DEFAULT_ROLL_INTEGRAL_MAX (10.0f)
#define DEFAULT_PITCH_KP (1.0f)
#define DEFAULT_PITCH_KI (0.0f)
#define DEFAULT_PITCH_KD (0.0f)
#define DEFAULT_PITCH_INTEGRAL_MIN (-10.0f)
#define DEFAULT_PITCH_INTEGRAL_MAX (10.0f)

#define SBUS_NUTRAL (1024)

void digitalWriteInv(uint8_t pin, uint8_t val) {
  digitalWrite(pin, val == HIGH ? LOW : HIGH);
}

int mapSbus2ServoDeg(uint16_t sbusValue) {
  float deg = (sbusValue - 1024.0)/11.2;
  return constrain(int(deg) + 90, 0, 180);
}

// --- 状態enums ---
enum FlightState {
    STATE_INIT,
    STATE_ERROR,
    STATE_AUTO,
    STATE_SEMIAUTO,
    STATE_MANUAL,
    STATE_SBUS_LOST,
};

// --- データ構造体定義 ---
struct SBUSData {
  uint32_t stamp_us;
  uint32_t last_stamp_us;
  int ch[16];
  bool ch17;
  bool ch18;
  bool failsafe;
  bool frameLost;
  bool lostConnection;
};

struct ServoData {
  int16_t aileron;
  int16_t elevator;
  int16_t rudder;
  int16_t throttle;
  int16_t gear;
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
  float integral_min;
  float integral_max;
  float derivative;

  float kp, ki, kd;
};

struct RPYData {
  PIDData roll, pitch, yaw;
};

struct UDPSendDataStruct {
  uint16_t stamp_ms;
  uint16_t sbus_data[8];
  uint8_t flight_state;
  float roll, pitch, yaw;
  float ax, ay, az;
  int16_t servo_aileron, servo_elevator, servo_rudder, servo_throttle, servo_gear;
} __attribute__((packed));

// --- UDP用構造 ---
union UDPSendData {
  UDPSendDataStruct data;
  uint8_t bytes[sizeof(UDPSendDataStruct)];
};

struct UDPReceiveDataStruct {
  uint16_t enable_stream;
  float roll_kp;
  float roll_ki;
  float roll_kd;
  float pitch_kp;
  float pitch_ki;
  float pitch_kd;
} __attribute__((packed));

union UDPReceiveData {
  UDPReceiveDataStruct data;
  uint8_t bytes[sizeof(UDPReceiveDataStruct)];
};

void ledControl(int on_time_ms, int off_time_ms, int repeat, int interval_ms) {
  if (off_time_ms == 0) {
    digitalWriteInv(LED_BUILTIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(interval_ms));
    return;
  }
  else if (on_time_ms == 0) {
    digitalWriteInv(LED_BUILTIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(interval_ms));
    return;
  }
  else {
    for (int i = 0; i < repeat; i++) {
      digitalWriteInv(LED_BUILTIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(on_time_ms));
      digitalWriteInv(LED_BUILTIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(off_time_ms));
    }
    vTaskDelay(pdMS_TO_TICKS(interval_ms));
  }
}

float pi2pi(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

int pideg2pideg(int deg) {
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return deg;
}