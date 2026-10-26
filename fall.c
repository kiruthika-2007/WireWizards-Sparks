/*
 * Fall Detection - ESP32 + MPU6050 + PASSIVE BUZZER + OLED (no buttons)
 *
 * Wiring:
 *   MPU6050 VCC -> 3V3   GND -> GND   SDA -> GPIO21  SCL -> GPIO22  AD0 -> GND
 *   OLED    VCC -> 3V3   GND -> GND   SDA -> GPIO32  SCL -> GPIO33
 *   Passive Buzzer (+) -> GPIO25   (-) -> GND
 *   LED -> GPIO2 (onboard)
 *
 * Logic:
 *   - Movement (3 samples)          -> "Fall detected" + beep + OLED "FALL!"
 *   - No movement for STILL_TIME_MS -> "Free fall detected" + OLED message
 *   - Alarm stops automatically after ALERT_TIME_MS
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Pins ----------
#define SDA_PIN     21     // MPU6050 (Wire)
#define SCL_PIN     22
#define OLED_SDA    32     // OLED (Wire1)
#define OLED_SCL    33
#define BUZZER_PIN  23
#define LED_PIN     2
#define BUZZER_CH   0      // used only on ESP32 core 2.x

// ---------- OLED ----------
#define OLED_ADDR   0x3C   // try 0x3D if screen stays blank
Adafruit_SSD1306 display(128, 64, &Wire1, -1);

void showMsg(const char* l1, const char* l2 = "") {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 8);
  display.println(l1);
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.println(l2);
  display.display();
}

// ---------- MPU6050 ----------
#define MPU_ADDR    0x68
#define ACCEL_SCALE 4096.0f   // LSB/g at +-8g
#define GYRO_SCALE  65.5f     // LSB/(deg/s) at +-500 dps

// ---------- Buzzer ----------
const int BUZZER_FREQ = 2000;   // Hz

// ---------- Tunable thresholds ----------
const float MOVE_ACCEL_DEV_G   = 0.35f;     // |SVM - 1g| above this = movement
const float MOVE_GYRO_DPS      = 60.0f;     // rotation above this = movement
const int   MOVE_CONFIRM_COUNT = 3;         // continuous samples needed
const unsigned long STILL_TIME_MS = 1000;   // no movement this long -> msg
const unsigned long ALERT_TIME_MS = 10000;  // alarm time
const unsigned long COOLDOWN_MS   = 3000;   // ignore sensor after alarm

// ---------- State ----------
enum State { IDLE, ALERT };
State state = IDLE;
unsigned long stateStart = 0;
unsigned long cooldownUntil = 0;
unsigned long lastMoveTime = 0;
bool stillMsgShown = false;
int moveCount = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// ---------- Passive buzzer helpers (core 2.x and 3.x) ----------
void buzzerInit() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(BUZZER_PIN, BUZZER_FREQ, 8);
#else
  ledcSetup(BUZZER_CH, BUZZER_FREQ, 8);
  ledcAttachPin(BUZZER_PIN, BUZZER_CH);
#endif
}

void buzzerTone(int freq) {   // 0 = silence
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(BUZZER_PIN, freq);
#else
  ledcWriteTone(BUZZER_CH, freq);
#endif
}

// ---------- MPU helpers ----------
void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuRead(float &svm, float &gyroMag) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  float fx = ax / ACCEL_SCALE, fy = ay / ACCEL_SCALE, fz = az / ACCEL_SCALE;
  float rx = (gx / GYRO_SCALE) - gyroBiasX;
  float ry = (gy / GYRO_SCALE) - gyroBiasY;
  float rz = (gz / GYRO_SCALE) - gyroBiasZ;

  svm     = sqrtf(fx * fx + fy * fy + fz * fz);
  gyroMag = sqrtf(rx * rx + ry * ry + rz * rz);
  return true;
}

void calibrateGyro() {
  Serial.println("Calibrating gyro - keep sensor still...");
  float sx = 0, sy = 0, sz = 0;
  const int N = 300;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6);
    int16_t gx = (Wire.read() << 8) | Wire.read();
    int16_t gy = (Wire.read() << 8) | Wire.read();
    int16_t gz = (Wire.read() << 8) | Wire.read();
    sx += gx / GYRO_SCALE;
    sy += gy / GYRO_SCALE;
    sz += gz / GYRO_SCALE;
    delay(5);
  }
  gyroBiasX = sx / N;
  gyroBiasY = sy / N;
  gyroBiasZ = sz / N;
  Serial.println("Calibration done.");
}

void setState(int s) {
  state =(State)s;
  stateStart = millis();
}

void stopAlarm() {
  buzzerTone(0);
  digitalWrite(LED_PIN, LOW);
}

void resetDetection() {
  moveCount = 0;
  lastMoveTime = millis();
  stillMsgShown = false;
}

void goIdle() {
  stopAlarm();
  cooldownUntil = millis() + COOLDOWN_MS;
  resetDetection();
  setState(IDLE);
  showMsg("READY", "Monitoring...");
}

// ---------- Fall alert ----------
void runAlert() {
  bool on = (millis() / 200) % 2;     // 200 ms ON / 200 ms OFF
  buzzerTone(on ? BUZZER_FREQ : 0);
  digitalWrite(LED_PIN, on);

  if (millis() - stateStart >= ALERT_TIME_MS) {
    Serial.println("Alert time finished.");
    goIdle();
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  buzzerInit();
  stopAlarm();

  // MPU6050 bus
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // OLED bus (separate pins)
  Wire1.begin(OLED_SDA, OLED_SCL);
  Wire1.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false)) {
    Serial.println("OLED not found - check wiring");
  }
  showMsg("Calibrating", "Keep sensor still");

  mpuWrite(0x6B, 0x00);  // wake up
  mpuWrite(0x1A, 0x03);  // low-pass filter ~44 Hz
  mpuWrite(0x1B, 0x08);  // gyro  +-500 dps
  mpuWrite(0x1C, 0x10);  // accel +-8 g
  delay(100);
  calibrateGyro();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  uint8_t id = Wire.read();
  Serial.printf("MPU6050 WHO_AM_I = 0x%02X (expect 0x68)\n", id);
  Serial.println("Fall detection ready.");

  resetDetection();
  showMsg("READY", "Monitoring...");
}

// ---------- Main loop (100 Hz) ----------
void loop() {
  static unsigned long lastSample = 0;
  if (millis() - lastSample < 10) return;
  lastSample = millis();

  float svm = 1.0f, gyro = 0.0f;
  bool sensorOk = mpuRead(svm, gyro);
  if (!sensorOk) {
    static unsigned long lastErr = 0;
    if (millis() - lastErr > 2000) {
      Serial.println("MPU6050 read error - check wiring");
      lastErr = millis();
    }
  }

  // Uncomment to plot values in Serial Plotter:
  // Serial.printf("%.2f,%.1f\n", svm, gyro);

  switch (state) {
    case IDLE: {
      if (!sensorOk) break;

      if (millis() < cooldownUntil) {
        lastMoveTime = millis();
        break;
      }

      bool moving = (fabsf(svm - 1.0f) > MOVE_ACCEL_DEV_G || gyro > MOVE_GYRO_DPS);

      if (moving) {
        lastMoveTime = millis();
        stillMsgShown = false;
        moveCount++;
        if (moveCount >= MOVE_CONFIRM_COUNT) {
          moveCount = 0;
          Serial.println("Fall detected");
          showMsg("FALL!", "Alarm ON");
          setState(ALERT);
        }
      } else {
        moveCount = 0;
        if (!stillMsgShown && (millis() - lastMoveTime >= STILL_TIME_MS)) {
          Serial.println("Free fall detected");
          showMsg("NO MOVEMENT", "Detected");
          stillMsgShown = true;
        }
      }
      break;
    }

    case ALERT:
      runAlert();
      break;
  }
}
