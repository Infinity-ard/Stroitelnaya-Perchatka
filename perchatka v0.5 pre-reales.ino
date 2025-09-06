#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <BleMouse.h>

// ================= ИНВЕРСИЯ НАПРАВЛЕНИЙ =================
bool inversionY = false;
bool inversionX = false;

// ================= НАСТРОЙКИ ЭКРАНА =================
int screenWidth = 1920;   // реальная ширина экрана
int screenHeight = 1080;  // реальная высота экрана

const int baseScreenWidth = 1920;   // эталонная ширина экрана
const int baseScreenHeight = 1080;  // эталонная высота экрана

// ================= СЕНСА =================
// Для режима акселерометра (движение всей рукой)
const float baseSensitivityAccelX = 0.05;
const float baseSensitivityAccelY = 0.05;

// Для режима магнитометра (движение кистью)
const float baseSensitivityMagX = 0.05;
const float baseSensitivityMagY = 0.05;

// Пересчитанные сенсы (под экран)
float sensitivityAccelX, sensitivityAccelY;
float sensitivityMagX, sensitivityMagY;

// ================= ПИНЫ =================
const int calibButtonPin = 1; // кнопка калибровки центра
const int SDA_PIN = 7;        
const int SCL_PIN = 8;        
const int lmbPin = 2;         // ЛКМ
const int rmbPin = 4;         // ПКМ
const int switchAccelPin = 5; // переключатель в режим акселя
const int switchMagPin   = 6; // переключатель в режим магнита

// ================= ПЕРЕМЕННЫЕ =================
float centerX = 0, centerY = 0;   // центр (для калибровки)
float velX = 0, velY = 0;         // фильтр скорости

// ================= ОБЪЕКТЫ =================
Adafruit_MPU6050 mpu;
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
BleMouse bleMouse;

// ================= ПАРАМЕТРЫ =================
const float alpha = 0.9;          // фильтр сглаживания
const float noiseThreshold = 0.1; // отсечение шума
const int updateInterval = 20;    // частота обновления курсора (мс)

void setup() {
  Serial.begin(115200);

  pinMode(calibButtonPin, INPUT_PULLUP);
  pinMode(lmbPin, INPUT_PULLUP);
  pinMode(rmbPin, INPUT_PULLUP);
  pinMode(switchAccelPin, INPUT_PULLUP);
  pinMode(switchMagPin, INPUT_PULLUP);

  // ==== I2C ====
  Wire.begin(SDA_PIN, SCL_PIN);

  // ==== MPU6050 (акселерометр + гироскоп) ====
  if (!mpu.begin()) {
    Serial.println("Не найден MPU6050!");
    while (1) delay(10);
  }

  // ==== LSM303 (акселерометр + магнитометр) ====
  if (!accel.begin()) {
    Serial.println("Не найден LSM303 Accel!");
    while (1) delay(10);
  }
  if (!mag.begin()) {
    Serial.println("Не найден LSM303 Mag!");
    while (1) delay(10);
  }

  // ==== BLE Мышь ====
  bleMouse.begin();

  // ==== Автоподстройка чувствительности ====
  sensitivityAccelX = baseSensitivityAccelX * ((float)screenWidth / baseScreenWidth);
  sensitivityAccelY = baseSensitivityAccelY * ((float)screenHeight / baseScreenHeight);

  sensitivityMagX = baseSensitivityMagX * ((float)screenWidth / baseScreenWidth);
  sensitivityMagY = baseSensitivityMagY * ((float)screenHeight / baseScreenHeight);

  Serial.println("Система готова!");
  Serial.printf("Accel sens: X=%.3f, Y=%.3f\n", sensitivityAccelX, sensitivityAccelY);
  Serial.printf("Mag   sens: X=%.3f, Y=%.3f\n", sensitivityMagX, sensitivityMagY);
}

void loop() {
  if (!bleMouse.isConnected()) return;

  // ==== Определение режима работы ====
  bool modeAccel = (digitalRead(switchAccelPin) == LOW);
  bool modeMag   = (digitalRead(switchMagPin) == LOW);

  // ==== Калибровка центра ====
  if (digitalRead(calibButtonPin) == LOW) {
    sensors_event_t aevent;
    accel.getEvent(&aevent);
    centerX = aevent.acceleration.x;
    centerY = aevent.acceleration.y;
    Serial.println("Калибровка центра выполнена!");
    delay(500);
  }

  // ==== Считывание датчиков ====
  float dx = 0, dy = 0;

  if (modeAccel) {
    // --- Режим 1: управление всей рукой (акселерометр) ---
    sensors_event_t aevent;
    accel.getEvent(&aevent);

    float ax = aevent.acceleration.x - centerX;
    float ay = aevent.acceleration.y - centerY;

    if (fabs(ax) < noiseThreshold) ax = 0;
    if (fabs(ay) < noiseThreshold) ay = 0;

    velX = alpha * velX + (1 - alpha) * ax;
    velY = alpha * velY + (1 - alpha) * ay;

    dx = (inversionX ? -velX : velX) * sensitivityAccelX;
    dy = (inversionY ? -velY : velY) * sensitivityAccelY;

  } else if (modeMag) {
    // --- Режим 2: управление кистью (магнитометр) ---
    sensors_event_t mevent;
    mag.getEvent(&mevent);

    // Здесь ось выбираем под удобство: X/Y магнетометра = поворот кисти
    float mx = mevent.magnetic.x;
    float my = mevent.magnetic.y;

    dx = (inversionX ? -mx : mx) * sensitivityMagX * 0.01; // уменьшаем масштаб
    dy = (inversionY ? -my : my) * sensitivityMagY * 0.01;
  }

  // ==== Обновление курсора ====
  static unsigned long lastMoveTime = 0;
  if (millis() - lastMoveTime > updateInterval) {
    if ((int)dx != 0 || (int)dy != 0) {
      bleMouse.move((int)dx, (int)-dy); // инверсия Y для экрана
      Serial.printf("dx=%d, dy=%d\n", (int)dx, (int)dy);
    }
    lastMoveTime = millis();
  }

  // ==== Кнопки мыши ====
  if (digitalRead(lmbPin) == LOW) bleMouse.press(MOUSE_LEFT);
  else bleMouse.release(MOUSE_LEFT);

  if (digitalRead(rmbPin) == LOW) bleMouse.press(MOUSE_RIGHT);
  else bleMouse.release(MOUSE_RIGHT);
}
