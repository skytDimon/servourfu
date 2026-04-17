#include <Servo.h>

const int servoPin = 9;
Servo myServo;

// ==========================================
// НАСТРОЙКИ ВРАЩЕНИЯ
// ==========================================
const int SERVO_STOP       = 90;
const int SERVO_MOVE_CW    = 83;   // < 90 = по часовой. Если крутит против — поставьте 97
const int HANDSHAKE_TIMEOUT_MS = 30000;  // 30 сек ожидание ответа от Python
const int SETTLE_DELAY_MS  = 300;  // Пауза после остановки для успокоения антенны
const int STEPS_PER_SCAN   = 36;   // 360 / 10
const int DEG_PER_STEP     = 10;

unsigned long timeFor10Deg = 213;  // ВАЖНО: откалибруйте командой 'C'!
// ==========================================


void drainSerial() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);
  myServo.write(SERVO_STOP);
  Serial.println("SYSTEM_READY");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    delay(5);
    drainSerial();

    if (cmd == 'S' || cmd == 's') {
      runScan();
    } else if (cmd == 'C' || cmd == 'c') {
      runCalibration();
    } else if (cmd == 'H' || cmd == 'h') {
      returnHome();
    }
  }
}

// ------------------------------------------
// КАЛИБРОВКА: полный оборот CW для проверки timeFor10Deg
// Перед запуском: установите антенну на отметку 0 градусов
// После: если антенна не вернулась к отметке — увеличьте/уменьшите timeFor10Deg
// ------------------------------------------
void runCalibration() {
  Serial.println("CAL_START");

  myServo.attach(servoPin);
  myServo.write(SERVO_MOVE_CW);

  unsigned long totalMs = timeFor10Deg * STEPS_PER_SCAN;
  delay(totalMs);

  myServo.write(SERVO_STOP);
  delay(SETTLE_DELAY_MS);
  myServo.detach();

  Serial.print("CAL_DONE:");
  Serial.println(totalMs);
}

// ------------------------------------------
// ВОЗВРАТ ДОМОЙ: доворот на 10 градусов CW после скана
// (после 35 шагов антенна стоит на 350°, нужно еще 10°)
// ------------------------------------------
void returnHome() {
  Serial.println("HOME_START");
  myServo.attach(servoPin);
  myServo.write(SERVO_MOVE_CW);
  delay(timeFor10Deg);
  myServo.write(SERVO_STOP);
  delay(SETTLE_DELAY_MS);
  myServo.detach();
  Serial.println("HOME_DONE");
}

// ------------------------------------------
// ОСНОВНОЙ СКАН: 36 точек с handshake
// ------------------------------------------
void runScan() {
  Serial.println("SCAN_START");

  for (int i = 0; i < STEPS_PER_SCAN; i++) {
    int currentAngle = i * DEG_PER_STEP;

    // 1. ОСТАНОВКА И УСПОКОЕНИЕ
    myServo.write(SERVO_STOP);
    delay(SETTLE_DELAY_MS);
    myServo.detach();

    // 2. СИГНАЛ ПИТОНУ: готов к замеру на угле currentAngle
    Serial.print("READY:");
    Serial.println(currentAngle);

    // 3. ЖДЕМ 'K' ОТ ПИТОНА С ТАЙМАУТОМ
    unsigned long startWait = millis();
    while (Serial.available() == 0) {
      if (millis() - startWait > HANDSHAKE_TIMEOUT_MS) {
        Serial.println("SCAN_TIMEOUT");
        myServo.attach(servoPin);
        myServo.write(SERVO_STOP);
        return;
      }
      delay(10);
    }
    Serial.read();
    drainSerial();

    // 4. ПОВОРОТ НА СЛЕДУЮЩИЕ 10°
    if (i < STEPS_PER_SCAN - 1) {
      myServo.attach(servoPin);
      myServo.write(SERVO_MOVE_CW);
      delay(timeFor10Deg);
    }
  }

  Serial.println("SCAN_FINISHED");
  myServo.attach(servoPin);
  myServo.write(SERVO_STOP);
}
