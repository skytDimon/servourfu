#include <Servo.h>

const int servoPin    = 9;
const int ledPin      = 13;
const int buttonPin   = 2;

Servo myServo;

// ==========================================
// НАСТРОЙКИ ВРАЩЕНИЯ
// ==========================================
const int SERVO_STOP          = 90;
const int SERVO_MOVE_CW       = 83;    // < 90 = по часовой. Если крутит против — поставьте 97
const int HANDSHAKE_TIMEOUT_MS = 30000;
const int SETTLE_DELAY_MS     = 300;
const int STEPS_PER_SCAN      = 36;    // 360 / 10
const int DEG_PER_STEP        = 10;

unsigned long timeFor10Deg = 213;      // Откалибруйте через команду 'C' в Python!
int positionDeg             = 0;       // Текущая позиция антенны
// ==========================================


void drainSerial() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void blinkLed(int count, int onMs, int offMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ledPin, HIGH);
    delay(onMs);
    digitalWrite(ledPin, LOW);
    if (i < count - 1) delay(offMs);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  myServo.attach(servoPin);
  myServo.write(SERVO_STOP);
  digitalWrite(ledPin, LOW);

  blinkLed(3, 100, 100);
  Serial.println("SYSTEM_READY");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == 'T' || cmd == 't') {
      String numStr = "";
      unsigned long startRead = millis();
      while (millis() - startRead < 2000) {
        if (Serial.available() > 0) {
          char c = Serial.read();
          if (c == '\n' || c == '\r') break;
          if (c >= '0' && c <= '9') numStr += c;
        }
      }
      if (numStr.length() > 0) {
        timeFor10Deg = numStr.toInt();
        Serial.print("TIME_SET:");
        Serial.println(timeFor10Deg);
      }
    } else {
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

  if (digitalRead(buttonPin) == LOW) {
    delay(50);
    if (digitalRead(buttonPin) == LOW) {
      while (digitalRead(buttonPin) == LOW) {
        delay(10);
      }
      Serial.println("BUTTON_PRESSED");
    }
  }
}

// ------------------------------------------
// КАЛИБРОВКА
// ------------------------------------------
void runCalibration() {
  Serial.println("CAL_START");
  positionDeg = 0;

  myServo.attach(servoPin);
  myServo.write(SERVO_MOVE_CW);
  digitalWrite(ledPin, HIGH);

  unsigned long totalMs = timeFor10Deg * STEPS_PER_SCAN;
  delay(totalMs);

  myServo.write(SERVO_STOP);
  digitalWrite(ledPin, LOW);
  delay(SETTLE_DELAY_MS);
  myServo.detach();

  positionDeg = 360;
  Serial.print("CAL_DONE:");
  Serial.println(totalMs);
}

// ------------------------------------------
// ВОЗВРАТ ДОМОЙ
// ------------------------------------------
void returnHome() {
  Serial.println("HOME_START");

  int remainingDeg = 360 - positionDeg;
  if (remainingDeg <= 0 || remainingDeg >= 360) {
    positionDeg = 0;
    Serial.println("HOME_DONE:ALREADY_AT_ZERO");
    return;
  }

  int remainingSteps = remainingDeg / DEG_PER_STEP;
  if (remainingSteps <= 0) {
    positionDeg = 0;
    Serial.println("HOME_DONE:ALREADY_AT_ZERO");
    return;
  }

  unsigned long rotateMs = (unsigned long)remainingSteps * timeFor10Deg;

  myServo.attach(servoPin);
  myServo.write(SERVO_MOVE_CW);
  digitalWrite(ledPin, HIGH);
  delay(rotateMs);
  myServo.write(SERVO_STOP);
  digitalWrite(ledPin, LOW);
  delay(SETTLE_DELAY_MS);
  myServo.detach();

  positionDeg = 0;
  Serial.print("HOME_DONE:ROTATED_");
  Serial.print(remainingDeg);
  Serial.println("DEG");
}

// ------------------------------------------
// СКАН: 36 точек с handshake
// ------------------------------------------
void runScan() {
  positionDeg = 0;
  Serial.println("SCAN_START");

  for (int i = 0; i < STEPS_PER_SCAN; i++) {
    positionDeg = i * DEG_PER_STEP;

    myServo.write(SERVO_STOP);
    delay(SETTLE_DELAY_MS);
    myServo.detach();

    digitalWrite(ledPin, HIGH);

    Serial.print("READY:");
    Serial.println(positionDeg);

    unsigned long startWait = millis();
    while (Serial.available() == 0) {
      if (millis() - startWait > HANDSHAKE_TIMEOUT_MS) {
        Serial.println("SCAN_TIMEOUT");
        myServo.attach(servoPin);
        myServo.write(SERVO_STOP);
        digitalWrite(ledPin, LOW);
        return;
      }
      delay(10);
    }
    Serial.read();
    drainSerial();

    digitalWrite(ledPin, LOW);

    if (i < STEPS_PER_SCAN - 1) {
      myServo.attach(servoPin);
      myServo.write(SERVO_MOVE_CW);
      delay(timeFor10Deg);
    }
  }

  positionDeg = (STEPS_PER_SCAN - 1) * DEG_PER_STEP;
  Serial.println("SCAN_FINISHED");

  myServo.attach(servoPin);
  myServo.write(SERVO_STOP);

  blinkLed(5, 200, 200);
}
