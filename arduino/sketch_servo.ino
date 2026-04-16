#include <Servo.h>

const int servoPin = 9;
Servo myServo;

// ==========================================
// НАСТРОЙКИ ВРАЩЕНИЯ И КАЛИБРОВКИ
// ==========================================
const int SERVO_STOP = 90;       // Точка идеальной остановки
// Для вращения ПО ЧАСОВОЙ стрелке используем значение МЕНЬШЕ 90. 
// Если вдруг крутится против часовой, поменяйте 70 на 110.
const int SERVO_MOVE_CW = 83;    
const int TIME_FOR_10_DEG = 213; // ВАЖНО: Впишите ваше время для поворота на 10 градусов!
// ==========================================

void setup() {
  Serial.begin(115200);
  myServo.attach(servoPin);
  myServo.write(SERVO_STOP);
  Serial.println("SYSTEM_READY");
}

void loop() {
  // Ждем команду от Питона
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'S' || cmd == 's') {
      runPerfectScan();
    }
  }
}

void runPerfectScan() {
  Serial.println("SCAN_START");

  for (int i = 0; i < 36; i++) {
    int currentAngle = i * 10;

    // 1. ПОЛНАЯ ОСТАНОВКА И ОТКЛЮЧЕНИЕ МОТОРА
    myServo.write(SERVO_STOP);
    delay(100);
    myServo.detach(); 

    // 2. ОТПРАВЛЯЕМ СИГНАЛ ПИТОНУ
    Serial.print("READY:");
    Serial.println(currentAngle);

    // 3. ЖДЕМ ОТВЕТА ОТ ПИТОНА (Handshake)
    // Питон сам отсчитает 2 секунды и пришлет букву 'K'
    while (Serial.available() == 0) {
      delay(10);
    }
    Serial.read(); // Очищаем буфер от буквы 'K'

    // 4. ПОВОРОТ НА СЛЕДУЮЩИЕ 10 ГРАДУСОВ
    if (i < 35) {
      myServo.attach(servoPin);
      myServo.write(SERVO_MOVE_CW);
      delay(TIME_FOR_10_DEG);
    }
  }

  Serial.println("SCAN_FINISHED");
  myServo.attach(servoPin);
  myServo.write(SERVO_STOP); // Фиксируем в конце
}