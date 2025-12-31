#include <Arduino.h>

// ===== Motor Pins =====
#define R1PWM 19  // Right forward
#define R2PWM 21  // Right backward
#define L1PWM 23  // Left forward
#define L2PWM 22  // Left backward

// ===== PWM Channels =====
#define R1 0
#define R2 1
#define L1 2
#define L2 3

// ===== Speed Settings =====
int baseSpeed = 150;
#define MIN_SPEED 80
#define MID_SPEED 120
#define STOP_DELAY 20



// ===== PROTOTYPES =====
void stopMotors();

// ===================================================
// ====================== SETUP ======================
// ===================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("ESP32 ready with ultrasonic safety stop.");

  // Motor PWM setup
  ledcSetup(R1, 20000, 8);
  ledcSetup(R2, 20000, 8);
  ledcSetup(L1, 20000, 8);
  ledcSetup(L2, 20000, 8);

  ledcAttachPin(R1PWM, R1);
  ledcAttachPin(R2PWM, R2);
  ledcAttachPin(L1PWM, L1);
  ledcAttachPin(L2PWM, L2);

  stopMotors();
}



// ===================================================
// ====================== LOOP =======================
// ===================================================
void loop() {
  


  // -------- READ YOLO COMMANDS FROM PI --------
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("Received: ");
    Serial.println(cmd);

    switch (cmd) {
      case 'W': moveForward(baseSpeed); break;

      case 'f': turnLeftSmooth(MIN_SPEED); break;
      case 'F': turnLeftSmooth(MID_SPEED); break;
      case 'X': turnLeftSmooth(MID_SPEED); break;

      case 'b': turnRightSmooth(MIN_SPEED); break;
      case 'B': turnRightSmooth(MID_SPEED); break;
      case 'Y': turnRightSmooth(MID_SPEED); break;

      case 'S': stopMotors(); break;

      case '+': baseSpeed = min(baseSpeed + 10, 255);
                Serial.print("Speed+ = "); Serial.println(baseSpeed);
                break;

      case '-': baseSpeed = max(baseSpeed - 10, MIN_SPEED);
                Serial.print("Speed- = "); Serial.println(baseSpeed);
                break;

      default:
        Serial.println("⚠ Unknown command");
        break;
    }
  }
}

// ===================================================
// ================== MOTOR FUNCTIONS =================
// ===================================================
void moveForward(int pwmVal) {
  pwmVal = constrain(pwmVal, MIN_SPEED, 255);
  Serial.print("Moving Forward @ "); Serial.println(pwmVal);
  ledcWrite(R1, 0); 
  ledcWrite(R2, pwmVal);
  ledcWrite(L1, pwmVal);
  ledcWrite(L2, 0);
}

void turnLeftSmooth(int pwmVal) {
  pwmVal = constrain(pwmVal, MIN_SPEED, 255);
  Serial.print("Turning Left @ "); Serial.println(pwmVal);
  ledcWrite(R1, 0); 
  ledcWrite(R2, pwmVal);
  ledcWrite(L1, 0);
  ledcWrite(L2, pwmVal / 2);
}

void turnRightSmooth(int pwmVal) {
  pwmVal = constrain(pwmVal, MIN_SPEED, 255);
  Serial.print("Turning Right @ "); Serial.println(pwmVal);
  ledcWrite(R1, 0);
  ledcWrite(R2, pwmVal / 2);
  ledcWrite(L1, pwmVal);
  ledcWrite(L2, 0);
}

void stopMotors() {
  ledcWrite(R1, 0);
  ledcWrite(R2, 0);
  ledcWrite(L1, 0);
  ledcWrite(L2, 0);
  delay(STOP_DELAY);
  Serial.println("Motors stopped.");
}
