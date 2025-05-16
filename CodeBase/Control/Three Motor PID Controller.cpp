// Cytron Code for three simultaneous motor position controls
// Connect encoder pins to interrupt pins 
// The input is taken from the serial monitor as angle of motors in degrees

// === Motor 1 Pins ===
#define PWM1 9
#define DIR1 8
#define ENC_A1 2
#define ENC_B1 3

// === Motor 2 Pins ===
#define PWM2 5
#define DIR2 4
#define ENC_A2 18
#define ENC_B2 19

// === Motor 3 Pins ===
#define PWM3 6
#define DIR3 7
#define ENC_A3 20
#define ENC_B3 21

// === Encoder Counts ===
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;

// === PID Variables ===
float Kp = 1.8;
float Ki = 2;
float Kd = 0.1;

long targets[3] = {0, 0, 0};
float errors[3] = {0, 0, 0};
float integrals[3] = {0, 0, 0};
float previousErrors[3] = {0, 0, 0};
unsigned long prevTime = 0;

void setup() {
  // Set motor pins as outputs
  pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT); pinMode(DIR3, OUTPUT);

  // Set encoder pins
  pinMode(ENC_A1, INPUT_PULLUP); pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP); pinMode(ENC_B2, INPUT_PULLUP);
  pinMode(ENC_A3, INPUT_PULLUP); pinMode(ENC_B3, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A1), isr1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), isr2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_A3), isr3, RISING);

  Serial.begin(9600);
  Serial.println("Enter 3 target positions (e.g., 1000 2000 1500):");
}

// === Encoder ISRs ===
void isr1() {
  count1 += (digitalRead(ENC_B1) == HIGH) ? 1 : -1;
}
void isr2() {
  count2 += (digitalRead(ENC_B2) == HIGH) ? 1 : -1;
}
void isr3() {
  count3 += (digitalRead(ENC_B3) == HIGH) ? 1 : -1;
}

// === Motor Control Function ===
void setMotor(int motorIndex, int speed) {
  int pwmPin, dirPin;
  if (motorIndex == 0) { pwmPin = PWM1; dirPin = DIR1; }
  else if (motorIndex == 1) { pwmPin = PWM2; dirPin = DIR2; }
  else { pwmPin = PWM3; dirPin = DIR3; }

  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(speed), 0, 255));
}

// === Get Encoder Value by Index ===
long getEncoder(int idx) {
  if (idx == 0) return count1;
  if (idx == 1) return count2;
  return count3;
}

void loop() {
  // Read Serial for new targets
  if (Serial.available()) {
    for (int i = 0; i < 3; i++) {
      targets[i] = Serial.parseInt();
      targets[i] = targets[i]*600/360;
    }
    while (Serial.available()) Serial.read(); // Clear buffer
    Serial.print("New targets: ");
    Serial.print(targets[0]); Serial.print(" ");
    Serial.print(targets[1]); Serial.print(" ");
    Serial.println(targets[2]);
  }

  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;

  if (dt >= 0.05) {
    for (int i = 0; i < 3; i++) {
      long current = getEncoder(i);
      errors[i] = targets[i] - current;
      integrals[i] += errors[i] * dt;
      float derivative = (errors[i] - previousErrors[i]) / dt;

      float output = (Kp * errors[i]) + (Ki * integrals[i]) + (Kd * derivative);
      if (errors[i] > 0) {
        output = (0.6 * Kp * errors[i]) + (3 * integrals[i]) + (Kd * derivative);
      }

      setMotor(i, output);
      previousErrors[i] = errors[i];

      Serial.print("M"); Serial.print(i+1);
      Serial.print(" Pos: "); Serial.print(current);
      Serial.print(" | Target: "); Serial.print(targets[i]);
      Serial.print(" | PWM: "); Serial.print(output);
      Serial.print(" || ");
    }
    Serial.println();
    prevTime = now;
  }
}
