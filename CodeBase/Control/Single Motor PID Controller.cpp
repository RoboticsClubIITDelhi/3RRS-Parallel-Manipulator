// Cytron Motor Driver Position Control with PID - Serial Input for Target Position
// Input is in pulse counts of encoder, not angle of motor

// Define motor driver pins
#define PWM_PIN 9       // PWM control pin (connect to PWM input on the driver)
#define DIR_PIN 8       // Direction control pin (connect to DIR input on the driver)
#define ENCODER_A 2     // Encoder signal A (Connect to an interrupt pin)
#define ENCODER_B 3     // Encoder signal B (Connect to an interrupt pin)

volatile long encoderCount = 0; // Encoder pulse count (changed to long for larger range)
unsigned long prevTime = 0;
float prevError = 0, integral = 0;

// PID Constants
float Kp = 2;
float Ki = 4.2;
float Kd = 0.09;

// Desired position in encoder counts
long targetPosition = 0;

void setup() {
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), countEncoder, RISING);
    Serial.begin(9600);
    Serial.println("Enter target position:");
}

void countEncoder() {
    if (digitalRead(ENCODER_B) == HIGH) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void setMotor(int speed) {
    digitalWrite(DIR_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM_PIN, constrain(abs(speed), 0, 255));
}

void loop() {
    // Read target position from Serial
    if (Serial.available() > 0) {
        targetPosition = Serial.parseInt();
        Serial.print("New Target Position: ");
        Serial.println(targetPosition);
        // Clear the input buffer
        while(Serial.available() > 0) {
            Serial.read();
        }
    }
    
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0; // Convert to seconds
    if (dt >= 0.05) { // Run control loop every 100ms
        float error = targetPosition - encoderCount;
        integral += error * dt;
        float derivative = (error - prevError) / dt;
        float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        if (error>0){
          output = (0.6*Kp * error) + (3 * integral) + (Kd * derivative);}
        
        setMotor(output);
        
        prevError = error;
        prevTime = currentTime;


        Serial.print("Target: "); Serial.print(targetPosition);
        Serial.print(" | Current: "); Serial.print(encoderCount);
        Serial.print(" | PWM: "); Serial.println(output);
    }
}
