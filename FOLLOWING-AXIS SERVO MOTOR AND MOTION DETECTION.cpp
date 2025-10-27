// Ursula Maurentti Amarely
// 24/533008/TK/59050

#include <Arduino.h>
#include <Wire.h> // For I2C communication used by MPU6050
#include <ESP32Servo.h> // For controlling servo motors in ESP32
#include <MPU6050.h> // For MPU6050 motion sensor

// Configure the PIN used for Servo, PIR Sensor, MPU6050 Sensor and LED,
#define SERVO1_PIN 13
#define SERVO2_PIN 12
#define SERVO3_PIN 14
#define SERVO4_PIN 27
#define SERVO5_PIN 26
#define PIR_PIN    33
#define LED_PIN    2
#define I2C_SDA    22 // MPU6050
#define I2C_SCL    21 // MPU6050

// Object Servo and MPU6050 Sensor
Servo servo1, servo2, servo3, servo4, servo5;
MPU6050 mpu;

// Control variable
float roll = 0, pitch = 0, yaw = 0; // Orientation angles
unsigned long lastUpdate = 0; // TIming yaw updates
unsigned long motionTimer = 0; // Timer for motion event
bool motionActive = false; // Indicates motion state in PIR

const int initialPos = 90; // Initialize center position or starting position

// Initial setup
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); // Starting I2C for MPU6050
  
  mpu.initialize(); // Starting MPU6050 sensor
  if (mpu.testConnection()) {
    Serial.println("MPU6050 Suscessfully Connected!");
  } else {
    Serial.println("MPU6050 Failed to Connect!"); // Simply stop the program if connection failed
    while (1);
  }

  pinMode(PIR_PIN, INPUT); // PIR sensor as input
  pinMode(LED_PIN, OUTPUT); // LED as output
  digitalWrite(LED_PIN, LOW); // Setting initial condition for LED

  // Attach the servo to their respective pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  servo5.attach(SERVO5_PIN);

  // Set all of the servo in the initial position, 90 degrees
  servo1.write(initialPos);
  servo2.write(initialPos);
  servo3.write(initialPos);
  servo4.write(initialPos);
  servo5.write(initialPos);

  lastUpdate = millis(); // Start time tracking
}

// Main loop function
void loop() {
  // Read the MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate roll and pitch
  roll = atan2(ay, az) * 180.0 / PI;
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI; 

  // Calculate yaw from gyro
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0; // Time difference in seconds
  lastUpdate = now;
  yaw += (gz / 131.0) * dt; // Integrate gyro rate to get yaw
  yaw = constrain(yaw, -90, 90); // Limit the yaw between -90 and 90 degrees

  // Roll Motion
  if (abs(roll) > 5 && !motionActive) { // If theres tlit detected and not interupted by PIR
    int offset = map((int)roll, -90, 90, -45, 45);
    // Opposite direction for balance
    servo1.write(constrain(initialPos - offset, 0, 180));
    servo2.write(constrain(initialPos + offset, 0, 180));
  } else if (!motionActive) {
    // If no tlit detected, return to neutral
    servo1.write(initialPos);
    servo2.write(initialPos);
  }

  // Pitch Motion
  if (abs(pitch) > 5 && !motionActive) { // Chech if the pitch angle is larger than 5 degree
    int offset = map((int)pitch, -90, 90, -45, 45); // Map pitch value (-90 to 90) and movement range (-45 to 45).
    // Move i servo 3 and 4 in the same direction and keeping the final position between 0 and 180 degree
    servo3.write(constrain(initialPos + offset, 0, 180));
    servo4.write(constrain(initialPos + offset, 0, 180));
  } else if (!motionActive) { // If theres no significant pitch movement and system is not responsing PIR
    // Reset servo 3 and 4 to their centered position which is 90
    servo3.write(initialPos);
    servo4.write(initialPos);
  }

 // Yaw Motion
  static unsigned long yawStart = 0; // Saving the time when yaw movement starts
  static bool yawMoving = false; 

  if (abs(yaw) > 5 && !motionActive && !yawMoving) { // Check whether the tlit is more than 5 degree
    int offset = map((int)yaw, -90, 90, -45, 45); // COnverts yaw angle (-90 to 90) into smaller servo range (-40 to 40)
    servo5.write(constrain(initialPos + offset, 0, 180)); // Use constrain to ensures servo position stays in its range
    // Mark the yaw movement in progress and the current time
    yawMoving = true;
    yawStart = millis();
  }

  // After 1 seconds, back to the initial position
  if (yawMoving && millis() - yawStart > 1000) {
    servo5.write(initialPos);
    yawMoving = false;
  }

  // Detect external motion using PIR
  int pirVal = digitalRead(PIR_PIN);
  if (pirVal == HIGH && !motionActive) {
    Serial.println("External Motion Detected!");
    motionActive = true;
    digitalWrite(LED_PIN, HIGH); // Turn the LED on to indicates motion detection
    motionTimer = millis();

    // Move to free position (we decide the degree)
    servo1.write(45);
    servo2.write(135);
    servo3.write(45);
    servo4.write(135);
    servo5.write(60);
  }

  // After 2 seconds, all the servos back to the initial position
  if (motionActive && millis() - motionTimer > 2000) {
    servo1.write(initialPos);
    servo2.write(initialPos);
    servo3.write(initialPos);
    servo4.write(initialPos);
    servo5.write(initialPos);
    digitalWrite(LED_PIN, LOW); // Turn the indicator light off
    motionActive = false;
    Serial.println("Back to the original position.");
  }

  // Debug and update the data every 0.5 seconds
  static unsigned long debugTimer = 0; // Stores the last time data was print
  if (millis() - debugTimer > 500) { // Check whether the last data print is more than 500 miliseconds ago
    Serial.print("Roll: "); Serial.print(roll);
    Serial.print(" | Pitch: "); Serial.print(pitch);
    Serial.print(" | Yaw: "); Serial.println(yaw);
    debugTimer = millis(); // Reset the timer
  }

  delay(50); // For stabiliy add delay
}
